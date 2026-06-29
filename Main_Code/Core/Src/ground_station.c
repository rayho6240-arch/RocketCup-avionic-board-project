/*
 * ground_station.c — 地面站接收器主流程（ROLE_GROUND）
 * ===========================================================================
 * 整檔以 #if IS_GROUND 包住：主/備航電編譯為空，零影響。
 *
 * 雙鏈路接收火箭下行 TelemetryPacket_t：
 *   E22 433（USART3 透傳）：中斷接收 → 位元組環形緩衝 → telem_rx 同步FSM
 *   E80 920（SX126x SPI3）：輪詢 DIO1 → LoRaE80_ReadPacket → 同 telem_rx 解析（含 RSSI/SNR）
 * + 讀自身 GPS（USART6），用 gs_timesync 把火箭 tick 對齊到 GPS 紀律的牆鐘。
 * 每筆有效封包組成 GsLogRecord → 三路落地：USB-CDC(CSV) / SD(CSV) / Flash(二進位 append)。
 *
 * GS_USB_SELFTEST=1：略過 LoRa，產生模擬遙測以「相同輸出格式」串流 USB（並寫 SD/Flash），
 * 用於在沒有任何 LoRa 流量下單獨確認 USB 列舉與資料管線正常。
 */
#include "board_config.h"
#if IS_GROUND

#include "ground_station.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

#include "telem_rx.h"
#include "gs_timesync.h"
#include "gs_log.h"
#include "gs_lora_test.h"
#include "gps.h"
#include "lora_e80.h"
#include "w25qxx.h"
#include "flash_ring_math.h"
#include "fatfs.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#ifndef GS_USB_SELFTEST
#define GS_USB_SELFTEST 0          /* 1 = USB 自測模式（不靠 LoRa） */
#endif
#define GS_TIMESYNC_EMA_SHIFT  3   /* rocket↔ground 偏移 EMA：alpha = 1/8 */
#define GS_POLL_DELAY_MS       2   /* 主迴圈輪詢週期（~500Hz） */

extern UART_HandleTypeDef huart3;  /* E22 433 透傳（main.c 定義） */
extern IWDG_HandleTypeDef hiwdg;   /* 看門狗（main.c 定義） */

/* ---- 狀態 ---- */
static TelemRx_t     s_rx433;
static TelemRx_t     s_rx920;
static GsTimeSync_t  s_ts;
static uint32_t      s_last_fix_tick = 0;

/* 大緩衝置於檔案範圍，降低任務堆疊壓力（地面站單一任務，安全） */
static char    s_row[GS_LOG_CSV_MAX];
static uint8_t s_e80buf[255];

/* ---- 指示燈（板上三顆，地面站用途）----
 *   PE2 LED_SYS    : 心跳閃爍（1Hz）= 韌體存活、主迴圈在跑
 *   PE3 LED_State1 : GPS 定位有效 = 恆亮（時間對齊錨點就緒）
 *   PE4 LED_State2 : 接收活動 = 收到下行封包即短亮，隨流量閃爍
 * 規格表標「低/高電平控制」；此處採 active-high（SET=亮，GPIO init 為 RESET=滅）。
 * 若實際硬體為 active-low，將 GS_LED_ON/GS_LED_OFF 對調即可。 */
#define GS_LED_ON       GPIO_PIN_SET
#define GS_LED_OFF      GPIO_PIN_RESET
#define GS_LED1_Pin     GPIO_PIN_3        /* PE3 LED_State1（main.h 未命名，直接用腳號） */
#define GS_RX_LED_MS    120U              /* 收到封包後亮燈持續（製造每包可見閃爍） */
#define GS_HEARTBEAT_MS 500U              /* LED_SYS 心跳半週期（1Hz 閃） */

static volatile uint32_t s_last_pkt_tick = 0;   /* 最後一筆有效封包 tick（任一鏈路） */

/* 依現況驅動三顆 LED。每個主迴圈週期呼叫一次。 */
static void gs_leds_update(uint32_t now, uint8_t gps_fix)
{
    static uint32_t hb_tick = 0;
    static uint8_t  hb_on   = 0;
    if ((now - hb_tick) >= GS_HEARTBEAT_MS) {       /* PE2：心跳 */
        hb_tick = now;
        hb_on ^= 1U;
        HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, hb_on ? GS_LED_ON : GS_LED_OFF);
    }
    HAL_GPIO_WritePin(GPIOE, GS_LED1_Pin,            /* PE3：GPS fix 恆亮 */
                      gps_fix ? GS_LED_ON : GS_LED_OFF);
    uint8_t rx_active = (s_last_pkt_tick != 0) && ((now - s_last_pkt_tick) < GS_RX_LED_MS);
    HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin,  /* PE4：接收活動 */
                      rx_active ? GS_LED_ON : GS_LED_OFF);
}

/* ---- USART3（E22）位元組環形緩衝：ISR 推入、任務取出 ---- */
#define U3_RING_SZ 1024U
static volatile uint8_t  s_u3_ring[U3_RING_SZ];
static volatile uint16_t s_u3_head = 0, s_u3_tail = 0;
static uint8_t           s_u3_rxbuf[96];   /* ReceiveToIdle 暫存 */

static void u3_push(uint8_t b)
{
    uint16_t nh = (uint16_t)((s_u3_head + 1U) % U3_RING_SZ);
    if (nh != s_u3_tail) { s_u3_ring[s_u3_head] = b; s_u3_head = nh; }  /* 滿則丟 */
}
static int u3_pop(uint8_t *b)
{
    if (s_u3_tail == s_u3_head) return 0;
    *b = s_u3_ring[s_u3_tail];
    s_u3_tail = (uint16_t)((s_u3_tail + 1U) % U3_RING_SZ);
    return 1;
}

void GroundStation_OnUart3RxEvent(uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++) u3_push(s_u3_rxbuf[i]);
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, s_u3_rxbuf, sizeof(s_u3_rxbuf));  /* 重新掛載 */
}

/* ---- SD（FatFS CSV） ---- */
static FIL      s_sd_file;
static uint8_t  s_sd_ok = 0;
static uint32_t s_sd_rows = 0;

static void gs_sd_open(void)
{
    char name[16];
    FILINFO fno;
    if (f_mount(&SDFatFS, SDPath, 1) != FR_OK) return;
    for (int i = 0; i < 1000; i++) {
        snprintf(name, sizeof(name), "GSLOG%03d.CSV", i);
        if (f_stat(name, &fno) == FR_NO_FILE) break;   /* 取下一個未用檔名 */
    }
    if (f_open(&s_sd_file, name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
        char hdr[GS_LOG_CSV_MAX];
        int n = GsLog_CsvHeader(hdr, sizeof(hdr));
        UINT bw;
        if (n > 0) f_write(&s_sd_file, hdr, (UINT)n, &bw);
        f_sync(&s_sd_file);
        s_sd_ok = 1;
    }
}
static void gs_sd_write(const char *row, uint16_t n)
{
    if (!s_sd_ok) return;
    UINT bw;
    f_write(&s_sd_file, row, (UINT)n, &bw);
    if ((++s_sd_rows % 16U) == 0U) f_sync(&s_sd_file);   /* 定期 flush 降低掉電損失 */
}

/* ---- Flash（W25Q128 ring 區順序 append；用前才擦的 erase-ahead） ---- */
static uint32_t s_fl_addr;        /* 寫入頭 */
static uint32_t s_fl_erased_end;  /* 已擦至此位址（exclusive） */

static void gs_flash_init(void)
{
    s_fl_addr = FLASH_RINGBUF_ADDR;
    s_fl_erased_end = FLASH_RINGBUF_ADDR;   /* 尚未擦任何 sector */
}
static void gs_flash_ensure_erased(uint32_t addr, uint32_t n)
{
    while (addr + n > s_fl_erased_end) {
        if (W25QXX_EraseSector(s_fl_erased_end) != W25QXX_OK) break;
        s_fl_erased_end += FLASH_RING_SECTOR_SIZE;
    }
}
static void gs_flash_append(const GsLogRecord_t *rec)
{
    if (s_fl_addr + GS_LOG_RECORD_SIZE > FLASH_RINGBUF_END + 1UL) {
        s_fl_addr = FLASH_RINGBUF_ADDR;          /* 回繞，重新從頭擦 */
        s_fl_erased_end = FLASH_RINGBUF_ADDR;
    }
    gs_flash_ensure_erased(s_fl_addr, GS_LOG_RECORD_SIZE);
    if (W25QXX_WriteData(s_fl_addr, (const uint8_t *)rec, GS_LOG_RECORD_SIZE) == W25QXX_OK) {
        s_fl_addr += GS_LOG_RECORD_SIZE;
    }
}

/* ---- USB-CDC（best-effort，PC 沒讀就丟） ---- */
static void gs_usb_send(const uint8_t *buf, uint16_t n)
{
    for (int i = 0; i < 8; i++) {
        if (CDC_Transmit_FS((uint8_t *)buf, n) == USBD_OK) return;
        osDelay(1);   /* 前一筆未送完，稍等再試 */
    }
}

/* ---- 共用：一筆封包 → 對齊時間 → 組紀錄 → 三路落地 ---- */
static void gs_handle_packet(uint8_t link, const TelemetryPacket_t *pkt,
                             int16_t rssi, int16_t snr)
{
    uint32_t rx_tick = HAL_GetTick();
    s_last_pkt_tick = rx_tick;          /* 接收活動指示燈（PE4）用 */
    GsTimeSync_OnPacket(&s_ts, pkt->tick_ms, rx_tick, GS_TIMESYNC_EMA_SHIFT);
    uint32_t rx_utc = GsTimeSync_GroundUtcMs(&s_ts, rx_tick);
    uint32_t al_utc = GsTimeSync_RocketAlignedUtcMs(&s_ts, pkt->tick_ms);
    int32_t  off    = GsTimeSync_Offset(&s_ts);

    const GPS_Data_t *g = GPS_GetData();
    GsLogRecord_t rec;
    GsLog_BuildRecord(&rec, link, rssi, snr, rx_tick, rx_utc, al_utc, off,
                      g->lat_1e6, g->lon_1e6, (int16_t)g->altitude_m,
                      g->satellites, g->fix_valid, pkt);

    int n = GsLog_FormatCsvRow(s_row, sizeof(s_row), &rec);
    if (n > (int)sizeof(s_row) - 1) n = (int)sizeof(s_row) - 1;  /* 防 snprintf 截斷回傳值溢位 */
    if (n > 0) {
        gs_usb_send((const uint8_t *)s_row, (uint16_t)n);
        gs_sd_write(s_row, (uint16_t)n);
    }
    gs_flash_append(&rec);

    /* 更新通訊測試統計 */
    GsLoraTest_UpdateStats(link, rssi, snr, 1 /* crc_ok */);
}

#if GS_USB_SELFTEST
/* 模擬器：以 ~10Hz 產生決定性「走動」遙測，串流相同格式驗證 USB（不碰 LoRa）。 */
static void gs_usb_selftest_loop(void)
{
    uint8_t  seq = 0;
    uint32_t tick = 0;
    int32_t  alt_m = 0;
    int      dir = 1;

    GsTimeSync_OnGpsFix(&s_ts, 120000U, HAL_GetTick());  /* 假錨點 12:00:00 */

    for (;;) {
        TelemetryPacket_t pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.sync0 = TELEM_SYNC0;
        pkt.sync1 = TELEM_SYNC1;
        pkt.seq = seq++;
        pkt.tick_ms = tick;
        pkt.fsm_state = (alt_m > 1000) ? 3 : 1;
        pkt.ekf_pos_z_cm = alt_m * 100;
        pkt.baro_alt_cm = alt_m * 100;
        pkt.gps_lat_1e6 = 25033000;
        pkt.gps_lon_1e6 = 121564000;
        pkt.gps_alt_m = (int16_t)alt_m;
        pkt.gps_sats = 9;
        pkt.gps_fix = 1;
        pkt.bat_mv = 11800;
        pkt.crc16 = crc16_ccitt_false((const uint8_t *)&pkt, (uint16_t)(TELEM_PACKET_SIZE - 2));

        uint8_t link = (seq & 1U) ? GS_LINK_920 : GS_LINK_433;     /* 兩鏈路交替 */
        int16_t rssi = (link == GS_LINK_920) ? (int16_t)-85 : GS_RSSI_NA;
        int16_t snr  = (link == GS_LINK_920) ? (int16_t)40  : GS_SNR_NA;
        gs_handle_packet(link, &pkt, rssi, snr);

        alt_m += dir * 20;                          /* 高度拋物線升降 */
        if (alt_m >= 3000) dir = -1;
        if (alt_m <= 0) { alt_m = 0; dir = 1; }
        tick += 100;

        gs_leds_update(HAL_GetTick(), 0);   /* 自測無實體 GPS：心跳 + 接收活動仍會動 */
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(100);   /* 10 Hz */
    }
}
#endif /* GS_USB_SELFTEST */

void GroundStation_Run(void)
{
    printf("\r\n[ROLE] GROUND_STATION  RX=E22(433)+E80(920)  GPS+SD+Flash+USB-CDC%s\r\n",
           GS_USB_SELFTEST ? "  [USB SELFTEST]" : "");
    printf("[LED] PE2 心跳(存活) / PE3 GPS定位 / PE4 接收活動\r\n");

    TelemRx_Init(&s_rx433);
    TelemRx_Init(&s_rx920);
    GsTimeSync_Init(&s_ts);
    gs_sd_open();
    gs_flash_init();
    GsLoraTest_Init();   /* 啟動 UART2 命令介面 + 統計 */

#if GS_USB_SELFTEST
    gs_usb_selftest_loop();   /* 不返回 */
#else
    /* 啟動 E22 USART3 中斷接收（E80 已於 main 進入連續 RX） */
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, s_u3_rxbuf, sizeof(s_u3_rxbuf));

    for (;;) {
        /* GPS：新 fix 時更新地面牆鐘錨點 */
        GPS_Update();
        const GPS_Data_t *g = GPS_GetData();
        if (g->fix_valid && g->last_fix_tick != s_last_fix_tick) {
            GsTimeSync_OnGpsFix(&s_ts, g->utc_hhmmss, g->last_fix_tick);
            s_last_fix_tick = g->last_fix_tick;
        }

        /* E22 433：取出環形緩衝餵入同步FSM */
        uint8_t b;
        TelemetryPacket_t pkt;
        while (u3_pop(&b)) {
            if (TelemRx_Feed(&s_rx433, b, &pkt)) {
                gs_handle_packet(GS_LINK_433, &pkt, GS_RSSI_NA, GS_SNR_NA);
            }
        }

        /* E80 920：DIO1 觸發則讀封包，payload 餵入同步FSM（含 RSSI/SNR） */
        if (LoRaE80_RxReady()) {
            uint8_t el = 0;
            int16_t rssi = GS_RSSI_NA, snr = GS_SNR_NA;
            HAL_StatusTypeDef rx_st = LoRaE80_ReadPacket(s_e80buf, &el, &rssi, &snr);
            if (rx_st == HAL_OK) {
                for (uint8_t i = 0; i < el; i++) {
                    if (TelemRx_Feed(&s_rx920, s_e80buf[i], &pkt)) {
                        gs_handle_packet(GS_LINK_920, &pkt, rssi, snr);
                    }
                }
            } else if (rx_st == HAL_ERROR) {
                /* CRC/header 錯誤：記入統計 */
                GsLoraTest_UpdateStats(GS_LINK_920, GS_RSSI_NA, GS_SNR_NA, 0 /* crc_err */);
            }
        }

        /* UART2 命令處理（地面站通訊測試） */
        GsLoraTest_Tick();

        /* 指示燈：心跳 / GPS fix / 接收活動 */
        gs_leds_update(HAL_GetTick(), g->fix_valid);

        HAL_IWDG_Refresh(&hiwdg);
        osDelay(GS_POLL_DELAY_MS);
    }
#endif /* GS_USB_SELFTEST */
}

#endif /* IS_GROUND */

/*
 * gs_lora_test.c — LoRa 通訊測試模組（地面站 UART2 命令介面 + 雙鏈路統計）
 * ===========================================================================
 * 整檔以 #if IS_GROUND 包住：航電板編譯為空。
 * 純換算 / 統計委派 lora_calc.h（與 host 測試 tests/test_lora_calc.c 共用）。
 */
#include "board_config.h"
#if IS_GROUND

#include "gs_lora_test.h"
#include "gs_log.h"        /* GS_LINK_433/920, GS_RSSI_NA, GS_SNR_NA */
#include "telemetry.h"     /* TELEM_PACKET_SIZE（空中時間估算用） */
#include "lora_calc.h"     /* 純換算 + lora_stats_t */
#include "uplink_proto.h"  /* 上行手動開傘命令框架（與火箭端共用） */
#include "lora_e22.h"
#include "lora_e80.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef  hspi3;    /* E80 重新初始化用 */
extern IWDG_HandleTypeDef hiwdg;    /* 上行 burst 期間餵狗 */

/* 上行命令以 burst 重複送 ~3s，確保落在火箭每 ~2s 一次的 1/10 接收窗 */
#define UPLINK_TX_REPEAT  24U
#define UPLINK_TX_GAP_MS  120U
static uint8_t s_uplink_seq = 0;

/* ============================================================
 *  UART2 環形接收緩衝
 * ============================================================ */
#define U2_RING_SZ  512U
static volatile uint8_t  s_u2_ring[U2_RING_SZ];
static volatile uint16_t s_u2_head = 0, s_u2_tail = 0;
static uint8_t           s_u2_rxbuf[64];   /* ReceiveToIdle 暫存 */

static void u2_push(uint8_t b)
{
    uint16_t nh = (uint16_t)((s_u2_head + 1U) % U2_RING_SZ);
    if (nh != s_u2_tail) { s_u2_ring[s_u2_head] = b; s_u2_head = nh; }
}
static int u2_pop(uint8_t *b)
{
    if (s_u2_tail == s_u2_head) return 0;
    *b = s_u2_ring[s_u2_tail];
    s_u2_tail = (uint16_t)((s_u2_tail + 1U) % U2_RING_SZ);
    return 1;
}

/* ============================================================
 *  狀態：統計 + 目前 RF 參數 + 自動列印
 * ============================================================ */
static lora_stats_t s_stat[2];   /* [0]=E22 433, [1]=E80 920 */

/* E80 影子參數：GsLoraTest_Init 以 LoRaE80_GetParams 從驅動同步實際套用值
 * （避免此處預設與 lora_e80.c 板級 #define 漂移，開機 `e80 show` 印錯值）。 */
static uint32_t s_e80_freq_hz  = 920000000UL;
static uint8_t  s_e80_sf       = 0x09U;  /* SF9 */
static uint8_t  s_e80_bw       = 0x05U;  /* 250kHz */
static uint8_t  s_e80_cr       = 0x01U;  /* 4/5 */
static int8_t   s_e80_pwr_dbm  = 22;
static uint16_t s_e80_preamble = 8U;
static uint32_t s_e22_freq_mhz = 432U;

static uint8_t  s_auto_stats   = 0;       /* 自動列印統計開關 */
static uint32_t s_auto_period_ms = 5000U;
static uint32_t s_auto_last_ms = 0;

static void stats_reset_all(void)
{
    lora_stats_reset(&s_stat[0]);
    lora_stats_reset(&s_stat[1]);
}

/* ============================================================
 *  列印
 * ============================================================ */
static void print_help(void)
{
    printf("\r\n=== LoRa 通訊測試命令（UART2 460800baud, 換行結尾）===\r\n"
           "  help              顯示此說明\r\n"
           "  role              回報角色/版本（GUI 自動偵測用）\r\n"
           "  ver               顯示 E80(LR1121) 版本/初始化診斷\r\n"
           "  stats             顯示雙鏈路統計\r\n"
           "  stats reset       清除統計\r\n"
           "  stats auto <sec>  每 N 秒自動列印統計（0=關閉）\r\n"
           "  e22 freq <mhz>    設 E22 頻率 (410-493)\r\n"
           "  e22 pwr  <0-3>    設 E22 功率 0=30 1=27 2=24 3=21dBm（3V3 供電建議 3）\r\n"
           "  e22 air  <0-7>    設 E22 空速 0=0.3k 1=1.2k 2=2.4k ... 7=62.5k（兩端須一致）\r\n"
           "  e22 show          顯示 E22 目前頻率\r\n"
           "  e80 freq <hz>     設 E80 中心頻率 Hz (e.g. 915000000)\r\n"
           "  e80 sf   <7-12>   設 E80 展頻因子\r\n"
           "  e80 bw   <idx>    設 E80 頻寬 (3=62.5k 4=125k 5=250k 6=500k)\r\n"
           "  e80 cr   <1-4>    設 E80 編碼率 (1=4/5 2=4/6 3=4/7 4=4/8)\r\n"
           "  e80 pwr  <dbm>    設 E80 發射功率 (-9~22 dBm)\r\n"
           "  e80 pre  <n>      設 E80 前導碼長度 (6~65535)\r\n"
           "  e80 show          顯示 E80 RF 參數 + 空中時間估算\r\n"
           "  e80 init          重新初始化 E80 並進入接收\r\n"
           "  e80 rxstart       重新進入連續接收\r\n"
           "  e80 airtime <len> 估算指定 payload 長度的空中時間\r\n"
           "  --- 上行手動開傘（433 反向，兩段式安全）---\r\n"
           "  ping              連線測試（火箭印出，不動作）\r\n"
           "  arm               武裝（火箭 30s 內允許開傘）\r\n"
           "  disarm            解除武裝\r\n"
           "  deploy drogue     手動開副傘（須先 arm）\r\n"
           "  deploy main       手動開主傘（須先 arm）\r\n"
           "  deploy both       手動同時開副傘+主傘（須先 arm）\r\n"
           "=====================================================\r\n");
}

static void print_one_stat(int i, const char *name)
{
    lora_stats_t *s = &s_stat[i];
    printf("\r\n[STATS] --- %s ---\r\n", name);
    printf("[STATS] pkt_ok=%lu  crc_err=%lu\r\n",
           (unsigned long)s->pkt_ok, (unsigned long)s->crc_err);

    uint32_t rate_x10 = lora_stats_rate_x10(s);
    if (rate_x10 > 0) {
        uint32_t el = (s->last_ms > s->first_ms) ? (s->last_ms - s->first_ms) : 0;
        printf("[STATS] rate=%lu.%lu pkt/s  elapsed=%lus\r\n",
               (unsigned long)(rate_x10 / 10), (unsigned long)(rate_x10 % 10),
               (unsigned long)(el / 1000));
    } else {
        printf("[STATS] rate=-- pkt/s\r\n");
    }

    if (i == GS_LINK_920 && s->rssi_cnt > 0) {
        printf("[STATS] RSSI: last=%d min=%d max=%d avg=%d dBm\r\n",
               (int)s->rssi_last, (int)s->rssi_min, (int)s->rssi_max,
               (int)lora_stats_rssi_avg(s));
        printf("[STATS] SNR:  last=%d min=%d max=%d avg=%d (x0.25dB)\r\n",
               (int)s->snr_last, (int)s->snr_min, (int)s->snr_max,
               (int)lora_stats_snr_avg(s));
    } else if (i == GS_LINK_433) {
        printf("[STATS] RSSI/SNR: N/A (E22 透傳模式無此資訊)\r\n");
    }
}

static void print_stats(void)
{
    print_one_stat(GS_LINK_433, "E22-433");
    print_one_stat(GS_LINK_920, "E80-920");
    printf("\r\n");
}

static void print_airtime(uint8_t payload_len)
{
    uint32_t toa = lora_time_on_air_us(s_e80_sf, s_e80_bw, s_e80_cr,
                                       s_e80_preamble, payload_len);
    if (toa == 0) { printf("[E80] airtime: 參數非法\r\n"); return; }
    /* 等效 bitrate = payload*8 bits / ToA ；以 bps 顯示 */
    uint32_t bps = (uint32_t)payload_len * 8u * 1000000u / toa;
    printf("[E80] payload=%u B  airtime=%lu.%03lu ms  ~%lu bps\r\n",
           (unsigned)payload_len,
           (unsigned long)(toa / 1000), (unsigned long)(toa % 1000),
           (unsigned long)bps);
}

static void print_e80_params(void)
{
    uint32_t bw_khz = lora_bw_to_khz(s_e80_bw);
    printf("[E80] freq=%lu Hz  SF%u  BW%lu kHz (idx=%u)  CR 4/%u  pwr=%d dBm  pre=%u  ldro=%u\r\n",
           (unsigned long)s_e80_freq_hz, (unsigned)s_e80_sf,
           (unsigned long)bw_khz, (unsigned)s_e80_bw,
           (unsigned)(s_e80_cr + 4U), (int)s_e80_pwr_dbm,
           (unsigned)s_e80_preamble, (unsigned)lora_ldro_required(s_e80_sf, s_e80_bw));
    print_airtime((uint8_t)TELEM_PACKET_SIZE);
}

static void print_e22_params(void)
{
    uint8_t ch = 0;
    e22_mhz_to_ch(s_e22_freq_mhz, &ch);
    printf("[E22] freq=%lu MHz  CH=%u\r\n", (unsigned long)s_e22_freq_mhz, (unsigned)ch);
}

static void print_version(void)
{
    int rd_st = -1; uint8_t busy = 0xFF, hw = 0, type = 0, gs = 0xFF;
    LoRaE80_GetInitDiag(&rd_st, &busy, &hw, &type, &gs);
    printf("[E80] LR1121 GetVersion HW=0x%02X Type=0x%02X (0x03=LR1121)  Stat1=0x%02X  init_rd=%d busy=%u\r\n",
           (unsigned)hw, (unsigned)type, (unsigned)gs, rd_st, (unsigned)busy);
}

static void apply_e80_reconfig(void)
{
    print_e80_params();
    HAL_StatusTypeDef st = LoRaE80_Reconfig(s_e80_freq_hz, s_e80_sf, s_e80_bw,
                                             s_e80_cr, s_e80_pwr_dbm, s_e80_preamble);
    printf(st == HAL_OK ? "[E80] reconfig OK -> RX restarted\r\n"
                        : "[E80] reconfig FAIL (st=%d)\r\n", (int)st);
}

/* ============================================================
 *  上行命令發送（手動開傘等，經 E22 433 反向打給火箭）
 * ============================================================ */
static void uplink_send(uint8_t cmd, uint8_t arg, const char *label)
{
    uint8_t f[UPLINK_FRAME_SIZE];
    uint8_t seq = s_uplink_seq++;
    UplinkProto_Build(f, cmd, arg, seq);
    printf("[UPLINK] 送 %s (cmd=0x%02X seq=%u) ×%u burst (~3s)…\r\n",
           label, (unsigned)cmd, (unsigned)seq, (unsigned)UPLINK_TX_REPEAT);
    for (uint8_t i = 0; i < UPLINK_TX_REPEAT; i++) {
        LoRaE22_Send(f, UPLINK_FRAME_SIZE);   /* 忙線跳過；burst 重複確保命中接收窗 */
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(UPLINK_TX_GAP_MS);
    }
    printf("[UPLINK] %s 送出完畢（看下行 DROGUE_FIRED/MAIN_DEPLOYED 旗標確認）\r\n", label);
}

/* ============================================================
 *  命令解析
 * ============================================================ */
static void str_tolower(char *s) { for (; *s; s++) *s = (char)tolower((unsigned char)*s); }

static void dispatch_cmd(char *line)
{
    str_tolower(line);

    char *tok[4] = {NULL, NULL, NULL, NULL};
    int   n = 0;
    char *p = line;
    while (*p && n < 4) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }
    if (n == 0) return;

    if (strcmp(tok[0], "help") == 0) {
        print_help();

    } else if (strcmp(tok[0], "role") == 0) {
        /* 與航電端 main.c Parse_Serial_Command 同格式（GUI 三角色自動偵測共用） */
        printf("[ROLE_ID] role=GROUND fw=%s\r\n", FIRMWARE_VERSION);

    } else if (strcmp(tok[0], "ping") == 0) {
        uplink_send(UPLINK_CMD_PING, 0, "PING");

    } else if (strcmp(tok[0], "arm") == 0) {
        uplink_send(UPLINK_CMD_ARM, 0, "ARM");

    } else if (strcmp(tok[0], "disarm") == 0) {
        uplink_send(UPLINK_CMD_DISARM, 0, "DISARM");

    } else if (strcmp(tok[0], "deploy") == 0 && n >= 2) {
        if (strcmp(tok[1], "drogue") == 0) {
            uplink_send(UPLINK_CMD_DEPLOY_DROGUE, 0, "DEPLOY-DROGUE");
        } else if (strcmp(tok[1], "main") == 0) {
            uplink_send(UPLINK_CMD_DEPLOY_MAIN, 0, "DEPLOY-MAIN");
        } else if (strcmp(tok[1], "both") == 0) {
            uplink_send(UPLINK_CMD_DEPLOY_BOTH, 0, "DEPLOY-BOTH");
        } else {
            printf("[UPLINK] 用法：deploy drogue|main|both（須先 arm）\r\n");
        }

    } else if (strcmp(tok[0], "ver") == 0) {
        print_version();

    } else if (strcmp(tok[0], "stats") == 0) {
        if (n >= 2 && strcmp(tok[1], "reset") == 0) {
            stats_reset_all();
            printf("[STATS] reset OK\r\n");
        } else if (n >= 3 && strcmp(tok[1], "auto") == 0) {
            uint32_t sec = (uint32_t)strtoul(tok[2], NULL, 10);
            if (sec == 0) {
                s_auto_stats = 0;
                printf("[STATS] auto off\r\n");
            } else {
                s_auto_stats = 1;
                s_auto_period_ms = sec * 1000U;
                s_auto_last_ms = HAL_GetTick();
                printf("[STATS] auto every %lus\r\n", (unsigned long)sec);
            }
        } else {
            print_stats();
        }

    } else if (strcmp(tok[0], "e22") == 0 && n >= 2) {
        if (strcmp(tok[1], "show") == 0) {
            print_e22_params();
        } else if (strcmp(tok[1], "freq") == 0 && n >= 3) {
            uint32_t mhz = (uint32_t)strtoul(tok[2], NULL, 10);
            uint8_t ch;
            if (!e22_mhz_to_ch(mhz, &ch)) {
                printf("[E22] freq 範圍 410-493 MHz\r\n"); return;
            }
            s_e22_freq_mhz = mhz;
            HAL_StatusTypeDef st = LoRaE22_SetFreqMHz(mhz);
            if (st == HAL_OK) printf("[E22] freq set %lu MHz (CH=%u) OK\r\n",
                                     (unsigned long)mhz, (unsigned)ch);
            else              printf("[E22] freq set FAIL (st=%d)\r\n", (int)st);
        } else if (strcmp(tok[1], "pwr") == 0 && n >= 3) {
            uint32_t lvl = (uint32_t)strtoul(tok[2], NULL, 10);
            if (lvl > 3U) {
                printf("[E22] pwr 等級 0=30dBm 1=27dBm 2=24dBm 3=21dBm（3V3 供電建議 3）\r\n");
                return;
            }
            HAL_StatusTypeDef st = LoRaE22_SetPowerLevel((uint8_t)lvl);
            if (st == HAL_OK) printf("[E22] pwr set level %lu OK\r\n", (unsigned long)lvl);
            else              printf("[E22] pwr set FAIL (st=%d)\r\n", (int)st);
        } else if (strcmp(tok[1], "air") == 0 && n >= 3) {
            uint32_t ar = (uint32_t)strtoul(tok[2], NULL, 10);
            if (ar > 7U) {
                printf("[E22] air 速率 0=0.3k 1=1.2k 2=2.4k 3=4.8k 4=9.6k 5=19.2k 6=38.4k 7=62.5k\r\n");
                return;
            }
            HAL_StatusTypeDef st = LoRaE22_SetAirRate((uint8_t)ar);
            if (st == HAL_OK) printf("[E22] air rate set %lu OK（兩端須一致）\r\n", (unsigned long)ar);
            else              printf("[E22] air rate set FAIL (st=%d)\r\n", (int)st);
        } else {
            printf("[E22] 未知子命令，輸入 help\r\n");
        }

    } else if (strcmp(tok[0], "e80") == 0 && n >= 2) {
        if (strcmp(tok[1], "show") == 0) {
            print_e80_params();

        } else if (strcmp(tok[1], "init") == 0) {
            HAL_StatusTypeDef st = LoRaE80_Init(&hspi3);
            if (st == HAL_OK) st = LoRaE80_StartRx();
            printf(st == HAL_OK ? "[E80] init + RX OK\r\n"
                                : "[E80] init FAIL (st=%d) — 檢查 SPI/RF開關/天線\r\n", (int)st);
            print_version();

        } else if (strcmp(tok[1], "rxstart") == 0) {
            HAL_StatusTypeDef st = LoRaE80_StartRx();
            printf(st == HAL_OK ? "[E80] RX restarted\r\n"
                                : "[E80] RX start FAIL (st=%d)\r\n", (int)st);

        } else if (strcmp(tok[1], "airtime") == 0 && n >= 3) {
            print_airtime((uint8_t)strtoul(tok[2], NULL, 10));

        } else if (strcmp(tok[1], "freq") == 0 && n >= 3) {
            uint32_t hz = (uint32_t)strtoul(tok[2], NULL, 10);
            if (hz < 862000000UL || hz > 928000000UL)
                printf("[E80] 注意：頻率建議 862-928 MHz，仍套用\r\n");
            s_e80_freq_hz = hz;
            apply_e80_reconfig();

        } else if (strcmp(tok[1], "sf") == 0 && n >= 3) {
            uint8_t sf = (uint8_t)atoi(tok[2]);
            if (sf < 7 || sf > 12) { printf("[E80] SF 範圍 7-12\r\n"); return; }
            s_e80_sf = sf; apply_e80_reconfig();

        } else if (strcmp(tok[1], "bw") == 0 && n >= 3) {
            uint8_t bw = (uint8_t)strtoul(tok[2], NULL, 10);
            if (!lora_bw_valid(bw)) {
                printf("[E80] BW idx 合法值: 0 1 2 3 4(125k) 5(250k) 6(500k) 8 9 10\r\n");
                return;
            }
            s_e80_bw = bw; apply_e80_reconfig();

        } else if (strcmp(tok[1], "cr") == 0 && n >= 3) {
            uint8_t cr = (uint8_t)atoi(tok[2]);
            if (cr < 1 || cr > 4) { printf("[E80] CR 範圍 1-4\r\n"); return; }
            s_e80_cr = cr; apply_e80_reconfig();

        } else if (strcmp(tok[1], "pwr") == 0 && n >= 3) {
            int pwr = atoi(tok[2]);
            if (pwr < -9 || pwr > 22) { printf("[E80] pwr 範圍 -9~22 dBm\r\n"); return; }
            s_e80_pwr_dbm = (int8_t)pwr; apply_e80_reconfig();

        } else if (strcmp(tok[1], "pre") == 0 && n >= 3) {
            long pre = atol(tok[2]);
            if (pre < 6 || pre > 65535) { printf("[E80] preamble 6~65535\r\n"); return; }
            s_e80_preamble = (uint16_t)pre; apply_e80_reconfig();

        } else {
            printf("[E80] 未知子命令，輸入 help\r\n");
        }

    } else {
        printf("[TEST] 未知命令 '%s'，輸入 help\r\n", tok[0]);
    }
}

/* ============================================================
 *  公開 API
 * ============================================================ */
void GsLoraTest_Init(void)
{
    stats_reset_all();
    /* 影子參數同步驅動實際套用值（LoRaE80_Init 已跑完；失敗時維持上方預設） */
    LoRaE80_GetParams(&s_e80_freq_hz, &s_e80_sf, &s_e80_bw,
                      &s_e80_cr, &s_e80_pwr_dbm, &s_e80_preamble);
    /* E22 已於 main() 初始化區（IS_GROUND 分支）init 完成（s_inited=1），故此處不重複
     * init —— 上行命令直接用 LoRaE22_Send 即可（透傳模式 TX/RX 皆已就緒）。 */
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_u2_rxbuf, sizeof(s_u2_rxbuf));
    printf("[TEST] LoRa 通訊測試模組就緒（UART2 460800），輸入 help\r\n");
}

void GsLoraTest_OnUart2RxEvent(uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) u2_push(s_u2_rxbuf[i]);
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_u2_rxbuf, sizeof(s_u2_rxbuf));
}

void GsLoraTest_UpdateStats(uint8_t link, int16_t rssi_dbm, int16_t snr_q, uint8_t crc_ok)
{
    if (link > 1U) return;
    int has_rssi = (link == GS_LINK_920) && (rssi_dbm != GS_RSSI_NA);
    int has_snr  = (link == GS_LINK_920) && (snr_q   != GS_SNR_NA);
    lora_stats_on_packet(&s_stat[link], crc_ok ? 1 : 0,
                         has_rssi, rssi_dbm, has_snr, snr_q, HAL_GetTick());
}

void GsLoraTest_Tick(void)
{
    /* 掃描 UART2 命令（換行結尾） */
    static char    s_cmd_buf[128];
    static uint8_t s_cmd_len = 0;

    uint8_t b;
    while (u2_pop(&b)) {
        if (b == '\r' || b == '\n') {
            /* 行終止符：\r、\n 或 \r\n 皆可（空行如 \r\n 的第二字元忽略） */
            if (s_cmd_len > 0) {
                s_cmd_buf[s_cmd_len] = '\0';
                dispatch_cmd(s_cmd_buf);
                s_cmd_len = 0;
            }
        } else if (s_cmd_len < (uint8_t)(sizeof(s_cmd_buf) - 1U)) {
            s_cmd_buf[s_cmd_len++] = (char)b;
        }
    }

    /* 自動列印統計 */
    if (s_auto_stats) {
        uint32_t now = HAL_GetTick();
        if ((now - s_auto_last_ms) >= s_auto_period_ms) {
            s_auto_last_ms = now;
            print_stats();
        }
    }
}

#endif /* IS_GROUND */

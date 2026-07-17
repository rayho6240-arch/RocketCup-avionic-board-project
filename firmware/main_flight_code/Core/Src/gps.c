/**
  ******************************************************************************
  * @file           : gps.c
  * @brief          : NMEA-0183 GPS driver (USART6, DMA-to-idle RX) — 實作
  *
  * GPS（NEO-M9N）實體掛載於 USART6（PC6=TX/PC7=RX），該埠已於 CubeMX 配置
  * 循環 DMA RX（DMA2_Stream1, ch5）＋ USART6 IDLE 中斷，故採 ReceiveToIdle_DMA：
  * DMA 於背景把位元組搬進 gps_dma_buf，IDLE/半傳輸/傳輸完成事件觸發
  * HAL_UARTEx_RxEventCallback，於其中把新位元組餵入 GPS_FeedByte 組句。
  ******************************************************************************
  */
#include "gps.h"
#include "rate_monitor.h"
#include "main.h"
#include <string.h>

extern IWDG_HandleTypeDef hiwdg;

/* ── GPS bring-up 隔離測試開關 ─────────────────────────────────────────────
 * 0 = 正常：460800 動態協商 + 10Hz + CFG-RST。★上板實測此路徑 GPS RAW 有資料，
 *     模組實際跑在 460800（推測由 BBR/備援電池保留上次設定），為正式組態。
 * 1 = 最小收訊模式：跳過所有 UBX 協商，USART6 固定於 GPS_MINIMAL_BAUD 直接收。
 *     ★上板實測 9600 完全收不到（模組不在 9600），僅保留供日後鮑率隔離除錯。 */
#ifndef GPS_BRINGUP_MINIMAL
#define GPS_BRINGUP_MINIMAL 0
#endif
/* 最小模式鮑率：當初測試成功用 9600；NEO-M9N 冷開機出廠預設為 38400，收不到時可改試。 */
#ifndef GPS_MINIMAL_BAUD
#define GPS_MINIMAL_BAUD    9600U
#endif

/* 純解析邏輯（組句狀態機 / checksum / GGA / RMC）已抽至 gps_parse.h（host 可測，
 * tests/test_gps.c），本檔僅保留不純的部分：DMA 環形差分、ISR↔task 交接、UBX 初始化。 */
#define GPS_LINE_MAX   GPS_PARSE_LINE_MAX

/* --- 模組狀態 --- */
static UART_HandleTypeDef *gps_huart = NULL;

/* USART6_RX 循環 DMA 緩衝：DMA2 僅能存取主 SRAM（非 CCM），此 static 落於 .bss 主 SRAM。
 * 256 bytes ≈ 115200 baud 下 22 ms 連續資料；GPS 每秒一陣 NMEA，IDLE/HT/TC 事件可即時排空。 */
#define GPS_DMA_BUF_SIZE  256U
static uint8_t  gps_dma_buf[GPS_DMA_BUF_SIZE];
static uint16_t gps_dma_old_pos = 0;               /* 上次已處理到的 DMA 寫入位置（環形） */

static GpsLineAsm_t gps_line_asm;                  /* ISR 內組句狀態機（gps_parse.h 純邏輯） */
static char     gps_ready[GPS_LINE_MAX];           /* 已就緒、待 task 解析的整句 */
static volatile uint8_t gps_line_ready = 0;        /* 1 = gps_ready 有一句待解析 */
static volatile uint32_t gps_overrun_drops = 0;    /* task 還沒解析就被新句覆蓋的次數 */

static GPS_Data_t gps_data;                        /* 對外解析結果 */

/* ------------------------------------------------------------------ */
/* 低階：RX 中斷組裝整句                                               */
/* ------------------------------------------------------------------ */

/* 由 HAL_UARTEx_RxEventCallback（ISR context）對每個新收到的位元組呼叫。
 * 組句規則在 gps_line_feed()（未見 '$' 不收、溢位 discard 至下一個 '$'），
 * 此處僅做 ready buffer 交接。 */
static void GPS_FeedByte(uint8_t b)
{
    if (gps_line_feed(&gps_line_asm, b)) {
        if (gps_line_ready) {
            /* 上一句還沒被 task 取走 → 覆蓋並計數（GPS 10Hz、主迴圈快，理論上不會發生） */
            gps_overrun_drops++;
        }
        memcpy(gps_ready, gps_line_asm.buf, (size_t)gps_line_asm.len + 1U);
        gps_line_ready = 1;
    }
}

/* ------------------------------------------------------------------ */
/* 公開 API                                                            */
/* ------------------------------------------------------------------ */

void GPS_Init(UART_HandleTypeDef *huart)
{
    gps_huart = huart;
    memset(&gps_data, 0, sizeof(gps_data));
    gps_line_asm_init(&gps_line_asm);
    gps_line_ready = 0;
    gps_dma_old_pos = 0;

    if (gps_huart) {
#if GPS_BRINGUP_MINIMAL
        /* ── 最小收訊模式：不送任何 UBX，USART6 固定鮑率直接收 NMEA ──
         * 對照「當初測試成功」的組態，隔離 460800 協商是否為收不到主因。 */
        HAL_UART_DeInit(gps_huart);
        gps_huart->Init.BaudRate = GPS_MINIMAL_BAUD;
        HAL_UART_Init(gps_huart);
        HAL_Delay(20);
        if (hiwdg.Instance != NULL) {
            HAL_IWDG_Refresh(&hiwdg);
        }
        __HAL_UART_CLEAR_OREFLAG(gps_huart);
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, gps_dma_buf, GPS_DMA_BUF_SIZE);
#else
        /* 0. 硬體重置 GPS 模組 (已停用：避免引腳拉低造成 MCU 重設或電源抖動，改採軟體協商) */
        // HAL_GPIO_WritePin(RST_GPS_GPIO_Port, RST_GPS_Pin, GPIO_PIN_RESET);
        // HAL_Delay(20);
        // HAL_GPIO_WritePin(RST_GPS_GPIO_Port, RST_GPS_Pin, GPIO_PIN_SET);
        // HAL_Delay(200);
        if (hiwdg.Instance != NULL) {
            HAL_IWDG_Refresh(&hiwdg);
        }

        /* 1. 定義 UBX 設置指令 */
        /* UBX-CFG-PRT: 設置 UART1 鮑率為 460800, 8N1, 輸入/輸出為 UBX+NMEA */
        const uint8_t UBX_CFG_PRT_460800[] = {
            0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 
            0xD0, 0x08, 0x00, 0x00, 0x00, 0x08, 0x07, 0x00, 0x07, 0x00, 
            0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xBC
        };
        
        /* UBX-CFG-RATE: 設置定位頻率為 10 Hz (100ms 測量週期) */
        const uint8_t UBX_CFG_RATE_10HZ[] = {
            0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 
            0x01, 0x00, 0x7A, 0x12
        };

        /* 2. 動態鮑率協商序列 (嘗試以多種鮑率發送設置命令，以相容各類初始狀態；加入 460800 保障熱重啟) */
        const uint32_t try_bauds[] = {38400, 115200, 9600, 460800};
        for (int i = 0; i < 4; i++) {
            /* 切換 MCU UART 至嘗試的鮑率 */
            HAL_UART_DeInit(gps_huart);
            gps_huart->Init.BaudRate = try_bauds[i];
            HAL_UART_Init(gps_huart);
            
            /* 發送變更鮑率命令 */
            HAL_UART_Transmit(gps_huart, (uint8_t*)UBX_CFG_PRT_460800, sizeof(UBX_CFG_PRT_460800), 50);
            HAL_Delay(50); /* 延長延遲：確保 command 傳送完成 (9600 bps 下 28 bytes 需 ~29ms) 且 GPS 處理完畢 */
            if (hiwdg.Instance != NULL) {
                HAL_IWDG_Refresh(&hiwdg);
            }
        }

        /* 3. 將 MCU UART 固定於最終目標鮑率 460800 */
        HAL_UART_DeInit(gps_huart);
        gps_huart->Init.BaudRate = 460800;
        HAL_UART_Init(gps_huart);
        HAL_Delay(20);
        if (hiwdg.Instance != NULL) {
            HAL_IWDG_Refresh(&hiwdg);
        }

        /* 4. 在 460800 鮑率下，發送設置定位更新率為 10Hz 命明 */
        HAL_UART_Transmit(gps_huart, (uint8_t*)UBX_CFG_RATE_10HZ, sizeof(UBX_CFG_RATE_10HZ), 50);
        HAL_Delay(10);

        /* 4b. 發送 UBX-CFG-RST 軟體重置命令 (GNSS-only 模式，重設尋星引擎但不影響 UART 鮑率) */
        const uint8_t UBX_CFG_RST_GNSS_ONLY[] = {
            0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68
        };
        HAL_UART_Transmit(gps_huart, (uint8_t*)UBX_CFG_RST_GNSS_ONLY, sizeof(UBX_CFG_RST_GNSS_ONLY), 50);
        HAL_Delay(20);

        /* 5. 啟動 IDLE-line + 循環 DMA 接收 (啟動前清空 ORE/FE 旗標，防止 DMA 因殘留錯誤拒絕啟動) */
        __HAL_UART_CLEAR_OREFLAG(gps_huart);
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, gps_dma_buf, GPS_DMA_BUF_SIZE);
#endif /* GPS_BRINGUP_MINIMAL */
    }
}

uint8_t GPS_Update(void)
{
    if (!gps_line_ready) return 0;

    /* 取出就緒句（以 NVIC 關閉 UART 中斷做局部臨界區保護，防止與 ISR 競爭 ready flag 與 buffer） */
    char line[GPS_LINE_MAX];
    HAL_NVIC_DisableIRQ(USART6_IRQn);
    memcpy(line, gps_ready, GPS_LINE_MAX);
    gps_line_ready = 0;
    HAL_NVIC_EnableIRQ(USART6_IRQn);

    printf("[GPS_RAW] %s\r\n", line);

    uint8_t kind = gps_parse_sentence(&gps_data, line, HAL_GetTick());
    if (kind == GPS_SENT_GGA || kind == GPS_SENT_RMC) {
        RATE_TICK_GPS();
    }
    /* GPS_SENT_SKIP：其他句型（GSV/GSA/VTG...）略過；GPS_SENT_BAD 已計 sentences_err */
    return 1;
}

const GPS_Data_t* GPS_GetData(void)
{
    return &gps_data;
}

uint8_t GPS_IsStale(uint32_t timeout_ms)
{
    if (!gps_data.fix_valid) return 1;
    return (HAL_GetTick() - gps_data.last_fix_tick) > timeout_ms;
}

/* ------------------------------------------------------------------ */
/* HAL UART 回呼處理（以 instance 過濾僅處理 USART6）                   */
/* 弱符號 HAL_UARTEx_RxEventCallback / HAL_UART_ErrorCallback 改由 main.c */
/* 統一定義，依 instance 轉接至此（GPS）與 Link_*（USART2 板間鏈路）。   */
/* ------------------------------------------------------------------ */

/* 循環 DMA 接收事件處理：IDLE-line、半傳輸 (HT)、傳輸完成 (TC) 皆觸發。
 * Size = 自緩衝起點至目前 DMA 寫入位置的累計位元組數；以 gps_dma_old_pos 差分取出新位元組。
 * 於 ISR context 執行；僅做位元組搬移與旗標設定，不呼叫任何 FreeRTOS API（ISR-safe）。 */
void GPS_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (!gps_huart || huart->Instance != gps_huart->Instance) return;

    if (Size != gps_dma_old_pos) {
        if (Size > gps_dma_old_pos) {
            for (uint16_t i = gps_dma_old_pos; i < Size; i++) GPS_FeedByte(gps_dma_buf[i]);
        } else {
            /* 環形回繞：先處理 old_pos..尾端，再 0..Size */
            for (uint16_t i = gps_dma_old_pos; i < GPS_DMA_BUF_SIZE; i++) GPS_FeedByte(gps_dma_buf[i]);
            for (uint16_t i = 0; i < Size; i++) GPS_FeedByte(gps_dma_buf[i]);
        }
        gps_dma_old_pos = Size;
        if (gps_dma_old_pos >= GPS_DMA_BUF_SIZE) gps_dma_old_pos = 0;
    }
}

void GPS_HandleUartError(UART_HandleTypeDef *huart)
{
    if (gps_huart && huart->Instance == gps_huart->Instance) {
        /* UART 錯誤（常見為 overrun）會中止 DMA 接收 → 清狀態並重啟，避免 GPS RX 死掉 */
        __HAL_UART_CLEAR_OREFLAG(gps_huart);
        gps_line_asm_init(&gps_line_asm);
        gps_dma_old_pos = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, gps_dma_buf, GPS_DMA_BUF_SIZE);
    }
}

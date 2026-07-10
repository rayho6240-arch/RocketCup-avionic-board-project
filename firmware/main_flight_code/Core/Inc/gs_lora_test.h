/*
 * gs_lora_test.h — LoRa 通訊測試模組（地面站 UART2 命令介面 + 雙鏈路統計）
 * ===========================================================================
 * 透過 UART2（PA2/TX, PA3/RX, 460800 baud）與電腦雙向通訊：
 *   → 電腦送文字命令（\n 結尾）調整 E22/E80 RF 參數
 *   ← STM32 回傳確認訊息、統計資料（與 printf 混用同一 UART2 TX）
 *
 * 命令列表（不分大小寫）：
 *   help                  → 列出所有命令
 *   stats                 → 顯示雙鏈路統計（封包數/CRC錯誤/RSSI/SNR/封包率）
 *   stats reset           → 清除統計
 *   e22 freq <mhz>        → 設 E22 頻率 (410-493 MHz)
 *   e80 freq <hz>         → 設 E80 中心頻率 Hz (e.g. 920000000)
 *   e80 sf <7-12>         → 設 E80 展頻因子 SF7~SF12
 *   e80 bw <idx>          → 設 E80 頻寬 (0=7.8k 3=62.5k 4=125k 5=250k 6=500k)
 *   e80 cr <1-4>          → 設 E80 編碼率 (1=4/5 2=4/6 3=4/7 4=4/8)
 *   e80 pwr <dbm>         → 設 E80 發射功率 (-3~22 dBm)
 *   e80 pre <n>           → 設 E80 前導碼長度 (6~65535)
 *   e80 show              → 顯示 E80 目前 RF 參數
 *   e22 show              → 顯示 E22 目前頻率
 *
 * 整檔以 #if IS_GROUND 包住：航電板編譯為空。
 */
#ifndef GS_LORA_TEST_H
#define GS_LORA_TEST_H

#include "stm32f4xx_hal.h"
#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if IS_GROUND

/** @brief 初始化：啟動 UART2 ReceiveToIdle 中斷。於 GroundStation_Run 開頭呼叫。 */
void GsLoraTest_Init(void);

/** @brief UART2 RxEvent 回呼（由 main.c HAL_UARTEx_RxEventCallback 轉接）。 */
void GsLoraTest_OnUart2RxEvent(uint16_t size);

/**
 * @brief 更新某鏈路統計（每收到一筆封包呼叫）。
 * @param link     GS_LINK_433 / GS_LINK_920
 * @param rssi_dbm RSSI (dBm)；GS_RSSI_NA 表示無此值
 * @param snr_q    SNR (SX126x 原值, dB×4)；GS_SNR_NA 表示無此值
 * @param crc_ok   1=有效封包，0=CRC/header 錯誤
 */
void GsLoraTest_UpdateStats(uint8_t link, int16_t rssi_dbm, int16_t snr_q, uint8_t crc_ok);

/** @brief 主迴圈 tick：掃描 UART2 RX 環形緩衝、解析並執行命令。每 poll 呼叫一次。 */
void GsLoraTest_Tick(void);

#endif /* IS_GROUND */

#ifdef __cplusplus
}
#endif

#endif /* GS_LORA_TEST_H */

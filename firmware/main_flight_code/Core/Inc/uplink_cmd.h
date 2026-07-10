/*
 * uplink_cmd.h — 火箭端上行命令處理（433 反向鏈路 → 手動開傘）
 * ===========================================================================
 * 主航電（FEATURE_UPLINK_DEPLOY）在下行遙測之外，於 USART3（E22 433 透傳）持續
 * 接收地面站上行命令（uplink_proto.h 框架）。命令解析、兩段式武裝（ARM→DEPLOY）、
 * ARM 逾時自動解除皆在本模組；實際點火動作（PD13 / TIM4）留在 main.c 飛控迴圈，
 * 經 UplinkCmd_TakeDeploy() 取出待辦旗標後執行（與既有 FSM 點火路徑並存）。
 *
 * 整檔以 #if FEATURE_UPLINK_DEPLOY 包住：備援/地面站編譯為空。
 */
#ifndef UPLINK_CMD_H
#define UPLINK_CMD_H

#include "stm32f4xx_hal.h"
#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if FEATURE_UPLINK_DEPLOY

/** @brief 啟動 USART3 上行接收（ReceiveToIdle 中斷）。於 main() E22 init 成功後呼叫。 */
void UplinkCmd_Init(void);

/** @brief USART3 RxEvent 回呼（由 main.c HAL_UARTEx_RxEventCallback 轉接）。 */
void UplinkCmd_OnUart3RxEvent(uint16_t size);

/** @brief USART3 錯誤(ORE/雜訊)復原：清旗標並重新掛載 ReceiveToIdle。
 *  由 main.c 的 HAL_UART_ErrorCallback 在 huart->Instance==USART3 時呼叫。 */
void UplinkCmd_OnUart3Error(void);

/** @brief 處理已收位元組：解析框架、執行 ARM/DISARM、設定待辦開傘旗標、ARM 逾時解除。
 *  由低優先遙測任務每週期呼叫一次（解析不可放 1kHz 飛控迴圈）。 */
void UplinkCmd_Poll(uint32_t now_ms);

/**
 * @brief 取出並清除待辦的手動開傘請求（一次性消費）。由飛控迴圈呼叫。
 * @param want_drogue 輸出：1 = 本次有手動副傘請求
 * @param want_main   輸出：1 = 本次有手動主傘請求
 * @return 1 = 至少一項待辦（呼叫端應執行點火）；0 = 無。
 */
uint8_t UplinkCmd_TakeDeploy(uint8_t *want_drogue, uint8_t *want_main);

/** @brief 目前是否武裝（ARM 窗內）。 */
uint8_t UplinkCmd_IsArmed(void);

/** @brief 診斷統計：有效命令數 / CRC 錯誤數 / 最後命令碼。可傳 NULL。 */
void UplinkCmd_GetStats(uint32_t *rx_ok, uint32_t *rx_crc_err, uint8_t *last_cmd);

#endif /* FEATURE_UPLINK_DEPLOY */

#ifdef __cplusplus
}
#endif

#endif /* UPLINK_CMD_H */

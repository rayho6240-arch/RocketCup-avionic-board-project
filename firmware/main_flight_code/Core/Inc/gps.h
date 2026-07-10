/**
  ******************************************************************************
  * @file           : gps.h
  * @brief          : NMEA-0183 GPS driver (USART6, DMA-to-idle RX)
  *
  * 自包含的 NMEA 解析器：解析 GGA（定位/高度/衛星數）與 RMC（定位/地速/航向/有效性）。
  * GPS（NEO-M9N）實體掛載於 USART6；RX 採循環 DMA + IDLE 中斷（HAL_UARTEx_ReceiveToIdle_DMA），
  * 於事件回呼內組裝整句、設旗標，由主迴圈呼叫 GPS_Update() 在 task context 解析，
  * 避免在 ISR 內做繁重字串處理。
  ******************************************************************************
  */
#ifndef __GPS_H
#define __GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "gps_parse.h"   /* GPS_Data_t 與純解析邏輯（host 可測）移居於此 */

/* --- API --- */

/* 綁定 UART 並啟動逐位元組中斷接收。需在 USART3 初始化後呼叫一次。 */
void GPS_Init(UART_HandleTypeDef *huart);

/* 主迴圈呼叫：若有整句就緒則解析並更新內部 GPS_Data。回傳本次是否有解析動作。
 * 可每個迴圈呼叫（無資料時極輕量，只檢查旗標）。 */
uint8_t GPS_Update(void);

/* 取得最近一次解析結果（指向內部 static，唯讀）。 */
const GPS_Data_t* GPS_GetData(void);

/* 距上次有效定位是否已逾時（ms）。無 fix 或長時間未更新時回傳 1。 */
uint8_t GPS_IsStale(uint32_t timeout_ms);

/* HAL 回呼處理（由 main.c 的統一 HAL_UARTEx_RxEventCallback / HAL_UART_ErrorCallback
 * 依 instance 轉接呼叫；內部自行過濾僅處理 GPS 綁定的 USART6）。 */
void GPS_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size);
void GPS_HandleUartError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_H */

/*
 * ground_station.h — 地面站接收器主流程（ROLE_GROUND）
 * ===========================================================================
 * 雙鏈路接收火箭下行遙測（E22 433 透傳 / E80 920 SX126x）+ 讀自身 GPS，
 * 對齊時間戳後記錄到 SD（CSV）+ Flash（二進位 append）並經 USB-CDC 串流給 PC。
 * 由 main.c 的 defaultTask 在 IS_GROUND 時呼叫 GroundStation_Run()（不返回）。
 *
 * 本檔內容整體以 #if IS_GROUND 包住：主/備航電編譯為空，零影響。
 */
#ifndef GROUND_STATION_H
#define GROUND_STATION_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 地面站主迴圈（不返回）。由 StartDefaultTask 在 IS_GROUND 時呼叫。 */
void GroundStation_Run(void);

/** @brief USART3（E22 433 透傳）RxEvent 回呼：把收到的 Size bytes 推入位元組環形緩衝。
 *  由 main.c 的 HAL_UARTEx_RxEventCallback 在 huart->Instance==USART3 時呼叫。 */
void GroundStation_OnUart3RxEvent(uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* GROUND_STATION_H */

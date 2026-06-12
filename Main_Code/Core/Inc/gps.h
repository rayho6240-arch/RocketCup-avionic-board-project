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

/* --- 解析後的 GPS 資料 --- */
typedef struct {
    uint8_t  fix_valid;        /* 1 = RMC 狀態 'A' 或 GGA fix>0（有有效定位） */
    uint8_t  fix_quality;      /* GGA fix quality：0=invalid,1=GPS,2=DGPS...   */
    uint8_t  satellites;       /* GGA 使用衛星數                               */
    int32_t  lat_1e6;          /* 緯度 ×1e6 (deg)，+北 / −南                    */
    int32_t  lon_1e6;          /* 經度 ×1e6 (deg)，+東 / −西                    */
    float    altitude_m;       /* GGA 海拔高度 (m, MSL)                         */
    float    geoid_sep_m;      /* GGA 大地水準面分離 (m)                        */
    float    speed_mps;        /* RMC 地速 (m/s，由 knots 換算)                 */
    float    course_deg;       /* RMC 航向 (deg true)                          */
    uint32_t utc_hhmmss;       /* UTC 時間 hhmmss（整數部分）                    */
    uint32_t last_fix_tick;    /* 最近一次有效定位的 HAL_GetTick() (ms)         */
    uint32_t sentences_ok;     /* 通過 checksum 並成功解析的句數（診斷）         */
    uint32_t sentences_err;    /* checksum 失敗或格式錯誤的句數（診斷）          */
} GPS_Data_t;

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

#ifdef __cplusplus
}
#endif

#endif /* __GPS_H */

/**
 ******************************************************************************
 * @file    lora_e22.h
 * @brief   E22-400T30S 433MHz LoRa 透傳模式驅動 (UART3)
 *
 * E22 為 SX1268 為核心之 UART 透傳模組。模式由 M1/M0 決定（見 main.h LORA433_*）：
 *     M1=0 M0=0 → 透傳(Normal)：寫入 UART 的位元組即經空中發送（本驅動使用）
 *     M1=0 M0=1 → WOR
 *     M1=1 M0=0 → 設定(暫存器)
 *     M1=1 M0=1 → 深度睡眠
 * AUX(PE11)：HIGH=空閒可發送，LOW=忙線(發送中/模式切換/開機)。
 * 接線（連線和基本硬體規格表.md）：UART3 TX=PD8/RX=PD9, M0=PD11, M1=PD10, RST=PD12, AUX=PE11。
 * 腳位與 UART 已由 CubeMX 初始化（M0=M1=0 透傳、RST 釋放、AUX 為輸入）。
 ******************************************************************************
 */
#ifndef __LORA_E22_H
#define __LORA_E22_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * @brief 綁定 UART 並將模組設為透傳模式（M0=M1=0）+ 硬體重置，等 AUX 拉高。
 *        於 main() 初始化區（scheduler 啟動前）呼叫一次。
 * @param huart E22 所掛的 UART（本專案為 &huart3）。
 */
void LoRaE22_Init(UART_HandleTypeDef *huart);

/**
 * @brief 透傳發送一筆位元組。
 * @return HAL_OK 已送出；HAL_BUSY 模組忙線(AUX low)本次跳過；HAL_ERROR 參數錯誤/未初始化。
 * @note  AUX 背壓：忙線即跳過不阻塞，由呼叫端（遙測任務）自我限流以對齊空中速率。
 */
HAL_StatusTypeDef LoRaE22_Send(const uint8_t *data, uint16_t len);

/** @brief 模組是否空閒可發送（AUX=HIGH 且已初始化）。 */
uint8_t LoRaE22_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_E22_H */

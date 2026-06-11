/**
 ******************************************************************************
 * @file    lora_e80.h
 * @brief   E80-900M2213S 920MHz LoRa 驅動 (SX126x/LLCC68 命令相容, SPI3)
 *
 * E80-900M2213S 以 Semtech SX126x/LLCC68 為核心，SPI 命令協定相同。
 * 接線（連線和基本硬體規格表.md）：SPI3(SCK=PB3/MISO=PB4/MOSI=PB5),
 *   CS=PD7(CSB_LORA920), RST=PD5, DIO1/INT=PD4(EXTI4 rising), BUSY=PD6。
 * SPI3 與 W25Q128 Flash 共用 → 經 spi3_bus.h 的 SPI3_Bus_Lock/Unlock 互斥。
 *
 * ★ RF 參數（頻率/SF/BW/CR/PA/TCXO/RF-switch）集中於 lora_e80.c 頂部 #define，
 *   預設為多數 Ebyte 900MHz SX126x 模組之常見組態，但**務必於上板 bring-up
 *   時對照 E80-900M2213S 規格書與地面站設定逐項驗證**（尤其 TCXO 與 DIO2 RF switch）。
 ******************************************************************************
 */
#ifndef __LORA_E80_H
#define __LORA_E80_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * @brief 初始化 E80（重置 → standby → LoRa 參數 → DIO IRQ），並讀回 sync word 驗活。
 * @param hspi E80 所掛之 SPI（本專案為 &hspi3）。
 * @return HAL_OK 晶片在線且設定完成；HAL_ERROR/HAL_TIMEOUT 未偵測到模組。
 * @note  於 main() 初始化區（scheduler 啟動前）呼叫一次。
 */
HAL_StatusTypeDef LoRaE80_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief 非阻塞發送一筆 LoRa 封包。
 *        若上一筆 TX 尚未完成（DIO1 TxDone 未到且未逾時）則跳過本次回傳 HAL_BUSY，
 *        否則載入緩衝區並啟動 TX 後立即返回（不空等空中傳輸完成）。
 * @param len 封包長度 (1..255 bytes)。
 * @return HAL_OK 已啟動；HAL_BUSY 前次未完成本次跳過；HAL_ERROR 參數/未初始化。
 */
HAL_StatusTypeDef LoRaE80_Send(const uint8_t *data, uint8_t len);

/** @brief 是否可開始新的一筆發送（已初始化、BUSY 低、且無 TX 進行中）。 */
uint8_t LoRaE80_IsReady(void);

/** @brief DIO1(EXTI4) 中斷服務：設定 TxDone 旗標。由 HAL_GPIO_EXTI_Callback 呼叫。 */
void LoRaE80_OnDio1IRQ(void);

/** @brief 填入初始化診斷結果供週期性遙測輸出。gs = GetStatus byte (0x22=OK). */
void LoRaE80_GetInitDiag(int *rd_st, uint8_t *busy, uint8_t *rb0, uint8_t *rb1, uint8_t *gs);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_E80_H */

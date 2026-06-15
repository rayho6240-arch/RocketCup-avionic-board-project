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

/* ============================================================
 *  模組啟用開關
 * ============================================================
 * LORA920_ENABLE = 0：開機不初始化 E80，改呼叫 LoRaE80_Shutdown() 把模組
 *   按在 reset（RST 拉低、CS 拉高）—— SX126x 於 NRST 拉低時所有腳位呈高阻，
 *   E80 因此「物理斷開」於 SPI3 共用的 SCK/MISO/MOSI，不會污染與其同匯流排
 *   的 W25Q128 Flash 讀寫（飛行資料記錄優先）。
 *   ★ E80 920MHz 鏈路目前停用（bring-up 問題待解）。硬體修好後改回 1 即恢復。
 * LORA920_ENABLE = 1：正常初始化；若偵測失敗（含 MISO 接地/浮空）亦會自動
 *   走 Shutdown 隔離，故障的 E80 絕不被放任掛在 SPI3 上。 */
#ifndef LORA920_ENABLE
#define LORA920_ENABLE 0
#endif

/**
 * @brief 初始化 E80（重置 → standby → LoRa 參數 → DIO IRQ），並讀回 sync word 驗活。
 * @param hspi E80 所掛之 SPI（本專案為 &hspi3）。
 * @return HAL_OK 晶片在線且設定完成；HAL_ERROR/HAL_TIMEOUT 未偵測到模組
 *         （含早期 MISO 健康檢查失敗：GetStatus=0x00 接地 / 0xFF 浮空）。
 * @note  於 main() 初始化區（scheduler 啟動前）呼叫一次。
 */
HAL_StatusTypeDef LoRaE80_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief 安全停用 E80 並從 SPI3 匯流排隔離：CS 拉高 + RST 拉低並保持。
 *        SX126x NRST 拉低時全腳高阻 → E80 不再驅動共用 MISO，保護同匯流排
 *        的 W25Q128 Flash。停用後 LoRaE80_Send/IsReady 一律不觸碰 SPI3。
 *        僅操作 GPIO（不碰 SPI/mutex），可於 scheduler 啟動前安全呼叫。
 *        用於：①LORA920_ENABLE=0 主動停用；②Init 偵測失敗的故障隔離。
 */
void LoRaE80_Shutdown(void);

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

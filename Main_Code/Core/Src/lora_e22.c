/**
 ******************************************************************************
 * @file    lora_e22.c
 * @brief   E22-400T30S 433MHz LoRa 透傳模式驅動 (UART3)
 ******************************************************************************
 */
#include "lora_e22.h"
#include "main.h"   /* LORA433_* 腳位巨集 */

#define LORA_E22_AUX_BOOT_TIMEOUT_MS  300U   /* 開機/重置後等 AUX 拉高上限 */
#define LORA_E22_TX_TIMEOUT_MS        100U   /* UART 阻塞傳輸逾時 */

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t             s_inited = 0;

/* AUX(PE11)：HIGH=空閒。等待拉高，含逾時保護。回傳 1=就緒, 0=逾時。 */
static uint8_t e22_wait_aux_high(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LORA433_BUSY_GPIO_Port, LORA433_BUSY_Pin) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return 0;
        }
    }
    return 1;
}

void LoRaE22_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    /* 透傳模式 M1=0, M0=0（CubeMX 已設，這裡顯式重申確保狀態正確） */
    HAL_GPIO_WritePin(LORA433_M0_GPIO_Port, LORA433_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);

    /* 硬體重置脈衝：RST 拉低 ~5ms 再釋放（RST 低電平有效） */
    HAL_GPIO_WritePin(LORA433_RST_GPIO_Port, LORA433_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LORA433_RST_GPIO_Port, LORA433_RST_Pin, GPIO_PIN_SET);

    /* 等模組開機完成（AUX 拉高）；逾時也繼續（模組可能未接，發送由 AUX 背壓守護） */
    (void)e22_wait_aux_high(LORA_E22_AUX_BOOT_TIMEOUT_MS);

    s_inited = 1;
}

uint8_t LoRaE22_IsReady(void)
{
    return (s_inited &&
            HAL_GPIO_ReadPin(LORA433_BUSY_GPIO_Port, LORA433_BUSY_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

HAL_StatusTypeDef LoRaE22_Send(const uint8_t *data, uint16_t len)
{
    if (!s_inited || s_huart == NULL || data == NULL || len == 0) {
        return HAL_ERROR;
    }
    /* AUX 背壓：忙線即跳過本次（不阻塞），讓遙測自我限流對齊空中速率 */
    if (HAL_GPIO_ReadPin(LORA433_BUSY_GPIO_Port, LORA433_BUSY_Pin) == GPIO_PIN_RESET) {
        return HAL_BUSY;
    }
    return HAL_UART_Transmit(s_huart, (uint8_t *)data, len, LORA_E22_TX_TIMEOUT_MS);
}

/**
 ******************************************************************************
 * @file    lora_e22.c
 * @brief   E22-400T30S 433MHz LoRa 透傳模式驅動 (UART3)
 ******************************************************************************
 */
#include "lora_e22.h"
#include "main.h"   /* LORA433_* 腳位巨集 */
#include <stdio.h>

#define LORA_E22_AUX_BOOT_TIMEOUT_MS  1000U  /* 開機/重置後等 AUX 拉高上限 */
#define LORA_E22_TX_TIMEOUT_MS        100U   /* UART 阻塞傳輸逾時 */

/* 頻率設定：E22-400T30S 頻率 = 410 + CH (MHz)，CH 寫入 EEPROM 暫存器 0x05 */
#define E22_FREQ_BASE_MHZ  410U
#define E22_TX_FREQ_MHZ    432U
#define E22_CH             ((uint8_t)(E22_TX_FREQ_MHZ - E22_FREQ_BASE_MHZ))

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

/* 進入設定模式，寫入指定頻道後回透傳模式（帶 ch 參數版，供外部呼叫） */
static HAL_StatusTypeDef e22_write_channel(uint8_t ch)
{
    /* 切設定模式（M1=1, M0=0），等 AUX HIGH（模式切換完成） */
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    if (!e22_wait_aux_high(200)) {
        HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
        return HAL_TIMEOUT;
    }

    /* Config mode 固定 9600 baud */
    s_huart->Init.BaudRate = 9600;
    HAL_UART_Init(s_huart);

    /* 讀回 CH 暫存器，若已一致則跳過寫入 */
    uint8_t rd[3] = {0xC1, 0x05, 0x01};
    HAL_UART_Transmit(s_huart, rd, sizeof(rd), 50);
    uint8_t rd_resp[4] = {0};
    HAL_UART_Receive(s_huart, rd_resp, sizeof(rd_resp), 100);
    printf("[LORA433] CH read-back: %02X %02X %02X %02X (want C1 05 01 %02X)\r\n",
           rd_resp[0], rd_resp[1], rd_resp[2], rd_resp[3], ch);

    HAL_StatusTypeDef ret = HAL_OK;
    if (rd_resp[0] == 0xC1 && rd_resp[3] == ch) {
        printf("[LORA433] CH already %d (%u MHz), skip write\r\n",
               ch, (unsigned)(E22_FREQ_BASE_MHZ + ch));
    } else {
        uint8_t wr[4] = {0xC0, 0x05, 0x01, ch};
        HAL_UART_Transmit(s_huart, wr, sizeof(wr), 50);
        uint8_t wr_resp[4] = {0};
        HAL_UART_Receive(s_huart, wr_resp, sizeof(wr_resp), 100);
        printf("[LORA433] CH write resp: %02X %02X %02X %02X\r\n",
               wr_resp[0], wr_resp[1], wr_resp[2], wr_resp[3]);
        if (!e22_wait_aux_high(300)) ret = HAL_TIMEOUT;
    }

    /* 恢復透傳模式 baud */
    s_huart->Init.BaudRate = 115200;
    HAL_UART_Init(s_huart);

    /* 回透傳模式（M1=0） */
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    return ret;
}

HAL_StatusTypeDef LoRaE22_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    /* 透傳模式 M1=0, M0=0 */
    HAL_GPIO_WritePin(LORA433_M0_GPIO_Port, LORA433_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);

    /* 硬體重置脈衝：RST 拉低 ~10ms 再釋放（RST 低電平有效） */
    HAL_GPIO_WritePin(LORA433_RST_GPIO_Port, LORA433_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA433_RST_GPIO_Port, LORA433_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(20);

    s_inited = 1;
    return HAL_OK;
}

uint8_t LoRaE22_IsReady(void)
{
    return s_inited;
}

HAL_StatusTypeDef LoRaE22_Send(const uint8_t *data, uint16_t len)
{
    if (!s_inited || s_huart == NULL || data == NULL || len == 0) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit(s_huart, (uint8_t *)data, len, LORA_E22_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef LoRaE22_SetFreqMHz(uint32_t freq_mhz)
{
    if (!s_inited || s_huart == NULL) return HAL_ERROR;
    if (freq_mhz < E22_FREQ_BASE_MHZ || freq_mhz > (E22_FREQ_BASE_MHZ + 83U)) {
        return HAL_ERROR;
    }
    uint8_t ch = (uint8_t)(freq_mhz - E22_FREQ_BASE_MHZ);
    return e22_write_channel(ch);
}

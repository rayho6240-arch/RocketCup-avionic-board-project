/**
  ******************************************************************************
  * @file           : adxl375.c
  * @brief          : ADXL375 High-G Accelerometer Driver Implementation
  ******************************************************************************
  */
#include "adxl375.h"

/* --- SPI Low-Level Helper Functions (使用 TransmitReceive 確保時鐘與清除標誌) --- */

static HAL_StatusTypeDef ADXL375_Reg_Write(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    tx_data[0] = reg & 0x7F; // Write mode: MSB = 0
    tx_data[1] = val;

    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_SET);

    return status;
}

static HAL_StatusTypeDef ADXL375_Reg_Read(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};

    tx_data[0] = reg | 0x80; // Read mode: MSB = 1

    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, HAL_MAX_DELAY); // 1 addr + 1 data = 2 bytes
    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_SET);

    if (status == HAL_OK) {
        *val = rx_data[1]; // Data is returned in the 2nd byte
    }
    return status;
}

/* --- Public Driver Functions --- */

HAL_StatusTypeDef ADXL375_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t chip_id = 0;
    HAL_StatusTypeDef status;

    // 確保 Chip Select 引腳處於高電平空閒狀態
    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    // 讀取 Chip ID (應為 0xE5)
    status = ADXL375_Reg_Read(hspi, ADXL375_DEVID_REG, &chip_id);
    if (status != HAL_OK || chip_id != ADXL375_DEVID_VAL) {
        return HAL_ERROR;
    }

    // 配置數據格式：全解析度 (Full Resolution)、±200g 量程、4線 SPI
    status = ADXL375_Reg_Write(hspi, ADXL375_DATA_FORMAT_REG, 0x0B);
    HAL_Delay(10);

    // 設定輸出頻率：3200Hz Output Data Rate (官方最高 ODR)
    status = ADXL375_Reg_Write(hspi, ADXL375_BW_RATE_REG, 0x0F); // 0x0F = 3200Hz
    HAL_Delay(10);

    // 啟動測量模式：將 POWER_CTL 暫存器設為 Measure (0x08)
    status = ADXL375_Reg_Write(hspi, ADXL375_POWER_CTL_REG, 0x08);
    HAL_Delay(20);

    return HAL_OK;
}

HAL_StatusTypeDef ADXL375_ReadData(SPI_HandleTypeDef *hspi, ADXL375_Data_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[7] = {0};
    uint8_t rx_data[7] = {0}; // 1 addr + 6 bytes data

    // 讀取 6 位元組連續數據 (DATAX0 至 DATAZ1)
    // Read Mode (Bit 7 = 1) + Multi-Byte Mode (Bit 6 = 1)
    tx_data[0] = ADXL375_DATAX0_REG | 0x80 | 0x40; // 0xF2

    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 7, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_SET);

    if (status != HAL_OK) return status;

    // 解析原始 16-bit 補碼數據 (LSB + MSB，跳過 rx_data[0] 的 address)
    data->x_raw = (int16_t)((rx_data[2] << 8) | rx_data[1]);
    data->y_raw = (int16_t)((rx_data[4] << 8) | rx_data[3]);
    data->z_raw = (int16_t)((rx_data[6] << 8) | rx_data[4]); // Note: rx_data[5] was missed in previous write, let's fix it!
    
    // Correct indices:
    // rx_data[0] = dummy/addr echo
    // rx_data[1] = DATAX0, rx_data[2] = DATAX1
    // rx_data[3] = DATAY0, rx_data[4] = DATAY1
    // rx_data[5] = DATAZ0, rx_data[6] = DATAZ1
    data->x_raw = (int16_t)((rx_data[2] << 8) | rx_data[1]);
    data->y_raw = (int16_t)((rx_data[4] << 8) | rx_data[3]);
    data->z_raw = (int16_t)((rx_data[6] << 8) | rx_data[5]);

    // 根據說明書，ADXL375 的解析度固定為 49mg/LSB = 0.049g/LSB
    // 備註：因 PCB 上標示相反，實際使用解算時 X 與 Y 應取負值以做軸向修正
    data->ax = (float)data->x_raw * 0.049f;
    data->ay = (float)data->y_raw * 0.049f;
    data->az = (float)data->z_raw * 0.049f;

    return HAL_OK;
}

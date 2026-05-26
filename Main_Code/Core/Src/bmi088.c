/**
  ******************************************************************************
  * @file           : bmi088.c
  * @brief          : BMI088 IMU Driver Implementation
  ******************************************************************************
  */
#include "bmi088.h"

/* --- SPI Low-Level Helper Functions --- */

static HAL_StatusTypeDef Accel_Reg_Write(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[2];
    tx_data[0] = reg & 0x7F; // Write mode: MSB = 0
    tx_data[1] = val;

    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi, tx_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_SET);

    return status;
}

static HAL_StatusTypeDef Accel_Reg_Read(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[3];
    uint8_t rx_data[3];

    tx_data[0] = reg | 0x80; // Read mode: MSB = 1
    tx_data[1] = 0x00;       // Dummy byte required for Accel SPI read
    tx_data[2] = 0x00;

    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_SET);

    if (status == HAL_OK) {
        *val = rx_data[2]; // Data is returned in the 3rd byte
    }
    return status;
}

static HAL_StatusTypeDef Gyro_Reg_Write(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[2];
    tx_data[0] = reg & 0x7F; // Write mode: MSB = 0
    tx_data[1] = val;

    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi, tx_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

    return status;
}

static HAL_StatusTypeDef Gyro_Reg_Read(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = reg | 0x80; // Read mode: MSB = 1
    tx_data[1] = 0x00;       // No dummy byte required for Gyro SPI read

    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

    if (status == HAL_OK) {
        *val = rx_data[1]; // Data is returned in the 2nd byte
    }
    return status;
}

/* --- Public Driver Functions --- */

HAL_StatusTypeDef BMI088_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t chip_id = 0;
    HAL_StatusTypeDef status;

    // 確保 Chip Select 引腳處於高電平空閒狀態
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    /* ==================== 1. 初始化加速度計 ==================== */
    
    // 軟體重置加速度計
    status = Accel_Reg_Write(hspi, BMI088_ACC_SOFTRESET_REG, 0xB6);
    HAL_Delay(50);

    // 讀取加速度計 Chip ID (應為 0x1E)
    status = Accel_Reg_Read(hspi, BMI088_ACC_CHIP_ID_REG, &chip_id);
    if (status != HAL_OK || chip_id != BMI088_ACC_CHIP_ID_VAL) {
        return HAL_ERROR;
    }

    // 啟動加速度計模式 (把 Accel 從 Suspend 喚醒)
    status = Accel_Reg_Write(hspi, BMI088_ACC_PWR_CONF_REG, 0x00); // 進入 Active 模式
    HAL_Delay(10);
    status = Accel_Reg_Write(hspi, BMI088_ACC_PWR_CTRL_REG, 0x04); // 開啟加速度計感測
    HAL_Delay(10);

    // 配置加速度計參數：量程設為 ±24g (火箭專用最高量程)
    status = Accel_Reg_Write(hspi, BMI088_ACC_RANGE_REG, 0x03);    // 0x03 = ±24g
    HAL_Delay(10);
    
    // 設定輸出頻率：1600Hz, OSR4 濾波器
    status = Accel_Reg_Write(hspi, BMI088_ACC_CONF_REG, 0xAC);
    HAL_Delay(10);

    /* ==================== 2. 初始化陀螺儀 ==================== */
    
    // 軟體重置陀螺儀
    status = Gyro_Reg_Write(hspi, BMI088_GYRO_SOFTRESET_REG, 0xB6);
    HAL_Delay(50);

    // 讀取陀螺儀 Chip ID (應為 0x0F)
    status = Gyro_Reg_Read(hspi, BMI088_GYRO_CHIP_ID_REG, &chip_id);
    if (status != HAL_OK || chip_id != BMI088_GYRO_CHIP_ID_VAL) {
        return HAL_ERROR;
    }

    // 配置陀螺儀量程：設為 ±2000 °/s (最大角速度)
    status = Gyro_Reg_Write(hspi, BMI088_GYRO_RANGE_REG, 0x00);    // 0x00 = ±2000 °/s
    HAL_Delay(10);

    // 設定陀螺儀輸出頻率與帶寬：1000Hz ODR, 帶寬 116Hz
    status = Gyro_Reg_Write(hspi, BMI088_GYRO_BANDWIDTH_REG, 0x02);
    HAL_Delay(10);

    return HAL_OK;
}

HAL_StatusTypeDef BMI088_ReadData(SPI_HandleTypeDef *hspi, BMI088_Data_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[10] = {0};
    uint8_t rx_data[10] = {0};

    /* ==================== 1. 讀取加速度計 6 位元組 ==================== */
    tx_data[0] = BMI088_ACC_DATA_REG | 0x80;
    tx_data[1] = 0x00; // Dummy byte

    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 8, HAL_MAX_DELAY); // 1 addr + 1 dummy + 6 data = 8 bytes
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_SET);

    if (status != HAL_OK) return status;

    // 解析加速度原始 16-bit 數據 (補碼格式)
    data->accel_x_raw = (int16_t)((rx_data[3] << 8) | rx_data[2]);
    data->accel_y_raw = (int16_t)((rx_data[5] << 8) | rx_data[4]);
    data->accel_z_raw = (int16_t)((rx_data[7] << 8) | rx_data[6]);

    // 換算成物理加速度 (單位: g)。±24g 下靈敏度為 1365 LSB/g (32768 / 24)
    data->ax = (float)data->accel_x_raw * 24.0f / 32768.0f;
    data->ay = (float)data->accel_y_raw * 24.0f / 32768.0f;
    data->az = (float)data->accel_z_raw * 24.0f / 32768.0f;

    /* ==================== 2. 讀取陀螺儀 6 位元組 ==================== */
    tx_data[0] = BMI088_GYRO_DATA_REG | 0x80;

    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 7, HAL_MAX_DELAY); // 1 addr + 6 data = 7 bytes
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

    if (status != HAL_OK) return status;

    // 解析陀螺儀原始 16-bit 數據
    data->gyro_x_raw = (int16_t)((rx_data[2] << 8) | rx_data[1]);
    data->gyro_y_raw = (int16_t)((rx_data[4] << 8) | rx_data[3]);
    data->gyro_z_raw = (int16_t)((rx_data[6] << 8) | rx_data[5]);

    // 換算成角速度 (單位: deg/s)。±2000°/s 下靈敏度為 16.384 LSB/(deg/s) (32768 / 2000)
    data->gx = (float)data->gyro_x_raw * 2000.0f / 32768.0f;
    data->gy = (float)data->gyro_y_raw * 2000.0f / 32768.0f;
    data->gz = (float)data->gyro_z_raw * 2000.0f / 32768.0f;

    return HAL_OK;
}

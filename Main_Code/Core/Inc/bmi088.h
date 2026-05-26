/**
  ******************************************************************************
  * @file           : bmi088.h
  * @brief          : Header for bmi088.c file.
  *                   This file contains the common defines of the BMI088 IMU.
  ******************************************************************************
  */
#ifndef __BMI088_H
#define __BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* --- BMI088 SPI Chip Select Port/Pin --- */
#define ACCEL_CS_PORT    GPIOB
#define ACCEL_CS_PIN     GPIO_PIN_12
#define GYRO_CS_PORT     GPIOB
#define GYRO_CS_PIN      GPIO_PIN_11

/* --- Accelerometer Registers --- */
#define BMI088_ACC_CHIP_ID_REG      0x00
#define BMI088_ACC_CHIP_ID_VAL      0x1E
#define BMI088_ACC_DATA_REG         0x12
#define BMI088_ACC_CONF_REG         0x40
#define BMI088_ACC_RANGE_REG        0x41
#define BMI088_ACC_PWR_CONF_REG     0x7C
#define BMI088_ACC_PWR_CTRL_REG     0x7D
#define BMI088_ACC_SOFTRESET_REG    0x7F

/* --- Gyroscope Registers --- */
#define BMI088_GYRO_CHIP_ID_REG     0x00
#define BMI088_GYRO_CHIP_ID_VAL     0x0F
#define BMI088_GYRO_DATA_REG        0x02
#define BMI088_GYRO_RANGE_REG       0x0F
#define BMI088_GYRO_BANDWIDTH_REG   0x10
#define BMI088_GYRO_SOFTRESET_REG   0x14

/* --- IMU Data Struct --- */
typedef struct {
    // 原始 16-bit 數據
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    // 物理量數據 (加速度 g, 角速度 deg/s)
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} BMI088_Data_t;

/* --- Function Prototypes --- */
HAL_StatusTypeDef BMI088_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef BMI088_ReadData(SPI_HandleTypeDef *hspi, BMI088_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_H */

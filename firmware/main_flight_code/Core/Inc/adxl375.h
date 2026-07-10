/**
  ******************************************************************************
  * @file           : adxl375.h
  * @brief          : Header for adxl375.c file.
  *                   This file contains the common defines of the ADXL375 High-G sensor.
  ******************************************************************************
  */
#ifndef __ADXL375_H
#define __ADXL375_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* --- ADXL375 SPI Chip Select Port/Pin --- */
#define HIGHG_CS_PORT    GPIOC
#define HIGHG_CS_PIN     GPIO_PIN_4

/* --- ADXL375 Registers --- */
#define ADXL375_DEVID_REG          0x00
#define ADXL375_DEVID_VAL          0xE5
#define ADXL375_BW_RATE_REG        0x2C
#define ADXL375_POWER_CTL_REG      0x2D
#define ADXL375_DATA_FORMAT_REG    0x31
#define ADXL375_DATAX0_REG         0x32

/* --- High-G Data Struct --- */
typedef struct {
    // 原始 16-bit 數據
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;

    // 物理量數據 (單位: g)
    float ax;
    float ay;
    float az;
} ADXL375_Data_t;

/* --- Function Prototypes --- */
HAL_StatusTypeDef ADXL375_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef ADXL375_ReadData(SPI_HandleTypeDef *hspi, ADXL375_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __ADXL375_H */

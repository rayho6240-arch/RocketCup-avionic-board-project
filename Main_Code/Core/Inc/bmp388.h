/**
  ******************************************************************************
  * @file           : bmp388.h
  * @brief          : Header for bmp388.c wrapper using official Bosch BMP3 API
  ******************************************************************************
  */
#ifndef __BMP388_H
#define __BMP388_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* --- BMP388 SPI Chip Select Port/Pin --- */
#define BARO_CS_PORT    GPIOA
#define BARO_CS_PIN     GPIO_PIN_4

/* --- BMP388 Data Struct --- */
typedef struct {
    // 物理量數據
    float pressure;    // 單位: Pa
    float temperature; // 單位: °C
    float altitude;    // 單位: m (以標準海平面氣壓換算)

    // 診斷暫存器 (用於調試與即時狀態監控)
    uint8_t err_reg;
    uint8_t status_reg;
    uint8_t pwr_ctrl_reg;
    uint8_t osr_reg;
    uint8_t odr_reg;
    uint8_t config_reg;
} BMP388_Data_t;

/* --- Function Prototypes --- */
HAL_StatusTypeDef BMP388_Init(SPI_HandleTypeDef *hspi, BMP388_Data_t *data);
HAL_StatusTypeDef BMP388_ReadData(SPI_HandleTypeDef *hspi, BMP388_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __BMP388_H */

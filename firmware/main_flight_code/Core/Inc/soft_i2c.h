/**
  ******************************************************************************
  * @file           : soft_i2c.h
  * @brief          : Software Bit-Bang I2C Driver for STM32F4
  ******************************************************************************
  */
#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* --- Software I2C GPIO Pin Definitions ---
 * On this avionics board (original crossed wiring):
 *   SCL = PB7
 *   SDA = PB8
 */
#define SOFT_I2C_SCL_PIN      GPIO_PIN_7
#define SOFT_I2C_SCL_PORT     GPIOB
#define SOFT_I2C_SDA_PIN      GPIO_PIN_8
#define SOFT_I2C_SDA_PORT     GPIOB

/* --- API --- */

/* Initialize Software I2C Pins as Output Open-Drain with Pull-ups */
void SoftI2C_Init(void);

/* Check if device with 7-bit dev_addr is ready/present on the bus */
HAL_StatusTypeDef SoftI2C_IsDeviceReady(uint16_t dev_addr, uint32_t trials, uint32_t timeout);

/* Drop-in replacement for HAL_I2C_Mem_Write */
HAL_StatusTypeDef SoftI2C_Mem_Write(uint16_t dev_addr, uint16_t mem_addr, uint8_t mem_addr_size, uint8_t *p_data, uint16_t size);

/* Drop-in replacement for HAL_I2C_Mem_Read */
HAL_StatusTypeDef SoftI2C_Mem_Read(uint16_t dev_addr, uint16_t mem_addr, uint8_t mem_addr_size, uint8_t *p_data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __SOFT_I2C_H */

/**
  ******************************************************************************
  * @file           : soft_i2c.c
  * @brief          : Software Bit-Bang I2C Driver for STM32F4 - Implementation
  ******************************************************************************
  */
#include "soft_i2c.h"

/* --- Low-level control functions --- */

static void SoftI2C_Delay(void)
{
    // 168MHz CPU clock. A volatile loop of ~40 iterations provides a delay of around 1.5-2.5 microseconds.
    // This establishes a software I2C clock frequency of ~100kHz - 200kHz.
    volatile uint32_t count = 45;
    while (count--);
}

static void SoftI2C_SCL_High(void)
{
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET);
}

static void SoftI2C_SCL_Low(void)
{
    HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET);
}

static void SoftI2C_SDA_High(void)
{
    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET);
}

static void SoftI2C_SDA_Low(void)
{
    HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET);
}

static uint8_t SoftI2C_SDA_Read(void)
{
    return (HAL_GPIO_ReadPin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

/* --- I2C Protocol Signaling --- */

static void SoftI2C_Start(void)
{
    SoftI2C_SDA_High();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    SoftI2C_SDA_Low();
    SoftI2C_Delay();
    SoftI2C_SCL_Low();
    SoftI2C_Delay();
}

static void SoftI2C_Stop(void)
{
    SoftI2C_SDA_Low();
    SoftI2C_Delay();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    SoftI2C_SDA_High();
    SoftI2C_Delay();
}

static uint8_t SoftI2C_WriteByte(uint8_t byte)
{
    // Write 8 bits
    for (int i = 0; i < 8; i++) {
        if (byte & 0x80) {
            SoftI2C_SDA_High();
        } else {
            SoftI2C_SDA_Low();
        }
        byte <<= 1;
        SoftI2C_Delay();
        SoftI2C_SCL_High();
        SoftI2C_Delay();
        SoftI2C_SCL_Low();
        SoftI2C_Delay();
    }
    
    // Read ACK/NACK
    SoftI2C_SDA_High(); // Release SDA line
    SoftI2C_Delay();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    uint8_t ack = SoftI2C_SDA_Read();
    SoftI2C_SCL_Low();
    SoftI2C_Delay();
    
    return ack; // 0 = ACK, 1 = NACK
}

static uint8_t SoftI2C_ReadByte(uint8_t ack)
{
    uint8_t byte = 0;
    SoftI2C_SDA_High(); // Release SDA line to allow slave to write
    SoftI2C_Delay();
    
    for (int i = 0; i < 8; i++) {
        SoftI2C_SCL_High();
        SoftI2C_Delay();
        byte <<= 1;
        if (SoftI2C_SDA_Read()) {
            byte |= 0x01;
        }
        SoftI2C_SCL_Low();
        SoftI2C_Delay();
    }
    
    // Send ACK or NACK
    if (ack) {
        SoftI2C_SDA_Low();  // 0 = ACK
    } else {
        SoftI2C_SDA_High(); // 1 = NACK
    }
    SoftI2C_Delay();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
    SoftI2C_SCL_Low();
    SoftI2C_Delay();
    SoftI2C_SDA_High(); // Release SDA
    
    return byte;
}

/* --- High-level Public API --- */

void SoftI2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIOB peripheral clock
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // Configure PB7 (SCL) and PB8 (SDA) as Output Open-Drain with internal Pull-up enabled.
    // Note: Open-Drain mode allows bidirectional communication and reading the line state
    // without reconfiguring the GPIO mode dynamically.
    GPIO_InitStruct.Pin = SOFT_I2C_SCL_PIN | SOFT_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Set SCL and SDA high to release the bus
    SoftI2C_SDA_High();
    SoftI2C_SCL_High();
    SoftI2C_Delay();
}

HAL_StatusTypeDef SoftI2C_IsDeviceReady(uint16_t dev_addr, uint32_t trials, uint32_t timeout)
{
    (void)timeout;
    for (uint32_t i = 0; i < trials; i++) {
        SoftI2C_Start();
        if (SoftI2C_WriteByte(dev_addr & 0xFE) == 0) { // Send address with Write bit
            SoftI2C_Stop();
            return HAL_OK;
        }
        SoftI2C_Stop();
        HAL_Delay(1);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef SoftI2C_Mem_Write(uint16_t dev_addr, uint16_t mem_addr, uint8_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    SoftI2C_Start();
    
    // Send device address in Write mode
    if (SoftI2C_WriteByte(dev_addr & 0xFE) != 0) {
        SoftI2C_Stop();
        return HAL_ERROR;
    }
    
    // Send memory/register address
    if (mem_addr_size == I2C_MEMADD_SIZE_16BIT) {
        if (SoftI2C_WriteByte((mem_addr >> 8) & 0xFF) != 0) {
            SoftI2C_Stop();
            return HAL_ERROR;
        }
    }
    if (SoftI2C_WriteByte(mem_addr & 0xFF) != 0) {
        SoftI2C_Stop();
        return HAL_ERROR;
    }
    
    // Send data bytes
    for (uint16_t i = 0; i < size; i++) {
        if (SoftI2C_WriteByte(p_data[i]) != 0) {
            SoftI2C_Stop();
            return HAL_ERROR;
        }
    }
    
    SoftI2C_Stop();
    return HAL_OK;
}

HAL_StatusTypeDef SoftI2C_Mem_Read(uint16_t dev_addr, uint16_t mem_addr, uint8_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    // 1. Send device write address & target register address
    SoftI2C_Start();
    
    if (SoftI2C_WriteByte(dev_addr & 0xFE) != 0) {
        SoftI2C_Stop();
        return HAL_ERROR;
    }
    
    if (mem_addr_size == I2C_MEMADD_SIZE_16BIT) {
        if (SoftI2C_WriteByte((mem_addr >> 8) & 0xFF) != 0) {
            SoftI2C_Stop();
            return HAL_ERROR;
        }
    }
    if (SoftI2C_WriteByte(mem_addr & 0xFF) != 0) {
        SoftI2C_Stop();
        return HAL_ERROR;
    }
    
    // 2. Send Repeated Start & device address in Read mode
    SoftI2C_Start();
    if (SoftI2C_WriteByte(dev_addr | 0x01) != 0) {
        SoftI2C_Stop();
        return HAL_ERROR;
    }
    
    // 3. Read data bytes sequentially
    for (uint16_t i = 0; i < size; i++) {
        // Send ACK (1) for all bytes except the very last one, which gets NACK (0)
        uint8_t send_ack = (i < (size - 1)) ? 1 : 0;
        p_data[i] = SoftI2C_ReadByte(send_ack);
    }
    
    SoftI2C_Stop();
    return HAL_OK;
}

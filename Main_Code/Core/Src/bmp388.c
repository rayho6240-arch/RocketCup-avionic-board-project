/**
  ******************************************************************************
  * @file           : bmp388.c
  * @brief          : BMP388 Driver implementation using official Bosch BMP3 API
  ******************************************************************************
  */
#include "bmp388.h"
#include "bmp3.h"
#include <string.h>
#include <math.h>

/* Bosch API 最大單次 SPI 傳輸長度 (len + 1 dummy byte + 1 addr):
 * 校正係數讀取最多 21 bytes + overhead 。定義 64 保留充足餲量。 */
#define BMP388_MAX_SPI_BUF  64U

/* --- 官方 API 所需的 device 實體 --- */
static struct bmp3_dev bmp_dev;

/* --- SPI 低階通訊與延時回調函式實作 --- */

static BMP3_INTF_RET_TYPE BMP388_SPI_Read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)intf_ptr;

    // Bosch bmp3.c 內部已將 len 加上 dummy_byte(=1) 後才呼叫 callback。
    // 尤此，callback 收到的 len = 真實資料長度 + 1 (dummy)。
    // HAL 傳輸總長: 1 (addr) + len (dummy + 資料) = len + 1
    uint32_t transfer_len = len + 1U;
    if (transfer_len > BMP388_MAX_SPI_BUF) return -1; // 防止溢位

    uint8_t tx_buf[BMP388_MAX_SPI_BUF] = {0};
    uint8_t rx_buf[BMP388_MAX_SPI_BUF] = {0};

    tx_buf[0] = reg_addr | 0x80; // Read mode: MSB = 1 (已由 bmp3.c 設定)

    HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, transfer_len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_SET);

    if (status == HAL_OK) {
        // rx_buf[0]: addr 回波垃圾，跳過
        // rx_buf[1..len]: dummy byte + 資料 — 全部回傳給 API，API 自己跳 dummy
        memcpy(read_data, &rx_buf[1], len);
        return BMP3_INTF_RET_SUCCESS;
    }
    return -1; // 失敗
}

static BMP3_INTF_RET_TYPE BMP388_SPI_Write(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)intf_ptr;

    // SPI Write: 1 addr + len data
    uint32_t transfer_len = len + 1U;
    if (transfer_len > BMP388_MAX_SPI_BUF) return -1; // 防止溢位

    uint8_t tx_buf[BMP388_MAX_SPI_BUF] = {0};
    uint8_t rx_buf[BMP388_MAX_SPI_BUF] = {0};

    tx_buf[0] = reg_addr & 0x7F; // Write mode: MSB = 0
    memcpy(&tx_buf[1], write_data, len);

    HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, transfer_len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_SET);

    if (status == HAL_OK) {
        return BMP3_INTF_RET_SUCCESS;
    }
    return -1; // 失敗
}

static void BMP388_Delay_Us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    // HAL_Delay 是毫秒級別延遲，向上取整以防不足
    uint32_t ms = (period + 999) / 1000;
    HAL_Delay(ms);
}

/* --- 公開驅動對接介面 --- */

HAL_StatusTypeDef BMP388_Init(SPI_HandleTypeDef *hspi, BMP388_Data_t *data)
{
    int8_t rslt = BMP3_E_DEV_NOT_FOUND;
    
    // 確保 Chip Select 引腳處於高電平空閒狀態
    HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
    
    // 配置設備結構體
    bmp_dev.intf_ptr = hspi;
    bmp_dev.intf = BMP3_SPI_INTF;
    bmp_dev.read = BMP388_SPI_Read;
    bmp_dev.write = BMP388_SPI_Write;
    bmp_dev.delay_us = BMP388_Delay_Us;
    
    // 嘗試初始化晶片 (連續嘗試最多 5 次以防模式切換未穩)
    for (int retry = 0; retry < 5; retry++) {
        rslt = bmp3_init(&bmp_dev);
        if (rslt == BMP3_OK) {
            break;
        }
        HAL_Delay(10);
    }
    
    if (rslt != BMP3_OK) {
        return HAL_ERROR;
    }
    
    // 配置感測器參數結構
    struct bmp3_settings settings = { 0 };
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os   = BMP3_NO_OVERSAMPLING;    // 氣壓無過採樣 (1x)
    settings.odr_filter.temp_os    = BMP3_NO_OVERSAMPLING;    // 溫度無過採樣 (1x)
    settings.odr_filter.odr        = BMP3_ODR_200_HZ;         // 200Hz 最高 ODR (1x OS 轉換時間 4.94ms < 5ms 週期)
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    
    uint16_t settings_sel = BMP3_SEL_PRESS_EN | 
                            BMP3_SEL_TEMP_EN | 
                            BMP3_SEL_PRESS_OS | 
                            BMP3_SEL_TEMP_OS | 
                            BMP3_SEL_ODR | 
                            BMP3_SEL_IIR_FILTER;
    
    // 寫入配置到晶片
    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &bmp_dev);
    if (rslt != BMP3_OK) {
        return HAL_ERROR;
    }
    
    // 將晶片設為 Normal Mode
    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &bmp_dev);
    if (rslt != BMP3_OK) {
        return HAL_ERROR;
    }
    
    HAL_Delay(20);
    
    return HAL_OK;
}

HAL_StatusTypeDef BMP388_ReadData(SPI_HandleTypeDef *hspi, BMP388_Data_t *data)
{
    (void)hspi;
    int8_t rslt;
    struct bmp3_data sensor_data;
    
    // 調用官方 API 讀取並自動進行高精度補償計算
    rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &sensor_data, &bmp_dev);
    if (rslt != BMP3_OK) {
        return HAL_ERROR;
    }
    
    // 存入公開數據結構體
    data->pressure = (float)sensor_data.pressure;
    data->temperature = (float)sensor_data.temperature;
    
    // 換算為海拔高度 (以標準海平面氣壓 101325 Pa 為基準)
    data->altitude = 44330.0f * (1.0f - powf(data->pressure / 101325.0f, 0.190295f));
    
    // 診斷暫存器讀取在正常運行中省略，以避免不必要的 SPI 傳輸 overhead
    // 如需診斷，可手動呼叫 bmp3_get_regs(BMP3_REG_ERR, ...) 等
    
    return HAL_OK;
}

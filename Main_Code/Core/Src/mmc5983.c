/**
  ******************************************************************************
  * @file           : mmc5983.c
  * @brief          : MMC5983MA 三軸地磁計驅動 (I2C1, 18-bit) 實作
  ******************************************************************************
  */
#include "mmc5983.h"
#include <math.h>
#include <stddef.h>

/* --- 內部組態 --- */
#define MMC5983_I2C_TIMEOUT_MS      (50U)    /* 單次 I2C 傳輸逾時 */
#define MMC5983_READY_TRIALS        (3U)     /* IsDeviceReady 重試次數 */
#define MMC5983_MEAS_TIMEOUT_MS     (10U)    /* 等待量測完成上限（400Hz BW ~2ms） */
#define MMC5983_SETRESET_SETTLE_MS  (1U)     /* SET/RESET 線圈脈衝後沉降時間 */
#define MMC5983_RAW_BYTES           (7U)     /* XOUT0..XYZOUT2 共 7 bytes */

static I2C_HandleTypeDef *mmc_hi2c = NULL;
static MMC5983_Data_t     mmc_data;

/* ------------------------------------------------------------------ */
/* 低階暫存器存取                                                       */
/* ------------------------------------------------------------------ */
static HAL_StatusTypeDef mmc_write_reg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(mmc_hi2c, MMC5983_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1U,
                             MMC5983_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef mmc_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(mmc_hi2c, MMC5983_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len,
                            MMC5983_I2C_TIMEOUT_MS);
}

/* 觸發一次 TM_M 量測、輪詢 STATUS 完成旗標、讀回 18-bit X/Y/Z 原始計數。 */
static HAL_StatusTypeDef mmc_measure_raw(int32_t *x, int32_t *y, int32_t *z)
{
    uint8_t  status = 0U;
    uint8_t  raw[MMC5983_RAW_BYTES] = {0};
    uint32_t t0;

    if (mmc_write_reg(MMC5983_REG_CTRL0, MMC5983_CTRL0_TM_M) != HAL_OK) {
        return HAL_ERROR;
    }

    t0 = HAL_GetTick();
    do {
        if (mmc_read_regs(MMC5983_REG_STATUS, &status, 1U) != HAL_OK) {
            return HAL_ERROR;
        }
        if ((status & MMC5983_STATUS_MEAS_M_DONE) != 0U) {
            break;
        }
    } while ((HAL_GetTick() - t0) < MMC5983_MEAS_TIMEOUT_MS);

    if ((status & MMC5983_STATUS_MEAS_M_DONE) == 0U) {
        return HAL_TIMEOUT;
    }

    if (mmc_read_regs(MMC5983_REG_XOUT0, raw, MMC5983_RAW_BYTES) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 18-bit 組裝：raw[0]=hi8, raw[1]=mid8, raw[6] 最高 2 bits 為最低 2 bits */
    *x = (int32_t)(((uint32_t)raw[0] << 10) | ((uint32_t)raw[1] << 2) |
                   (((uint32_t)raw[6] >> 6) & 0x03U));
    *y = (int32_t)(((uint32_t)raw[2] << 10) | ((uint32_t)raw[3] << 2) |
                   (((uint32_t)raw[6] >> 4) & 0x03U));
    *z = (int32_t)(((uint32_t)raw[4] << 10) | ((uint32_t)raw[5] << 2) |
                   (((uint32_t)raw[6] >> 2) & 0x03U));
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* 公開 API                                                            */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef MMC5983_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0U;

    mmc_hi2c = hi2c;
    mmc_data.ok = 0U;
    mmc_data.reads_ok = 0U;
    mmc_data.reads_err = 0U;
    for (int i = 0; i < 3; i++) {
        mmc_data.raw[i] = 0;
        mmc_data.offset[i] = (int32_t)MMC5983_ZERO_COUNT;  /* 預設中點，待校準覆蓋 */
        mmc_data.gauss[i] = 0.0f;
    }
    mmc_data.heading_deg = 0.0f;

    if (mmc_hi2c == NULL) {
        return HAL_ERROR;
    }

    if (HAL_I2C_IsDeviceReady(mmc_hi2c, MMC5983_I2C_ADDR,
                              MMC5983_READY_TRIALS,
                              MMC5983_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    if (mmc_read_regs(MMC5983_REG_PRODUCT_ID, &id, 1U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (id != MMC5983_PRODUCT_ID) {
        return HAL_ERROR;
    }

    /* 軟體重置 → 等待開機 (datasheet 建議 ~10ms) */
    if (mmc_write_reg(MMC5983_REG_CTRL1, MMC5983_CTRL1_SW_RST) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(15);

    /* 設定頻寬 400Hz（量測 ~2ms，雜訊適中） */
    if (mmc_write_reg(MMC5983_REG_CTRL1, MMC5983_CTRL1_BW_400HZ) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 初次 SET/RESET 橋偏校準（同時把感測器留在 SET 極性） */
    if (MMC5983_Recalibrate() != HAL_OK) {
        return HAL_ERROR;
    }

    mmc_data.ok = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef MMC5983_Recalibrate(void)
{
    int32_t s[3] = {0};
    int32_t r[3] = {0};

    if (mmc_hi2c == NULL) {
        return HAL_ERROR;
    }

    /* 1) SET 脈衝 → +極性量測 */
    if (mmc_write_reg(MMC5983_REG_CTRL0, MMC5983_CTRL0_SET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(MMC5983_SETRESET_SETTLE_MS);
    if (mmc_measure_raw(&s[0], &s[1], &s[2]) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 2) RESET 脈衝 → −極性量測 */
    if (mmc_write_reg(MMC5983_REG_CTRL0, MMC5983_CTRL0_RESET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(MMC5983_SETRESET_SETTLE_MS);
    if (mmc_measure_raw(&r[0], &r[1], &r[2]) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 3) 橋偏 = (SET + RESET) / 2；磁場分量 = (SET − RESET) / 2 */
    for (int i = 0; i < 3; i++) {
        mmc_data.offset[i] = (s[i] + r[i]) / 2;
    }

    /* 4) 收尾再做一次 SET，讓感測器停在 +極性供後續快速讀取 */
    if (mmc_write_reg(MMC5983_REG_CTRL0, MMC5983_CTRL0_SET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(MMC5983_SETRESET_SETTLE_MS);

    return HAL_OK;
}

HAL_StatusTypeDef MMC5983_Read(void)
{
    int32_t x = 0, y = 0, z = 0;

    if (mmc_hi2c == NULL) {
        return HAL_ERROR;
    }

    if (mmc_measure_raw(&x, &y, &z) != HAL_OK) {
        mmc_data.reads_err++;
        return HAL_ERROR;
    }

    mmc_data.raw[0] = x;
    mmc_data.raw[1] = y;
    mmc_data.raw[2] = z;

    /* 扣除橋偏 → Gauss（+極性，故直接相減即為磁場分量） */
    mmc_data.gauss[0] = (float)(x - mmc_data.offset[0]) / MMC5983_LSB_PER_GAUSS;
    mmc_data.gauss[1] = (float)(y - mmc_data.offset[1]) / MMC5983_LSB_PER_GAUSS;
    mmc_data.gauss[2] = (float)(z - mmc_data.offset[2]) / MMC5983_LSB_PER_GAUSS;

    /* 粗略水平航向（僅供診斷；真正 yaw 修正由 EKF 傾斜補償完成） */
    float h = atan2f(mmc_data.gauss[1], mmc_data.gauss[0]) * (180.0f / 3.14159265f);
    if (h < 0.0f) {
        h += 360.0f;
    }
    mmc_data.heading_deg = h;

    mmc_data.reads_ok++;
    return HAL_OK;
}

const MMC5983_Data_t* MMC5983_GetData(void)
{
    return &mmc_data;
}

#include "mag.h"

#include "main.h"
#include <stddef.h>

#define MAG_I2C_READY_TRIALS            (3U)
#define MAG_I2C_TIMEOUT_MS              (100U)
#define MAG_PRODUCT_ID_SIZE             (1U)
#define MAG_RAW_DATA_SIZE               (7U)
#define MAG_MEASUREMENT_TIMEOUT_MS      (100U)
#define MAG_POLL_DELAY_MS               (1U)
#define MAG_XYZOUT2_X_LSB_SHIFT         (6U)
#define MAG_XYZOUT2_Y_LSB_SHIFT         (4U)
#define MAG_XYZOUT2_Z_LSB_SHIFT         (2U)
#define MAG_XYZOUT2_LSB_MASK            (0x03U)

static I2C_HandleTypeDef *mag_hi2c;

static bool MAG_ReadProductID(uint8_t *id);
static bool MAG_ReadRaw(int32_t *mx, int32_t *my, int32_t *mz);

bool MAG_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t product_id = 0U;

  if (hi2c == NULL)
  {
    return false;
  }

  mag_hi2c = hi2c;

  if (HAL_I2C_IsDeviceReady(mag_hi2c,
                            MMC5983MA_I2C_ADDR,
                            MAG_I2C_READY_TRIALS,
                            MAG_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  if (!MAG_ReadProductID(&product_id))
  {
    return false;
  }

  return (product_id == MMC5983MA_PRODUCT_ID_EXPECTED);
}

bool MAG_ReadData(MAG_RawData_t *data)
{
  if (data == NULL)
  {
    return false;
  }

  data->valid = false;
  data->timestamp_ms = HAL_GetTick();

  if (mag_hi2c == NULL)
  {
    return false;
  }

  if (!MAG_ReadRaw(&data->raw_x, &data->raw_y, &data->raw_z))
  {
    return false;
  }

  data->timestamp_ms = HAL_GetTick();
  data->valid = true;
  return true;
}

static bool MAG_ReadProductID(uint8_t *id)
{
  if ((mag_hi2c == NULL) || (id == NULL))
  {
    return false;
  }

  return (HAL_I2C_Mem_Read(mag_hi2c,
                           MMC5983MA_I2C_ADDR,
                           MMC5983MA_PRODUCT_ID_REG,
                           I2C_MEMADD_SIZE_8BIT,
                           id,
                           MAG_PRODUCT_ID_SIZE,
                           MAG_I2C_TIMEOUT_MS) == HAL_OK);
}

static bool MAG_ReadRaw(int32_t *mx, int32_t *my, int32_t *mz)
{
  uint8_t control_value = MMC5983MA_CONTROL_0_TM_M;
  uint8_t status = 0U;
  uint8_t raw[MAG_RAW_DATA_SIZE] = {0};
  uint32_t start_tick;

  if ((mag_hi2c == NULL) || (mx == NULL) || (my == NULL) || (mz == NULL))
  {
    return false;
  }

  if (HAL_I2C_Mem_Write(mag_hi2c,
                        MMC5983MA_I2C_ADDR,
                        MMC5983MA_CONTROL_0_REG,
                        I2C_MEMADD_SIZE_8BIT,
                        &control_value,
                        1U,
                        MAG_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  start_tick = HAL_GetTick();
  do
  {
    if (HAL_I2C_Mem_Read(mag_hi2c,
                         MMC5983MA_I2C_ADDR,
                         MMC5983MA_STATUS_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         &status,
                         1U,
                         MAG_I2C_TIMEOUT_MS) != HAL_OK)
    {
      return false;
    }

    if ((status & MMC5983MA_STATUS_MEAS_M_DONE) != 0U)
    {
      break;
    }

    HAL_Delay(MAG_POLL_DELAY_MS);
  } while ((HAL_GetTick() - start_tick) < MAG_MEASUREMENT_TIMEOUT_MS);

  if ((status & MMC5983MA_STATUS_MEAS_M_DONE) == 0U)
  {
    return false;
  }

  if (HAL_I2C_Mem_Read(mag_hi2c,
                       MMC5983MA_I2C_ADDR,
                       MMC5983MA_XOUT0_REG,
                       I2C_MEMADD_SIZE_8BIT,
                       raw,
                       MAG_RAW_DATA_SIZE,
                       MAG_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  *mx = (int32_t)(((uint32_t)raw[0] << 10U) |
                  ((uint32_t)raw[1] << 2U) |
                  (((uint32_t)raw[6] >> MAG_XYZOUT2_X_LSB_SHIFT) & MAG_XYZOUT2_LSB_MASK));
  *my = (int32_t)(((uint32_t)raw[2] << 10U) |
                  ((uint32_t)raw[3] << 2U) |
                  (((uint32_t)raw[6] >> MAG_XYZOUT2_Y_LSB_SHIFT) & MAG_XYZOUT2_LSB_MASK));
  *mz = (int32_t)(((uint32_t)raw[4] << 10U) |
                  ((uint32_t)raw[5] << 2U) |
                  (((uint32_t)raw[6] >> MAG_XYZOUT2_Z_LSB_SHIFT) & MAG_XYZOUT2_LSB_MASK));

  return true;
}

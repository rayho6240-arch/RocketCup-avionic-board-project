#ifndef MAG_H
#define MAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct __I2C_HandleTypeDef I2C_HandleTypeDef;

#define MMC5983MA_I2C_ADDR_7BIT         (0x30U)
#define MMC5983MA_I2C_ADDR              (MMC5983MA_I2C_ADDR_7BIT << 1U)
#define MMC5983MA_STATUS_REG            (0x08U)
#define MMC5983MA_CONTROL_0_REG         (0x09U)
#define MMC5983MA_XOUT0_REG             (0x00U)
#define MMC5983MA_PRODUCT_ID_REG        (0x2FU)
#define MMC5983MA_PRODUCT_ID_EXPECTED   (0x30U)
#define MMC5983MA_STATUS_MEAS_M_DONE    (0x01U)
#define MMC5983MA_CONTROL_0_TM_M        (0x01U)

typedef struct
{
  int32_t raw_x;
  int32_t raw_y;
  int32_t raw_z;
  uint32_t timestamp_ms;
  bool valid;
} MAG_RawData_t;

bool MAG_Init(I2C_HandleTypeDef *hi2c);
bool MAG_ReadData(MAG_RawData_t *data);

#ifdef __cplusplus
}
#endif

#endif /* MAG_H */

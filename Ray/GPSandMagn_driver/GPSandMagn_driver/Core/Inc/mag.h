#ifndef MAG_H
#define MAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MMC5983MA_I2C_ADDR_7BIT         (0x30U)
#define MMC5983MA_I2C_ADDR              (MMC5983MA_I2C_ADDR_7BIT << 1U)
#define MMC5983MA_STATUS_REG            (0x08U)
#define MMC5983MA_CONTROL_0_REG         (0x09U)
#define MMC5983MA_XOUT0_REG             (0x00U)
#define MMC5983MA_PRODUCT_ID_REG        (0x2FU)
#define MMC5983MA_PRODUCT_ID_EXPECTED   (0x30U)
#define MMC5983MA_STATUS_MEAS_M_DONE    (0x01U)
#define MMC5983MA_CONTROL_0_TM_M        (0x01U)

bool MAG_Init(void);
bool MAG_ReadProductID(uint8_t *id);
bool MAG_ReadRaw(int32_t *mx, int32_t *my, int32_t *mz);

#ifdef __cplusplus
}
#endif

#endif /* MAG_H */

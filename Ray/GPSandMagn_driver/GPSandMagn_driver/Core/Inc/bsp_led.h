#ifndef BSP_LED_H
#define BSP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define LED_BOOT_OK_BLINKS              (3U)
#define LED_I2C_DEVICE_FOUND_BLINKS     (2U)
#define LED_I2C_SCAN_FAILED_BLINKS      (5U)
#define LED_GPS_BYTE_RX_BLINKS          (1U)
#define LED_GPS_SENTENCE_RX_BLINKS      (3U)

#define LED_BLINK_FAST_DELAY_MS         (80U)
#define LED_BLINK_SHORT_DELAY_MS        (20U)
#define LED_HEARTBEAT_ON_MS             (100U)
#define LED_HEARTBEAT_PERIOD_MS         (1000U)
#define LED_DWT_CTRL_CYCCNTENA_Msk      (1UL)

void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
void LED_Blink(uint8_t times, uint16_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LED_H */

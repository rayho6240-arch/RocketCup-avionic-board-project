#include "bsp_i2c.h"

#include "bsp_led.h"
#include "i2c.h"

#define I2C_SCAN_FIRST_7BIT_ADDR        (0x08U)
#define I2C_SCAN_LAST_7BIT_ADDR         (0x77U)
#define I2C_SCAN_TRIALS                 (2U)
#define I2C_SCAN_TIMEOUT_MS             (10U)

bool I2C_ScanDevices(void)
{
  bool found_device = false;

  for (uint8_t address = I2C_SCAN_FIRST_7BIT_ADDR;
       address <= I2C_SCAN_LAST_7BIT_ADDR;
       address++)
  {
    uint16_t hal_address = (uint16_t)(address << 1U);

    if (HAL_I2C_IsDeviceReady(&hi2c1,
                              hal_address,
                              I2C_SCAN_TRIALS,
                              I2C_SCAN_TIMEOUT_MS) == HAL_OK)
    {
      found_device = true;
      break;
    }
  }

  if (found_device)
  {
    LED_Blink(LED_I2C_DEVICE_FOUND_BLINKS, LED_BLINK_FAST_DELAY_MS);
  }
  else
  {
    LED_Blink(LED_I2C_SCAN_FAILED_BLINKS, LED_BLINK_FAST_DELAY_MS);
  }

  return found_device;
}

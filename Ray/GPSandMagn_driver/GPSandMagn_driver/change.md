# Change Log

## Latest Patch: MMC5983MA Raw XYZ Read

Goal: go beyond Product ID detection and verify that MMC5983MA can perform a magnetic measurement and return raw XYZ data.

No existing DMA, GPS, I2C, or USART structure was deleted.

## Hardware Assumptions

- MCU: STM32F103C8T6
- Magnetometer: MMC5983MA
- I2C1 SCL: PB6
- I2C1 SDA: PB7
- I2C speed: 100 kHz
- LED_debug: PB11
- Debug output: LED only

## Files Modified

- `Core/Inc/mag.h`
- `Core/Src/mag.c`
- `Core/Src/main.c`
- `change.md`

## MAG API

```c
bool MAG_Init(void);
bool MAG_ReadProductID(uint8_t *id);
bool MAG_ReadRaw(int32_t *mx, int32_t *my, int32_t *mz);
```

## MMC5983MA Registers

```c
#define MMC5983MA_I2C_ADDR_7BIT         (0x30U)
#define MMC5983MA_I2C_ADDR              (MMC5983MA_I2C_ADDR_7BIT << 1U)
#define MMC5983MA_STATUS_REG            (0x08U)
#define MMC5983MA_CONTROL_0_REG         (0x09U)
#define MMC5983MA_XOUT0_REG             (0x00U)
#define MMC5983MA_PRODUCT_ID_REG        (0x2FU)
#define MMC5983MA_PRODUCT_ID_EXPECTED   (0x30U)
#define MMC5983MA_STATUS_MEAS_M_DONE    (0x01U)
#define MMC5983MA_CONTROL_0_TM_M        (0x01U)
```

## MAG_ReadRaw Flow

`MAG_ReadRaw()` does:

1. Writes `TM_M = 1` to `CONTROL_0` register `0x09`.
2. Polls `STATUS` register `0x08`.
3. Waits until `MEAS_M_DONE` bit0 becomes `1`.
4. Times out after 100 ms.
5. Reads 7 bytes starting at `XOUT0` register `0x00`.
6. Combines X/Y/Z as unsigned 18-bit raw values:
   - X: `XOUT0`, `XOUT1`, `XYZOUT2[7:6]`
   - Y: `YOUT0`, `YOUT1`, `XYZOUT2[5:4]`
   - Z: `ZOUT0`, `ZOUT1`, `XYZOUT2[3:2]`

## Main Behavior

Initialization order remains:

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_DMA_Init();
MX_I2C1_Init();
MX_USART1_UART_Init();
```

Then `MAG_Init()` runs once.

In `while (1)`, every 500 ms:

- If `MAG_ReadRaw(&mag_raw_x, &mag_raw_y, &mag_raw_z)` succeeds:
  - LED is normally ON.
  - LED turns OFF for 50 ms.
  - LED turns ON again.

- If `MAG_ReadRaw()` fails:
  - LED fast blinks 3 times.

## LED Polarity

`main.c` supports active-high and active-low LED wiring:

```c
#define LED_ACTIVE_HIGH                 (1U)
#define LED_ACTIVE_LOW                  (0U)
#define LED_ACTIVE_POLARITY             LED_ACTIVE_HIGH
```

All LED operations go through:

```c
LED_On();
LED_Off();
LED_Blink();
```

All LED GPIO access uses CubeMX symbols:

- `LED_debug_GPIO_Port`
- `LED_debug_Pin`

## Target Interpretation

- LED normally ON with one short OFF pulse every 500 ms:
  MMC5983MA measurement trigger, status polling, and raw XYZ read are working.

- LED fast blinks 3 times:
  Measurement trigger, measurement-done polling, or raw data read failed.

If failure repeats, check:

- PB6 is connected to SCL
- PB7 is connected to SDA
- 3.3 V and GND are connected
- I2C pull-ups are present
- MMC5983MA address is really `0x30`
- Status register bit0 becomes `MEAS_M_DONE`

## Constraints Kept

- No `printf`
- No `malloc`
- No I2C DMA
- No interrupt LED blinking
- GPS files remain in the project
- USART files remain in the project
- DMA infrastructure remains in the project

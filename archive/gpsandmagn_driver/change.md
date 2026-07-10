# Change Log

## Latest Patch: Portable GPS / MAG Driver API

Goal: keep the project structure, but make GPS and MMC5983MA drivers easier to move from STM32F103C8T6 to STM32F407 later.

No DMA, GPS, I2C, or USART infrastructure was deleted.

## Driver API

GPS public API:

```c
bool GPS_Init(UART_HandleTypeDef *huart);
bool GPS_ReadData(GPS_RawData_t *data);
```

MAG public API:

```c
bool MAG_Init(I2C_HandleTypeDef *hi2c);
bool MAG_ReadData(MAG_RawData_t *data);
```

Drivers no longer hardcode:

- `huart1`
- `hi2c1`
- GPIO pins

For an STM32F407 port, pass the handles from `main.c`:

```c
GPS_Init(&huartX);
MAG_Init(&hi2cX);
```

## Data Types

GPS raw data:

```c
typedef struct
{
  char sentence[GPS_NMEA_BUFFER_SIZE];
  uint16_t length;
  uint32_t timestamp_ms;
  bool valid;
} GPS_RawData_t;
```

MAG raw data:

```c
typedef struct
{
  int32_t raw_x;
  int32_t raw_y;
  int32_t raw_z;
  uint32_t timestamp_ms;
  bool valid;
} MAG_RawData_t;
```

## GPS Driver

`GPS_Init()`:

- Stores the passed UART handle in a static `gps_huart`.
- Starts `HAL_UART_Receive_IT(gps_huart, &gps_rx_byte, 1U)`.
- Returns `true` only if receive interrupt starts successfully.

`HAL_UART_RxCpltCallback()`:

- Checks `huart == gps_huart`.
- Stores received bytes into an internal NMEA buffer.
- Treats `\n` or `\r` as raw sentence end.
- Avoids counting `\r\n` as two valid sentences by ignoring empty lines.
- Does not parse latitude or longitude.
- Does not call `HAL_Delay()`.
- Restarts `HAL_UART_Receive_IT()`.

`GPS_ReadData()`:

- Non-blocking.
- Returns `true` only when a complete raw sentence is available.
- Copies the internal sentence into `GPS_RawData_t`.
- Sets `length`, `timestamp_ms`, and `valid`.

## MAG Driver

`MAG_Init()`:

- Stores the passed I2C handle in static `mag_hi2c`.
- Checks MMC5983MA device ready.
- Reads Product ID register `0x2F`.
- Returns `true` only if Product ID is `0x30`.

`MAG_ReadData()`:

- Triggers one magnetic measurement.
- Polls measurement done with timeout.
- Reads 7 raw bytes from `XOUT0`.
- Combines unsigned 18-bit X/Y/Z raw values.
- Sets `raw_x`, `raw_y`, `raw_z`, `timestamp_ms`, and `valid`.
- Does not perform calibration, unit conversion, or attitude calculation.

## LED Debug

CubeMX-generated LED pins:

- GPS debug: `GPS_debug_GPIO_Port`, `GPS_debug_Pin`
- Magnetometer debug: `Magn_debug_GPIO_Port`, `Magn_debug_Pin`

LED helpers in `main.c`:

```c
LED_On(port, pin);
LED_Off(port, pin);
LED_BlinkSlow(port, pin);
LED_BlinkFast3(port, pin);
```

LED behavior:

- GPS init success: GPS LED ON.
- GPS raw sentence received: GPS LED slow blinks once.
- GPS no raw sentence for 10 seconds: GPS LED fast blinks 3 times.
- MAG init success: MAG LED ON.
- MAG raw XYZ read success every 500 ms: MAG LED slow blinks once.
- MAG raw XYZ read failure: MAG LED fast blinks 3 times.

## Main Flow

Initialization remains:

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
GPS reset release;
MX_DMA_Init();
MX_I2C1_Init();
MX_USART1_UART_Init();

gps_ok = GPS_Init(&huart1);
mag_ok = MAG_Init(&hi2c1);
```

The GPS reset pin is still released in `main.c`, not in the GPS driver, so GPIO mapping stays outside the driver.

## Constraints Kept

- No `printf`
- No `malloc`
- No UART DMA receive
- No I2C DMA
- No LED blinking inside interrupt callbacks
- GPS driver does not parse NMEA fields
- MAG driver does not calibrate or convert units
- F103 pin mapping remains in CubeMX / `main.c`

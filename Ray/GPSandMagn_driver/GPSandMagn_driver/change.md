# Change Log

## Latest Patch: GPS RX 10-Second Timeout LED Logic

Finding and reason:

- With `RST_GPS` disconnected, GPS UART data is received.
- With `RST_GPS` connected, no UART bytes are received.
- After reset handling was fixed, LED still showed mixed slow/fast blink.
- The old 1-second no-byte check was too sensitive.

## Reset Pin

CubeMX generated:

```c
#define RST_GPS_Pin        GPIO_PIN_9
#define RST_GPS_GPIO_Port  GPIOB
```

The firmware uses these CubeMX names. It does not hardcode the reset GPIO in GPS reset control.

Assumption:

- NEO-M9N reset is active-low.
- LOW means hold reset.
- HIGH means release reset.

## GPS Reset API

Added to `gps.h` / `gps.c`:

```c
void GPS_ResetPin_InitState(void);
void GPS_ReleaseReset(void);
void GPS_HoldReset(void);
```

Behavior:

```c
GPS_HoldReset();     // RST_GPS = LOW
GPS_ReleaseReset();  // RST_GPS = HIGH
```

## GPIO Init Change

`MX_GPIO_Init()` now sets `RST_GPS` HIGH by default.

This prevents the GPS from being held in reset immediately after GPIO initialization.

Note:

- Current firmware assumes push-pull output.
- If the NEO-M9N reset line requires open-drain with an external pull-up, change only the `RST_GPS` pin mode in CubeMX/GPIO config.

## Main Init Order

Current startup sequence:

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
GPS_ReleaseReset();
HAL_Delay(500);
MX_DMA_Init();
MX_I2C1_Init();
MX_USART1_UART_Init();
GPS_Init();
```

The 500 ms delay gives the GPS time after reset release before USART RX interrupt listening starts.

## LED Debug Logic

The GPS byte counter LED debug now uses a 10-second no-data timeout.

Tracked state in `main.c`:

```c
last_gps_rx_byte_count
last_gps_rx_time_ms
last_gps_alive_blink_ms
```

Behavior:

- If `gps_rx_byte_count` increases:
  - `last_gps_rx_time_ms = HAL_GetTick()`
  - GPS is considered alive.
  - LED slow blinks once per second.

- If `HAL_GetTick() - last_gps_rx_time_ms > 10000`:
  - No UART byte was received for 10 continuous seconds.
  - LED fast blinks 3 times.

## Target Interpretation

- LED slow blinks once per second:
  GPS UART RX is receiving bytes often enough to stay alive.

- LED fast blinks 3 times:
  No UART byte was received for more than 10 seconds.

## Constraints Kept

- No `printf`
- No `malloc`
- No UART DMA receive
- No GPS parsing
- No full NMEA wait
- No `HAL_Delay()` inside interrupt callbacks
- DMA, I2C, GPS, and USART files remain in the project

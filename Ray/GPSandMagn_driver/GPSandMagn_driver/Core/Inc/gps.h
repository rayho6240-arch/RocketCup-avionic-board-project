#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

#define GPS_NMEA_BUFFER_SIZE            (128U)

typedef struct
{
  char sentence[GPS_NMEA_BUFFER_SIZE];
  uint16_t length;
  uint32_t timestamp_ms;
  bool valid;
} GPS_RawData_t;

bool GPS_Init(UART_HandleTypeDef *huart);
bool GPS_ReadData(GPS_RawData_t *data);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */

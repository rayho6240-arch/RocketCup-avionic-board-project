#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define GPS_NMEA_BUFFER_SIZE            (128U)

extern volatile uint32_t gps_rx_byte_count;
extern volatile uint32_t gps_rx_callback_count;
extern volatile bool gps_byte_received_flag;
extern volatile bool gps_uart_error_flag;

bool GPS_Init(void);
void GPS_ProcessByte(uint8_t byte);
bool GPS_HasReceivedByte(void);
void GPS_ClearReceivedByteFlag(void);
bool GPS_HasNewSentence(void);
void GPS_ClearNewSentenceFlag(void);
void GPS_ClearUartErrorFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */

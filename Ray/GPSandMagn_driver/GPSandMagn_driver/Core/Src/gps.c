#include "gps.h"

#include "usart.h"

#define GPS_NMEA_LINE_FEED              ('\n')
#define GPS_NMEA_CARRIAGE_RETURN        ('\r')
#define GPS_RX_BYTE_COUNT               (1U)

volatile uint32_t gps_rx_byte_count;
volatile uint32_t gps_rx_callback_count;
volatile bool gps_byte_received_flag;
volatile bool gps_uart_error_flag;

static uint8_t gps_rx_byte;
static uint8_t gps_nmea_buffer[GPS_NMEA_BUFFER_SIZE];
static uint16_t gps_nmea_index;
static volatile bool gps_new_sentence;
static bool gps_last_byte_was_line_end;

static bool GPS_IsLineEnd(uint8_t byte);

bool GPS_Init(void)
{
  gps_nmea_index = 0U;
  gps_rx_byte_count = 0U;
  gps_rx_callback_count = 0U;
  gps_byte_received_flag = false;
  gps_new_sentence = false;
  gps_uart_error_flag = false;
  gps_last_byte_was_line_end = false;

  return (HAL_UART_Receive_IT(&huart1, &gps_rx_byte, GPS_RX_BYTE_COUNT) == HAL_OK);
}

void GPS_ProcessByte(uint8_t byte)
{
  if (gps_nmea_index < (GPS_NMEA_BUFFER_SIZE - 1U))
  {
    gps_nmea_buffer[gps_nmea_index] = byte;
    gps_nmea_index++;
  }
  else
  {
    gps_nmea_index = 0U;
  }

  if (GPS_IsLineEnd(byte))
  {
    gps_nmea_buffer[gps_nmea_index] = '\0';
    gps_nmea_index = 0U;
  }
}

bool GPS_HasReceivedByte(void)
{
  return gps_byte_received_flag;
}

void GPS_ClearReceivedByteFlag(void)
{
  gps_byte_received_flag = false;
}

bool GPS_HasNewSentence(void)
{
  return gps_new_sentence;
}

void GPS_ClearNewSentenceFlag(void)
{
  gps_new_sentence = false;
}

void GPS_ClearUartErrorFlag(void)
{
  gps_uart_error_flag = false;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    gps_rx_callback_count++;
    gps_rx_byte_count++;
    gps_byte_received_flag = true;

    (void)HAL_UART_Receive_IT(&huart1, &gps_rx_byte, GPS_RX_BYTE_COUNT);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    gps_uart_error_flag = true;

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    (void)HAL_UART_Receive_IT(&huart1, &gps_rx_byte, GPS_RX_BYTE_COUNT);
  }
}

static bool GPS_IsLineEnd(uint8_t byte)
{
  return ((byte == (uint8_t)GPS_NMEA_LINE_FEED) ||
          (byte == (uint8_t)GPS_NMEA_CARRIAGE_RETURN));
}

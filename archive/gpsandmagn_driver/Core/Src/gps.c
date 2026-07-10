#include "gps.h"

#include "main.h"
#include <stddef.h>
#include <string.h>

#define GPS_RX_BYTE_COUNT               (1U)
#define GPS_LINE_FEED                   ('\n')
#define GPS_CARRIAGE_RETURN             ('\r')

static UART_HandleTypeDef *gps_huart;
static uint8_t gps_rx_byte;
static char gps_work_buffer[GPS_NMEA_BUFFER_SIZE];
static char gps_sentence_buffer[GPS_NMEA_BUFFER_SIZE];
static uint16_t gps_work_length;
static uint16_t gps_sentence_length;
static volatile bool gps_has_new_sentence;

static bool GPS_IsLineEnd(uint8_t byte);
static void GPS_ProcessRxByte(uint8_t byte);

bool GPS_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return false;
  }

  gps_huart = huart;
  gps_work_length = 0U;
  gps_sentence_length = 0U;
  gps_has_new_sentence = false;
  memset(gps_work_buffer, 0, sizeof(gps_work_buffer));
  memset(gps_sentence_buffer, 0, sizeof(gps_sentence_buffer));

  return (HAL_UART_Receive_IT(gps_huart, &gps_rx_byte, GPS_RX_BYTE_COUNT) == HAL_OK);
}

bool GPS_ReadData(GPS_RawData_t *data)
{
  if (data == NULL)
  {
    return false;
  }

  data->valid = false;
  data->length = 0U;
  data->timestamp_ms = HAL_GetTick();
  data->sentence[0] = '\0';

  if (!gps_has_new_sentence)
  {
    return false;
  }

  __disable_irq();
  uint16_t length = gps_sentence_length;
  if (length >= GPS_NMEA_BUFFER_SIZE)
  {
    length = GPS_NMEA_BUFFER_SIZE - 1U;
  }
  memcpy(data->sentence, gps_sentence_buffer, length);
  data->sentence[length] = '\0';
  gps_has_new_sentence = false;
  __enable_irq();

  data->length = length;
  data->timestamp_ms = HAL_GetTick();
  data->valid = true;
  return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((gps_huart != NULL) && (huart == gps_huart))
  {
    GPS_ProcessRxByte(gps_rx_byte);
    (void)HAL_UART_Receive_IT(gps_huart, &gps_rx_byte, GPS_RX_BYTE_COUNT);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((gps_huart != NULL) && (huart == gps_huart))
  {
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    gps_work_length = 0U;
    (void)HAL_UART_Receive_IT(gps_huart, &gps_rx_byte, GPS_RX_BYTE_COUNT);
  }
}

static void GPS_ProcessRxByte(uint8_t byte)
{
  if (GPS_IsLineEnd(byte))
  {
    if (gps_work_length > 0U)
    {
      gps_work_buffer[gps_work_length] = '\0';
      memcpy(gps_sentence_buffer, gps_work_buffer, gps_work_length + 1U);
      gps_sentence_length = gps_work_length;
      gps_has_new_sentence = true;
      gps_work_length = 0U;
    }
    return;
  }

  if (gps_work_length < (GPS_NMEA_BUFFER_SIZE - 1U))
  {
    gps_work_buffer[gps_work_length] = (char)byte;
    gps_work_length++;
  }
  else
  {
    gps_work_length = 0U;
  }
}

static bool GPS_IsLineEnd(uint8_t byte)
{
  return ((byte == (uint8_t)GPS_LINE_FEED) ||
          (byte == (uint8_t)GPS_CARRIAGE_RETURN));
}

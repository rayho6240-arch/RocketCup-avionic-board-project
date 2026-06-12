/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps.h"
#include "mag.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_SLOW_BLINK_MS               (80U)
#define LED_FAST_BLINK_MS               (80U)
#define LED_FAST_BLINK_TIMES            (3U)
#define GPS_NO_DATA_TIMEOUT_MS          (10000U)
#define MAG_READ_PERIOD_MS              (500U)
#define GPS_RESET_RELEASE_DELAY_MS      (500U)
#define LED_ACTIVE_HIGH                 (1U)
#define LED_ACTIVE_LOW                  (0U)
#define LED_ACTIVE_POLARITY             LED_ACTIVE_HIGH

#if (LED_ACTIVE_POLARITY == LED_ACTIVE_HIGH)
#define LED_ON_STATE                    GPIO_PIN_SET
#define LED_OFF_STATE                   GPIO_PIN_RESET
#else
#define LED_ON_STATE                    GPIO_PIN_RESET
#define LED_OFF_STATE                   GPIO_PIN_SET
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static bool gps_ok;
static bool mag_ok;
static uint32_t last_gps_sentence_ms;
static uint32_t last_mag_read_ms;
static GPS_RawData_t gps_raw;
static MAG_RawData_t mag_raw;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void LED_On(GPIO_TypeDef *port, uint16_t pin);
static void LED_Off(GPIO_TypeDef *port, uint16_t pin);
static void LED_BlinkSlow(GPIO_TypeDef *port, uint16_t pin);
static void LED_BlinkFast3(GPIO_TypeDef *port, uint16_t pin);
static void GPS_ResetReleasePin(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  GPS_ResetReleasePin();
  HAL_Delay(GPS_RESET_RELEASE_DELAY_MS);
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LED_Off(GPS_debug_GPIO_Port, GPS_debug_Pin);
  LED_Off(Magn_debug_GPIO_Port, Magn_debug_Pin);

  gps_ok = GPS_Init(&huart1);
  mag_ok = MAG_Init(&hi2c1);
  last_gps_sentence_ms = HAL_GetTick();
  last_mag_read_ms = HAL_GetTick();

  if (gps_ok)
  {
    LED_On(GPS_debug_GPIO_Port, GPS_debug_Pin);
  }
  else
  {
    LED_Off(GPS_debug_GPIO_Port, GPS_debug_Pin);
  }

  if (mag_ok)
  {
    LED_On(Magn_debug_GPIO_Port, Magn_debug_Pin);
  }
  else
  {
    LED_Off(Magn_debug_GPIO_Port, Magn_debug_Pin);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (gps_ok && GPS_ReadData(&gps_raw))
    {
      last_gps_sentence_ms = HAL_GetTick();
      LED_BlinkSlow(GPS_debug_GPIO_Port, GPS_debug_Pin);
    }
    else if (!gps_ok || ((HAL_GetTick() - last_gps_sentence_ms) > GPS_NO_DATA_TIMEOUT_MS))
    {
      LED_BlinkFast3(GPS_debug_GPIO_Port, GPS_debug_Pin);
      last_gps_sentence_ms = HAL_GetTick();
    }

    if ((HAL_GetTick() - last_mag_read_ms) >= MAG_READ_PERIOD_MS)
    {
      last_mag_read_ms = HAL_GetTick();
      if (mag_ok && MAG_ReadData(&mag_raw))
      {
        LED_BlinkSlow(Magn_debug_GPIO_Port, Magn_debug_Pin);
      }
      else
      {
        LED_BlinkFast3(Magn_debug_GPIO_Port, Magn_debug_Pin);
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void LED_On(GPIO_TypeDef *port, uint16_t pin)
{
  HAL_GPIO_WritePin(port, pin, LED_ON_STATE);
}

static void LED_Off(GPIO_TypeDef *port, uint16_t pin)
{
  HAL_GPIO_WritePin(port, pin, LED_OFF_STATE);
}

static void LED_BlinkSlow(GPIO_TypeDef *port, uint16_t pin)
{
  LED_Off(port, pin);
  HAL_Delay(LED_SLOW_BLINK_MS);
  LED_On(port, pin);
}

static void LED_BlinkFast3(GPIO_TypeDef *port, uint16_t pin)
{
  for (uint8_t index = 0U; index < LED_FAST_BLINK_TIMES; index++)
  {
    LED_Off(port, pin);
    HAL_Delay(LED_FAST_BLINK_MS);
    LED_On(port, pin);
    HAL_Delay(LED_FAST_BLINK_MS);
  }
}

static void GPS_ResetReleasePin(void)
{
  HAL_GPIO_WritePin(RST_GPS_GPIO_Port, RST_GPS_Pin, GPIO_PIN_SET);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

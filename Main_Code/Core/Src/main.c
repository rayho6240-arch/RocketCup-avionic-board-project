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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi088.h"
#include "adxl375.h"
#include "bmp388.h"
#include "w25qxx.h"
#include "w25q128.h"
#include "rate_monitor.h"  /* 採樣率監測模組，要停用請在 rate_monitor.h 註解 #define RATE_MONITOR_ENABLE */
#include "ekf.h"
#include "gps.h"
#include "mmc5983.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,  /* 2048 bytes: Bosch BMP3 API 深度呼叫 + VLA 需要足夠的堆疊空間 */
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
FlightState_t current_fsm_state = STATE_INIT;
uint32_t flight_start_tick = 0;
float last_vel_z = 0.0f;
uint8_t sd_logging_active = 0;
/* 落地時刻 tick（0 = 尚未落地）。SD 全程記錄據此由 100Hz 降為 10Hz，並於逾時後自動關檔 (Item E)。 */
volatile uint32_t g_touchdown_tick = 0;

BMI088_Data_t imu_data;
ADXL375_Data_t highg_data;
BMP388_Data_t baro_data;

uint8_t bmi088_ok = 0;
uint8_t adxl375_ok = 0;
uint8_t bmp388_ok = 0;
uint8_t mag_ok = 0;       /* MMC5983MA 地磁計（I2C1）初始化狀態 */

/* 採樣率監測器：在 IDE Watch / Live Expressions 加入《g_sampling_rate》即可一次觀察所有感測器 */
SAMPLING_RATE_DECL();   /* 展開為: SamplingRateAll_t g_sampling_rate */

/* --- EKF 1000 Hz Double Buffers in SRAM --- */
EKF_Buffer_t g_ekf_buffers[2];
volatile uint8_t g_ekf_active_idx = 0;
volatile uint8_t g_ekf_sample_count = 0;
/* EKF queue put 失敗計數：消費端 (EKF_Task) 落後造成 buffer 來不及入列時遞增。
 * 由 [RATE] 行輸出供觀測；若長期 >0 才需考慮擴大 buffer pool + queue 深度 (Item I)。 */
volatile uint32_t g_ekf_queue_drops = 0;

/* --- TIM3 (3.2 kHz) ADXL375 Ping-Pong 雙緩衝區宣告 --- */
#define ADXL_BATCH_SIZE 32
typedef struct {
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    float ax;
    float ay;
    float az;
} ADXL_Sample_t;

ADXL_Sample_t g_adxl_batches[2][ADXL_BATCH_SIZE];
volatile uint8_t g_adxl_active_batch = 0;   // 當前寫入的半區 (0 or 1)
volatile uint8_t g_adxl_sample_count = 0;   // 當前半區已累積的樣本數 (0..31)
volatile uint8_t g_adxl_new_batch_ready = 0;// 1 代表有一批 (32筆) 數據集滿
volatile uint8_t g_adxl_ready_batch_idx = 0;// 已集滿可讀取的半區索引

/* --- TIM6 (1.6 kHz) BMI088 Accel Ping-Pong 雙緩衝區宣告 --- */
#define BMI_ACC_BATCH_SIZE 16
typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    float ax, ay, az;
} BMI_Accel_Sample_t;

BMI_Accel_Sample_t g_bmi_acc_batches[2][BMI_ACC_BATCH_SIZE];
volatile uint8_t g_bmi_acc_active_batch = 0;   // 當前寫入的半區 (0 or 1)
volatile uint8_t g_bmi_acc_sample_count = 0;   // 當前半區已累積的樣本數 (0..15)
volatile uint8_t g_bmi_acc_new_batch_ready = 0;// 1 代表有一批 (16筆) 數據集滿
volatile uint8_t g_bmi_acc_ready_batch_idx = 0;// 已集滿可讀取的半區索引

/* --- TIM7 (2.0 kHz) BMI088 Gyro Ping-Pong 雙緩衝區宣告 --- */
#define BMI_GYRO_BATCH_SIZE 20
typedef struct {
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    float gx, gy, gz;
} BMI_Gyro_Sample_t;

BMI_Gyro_Sample_t g_bmi_gyro_batches[2][BMI_GYRO_BATCH_SIZE];
volatile uint8_t g_bmi_gyro_active_batch = 0;   // 當前寫入的半區 (0 or 1)
volatile uint8_t g_bmi_gyro_sample_count = 0;   // 當前半區已累積的樣本數 (0..19)
volatile uint8_t g_bmi_gyro_new_batch_ready = 0;// 1 代表有一批 (20筆) 數據集滿
volatile uint8_t g_bmi_gyro_ready_batch_idx = 0;// 已集滿可讀取的半區索引
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_CRC_Init(void);
static void MX_RNG_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  if (BMI088_Init(&hspi2) == HAL_OK) {
      bmi088_ok = 1;
      HAL_TIM_Base_Start_IT(&htim6);                           // 啟動 TIM6 1.6 kHz 中斷採樣 (Accel)
      HAL_TIM_Base_Start_IT(&htim7);                           // 啟動 TIM7 2.0 kHz 中斷採樣 (Gyro)
  }
  if (ADXL375_Init(&hspi1) == HAL_OK) {
      adxl375_ok = 1;
      HAL_TIM_Base_Start_IT(&htim3);                           // 啟動 TIM3 3.2 kHz 中斷採樣
  }
  if (BMP388_Init(&hspi1, &baro_data) == HAL_OK) {
      bmp388_ok = 1;
      /* 以發射台環境溫度鎖定高度換算參考溫度 T0 (Item H)。
       * 取 16 筆讀數平均濾除單筆雜訊；必須在 EKF 校準（RTOS 啟動後）之前完成，
       * 使 launchpad 參考與飛行讀數共用同一 T0，相對高度於發射台恆為 0 且全程連續。
       * BMP388 與 ADXL375 共用 SPI1，讀取時暫關 TIM3 中斷以避免匯流排競爭（同主迴圈做法）。 */
      float t0_sum = 0.0f;
      int   t0_n   = 0;
      for (int i = 0; i < 16; i++) {
          __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
          HAL_StatusTypeDef t0_res = BMP388_ReadData(&hspi1, &baro_data);
          __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
          if (t0_res == HAL_OK) {
              t0_sum += baro_data.temperature;
              t0_n++;
          }
          HAL_Delay(5);   /* BMP388 200Hz ODR → 5ms 取得新樣本 */
      }
      if (t0_n > 0) {
          float t0_avg = t0_sum / (float)t0_n;
          BMP388_SetReferenceTemp(t0_avg);
          printf("[BMP388] 高度換算參考溫度 T0 鎖定為 %d.%02d °C\r\n",
                 (int)t0_avg,
                 (int)(fabsf(t0_avg - (int)t0_avg) * 100.0f));
      }
  }

  /* GPS 驅動啟動 (USART6, NMEA-0183, 循環 DMA + IDLE 接收)。
   * NEO-M9N 實體掛載於 USART6（PC6/PC7）；解析於 task context（主迴圈 GPS_Update()）進行，
   * ISR/DMA 事件回呼僅組句設旗標。 */
  GPS_Init(&huart6);

  /* MMC5983MA 三軸地磁計啟動 (I2C1, SCL=PB7/SDA=PB8, 位址 0x30)。
   * 初始化含 Product ID 驗證、軟體重置、400Hz 頻寬與一次 SET/RESET 橋偏校準。
   * 失敗（晶片未上線）僅記錄旗標，不阻擋系統啟動。 */
  if (MMC5983_Init(&hi2c1) == HAL_OK) {
      mag_ok = 1;
      printf("[MAG] MMC5983MA online. offset[X,Y,Z]=%ld,%ld,%ld\r\n",
             (long)MMC5983_GetData()->offset[0],
             (long)MMC5983_GetData()->offset[1],
             (long)MMC5983_GetData()->offset[2]);
  } else {
      mag_ok = 0;
      printf("[MAG] MMC5983MA NOT detected (I2C1).\r\n");
  }

  /* W25Qxx SPI Flash 啟動自檢 (SPI3, CS=PA15) */
  Flash_Test();

  /* Buzzer：啟動 TIM2 CH1 PWM，初始靜音 (CCR1=0) */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xEKFQueue = osMessageQueueNew(2, sizeof(EKF_Buffer_t*), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadNew(EKF_Task, NULL, &EKF_Task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;                   // 16 分頻 -> 32kHz / 16 = 2000Hz (每秒計數 2000 下)
  hiwdg.Init.Reload = 4095;                                   // 4095 載入值 -> 4095 / 2000 ≈ 2.05 秒超時 (飛行中快速復位)
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 2;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5;                                   // 5 + 1 = 6 分頻 -> 84MHz / 6 = 14MHz
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4374;                                    // 4374 + 1 = 4375 週期 -> 14MHz / 4375 = 3200 Hz (3.2 kHz)
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{
  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 41;                                  // 41 + 1 = 42 分頻 -> 84MHz / 42 = 2.0MHz
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;                                    // 999 + 1 = 1000 週期 -> 2.0MHz / 1000 = 2000 Hz (2.0 kHz)
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 41;                                  // 41 + 1 = 42 分頻 -> 84MHz / 42 = 2.0MHz
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1249;                                   // 1249 + 1 = 1250 週期 -> 2.0MHz / 1250 = 1600 Hz (1.6 kHz)
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_SYS_Pin|GPIO_PIN_3|LED_STAT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSB_BARO_Pin|RST_GPS_Pin|FLASH_CSB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSB_HIGHG_GPIO_Port, CSB_HIGHG_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CSB_GYRO_Pin|CSB_ACCEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LORA433_M1_Pin|LORA433_M0_Pin|FIRE_Pin|LORA920_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LORA433_RST_Pin|CSB_LORA920_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_SYS_Pin PE3 LED_STAT2_Pin */
  GPIO_InitStruct.Pin = LED_SYS_Pin|GPIO_PIN_3|LED_STAT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BT2_Pin */
  GPIO_InitStruct.Pin = USER_BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BT1_Pin */
  GPIO_InitStruct.Pin = USER_BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSB_BARO_Pin */
  GPIO_InitStruct.Pin = CSB_BARO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSB_BARO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSB_HIGHG_Pin */
  GPIO_InitStruct.Pin = CSB_HIGHG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSB_HIGHG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_HIGHG2_Pin */
  GPIO_InitStruct.Pin = INT_HIGHG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(INT_HIGHG2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_HIGHG1_Pin INT_BARO_Pin */
  GPIO_InitStruct.Pin = INT_HIGHG1_Pin|INT_BARO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_ACCEL1_Pin INT_ACCEL2_Pin INT_GYRO1_Pin INT_GYRO2_Pin
                           INT_MAGN_Pin */
  GPIO_InitStruct.Pin = INT_ACCEL1_Pin|INT_ACCEL2_Pin|INT_GYRO1_Pin|INT_GYRO2_Pin
                          |INT_MAGN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA433_BUSY_Pin */
  GPIO_InitStruct.Pin = LORA433_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA433_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSB_GYRO_Pin */
  GPIO_InitStruct.Pin = CSB_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CSB_GYRO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSB_ACCEL_Pin */
  GPIO_InitStruct.Pin = CSB_ACCEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSB_ACCEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA433_M1_Pin LORA433_M0_Pin */
  GPIO_InitStruct.Pin = LORA433_M1_Pin|LORA433_M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA433_RST_Pin */
  GPIO_InitStruct.Pin = LORA433_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA433_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FIRE_Pin LORA920_RST_Pin CSB_LORA920_Pin */
  GPIO_InitStruct.Pin = FIRE_Pin|LORA920_RST_Pin|CSB_LORA920_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_GPS_Pin */
  GPIO_InitStruct.Pin = RST_GPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CSB_Pin */
  GPIO_InitStruct.Pin = FLASH_CSB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FLASH_CSB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_DET_Pin */
  GPIO_InitStruct.Pin = SDIO_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDIO_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA920_INT_Pin */
  GPIO_InitStruct.Pin = LORA920_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LORA920_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA920_BUSY_Pin */
  GPIO_InitStruct.Pin = LORA920_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA920_BUSY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

static void FSM_Update(void)
{
    EKF_State_t ekf_state = EKF_GetState();
    float h_est = ekf_state.pos_z;  // 卡爾曼估計高度 (m)
    float v_est = ekf_state.vel_z;  // 卡爾曼估計垂直速度 (m/s)
    float a_z = highg_data.az;      // 高 G 垂直加速度 (g)
    uint32_t now = HAL_GetTick();
    static uint32_t state_entered_tick = 0;
    static float max_altitude = 0.0f;
    static uint8_t consec_apogee_counts = 0;
    
    // 更新觀測到的最大高度
    if (h_est > max_altitude) {
        max_altitude = h_est;
    }
    
    switch (current_fsm_state) {
        case STATE_PAD:
            // 等待靜態校準完成
            if (EKF_calibrated) {
                // 起飛觸發條件：高G加速度 > 3.0g 或是高度 > 10.0m
                if (a_z > 3.0f || h_est > 10.0f) {
                    current_fsm_state = STATE_BOOST;
                    flight_start_tick = now;
                    state_entered_tick = now;
                    printf("[FSM] 🚀 LIFTOFF DETECTED! Transition to STATE_BOOST.\r\n");
                }
            }
            break;
            
        case STATE_BOOST:
            // 馬達燒完判定：加速度 < 0.5g 且 flight time > 1.5 秒
            if (a_z < 0.5f && (now - state_entered_tick) > 1500) {
                current_fsm_state = STATE_COAST;
                state_entered_tick = now;
                printf("[FSM] 🔥 MOTOR BURNOUT! Entering STATE_COAST.\r\n");
            }
            break;
            
        case STATE_COAST: {
            // 動態頂點預估 (預估 4.0s 前開副傘)
            float decel = -9.80665f;
            if (last_vel_z != 0.0f) {
                float a_z_nav = (v_est - last_vel_z) / 0.010f; // 10ms 速度差所得加速度
                if (a_z_nav < -5.0f && a_z_nav > -25.0f) {
                    decel = a_z_nav;
                }
            }
            
            float t_to_apogee = -v_est / decel;
            
            // 頂點判定條件：
            // 1. 動態預測時間 <= 4.0s (且仍處於上升狀態 v_est > 0)
            // 2. 備用安全判定：垂直速度過零 (v_est < -0.2 m/s) 或是高度從峰值下降超過 5.0m
            // 同時必須滿足起飛時間鎖（起飛後累計大於 3.0 秒）
            uint8_t apogee_condition = 0;
            if (v_est > 0.0f && t_to_apogee <= DROGUE_LEAD_TIME_S) {
                apogee_condition = 1;
            } else if (v_est < -0.2f || (max_altitude - h_est) > 5.0f) {
                apogee_condition = 1;
            }
            
            if (apogee_condition && (now - flight_start_tick) > 3000) {
                consec_apogee_counts++;
                if (consec_apogee_counts >= 5) { // 連續 5 個週期 (50ms) 成立以防雜訊
                    current_fsm_state = STATE_APOGEE;
                    state_entered_tick = now;
                    printf("[FSM] 🎪 DYNAMIC APOGEE DETECTED! Expected apogee in %.2f s (h_est: %.2f m). Deploying DROGUE.\r\n", 
                           t_to_apogee, h_est);
                    
                    // 1. 導通副傘引爆 MOSFET (PD13 = HIGH)
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
                }
            } else {
                consec_apogee_counts = 0;
            }
            break;
        }
            
        case STATE_APOGEE:
            // 點火限時導通保護：持續導通 2.0 秒後強制拉低
            if (now - state_entered_tick >= 2000) {
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // 斷開點火
                current_fsm_state = STATE_DESCENT;
                state_entered_tick = now;
                printf("[FSM] 🪂 Drogue deployed successfully. Entering STATE_DESCENT.\r\n");
            }
            break;
            
        case STATE_DESCENT: {
            // 動態主傘部署高度計算：h_trigger = h_target + |v_fall| * t_delay
            float v_fall = (v_est < 0.0f) ? -v_est : 0.0f;
            float h_trigger_main = TARGET_MAIN_ALTITUDE + v_fall * MAIN_DEPLOY_DELAY_S;
            
            // 觸發條件：高度低於觸發高度，或是飛行總時間看門狗超時 (25秒)
            if (h_est <= h_trigger_main || (now - flight_start_tick) > 25000) {
                current_fsm_state = STATE_MAIN_DEPLOY;
                state_entered_tick = now;
                printf("[FSM] 🪁 DYNAMIC LOW ALTITUDE REACHED! Trigger H: %.2f m (Target H: %.2f m, Fall V: %.2f m/s). Deploying MAIN.\r\n", 
                       h_est, TARGET_MAIN_ALTITUDE, v_est);
                
                // 2. 部署主傘：控制 PD14 釋放舵機 (設 PWM 脈寬為 2000，即釋放角度)
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 2000);
            }
            break;
        }
            
        case STATE_MAIN_DEPLOY:
            // 等待 3 秒讓主傘充氣張開，隨後進入落地偵測
            if (now - state_entered_tick >= 3000) {
                current_fsm_state = STATE_LANDED;
                state_entered_tick = now;
                printf("[FSM] Main deployed. Entering landing detection.\r\n");
            }
            break;
            
        case STATE_LANDED:
            // 落地判定：下墜速度趨近零，且高度小於 20m（僅觸發一次）
            if (g_touchdown_tick == 0 && fabsf(v_est) < 0.3f && h_est < 20.0f) {
                g_touchdown_tick = now;   // 記錄落地時刻：SD 記錄降為 10Hz，逾時或按 USER_BT1 後關檔
                printf("[FSM] 🏁 TOUCHDOWN! 持續記錄中（降頻 10Hz）。按 USER_BT1 或逾時後自動關檔。\r\n");

                // 開啟板載尋標蜂鳴器（持續鳴叫，利於落點尋標）
                htim2.Instance->ARR  = 999;
                htim2.Instance->CCR1 = 500;
                htim2.Instance->EGR  = TIM_EGR_UG;

                // 注意：不再立即關閉 SD、也不再 vTaskSuspend。改由主迴圈 SD 記錄區塊持續記錄，
                // 待停止條件達成時才安全關檔，使遙測 / Flash ring / 尋標蜂鳴器在落地後仍持續運作 (Item E)。
            }
            break;
            
        default:
            break;
    }
    
    last_vel_z = v_est; // 儲存速度歷史
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t tick = 0;

  /* === LED 開機自檢：全亮 2 秒確認三顆 LED 正常，之後全滅交由主迴圈接管 ===
   * 分 4 × 500ms 餵狗，避免 IWDG 2.05s timeout 在此期間觸發 reset。 */
  HAL_GPIO_WritePin(GPIOE, LED_SYS_Pin | GPIO_PIN_3 | LED_STAT2_Pin, GPIO_PIN_SET);
  for (int _li = 0; _li < 4; _li++) {
      osDelay(500);
      HAL_IWDG_Refresh(&hiwdg);
  }
  HAL_GPIO_WritePin(GPIOE, LED_SYS_Pin | GPIO_PIN_3 | LED_STAT2_Pin, GPIO_PIN_RESET);

  /* 開機按住 USER_BT1 才完整 dump W25Q128 三分區到 USART2（讀取上一次飛行數據、驗證 Flash 狀態）。
   * USER_BT1 為上拉輸入 → 按下讀到 LOW (GPIO_PIN_RESET)。
   * 平時開機跳過 dump，可省下整顆 Flash 輸出 (~3–5 秒) 的初始化時間 (Item N)。 */
  if (HAL_GPIO_ReadPin(USER_BT1_GPIO_Port, USER_BT1_Pin) == GPIO_PIN_RESET) {
      Flash_DumpAll(&hspi3);
  }

  /* 初始化採樣率監測器（Watch 視窗加入 g_sampling_rate 即可一次觀察全部 Hz） */
  SAMPLING_RATE_INIT();

  /* === Buzzer 開機提示：兩聲漸高 (2kHz → 4kHz) === */
  /* 聲1：2kHz，100ms */
  htim2.Instance->ARR  = 499;
  htim2.Instance->CCR1 = 250;
  htim2.Instance->EGR  = TIM_EGR_UG;
  osDelay(100);
  htim2.Instance->CCR1 = 0;
  osDelay(100);
  /* 聲2：4kHz，100ms */
  htim2.Instance->ARR  = 249;
  htim2.Instance->CCR1 = 125;
  htim2.Instance->EGR  = TIM_EGR_UG;
  osDelay(100);
  htim2.Instance->CCR1 = 0;

  /* === SD 卡初始化與 10 秒日誌啟動 === */
  extern FATFS SDFatFS;
  extern FIL SDFile;
  extern char SDPath[4];
  sd_logging_active = 0;
  uint32_t sd_log_start_tick = 0;
  char sd_fname[16] = {0};   // 本次開機使用的 SD 檔名 (e.g. HIL_003.CSV)
  
  // 為了防止 SDIO 在初始化、掛載與建立檔案時被高頻中斷干擾而超時失敗，在此期間暫停高頻中斷
  __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
  __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
  __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);

  printf("\r\n[SD] 正在檢測 SD 卡與進行掛載...\r\n");
  HAL_IWDG_Refresh(&hiwdg);
  
  printf("[SD] 正在初始化 SDIO 介面並檢測實體 SD 卡插入狀態...\r\n");
  HAL_IWDG_Refresh(&hiwdg);
  
  // 1. 物理插卡檢測，避免無卡時 f_mount 總線超時卡死
  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET) {
      printf("[SD] [OK] 檢測到實體 SD 卡已插入！正在掛載文件系統...\r\n");
      HAL_IWDG_Refresh(&hiwdg);
      
      // 2. 立即掛載 SD 卡 (opt=1)，避免 lazy mount 導致 f_stat 遇到 FR_NOT_READY
      if (f_mount(&SDFatFS, SDPath, 1) == FR_OK) {
          printf("[SD] [OK] 文件系統掛載成功！正在建立 HIL_xxx.CSV 檔案...\r\n");
          HAL_IWDG_Refresh(&hiwdg);
          
          // 找到第一個不存在的 HIL_xxx.CSV，確保每次開機寫入全新獨立檔案
          FILINFO fno;
          uint16_t fnum;
          for (fnum = 1; fnum <= 999; fnum++) {
              snprintf(sd_fname, sizeof(sd_fname), "HIL_%03u.CSV", fnum);
              FRESULT fres = f_stat(sd_fname, &fno);
              if (fres == FR_NO_FILE) break;      // 檔案不存在，用這個名稱
              if (fres != FR_OK) {                // 真正的錯誤（卡片未就緒等）
                  printf("[SD] [ERROR] f_stat(%s) 失敗，錯誤碼: %d\r\n", sd_fname, (int)fres);
                  fnum = 1000;
                  break;
              }
              // fres == FR_OK：檔案已存在，繼續找下一個號碼
          }
          printf("[SD] 準備建立新檔案: %s\r\n", sd_fname);
          HAL_IWDG_Refresh(&hiwdg);

          FRESULT fopen_res = f_open(&SDFile, sd_fname, FA_CREATE_NEW | FA_WRITE);
          if (fopen_res == FR_OK) {
              sd_logging_active = 1;
              sd_log_start_tick = HAL_GetTick();
              printf("[SD] [SUCCESS] %s 建立成功！開始全程飛行記錄（飛行 100Hz / 落地後 10Hz）...\r\n", sd_fname);
              char header[] = "tick_ms,bmi_ax,bmi_ay,bmi_az,adxl_ax,adxl_ay,adxl_az,temp,press,alt,fsm,ekf_alt_cm,ekf_vel_cms\r\n";
              UINT bw;
              f_write(&SDFile, header, sizeof(header) - 1, &bw);
          } else {
              printf("[SD] [ERROR] 建立檔案 %s 失敗！錯誤碼: %d\r\n", sd_fname, (int)fopen_res);
          }
      } else {
          printf("[SD] [ERROR] 掛載 SD 卡文件系統失敗！\r\n");
      }
  } else {
      printf("[SD] [WARNING] 未檢測到實體 SD 卡插入（SDIO_DET 為高電平），跳過 SD 飛行記錄。\r\n");
  }

  // SD 卡掛載與開檔流程結束後，重新恢復高頻中斷
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);

  /* === Flash Ring Buffer 初始化（SPI3，不影響 SPI1 感測器中斷） ===
   * 掃描寫入頭 + 預擦 10 個 Sector（~10×200ms = 2s），期間由函式內部餵狗 */
  FlashRing_Init();
 
  /* === 空中熱啟動（Hot-Restart）恢復偵測 === */
  FlashRingPacket_t last_pkt;
  if (FlashRing_GetLastPacket(&last_pkt) == W25QXX_OK) {
      if (last_pkt.fsm_state >= STATE_BOOST && last_pkt.fsm_state <= STATE_DESCENT) {
          printf("[FSM] ⚠️ WARNING: 檢測到空中斷電重啟！正在嘗試恢復飛行狀態...\r\n");
          printf("[FSM] 恢復前一狀態: %d, 飛行相對 Tick: %lu ms\r\n", last_pkt.fsm_state, last_pkt.tick_ms);
          
          EKF_calibrated = 1;     // 強制跳過 EKF 靜態校準
          EKF_in_flight = 1;      // 強制將 EKF 設為飛行狀態
          current_fsm_state = (FlightState_t)last_pkt.fsm_state;
          flight_start_tick = HAL_GetTick() - last_pkt.tick_ms; // 恢復起飛基準 Tick
      } else {
          current_fsm_state = STATE_PAD; // 正常地面起飛
      }
  } else {
      current_fsm_state = STATE_PAD; // 正常地面起飛
  }

  /* FlashRing_Init 阻塞約 2s，BMP388（主迴圈讀取）在此期間無法累積計數。
   * 重置所有採樣率監測器，確保第一個 [RATE] 報告反映穩態採樣率。 */
  SAMPLING_RATE_INIT();
  HAL_IWDG_Refresh(&hiwdg);

  /* Infinite loop */
  for(;;)
  {
    /* --- 餵狗 (Feed the independent watchdog) --- */
    HAL_IWDG_Refresh(&hiwdg);

    /* === SD 卡全程飛行記錄（FSM 驅動，取代原 10 秒上限）===
     * 升空至落地前以 100Hz 記錄；落地後 (g_touchdown_tick != 0) 降為 10Hz；
     * 停止條件：按下 USER_BT1，或落地後超過 SD_LANDED_LOG_TIMEOUT_MS；
     * 每秒 f_sync 一次，限制斷電造成的資料遺失 (Item E)。 */
    if (sd_logging_active) {
        uint8_t  landed   = (g_touchdown_tick != 0);
        uint32_t rate_div = landed ? 100U : 10U;     // 落地 10Hz / 飛行 100Hz

        uint8_t stop_request = 0;
        if (HAL_GPIO_ReadPin(USER_BT1_GPIO_Port, USER_BT1_Pin) == GPIO_PIN_RESET) {
            stop_request = 1;
            printf("[SD] USER_BT1 按下，停止記錄並安全關檔。\r\n");
        } else if (landed && (HAL_GetTick() - g_touchdown_tick) > SD_LANDED_LOG_TIMEOUT_MS) {
            stop_request = 1;
            printf("[SD] 落地後記錄逾時，停止記錄並安全關檔。\r\n");
        }

        if (stop_request) {
            f_close(&SDFile);
            f_mount(NULL, SDPath, 1);
            sd_logging_active = 0;
            printf("[SD] [SUCCESS] 檔案已安全關閉並儲存 (%s)。\r\n", sd_fname);
        } else if (tick % rate_div == 0) {
            EKF_State_t sd_ekf = EKF_GetState();
            uint32_t elapsed = HAL_GetTick() - sd_log_start_tick;
            char csv_buf[160];
            int len = snprintf(csv_buf, sizeof(csv_buf),
                               "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld\r\n",
                               (unsigned long)elapsed,
                               (int)(imu_data.ax * 1000.0f),
                               (int)(imu_data.ay * 1000.0f),
                               (int)(imu_data.az * 1000.0f),
                               (int)(highg_data.ax * 1000.0f),
                               (int)(highg_data.ay * 1000.0f),
                               (int)(highg_data.az * 1000.0f),
                               (int)(baro_data.temperature * 100.0f),
                               (int)(baro_data.pressure),
                               (int)(baro_data.altitude * 100.0f),
                               (int)current_fsm_state,
                               (long)(sd_ekf.pos_z * 100.0f),
                               (long)(sd_ekf.vel_z * 100.0f));
            UINT bytes_written;
            FRESULT res = f_write(&SDFile, csv_buf, len, &bytes_written);
            if (res != FR_OK) {
                printf("[SD] [ERROR] 寫入數據失敗！res=%d，停止記錄並關檔。\r\n", res);
                f_close(&SDFile);
                f_mount(NULL, SDPath, 1);
                sd_logging_active = 0;
            } else if (tick % 1000 == 0) {
                f_sync(&SDFile);   // 每秒 flush FAT/目錄，限制斷電資料遺失
            }
        }
    }

    /* === EKF 1000 Hz Downsampling & Synchronization (Runs when new sensor batches are ready) === */
    if (bmi088_ok && g_bmi_acc_new_batch_ready && g_bmi_gyro_new_batch_ready) {
        g_bmi_acc_new_batch_ready = 0;
        g_bmi_gyro_new_batch_ready = 0;

        uint8_t acc_ready_idx = g_bmi_acc_ready_batch_idx;
        uint8_t gyro_ready_idx = g_bmi_gyro_ready_batch_idx;

        /* 改善項目 A：高G感測器切換前置快照。
         * 在進入 EKF 樣本迴圈前鎖定 ADXL batch idx，避免迴圈執行期間 TIM3 ISR 完成新批
         * 次並更新 g_adxl_ready_batch_idx，導致同一個 10ms 窗格內讀到混合批次的資料。
         * 注意：不在此清除 g_adxl_new_batch_ready，保留給稍後的 highg_data 更新（~1808）。 */
        uint8_t adxl_hiG_avail = adxl375_ok && (g_adxl_new_batch_ready != 0);
        uint8_t adxl_hiG_bidx  = g_adxl_ready_batch_idx;

        // Record the frame start cycle (using DWT->CYCCNT)
        uint32_t frame_start_cycles = DWT->CYCCNT;

        // Loop to generate 10 synchronized samples for the current 10ms frame
        for (int i = 0; i < 10; i++) {
            float t = (float)i * 1.0f; // time in ms relative to frame start (0.0 to 9.0 ms)

            // 1. Accel Interpolation (16 samples over 10ms, spaced at 0.625 ms)
            // j = floor(t / 0.625) = floor(i * 1.6)
            float j_float = t / 0.625f;
            int j = (int)j_float;
            if (j < 0) j = 0;
            if (j > 14) j = 14;
            float fraction = j_float - (float)j;

            float ax = (1.0f - fraction) * g_bmi_acc_batches[acc_ready_idx][j].ax + fraction * g_bmi_acc_batches[acc_ready_idx][j+1].ax;
            float ay = (1.0f - fraction) * g_bmi_acc_batches[acc_ready_idx][j].ay + fraction * g_bmi_acc_batches[acc_ready_idx][j+1].ay;
            float az = (1.0f - fraction) * g_bmi_acc_batches[acc_ready_idx][j].az + fraction * g_bmi_acc_batches[acc_ready_idx][j+1].az;

            // Convert to m/s^2 by multiplying by 9.80665f (since raw readings are in g)
            ax *= 9.80665f;
            ay *= 9.80665f;
            az *= 9.80665f;

            /* 改善項目 A：高G感測器切換 —— |a_bmi| > 20g（196 m/s²）時以 ADXL375（±200g）替換
             * BMI088 最大量程 ±24g，超出後飽和；ADXL375 量程 ±200g，解析度 0.049g/LSB。
             * 本迴圈以 ADXL batch（3200Hz，32筆/10ms）中對應時刻的樣本索引取代 BMI，
             * i∈[0,9]→adxl_i=i×3∈[0,27]，近似 1ms 間距（精確 3200Hz/1000Hz=3.2）。
             * 單位：ADXL 輸出為 g（×0.049），乘 9.80665 → m/s²，與 BMI 路徑一致。
             * 偏差校正：EKF 校準期（發射台靜置 3s）以 BMI 資料計算 accel_bias；由於
             * ADXL Z 與 BMI Z 方向一致（均感測重力 ≈ 9.81 m/s²），BMI bias 對 ADXL Z 軸
             * 同樣適用（靜態時 accel_bias[2] ≈ 0）；ADXL X/Y 靜態偏置通常 < 50mg，
             * 在 20g+ 環境可忽略。
             * ⚠️ PCB X/Y 軸向對齊：BMI 與 ADXL 因板卡佈局 X/Y 反向可能不一致，需上板
             * bench 驗證（旋轉火箭艙，確認 ADXL ax/ay 符號與 BMI 一致）；若方向相反，
             * 於下方 ax/ay 各加負號（並加上 #define 以便開關）。 */
            if (adxl_hiG_avail) {
                float a_mag_sq = ax*ax + ay*ay + az*az;
                if (a_mag_sq > (20.0f * 9.80665f) * (20.0f * 9.80665f)) {  /* > 20g, 避免 sqrt */
                    int adxl_i = i * 3;
                    if (adxl_i > ADXL_BATCH_SIZE - 1) adxl_i = ADXL_BATCH_SIZE - 1;
                    ax = g_adxl_batches[adxl_hiG_bidx][adxl_i].ax * 9.80665f;
                    ay = g_adxl_batches[adxl_hiG_bidx][adxl_i].ay * 9.80665f;
                    az = g_adxl_batches[adxl_hiG_bidx][adxl_i].az * 9.80665f;
                }
            }

            // 2. Gyro Decimation (20 samples over 10ms, spaced at 0.5 ms)
            // Time match is exact: 2 * i * 0.5 ms = i * 1.0 ms
            int gyro_idx = 2 * i;
            if (gyro_idx > 19) gyro_idx = 19;

            float gx = g_bmi_gyro_batches[gyro_ready_idx][gyro_idx].gx;
            float gy = g_bmi_gyro_batches[gyro_ready_idx][gyro_idx].gy;
            float gz = g_bmi_gyro_batches[gyro_ready_idx][gyro_idx].gz;

            // Convert to rad/s by multiplying by PI / 180.0f (since raw readings are in dps)
            gx *= (M_PI / 180.0f);
            gy *= (M_PI / 180.0f);
            gz *= (M_PI / 180.0f);

            // 3. Assemble the sample
            EKF_Sample_t* p_sample = &g_ekf_buffers[g_ekf_active_idx].samples[g_ekf_sample_count];
            p_sample->ax = ax;
            p_sample->ay = ay;
            p_sample->az = az;
            p_sample->gx = gx;
            p_sample->gy = gy;
            p_sample->gz = gz;

            // Compute microsecond timestamp
            // 1ms is 168,000 CPU cycles at 168 MHz
            uint32_t cycles_offset = i * 168000;
            p_sample->timestamp_us = (frame_start_cycles + cycles_offset) / (SystemCoreClock / 1000000);

            // 4. Interleave Barometer readings at 200 Hz
            // This corresponds to index 0 and index 5 of our 10 synchronized samples
            if ((i == 0 || i == 5) && bmp388_ok) {
                p_sample->has_baro = 1;
                p_sample->baro_alt = baro_data.altitude;
            } else {
                p_sample->has_baro = 0;
                p_sample->baro_alt = 0.0f;
            }

            g_ekf_sample_count++;

            // 5. If buffer is full (100 samples), send it to the EKF playback task
            if (g_ekf_sample_count >= EKF_BUFFER_SIZE) {
                // Record buffer start time
                g_ekf_buffers[g_ekf_active_idx].start_time_us = g_ekf_buffers[g_ekf_active_idx].samples[0].timestamp_us;

                // Get pointer to full buffer
                EKF_Buffer_t* p_full_buffer = &g_ekf_buffers[g_ekf_active_idx];

                // Non-blocking queue send；put 失敗 (queue 滿) 代表 EKF_Task 消費不及，計數供觀測
                if (osMessageQueuePut(xEKFQueue, &p_full_buffer, 0, 0) != osOK) {
                    g_ekf_queue_drops++;
                }

                // Switch buffer index
                g_ekf_active_idx = !g_ekf_active_idx;
                g_ekf_sample_count = 0;
            }
        }

        // Also update standard imu_data structure for standard telemetry and logging
        imu_data.accel_x_raw = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].accel_x_raw;
        imu_data.accel_y_raw = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].accel_y_raw;
        imu_data.accel_z_raw = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].accel_z_raw;
        imu_data.ax          = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].ax;
        imu_data.ay          = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].ay;
        imu_data.az          = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].az;

        imu_data.gyro_x_raw  = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gyro_x_raw;
        imu_data.gyro_y_raw  = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gyro_y_raw;
        imu_data.gyro_z_raw  = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gyro_z_raw;
        imu_data.gx          = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gx;
        imu_data.gy          = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gy;
        imu_data.gz          = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gz;

        // 呼叫 FSM 狀態機進行 100Hz 定時狀態轉移判定
        FSM_Update();
    }

    /* === ADXL375 Ping-Pong 雙緩衝區批次數據提取 (3.2 kHz 中斷在背後默默採樣並填充) === */
    if (adxl375_ok && g_adxl_new_batch_ready) {
        g_adxl_new_batch_ready = 0;
        uint8_t ready_idx = g_adxl_ready_batch_idx;
        
        // 自 32 筆最新採樣數據中，提取最後一筆最新值供即時遙測傳送與顯示
        highg_data.x_raw = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].x_raw;
        highg_data.y_raw = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].y_raw;
        highg_data.z_raw = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].z_raw;
        highg_data.ax    = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].ax;
        highg_data.ay    = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].ay;
        highg_data.az    = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].az;
        
        // 提示：如需寫入 SD 卡，可在此處直接寫入整個 g_adxl_batches[ready_idx] 共 32 筆資料，即為 3.2 kHz HIL 實時存檔
    }

    /* === GPS（USART6 NMEA）：每迴圈嘗試消化一整句 ===
     * 無整句就緒時僅檢查旗標，極輕量；循環 DMA + IDLE 事件已於背景組句。
     * 每迴圈呼叫可即時排空單句緩衝，避免連續輸出時被新句覆蓋。 */
    GPS_Update();

    /* === GPS → EKF：偵測到新的有效定位才提交一次水平位置量測 ===
     * 以 last_fix_tick 變化判斷新定位，避免每迴圈 (1kHz) 重複提交同一筆。
     * EKF_SubmitGPS 內部負責原點鎖定與 ENU 換算；發射台 (ZUPT) 階段提交的
     * 定位會於 EKF_Task 內被丟棄，僅飛行中真正融合。 */
    {
        const GPS_Data_t *gfix = GPS_GetData();
        static uint32_t last_gps_fused_tick = 0;
        if (gfix->fix_valid && gfix->last_fix_tick != last_gps_fused_tick) {
            last_gps_fused_tick = gfix->last_fix_tick;
            EKF_SubmitGPS(gfix->lat_1e6, gfix->lon_1e6, gfix->satellites);
        }
    }

    /* === MMC5983MA 地磁計 @ 10 Hz (每 100ms 一次 one-shot 量測，~3ms 阻塞) ===
     * 地磁航向僅在發射台靜置階段用於 EKF yaw 修正，10Hz 已足夠；量測在 task context。 */
    if (mag_ok && (tick % 100 == 0)) {
        if (MMC5983_Read() == HAL_OK) {
            /* 提交 body-frame 磁場向量供 EKF 在發射台階段做 yaw 修正。
             * 軸向假設與 IMU body frame 對齊；若 bench 測試航向反向，於此處對相應軸取負。 */
            const MMC5983_Data_t *mg = MMC5983_GetData();
            EKF_SubmitMag(mg->gauss[0], mg->gauss[1], mg->gauss[2]);
        }
    }
    /* 每 10 秒重做一次 SET/RESET 橋偏校準以補溫漂；飛行中不做（地磁融合僅限發射台）。 */
    if (mag_ok && !EKF_in_flight && (tick % 10000 == 0) && (tick > 0)) {
        MMC5983_Recalibrate();
    }

    /* === BMP388 @ 200 Hz (每 5ms，Normal Mode 最高 ODR=200Hz，1x OS 轉換時間 4.94ms < 5ms 週期) === */
    if (tick % 5 == 0 && bmp388_ok) {
        // 在任務讀取 BMP388 (SPI1) 期間，暫時關閉 TIM3 中斷，防止高優先權中斷搶佔 SPI1 匯流排
        __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
        BMP388_ReadData(&hspi1, &baro_data);
        __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
        
        RATE_TICK_BMP388();   /* 統計 BMP388 實際採樣率 → g_sampling_rate.bmp388.rate_hz */
    }

    /* === Flash Ring Buffer 寫入：依 FSM 狀態分級寫入速率（改善項目 F） ===
     * Flash 在 SPI3，感測器在 SPI1，兩者獨立，無需停中斷。
     *   INIT/PAD（發射台/初始化）：20 Hz —— 足夠且省 Flash 壽命
     *   BOOST..MAIN_DEPLOY（飛行各相位）：50 Hz —— 提高動態階段時間解析度
     *   LANDED（著陸後）：1 Hz —— 長時間尋標等待，大幅省 Flash 壽命
     * 飛行率刻意取 50 Hz 而非計畫原稿的 100 Hz：FlashRing_WritePacket 內含「同步」滾動
     * Sector 擦除（W25Q128 4KB sector 最壞 ~400ms 阻塞），會卡住主迴圈的感測器批次提取
     * 與 EKF 饋入。50 Hz 下擦除約每 1.56s 一次（與目前 20Hz 的 3.9s 同數量級、可接受）；
     * 要安全提升至 100 Hz 須先把擦除移到獨立低優先任務（改善項目 G，需 SPI3 mutex 與
     * 上板時序驗證）。 */
    uint32_t ring_period_ms;
    if (current_fsm_state == STATE_LANDED) {
        ring_period_ms = 1000;   /* 1 Hz */
    } else if (current_fsm_state >= STATE_BOOST && current_fsm_state <= STATE_MAIN_DEPLOY) {
        ring_period_ms = 20;     /* 50 Hz */
    } else {
        ring_period_ms = 50;     /* 20 Hz (INIT/PAD) */
    }
    if (tick % ring_period_ms == 0) {
        /* Flash Ring Buffer：將目前感測器數據打包寫入 */
        FlashRingPacket_t ring_pkt = {0};
        ring_pkt.tick_ms     = HAL_GetTick();
        ring_pkt.bmi_ax      = imu_data.accel_x_raw;
        ring_pkt.bmi_ay      = imu_data.accel_y_raw;
        ring_pkt.bmi_az      = imu_data.accel_z_raw;
        ring_pkt.bmi_gx      = imu_data.gyro_x_raw;
        ring_pkt.bmi_gy      = imu_data.gyro_y_raw;
        ring_pkt.bmi_gz      = imu_data.gyro_z_raw;
        ring_pkt.adxl_x      = highg_data.x_raw;
        ring_pkt.adxl_y      = highg_data.y_raw;
        ring_pkt.adxl_z      = highg_data.z_raw;
        ring_pkt.temperature = (int32_t)(baro_data.temperature * 100.0f);
        ring_pkt.pressure    = (uint32_t)(baro_data.pressure);
        ring_pkt.altitude_cm = (int32_t)(baro_data.altitude * 100.0f);
        /* GPS 經緯度（×1e6 deg）：僅在有有效定位時填入，否則維持 0 */
        const GPS_Data_t *gps_pkt = GPS_GetData();
        if (gps_pkt->fix_valid) {
            ring_pkt.gps_lat = gps_pkt->lat_1e6;
            ring_pkt.gps_lon = gps_pkt->lon_1e6;
        } else {
            ring_pkt.gps_lat = 0;
            ring_pkt.gps_lon = 0;
        }
        ring_pkt.fsm_state   = (uint8_t)current_fsm_state;
        ring_pkt.flags       = 0;
        FlashRing_WritePacket(&ring_pkt);
    }

    /* === 每 100ms：10 Hz UART 遙測輸出 (減少 CPU 阻塞開銷) === */
    if (tick % 100 == 0) {
        /* UART 遙測輸出
         * 格式: bmi_ax(mg),bmi_ay(mg),bmi_az(mg),adxl_ax(mg),adxl_ay(mg),adxl_az(mg),
         *        temp(*100 degC),press(Pa),alt(cm)
         */
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               (int)(imu_data.ax * 1000.0f),
               (int)(imu_data.ay * 1000.0f),
               (int)(imu_data.az * 1000.0f),
               (int)(highg_data.ax * 1000.0f),
               (int)(highg_data.ay * 1000.0f),
               (int)(highg_data.az * 1000.0f),
               (int)(baro_data.temperature * 100.0f),
               (int)(baro_data.pressure),
               (int)(baro_data.altitude * 100.0f));
    }

    /* === 每 1000ms：1 Hz GPS 狀態遙測（NMEA 標準輸出多為 1Hz）===
     * 經緯度以正負號 + 絕對值列印，避免「整數部為 0 但實際為負」時遺失負號。 */
    if (tick % 1000 == 0) {
        const GPS_Data_t *g = GPS_GetData();
        char     lat_sign = (g->lat_1e6 < 0) ? '-' : '+';
        char     lon_sign = (g->lon_1e6 < 0) ? '-' : '+';
        uint32_t lat_abs  = (g->lat_1e6 < 0) ? (uint32_t)(-g->lat_1e6) : (uint32_t)g->lat_1e6;
        uint32_t lon_abs  = (g->lon_1e6 < 0) ? (uint32_t)(-g->lon_1e6) : (uint32_t)g->lon_1e6;
        printf("[GPS] fix:%d q:%d sat:%d %c%lu.%06lu,%c%lu.%06lu alt:%dm spd:%dcm/s stale:%d ok:%lu err:%lu\r\n",
               (int)g->fix_valid,
               (int)g->fix_quality,
               (int)g->satellites,
               lat_sign, (unsigned long)(lat_abs / 1000000U), (unsigned long)(lat_abs % 1000000U),
               lon_sign, (unsigned long)(lon_abs / 1000000U), (unsigned long)(lon_abs % 1000000U),
               (int)g->altitude_m,
               (int)(g->speed_mps * 100.0f),
               (int)GPS_IsStale(2000),
               (unsigned long)g->sentences_ok,
               (unsigned long)g->sentences_err);

        /* 地磁計 1 Hz 診斷：磁場 (mGauss) 與粗略水平航向 (deg)。 */
        if (mag_ok) {
            const MMC5983_Data_t *m = MMC5983_GetData();
            printf("[MAG] B[mG]:%d,%d,%d hdg:%d ok:%lu err:%lu\r\n",
                   (int)(m->gauss[0] * 1000.0f),
                   (int)(m->gauss[1] * 1000.0f),
                   (int)(m->gauss[2] * 1000.0f),
                   (int)m->heading_deg,
                   (unsigned long)m->reads_ok,
                   (unsigned long)m->reads_err);
        }
    }

    /* === 每 500ms 翻轉 LED 一次，形成溫和的 1 Hz 視覺心跳 (0.5s 亮 / 0.5s 暗) === */
    if (tick % 500 == 0) {
        HAL_GPIO_TogglePin(LED_SYS_GPIO_Port, LED_SYS_Pin);
    }

    /* === 感測器狀態 LED（每 100ms 更新，與 mag 讀取週期對齊，避免 1kHz 高頻寫 GPIO）===
     * STAT1 (PE3)：GPS 有有效定位 (fix_valid=1) 則常亮，否則熄滅
     * STAT2 (PE4)：磁力計初始化成功 (mag_ok=1) 且最近一次讀取 OK (reads_ok>0) 則常亮 */
    if (tick % 100 == 0) {
        /* GPS STAT1 */
        const GPS_Data_t *led_gps = GPS_GetData();
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,
                          led_gps->fix_valid ? GPIO_PIN_SET : GPIO_PIN_RESET);

        /* Mag STAT2 */
        uint8_t mag_led = 0;
        if (mag_ok) {
            const MMC5983_Data_t *led_mag = MMC5983_GetData();
            mag_led = (led_mag->ok && led_mag->reads_ok > 0) ? 1 : 0;
        }
        HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin,
                          mag_led ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

#ifdef RATE_MONITOR_ENABLE
    /* 每 1000ms 輸出一次各感測器實際採樣率，便於自動化測試腳本讀取與斷言分析 */
    if (tick % 1000 == 0) {
        printf("[RATE] BMI088_A:%uHz, BMI088_G:%uHz, ADXL375:%uHz, BMP388:%uHz, SD_DET:%d, EKF_DROP:%lu\r\n",
               (unsigned int)g_sampling_rate.bmi088_acc.rate_hz,
               (unsigned int)g_sampling_rate.bmi088_gyro.rate_hz,
               (unsigned int)g_sampling_rate.adxl375.rate_hz,
               (unsigned int)g_sampling_rate.bmp388.rate_hz,
               (int)HAL_GPIO_ReadPin(SDIO_DET_GPIO_Port, SDIO_DET_Pin),
               (unsigned long)g_ekf_queue_drops);
        printf("[FLASH_RING] PKT_TOTAL:%lu ADDR:0x%06lX\r\n",
               FlashRing_GetPacketCount(), FlashRing_GetWriteAddr());
    }
#endif

    tick++;
    osDelay(1); /* 1ms tick — 決定整體採樣基準 */
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM3)
  {
      if (adxl375_ok)
      {
          ADXL375_Data_t sensor_data;
          // 直接在中斷中以極速阻塞讀取 ADXL375 (SPI1, ~5.6us)
          if (ADXL375_ReadData(&hspi1, &sensor_data) == HAL_OK)
          {
              uint8_t current_batch = g_adxl_active_batch;
              g_adxl_batches[current_batch][g_adxl_sample_count].x_raw = sensor_data.x_raw;
              g_adxl_batches[current_batch][g_adxl_sample_count].y_raw = sensor_data.y_raw;
              g_adxl_batches[current_batch][g_adxl_sample_count].z_raw = sensor_data.z_raw;
              g_adxl_batches[current_batch][g_adxl_sample_count].ax = sensor_data.ax;
              g_adxl_batches[current_batch][g_adxl_sample_count].ay = sensor_data.ay;
              g_adxl_batches[current_batch][g_adxl_sample_count].az = sensor_data.az;

              g_adxl_sample_count++;
              
              // 在中斷內進行採樣率累加，回報精準的 3200 Hz 物理採樣頻率
              RATE_TICK_ADXL375();

              if (g_adxl_sample_count >= ADXL_BATCH_SIZE)
              {
                  g_adxl_ready_batch_idx = current_batch;
                  g_adxl_active_batch = !current_batch;
                  g_adxl_sample_count = 0;
                  g_adxl_new_batch_ready = 1;
              }
          }
      }
  }
  else if (htim->Instance == TIM6)
  {
      if (bmi088_ok)
      {
          BMI088_Accel_t acc_raw;
          // TIM6 1.6 kHz 中斷只讀取 BMI088 加速度計 (SPI2)
          if (BMI088_ReadAccel(&hspi2, &acc_raw) == HAL_OK)
          {
              uint8_t current_batch = g_bmi_acc_active_batch;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].accel_x_raw = acc_raw.accel_x_raw;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].accel_y_raw = acc_raw.accel_y_raw;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].accel_z_raw = acc_raw.accel_z_raw;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].ax          = acc_raw.ax;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].ay          = acc_raw.ay;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].az          = acc_raw.az;

              g_bmi_acc_sample_count++;

              // 累加 Accel 採樣計數
              RATE_TICK_BMI088_ACC();

              if (g_bmi_acc_sample_count >= BMI_ACC_BATCH_SIZE)
              {
                  g_bmi_acc_ready_batch_idx = current_batch;
                  g_bmi_acc_active_batch = !current_batch;
                  g_bmi_acc_sample_count = 0;
                  g_bmi_acc_new_batch_ready = 1;
              }
          }
      }
  }
  else if (htim->Instance == TIM7)
  {
      if (bmi088_ok)
      {
          BMI088_Gyro_t gyro_raw;
          // TIM7 2.0 kHz 中斷只讀取 BMI088 陀螺儀 (SPI2)
          if (BMI088_ReadGyro(&hspi2, &gyro_raw) == HAL_OK)
          {
              uint8_t current_batch = g_bmi_gyro_active_batch;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gyro_x_raw = gyro_raw.gyro_x_raw;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gyro_y_raw = gyro_raw.gyro_y_raw;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gyro_z_raw = gyro_raw.gyro_z_raw;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gx         = gyro_raw.gx;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gy         = gyro_raw.gy;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gz         = gyro_raw.gz;

              g_bmi_gyro_sample_count++;

              // 累加 Gyro 採樣計數
              RATE_TICK_BMI088_GYRO();

              if (g_bmi_gyro_sample_count >= BMI_GYRO_BATCH_SIZE)
              {
                  g_bmi_gyro_ready_batch_idx = current_batch;
                  g_bmi_gyro_active_batch = !current_batch;
                  g_bmi_gyro_sample_count = 0;
                  g_bmi_gyro_new_batch_ready = 1;
              }
          }
      }
  }
  /* USER CODE END Callback 1 */
}

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

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
#include "rate_monitor.h"  /* 採樣率監測模組，要停用請在 rate_monitor.h 註解 #define RATE_MONITOR_ENABLE */
#include "ekf.h"
#include "gps.h"
#include "mmc5983.h"
#include "sensor_axis.h"   /* 感測器->body 軸向映射（唯一真相來源，見 sensor_axis.h） */
#include "telemetry.h"     /* 共用二進制+CRC16 下行遙測封包 */
#include "sensor_health.h" /* P0-D：感測器失流/卡死/範圍偵測（host 可測純邏輯） */
#include "lora_e22.h"      /* E22-400T30S 433MHz LoRa 透傳 (UART3) */
#include "lora_e80.h"      /* E80-900M2213S 920MHz LoRa (SX126x SPI3) */
#include "spi3_bus.h"      /* SPI3 共用匯流排互斥鎖 (Flash + E80) */
#include "board_config.h"  /* 主/備角色與功能旗標（單一程式碼雙板） */
#include "link_hw.h"       /* 板間鏈路（USART2 全雙工）；FEATURE_LINK 下編入 */
#include "ground_station.h" /* 地面站主流程（IS_GROUND 下編入；其餘角色為空） */
#include "gs_lora_test.h"   /* 地面站 LoRa 通訊測試模組（UART2 命令 + 雙鏈路統計） */
#include "uplink_cmd.h"     /* 上行手動開傘（FEATURE_UPLINK_DEPLOY 下編入；其餘為空） */
#include "usb_device.h"     /* USB-CDC 初始化（FEATURE_USB_CDC 下啟用） */
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
DMA_HandleTypeDef hdma_i2c1_rx; // Dummy handle to prevent linker/compiler errors in generated code

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,  /* 2048 bytes: Bosch BMP3 API 深度呼叫 + VLA 需要足夠的堆疊空間 */
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
#define ENABLE_DIAGNOSTICS 1  /* 註解此行即可完全關閉診斷任務以節省資源 */

volatile float g_main_task_cpu_usage = 0.0f;
FlightState_t current_fsm_state = STATE_INIT;   /* 由 FSM 包裝層自 g_fsm_ctx 鏡射（P0-A） */
uint32_t flight_start_tick = 0;                 /* 同上；速度歷史 last_vel_z 已收入 FSM_Context_t */
uint8_t sd_logging_active = 0;

/* P1：任務堆疊水位觀測用 handle（defaultTask 用 CubeMX 既有的 defaultTaskHandle） */
static osThreadId_t g_ekf_task_handle  = NULL;
static osThreadId_t g_lora_task_handle = NULL;

/* P0-D：感測器健康監測（失流/卡死/範圍）。彙整位 SH_BIT_* 供 FSM/遙測/[HEALTH] 行。 */
static SensorMon_t g_mon_bmi088;
static SensorMon_t g_mon_adxl375;
static SensorMon_t g_mon_bmp388;
volatile uint8_t g_sensor_fault_bits = 0;
/* 落地時刻 tick（0 = 尚未落地）。SD 全程記錄據此由 100Hz 降為 10Hz，並於逾時後自動關檔 (Item E)。 */
volatile uint32_t g_touchdown_tick = 0;

BMI088_Data_t imu_data;
ADXL375_Data_t highg_data;
BMP388_Data_t baro_data;

uint8_t bmi088_ok = 0;
uint8_t adxl375_ok = 0;
uint8_t bmp388_ok = 0;
uint8_t mag_ok = 0;       /* MMC5983MA 地磁計（I2C1）初始化狀態 */
uint8_t lora433_ok = 0;   /* E22-400T30S 433MHz LoRa (UART3 透傳) 初始化狀態 */
uint8_t lora920_ok = 0;   /* E80-900M2213S 920MHz LoRa (SX126x SPI3) 初始化狀態 */

/* 最新電池電壓 (mV)，由飛控迴圈讀 ADC 後更新；遙測與 [PWR] 行唯讀此值（避免 ADC 並發） */
volatile uint16_t g_bat_voltage_mv = 0;

/* SPI3 共用匯流排互斥鎖（W25Q128 Flash + E80 920MHz LoRa）。於 RTOS init 建立。 */
osMutexId_t g_spi3_mutex = NULL;

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
#if !FEATURE_USB_CDC
static void MX_USB_OTG_FS_PCD_Init(void);
#endif
static void MX_CRC_Init(void);
static void MX_RNG_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
uint16_t ADC_Read_Battery_mv(void);
void LoRaTelemetry_Task(void *argument);   /* 5Hz 下行遙測：E22(433) + E80(920) */
#ifdef ENABLE_DIAGNOSTICS
void StartDiagnosticTask(void *argument);
#endif
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
  /* 擷取上一次重置原因（診斷「拔掉 UART 就一直重啟」）。RCC CSR 旗標跨重置保留至清除。 */
  uint32_t reset_csr = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
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
#if !FEATURE_USB_CDC
  MX_USB_OTG_FS_PCD_Init();   /* 飛行版：低階 PCD；地面站改由 USB-CDC 的 usbd_conf 帶起 */
#endif
  MX_CRC_Init();
  MX_RNG_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);   /* 關閉 stdout 緩衝，使所有 FreeRTOS 任務的 printf 立即送往 UART，防慢速任務因獨立 reent 結構積壓不印 */
  HAL_Delay(500);   /* 延遲 500ms 讓 USB-TTL 串口驅動在 MCU 重置後有時間穩定，防止 PC 端漏字 */
  printf("\r\n============================================================\r\n");
  printf("[VERSION] Firmware Version: %s\r\n", FIRMWARE_VERSION);
  printf("============================================================\r\n");
  /* 開機印出重置原因：POR/BOR=電源/欠壓(拔 UART 掉電/LoRa 發射突波最常見)；
   * IWDG=看門狗逾時(任務卡住)；SOFT=軟體重置；PIN=外部復位腳。 */
  printf("[RESET] %s%s%s%s%s%s%s(CSR=0x%08lX)\r\n",
         (reset_csr & RCC_CSR_BORRSTF)  ? "BOR(欠壓) "          : "",
         (reset_csr & RCC_CSR_PORRSTF)  ? "POR/PDR(上電/欠壓) " : "",
         (reset_csr & RCC_CSR_IWDGRSTF) ? "IWDG(看門狗) "       : "",
         (reset_csr & RCC_CSR_WWDGRSTF) ? "WWDG "               : "",
         (reset_csr & RCC_CSR_SFTRSTF)  ? "SOFT(軟體) "         : "",
         (reset_csr & RCC_CSR_LPWRRSTF) ? "LPWR "               : "",
         (reset_csr & RCC_CSR_PINRSTF)  ? "PIN(外部腳) "        : "",
         (unsigned long)reset_csr);
  fflush(stdout);
#if FEATURE_USB_CDC
  MX_USB_DEVICE_Init();   /* 地面站：USB 虛擬序列埠 (CDC)。飛行版不啟用（不列舉、不佔 CPU）。 */
#endif
#if FEATURE_FLIGHT   /* 主/備：飛控感測器 (IMU/高G/氣壓)；地面站 (ROLE_GROUND) 不啟用以省資源 */
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
#else
  bmi088_ok = 0;
  adxl375_ok = 0;
  bmp388_ok = 0;   /* 地面站：不啟用 IMU/高G/氣壓感測器 */
#endif /* FEATURE_FLIGHT */

  /* GPS 驅動啟動 (USART6, NMEA-0183, 循環 DMA + IDLE 接收)。
   * NEO-M9N 實體掛載於 USART6（PC6/PC7）；解析於 task context（主迴圈 GPS_Update()）進行，
   * ISR/DMA 事件回呼僅組句設旗標。 */
#if FEATURE_GPS
  GPS_Init(&huart6);
#endif

  /* MMC5983MA 三軸地磁計啟動 (I2C1, SCL=PB7/SDA=PB8, 位址 0x30)。
   * 初始化含 Product ID 驗證、軟體重置、400Hz 頻寬與一次 SET/RESET 橋偏校準。
   * 失敗（晶片未上線）僅記錄旗標，不阻擋系統啟動。 */
#if FEATURE_MAG
  if (MMC5983_Init() == HAL_OK) {
      mag_ok = 1;
      printf("[MAG] MMC5983MA online. offset[X,Y,Z]=%ld,%ld,%ld\r\n",
             (long)MMC5983_GetData()->offset[0],
             (long)MMC5983_GetData()->offset[1],
             (long)MMC5983_GetData()->offset[2]);
  } else {
      mag_ok = 0;
      printf("[MAG] MMC5983MA NOT detected (I2C1).\r\n");
  }
#else
  mag_ok = 0;   /* 備板：磁力計 (I2C1) 關閉 */
#endif

#if FEATURE_LORA
  /* E22-400T30S 433MHz LoRa 啟動（UART3 透傳模式 M0=M1=0）。
   * 重置 + 等 AUX 拉高；未接模組也不阻擋啟動（發送由 AUX 背壓守護）。
   * 地面站固定啟用（收 433 下行）；主航電由 LORA433_ENABLE 決定是否啟用 433。 */
#if IS_GROUND || LORA433_ENABLE
  lora433_ok = 0;
  for (int retry = 0; retry < 3; retry++) {
      HAL_IWDG_Refresh(&hiwdg);
      if (LoRaE22_Init(&huart3) == HAL_OK) {
          lora433_ok = 1;
          break;
      }
      HAL_Delay(50);
  }
  if (lora433_ok) {
      printf("[LORA433] E22 偵測成功（config 回讀 OK），透傳就緒 (UART3)。\r\n");
  } else {
      printf("[LORA433] ⚠ E22 未偵測到（config 無回應）—— 確認模組/接線/供電；主航電遙測任務每 10s 重試。\r\n");
  }
#if FEATURE_UPLINK_DEPLOY
  /* 主航電：在下行之外開啟 USART3 上行接收（地面站手動開傘命令）。
   * E22 透傳模式不發送時即在接收；遙測任務每 10 槽空出 1 槽不發 433 留給上行。 */
  UplinkCmd_Init();
#endif
#else
  lora433_ok = 0;
  printf("[LORA433] disabled (LORA433_ENABLE=0) — 433 下行停用，UART3 不發送。\r\n");
#endif

  HAL_IWDG_Refresh(&hiwdg);

  /* E80-900M2213S 920MHz LoRa 啟動（SX126x/LLCC68, SPI3，與 Flash 共用匯流排）。 */
#if IS_GROUND
  lora920_ok = 0;
  for (int retry = 0; retry < 3; retry++) {
      HAL_IWDG_Refresh(&hiwdg);
      if (LoRaE80_Init(&hspi3) == HAL_OK && LoRaE80_StartRx() == HAL_OK) {
          lora920_ok = 1;
          break;
      }
      HAL_Delay(50);
  }
  if (lora920_ok) {
      printf("[LORA920] E80 RX online (SX126x SPI3, continuous RX).\r\n");
  } else {
      LoRaE80_Shutdown();
      printf("[LORA920] E80 RX init failed — held in reset (SPI3 freed for Flash).\r\n");
  }
#elif LORA920_ENABLE
  lora920_ok = 0;
  for (int retry = 0; retry < 3; retry++) {
      HAL_IWDG_Refresh(&hiwdg);
      if (LoRaE80_Init(&hspi3) == HAL_OK) {
          lora920_ok = 1;
          break;
      }
      HAL_Delay(50);
  }
  if (lora920_ok) {
      /* Init 已完成全部 RF 設定（standby_rc → RF 開關 → LoRa 封包型態 → HP PA/VBAT →
       * 920MHz/SF9/BW250/CR4-5/+22dBm → 封包參數 → sync word → TxDone IRQ）。
       * 不再啟動連續載波(CW)；E80 留在 standby，由 LoRaTelemetry_Task 每包脈衝 SetTx 發送下行。 */
      printf("[LORA920] E80 online (LR1121 SPI3) — standby，下行由遙測任務每包脈衝發送。\r\n");
  } else {
      LoRaE80_Shutdown();   /* 偵測失敗（含 MISO 接地/浮空）→ RST 拉低隔離，釋放 SPI3 */
      printf("[LORA920] E80 NOT detected — held in reset to free SPI3 (protect Flash).\r\n");
  }
#else
  lora920_ok = 0;
  LoRaE80_Shutdown();       /* 主動停用：RST 拉低使 E80 不驅動共用 SPI3 MISO */
  printf("[LORA920] disabled (LORA920_ENABLE=0) — held in reset, SPI3 freed for Flash.\r\n");
#endif
#else  /* !FEATURE_LORA（備援航電）：不啟用任何 LoRa；仍按住 E80 RST 保護 SPI3 上的 Flash */
  lora433_ok = 0;
  lora920_ok = 0;
  LoRaE80_Shutdown();       /* RST 拉低使 E80 不驅動共用 SPI3 MISO（飛行資料記錄保護） */
#endif
  /* E22(最壞 900ms) + E80(最壞 560ms) 合計可達 ~1.5s；補餵一次防 IWDG 2.05s 到期。
   * Flash_Test() 含 sector erase(400ms) + write(500ms) 再補一次。 */
  HAL_IWDG_Refresh(&hiwdg);

#if FEATURE_FLASH
  /* W25Qxx SPI Flash 啟動自檢 (SPI3, CS=PA15) */
  Flash_Test();
  HAL_IWDG_Refresh(&hiwdg);
#endif

  /* Buzzer 開機提示音：依角色不同節奏，耳朵即可確認「燒對角色」（呼應 board_config 角色易混）。
   * TIM2 時基 1MHz → 頻率 = 1e6/(ARR+1)。所有角色都會響（TIM2 與此段皆無條件執行）。 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
#if IS_GROUND
  for (int b = 0; b < 3; b++) {                 /* 地面站：三短聲 3kHz */
      htim2.Instance->ARR = 332; htim2.Instance->CCR1 = 166; htim2.Instance->EGR = TIM_EGR_UG;
      HAL_Delay(90);
      htim2.Instance->CCR1 = 0;
      HAL_Delay(90);
  }
#elif IS_BACKUP
  htim2.Instance->ARR = 499; htim2.Instance->CCR1 = 250; htim2.Instance->EGR = TIM_EGR_UG;  /* 備援：一長聲 2kHz */
  HAL_Delay(500);
  htim2.Instance->CCR1 = 0;
#else                                            /* 主航電：兩聲漸高 2kHz→4kHz */
  htim2.Instance->ARR = 499; htim2.Instance->CCR1 = 250; htim2.Instance->EGR = TIM_EGR_UG;
  HAL_Delay(100);
  htim2.Instance->CCR1 = 0;
  HAL_Delay(100);
  htim2.Instance->ARR = 249; htim2.Instance->CCR1 = 125; htim2.Instance->EGR = TIM_EGR_UG;
  HAL_Delay(100);
  htim2.Instance->CCR1 = 0;
#endif
  HAL_IWDG_Refresh(&hiwdg);   /* 提示音占用 ~0.4–0.5s，補餵一次防 IWDG */

#if FEATURE_LINK
  /* 板間鏈路（USART2 全雙工）：重設 baud 為 LINK_BAUD 並啟動循環 DMA/IDLE 接收。
   * 主備兩板程式相同，靠板間排線把 TX/RX 交叉對接。啟用後 USART2 不再作 printf 除錯橋。 */
  Link_Init();
#endif
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* SPI3 匯流排互斥鎖：W25Q128 Flash 與 E80 920MHz LoRa 共用 SPI3，須互斥存取。
   * 遞迴 + 優先級繼承：容忍 flash 驅動 CS 巨集的防禦性釋放，並避免高優先飛控任務
   * 等待 mutex 時被低優先 LoRa 任務造成優先級反轉。 */
  static const osMutexAttr_t spi3_mutex_attr = {
    .name = "spi3Bus",
    .attr_bits = osMutexRecursive | osMutexPrioInherit,
  };
  g_spi3_mutex = osMutexNew(&spi3_mutex_attr);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
#if FEATURE_FLIGHT
  xEKFQueue = osMessageQueueNew(2, sizeof(EKF_Buffer_t*), NULL);
#endif
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#if FEATURE_FLIGHT
  g_ekf_task_handle = osThreadNew(EKF_Task, NULL, &EKF_Task_attributes);
#endif
#if defined(ENABLE_DIAGNOSTICS) && !IS_GROUND
  static const osThreadAttr_t diagnosticTask_attributes = {
    .name = "diagTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityLow,
  };
  osThreadNew(StartDiagnosticTask, NULL, &diagnosticTask_attributes);
#endif
#if FEATURE_LORA_TX
  /* LoRa 下行遙測任務（低優先，不影響 1kHz 飛控迴圈時序）。地面站 (FEATURE_LORA_TX=0) 只收不發。
   * 即使兩個 LoRa 模組都未初始化也建立——任務內自會跳過未就緒的鏈路。
   * 備援航電（FEATURE_LORA=0）不建立此任務：無 LoRa 下行，省去 E22 週期重試。 */
  static const osThreadAttr_t loraTelemTask_attributes = {
    .name = "loraTxTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityLow,
  };
  g_lora_task_handle = osThreadNew(LoRaTelemetry_Task, NULL, &loraTelemTask_attributes);
#endif
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
#if !FEATURE_USB_CDC   /* 地面站不需低階 PCD：USB-CDC 由 usbd_conf 的 USBD_LL_Init 帶起 */
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
#endif /* !FEATURE_USB_CDC */

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
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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

  /* P1：LORA920_BUSY (PD6) 由 CubeMX 的 IT_RISING_FALLING 覆寫為純輸入。
   * lora_e80.c 只輪詢 BUSY、從不用中斷；E80 SPI 交易期間 BUSY 高頻 toggle
   * 會狂觸 EXTI9_5 ISR（無動作純開銷，與 BMI/ADXL 中斷搶 CPU）。
   * DeInit 清除 EXTI line 6 映射（PD6 為該 line 唯一使用者）再重設為輸入；
   * 放在 USER CODE 區塊，CubeMX 重新產生程式碼後仍保留本覆寫。 */
  HAL_GPIO_DeInit(LORA920_BUSY_GPIO_Port, LORA920_BUSY_Pin);
  GPIO_InitStruct.Pin  = LORA920_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA920_BUSY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;
#if FEATURE_LINK
  /* USART2 已作板間鏈路；printf 改走 SWO/ITM（需 SWD/SWV 觀看；無除錯器時 ITM_SendChar
   * 自動略過、不阻塞）。如需回復 USART2 文字除錯橋，建置時設 -DFEATURE_LINK=0。 */
  for (int i = 0; i < len; i++) {
    ITM_SendChar((uint32_t)(uint8_t)ptr[i]);
  }
#else
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
#endif
  return len;
}

/* === FSM 包裝層（P0-A）：純邏輯已抽離至 fsm.c/h，由 tests/test_fsm.c 驗證 ===
 * 此處職責：組輸入快照 → FSM_Step() → 執行硬體動作 → 鏡射全域 → 事件列印。
 * 硬體動作（點火/舵機/蜂鳴器）嚴格先於 printf：避免 UART 阻塞（~9ms/行）延遲開傘。 */
static FSM_Context_t g_fsm_ctx;
volatile uint8_t g_fsm_failsafe_fired = 0;   /* 失效保護點火鎖存（telemetry TELEM_FLAG_FAILSAFE 讀取） */
volatile uint8_t g_hotstart_restored  = 0;   /* P0-F：熱啟動恢復鎖存（telemetry TELEM_FLAG_HOTSTART） */

static void FSM_Update(void)
{
    EKF_State_t ekf_state = EKF_GetState();
    uint32_t now = HAL_GetTick();

    /* === P0-B：baro 發射台基準（pad_ref） ===
     * PAD/INIT 期每 30s 重零（抗氣象漂移），起飛後凍結；
     * 首筆有效氣壓樣本（pressure > 1000 Pa）才建立基準，避免開機初期髒值。 */
    static float    pad_ref       = 0.0f;
    static uint8_t  pad_ref_valid = 0U;
    static uint32_t pad_ref_tick  = 0U;
    if (g_fsm_ctx.state <= STATE_PAD && baro_data.pressure > 1000.0f) {
        if (!pad_ref_valid || (now - pad_ref_tick) >= 30000U) {
            pad_ref       = baro_data.altitude;
            pad_ref_valid = 1U;
            pad_ref_tick  = now;
        }
    }

    /* P0-D：彙整感測器健康位（100Hz；初始化失敗從未餵入 → STALE 自動涵蓋） */
    uint8_t sens_bits = 0U;
    if (sensor_mon_status(&g_mon_bmi088,  now) != 0U) sens_bits |= SH_BIT_BMI088;
    if (sensor_mon_status(&g_mon_adxl375, now) != 0U) sens_bits |= SH_BIT_ADXL375;
    if (sensor_mon_status(&g_mon_bmp388,  now) != 0U) sens_bits |= SH_BIT_BMP388;
    g_sensor_fault_bits = sens_bits;

    FSM_Input_t in;
    in.now_ms         = now;
    in.h_est          = ekf_state.pos_z;   // 卡爾曼估計高度 (m)
    in.v_est          = ekf_state.vel_z;   // 卡爾曼估計垂直速度 (m/s)
    in.a_z_g          = highg_data.az;     // 高 G 垂直加速度 (g)
    in.baro_alt_rel   = pad_ref_valid ? (baro_data.altitude - pad_ref) : 0.0f;
    in.ekf_calibrated = EKF_calibrated;
    /* P0-C：健康位全 0 且 EKF_Task 300ms 內有更新才視為 healthy（餓死防護） */
    in.ekf_healthy    = (EKF_GetHealthBits() == 0U &&
                         (now - EKF_GetLastUpdateTick()) <= 300U) ? 1U : 0U;
    /* P0-D：baro 路徑閘控 = sensor_health 的 BMP388 位 + pad 基準已建立 */
    in.sensor_bits    = (((sens_bits & SH_BIT_BMP388) == 0U) && pad_ref_valid)
                        ? 0U : FSM_SB_BARO_FAULT;

    FSM_Action_t act = FSM_Step(&g_fsm_ctx, &in);

    /* --- 1. 硬體動作（先於任何列印） --- */
#if (IS_BACKUP && FEATURE_LINK)
    /* 備援航電（不對稱冗餘）：自身 FSM 照常運算並推進狀態，但點火/部署輸出受閘控——
     * 已收到主板開傘通知（鎖存）即抑制（共用點火頭，主板已點）；未收到則等
     * BACKUP_GRACE_MS 後自行補點（主板漏點 / 失聯 / 死亡）。release/buzzer 照常輸出
     * （自身未點火時 release 為無動作）。 */
    {
        static BackupGate_t s_drogue_gate, s_main_gate;
        static uint8_t s_gate_init = 0U;
        if (!s_gate_init) {
            BackupGate_Init(&s_drogue_gate);
            BackupGate_Init(&s_main_gate);
            s_gate_init = 1U;
        }
        const LinkPeer_t *peer = Link_GetPeer();

        if (BackupGate_Step(&s_drogue_gate, act.fire_drogue, peer->drogue_latched, now, BACKUP_GRACE_MS)) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);    // 補點副傘引爆 MOSFET（主板未開傘）
        }
        if (act.release_drogue) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);  // 自身點火限時導通保護：斷開
        }
        if (BackupGate_Step(&s_main_gate, act.deploy_main, peer->main_latched, now, BACKUP_GRACE_MS)) {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 2000);     // 補部署主傘舵機（主板未部署）
        }
    }
#else
    if (act.fire_drogue) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);    // 導通副傘引爆 MOSFET
    }
    if (act.release_drogue) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);  // 點火限時導通保護：斷開
    }
    if (act.deploy_main) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 2000);     // 主傘釋放舵機（釋放角度）
    }
#endif

#if FEATURE_UPLINK_DEPLOY
    /* 上行手動開傘（地面站 433 命令，uplink_cmd 已經 ARM→DEPLOY 兩段式驗證）。
     * 與 FSM 自動點火並存：直接驅動點火輸出並鎖存 drogue_fired（防 FSM 二次點火）；
     * 下行 DROGUE_FIRED/MAIN_DEPLOYED 旗標（由 PD13/TIM4 實際狀態推導）即為地面站確認。 */
    {
        static uint8_t  s_man_drogue_active = 0U;
        static uint32_t s_man_drogue_tick   = 0U;
        uint8_t man_drogue = 0U, man_main = 0U;
        if (UplinkCmd_TakeDeploy(&man_drogue, &man_main)) {
            if (man_drogue) {
                HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);   // 手動副傘引爆 MOSFET
                g_fsm_ctx.drogue_fired = 1U;
                s_man_drogue_active = 1U;
                s_man_drogue_tick   = now;
            }
            if (man_main) {
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 2000);          // 手動主傘舵機釋放角度
            }
        }
        /* 手動副傘導通限時保護：FSM_PYRO_HOLD_MS 後斷開 MOSFET（同 FSM 自身點火限時）。 */
        if (s_man_drogue_active && (now - s_man_drogue_tick) >= FSM_PYRO_HOLD_MS) {
            HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);
            s_man_drogue_active = 0U;
        }
    }
#endif

    if (act.start_buzzer) {
        // 開啟板載尋標蜂鳴器（持續鳴叫，利於落點尋標）
        htim2.Instance->ARR  = 999;
        htim2.Instance->CCR1 = 500;
        htim2.Instance->EGR  = TIM_EGR_UG;
    }

    /* --- 2. 鏡射回全域（telemetry / flash ring / SD / EKF 既有讀取點不變） --- */
    current_fsm_state = g_fsm_ctx.state;
    flight_start_tick = g_fsm_ctx.flight_start_ms;
    if (act.event == FSM_EVT_TOUCHDOWN) {
        g_touchdown_tick = now;   // 記錄落地時刻：SD 記錄降為 10Hz，逾時或按 USER_BT1 後關檔
    }

    /* P0-E：飛行態（BOOST..MAIN_DEPLOY）禁止 Flash 同步滾動擦除（~400ms 阻塞） */
    FlashRing_SetEraseAllowed((g_fsm_ctx.state >= STATE_BOOST &&
                               g_fsm_ctx.state <= STATE_MAIN_DEPLOY) ? 0U : 1U);

    /* --- 3. 事件列印（訊息逐字保留自原 FSM_Update） --- */
    switch ((FSM_Event_t)act.event) {
        case FSM_EVT_LIFTOFF:
            printf("[FSM] [LIFTOFF] LIFTOFF DETECTED! Transition to STATE_BOOST.\r\n");
            break;
        case FSM_EVT_BURNOUT:
            printf("[FSM] [BURNOUT] MOTOR BURNOUT! Entering STATE_COAST.\r\n");
            break;
        case FSM_EVT_APOGEE:
            printf("[FSM] [APOGEE] DYNAMIC APOGEE DETECTED! Expected apogee in %.2f s (h_est: %.2f m). Deploying DROGUE.\r\n",
                   act.apogee_t_pred, in.h_est);
            break;
        case FSM_EVT_APOGEE_FAILSAFE:
            g_fsm_failsafe_fired = 1U;
            printf("[FSM] [FAILSAFE] FAILSAFE APOGEE TIMEOUT (%lu ms)! Forcing DROGUE deployment (h_est: %.2f m, baro_rel: %.2f m).\r\n",
                   (unsigned long)FSM_FAILSAFE_APOGEE_MS, in.h_est, in.baro_alt_rel);
            break;
        case FSM_EVT_DROGUE_DONE:
            printf("[FSM] [DROGUE] Drogue deployed successfully. Entering STATE_DESCENT.\r\n");
            break;
        case FSM_EVT_MAIN_DEPLOY:
            printf("[FSM] [MAIN] DYNAMIC LOW ALTITUDE REACHED! Trigger H: %.2f m (Target H: %.2f m, Fall V: %.2f m/s). Deploying MAIN.\r\n",
                   in.h_est, TARGET_MAIN_ALTITUDE, in.v_est);
            break;
        case FSM_EVT_MAIN_OPEN:
            printf("[FSM] Main deployed. Entering landing detection.\r\n");
            break;
        case FSM_EVT_TOUCHDOWN:
            printf("[FSM] [TOUCHDOWN] TOUCHDOWN! 持續記錄中（降頻 10Hz）。按 USER_BT1 或逾時後自動關檔。\r\n");
            break;
        default:
            break;
    }
}

#if FEATURE_LINK
/* === 板間鏈路：自身狀態組裝 + 週期廣播 === */
static void Link_BuildOwnStatus(LinkStatus_t *ls)
{
    EKF_State_t e = EKF_GetState();
    ls->board_id  = IS_BACKUP ? LINK_BOARD_BACKUP : LINK_BOARD_PRIMARY;
    ls->seq       = 0U;   /* 由 Link_SendStatus 補遞增序號 */
    ls->fsm_state = (uint8_t)current_fsm_state;

    /* flags 與下行遙測同語意（TELEM_FLAG_*）；備板據主板 DROGUE_FIRED/MAIN_DEPLOYED 抑制 */
    uint8_t flags = 0U;
    if (HAL_GPIO_ReadPin(FIRE_GPIO_Port, FIRE_Pin) == GPIO_PIN_SET)   flags |= TELEM_FLAG_DROGUE_FIRED;
    if (__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3) >= 2000)         flags |= TELEM_FLAG_MAIN_DEPLOYED;
    if (g_fsm_failsafe_fired)                                         flags |= TELEM_FLAG_FAILSAFE;
    if (EKF_GetHealthBits() != 0U)                                    flags |= TELEM_FLAG_EKF_UNHEALTHY;
    if (g_sensor_fault_bits != 0U)                                    flags |= TELEM_FLAG_SENSOR_FAULT;
    ls->flags = flags;

    ls->tick_ms     = HAL_GetTick();
    ls->h_est_cm    = (int32_t)(e.pos_z * 100.0f);
    ls->v_est_cms   = (int32_t)(e.vel_z * 100.0f);
    ls->baro_alt_cm = (int32_t)(baro_data.altitude * 100.0f);
    ls->a_z_cg      = (int16_t)(highg_data.az * 100.0f);
}

/* 飛控迴圈每 LINK_TX_PERIOD_MS 呼叫一次：廣播自身狀態給對端板。 */
static void Link_PublishTick(void)
{
    LinkStatus_t ls;
    Link_BuildOwnStatus(&ls);
    Link_SendStatus(&ls);
}
#endif /* FEATURE_LINK */

/* === HAL UART 回呼統一分派（單一定義；gps.c 已改為 GPS_Handle* 由此轉接） ===
 * USART6 → GPS（FEATURE_GPS）；USART2 → 板間鏈路（FEATURE_LINK）。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
#if FEATURE_GPS
    GPS_HandleRxEvent(huart, Size);
#endif
#if FEATURE_LINK
    if (huart->Instance == USART2) Link_OnRxEvent(Size);
#endif
#if IS_GROUND
    if (huart->Instance == USART3) GroundStation_OnUart3RxEvent(Size);  /* E22 433 透傳 RX */
    if (huart->Instance == USART2) GsLoraTest_OnUart2RxEvent(Size);     /* 通訊測試命令介面 */
#endif
#if FEATURE_UPLINK_DEPLOY
    if (huart->Instance == USART3) UplinkCmd_OnUart3RxEvent(Size);      /* 主航電：433 上行命令 RX */
#endif
#if (!FEATURE_GPS && !FEATURE_LINK && !IS_GROUND)
    (void)huart; (void)Size;
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
#if FEATURE_GPS
    GPS_HandleUartError(huart);
#endif
#if FEATURE_LINK
    if (huart->Instance == USART2) Link_OnError();
#endif
#if (!FEATURE_GPS && !FEATURE_LINK)
    (void)huart;
#endif
}

#if FEATURE_LINK
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) Link_OnTxComplete();
}
#endif

uint16_t ADC_Read_Battery_mv(void) {
    uint32_t val = 0;
    HAL_ADC_Start(&hadc1);
    /* P0-E：逾時 10ms→2ms。ADC1 12-bit 轉換僅 ~µs 級，2ms 已極寬裕；
     * 10ms 逾時在 ADC 故障時會凍結 1kHz 飛控迴圈 10 個週期。 */
    if (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK) {
        val = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return (uint16_t)((val * 3300UL * 11UL) / 4095UL);
}

/* ===== SPI3 共用匯流排互斥鎖包裝（W25Q128 Flash + E80 920MHz LoRa，見 spi3_bus.h） =====
 * NULL-guard：mutex 建立前（scheduler 啟動前的單執行緒初始化階段）為 no-op。
 * 非持有者釋放：遞迴 mutex 於非持有者呼叫 osMutexRelease 會安全回傳錯誤（忽略），
 * 故 flash 驅動 CS 巨集中偶發的防禦性 CS_HIGH 不會誤釋放他人持有的鎖。 */
void SPI3_Bus_Lock(void)
{
    if (g_spi3_mutex != NULL) {
        osMutexAcquire(g_spi3_mutex, osWaitForever);
    }
}

void SPI3_Bus_Unlock(void)
{
    if (g_spi3_mutex != NULL) {
        osMutexRelease(g_spi3_mutex);
    }
}

/* ===== LoRa 下行遙測任務 =====
 * 每 LORA_TELEM_PERIOD_MS 打包一筆 TelemetryPacket_t，分別經 E22(433) 與 E80(920) 發送。
 * 兩條鏈路各自靠 AUX/BUSY + TxDone 背壓自我限流（忙線即跳過），互不阻塞。
 * 低優先任務：blocking UART 傳輸會被高優先飛控任務搶佔，完全不影響 1kHz 迴圈時序。 */
#define LORA_TELEM_PERIOD_MS  200U   /* 5Hz 嘗試率（實際受空中速率限制，可於此調整） */
#define UPLINK_LISTEN_EVERY   10U    /* 每 N 個 433 時槽空出 1 槽不發射，留給地面站上行 */
void LoRaTelemetry_Task(void *argument)
{
    (void)argument;
    static uint8_t tx_buf[TELEM_PACKET_SIZE];
    uint32_t slot = 0;   /* 433 時槽計數（用於空出上行接收窗） */

    /* 讓開機自檢 / SD 掛載 / FlashRing 初始化先跑一段再開始發送（非必要，SPI3 mutex 已保護）。 */
    osDelay(2000);

    uint32_t wake = osKernelGetTickCount();
#if LORA433_ENABLE
    uint32_t e22_retry_tick = 0;
#endif
    for (;;) {
        uint16_t len = Telemetry_Build(tx_buf);

#if LORA433_ENABLE
        /* P1：E22 初始化失敗時每 10s 重試一次（本任務最低優先，~305ms 阻塞無妨）。
         * LORA433_ENABLE=0 時整段編譯移除——停用後不重試、不復活 E22。 */
        if (!lora433_ok && (HAL_GetTick() - e22_retry_tick) >= 10000U) {
            e22_retry_tick = HAL_GetTick();
            if (LoRaE22_Init(&huart3) == HAL_OK) {
                lora433_ok = 1;
                printf("[LORA] E22 433MHz 重試成功，恢復下行。\r\n");
            }
        }
#endif

        HAL_StatusTypeDef st433 = HAL_BUSY;
        uint8_t tried433 = 0;
#if FEATURE_UPLINK_DEPLOY
        /* 上行接收窗：每 UPLINK_LISTEN_EVERY 槽空出 1 槽不發 433（E22 透傳此時即在接收），
         * 讓地面站的上行命令有空中無干擾時段可被收到。920 下行不受影響、維持全速率。 */
        uint8_t listen_slot = ((slot % UPLINK_LISTEN_EVERY) == (UPLINK_LISTEN_EVERY - 1U));
        if (lora433_ok && !listen_slot) {
            st433 = LoRaE22_Send(tx_buf, len);
            tried433 = 1;
        }
        UplinkCmd_Poll(HAL_GetTick());            /* 解析上行、處理 ARM/DEPLOY、ARM 逾時 */
#else
        if (lora433_ok) {
            st433 = LoRaE22_Send(tx_buf, len);    /* 透傳：HAL_OK=已推入 UART→E22 */
            tried433 = 1;
        }
#endif
        /* 433 下行發送狀態 log（確認 MCU 確實把封包推給 E22 + AUX 腳狀態）。
         * AUX：HIGH=E22 空閒可收發；持續 LOW=E22 忙/異常。每 25 槽(~5s)印一次。 */
        {
            static uint32_t s_433_try = 0, s_433_ok = 0;
            if (tried433) { s_433_try++; if (st433 == HAL_OK) s_433_ok++; }
            if (slot % 25U == 0U) {
                printf("[LORA_TX_LOG] 433MHz tx ok=%lu/try=%lu len=%u AUX=%d last=%s\r\n",
                       (unsigned long)s_433_ok, (unsigned long)s_433_try, (unsigned)len,
                       (int)HAL_GPIO_ReadPin(LORA433_BUSY_GPIO_Port, LORA433_BUSY_Pin),
                       tried433 ? ((st433 == HAL_OK) ? "OK" : ((st433 == HAL_BUSY) ? "BUSY" : "ERR"))
                                : "skip(listen/off)");
            }
        }
        if (lora920_ok) {
            HAL_StatusTypeDef st920 = LoRaE80_Send(tx_buf, (uint8_t)len);
            static uint32_t s_920_tx_cnt = 0;
            if (st920 == HAL_OK) s_920_tx_cnt++;
            if (slot % 25 == 0) {
                printf("[LORA_TX_LOG] 920MHz Sent Pkts: %lu (Last Status: %s)\r\n",
                       (unsigned long)s_920_tx_cnt,
                       (st920 == HAL_OK) ? "OK" : ((st920 == HAL_BUSY) ? "BUSY" : "ERR"));
            }
        }

        slot++;
        wake += LORA_TELEM_PERIOD_MS;
        osDelayUntil(wake);
    }
}

#if defined(ENABLE_DIAGNOSTICS) && !IS_GROUND
void Poll_Serial_Commands(void);
void Parse_Serial_Command(const char* cmd);

void StartDiagnosticTask(void *argument)
{
  (void)argument;
  uint32_t tick = 0;
  uint32_t wake_tick = osKernelGetTickCount();
  for (;;)
  {
#if !FEATURE_LINK
      Poll_Serial_Commands();   /* USART2 文字命令台；FEATURE_LINK 時 USART2 改作板間鏈路 */
#endif

      if (tick % 50 == 0) {
          // 1. GPS 遙測
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

          // 2. 地磁計遙測
          if (mag_ok) {
              const MMC5983_Data_t *m = MMC5983_GetData();
              // 重映射至 body frame (sensor_axis.h)：mag X->-Y, Y->-X, Z->-Z
              float mx_body, my_body, mz_body;
              sensor_mag_to_body(m->gauss[0], m->gauss[1], m->gauss[2], &mx_body, &my_body, &mz_body); 
              float hdg_body = atan2f(-mx_body, my_body) * (180.0f / 3.14159265f);
              if (hdg_body < 0.0f) hdg_body += 360.0f;
              printf("[MAG] B[mG]:%d,%d,%d hdg:%d ok:%lu err:%lu\r\n",
                     (int)(mx_body * 1000.0f),
                     (int)(my_body * 1000.0f),
                     (int)(mz_body * 1000.0f),
                     (int)hdg_body,
                     (unsigned long)m->reads_ok,
                     (unsigned long)m->reads_err);
          }

#ifdef RATE_MONITOR_ENABLE
          // 3. 採樣率遙測
          float r_bmi_a = g_sampling_rate.bmi088_acc.rate_hz;
          float r_bmi_g = g_sampling_rate.bmi088_gyro.rate_hz;
          float r_adxl  = g_sampling_rate.adxl375.rate_hz;
          float r_bmp   = g_sampling_rate.bmp388.rate_hz;
          float r_mmc   = g_sampling_rate.mmc5983.rate_hz;
          float r_gps   = g_sampling_rate.gps.rate_hz;

          printf("[RATE] BMI088_A:%d.%02uHz, BMI088_G:%d.%02uHz, ADXL375:%d.%02uHz, BMP388:%d.%02uHz, MMC5983:%d.%02uHz, GPS:%d.%02uHz, SD_DET:%d, EKF_DROP:%lu\r\n",
                 (int)r_bmi_a, (unsigned int)((r_bmi_a - (int)r_bmi_a) * 100.0f),
                 (int)r_bmi_g, (unsigned int)((r_bmi_g - (int)r_bmi_g) * 100.0f),
                 (int)r_adxl,  (unsigned int)((r_adxl  - (int)r_adxl)  * 100.0f),
                 (int)r_bmp,   (unsigned int)((r_bmp   - (int)r_bmp)   * 100.0f),
                 (int)r_mmc,   (unsigned int)((r_mmc   - (int)r_mmc)   * 100.0f),
                 (int)r_gps,   (unsigned int)((r_gps   - (int)r_gps)   * 100.0f),
                 (int)HAL_GPIO_ReadPin(SDIO_DET_GPIO_Port, SDIO_DET_Pin),
                 (unsigned long)g_ekf_queue_drops);

          // 4. W25Q128 Flash 記錄遙測
          printf("[FLASH_RING] PKT_TOTAL:%lu ADDR:0x%06lX\r\n",
                 FlashRing_GetPacketCount(), FlashRing_GetWriteAddr());

          // 5. CPU 佔用率遙測
          float r_ekf_cpu = EKF_GetCPUUsage();
          printf("[CPU] MainTask+ISR:%d.%02u%%, EKFTask:%d.%02u%%\r\n",
                 (int)g_main_task_cpu_usage, (unsigned int)((g_main_task_cpu_usage - (int)g_main_task_cpu_usage) * 100.0f),
                 (int)r_ekf_cpu, (unsigned int)((r_ekf_cpu - (int)r_ekf_cpu) * 100.0f));
#endif

          // 6. IMU & High-G 原始數據用於方向對齊驗證
          printf("[IMU] a[mG]:%d,%d,%d g[dps]:%d,%d,%d\r\n",
                 (int)(imu_data.ax * 1000.0f),
                 (int)(imu_data.ay * 1000.0f),
                 (int)(imu_data.az * 1000.0f),
                 (int)(imu_data.gx),
                 (int)(imu_data.gy),
                 (int)(imu_data.gz));
          if (adxl375_ok) {
              printf("[HIGHG] a[mG]:%d,%d,%d\r\n",
                     (int)(highg_data.ax * 1000.0f),
                     (int)(highg_data.ay * 1000.0f),
                     (int)(highg_data.az * 1000.0f));
          }

          // 7. 電源監測遙測（REQ-SYS-01：板載 ADC 電池電壓採樣 + 遙測顯示）
          printf("[PWR] bat:%dmV\r\n", (int)g_bat_voltage_mv);

          // 8. LoRa 下行鏈路就緒狀態（433 透傳 / 920 SX126x）+ E80 init 診斷
          {
              int    e80_rd; uint8_t e80_busy, e80_rb0, e80_rb1, e80_gs;
              LoRaE80_GetInitDiag(&e80_rd, &e80_busy, &e80_rb0, &e80_rb1, &e80_gs);
              /* gs=0x22 → SX126x STBY_RC 正常; gs=0x00 → MISO接地; gs=0xFF → MISO浮空 */
              printf("[LORA] 433:%s 920:%s | gs=0x%02X busy=%d rd=%d sw=0x%02X,0x%02X\r\n",
                     lora433_ok ? "rdy" : "off",
                     lora920_ok ? "rdy" : "off",
                     e80_gs, (int)e80_busy, e80_rd, e80_rb0, e80_rb1);
          }
      }

      tick++;
      wake_tick += 20U;
      osDelayUntil(wake_tick);
  }
}
#endif

#define CMD_BUFFER_SIZE 64
static char g_cmd_buf[CMD_BUFFER_SIZE];
static uint8_t g_cmd_idx = 0;

void Parse_Serial_Command(const char* cmd) {
    if (strncmp(cmd, "CMD_MAG_CAL:", 12) == 0) {
        float cx = 0.0f, cy = 0.0f, cz = 0.0f;
        if (sscanf(cmd + 12, "%f,%f,%f", &cx, &cy, &cz) == 3) {
            EKF_SaveMagCalibration(cx, cy, cz);
        } else {
            printf("[CAL] ERROR: Invalid mag cal command format\r\n");
        }
    }
    else if (strncmp(cmd, "CMD_MAG_YAW_LOCK:", 17) == 0) {
        int val = 0;
        if (sscanf(cmd + 17, "%d", &val) == 1) {
            g_mag_yaw_lock = (val != 0) ? 1 : 0;
            printf("[CAL] EKF Mag Yaw Lock set to: %d\r\n", g_mag_yaw_lock);
        }
    }
    else if (strcmp(cmd, "CMD_RESET_CAL") == 0) {
        EKF_ResetCalibration();
    }
}

void Poll_Serial_Commands(void) {
    uint8_t rx_byte;
    /* 每次最多處理 CMD_BUFFER_SIZE 個 byte（上限保護）：UART 轉接頭拔掉時 RX 即使有殘餘
     * 雜訊，也不會在此 while 無限狂讀、霸佔 CPU 而餓死餵狗任務（PA3 已改上拉為主要防線，
     * 此為第二道防線）。 */
    for (int guard = 0; guard < CMD_BUFFER_SIZE; guard++) {
        if (HAL_UART_Receive(&huart2, &rx_byte, 1, 0) != HAL_OK) break;
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (g_cmd_idx > 0) {
                g_cmd_buf[g_cmd_idx] = '\0';
                Parse_Serial_Command(g_cmd_buf);
                g_cmd_idx = 0;
            }
        } else if (g_cmd_idx < CMD_BUFFER_SIZE - 1) {
            g_cmd_buf[g_cmd_idx++] = (char)rx_byte;
        } else {
            g_cmd_idx = 0; // overflow reset
        }
    }
    /* 清 USART2 浮動/雜訊造成的 ORE/FE/NE/PE（讀 SR 後讀 DR），避免錯誤旗標卡住接收 */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    (void)huart2.Instance->SR;
    (void)huart2.Instance->DR;
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
#if IS_GROUND
  GroundStation_Run();   /* 地面站主迴圈（接收 LoRa 433+920 + GPS → 對齊時間戳 → SD/Flash/USB），不返回 */
#endif
  uint32_t tick = 0;

  printf("\r\n============================================================\r\n");
  printf("[ROLE] %s  [LORA RADIO PARAMETERS]\r\n", IS_PRIMARY ? "PRIMARY_AV (主航電)" : "BACKUP_AV (備援航電)");
  printf("[LORA 433MHz] E22-400T30S | Freq: 432.000 MHz (CH=22) | Baud: 115200 bps\r\n");
  LoRaE80_PrintConfig();
  printf("============================================================\r\n\r\n");

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
      SPI3_Bus_Lock();          /* 整段 dump 持 SPI3 鎖，保證輸出不被 E80 遙測交錯 */
      Flash_DumpAll();          /* P2：已併入 w25qxx.c，SPI handle 由驅動內部綁定 */
      SPI3_Bus_Unlock();
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

#if FEATURE_FLASH
  /* === Flash Ring Buffer 初始化（SPI3，不影響 SPI1 感測器中斷） === */
  FlashRing_Init();
  FlashRingPacket_t last_pkt;
  uint8_t pkt_valid = (FlashRing_GetLastPacket(&last_pkt) == W25QXX_OK) ? 1U : 0U;
#else
  uint8_t pkt_valid = 0;
  FlashRingPacket_t last_pkt = {0};
#endif

  /* 取當下首筆 baro 供高度連續性檢查（與主迴圈相同的 TIM3 遮蔽保護） */
  __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
  BMP388_ReadData(&hspi1, &baro_data);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);

  FSM_HotStartDecision_t hs = FSM_HotStartDecide(
      pkt_valid,
      pkt_valid ? last_pkt.fsm_state : 0U,
      pkt_valid ? last_pkt.tick_ms : 0U,
      pkt_valid ? ((float)last_pkt.baro_alt_cm / 100.0f) : 0.0f,
      baro_data.altitude,
      (pkt_valid && (last_pkt.flags & 0x01U)) ? 1U : 0U);

  if (hs.restore) {
      g_hotstart_restored = 1U;   /* 遙測 TELEM_FLAG_HOTSTART（地面站需特別標示） */
      EKF_in_flight = 1;          /* EKF 跳過發射台 ZUPT / launch 偵測 */
      current_fsm_state = hs.state;
      flight_start_tick = HAL_GetTick() - last_pkt.tick_ms;  /* 起飛基準（unsigned 迴繞安全） */
      FSM_Init(&g_fsm_ctx, hs.state, HAL_GetTick(), flight_start_tick, hs.drogue_fired);

      /* 注入空中 EKF 狀態（封包 EKF 高度/垂直速度/四元數）。
       * 此刻 EKF_Task 阻塞於空佇列（主迴圈尚未饋入 buffer），寫入安全。 */
      float hs_q[4] = { last_pkt.ekf_q0, last_pkt.ekf_q1, last_pkt.ekf_q2, last_pkt.ekf_q3 };
      EKF_HotRestartRestore((float)last_pkt.ekf_pos_z_cm / 100.0f,
                            (float)last_pkt.ekf_vel_z_cms / 100.0f, hs_q);

      printf("[FSM] [WARNING] HOT-RESTART：恢復狀態 %d（drogue_fired=%u），飛行 tick %lu ms\r\n",
             (int)hs.state, hs.drogue_fired, (unsigned long)last_pkt.tick_ms);
  } else {
      current_fsm_state = STATE_PAD; // 正常地面起飛（或驗證未過回 PAD 重校準）
      FSM_Init(&g_fsm_ctx, STATE_PAD, HAL_GetTick(), 0U, 0U);
      if (pkt_valid &&
          last_pkt.fsm_state >= STATE_BOOST && last_pkt.fsm_state <= STATE_DESCENT) {
          printf("[FSM] 偵測到飛行中封包但熱啟動驗證未過（tick=%lu ms），回 PAD 重校準。\r\n",
                 (unsigned long)last_pkt.tick_ms);
      }
  }

  /* FlashRing_Init 阻塞約 2s，BMP388（主迴圈讀取）在此期間無法累積計數。
   * 重置所有採樣率監測器，確保第一個 [RATE] 報告反映穩態採樣率。 */
  SAMPLING_RATE_INIT();
  HAL_IWDG_Refresh(&hiwdg);

  /* Infinite loop */
  uint32_t wake_tick = osKernelGetTickCount();
  static uint32_t main_task_accumulated_cycles = 0;
  static uint32_t main_task_last_calc_tick = 0;
  DWT_Init(); // Ensure DWT is initialized
  for(;;)
  {
    uint32_t loop_start_cycles = DWT->CYCCNT;
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

            // 重映射至 body frame (X=右, Y=前, Z=上)，依 sensor_axis.h（唯一真相來源）。
            // 輸入輸出可同變數：函式先以傳值複製 sx/sy/sz 再寫出，alias 安全。
            sensor_imu_to_body(ax, ay, az, &ax, &ay, &az);

            if (adxl_hiG_avail) {
                float a_mag_sq = ax*ax + ay*ay + az*az;
                if (a_mag_sq > (20.0f * 9.80665f) * (20.0f * 9.80665f)) {  /* > 20g, 避免 sqrt */
                    int adxl_i = (i * 16 + 2) / 5;
                    if (adxl_i > ADXL_BATCH_SIZE - 1) adxl_i = ADXL_BATCH_SIZE - 1;
                    // ADXL375 重映射至同一 body frame，依 sensor_axis.h，與 BMI088 對齊以無縫替換。
                    float hg_sx = g_adxl_batches[adxl_hiG_bidx][adxl_i].ax * 9.80665f;
                    float hg_sy = g_adxl_batches[adxl_hiG_bidx][adxl_i].ay * 9.80665f;
                    float hg_sz = g_adxl_batches[adxl_hiG_bidx][adxl_i].az * 9.80665f;
                    sensor_highg_to_body(hg_sx, hg_sy, hg_sz, &ax, &ay, &az);
                }
            }

            // 2. Gyro Decimation (20 samples over 10ms, spaced at 0.5 ms)
            // Time match is exact: 2 * i * 0.5 ms = i * 1.0 ms
            int gyro_idx = 2 * i;
            if (gyro_idx > 19) gyro_idx = 19;

            float sgx = g_bmi_gyro_batches[gyro_ready_idx][gyro_idx].gx;
            float sgy = g_bmi_gyro_batches[gyro_ready_idx][gyro_idx].gy;
            float sgz = g_bmi_gyro_batches[gyro_ready_idx][gyro_idx].gz;

            // body 角速度 (rad/s)，與 accel 同一映射 (sensor_axis.h)。陀螺為 axial vector，
            // 在 proper rotation (det=+1) 下與一般向量同變換，故直接套用 IMU 映射。
            float gx, gy, gz;
            sensor_imu_to_body(sgx * (M_PI / 180.0f), sgy * (M_PI / 180.0f), sgz * (M_PI / 180.0f),
                               &gx, &gy, &gz);

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
            // P0-D：BMP388 故障（卡死/失流/範圍）時不再餵入 EKF —— 觸發
            // EKF_HB_BARO_TIMEOUT → FSM 降級鏈，而非讓 EKF 吃陳舊/壞值。
            if ((i == 0 || i == 5) && bmp388_ok &&
                (g_sensor_fault_bits & SH_BIT_BMP388) == 0U) {
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
        
        // 重映射至 body frame (sensor_axis.h)；與 EKF 饋入完全一致。
        float raw_ax         = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].ax;
        float raw_ay         = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].ay;
        float raw_az         = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].az;
        sensor_imu_to_body(raw_ax, raw_ay, raw_az, &imu_data.ax, &imu_data.ay, &imu_data.az);

        imu_data.gyro_x_raw  = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gyro_x_raw;
        imu_data.gyro_y_raw  = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gyro_y_raw;
        imu_data.gyro_z_raw  = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gyro_z_raw;
        
        float raw_gx         = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gx;
        float raw_gy         = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gy;
        float raw_gz         = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gz;
        sensor_imu_to_body(raw_gx, raw_gy, raw_gz, &imu_data.gx, &imu_data.gy, &imu_data.gz);

        /* P0-D：BMI088 健康餵入（accel+gyro raw 簽章；卡死/失流偵測） */
        sensor_mon_feed(&g_mon_bmi088, HAL_GetTick(),
                        sensor_sig3(imu_data.accel_x_raw,
                                    imu_data.accel_y_raw,
                                    imu_data.accel_z_raw) ^
                        sensor_sig3(imu_data.gyro_x_raw,
                                    imu_data.gyro_y_raw,
                                    imu_data.gyro_z_raw),
                        1U /* BMI088 無範圍檢查 */);
    }

    /* === ADXL375 Ping-Pong 雙緩衝區批次數據提取 (3.2 kHz 中斷在背後默默採樣並填充) === */
    if (adxl375_ok && g_adxl_new_batch_ready) {
        g_adxl_new_batch_ready = 0;
        uint8_t ready_idx = g_adxl_ready_batch_idx;
        
        // 自 32 筆最新採樣數據中，提取最後一筆最新值供即時遙測傳送與顯示
        highg_data.x_raw = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].x_raw;
        highg_data.y_raw = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].y_raw;
        highg_data.z_raw = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].z_raw;
        
        float raw_adxl_ax = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].ax;
        float raw_adxl_ay = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].ay;
        float raw_adxl_az = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].az;
        sensor_highg_to_body(raw_adxl_ax, raw_adxl_ay, raw_adxl_az,
                             &highg_data.ax, &highg_data.ay, &highg_data.az);

        /* P0-D：ADXL375 健康餵入（±200g 量程，>250g 持續 0.5s 視為範圍失效） */
        {
            uint8_t adxl_range_ok = (fabsf(highg_data.ax) <= 250.0f &&
                                     fabsf(highg_data.ay) <= 250.0f &&
                                     fabsf(highg_data.az) <= 250.0f) ? 1U : 0U;
            sensor_mon_feed(&g_mon_adxl375, HAL_GetTick(),
                            sensor_sig3(highg_data.x_raw,
                                        highg_data.y_raw,
                                        highg_data.z_raw),
                            adxl_range_ok);
        }
        
        // 提示：如需寫入 SD 卡，可在此處直接寫入整個 g_adxl_batches[ready_idx] 共 32 筆資料，即為 3.2 kHz HIL 實時存檔
    }

    /* === FSM 100Hz 定時執行（P0-A：自 BMI088 批次條件塊移出） ===
     * 舊版在 if (bmi088_ok && batch_ready) 內呼叫 → BMI088 初始化失敗或飛行中
     * 批次斷流時，FSM（含後續 P0-B 失效保護計時器）會整個停擺。
     * 改為無條件 tick 定時：感測器斷流時 FSM 仍持續運作（吃最後快照 + EKF 狀態）。 */
    if (tick % FSM_STEP_PERIOD_MS == 0) {
        FSM_Update();
    }

#if FEATURE_LINK
    /* 板間鏈路：每 LINK_TX_PERIOD_MS 廣播自身狀態（主板開傘通知據此抑制備板）。 */
    if (tick % LINK_TX_PERIOD_MS == 0) {
        Link_PublishTick();
    }
#endif

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

    /* === MMC5983MA 地磁計 @ 100 Hz 軟體 I2C 讀取 (每 10ms 一次讀取暫存器) === */
    if (mag_ok && (tick % 10 == 0)) {
        MMC5983_Read_Continuous();
    }

    /* === MMC5983MA 地磁計 EKF 融合 @ 10 Hz (每 100ms 提交一次軟體濾波後的純淨磁場資料) === */
    if (mag_ok && (tick % 100 == 0)) {
        float mx = 0.0f, my = 0.0f, mz = 0.0f;
        MMC5983_GetFilteredGauss(&mx, &my, &mz);
        /* 重映射至 body frame (sensor_axis.h)：mag X->-Y, Y->-X, Z->-Z */
        float mx_body, my_body, mz_body;
        sensor_mag_to_body(mx, my, mz, &mx_body, &my_body, &mz_body);
        EKF_SubmitMag(mx_body, my_body, mz_body);
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

        /* P0-D：BMP388 健康餵入（氣壓 26–110 kPa、溫度 −40~85°C；
         * 簽章取整數 Pa —— 卡死的感測器回傳逐位元相同的浮點值） */
        {
            uint8_t bmp_range_ok = (baro_data.pressure    >= 26000.0f &&
                                    baro_data.pressure    <= 110000.0f &&
                                    baro_data.temperature >= -40.0f &&
                                    baro_data.temperature <=  85.0f) ? 1U : 0U;
            sensor_mon_feed(&g_mon_bmp388, HAL_GetTick(),
                            (int32_t)baro_data.pressure, bmp_range_ok);
        }
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
        /* Flash Ring Buffer：將目前感測器與估計數據打包為 80-byte 格式寫入 */
        FlashRingPacket_t ring_pkt = {0};
        ring_pkt.tick_ms     = HAL_GetTick();
        
        // 抓取系統開傘狀態旗標
        uint8_t sys_flags = 0;
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET) sys_flags |= 0x01; // 副傘已點火
        if (__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3) >= 2000) sys_flags |= 0x02; // 主傘已釋放
        ring_pkt.flags       = sys_flags;
        
        ring_pkt.bat_voltage_mv = g_bat_voltage_mv;   /* P0-E：改用 1Hz 快取（ADC 讀取移出 50Hz 寫入路徑） */
        
        ring_pkt.bmi_ax      = imu_data.accel_x_raw;
        ring_pkt.bmi_ay      = imu_data.accel_y_raw;
        ring_pkt.bmi_az      = imu_data.accel_z_raw;
        ring_pkt.bmi_gx      = imu_data.gyro_x_raw;
        ring_pkt.bmi_gy      = imu_data.gyro_y_raw;
        ring_pkt.bmi_gz      = imu_data.gyro_z_raw;
        
        ring_pkt.adxl_x      = highg_data.x_raw;
        ring_pkt.adxl_y      = highg_data.y_raw;
        ring_pkt.adxl_z      = highg_data.z_raw;
        
        ring_pkt.baro_temp_c_x100 = (int16_t)(baro_data.temperature * 100.0f);
        ring_pkt.baro_press_pa    = (uint32_t)(baro_data.pressure);
        ring_pkt.baro_alt_cm      = (int32_t)(baro_data.altitude * 100.0f);
        
        EKF_State_t sd_ekf = EKF_GetState();
        ring_pkt.ekf_pos_z_cm     = (int32_t)(sd_ekf.pos_z * 100.0f);
        ring_pkt.ekf_vel_z_cms    = (int32_t)(sd_ekf.vel_z * 100.0f);
        ring_pkt.ekf_q0           = sd_ekf.q[0];
        ring_pkt.ekf_q1           = sd_ekf.q[1];
        ring_pkt.ekf_q2           = sd_ekf.q[2];
        ring_pkt.ekf_q3           = sd_ekf.q[3];
        
        const GPS_Data_t *gps_pkt = GPS_GetData();
        if (gps_pkt->fix_valid) {
            ring_pkt.gps_lat      = gps_pkt->lat_1e6;
            ring_pkt.gps_lon      = gps_pkt->lon_1e6;
            ring_pkt.gps_alt_m    = (int16_t)gps_pkt->altitude_m;
            ring_pkt.gps_spd_cms  = (int16_t)(gps_pkt->speed_mps * 100.0f);
            ring_pkt.gps_sats     = gps_pkt->satellites;
            ring_pkt.gps_fix      = gps_pkt->fix_valid;
        }
        
        ring_pkt.fsm_state   = (uint8_t)current_fsm_state;
        
#if FEATURE_FLASH
        FlashRing_WritePacket(&ring_pkt);
#endif
    }

    /* === P0-E：1Hz 電池電壓 ADC 讀取（自 50Hz Flash 寫入路徑移出；唯一 ADC 讀取點） === */
    if (tick % 1000 == 250) {
        g_bat_voltage_mv = ADC_Read_Battery_mv();
    }

#if FEATURE_FLASH
    /* === P0-E：PAD/INIT 期背景預擦（每 1s 擦 1 個 sector，單次最壞 ~400ms < IWDG 2.05s）
     * 發射檢核表：上架後等待 [FLASH] pool 達 64 再允許起飛。 === */
    if (tick % 1000 == 500 && current_fsm_state <= STATE_PAD) {
        if (FlashRing_PreEraseOne() == 0U) {
            printf("[FLASH] pool=%lu/%u\r\n",
                   (unsigned long)FlashRing_GetPoolSectors(),
                   (unsigned)FLASH_RING_PREERASE_TARGET);
        }
    }
#endif

    /* === 每 100ms：10 Hz UART 遙測輸出 (減少 CPU 阻塞開銷)
     * P0-E：飛行態（BOOST..MAIN_DEPLOY）抑制 —— 每行 ~8.7ms 阻塞 UART，
     * SD/Flash/LoRa 記錄不受影響，USB 串口僅 bench 使用。 === */
    if (tick % 100 == 0 &&
        !(current_fsm_state >= STATE_BOOST && current_fsm_state <= STATE_MAIN_DEPLOY)) {
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



    /* === 每 500ms 翻轉 LED 一次，形成溫和的 1 Hz 視覺心跳 (0.5s 亮 / 0.5s 暗) === */
    if (tick % 500 == 0) {
        HAL_GPIO_TogglePin(LED_SYS_GPIO_Port, LED_SYS_Pin);
    }

    /* === P0-D：1Hz 健康診斷行（sens=SH_BIT_BMI088/ADXL375/BMP388、ekf=EKF_HB_*） === */
    if (tick % 1000 == 0) {
        printf("[HEALTH] sens=0x%02X ekf=0x%02X\r\n",
               (unsigned)g_sensor_fault_bits, (unsigned)EKF_GetHealthBits());
    }

    /* === P1：0.2Hz 任務堆疊水位（osThreadGetStackSpace = 歷史最低剩餘 bytes）。
     * 值逼近 0 = 溢位前兆；HIL 斷言每個任務 ≥256 bytes 餘裕。 === */
    if (tick % 5000 == 0) {
        printf("[STACK] main=%lu ekf=%lu lora=%lu\r\n",
               (unsigned long)(defaultTaskHandle ? osThreadGetStackSpace(defaultTaskHandle) : 0),
               (unsigned long)(g_ekf_task_handle ? osThreadGetStackSpace(g_ekf_task_handle) : 0),
               (unsigned long)(g_lora_task_handle ? osThreadGetStackSpace(g_lora_task_handle) : 0));
    }

    /* === 感測器狀態 LED（每 100ms 更新）===
     * STAT1 (PE3)：GPS 模組有在送 NMEA 訊號 (sentences_ok>0) 常亮；
     *              有有效定位 (fix_valid=1) 時進一步 2Hz 快閃（亮=有收星但無 fix，
     *              不閃常亮=有 fix，熄滅=沒有 NMEA 訊號）。
     *              用途：室內可確認 GPS 模組通訊正常，戶外可看到 fix 狀態。
     * STAT2 (PE4)：磁力計 I2C 初始化成功 (mag_ok=1) 則常亮，否則熄滅。 */
    if (tick % 100 == 0) {
        /* GPS STAT1：有 NMEA 訊號才亮；有 fix 常亮，無 fix 每 500ms 閃一次 */
        const GPS_Data_t *led_gps = GPS_GetData();
        if (led_gps->sentences_ok > 0) {
            if (led_gps->fix_valid) {
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   /* 常亮 = 有 fix */
            } else {
                /* 2Hz 閃爍 = 有 NMEA 但無 fix：以 tick/500 的奇偶決定亮滅 */
                GPIO_PinState blink = ((tick / 500) % 2 == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, blink);
            }
        } else {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);     /* 熄滅 = 無訊號 */
        }

        /* Mag STAT2：初始化成功即常亮（不需要讀取到資料才亮） */
        HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin,
                          mag_ok ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }



    uint32_t loop_end_cycles = DWT->CYCCNT;
    uint32_t loop_elapsed_cycles = loop_end_cycles - loop_start_cycles;

    /* P0-E：單迴圈最大耗時水位（HIL 斷言飛行態 < 20ms 的依據） */
    static uint32_t loop_max_cycles = 0;
    if (loop_elapsed_cycles > loop_max_cycles) {
        loop_max_cycles = loop_elapsed_cycles;
    }

    uint32_t current_tick = HAL_GetTick();
    if (main_task_last_calc_tick == 0) {
        main_task_last_calc_tick = current_tick;
    }
    main_task_accumulated_cycles += loop_elapsed_cycles;
    if (current_tick - main_task_last_calc_tick >= 1000) {
        uint32_t elapsed_ms_time = current_tick - main_task_last_calc_tick;
        uint32_t total_possible_cycles = elapsed_ms_time * (SystemCoreClock / 1000);
        if (total_possible_cycles > 0) {
            g_main_task_cpu_usage = (float)main_task_accumulated_cycles / (float)total_possible_cycles * 100.0f;
        }
        /* P0-E：1Hz 印出本秒最大單迴圈耗時（µs）並重置水位 */
        printf("[LOOP] max_us=%lu\r\n",
               (unsigned long)(loop_max_cycles / (SystemCoreClock / 1000000U)));
        loop_max_cycles = 0;
        main_task_accumulated_cycles = 0;
        main_task_last_calc_tick = current_tick;
    }

    tick++;
    wake_tick += 1U;
    osDelayUntil(wake_tick);
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fsm.h"   /* FlightState_t 與 FSM 參數（P0-A 抽離至純邏輯模組） */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* FlightState_t 已移至 fsm.h（host 可測純邏輯模組，P0-A） */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_SYS_Pin GPIO_PIN_2
#define LED_SYS_GPIO_Port GPIOE
#define LED_STAT2_Pin GPIO_PIN_4
#define LED_STAT2_GPIO_Port GPIOE
#define BAT_SENSE_Pin GPIO_PIN_0
#define BAT_SENSE_GPIO_Port GPIOC
#define USER_BT2_Pin GPIO_PIN_3
#define USER_BT2_GPIO_Port GPIOC
#define USER_BT1_Pin GPIO_PIN_1
#define USER_BT1_GPIO_Port GPIOA
#define CSB_BARO_Pin GPIO_PIN_4
#define CSB_BARO_GPIO_Port GPIOA
#define CSB_HIGHG_Pin GPIO_PIN_4
#define CSB_HIGHG_GPIO_Port GPIOC
#define INT_HIGHG2_Pin GPIO_PIN_5
#define INT_HIGHG2_GPIO_Port GPIOC
#define INT_HIGHG2_EXTI_IRQn EXTI9_5_IRQn
#define INT_HIGHG1_Pin GPIO_PIN_0
#define INT_HIGHG1_GPIO_Port GPIOB
#define INT_BARO_Pin GPIO_PIN_1
#define INT_BARO_GPIO_Port GPIOB
#define INT_ACCEL1_Pin GPIO_PIN_7
#define INT_ACCEL1_GPIO_Port GPIOE
#define INT_ACCEL1_EXTI_IRQn EXTI9_5_IRQn
#define INT_ACCEL2_Pin GPIO_PIN_8
#define INT_ACCEL2_GPIO_Port GPIOE
#define INT_ACCEL2_EXTI_IRQn EXTI9_5_IRQn
#define INT_GYRO1_Pin GPIO_PIN_9
#define INT_GYRO1_GPIO_Port GPIOE
#define INT_GYRO1_EXTI_IRQn EXTI9_5_IRQn
#define INT_GYRO2_Pin GPIO_PIN_10
#define INT_GYRO2_GPIO_Port GPIOE
#define INT_GYRO2_EXTI_IRQn EXTI15_10_IRQn
#define LORA433_BUSY_Pin GPIO_PIN_11
#define LORA433_BUSY_GPIO_Port GPIOE
#define INT_MAGN_Pin GPIO_PIN_12
#define INT_MAGN_GPIO_Port GPIOE
#define INT_MAGN_EXTI_IRQn EXTI15_10_IRQn
#define CSB_GYRO_Pin GPIO_PIN_11
#define CSB_GYRO_GPIO_Port GPIOB
#define CSB_ACCEL_Pin GPIO_PIN_12
#define CSB_ACCEL_GPIO_Port GPIOB
#define LORA433_M1_Pin GPIO_PIN_10
#define LORA433_M1_GPIO_Port GPIOD
#define LORA433_M0_Pin GPIO_PIN_11
#define LORA433_M0_GPIO_Port GPIOD
#define LORA433_RST_Pin GPIO_PIN_12
#define LORA433_RST_GPIO_Port GPIOD
#define FIRE_Pin GPIO_PIN_13
#define FIRE_GPIO_Port GPIOD
#define PWM_Servo_Pin GPIO_PIN_14
#define PWM_Servo_GPIO_Port GPIOD
#define RST_GPS_Pin GPIO_PIN_8
#define RST_GPS_GPIO_Port GPIOA
#define FLASH_CSB_Pin GPIO_PIN_15
#define FLASH_CSB_GPIO_Port GPIOA
#define SDIO_DET_Pin GPIO_PIN_3
#define SDIO_DET_GPIO_Port GPIOD
#define LORA920_INT_Pin GPIO_PIN_4
#define LORA920_INT_GPIO_Port GPIOD
#define LORA920_INT_EXTI_IRQn EXTI4_IRQn
#define LORA920_RST_Pin GPIO_PIN_5
#define LORA920_RST_GPIO_Port GPIOD
#define LORA920_BUSY_Pin GPIO_PIN_6
#define LORA920_BUSY_GPIO_Port GPIOD
#define LORA920_BUSY_EXTI_IRQn EXTI9_5_IRQn
#define CSB_LORA920_Pin GPIO_PIN_7
#define CSB_LORA920_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
/* TARGET_MAIN_ALTITUDE / MAIN_DEPLOY_DELAY_S / DROGUE_LEAD_TIME_S 已移至 fsm.h（P0-A） */
#define SD_LANDED_LOG_TIMEOUT_MS 1800000UL  // 落地後 SD 持續記錄上限 (30 分鐘)，逾時自動關檔
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

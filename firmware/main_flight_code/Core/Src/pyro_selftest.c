/*
 * pyro_selftest.c — 開傘地面點火自測（對稱：主/副兩板同一序列，重啟後跑一次）
 * 詳見 pyro_selftest.h 的序列 / 腳位 / 啟用 / 刪除說明。整檔以 FEATURE_PYRO_SELFTEST 包住。
 */
#include "pyro_selftest.h"

#if PYRO_SELFTEST_AVAILABLE

#include <stdio.h>
#include "main.h"        /* FIRE(PD13) / PWM_Servo(PD14) / LED_SYS(PE2) / LED_STAT2(PE4) 腳位 */
#include "fsm.h"         /* FSM_DROGUE_MOTOR_RUN_MS：副傘 DC 馬達導通時間（與飛行一致，8s） */

/* 由 main.c 建立的周邊 handle。 */
extern TIM_HandleTypeDef  htim2;   /* Buzzer   : TIM2 CH1 */
extern TIM_HandleTypeDef  htim4;   /* 開傘舵機 : TIM4 CH3（PD14） */
extern IWDG_HandleTypeDef hiwdg;   /* 看門狗約 2.05s 逾時 */

/* LED 腳位（皆 GPIOE）：State1=PE3 於 main.h 未命名，直接用腳號。 */
#define PYRO_LED_STATE1_Pin   GPIO_PIN_3

/* 依系統時鐘更新 SYS LED（1Hz 閃）。 */
static void sys_led_pump(void)
{
    uint8_t on = ((HAL_GetTick() / PYRO_SELFTEST_SYS_BLINK_MS) & 1U) != 0U;
    HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, on ? PYRO_LED_ON : PYRO_LED_OFF);
}

/* 分段延遲：續閃 SYS + 餵看門狗（IWDG≈2.05s，以 50ms 粒度）。 */
static void delay_fed(uint32_t ms)
{
    while (ms > 0U) {
        uint32_t chunk = (ms > 50U) ? 50U : ms;
        HAL_Delay(chunk);
        sys_led_pump();
        HAL_IWDG_Refresh(&hiwdg);
        ms -= chunk;
    }
}

static void servo_set_us(uint32_t us)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, us);
}

/* 開機 Buzzer：ARR 決定音高，CCR1=ARR/2 出聲、=0 靜音。 */
static void buzzer_beep(uint32_t ms)
{
    htim2.Instance->ARR  = PYRO_BUZZER_ARR;
    htim2.Instance->CCR1 = PYRO_BUZZER_ARR / 2U;
    htim2.Instance->EGR  = TIM_EGR_UG;
    delay_fed(ms);
    htim2.Instance->CCR1 = 0U;
}

void PyroSelfTest_RunSequence(void)
{
    printf("\r\n============================================================\r\n");
    printf("[PYRO-SELFTEST] 開傘電火自測（主/副同序列，每次重啟跑一次）\r\n");
    printf("[PYRO-SELFTEST] 序列：PD13 副傘馬達 %ums → 等 %ums → 舵機 0°→180°(停 %ums)→0°\r\n",
           (unsigned)FSM_DROGUE_MOTOR_RUN_MS, (unsigned)PYRO_SELFTEST_GAP_MS,
           (unsigned)PYRO_SELFTEST_SERVO_HOLD_MS);
    printf("[PYRO-SELFTEST] LED: SYS(PE2)閃 / State1(PE3)=PD13高 / State2(PE4)=舵機PWM\r\n");
    printf("[PYRO-SELFTEST] ⚠ PD13 會實際導通引爆 MOSFET，確認負載安全再繼續！\r\n");
    fflush(stdout);

    /* 起始安全狀態：FIRE 拉低、State1/State2 熄、SYS 起閃。
     * PD14 已於 main.c Servo_HoldLow()（開機）拉為 GPIO 低，此處不動，維持部署前無訊號。 */
    HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_OFF);
    HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin, PYRO_LED_OFF);
    sys_led_pump();

    /* 開機 Buzzer 兩聲。 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    buzzer_beep(PYRO_BUZZER_BEEP_MS);
    delay_fed(100U);
    buzzer_beep(PYRO_BUZZER_BEEP_MS);

    /* 退避倒數（讓人員遠離點火頭/舵機連桿）。 */
    for (uint32_t s = PYRO_SELFTEST_COUNTDOWN_S; s > 0U; s--) {
        printf("[PYRO-SELFTEST] 點火倒數 %lu ...\r\n", (unsigned long)s);
        fflush(stdout);
        delay_fed(1000U);
    }

    /* === 步驟 1：PD13 (FIRE / 副傘 DC 馬達) 拉高 FSM_DROGUE_MOTOR_RUN_MS(8s)，State1 LED 亮 === */
    printf("[PYRO-SELFTEST] [1] PD13(FIRE)=HIGH + State1 亮，維持 %ums\r\n",
           (unsigned)FSM_DROGUE_MOTOR_RUN_MS);
    fflush(stdout);
    HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_ON);
    HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);
    delay_fed(FSM_DROGUE_MOTOR_RUN_MS);
    HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_OFF);
    printf("[PYRO-SELFTEST] [1] PD13(FIRE)=LOW + State1 熄\r\n");
    fflush(stdout);

    /* === 步驟 2：等待 === */
    printf("[PYRO-SELFTEST] [2] 等待 %ums\r\n", (unsigned)PYRO_SELFTEST_GAP_MS);
    fflush(stdout);
    delay_fed(PYRO_SELFTEST_GAP_MS);

    /* === 步驟 3：PD14 由 GPIO 低切回 TIM4 AF，舵機 0°→180°→(停)→0°，PWM 期間 State2 LED 亮 ===
     * 部署前 PD14 一直是硬體低；此處才切 AF 啟動 PWM（比照 main.c Servo_DeployMain 的紀律）。 */
    {
        GPIO_InitTypeDef gi = {0};
        gi.Pin       = PWM_Servo_Pin;
        gi.Mode      = GPIO_MODE_AF_PP;
        gi.Pull      = GPIO_NOPULL;
        gi.Speed     = GPIO_SPEED_FREQ_LOW;
        gi.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(PWM_Servo_GPIO_Port, &gi);
    }
    printf("[PYRO-SELFTEST] [3] 舵機 PWM 啟動 + State2 亮：0°(%uus)→180°(%uus)\r\n",
           (unsigned)PYRO_SELFTEST_SERVO_0DEG_US, (unsigned)PYRO_SELFTEST_SERVO_180DEG_US);
    fflush(stdout);
    servo_set_us(PYRO_SELFTEST_SERVO_0DEG_US);         /* 先定 0° 再啟 PWM，避免首幀跳到殘值 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin, PYRO_LED_ON);
    delay_fed(500U);                                   /* 歸 0° 定位 */
    servo_set_us(PYRO_SELFTEST_SERVO_180DEG_US);       /* 轉 180° */
    delay_fed(PYRO_SELFTEST_SERVO_HOLD_MS);
    printf("[PYRO-SELFTEST] [3] 舵機 180°→0°(%uus)\r\n",
           (unsigned)PYRO_SELFTEST_SERVO_0DEG_US);
    fflush(stdout);
    servo_set_us(PYRO_SELFTEST_SERVO_0DEG_US);
    delay_fed(1000U);                                  /* 給舵機回程時間 */
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);           /* 停 PWM */
    HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin, PYRO_LED_OFF);

    printf("[PYRO-SELFTEST] 序列完成，返回呼叫端。\r\n");
    fflush(stdout);
    /* 注意：PD13(FIRE) 已於步驟 1 尾拉低；PD14 舵機此時為 TIM4 AF、PWM 已 Stop。
     * 遠端 BENCH 呼叫端（main.c 診斷任務）須於返回後 Servo_HoldLow() 把 PD14 復位為 GPIO 低，
     * 恢復部署前「無訊號」安全狀態再回歸正常 FSM。 */
}

#if FEATURE_PYRO_SELFTEST
void PyroSelfTest_RunOnce(void)
{
    PyroSelfTest_RunSequence();
    printf("[PYRO-SELFTEST] 停在此處（SYS 續閃，不進飛控）——電源重置可再測一次。\r\n");
    fflush(stdout);
    /* 停在原地：SYS 續閃 + 餵狗，確保「一次重啟＝一次測試」，不落入正常 FSM。 */
    for (;;) {
        sys_led_pump();
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(50);
    }
}
#endif /* FEATURE_PYRO_SELFTEST */

#endif /* PYRO_SELFTEST_AVAILABLE */

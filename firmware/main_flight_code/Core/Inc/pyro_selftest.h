/*
 * pyro_selftest.h — 開傘地面點火自測（PD13 引爆 + PD14 舵機 + LED/Buzzer 指示）
 * ===========================================================================
 * 用途：在工作台上驗證「開傘」完整電火動作。MCU 一上電/重啟就自動執行「一次」序列，
 *       跑完停在原地（持續餵看門狗、SYS LED 續閃），不進正常飛控 FSM，故「每次重啟＝一次測試」。
 *
 * ─── 測試序列（雙板協調；依 BOARD_ROLE 分流。需 USART2 交叉排線接兩板 + FEATURE_LINK）───
 *   開機：Buzzer 響兩聲（TIM2 CH1）；全程 SYS LED (PE2) 1Hz 閃 = 韌體存活；板間鏈路上線。
 *   ● 主航電 (PRIMARY)：
 *     1. PD13 (FIRE / 副傘 DC 馬達) 拉高 FSM_DROGUE_MOTOR_RUN_MS(8s)（State1/PE3 亮），再拉低。
 *     2. 等待 PYRO_SELFTEST_GAP_MS。
 *     3. PD14 舵機 0°→180°(停)→0°；「啟動 servo 同時」連續廣播 MAIN_DEPLOYED 命令，通知副板點火。
 *   ● 副航電 (BACKUP)：不自主動作。監聽板間鏈路，收到主板命令(peer MAIN_DEPLOYED)才把 PD13
 *     (FIRE / 主傘點火頭) 拉高 FSM_IGNITER_PULSE_MS(~1s) 脈衝後放開；無舵機動作。
 *     單獨上電（無主板）→ 一直等、不點火（設計如此）。
 *   （序列最前面另有可調退避倒數 PYRO_SELFTEST_COUNTDOWN_S，供人員遠離後才點火。）
 *
 * ─── LED / Buzzer 腳位（GPIOE，active-high；如硬體為 active-low 改 PYRO_LED_ON/OFF）───
 *   SYS    = PE2 (LED_SYS)      : 持續閃爍
 *   State1 = PE3               : PD13 拉高時亮
 *   State2 = PE4 (LED_STAT2)   : PD14 產生 PWM 時亮
 *   Buzzer = TIM2 CH1          : 開機兩聲
 *
 * ─── 如何啟用 ───  board_config.h 把 FEATURE_PYRO_SELFTEST 設 1，重新燒錄。
 * ─── 如何刪除 ───
 *   1. 最快：FEATURE_PYRO_SELFTEST 設回 0 —— 本模組與 main.c 呼叫全部 #if 編譯掉。
 *   2. 徹底：刪本檔 + pyro_selftest.c，移除 main.c「PYRO SELF-TEST」段（含 #include），
 *      並從 Main_Code/Debug/objects.list 與 subdir.mk 移除 pyro_selftest.o / .c。
 *
 * ⚠ 安全：步驟 1 會實際導通引爆 MOSFET（PD13）6 秒 —— 若已接火藥/電熱絲會真的點火！
 *   上台前務必確認負載安全或以電表/假負載替代；序列開頭保留退避倒數供人員退避。
 */
#ifndef PYRO_SELFTEST_H
#define PYRO_SELFTEST_H

#include "board_config.h"

#if FEATURE_PYRO_SELFTEST

/* === 可調參數 === */
/* 步驟 1：PD13(FIRE) 引爆 MOSFET 導通時間 */
#ifndef PYRO_SELFTEST_FIRE_MS
#define PYRO_SELFTEST_FIRE_MS       6000U   /* PD13 拉高 6s */
#endif
/* 步驟 2：引爆後、動舵機前的等待 */
#ifndef PYRO_SELFTEST_GAP_MS
#define PYRO_SELFTEST_GAP_MS        5000U   /* 等待 5s */
#endif

/* 步驟 3：PWM 舵機角度對應脈寬（TIM4 Prescaler=83→1MHz，1 tick=1µs，比較值＝脈寬 µs）。
 * 標準 180° 舵機常見 0°≈500µs、180°≈2500µs；若你的舵機是 1000–2000µs 行程請改這兩個值。 */
#ifndef PYRO_SELFTEST_SERVO_0DEG_US
#define PYRO_SELFTEST_SERVO_0DEG_US    500U    /* 0°   */
#endif
#ifndef PYRO_SELFTEST_SERVO_180DEG_US
#define PYRO_SELFTEST_SERVO_180DEG_US  2500U   /* 180° */
#endif
/* 舵機停在 180° 的保持時間 */
#ifndef PYRO_SELFTEST_SERVO_HOLD_MS
#define PYRO_SELFTEST_SERVO_HOLD_MS    5000U   /* 180° 停留 5s，再轉回 0° */
#endif

/* 序列最前面的退避倒數（0 = 立即開始點火） */
#ifndef PYRO_SELFTEST_COUNTDOWN_S
#define PYRO_SELFTEST_COUNTDOWN_S      5U
#endif

/* SYS LED 閃爍半週期（500ms → 1Hz 閃） */
#ifndef PYRO_SELFTEST_SYS_BLINK_MS
#define PYRO_SELFTEST_SYS_BLINK_MS     500U
#endif

/* LED 亮/滅電位（active-high；硬體若為 active-low 對調此二值） */
#ifndef PYRO_LED_ON
#define PYRO_LED_ON   GPIO_PIN_SET
#endif
#ifndef PYRO_LED_OFF
#define PYRO_LED_OFF  GPIO_PIN_RESET
#endif

/* 開機 Buzzer：TIM2 時基 1MHz → 頻率 = 1e6/(ARR+1)。ARR=499 ≈ 2kHz。 */
#ifndef PYRO_BUZZER_ARR
#define PYRO_BUZZER_ARR   499U
#endif
#ifndef PYRO_BUZZER_BEEP_MS
#define PYRO_BUZZER_BEEP_MS  120U
#endif

/*
 * 執行一次開傘電火自測序列後「不返回」（停在無窮迴圈，續閃 SYS 並餵 IWDG）。
 * 應在周邊初始化（MX_GPIO_Init / MX_TIM4_Init / MX_TIM2_Init / MX_IWDG_Init）與
 * 開機橫幅之後、進入正常飛控前呼叫。
 */
void PyroSelfTest_RunOnce(void);

#endif /* FEATURE_PYRO_SELFTEST */
#endif /* PYRO_SELFTEST_H */

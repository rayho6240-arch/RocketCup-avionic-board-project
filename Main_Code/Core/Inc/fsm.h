/*
 * fsm.h — 飛行狀態機純邏輯模組（P0-A 自 main.c FSM_Update 抽離）
 * ===========================================================================
 * 設計原則（比照 sensor_axis.h / attitude_math.h 的 host 共測模式）：
 *   - 本檔與 fsm.c 不依賴任何 HAL / CMSIS / FreeRTOS，僅 <stdint.h>，
 *     可在 host 上以 tests/test_fsm.c 模擬整段飛行剖面逐 cycle 驗證。
 *   - FSM_Step() 為純函式：輸入快照 (FSM_Input_t) → 狀態轉移 → 動作 (FSM_Action_t)。
 *     GPIO / PWM / 蜂鳴器 / printf 一律由呼叫端（main.c 的 FSM_Update 包裝）執行，
 *     且硬體動作必須先於事件列印（點火不被 UART 阻塞延遲）。
 *   - 呼叫頻率契約：100 Hz（每 10ms 一次）。速度差分與「連續 N 週期」防雜訊
 *     計數皆以此為前提。
 */
#ifndef FSM_H
#define FSM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === 飛行狀態（原 main.h USER CODE ET 區塊移入；數值不變） === */
typedef enum {
    STATE_INIT = 0,        // 系統初始化與自檢
    STATE_PAD = 1,         // 發射架上（等待校準完成）
    STATE_BOOST = 2,       // 動力上升（馬達燃燒）
    STATE_COAST = 3,       // 慣性滑行（頂點預測與監控啟用）
    STATE_APOGEE = 4,      // 達到頂點（觸發副傘/drogue部署，PD13 點火）
    STATE_DESCENT = 5,     // 副傘下降（監控主傘部署高度）
    STATE_MAIN_DEPLOY = 6, // 主傘部署（動態高度觸發舵機旋轉）
    STATE_LANDED = 7       // 安全著陸（尋標蜂鳴器與安全關檔）
} FlightState_t;

/* === FSM 參數（原 main.h 三常數 + FSM_Update 內魔術數字集中；值逐字不變） === */
#define TARGET_MAIN_ALTITUDE     150.0f  // 目標主傘完全張開高度 (m)
#define MAIN_DEPLOY_DELAY_S      3.5f    // 主傘機構部署延遲時間 (s)
#define DROGUE_LEAD_TIME_S       4.0f    // 副傘頂點預估提前開傘時間 (s)

#define FSM_LIFTOFF_ACCEL_G      3.0f    // 起飛觸發：高G垂直加速度門檻 (g)
#define FSM_LIFTOFF_ALT_M        10.0f   // 起飛觸發：EKF 高度門檻 (m)
#define FSM_BURNOUT_ACCEL_G      0.5f    // 馬達燒完：加速度低於此值 (g)
#define FSM_BURNOUT_MIN_MS       1500U   // 馬達燒完：起飛後最短時間 (ms)
#define FSM_APOGEE_MIN_FLIGHT_MS 3000U   // 頂點判定：起飛時間鎖 (ms)
#define FSM_APOGEE_CONSEC_N      5U      // 頂點判定：連續成立週期數（5×10ms=50ms 防雜訊）
#define FSM_APOGEE_VFALL_MPS     0.2f    // 頂點備用判定：速度過零門檻 (v_est < -0.2)
#define FSM_APOGEE_ALT_DROP_M    5.0f    // 頂點備用判定：自峰值下降高度 (m)
#define FSM_PYRO_HOLD_MS         2000U   // 副傘點火 MOSFET 導通限時 (ms)
#define FSM_MAIN_WATCHDOG_MS     25000U  // 主傘部署：飛行總時間看門狗 (ms)
#define FSM_MAIN_INFLATE_MS      3000U   // 主傘充氣張開等待時間 (ms)
#define FSM_TOUCHDOWN_V_MPS      0.3f    // 落地判定：|v_est| 門檻 (m/s)
#define FSM_TOUCHDOWN_ALT_M      20.0f   // 落地判定：高度門檻 (m)

#define FSM_STEP_PERIOD_MS       10U     // 呼叫頻率契約：100 Hz
#define FSM_STEP_PERIOD_S        0.010f  // COAST 速度差分用的週期 (s)

/* === 事件（供呼叫端列印 / 記錄；一次 FSM_Step 至多一個事件） === */
typedef enum {
    FSM_EVT_NONE = 0,
    FSM_EVT_LIFTOFF,       // PAD → BOOST
    FSM_EVT_BURNOUT,       // BOOST → COAST
    FSM_EVT_APOGEE,        // COAST → APOGEE（同時 fire_drogue）
    FSM_EVT_DROGUE_DONE,   // APOGEE → DESCENT（同時 release_drogue）
    FSM_EVT_MAIN_DEPLOY,   // DESCENT → MAIN_DEPLOY（同時 deploy_main）
    FSM_EVT_MAIN_OPEN,     // MAIN_DEPLOY → LANDED（充氣等待結束）
    FSM_EVT_TOUCHDOWN      // LANDED 內落地確認（同時 start_buzzer）
} FSM_Event_t;

/* === 輸入快照（呼叫端 main.c 組裝） === */
typedef struct {
    uint32_t now_ms;         // HAL_GetTick()
    float    h_est;          // EKF 估計高度 (m)
    float    v_est;          // EKF 估計垂直速度 (m/s)
    float    a_z_g;          // 高 G 垂直加速度 body-frame (g)
    float    baro_alt_rel;   // baro 原始高度 − pad 基準 (m)；P0-B 起使用
    uint8_t  ekf_calibrated; // EKF 靜態校準完成
    uint8_t  ekf_healthy;    // EKF 健康（P0-C 起接 EKF_GetHealthBits()；目前恆 1）
    uint8_t  sensor_bits;    // 感測器故障位元（P0-D 起接 sensor_health；目前恆 0）
} FSM_Input_t;

/* === 動作（呼叫端立即執行；硬體動作先於 printf） === */
typedef struct {
    uint8_t fire_drogue;     // 1 = 導通副傘引爆 MOSFET（PD13 HIGH）
    uint8_t release_drogue;  // 1 = 斷開點火（PD13 LOW）
    uint8_t deploy_main;     // 1 = 主傘釋放舵機（TIM4_CH3 CCR=2000）
    uint8_t start_buzzer;    // 1 = 開啟尋標蜂鳴器
    uint8_t event;           // FSM_Event_t
    float   apogee_t_pred;   // EVT_APOGEE 時的預估頂點時間 (s)，供事件列印
} FSM_Action_t;

/* === 內部狀態（原 FSM_Update 的 static 區域變數 + main.c 全域收納） === */
typedef struct {
    FlightState_t state;
    uint32_t flight_start_ms;     // 起飛基準 tick（原 flight_start_tick）
    uint32_t state_entered_ms;    // 當前狀態進入 tick
    float    max_altitude;        // 觀測到的最大 EKF 高度
    float    last_vel_z;          // 上一週期速度（COAST 差分估加速度用）
    uint8_t  consec_apogee_counts;
    uint8_t  touchdown_latched;   // 落地一次性觸發（原 g_touchdown_tick==0 判斷）
    uint8_t  drogue_fired;        // 副傘已點火鎖存（熱啟動防二次點火，P0-F 使用）
} FSM_Context_t;

/*
 * 初始化 / 熱啟動進入指定狀態。
 *   s0                  : 初始狀態（正常開機 STATE_PAD；熱啟動為恢復狀態）
 *   now_ms              : 當前 tick（state_entered_ms 基準）
 *   flight_start_ms     : 起飛基準 tick（正常開機 0；熱啟動 = now − 封包 tick）
 *   drogue_already_fired: 熱啟動時自 Flash 封包 flags 還原（防二次點火）
 */
void FSM_Init(FSM_Context_t *ctx, FlightState_t s0, uint32_t now_ms,
              uint32_t flight_start_ms, uint8_t drogue_already_fired);

/* 單步執行（100 Hz）。回傳本週期需執行的動作與事件。 */
FSM_Action_t FSM_Step(FSM_Context_t *ctx, const FSM_Input_t *in);

#ifdef __cplusplus
}
#endif

#endif /* FSM_H */

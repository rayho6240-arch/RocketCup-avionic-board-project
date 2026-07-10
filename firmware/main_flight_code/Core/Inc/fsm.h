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
#define FSM_LIFTOFF_ALT_M        5.0f    // 起飛觸發：EKF 高度門檻 (m)（測試調降 10→5）
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

/* === P0-B：頂點失效保護與 baro 原始趨勢交叉檢查（皆不依賴 EKF） === */
/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !! FSM_FAILSAFE_APOGEE_MS 為「無飛行模擬資料」下的保守暫定值。      !!
 * !! 飛行前必須以 OpenRocket 模擬重新推導：取 1.5 × t_apogee_sim，   !!
 * !! 且滿足 ≥ t_apogee_sim + 5s、≤ FSM_MAIN_WATCHDOG_MS − 5s。       !!
 * !! 推導依據：150m 主傘目標 + 25s 全程看門狗 → 本級別火箭頂點約     !!
 * !! 8–12s，15s 高於任何合理頂點 ≥3s，仍保留副傘→主傘序列時間。      !!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
#define FSM_FAILSAFE_APOGEE_MS   15000U  // 起飛起算強制點火副傘（BOOST 與 COAST 皆生效）
#define FSM_BARO_APOGEE_DROP_M   10.0f   // baro 原始高度自峰值回落門檻 (m)（≈BMP388 噪聲 16σ）
#define FSM_BARO_APOGEE_CONSEC   20U     // 連續 20 週期（200ms）成立（防雜訊）
#define FSM_LIFTOFF_BARO_ALT_M   5.0f    // 起飛第三冗餘：baro 相對高度門檻 (m)（測試調降 20→5）

/* sensor_bits 輸入位元契約（P0-D 起由 sensor_health 餵入；P0-B 起 FSM 即依此閘控 baro 路徑） */
#define FSM_SB_BARO_FAULT        0x01U   // baro 失效/不可信 → 停用 baro 交叉檢查與 baro 起飛冗餘

/* === P0-C：EKF unhealthy 時的 raw-baro 降級開傘鏈參數 ===
 * ekf_healthy=0（EKF_GetHealthBits()!=0 或 EKF_Task >300ms 無更新）時：
 *   起飛   = a_z 或 baro 冗餘（h_est 路徑停用）
 *   頂點   = baro 趨勢規則（P0-B；EKF 三路徑停用，防發散值誤點火）
 *   主傘   = baro 相對高度 ≤ 200m 或既有 25s 看門狗
 *   落地   = 2s 視窗內 |Δbaro| < 2m 且 baro < 30m */
#define FSM_FB_MAIN_ALT_M        200.0f  // 降級主傘高度（150m 目標 + 副傘 ~20m/s × 2.5s 餘裕，無速度項取固定值）
#define FSM_FB_TOUCHDOWN_WIN_MS  2000U   // 降級落地判定視窗
#define FSM_FB_TOUCHDOWN_DELTA_M 2.0f    // 視窗內 baro 變化量門檻
#define FSM_FB_TOUCHDOWN_ALT_M   30.0f   // 降級落地高度門檻

/* === P0-F：熱啟動驗證鏈參數 === */
#define FSM_HOTSTART_MAX_TICK_MS    60000U  // 封包飛行 tick 合理上限（防上次飛行殘留資料誤恢復）
#define FSM_HOTSTART_MAX_ALT_DIFF_M 300.0f  // 封包 baro 與當下 baro 容許差（IWDG 2.05s + 開機期下落餘裕）

/* === 事件（供呼叫端列印 / 記錄；一次 FSM_Step 至多一個事件） === */
typedef enum {
    FSM_EVT_NONE = 0,
    FSM_EVT_LIFTOFF,       // PAD → BOOST
    FSM_EVT_BURNOUT,       // BOOST → COAST
    FSM_EVT_APOGEE,        // COAST → APOGEE（同時 fire_drogue）
    FSM_EVT_APOGEE_FAILSAFE, // 失效保護計時器強制點火（BOOST/COAST → APOGEE，同時 fire_drogue）
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
    float    max_alt_baro;        // COAST 期 baro 原始相對高度滾動峰值（P0-B 交叉檢查）
    uint8_t  consec_baro_drop;    // baro 自峰值回落連續週期計數（P0-B）
    uint8_t  failsafe_fired;      // 失效保護計時器已觸發（遙測 TELEM_FLAG_FAILSAFE）
    float    fb_td_ref_alt;       // 降級落地判定：2s 視窗基準 baro 高度（P0-C）
    uint32_t fb_td_ref_tick;      // 降級落地判定：視窗起始 tick（0=未初始化）
} FSM_Context_t;

/* === P0-F：熱啟動決策（純函式，main.c 收集輸入後呼叫） === */
typedef struct {
    uint8_t       restore;       // 1 = 恢復飛行狀態；0 = 回 STATE_PAD 完整重校準
    FlightState_t state;         // 恢復目標狀態（已套用防二次點火政策）
    uint8_t       drogue_fired;  // 交給 FSM_Init 的點火鎖存
} FSM_HotStartDecision_t;

/*
 * 熱啟動驗證鏈（任一失敗 → restore=0 回 PAD）：
 *   1. pkt_valid：ring 末筆封包 CRC 有效
 *   2. 封包狀態 ∈ BOOST..DESCENT（正常落地後末筆為 LANDED → 自然回 PAD）
 *   3. pkt_tick_ms < 60s：防上次飛行殘留的飛行中封包在地面誤恢復
 *   4. |封包 baro − 當下 baro| < 300m：高度連續性
 * 重點火政策：pkt_drogue_fired=1 → 恢復目標強制 ≥ STATE_DESCENT（杜絕二次點火）。
 */
FSM_HotStartDecision_t FSM_HotStartDecide(uint8_t pkt_valid,
                                          uint8_t pkt_fsm_state,
                                          uint32_t pkt_tick_ms,
                                          float pkt_baro_alt_m,
                                          float cur_baro_alt_m,
                                          uint8_t pkt_drogue_fired);

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

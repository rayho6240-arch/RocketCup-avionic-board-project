/*
 * fsm.c — 飛行狀態機純邏輯（P0-A 自 main.c FSM_Update 逐字搬移，行為保存）
 * ===========================================================================
 * 不依賴 HAL / RTOS；所有條件式與參數值與原 main.c:1425-1562 完全一致，
 * 由 tests/test_fsm.c 黃金飛行剖面鎖定行為。硬體動作由呼叫端執行。
 */
#include "fsm.h"
#include <math.h>
#include <string.h>

void FSM_Init(FSM_Context_t *ctx, FlightState_t s0, uint32_t now_ms,
              uint32_t flight_start_ms, uint8_t drogue_already_fired)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state            = s0;
    ctx->flight_start_ms  = flight_start_ms;
    ctx->state_entered_ms = now_ms;
    ctx->drogue_fired     = (drogue_already_fired != 0U);
}

FSM_Action_t FSM_Step(FSM_Context_t *ctx, const FSM_Input_t *in)
{
    FSM_Action_t act;
    memset(&act, 0, sizeof(act));

    const float    h_est = in->h_est;   // 卡爾曼估計高度 (m)
    const float    v_est = in->v_est;   // 卡爾曼估計垂直速度 (m/s)
    const float    a_z   = in->a_z_g;   // 高 G 垂直加速度 (g)
    const uint32_t now   = in->now_ms;

    // 更新觀測到的最大高度
    if (h_est > ctx->max_altitude) {
        ctx->max_altitude = h_est;
    }

    const uint8_t baro_ok = ((in->sensor_bits & FSM_SB_BARO_FAULT) == 0U);

    /* === P0-B：頂點絕對失效保護（最後防線，不依賴任何感測器/EKF） ===
     * BOOST 與 COAST 皆生效：即使燒完判定失效卡在 BOOST、或 EKF 向上發散使
     * 頂點條件永不成立，起飛後 FSM_FAILSAFE_APOGEE_MS 仍強制點火副傘。 */
    if ((ctx->state == STATE_BOOST || ctx->state == STATE_COAST) &&
        (now - ctx->flight_start_ms) >= FSM_FAILSAFE_APOGEE_MS) {
        ctx->state            = STATE_APOGEE;
        ctx->state_entered_ms = now;
        ctx->drogue_fired     = 1U;
        ctx->failsafe_fired   = 1U;
        act.fire_drogue       = 1U;   // 強制導通副傘引爆 MOSFET (PD13 = HIGH)
        act.event             = FSM_EVT_APOGEE_FAILSAFE;
        ctx->last_vel_z = v_est;
        return act;
    }

    switch (ctx->state) {
        case STATE_PAD:
            // 等待靜態校準完成
            if (in->ekf_calibrated) {
                // 起飛觸發條件：高G加速度 > 3.0g 或高度 > 10.0m
                // 或 baro 相對高度 > 20.0m（P0-B 第三冗餘：ADXL 與 EKF 雙失效仍可偵測起飛）
                if (a_z > FSM_LIFTOFF_ACCEL_G || h_est > FSM_LIFTOFF_ALT_M ||
                    (baro_ok && in->baro_alt_rel > FSM_LIFTOFF_BARO_ALT_M)) {
                    ctx->state            = STATE_BOOST;
                    ctx->flight_start_ms  = now;
                    ctx->state_entered_ms = now;
                    act.event = FSM_EVT_LIFTOFF;
                }
            }
            break;

        case STATE_BOOST:
            // 馬達燒完判定：加速度 < 0.5g 且 flight time > 1.5 秒
            if (a_z < FSM_BURNOUT_ACCEL_G &&
                (now - ctx->state_entered_ms) > FSM_BURNOUT_MIN_MS) {
                ctx->state            = STATE_COAST;
                ctx->state_entered_ms = now;
                act.event = FSM_EVT_BURNOUT;
            }
            break;

        case STATE_COAST: {
            // 動態頂點預估 (預估 4.0s 前開副傘)
            float decel = -9.80665f;
            if (ctx->last_vel_z != 0.0f) {
                float a_z_nav = (v_est - ctx->last_vel_z) / FSM_STEP_PERIOD_S; // 10ms 速度差所得加速度
                if (a_z_nav < -5.0f && a_z_nav > -25.0f) {
                    decel = a_z_nav;
                }
            }

            float t_to_apogee = -v_est / decel;

            /* P0-B：baro 原始趨勢交叉檢查（完全不依賴 EKF）。
             * 追蹤 COAST 期 baro 相對高度滾動峰值，自峰值回落 ≥10m 並連續 20 週期
             * （200ms）即視為已過頂點。EKF 向上發散時這是真正在頂點附近開傘的
             * 主路徑（失效保護計時器只是更晚的最後防線）。 */
            uint8_t baro_apogee = 0;
            if (baro_ok) {
                if (in->baro_alt_rel > ctx->max_alt_baro) {
                    ctx->max_alt_baro = in->baro_alt_rel;
                }
                if ((ctx->max_alt_baro - in->baro_alt_rel) >= FSM_BARO_APOGEE_DROP_M) {
                    if (ctx->consec_baro_drop < 255U) ctx->consec_baro_drop++;
                } else {
                    ctx->consec_baro_drop = 0U;
                }
                baro_apogee = (ctx->consec_baro_drop >= FSM_BARO_APOGEE_CONSEC) ? 1U : 0U;
            }

            // 頂點判定條件：
            // 1. 動態預測時間 <= 4.0s (且仍處於上升狀態 v_est > 0)
            // 2. 備用安全判定：垂直速度過零 (v_est < -0.2 m/s) 或是高度從峰值下降超過 5.0m
            // 3. P0-B：baro 原始趨勢交叉檢查（上方 baro_apogee）
            // 同時必須滿足起飛時間鎖（起飛後累計大於 3.0 秒）
            uint8_t apogee_condition = 0;
            if (v_est > 0.0f && t_to_apogee <= DROGUE_LEAD_TIME_S) {
                apogee_condition = 1;
            } else if (v_est < -FSM_APOGEE_VFALL_MPS ||
                       (ctx->max_altitude - h_est) > FSM_APOGEE_ALT_DROP_M) {
                apogee_condition = 1;
            } else if (baro_apogee) {
                apogee_condition = 1;
            }

            if (apogee_condition &&
                (now - ctx->flight_start_ms) > FSM_APOGEE_MIN_FLIGHT_MS) {
                ctx->consec_apogee_counts++;
                if (ctx->consec_apogee_counts >= FSM_APOGEE_CONSEC_N) { // 連續 5 個週期 (50ms) 成立以防雜訊
                    ctx->state            = STATE_APOGEE;
                    ctx->state_entered_ms = now;
                    ctx->drogue_fired     = 1U;
                    act.fire_drogue       = 1U;   // 導通副傘引爆 MOSFET (PD13 = HIGH)
                    act.event             = FSM_EVT_APOGEE;
                    act.apogee_t_pred     = t_to_apogee;
                }
            } else {
                ctx->consec_apogee_counts = 0;
            }
            break;
        }

        case STATE_APOGEE:
            // 點火限時導通保護：持續導通 2.0 秒後強制拉低
            if (now - ctx->state_entered_ms >= FSM_PYRO_HOLD_MS) {
                ctx->state            = STATE_DESCENT;
                ctx->state_entered_ms = now;
                act.release_drogue    = 1U;   // 斷開點火 (PD13 = LOW)
                act.event             = FSM_EVT_DROGUE_DONE;
            }
            break;

        case STATE_DESCENT: {
            // 動態主傘部署高度計算：h_trigger = h_target + |v_fall| * t_delay
            float v_fall = (v_est < 0.0f) ? -v_est : 0.0f;
            float h_trigger_main = TARGET_MAIN_ALTITUDE + v_fall * MAIN_DEPLOY_DELAY_S;

            // 觸發條件：高度低於觸發高度，或是飛行總時間看門狗超時 (25秒)
            if (h_est <= h_trigger_main ||
                (now - ctx->flight_start_ms) > FSM_MAIN_WATCHDOG_MS) {
                ctx->state            = STATE_MAIN_DEPLOY;
                ctx->state_entered_ms = now;
                act.deploy_main       = 1U;   // 部署主傘：PD14 釋放舵機 (PWM 脈寬 2000)
                act.event             = FSM_EVT_MAIN_DEPLOY;
            }
            break;
        }

        case STATE_MAIN_DEPLOY:
            // 等待 3 秒讓主傘充氣張開，隨後進入落地偵測
            if (now - ctx->state_entered_ms >= FSM_MAIN_INFLATE_MS) {
                ctx->state            = STATE_LANDED;
                ctx->state_entered_ms = now;
                act.event             = FSM_EVT_MAIN_OPEN;
            }
            break;

        case STATE_LANDED:
            // 落地判定：下墜速度趨近零，且高度小於 20m（僅觸發一次）
            if (!ctx->touchdown_latched &&
                fabsf(v_est) < FSM_TOUCHDOWN_V_MPS && h_est < FSM_TOUCHDOWN_ALT_M) {
                ctx->touchdown_latched = 1U;
                act.start_buzzer       = 1U;   // 開啟板載尋標蜂鳴器（持續鳴叫，利於落點尋標）
                act.event              = FSM_EVT_TOUCHDOWN;
            }
            break;

        default:
            break;
    }

    ctx->last_vel_z = v_est; // 儲存速度歷史

    return act;
}

/*
 * test_fsm_elevator.c — 電梯場測 profile 黃金剖面單元測試（純 host 編譯，不需硬體）
 * ===========================================================================
 *   cd tests && make run                （或單獨：make test_fsm_elevator && ./test_fsm_elevator）
 *   編譯旗標：-DFLIGHT_PROFILE_ELEVATOR=1（見 tests/Makefile），
 *   使 fsm.h 的 profile 分組常數切換為電梯門檻（board_config.h FLIGHT_PROFILE_ELEVATOR）。
 *
 * 目的：
 *   電梯場測與真實飛行共用同一份 fsm.c 邏輯，僅門檻不同（board_config.h 的
 *   FEATURE_FORCE_BARO_ONLY 由 FLIGHT_PROFILE_ELEVATOR 推導，全程強制
 *   ekf_healthy=0，開傘鏈完全走純氣壓路徑）。本測試鎖定電梯剖面下的行為，
 *   並證明 EKF 輸入（垃圾值）在此 profile 下完全不影響任何判定。
 *
 * 涵蓋：
 *   [1] 標稱電梯行程：PAD→BOOST→COAST→APOGEE→DESCENT→MAIN_DEPLOY→LANDED
 *       全程四動作（fire/release/main/buzzer）各恰一次，時刻皆解析推導。
 *   [2] 干擾免疫：頂樓單週期 baro 尖刺（consec 40 防護）/ 上升期抖動 / PAD 抖動
 *   [3] 垃圾 EKF 值免疫：h_est/v_est 任意異常值不得影響開傘判定
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "fsm.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

/* === 模擬器：100Hz 推進，累計動作次數與事件時刻（精簡複製自 test_fsm.c） === */
typedef struct {
    FSM_Context_t ctx;
    FSM_Input_t   in;
    uint32_t      now;
    int fire_n, release_n, main_n, buzzer_n;
    uint32_t t_liftoff, t_burnout, t_apogee, t_drogue_done, t_main, t_main_open, t_touchdown;
} Sim_t;

/* 電梯 profile 全程 ekf_healthy=0（對應 FEATURE_FORCE_BARO_ONLY），且全程餵垃圾
 * h_est/v_est，證明 EKF 輸入在本 profile 下完全被無效化（僅 baro + a_z 生效）。 */
static void sim_init(Sim_t *s, FlightState_t s0, uint32_t t0,
                     uint32_t flight_start, uint8_t drogue_fired) {
    memset(s, 0, sizeof(*s));
    s->now = t0;
    FSM_Init(&s->ctx, s0, t0, flight_start, drogue_fired);
    s->in.ekf_calibrated = 0;      /* 不重要：ekf_healthy=0 時完全不看此欄位 */
    s->in.ekf_healthy    = 0;      /* 電梯 profile：FEATURE_FORCE_BARO_ONLY 恆強制降級 */
    s->in.a_z_g          = 1.0f;   /* 電梯全程等速，恆 1g（無淨加速度） */
    s->in.h_est           = 500.0f;  /* 垃圾值：若被誤用會嚴重誤判起飛/頂點/落地 */
    s->in.v_est           = -50.0f;  /* 垃圾值：若被誤用會誤判「正在墜落」 */
}

static FSM_Action_t sim_step(Sim_t *s) {
    s->in.now_ms = s->now;
    FSM_Action_t a = FSM_Step(&s->ctx, &s->in);
    s->fire_n    += a.fire_drogue;
    s->release_n += a.release_drogue;
    s->main_n    += a.deploy_main;
    s->buzzer_n  += a.start_buzzer;
    switch ((FSM_Event_t)a.event) {
        case FSM_EVT_LIFTOFF:     s->t_liftoff     = s->now; break;
        case FSM_EVT_BURNOUT:     s->t_burnout     = s->now; break;
        case FSM_EVT_DEPLOY_DROGUE: s->t_apogee    = s->now; break;
        case FSM_EVT_DROGUE_DONE: s->t_drogue_done = s->now; break;
        case FSM_EVT_MAIN_DEPLOY: s->t_main        = s->now; break;
        case FSM_EVT_MAIN_OPEN:   s->t_main_open   = s->now; break;
        case FSM_EVT_TOUCHDOWN:   s->t_touchdown   = s->now; break;
        default: break;
    }
    s->now += FSM_STEP_PERIOD_MS;
    return a;
}

static int near_ms(uint32_t actual, uint32_t expect, uint32_t tol) {
    uint32_t d = (actual > expect) ? (actual - expect) : (expect - actual);
    return d <= tol;
}

/* 電梯高度剖面：PAD 靜置至 t0 → 爬升(1.5m/s) → 頂樓 30m 持平 → 下降(1.5m/s) → 地面持平。
 * 回傳 baro 相對高度 (m)。爬升/下降時間 = 30 / 1.5 = 20s。 */
#define ELEV_T0        2000U    /* PAD 靜置結束、開始爬升 */
#define ELEV_RISE_MS   20000U   /* 30m ÷ 1.5 m/s */
#define ELEV_HOLD_MS   5000U    /* 頂樓持平 */
#define ELEV_FALL_MS   20000U   /* 30m ÷ 1.5 m/s */
#define ELEV_TOP_M     30.0f

static float elevator_profile(uint32_t t) {
    if (t < ELEV_T0) return 0.0f;
    uint32_t dt = t - ELEV_T0;
    if (dt < ELEV_RISE_MS) return ELEV_TOP_M * (float)dt / (float)ELEV_RISE_MS;
    dt -= ELEV_RISE_MS;
    if (dt < ELEV_HOLD_MS) return ELEV_TOP_M;
    dt -= ELEV_HOLD_MS;
    if (dt < ELEV_FALL_MS) return ELEV_TOP_M - ELEV_TOP_M * (float)dt / (float)ELEV_FALL_MS;
    return 0.0f;
}

/* ---------------------------------------------------------------- */
/*
 * [1] 標稱電梯行程 —— 解析推導（10ms 步階網格，皆為 10 的倍數）：
 *   liftoff：baro>3.0 首次成立於 t0+2010=4010（1.5·2.01=3.015>3.0；t=4000 恰
 *            3.0 不算）。
 *   burnout：a_z=1.0<2.0（電梯恆成立）僅受時間鎖：state_entered(4010)+1510=5520
 *            （>1500 首次成立於 +1510，同 test_fsm.c 模式）。
 *   apogee ：COAST 進峰值持續累積至 30.0（t=27000 為 hold 段最後一筆峰值）；
 *            下降段 baro(t)=30−0.015n，n=(t−27000)/10。首次 (peak−baro)≥2.0
 *            於 n=134（t=28340，baro=27.99）；consec_baro_drop 需連續 40 週期
 *            → 於 n=173（t=28730）達 40；再需 FSM_APOGEE_CONSEC_N=5 週期確認
 *            → 於 n=177（t=28770）點火。
 *   drogue_done：apogee + FSM_DROGUE_MOTOR_RUN_MS(8000) = 36770（DEPLOY_DROGUE 馬達
 *            停止；APOGEE 僅存續 1 週期即轉 DESCENT，仍遠早於 baro 自然降到 main 門檻的
 *            t=41670，下游時刻不受影響）。
 *   main   ：baro≤FSM_FB_MAIN_ALT_M(8.0) 首次成立於 n=1467（t=41670，baro=7.995）。
 *   main_open：main + FSM_MAIN_INFLATE_MS(3000) = 44670。
 *   touchdown：LANDED 於 44670 取樣基準 baro=3.495；t=46670 視窗到期但
 *            |Δ|=3.0≥2.0 不成立，基準重置為 baro(46670)=0.495；下一視窗於
 *            48670 到期，此時已進入地面持平（t≥47000 起 baro=0），
 *            |Δ|=0.495<2.0 且 baro=0<5.0 → 觸發。
 * 以上為解析推導；斷言仍以 near_ms 容忍浮點捨入（±1 步階內）。
 */
static void test_nominal_elevator(void) {
    printf("[1] 標稱電梯行程（解析推導，電梯 profile）\n");
    Sim_t s;
    sim_init(&s, STATE_PAD, 0, 0, 0);

    const uint32_t END_MS = 51000U;
    while (s.now < END_MS) {
        s.in.baro_alt_rel = elevator_profile(s.now);
        sim_step(&s);
    }

    check("LIFTOFF 於 baro>3.0m (t=4010)", s.t_liftoff == 4010 && s.fire_n <= 1);
    check("BURNOUT 受 1500ms 時間鎖 (t=5520)", near_ms(s.t_burnout, 5520, 10));
    check("APOGEE 於自峰值回落2m+連續40+5週期 (t≈28770)", near_ms(s.t_apogee, 28770, 20));
    check("馬達停止於 +8000ms (t≈36770)", near_ms(s.t_drogue_done, s.t_apogee + 8000U, 10));
    check("MAIN 部署於 baro≤8m (t≈41670)", near_ms(s.t_main, 41670, 20));
    check("MAIN_OPEN 於 +3000ms (t≈44670)", near_ms(s.t_main_open, s.t_main + 3000U, 10));
    check("TOUCHDOWN 於地面穩定2s視窗後 (t≈48670)", near_ms(s.t_touchdown, 48670, 30));

    check("狀態機走完整序列至 LANDED", s.ctx.state == STATE_LANDED);
    check("全程四動作各恰一次",
          s.fire_n == 1 && s.release_n == 1 && s.main_n == 1 && s.buzzer_n == 1);
}

/* ---------------------------------------------------------------- */
static void test_interference_immunity(void) {
    printf("[2] 干擾免疫（電梯 profile）\n");
    Sim_t s;

    /* 2a. 頂樓持平期單週期 +2.5m baro 尖刺：consec 40（400ms）防護——
     * 尖刺後短時間窗（<400ms）內不得觸發。 */
    sim_init(&s, STATE_COAST, 20000, 20000 - 5000, 0);  /* 時間鎖已過 */
    s.in.baro_alt_rel = 30.0f;
    for (int i = 0; i < 50; i++) sim_step(&s);   /* 建立穩定頂樓峰值 30.0 */
    check("頂樓穩定持平不誤觸", s.fire_n == 0 && s.ctx.state == STATE_COAST);
    s.in.baro_alt_rel = 32.5f;                    /* 單週期尖刺 */
    sim_step(&s);
    s.in.baro_alt_rel = 30.0f;                    /* 立即回穩 */
    for (int i = 0; i < 30; i++) sim_step(&s);     /* 尖刺後 300ms（<40 週期 consec 門檻）*/
    check("單週期尖刺後 300ms 內未觸發（consec 40 防護生效中）",
          s.fire_n == 0 && s.ctx.state == STATE_COAST);

    /* 2b. 上升期 ±0.5m 交替抖動（遠低於電梯 2.0m 門檻）：不得誤判頂點。
     * baro 整體仍隨時間上升（峰值持續刷新），抖動不足以造成 2m 回落。 */
    sim_init(&s, STATE_COAST, 10000, 10000 - 5000, 0);  /* 時間鎖已過 */
    {
        float base = 10.0f;
        for (int i = 0; i < 300; i++) {   /* 3s：上升 1.5m/s + ±0.5m 抖動 */
            base += 1.5f * (float)FSM_STEP_PERIOD_MS / 1000.0f;
            s.in.baro_alt_rel = base + ((i % 2) ? 0.5f : -0.5f);
            sim_step(&s);
        }
    }
    check("上升期抖動不誤判頂點", s.fire_n == 0 && s.ctx.state == STATE_COAST);

    /* 2c. PAD 期 baro 抖動 ±2m（皆 < FSM_LIFTOFF_BARO_ALT_M=3.0m 門檻）：不得誤起飛。 */
    sim_init(&s, STATE_PAD, 0, 0, 0);
    for (int i = 0; i < 400; i++) {   /* 4s */
        s.in.baro_alt_rel = ((i % 2) ? 2.0f : -2.0f);
        sim_step(&s);
    }
    check("PAD 抖動（<3m 門檻）不誤起飛", s.ctx.state == STATE_PAD && s.t_liftoff == 0);
}

/* ---------------------------------------------------------------- */
static void test_garbage_ekf_immunity(void) {
    printf("[3] 垃圾 EKF 值免疫（ekf_healthy=0 路徑全關）\n");
    Sim_t s;

    /* 上升途中：baro 正常爬升（不觸發任何 baro 條件），h_est 瞬間歸零
     * （模擬 rest-reset 下游效應）、v_est=-5（看似正在墜落）。
     * ekf_healthy=0 時 EKF 三路徑完全停用，不得因此點火。 */
    sim_init(&s, STATE_COAST, 10000, 10000 - 5000, 0);   /* 時間鎖已過 */
    s.in.baro_alt_rel = 12.0f;   /* 建立穩定峰值，遠未回落 */
    for (int i = 0; i < 50; i++) sim_step(&s);
    check("垃圾值注入前：baro 平穩不誤觸", s.fire_n == 0);

    s.in.h_est = 0.0f;    /* 瞬間歸零：健康時會誤判「已回到地面」 */
    s.in.v_est = -5.0f;   /* 看似正在墜落 */
    for (int i = 0; i < 500; i++) {   /* 持續 5s，baro 仍持平不回落 */
        sim_step(&s);
    }
    check("h_est 歸零 + v_est=-5：不點火（ekf_healthy=0 全免疫）",
          s.fire_n == 0 && s.ctx.state == STATE_COAST);
}

/* ---------------------------------------------------------------- */
int main(void) {
    printf("=== test_fsm_elevator：電梯場測 profile 黃金剖面（FLIGHT_PROFILE_ELEVATOR=1） ===\n");
    test_nominal_elevator();
    test_interference_immunity();
    test_garbage_ekf_immunity();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}

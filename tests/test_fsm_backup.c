/*
 * test_fsm_backup.c — 副航電（BOARD_ROLE=ROLE_BACKUP）baro-only 300m 點火鏈
 * ===========================================================================
 * 編譯：-DBOARD_ROLE=1 -DFLIGHT_PROFILE_ELEVATOR=0
 *   → IS_BACKUP=1 → fsm.c 的 STATE_DESCENT baro 分支走副板專屬邏輯：
 *       FSM_BARO_MAIN_DEPLOY_ALT_M = TARGET_MAIN_ALTITUDE(300m)（非主板降級的 200m）；
 *       雙路徑 OR（自身 baro≤300 或 peer_main_cmd）＋「baro 峰值曾>300」arm 互鎖。
 * 副板恆 baro-only（ekf_healthy=0），與 main.c 對 IS_BACKUP 的強制一致。
 */
#include <stdio.h>
#include <string.h>
#include "fsm.h"

static int g_fail = 0, g_total = 0;
static void check(const char *n, int c) {
    g_total++;
    printf(c ? "  [PASS] %s\n" : "  [FAIL] %s\n", n);
    if (!c) g_fail++;
}

typedef struct { FSM_Context_t ctx; FSM_Input_t in; uint32_t now; int fire_n, main_n; } Sim;

static void sinit(Sim *s, FlightState_t s0, uint32_t t0, uint32_t fstart) {
    memset(s, 0, sizeof(*s));
    s->now = t0;
    FSM_Init(&s->ctx, s0, t0, fstart, 0);
    s->in.ekf_healthy = 0;      /* 副板恆 baro-only */
    s->in.a_z_g       = 1.0f;
}

static FSM_Action_t sstep(Sim *s) {
    s->in.now_ms = s->now;
    FSM_Action_t a = FSM_Step(&s->ctx, &s->in);
    s->fire_n += a.fire_drogue;
    s->main_n += a.deploy_main;
    s->now    += FSM_STEP_PERIOD_MS;
    return a;
}

/* [1] arm 互鎖：baro 峰值未過 300m → baro≤300 與主板命令皆不得點火（pad/低空防誤點）。 */
static void test_arm_blocks(void) {
    printf("[1] arm 互鎖：baro 峰值未過 300m → 即使 peer_main_cmd 也不點火\n");
    Sim s; sinit(&s, STATE_DESCENT, 10000, 5000);
    s.in.baro_alt_rel  = 250.0f;    /* ≤300 但峰值僅 250：arm 未解鎖 */
    s.in.peer_main_cmd = 1;         /* 主板命令 */
    for (int i = 0; i < 20; i++) sstep(&s);
    check("峰值 250<300：baro_low + 主板命令皆不點火（arm 未解鎖）",
          s.main_n == 0 && s.ctx.state == STATE_DESCENT);
}

/* [2] arm 解鎖（峰值>300）後，自身 baro≤300（且>200）→ 點火恰一次。
 *     baro=280 同時驗證副板門檻＝300（若誤用主板 200，280>200 不會點）。 */
static void test_arm_then_baro(void) {
    printf("[2] arm 解鎖後自身 baro≤300 → 點火恰一次（門檻＝300 非 200）\n");
    Sim s; sinit(&s, STATE_DESCENT, 10000, 5000);
    s.in.baro_alt_rel = 350.0f;     /* >300：baro_low=0，但累積峰值→解鎖 arm */
    for (int i = 0; i < 5; i++) sstep(&s);
    check("baro>300 期間不點火（未達門檻）", s.main_n == 0 && s.ctx.state == STATE_DESCENT);
    s.in.baro_alt_rel = 280.0f;     /* ≤300（且>200）→ 觸發 */
    for (int i = 0; i < 5; i++) sstep(&s);
    check("baro≤300 且 arm 已解鎖 → 點火恰一次", s.main_n == 1 && s.ctx.state == STATE_MAIN_DEPLOY);
}

/* [3] 主板命令點火：arm 解鎖 + peer_main_cmd（自身 baro 尚未≤300）→ 立即點火。 */
static void test_cmd_path(void) {
    printf("[3] 主板命令點火：arm 解鎖 + peer_main_cmd（baro 尚未≤300）→ 立即點火\n");
    Sim s; sinit(&s, STATE_DESCENT, 10000, 5000);
    s.in.baro_alt_rel = 350.0f;     /* >300：解鎖 arm，baro_low=0 */
    for (int i = 0; i < 3; i++) sstep(&s);
    check("baro>300、無命令：不點火", s.main_n == 0 && s.ctx.state == STATE_DESCENT);
    s.in.peer_main_cmd = 1;         /* 主板開 servo → 命令副板點火 */
    sstep(&s);
    check("收主板命令 + arm 已解鎖 → 立即點火（早於自身 baro≤300）",
          s.main_n == 1 && s.ctx.state == STATE_MAIN_DEPLOY);
}

/* [4] baro-only 頂點（趨勢）+ 下降 300m 點火整合：確認全鏈在副板 baro-only 下走通。 */
static void test_baro_trend_to_main(void) {
    printf("[4] baro-only：COAST 趨勢頂點 → DESCENT → 下降 300m 點火\n");
    Sim s; sinit(&s, STATE_COAST, 10000, 6000);   /* 起飛時間鎖已過 */
    /* 升段：baro 100→400（峰值 400>300 解鎖 arm） */
    for (float b = 100.0f; b <= 400.0f; b += 10.0f) { s.in.baro_alt_rel = b; sstep(&s); }
    /* 降段：自峰值回落 ≥10m 連續 → baro 趨勢頂點 → DEPLOY_DROGUE */
    int guard = 0;
    while (s.ctx.state == STATE_COAST && guard < 2000) {
        s.in.baro_alt_rel -= 1.0f; sstep(&s); guard++;
    }
    check("baro 趨勢頂點 → 離開 COAST（進 DEPLOY_DROGUE）", s.ctx.state == STATE_DEPLOY_DROGUE);
    check("副板 FSM 於頂點仍會 set fire_drogue（實體輸出由 main.c 閘掉）", s.fire_n == 1);
    /* DEPLOY_DROGUE 8s → APOGEE → DESCENT（baro 維持高於 300 以隔離出 DESCENT 才點火） */
    s.in.baro_alt_rel = 360.0f;
    guard = 0;
    while (s.ctx.state != STATE_DESCENT && guard < 2000) { sstep(&s); guard++; }
    check("8s 馬達段後進 DESCENT（尚未≤300 不點主傘）", s.ctx.state == STATE_DESCENT && s.main_n == 0);
    /* 下降穿越 300m → 點火恰一次（峰值 400 已解鎖 arm） */
    s.in.baro_alt_rel = 290.0f;
    for (int i = 0; i < 5; i++) sstep(&s);
    check("下降 baro≤300 → 主傘點火恰一次", s.main_n == 1 && s.ctx.state == STATE_MAIN_DEPLOY);
}

int main(void) {
    printf("=== test_fsm_backup：副航電 baro-only 300m 點火鏈（IS_BACKUP） ===\n");
    test_arm_blocks();
    test_arm_then_baro();
    test_cmd_path();
    test_baro_trend_to_main();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}

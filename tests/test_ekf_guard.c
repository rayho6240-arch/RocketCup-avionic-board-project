/*
 * test_ekf_guard.c — EKF 防護純函式單元測試（P0-C，純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 驗證 ekf_guard.h 四個純函式的邊界行為：
 *   [1] baro 創新值閘控：5σ 與 25m 絕對下限的取大邏輯
 *   [2] 協方差對角夾限：位置/速度上限、共同下限、非對角不動
 *   [3] 狀態理智界限：六個狀態各自的上下界
 *   [4] NaN/Inf 掃描：狀態、協方差對角、四元數任一非有限值即回報
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "ekf_guard.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}
static int feq(float a, float b) { return fabsf(a - b) < 1e-3f; }

static void test_baro_gate(void) {
    printf("[1] baro 創新值閘控 max(5·sqrt(S), 25m)\n");
    /* 收斂後 S 小（R_baro=0.36 → 5σ=3m）→ 25m 絕對下限生效 */
    float S_small = 0.36f;
    check("S 小：|y|=24.9 接受（25m 下限）",  ekf_guard_baro_accept( 24.9f, S_small) == 1);
    check("S 小：|y|=-24.9 接受",             ekf_guard_baro_accept(-24.9f, S_small) == 1);
    check("S 小：|y|=25.1 拒收",              ekf_guard_baro_accept( 25.1f, S_small) == 0);
    check("S 小：±10km 跳變拒收",             ekf_guard_baro_accept(10000.0f, S_small) == 0 &&
                                              ekf_guard_baro_accept(-10000.0f, S_small) == 0);
    /* S 大（σ=10m → 5σ=50m > 25m）→ 5σ 閘生效 */
    float S_big = 100.0f;
    check("S 大：|y|=49 接受（5σ=50）",       ekf_guard_baro_accept(49.0f, S_big) == 1);
    check("S 大：|y|=51 拒收",                ekf_guard_baro_accept(51.0f, S_big) == 0);
}

static void test_clamp_P(void) {
    printf("[2] 協方差對角夾限\n");
    float P[6][6];

    memset(P, 0, sizeof(P));
    for (int i = 0; i < 6; i++) P[i][i] = 1.0f;
    P[0][3] = 123.0f;   /* 非對角，不應被動 */
    check("全在界內：回傳 0、不變動", ekf_guard_clamp_P(P) == 0 && feq(P[0][0], 1.0f) && feq(P[0][3], 123.0f));

    P[1][1] = 2.0e4f;   /* 位置超上限 */
    P[4][4] = 5.0e3f;   /* 速度超上限 */
    P[2][2] = 0.0f;     /* 低於下限 */
    check("夾限發生：回傳 1", ekf_guard_clamp_P(P) == 1);
    check("位置對角夾至 1e4",  feq(P[1][1], EKF_GUARD_P_POS_MAX));
    check("速度對角夾至 2.5e3", feq(P[4][4], EKF_GUARD_P_VEL_MAX));
    check("下限夾至 1e-6",      P[2][2] == EKF_GUARD_P_MIN);
    check("非對角仍不動",       feq(P[0][3], 123.0f));
}

static void test_clamp_state(void) {
    printf("[3] 狀態理智界限 [pE,pN,pU,vE,vN,vU]\n");
    float x[6];

    memset(x, 0, sizeof(x));
    x[2] = 500.0f; x[5] = 100.0f;
    check("典型飛行值不夾限", ekf_guard_clamp_state(x) == 0 && feq(x[2], 500.0f));

    float bad[6] = { 6000.0f, -6000.0f, 3000.0f, 300.0f, -300.0f, 400.0f };
    check("全越界：回傳 1", ekf_guard_clamp_state(bad) == 1);
    check("pE 夾至 +5000",  feq(bad[0],  EKF_GUARD_POS_H_MAX));
    check("pN 夾至 -5000",  feq(bad[1], -EKF_GUARD_POS_H_MAX));
    check("pU 夾至 +2000",  feq(bad[2],  EKF_GUARD_POS_Z_MAX));
    check("vE 夾至 +200",   feq(bad[3],  EKF_GUARD_VEL_H_MAX));
    check("vN 夾至 -200",   feq(bad[4], -EKF_GUARD_VEL_H_MAX));
    check("vU 夾至 +350",   feq(bad[5],  EKF_GUARD_VEL_Z_MAX));

    float bad2[6] = { 0, 0, -150.0f, 0, 0, -250.0f };
    ekf_guard_clamp_state(bad2);
    check("pU 下界 -100",   feq(bad2[2], EKF_GUARD_POS_Z_MIN));
    check("vU 下界 -200",   feq(bad2[5], EKF_GUARD_VEL_Z_MIN));
}

static void test_scan_nan(void) {
    printf("[4] NaN/Inf 掃描\n");
    float x[6] = {0}, P[6][6] = {{0}}, q[4] = {1, 0, 0, 0};
    for (int i = 0; i < 6; i++) P[i][i] = 1.0f;
    check("乾淨狀態回 0", ekf_guard_scan_nan(x, P, q) == 0);

    x[4] = nanf("");
    check("x 含 NaN 回 1", ekf_guard_scan_nan(x, P, q) == 1);
    x[4] = 0.0f;

    P[2][2] = INFINITY;
    check("P 對角含 Inf 回 1", ekf_guard_scan_nan(x, P, q) == 1);
    P[2][2] = 1.0f;

    q[3] = nanf("");
    check("q 含 NaN 回 1", ekf_guard_scan_nan(x, P, q) == 1);
}

int main(void) {
    printf("=== test_ekf_guard：EKF 防護最小集（P0-C） ===\n");
    test_baro_gate();
    test_clamp_P();
    test_clamp_state();
    test_scan_nan();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}

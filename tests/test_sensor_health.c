/*
 * test_sensor_health.c — 感測器健康監測單元測試（P0-D，純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 驗證 sensor_health.h 三種失效偵測的精確邊界：
 *   [1] 卡死（STUCK）：簽章無變化恰於 1.0s 置位；正常噪聲序列永不置位
 *   [2] 失流（STALE）：批次間隔 >100ms 置位；從未餵入視同 STALE
 *   [3] 範圍（RANGE）：越界持續 0.5s 置位；單發越界不置位
 *   [4] 恢復：各失效解除後位元清除
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "sensor_health.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static void test_stuck(void) {
    printf("[1] 卡死偵測（簽章 1s 無變化）\n");
    SensorMon_t m;
    sensor_mon_init(&m);

    /* 正常噪聲序列：每 10ms 簽章都在變 → 永不 STUCK（跑 5s） */
    for (uint32_t t = 0; t <= 5000; t += 10) {
        sensor_mon_feed(&m, t, (int32_t)(1000 + (t % 7)), 1U);
    }
    check("正常噪聲 5s 不置位", sensor_mon_status(&m, 5000) == 0);

    /* 凍結簽章：自 5000 起完全相同 → 恰於 +1000ms 置 STUCK */
    for (uint32_t t = 5010; t < 5990; t += 10) {
        sensor_mon_feed(&m, t, 42, 1U);
    }
    /* 注意：簽章在 5010 第一次「變為 42」（變化），之後不變 → 起算點 5010 */
    check("凍結 990ms 尚未置位", (sensor_mon_status(&m, 5990) & SENSOR_MON_STUCK) == 0);
    sensor_mon_feed(&m, 6000, 42, 1U);
    sensor_mon_feed(&m, 6010, 42, 1U);
    check("凍結滿 1.0s 置位", (sensor_mon_status(&m, 6010) & SENSOR_MON_STUCK) != 0);

    /* 恢復變化 → 立即解除 */
    sensor_mon_feed(&m, 6020, 43, 1U);
    check("簽章恢復變化即解除", (sensor_mon_status(&m, 6020) & SENSOR_MON_STUCK) == 0);
}

static void test_stale(void) {
    printf("[2] 失流偵測（批次間隔 >100ms）\n");
    SensorMon_t m;
    sensor_mon_init(&m);

    check("從未餵入 → STALE（涵蓋初始化失敗）", (sensor_mon_status(&m, 0) & SENSOR_MON_STALE) != 0);

    sensor_mon_feed(&m, 1000, 1, 1U);
    check("剛餵入不置位",        (sensor_mon_status(&m, 1000) & SENSOR_MON_STALE) == 0);
    check("間隔 100ms 邊界不置位", (sensor_mon_status(&m, 1100) & SENSOR_MON_STALE) == 0);
    check("間隔 101ms 置位",      (sensor_mon_status(&m, 1101) & SENSOR_MON_STALE) != 0);

    sensor_mon_feed(&m, 1200, 2, 1U);
    check("恢復餵入即解除",      (sensor_mon_status(&m, 1200) & SENSOR_MON_STALE) == 0);
}

static void test_range(void) {
    printf("[3] 範圍偵測（越界持續 0.5s）\n");
    SensorMon_t m;
    sensor_mon_init(&m);

    /* 單發越界（一筆壞值馬上恢復）→ 不置位 */
    sensor_mon_feed(&m, 0, 1, 1U);
    sensor_mon_feed(&m, 10, 2, 0U);
    sensor_mon_feed(&m, 20, 3, 1U);
    check("單發越界不置位", (sensor_mon_status(&m, 20) & SENSOR_MON_RANGE) == 0);

    /* 持續越界：自 100 起 → 恰於 +500ms 置位 */
    uint32_t t = 100;
    for (; t < 590; t += 10) {
        sensor_mon_feed(&m, t, (int32_t)t, 0U);
    }
    check("越界 480ms 尚未置位", (sensor_mon_status(&m, 580) & SENSOR_MON_RANGE) == 0);
    sensor_mon_feed(&m, 600, 600, 0U);
    check("越界滿 0.5s 置位",    (sensor_mon_status(&m, 600) & SENSOR_MON_RANGE) != 0);

    /* 回到範圍內 → 立即解除 */
    sensor_mon_feed(&m, 610, 610, 1U);
    check("回界即解除", (sensor_mon_status(&m, 610) & SENSOR_MON_RANGE) == 0);
}

static void test_combined(void) {
    printf("[4] 組合行為\n");
    SensorMon_t m;
    sensor_mon_init(&m);

    /* 失流時卡死同時成立（最後簽章也早已無變化） */
    sensor_mon_feed(&m, 0, 1, 1U);
    uint8_t st = sensor_mon_status(&m, 2000);
    check("斷流 2s：STALE+STUCK 同時置位",
          (st & SENSOR_MON_STALE) != 0 && (st & SENSOR_MON_STUCK) != 0);

    /* sensor_sig3：任一軸變化 → 簽章變化 */
    check("sig3 任一軸變化即不同",
          sensor_sig3(1, 2, 3) != sensor_sig3(1, 2, 4) &&
          sensor_sig3(1, 2, 3) != sensor_sig3(0, 2, 3) &&
          sensor_sig3(1, 2, 3) == sensor_sig3(1, 2, 3));
}

int main(void) {
    printf("=== test_sensor_health：感測器健康監測（P0-D） ===\n");
    test_stuck();
    test_stale();
    test_range();
    test_combined();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}

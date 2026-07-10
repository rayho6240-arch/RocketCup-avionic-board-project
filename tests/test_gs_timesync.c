/*
 * test_gs_timesync.c — 地面站時間戳對齊邏輯測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 *   [1] hhmmss → 當日毫秒換算
 *   [2] 地面牆鐘以本機 tick 內插
 *   [3] 跨日翻轉（午夜回繞）
 *   [4] rocket↔ground 偏移：首筆直接設定（含負偏移）
 *   [5] 偏移 EMA 收斂（朝目標單調逼近）
 *   [6] 火箭事件對齊到 UTC（錨點 + 偏移）
 *   [7] 未取得 GPS / 偏移前回傳 0（哨兵）
 */
#include <stdio.h>
#include "gs_timesync.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static void test_hhmmss(void) {
    printf("[1] hhmmss → 當日毫秒\n");
    check("142530 -> 51930000", gs_hhmmss_to_ms(142530) == 51930000U);
    check("000000 -> 0",        gs_hhmmss_to_ms(0) == 0U);
    check("235959 -> 86399000", gs_hhmmss_to_ms(235959) == 86399000U);
}

static void test_interp(void) {
    printf("[2] 地面牆鐘內插\n");
    GsTimeSync_t ts; GsTimeSync_Init(&ts);
    GsTimeSync_OnGpsFix(&ts, 142530, 1000);
    check("錨點時刻 == 51930000", GsTimeSync_GroundUtcMs(&ts, 1000) == 51930000U);
    check("+500ms == 51930500",  GsTimeSync_GroundUtcMs(&ts, 1500) == 51930500U);
    check("+3s == 51933000",     GsTimeSync_GroundUtcMs(&ts, 4000) == 51933000U);
}

static void test_rollover(void) {
    printf("[3] 跨日翻轉\n");
    GsTimeSync_t ts; GsTimeSync_Init(&ts);
    GsTimeSync_OnGpsFix(&ts, 235959, 1000);   /* 23:59:59 → 86399000 */
    check("午夜後 +2s 回繞到 1000", GsTimeSync_GroundUtcMs(&ts, 3000) == 1000U);
}

static void test_offset_first(void) {
    printf("[4] 偏移首筆直接設定\n");
    GsTimeSync_t ts; GsTimeSync_Init(&ts);
    GsTimeSync_OnPacket(&ts, 1000, 1200, 3);   /* rx - rocket = 200 */
    check("首筆 offset == 200", GsTimeSync_Offset(&ts) == 200);

    GsTimeSync_t ts2; GsTimeSync_Init(&ts2);
    GsTimeSync_OnPacket(&ts2, 2000, 1000, 3);  /* 負偏移 */
    check("負偏移 offset == -1000", GsTimeSync_Offset(&ts2) == -1000);
}

static void test_offset_ema(void) {
    printf("[5] 偏移 EMA 收斂\n");
    GsTimeSync_t ts; GsTimeSync_Init(&ts);
    GsTimeSync_OnPacket(&ts, 1000, 1000, 3);   /* 首筆 offset=0 */
    GsTimeSync_OnPacket(&ts, 1000, 1800, 3);   /* 目標 800：0 + (800)>>3 = 100 */
    check("一步後 offset == 100", GsTimeSync_Offset(&ts) == 100);
    int32_t prev = GsTimeSync_Offset(&ts);
    int mono = 1;
    for (int i = 0; i < 60; i++) {
        GsTimeSync_OnPacket(&ts, 1000, 1800, 3);
        int32_t cur = GsTimeSync_Offset(&ts);
        if (cur < prev) mono = 0;              /* 應單調不減 */
        prev = cur;
    }
    check("單調逼近目標", mono == 1);
    check("收斂接近 800 (>=785, <=800)",
          GsTimeSync_Offset(&ts) >= 785 && GsTimeSync_Offset(&ts) <= 800);
}

static void test_aligned_utc(void) {
    printf("[6] 火箭事件對齊 UTC\n");
    GsTimeSync_t ts; GsTimeSync_Init(&ts);
    GsTimeSync_OnGpsFix(&ts, 142530, 10000);   /* anchor_ms=51930000, tick=10000 */
    GsTimeSync_OnPacket(&ts, 1000, 1500, 3);   /* 首筆 offset = 500 */
    check("offset == 500", GsTimeSync_Offset(&ts) == 500);
    /* rocket_tick=9500 → ground_local=10000 == 錨點 tick → UTC=51930000 */
    check("rocket 9500 -> 51930000", GsTimeSync_RocketAlignedUtcMs(&ts, 9500) == 51930000U);
    /* rocket_tick=10500 → ground_local=11000 → +1000 → 51931000 */
    check("rocket 10500 -> 51931000", GsTimeSync_RocketAlignedUtcMs(&ts, 10500) == 51931000U);
    /* 事件早於錨點：rocket_tick=8500 → ground_local=9000 → -1000 → 51929000 */
    check("rocket 8500 -> 51929000", GsTimeSync_RocketAlignedUtcMs(&ts, 8500) == 51929000U);
}

static void test_sentinel(void) {
    printf("[7] 未取得 GPS/偏移前回傳 0\n");
    GsTimeSync_t ts; GsTimeSync_Init(&ts);
    check("GroundUtcMs 無錨點回 0",     GsTimeSync_GroundUtcMs(&ts, 5000) == 0U);
    check("AlignedUtc 無錨點回 0",       GsTimeSync_RocketAlignedUtcMs(&ts, 5000) == 0U);
    GsTimeSync_OnGpsFix(&ts, 142530, 1000);    /* 有 UTC 但無 offset */
    check("AlignedUtc 無 offset 回 0",   GsTimeSync_RocketAlignedUtcMs(&ts, 5000) == 0U);
}

int main(void) {
    printf("=== test_gs_timesync：地面站時間戳對齊 ===\n");
    test_hhmmss();
    test_interp();
    test_rollover();
    test_offset_first();
    test_offset_ema();
    test_aligned_utc();
    test_sentinel();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}

/*
 * test_gs_log.c — 地面站紀錄 + CSV 格式化測試（純 host 編譯）
 * ===========================================================================
 *   [1] 紀錄含完整封包；BuildRecord → RecordValid，位元翻轉 → invalid
 *   [2] CSV 表頭與資料列「欄位數一致」（逗號數相同）
 *   [3] 資料列不溢位（< GS_LOG_CSV_MAX）且含預期內容（時間字串 / 鏈路 / 縮放值）
 */
#include <stdio.h>
#include <string.h>
#include "gs_log.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}
static int count_commas(const char *s) { int n = 0; for (; *s; s++) if (*s == ',') n++; return n; }

static TelemetryPacket_t sample_pkt(void) {
    TelemetryPacket_t p; memset(&p, 0, sizeof(p));
    p.sync0 = TELEM_SYNC0; p.sync1 = TELEM_SYNC1;
    p.seq = 7; p.fsm_state = 3; p.tick_ms = 123456;
    p.ekf_pos_z_cm = 25032; p.ekf_vel_z_cms = -1850; p.baro_alt_cm = 24990;
    p.gps_lat_1e6 = 25033000; p.gps_lon_1e6 = 121564000;
    p.gps_alt_m = 250; p.gps_sats = 9; p.gps_fix = 1; p.bat_mv = 11850;
    p.flags = TELEM_FLAG_DROGUE_FIRED;
    return p;
}

static GsLogRecord_t sample_rec(void) {
    TelemetryPacket_t p = sample_pkt();
    GsLogRecord_t r;
    GsLog_BuildRecord(&r, GS_LINK_920, -85, 40,
                      /*rx_tick*/200000, /*rx_utc_ms*/51930500U,
                      /*aligned_utc_ms*/51930000U, /*offset_ms*/76544,
                      /*gs gps*/ 25030000, 121560000, 12, 8, 1, &p);
    return r;
}

static void test_record(void) {
    printf("[1] 紀錄組裝與 CRC\n");
    GsLogRecord_t r = sample_rec();
    check("magic 正確", r.magic0 == GS_LOG_MAGIC0 && r.magic1 == GS_LOG_MAGIC1);
    check("內含封包 seq 一致", r.pkt.seq == 7);
    check("RecordValid 通過", GsLog_RecordValid(&r) == 1);
    GsLogRecord_t bad = r;
    ((uint8_t *)&bad)[8] ^= 0x01;       /* 翻轉一個 byte */
    check("位元翻轉 → invalid", GsLog_RecordValid(&bad) == 0);
}

static void test_csv_columns(void) {
    printf("[2] 表頭/資料列欄位數一致\n");
    char hdr[GS_LOG_CSV_MAX], row[GS_LOG_CSV_MAX];
    int nh = GsLog_CsvHeader(hdr, sizeof(hdr));
    GsLogRecord_t r = sample_rec();
    int nr = GsLog_FormatCsvRow(row, sizeof(row), &r);
    check("表頭未截斷", nh > 0 && nh < (int)sizeof(hdr));
    check("資料列未截斷", nr > 0 && nr < (int)sizeof(row));
    check("逗號數相同（欄位對齊）", count_commas(hdr) == count_commas(row));
}

static void test_csv_content(void) {
    printf("[3] 資料列內容\n");
    char row[GS_LOG_CSV_MAX];
    GsLogRecord_t r = sample_rec();
    GsLog_FormatCsvRow(row, sizeof(row), &r);
    check("含 rx_utc 字串 14:25:30.500", strstr(row, "14:25:30.500") != NULL);
    check("含 aligned 字串 14:25:30.000", strstr(row, "14:25:30.000") != NULL);
    check("鏈路標 920",                  strstr(row, ",920,") != NULL);
    check("含火箭高度 25032",            strstr(row, ",25032,") != NULL);
    check("含 offset 76544",             strstr(row, ",76544,") != NULL);
    check("以 CRLF 結尾",                row[strlen(row)-2] == '\r' && row[strlen(row)-1] == '\n');
}

int main(void) {
    printf("=== test_gs_log：地面站紀錄 + CSV ===\n");
    test_record();
    test_csv_columns();
    test_csv_content();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}

/*
 * test_link_proto.c — 板間鏈路封包契約測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 *   [1] LinkPacket_t 大小 = 27 bytes，欄位 byte offset 逐一鎖定（解碼契約）
 *   [2] LinkProto_Build → LinkRx_Feed 往返一致（含 sync 對齊與 CRC）
 *   [3] 單一位元翻轉 → CRC 不符 → 不吐封包
 *   [4] 前綴雜訊 / 連續兩筆 → 正確對齊並解出
 *   [5] 連續 sync0 不會卡死對齊
 */
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "link_proto.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static LinkStatus_t sample_status(void) {
    LinkStatus_t st;
    st.board_id    = LINK_BOARD_PRIMARY;
    st.seq         = 0x2A;
    st.fsm_state   = 3;            /* STATE_COAST */
    st.flags       = TELEM_FLAG_DROGUE_FIRED | TELEM_FLAG_FAILSAFE;
    st.tick_ms     = 123456;
    st.h_est_cm    = 25032;        /* 250.32 m */
    st.v_est_cms   = -1850;        /* -18.50 m/s */
    st.baro_alt_cm = 24990;
    st.a_z_cg      = -981;         /* -9.81 g (cg) */
    return st;
}

static void test_layout(void) {
    printf("[1] 封包大小與欄位 offset（解碼契約）\n");
    check("sizeof(LinkPacket_t) == 26", sizeof(LinkPacket_t) == 26);
    check("LINK_PACKET_SIZE == 26",     LINK_PACKET_SIZE == 26);
#define OFF(field, expect) \
    check("offsetof " #field " == " #expect, offsetof(LinkPacket_t, field) == (expect))
    OFF(sync0,       0);
    OFF(sync1,       1);
    OFF(board_id,    2);
    OFF(seq,         3);
    OFF(fsm_state,   4);
    OFF(flags,       5);
    OFF(tick_ms,     6);
    OFF(h_est_cm,    10);
    OFF(v_est_cms,   14);
    OFF(baro_alt_cm, 18);
    OFF(a_z_cg,      22);
    OFF(crc16,       24);
#undef OFF
}

static void test_roundtrip(void) {
    printf("[2] Build -> Feed 往返一致\n");
    LinkStatus_t st = sample_status();
    uint8_t buf[LINK_PACKET_SIZE];
    uint16_t n = LinkProto_Build(buf, &st);
    check("Build 回傳長度 == 26", n == LINK_PACKET_SIZE);
    check("buf[0],buf[1] == sync", buf[0] == LINK_SYNC0 && buf[1] == LINK_SYNC1);

    LinkRx_t rx; LinkRx_Init(&rx);
    LinkPacket_t out; memset(&out, 0, sizeof(out));
    int got = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (LinkRx_Feed(&rx, buf[i], &out)) got++;
    }
    check("恰好解出 1 筆", got == 1);
    check("board_id 一致",    out.board_id    == st.board_id);
    check("seq 一致",         out.seq         == st.seq);
    check("fsm_state 一致",   out.fsm_state   == st.fsm_state);
    check("flags 一致",       out.flags       == st.flags);
    check("tick_ms 一致",     out.tick_ms     == st.tick_ms);
    check("h_est_cm 一致",    out.h_est_cm    == st.h_est_cm);
    check("v_est_cms 一致",   out.v_est_cms   == st.v_est_cms);
    check("baro_alt_cm 一致", out.baro_alt_cm == st.baro_alt_cm);
    check("a_z_cg 一致",      out.a_z_cg      == st.a_z_cg);
}

static void test_bad_crc(void) {
    printf("[3] 位元翻轉 → CRC 不符 → 不吐封包\n");
    LinkStatus_t st = sample_status();
    uint8_t buf[LINK_PACKET_SIZE];
    LinkProto_Build(buf, &st);
    buf[10] ^= 0x01;               /* 翻轉酬載一個 bit */

    LinkRx_t rx; LinkRx_Init(&rx);
    LinkPacket_t out;
    int got = 0;
    for (uint16_t i = 0; i < LINK_PACKET_SIZE; i++)
        if (LinkRx_Feed(&rx, buf[i], &out)) got++;
    check("壞 CRC 解出 0 筆", got == 0);
}

static void test_noise_and_two_frames(void) {
    printf("[4] 前綴雜訊 + 連續兩筆\n");
    LinkStatus_t st = sample_status();
    uint8_t buf[LINK_PACKET_SIZE];
    LinkProto_Build(buf, &st);

    LinkRx_t rx; LinkRx_Init(&rx);
    LinkPacket_t out;
    int got = 0;
    const uint8_t noise[] = {0x00, 0xFF, 0x12, 0xC3, 0x99, 0x5A, 0x01};
    for (size_t i = 0; i < sizeof(noise); i++) LinkRx_Feed(&rx, noise[i], &out);
    for (uint16_t i = 0; i < LINK_PACKET_SIZE; i++)
        if (LinkRx_Feed(&rx, buf[i], &out)) got++;
    for (uint16_t i = 0; i < LINK_PACKET_SIZE; i++)
        if (LinkRx_Feed(&rx, buf[i], &out)) got++;
    check("雜訊後仍解出 2 筆", got == 2);
}

static void test_consecutive_sync0(void) {
    printf("[5] 連續 sync0 不卡死對齊\n");
    LinkStatus_t st = sample_status();
    uint8_t buf[LINK_PACKET_SIZE];
    LinkProto_Build(buf, &st);

    LinkRx_t rx; LinkRx_Init(&rx);
    LinkPacket_t out;
    int got = 0;
    LinkRx_Feed(&rx, LINK_SYNC0, &out);   /* 多餘的 sync0 */
    LinkRx_Feed(&rx, LINK_SYNC0, &out);   /* 再一個 sync0 */
    for (uint16_t i = 0; i < LINK_PACKET_SIZE; i++)
        if (LinkRx_Feed(&rx, buf[i], &out)) got++;
    check("連續 sync0 後仍解出 1 筆", got == 1);
}

int main(void) {
    printf("=== test_link_proto：板間鏈路封包契約 ===\n");
    test_layout();
    test_roundtrip();
    test_bad_crc();
    test_noise_and_two_frames();
    test_consecutive_sync0();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}

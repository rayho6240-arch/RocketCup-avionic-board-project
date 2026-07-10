/*
 * test_link.c — 板間鏈路對端狀態 + 備板開傘仲裁情境測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 鎖定不對稱冗餘（主決策、備補位）的核心安全行為：
 *   [1] freshness：未收包→失聯；收包後 timeout 內新鮮、逾時失聯
 *   [2] 開傘旗標鎖存：收過一次即維持，後續無旗標封包不清除
 *   [3] 情境A：主板先開傘（已鎖存）→ 備板被抑制，永不點火
 *   [4] 情境B：主板失聯（無封包）→ 備板 grace 到期自行點火，且只點一次
 *   [5] 情境C：主板在線但從不送開傘通知 → 備板 grace 到期自行點火
 *   [6] 情境D：grace 期間才收到主板開傘 → 備板抑制
 *   [7] 點火不重複：已點火後即使 FSM 再次要求亦不重複輸出
 */
#include <stdio.h>
#include <string.h>
#include "board_config.h"
#include "link.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static LinkPacket_t make_pkt(uint8_t board_id, uint8_t fsm_state, uint8_t flags, uint32_t tick) {
    LinkPacket_t p; memset(&p, 0, sizeof(p));
    p.sync0 = LINK_SYNC0; p.sync1 = LINK_SYNC1;
    p.board_id = board_id; p.fsm_state = fsm_state; p.flags = flags; p.tick_ms = tick;
    return p;
}

static void test_freshness(void) {
    printf("[1] freshness（timeout=%u ms）\n", (unsigned)LINK_PEER_TIMEOUT_MS);
    LinkPeer_t pr; LinkPeer_Init(&pr);
    check("未收包 → 失聯", !LinkPeer_Fresh(&pr, 0, LINK_PEER_TIMEOUT_MS));
    LinkPacket_t pkt = make_pkt(LINK_BOARD_PRIMARY, 3, 0, 1000);
    LinkPeer_OnPacket(&pr, &pkt, 1000);
    check("收包當下 → 新鮮",            LinkPeer_Fresh(&pr, 1000, LINK_PEER_TIMEOUT_MS));
    check("timeout-1 內 → 新鮮",        LinkPeer_Fresh(&pr, 1000 + LINK_PEER_TIMEOUT_MS - 1, LINK_PEER_TIMEOUT_MS));
    check("達 timeout → 失聯",          !LinkPeer_Fresh(&pr, 1000 + LINK_PEER_TIMEOUT_MS, LINK_PEER_TIMEOUT_MS));
}

static void test_latch(void) {
    printf("[2] 開傘旗標鎖存\n");
    LinkPeer_t pr; LinkPeer_Init(&pr);
    LinkPacket_t a = make_pkt(LINK_BOARD_PRIMARY, 4, TELEM_FLAG_DROGUE_FIRED, 100);
    LinkPeer_OnPacket(&pr, &a, 100);
    check("收到 DROGUE_FIRED → drogue_latched", pr.drogue_latched == 1);
    check("尚未收 MAIN → main_latched=0",       pr.main_latched == 0);

    LinkPacket_t b = make_pkt(LINK_BOARD_PRIMARY, 5, 0, 200);   /* 後續無旗標 */
    LinkPeer_OnPacket(&pr, &b, 200);
    check("無旗標封包不清除 drogue 鎖存", pr.drogue_latched == 1);

    LinkPacket_t c = make_pkt(LINK_BOARD_PRIMARY, 6, TELEM_FLAG_MAIN_DEPLOYED, 300);
    LinkPeer_OnPacket(&pr, &c, 300);
    check("收到 MAIN_DEPLOYED → main_latched", pr.main_latched == 1);
    check("drogue 鎖存仍維持",                 pr.drogue_latched == 1);
}

/* 模擬 100 Hz 迴圈跑備板閘；local_wants 只在 want_at_ms 那一週期為 1（FSM 一次性）。
 * peer_latched_from_ms < 0 表示對端整段都未開傘。回傳 (首次點火時間, 點火次數)。 */
static void run_gate(uint32_t want_at_ms, long peer_latched_from_ms,
                     uint32_t end_ms, uint32_t *first_fire_ms, int *fire_count) {
    BackupGate_t g; BackupGate_Init(&g);
    *first_fire_ms = 0xFFFFFFFFu; *fire_count = 0;
    for (uint32_t t = 0; t <= end_ms; t += 10) {
        uint8_t local = (t == want_at_ms) ? 1U : 0U;
        uint8_t peer  = (peer_latched_from_ms >= 0 && t >= (uint32_t)peer_latched_from_ms) ? 1U : 0U;
        if (BackupGate_Step(&g, local, peer, t, BACKUP_GRACE_MS)) {
            if (*fire_count == 0) *first_fire_ms = t;
            (*fire_count)++;
        }
    }
}

static void test_scenario_A_peer_fired_first(void) {
    printf("[3] 情境A：主板先開傘 → 備板抑制\n");
    uint32_t first; int cnt;
    run_gate(/*want_at*/100, /*peer_latched_from*/0, /*end*/2000, &first, &cnt);
    check("主板已開傘 → 備板 0 次點火", cnt == 0);
}

static void test_scenario_B_peer_silent(void) {
    printf("[4] 情境B：主板失聯 → 備板 grace 後自行點火\n");
    uint32_t first; int cnt;
    run_gate(/*want_at*/100, /*peer_latched_from*/-1, /*end*/2000, &first, &cnt);
    check("備板恰點火 1 次", cnt == 1);
    check("點火時間 = want + grace", first == 100 + BACKUP_GRACE_MS);
}

static void test_scenario_C_peer_alive_no_fire(void) {
    printf("[5] 情境C：主板在線但從不開傘 → 備板自行點火\n");
    /* 對端封包持續到、但 flags 永遠不含開傘旗標 → 對閘而言等同未鎖存 */
    LinkPeer_t pr; LinkPeer_Init(&pr);
    BackupGate_t g; BackupGate_Init(&g);
    uint32_t first = 0xFFFFFFFFu; int cnt = 0;
    for (uint32_t t = 0; t <= 2000; t += 10) {
        if ((t % 50) == 0) {                   /* 對端 20 Hz 心跳，但無開傘旗標 */
            LinkPacket_t hb = make_pkt(LINK_BOARD_PRIMARY, 3, 0, t);
            LinkPeer_OnPacket(&pr, &hb, t);
        }
        uint8_t local = (t == 100) ? 1U : 0U;
        if (BackupGate_Step(&g, local, pr.drogue_latched, t, BACKUP_GRACE_MS)) {
            if (cnt == 0) first = t;
            cnt++;
        }
    }
    check("對端在線(fresh)但未開傘", LinkPeer_Fresh(&pr, 2000, LINK_PEER_TIMEOUT_MS));
    check("備板恰點火 1 次",        cnt == 1);
    check("點火時間 = want + grace", first == 100 + BACKUP_GRACE_MS);
}

static void test_scenario_D_peer_latched_during_grace(void) {
    printf("[6] 情境D：grace 期間收到主板開傘 → 備板抑制\n");
    uint32_t first; int cnt;
    /* want 在 100，主板於 grace 期間（200，< 100+300）才鎖存 */
    run_gate(/*want_at*/100, /*peer_latched_from*/200, /*end*/2000, &first, &cnt);
    check("grace 期間被主板搶先 → 0 次點火", cnt == 0);
}

static void test_no_double_fire(void) {
    printf("[7] 點火不重複\n");
    BackupGate_t g; BackupGate_Init(&g);
    int cnt = 0;
    /* want 在 t=100 與 t=1000 各觸發一次，主板全程未開 */
    for (uint32_t t = 0; t <= 2000; t += 10) {
        uint8_t local = (t == 100 || t == 1000) ? 1U : 0U;
        if (BackupGate_Step(&g, local, 0U, t, BACKUP_GRACE_MS)) cnt++;
    }
    check("整段僅點火 1 次（fired 鎖存）", cnt == 1);
}

int main(void) {
    printf("=== test_link：對端狀態 + 備板開傘仲裁 ===\n");
    test_freshness();
    test_latch();
    test_scenario_A_peer_fired_first();
    test_scenario_B_peer_silent();
    test_scenario_C_peer_alive_no_fire();
    test_scenario_D_peer_latched_during_grace();
    test_no_double_fire();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}

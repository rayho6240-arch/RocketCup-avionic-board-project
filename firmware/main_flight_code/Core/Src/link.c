/*
 * link.c — 板間鏈路對端狀態 + 備板開傘仲裁（純邏輯，host 可測）
 * ===========================================================================
 * 無 HAL / RTOS 依賴。tick 差以 uint32_t 無號相減，自然處理 wrap（差值 < 2^31）。
 */
#include "link.h"
#include <string.h>

void LinkPeer_Init(LinkPeer_t *p)
{
    memset(p, 0, sizeof(*p));
}

void LinkPeer_OnPacket(LinkPeer_t *p, const LinkPacket_t *pkt, uint32_t now_ms)
{
    p->valid        = 1U;
    p->board_id     = pkt->board_id;
    p->fsm_state    = pkt->fsm_state;
    p->flags        = pkt->flags;
    p->peer_tick_ms = pkt->tick_ms;
    p->last_rx_ms   = now_ms;

    /* 開傘旗標一旦收到即鎖存（共用點火頭已被主板點過，永久抑制本事件） */
    if (pkt->flags & TELEM_FLAG_DROGUE_FIRED)  p->drogue_latched = 1U;
    if (pkt->flags & TELEM_FLAG_MAIN_DEPLOYED) p->main_latched   = 1U;
}

uint8_t LinkPeer_Fresh(const LinkPeer_t *p, uint32_t now_ms, uint32_t timeout_ms)
{
    if (!p->valid) return 0U;
    return ((now_ms - p->last_rx_ms) < timeout_ms) ? 1U : 0U;
}

void BackupGate_Init(BackupGate_t *g)
{
    memset(g, 0, sizeof(*g));
}

uint8_t BackupGate_Step(BackupGate_t *g, uint8_t local_wants_fire,
                        uint8_t peer_latched, uint32_t now_ms, uint32_t grace_ms)
{
    if (g->fired) {
        return 0U;                 /* 已點過一次，永不重複 */
    }
    if (peer_latched) {
        g->pending = 0U;           /* 主板已開傘 → 抑制本事件（取消 grace） */
        return 0U;
    }
    if (local_wants_fire && !g->pending) {
        g->pending          = 1U;  /* 本板 FSM 判到開傘（一次性）→ 起算 grace */
        g->pending_since_ms = now_ms;
    }
    if (g->pending && (now_ms - g->pending_since_ms) >= grace_ms) {
        g->pending = 0U;
        g->fired   = 1U;           /* grace 到期、主板仍未開 → 備板自行點火 */
        return 1U;
    }
    return 0U;
}

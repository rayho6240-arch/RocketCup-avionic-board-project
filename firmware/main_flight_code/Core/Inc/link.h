/*
 * link.h — 板間鏈路對端狀態追蹤 + 備板開傘仲裁（純邏輯，host 可測）
 * ===========================================================================
 * 不對稱冗餘（主決策、備補位）的核心判斷集中於此，刻意保持純函式（不依賴 HAL /
 * RTOS），由 tests/test_link.c 以情境鎖定行為。HAL 端（USART2 DMA / 軟體 UART /
 * 週期廣播）由 main.c 在 FEATURE_LINK 下接線，呼叫本檔函式。
 *
 *   對端狀態：LinkPeer_OnPacket() 餵入解析成功的封包；開傘旗標一旦收到即「鎖存」
 *             （即使之後鏈路掉線仍維持抑制，因共用點火頭已被主板點過）。
 *   備板點火閘：BackupGate_Step() 每飛控週期呼叫；本板 FSM 判到開傘後先等 grace，
 *             期間若收到主板開傘鎖存 → 抑制；grace 到期仍未收到 → 自行點火。
 */
#ifndef LINK_H
#define LINK_H

#include <stdint.h>
#include "link_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* === 對端（peer）狀態 === */
typedef struct {
    uint8_t  valid;          /* 曾收過至少一筆有效封包 */
    uint8_t  board_id;       /* 對端 board_id */
    uint8_t  fsm_state;      /* 對端最近回報的 FSM 狀態 */
    uint8_t  flags;          /* 對端最近一筆 flags */
    uint8_t  peer_ack_state; /* 對端 echo 回的「它已採用的『我方』FSM 狀態」＝對端對我的 ACK */
    uint32_t peer_tick_ms;   /* 對端封包內的飛行 tick */
    uint32_t last_rx_ms;     /* 本機收到該封包時的 tick（freshness 基準） */
    uint8_t  drogue_latched; /* 對端曾通報 DROGUE_FIRED（鎖存，不清除） */
    uint8_t  main_latched;   /* 對端曾通報 MAIN_DEPLOYED（鎖存，不清除） */
} LinkPeer_t;

void    LinkPeer_Init(LinkPeer_t *p);
void    LinkPeer_OnPacket(LinkPeer_t *p, const LinkPacket_t *pkt, uint32_t now_ms);
/* 對端是否仍在線（valid 且距上次收包 < timeout_ms） */
uint8_t LinkPeer_Fresh(const LinkPeer_t *p, uint32_t now_ms, uint32_t timeout_ms);

/**
 * @brief 對端是否已確認跟進到「我方」的當前狀態（雙向心跳 echo＝ACK）。
 * @param my_state 本板當前 FSM 狀態
 * @return 1 = 對端最近封包 echo 回的 ack_state 等於 my_state（已同步）；否則 0。
 * @note  純比較，不看 freshness——呼叫端若需「新鮮且已同步」自行併 LinkPeer_Fresh()。
 */
uint8_t LinkPeer_Synced(const LinkPeer_t *p, uint8_t my_state);

/* === 備板開傘閘（drogue 與 main 各持一份狀態） === */
typedef struct {
    uint8_t  pending;          /* 本板 FSM 已判到開傘、grace 計時中 */
    uint32_t pending_since_ms; /* grace 起算 tick */
    uint8_t  fired;            /* 已實際輸出一次（防重複） */
} BackupGate_t;

void BackupGate_Init(BackupGate_t *g);

/**
 * @brief 備板開傘閘：每飛控週期呼叫一次。回傳 1 = 本週期應實際驅動點火輸出。
 * @param local_wants_fire 本板 FSM 本週期是否要求開傘（act.fire_drogue / deploy_main）
 * @param peer_latched     對端是否已通報開傘該事件（LinkPeer_t.drogue_latched / main_latched）
 * @param grace_ms         寬限期（BACKUP_GRACE_MS）
 * @note  語意：對端已開（peer_latched）→ 永久抑制本事件；否則本板想開即起算 grace，
 *        grace 到期仍未收到對端開傘 → 點火一次（之後不再重複）。
 */
uint8_t BackupGate_Step(BackupGate_t *g, uint8_t local_wants_fire,
                        uint8_t peer_latched, uint32_t now_ms, uint32_t grace_ms);

#ifdef __cplusplus
}
#endif

#endif /* LINK_H */

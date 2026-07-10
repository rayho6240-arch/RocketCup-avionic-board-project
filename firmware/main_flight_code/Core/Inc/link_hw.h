/*
 * link_hw.h — 板間鏈路硬體層（USART2 全雙工：DMA/IDLE 收 + IT 送）
 * ===========================================================================
 * 把純邏輯（link_proto / link）接到 HAL：主備兩板程式相同，差別只在角色與接線
 * （板間排線 TX/RX 交叉一次）。收到的封包於 ISR 內更新 LinkPeer_t；main.c 的備板
 * overlay 讀 Link_GetPeer() 的鎖存旗標決定是否抑制點火。
 *
 * 僅在 FEATURE_LINK 編入。
 */
#ifndef LINK_HW_H
#define LINK_HW_H

#include "board_config.h"

#if FEATURE_LINK

#include "link.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 啟動 USART2 板間鏈路（重設 baud 為 LINK_BAUD + 啟動循環 DMA/IDLE 接收）。 */
void Link_Init(void);

/* 非阻塞送出一筆自身狀態（IT 傳輸；自動補遞增 seq；上一筆未送完則略過本次）。 */
void Link_SendStatus(const LinkStatus_t *st);

/* 取得對端狀態（含鎖存的 drogue/main 開傘旗標），供備板 overlay 仲裁。 */
const LinkPeer_t *Link_GetPeer(void);

/* 對端是否仍在線（valid 且距上次收包 < LINK_PEER_TIMEOUT_MS）。 */
uint8_t Link_PeerFresh(uint32_t now_ms);

/* --- 以下由 main.c 的 HAL 回呼依 instance 轉接（ISR context） --- */
void Link_OnRxEvent(uint16_t Size);   /* HAL_UARTEx_RxEventCallback(USART2) */
void Link_OnError(void);              /* HAL_UART_ErrorCallback(USART2) */
void Link_OnTxComplete(void);         /* HAL_UART_TxCpltCallback(USART2) */

#ifdef __cplusplus
}
#endif

#endif /* FEATURE_LINK */
#endif /* LINK_HW_H */

/*
 * uplink_cmd.c — 火箭端上行命令處理（433 反向鏈路 → 手動開傘）
 * ===========================================================================
 * 整檔以 #if FEATURE_UPLINK_DEPLOY 包住：備援/地面站編譯為空。
 */
#include "board_config.h"
#if FEATURE_UPLINK_DEPLOY

#include "uplink_cmd.h"
#include "uplink_proto.h"
#include "main.h"
#include <stdio.h>

/* ARM 窗逾時：武裝後若 N ms 內未開傘，自動解除（防誤觸與遺留武裝）。 */
#ifndef UPLINK_ARM_TIMEOUT_MS
#define UPLINK_ARM_TIMEOUT_MS  30000U
#endif

extern UART_HandleTypeDef huart3;   /* E22 433 透傳（main.c 定義） */

/* ---- USART3 位元組環形緩衝：ISR 推入、遙測任務取出 ---- */
#define U3R_SZ 256U
static volatile uint8_t  s_ring[U3R_SZ];
static volatile uint16_t s_head = 0, s_tail = 0;
static uint8_t           s_rxbuf[32];   /* ReceiveToIdle 暫存 */

static UplinkRx_t s_rx;

/* ---- 武裝 / 待辦開傘 ---- */
static uint8_t           s_armed = 0;
static uint32_t          s_arm_tick = 0;
static volatile uint8_t  s_pending_drogue = 0;
static volatile uint8_t  s_pending_main   = 0;

/* ---- 診斷 ---- */
static uint8_t  s_last_cmd = 0;

static void ring_push(uint8_t b)
{
    uint16_t nh = (uint16_t)((s_head + 1U) % U3R_SZ);
    if (nh != s_tail) { s_ring[s_head] = b; s_head = nh; }   /* 滿則丟 */
}
static int ring_pop(uint8_t *b)
{
    if (s_tail == s_head) return 0;
    *b = s_ring[s_tail];
    s_tail = (uint16_t)((s_tail + 1U) % U3R_SZ);
    return 1;
}

void UplinkCmd_Init(void)
{
    UplinkRx_Init(&s_rx);
    s_armed = 0;
    s_pending_drogue = 0;
    s_pending_main = 0;
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, s_rxbuf, sizeof(s_rxbuf));
    printf("[UPLINK] 上行命令接收就緒（433 反向鏈路，ARM->DEPLOY 兩段式）\r\n");
}

void UplinkCmd_OnUart3RxEvent(uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) ring_push(s_rxbuf[i]);
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, s_rxbuf, sizeof(s_rxbuf));   /* 重新掛載 */
}

void UplinkCmd_Poll(uint32_t now_ms)
{
    /* ARM 逾時自動解除 */
    if (s_armed && (now_ms - s_arm_tick) > UPLINK_ARM_TIMEOUT_MS) {
        s_armed = 0;
        printf("[UPLINK] ARM 逾時自動解除\r\n");
    }

    uint8_t b, cmd = 0, arg = 0, seq = 0;
    while (ring_pop(&b)) {
        if (!UplinkRx_Feed(&s_rx, b, &cmd, &arg, &seq)) continue;
        s_last_cmd = cmd;
        switch (cmd) {
            case UPLINK_CMD_PING:
                printf("[UPLINK] PING seq=%u（armed=%u）\r\n", (unsigned)seq, (unsigned)s_armed);
                break;
            case UPLINK_CMD_ARM:
                s_armed = 1; s_arm_tick = now_ms;
                printf("[UPLINK] *** ARMED ***（%lus 內有效）\r\n",
                       (unsigned long)(UPLINK_ARM_TIMEOUT_MS / 1000U));
                break;
            case UPLINK_CMD_DISARM:
                s_armed = 0;
                printf("[UPLINK] DISARMED\r\n");
                break;
            case UPLINK_CMD_DEPLOY_DROGUE:
            case UPLINK_CMD_DEPLOY_MAIN:
            case UPLINK_CMD_DEPLOY_BOTH:
                if (!s_armed) {
                    printf("[UPLINK] 開傘命令忽略：未武裝（先送 ARM）\r\n");
                    break;
                }
                if (cmd != UPLINK_CMD_DEPLOY_MAIN)   s_pending_drogue = 1;
                if (cmd != UPLINK_CMD_DEPLOY_DROGUE) s_pending_main   = 1;
                printf("[UPLINK] *** 手動開傘 *** drogue=%u main=%u\r\n",
                       (unsigned)(cmd != UPLINK_CMD_DEPLOY_MAIN),
                       (unsigned)(cmd != UPLINK_CMD_DEPLOY_DROGUE));
                break;
            default:
                printf("[UPLINK] 未知命令 0x%02X\r\n", (unsigned)cmd);
                break;
        }
    }
}

uint8_t UplinkCmd_TakeDeploy(uint8_t *want_drogue, uint8_t *want_main)
{
    uint8_t any = 0;
    if (s_pending_drogue) { if (want_drogue) *want_drogue = 1; s_pending_drogue = 0; any = 1; }
    else if (want_drogue) *want_drogue = 0;
    if (s_pending_main)   { if (want_main)   *want_main   = 1; s_pending_main   = 0; any = 1; }
    else if (want_main)   *want_main = 0;
    return any;
}

uint8_t UplinkCmd_IsArmed(void) { return s_armed; }

void UplinkCmd_GetStats(uint32_t *rx_ok, uint32_t *rx_crc_err, uint8_t *last_cmd)
{
    if (rx_ok)      *rx_ok      = s_rx.ok;
    if (rx_crc_err) *rx_crc_err = s_rx.crc_err;
    if (last_cmd)   *last_cmd   = s_last_cmd;
}

#endif /* FEATURE_UPLINK_DEPLOY */

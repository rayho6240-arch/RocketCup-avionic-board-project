/*
 * uplink_cmd.c — 火箭端上行命令處理（433 反向鏈路 → 手動開傘）
 * ===========================================================================
 * 整檔以 #if FEATURE_UPLINK_DEPLOY 包住：備援/地面站編譯為空。
 */
#include "board_config.h"
#if FEATURE_UPLINK_DEPLOY

#include "uplink_cmd.h"
#include "uplink_proto.h"
#include "uplink_text_proto.h"   /* 文字上行幀（0x55/0xBB）：帶參數指令 → Parse_Serial_Command */
#include "ack_proto.h"           /* ACK 狀態碼 ACK_OK/UNKNOWN/... */
#include "main.h"
#include <stdio.h>
#include <string.h>

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

static UplinkRx_t     s_rx;       /* 二進制幀（ARM/DEPLOY/BENCH） */
static UplinkTextRx_t s_trx;      /* 文字幀（帶參數指令） */

/* ---- 武裝 / 待辦開傘 ---- */
static uint8_t           s_armed = 0;
static uint32_t          s_arm_tick = 0;
static volatile uint8_t  s_pending_drogue = 0;
static volatile uint8_t  s_pending_main   = 0;

/* ---- 待辦文字命令（診斷任務取走 → Parse_Serial_Command） ---- */
static volatile uint8_t  s_pending_text = 0;
static char              s_text[UPLINK_TEXT_MAX + 1];
static uint8_t           s_text_seq = 0;

/* ---- 待辦 bench（已過 ARM 閘；pad-only 閘由執行端 main.c 再查） ---- */
static volatile uint8_t  s_pending_bench = 0;
static uint8_t           s_bench_seq = 0;

/* ---- 待送 ACK（執行端 SetAck 寫、遙測任務 TakePendingAck 取；best-effort 單槽） ---- */
static char              s_ack_text[ACK_TEXT_MAX + 1];
static uint8_t           s_ack_len = 0;
static uint8_t           s_ack_seq = 0;
static uint8_t           s_ack_status = 0;
static volatile uint8_t  s_ack_valid = 0;   /* 最後寫，確保緩衝先填妥 */

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
    UplinkTextRx_Init(&s_trx);
    s_armed = 0;
    s_pending_drogue = 0;
    s_pending_main = 0;
    s_pending_text = 0;
    s_pending_bench = 0;
    s_ack_valid = 0;
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, s_rxbuf, sizeof(s_rxbuf));
    printf("[UPLINK] 上行命令接收就緒（433 反向鏈路：二進制 ARM->DEPLOY/BENCH + 文字指令幀）\r\n");
}

void UplinkCmd_OnUart3RxEvent(uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) ring_push(s_rxbuf[i]);
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, s_rxbuf, sizeof(s_rxbuf));   /* 重新掛載 */
}

/* USART3(433 上行 RX) 錯誤復原：清 ORE/雜訊旗標並重新掛載 ReceiveToIdle。
 * 由 main.c 的 HAL_UART_ErrorCallback 在 USART3 出錯時轉接。不做則一次溢位就讓
 * 上行接收永久停擺（之後收不到地面站的 ARM/DEPLOY 命令）。 */
void UplinkCmd_OnUart3Error(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    (void)huart3.Instance->SR;
    (void)huart3.Instance->DR;
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
    char    tseq_text[UPLINK_TEXT_MAX + 1];
    uint8_t tlen = 0, tseq = 0;
    while (ring_pop(&b)) {
        /* 同一位元組流並排餵兩個解析器：各自忽略不符自身 sync 的位元組，互不干擾。 */

        /* --- 文字命令幀（0x55/0xBB）：帶參數指令 → 交診斷任務餵 Parse_Serial_Command --- */
        if (UplinkTextRx_Feed(&s_trx, b, tseq_text, &tlen, &tseq)) {
            if (!s_pending_text) {            /* 前一筆尚未被取走則丟棄本筆（極少見；避免覆蓋） */
                memcpy(s_text, tseq_text, (size_t)tlen + 1U);
                s_text_seq = tseq;
                s_pending_text = 1;           /* 最後設，確保 s_text 已填妥 */
                printf("[UPLINK] 文字命令 seq=%u: \"%s\"\r\n", (unsigned)tseq, s_text);
            }
            continue;
        }

        /* --- 二進制幀（0x55/0xAA）：ARM/DISARM/DEPLOY/BENCH --- */
        if (!UplinkRx_Feed(&s_rx, b, &cmd, &arg, &seq)) continue;
        s_last_cmd = cmd;
        switch (cmd) {
            case UPLINK_CMD_PING:
                printf("[UPLINK] PING seq=%u（armed=%u）\r\n", (unsigned)seq, (unsigned)s_armed);
                UplinkCmd_SetAck(seq, ACK_OK, "ping");
                break;
            case UPLINK_CMD_ARM:
                s_armed = 1; s_arm_tick = now_ms;
                printf("[UPLINK] *** ARMED ***（%lus 內有效）\r\n",
                       (unsigned long)(UPLINK_ARM_TIMEOUT_MS / 1000U));
                UplinkCmd_SetAck(seq, ACK_OK, "arm");
                break;
            case UPLINK_CMD_DISARM:
                s_armed = 0;
                printf("[UPLINK] DISARMED\r\n");
                UplinkCmd_SetAck(seq, ACK_OK, "disarm");
                break;
            case UPLINK_CMD_DEPLOY_DROGUE:
            case UPLINK_CMD_DEPLOY_MAIN:
            case UPLINK_CMD_DEPLOY_BOTH:
                if (!s_armed) {
                    printf("[UPLINK] 開傘命令忽略：未武裝（先送 ARM）\r\n");
                    UplinkCmd_SetAck(seq, ACK_UNARMED, "deploy");
                    break;
                }
                if (cmd != UPLINK_CMD_DEPLOY_MAIN)   s_pending_drogue = 1;
                if (cmd != UPLINK_CMD_DEPLOY_DROGUE) s_pending_main   = 1;
                printf("[UPLINK] *** 手動開傘 *** drogue=%u main=%u\r\n",
                       (unsigned)(cmd != UPLINK_CMD_DEPLOY_MAIN),
                       (unsigned)(cmd != UPLINK_CMD_DEPLOY_DROGUE));
                UplinkCmd_SetAck(seq, ACK_OK, "deploy");
                break;
            case UPLINK_CMD_BENCH:
                /* 桌面測試（跑一次 pyro/servo 自測後回歸）：ARM 閘在此，pad-only 閘由執行端
                 * main.c 再查（那裡有 current_fsm_state）。未 ARM → 立即回 UNARMED、不排程。 */
                if (!s_armed) {
                    printf("[UPLINK] BENCH 忽略：未武裝（先送 ARM）\r\n");
                    UplinkCmd_SetAck(seq, ACK_UNARMED, "bench");
                    break;
                }
                if (!s_pending_bench) {
                    s_bench_seq = seq;
                    s_pending_bench = 1;
                    printf("[UPLINK] *** BENCH 已排程 ***（過 ARM 閘；執行端再查未起飛閘）\r\n");
                }
                break;
            default:
                printf("[UPLINK] 未知命令 0x%02X\r\n", (unsigned)cmd);
                UplinkCmd_SetAck(seq, ACK_UNKNOWN, NULL);
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

uint8_t UplinkCmd_TakeTextCmd(char *out, uint16_t sz, uint8_t *seq)
{
    if (!s_pending_text) return 0;
    if (out && sz > 0U) {
        size_t n = strlen(s_text);
        if (n > (size_t)(sz - 1U)) n = (size_t)(sz - 1U);
        memcpy(out, s_text, n);
        out[n] = '\0';
    }
    if (seq) *seq = s_text_seq;
    s_pending_text = 0;   /* 消費後清，讓下一筆可入 */
    return 1;
}

uint8_t UplinkCmd_TakeBench(uint8_t *seq)
{
    if (!s_pending_bench) return 0;
    if (seq) *seq = s_bench_seq;
    s_pending_bench = 0;
    return 1;
}

void UplinkCmd_SetAck(uint8_t seq, uint8_t status, const char *text)
{
    uint8_t len = 0;
    if (text) {
        size_t n = strlen(text);
        if (n > ACK_TEXT_MAX) n = ACK_TEXT_MAX;
        memcpy(s_ack_text, text, n);
        s_ack_text[n] = '\0';
        len = (uint8_t)n;
    } else {
        s_ack_text[0] = '\0';
    }
    s_ack_len    = len;
    s_ack_seq    = seq;
    s_ack_status = status;
    s_ack_valid  = 1;     /* 最後設，確保上面欄位已填妥 */
}

uint8_t UplinkCmd_TakePendingAck(uint8_t *seq, uint8_t *status, char *out_text, uint8_t *out_len)
{
    if (!s_ack_valid) return 0;
    if (seq)    *seq    = s_ack_seq;
    if (status) *status = s_ack_status;
    if (out_text) { memcpy(out_text, s_ack_text, (size_t)s_ack_len + 1U); }
    if (out_len)  *out_len = s_ack_len;
    s_ack_valid = 0;
    return 1;
}

uint8_t UplinkCmd_IsArmed(void) { return s_armed; }

void UplinkCmd_GetStats(uint32_t *rx_ok, uint32_t *rx_crc_err, uint8_t *last_cmd)
{
    if (rx_ok)      *rx_ok      = s_rx.ok;
    if (rx_crc_err) *rx_crc_err = s_rx.crc_err;
    if (last_cmd)   *last_cmd   = s_last_cmd;
}

#endif /* FEATURE_UPLINK_DEPLOY */

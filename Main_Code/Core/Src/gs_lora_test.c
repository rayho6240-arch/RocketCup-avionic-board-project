/*
 * gs_lora_test.c — LoRa 通訊測試模組（地面站 UART2 命令介面 + 雙鏈路統計）
 * ===========================================================================
 * 整檔以 #if IS_GROUND 包住：航電板編譯為空。
 */
#include "board_config.h"
#if IS_GROUND

#include "gs_lora_test.h"
#include "gs_log.h"        /* GS_LINK_433/920, GS_RSSI_NA, GS_SNR_NA */
#include "lora_e22.h"
#include "lora_e80.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* ============================================================
 *  UART2 環形接收緩衝
 * ============================================================ */
#define U2_RING_SZ  512U
static volatile uint8_t  s_u2_ring[U2_RING_SZ];
static volatile uint16_t s_u2_head = 0, s_u2_tail = 0;
static uint8_t           s_u2_rxbuf[64];   /* ReceiveToIdle 暫存 */

extern UART_HandleTypeDef huart2;

static void u2_push(uint8_t b)
{
    uint16_t nh = (uint16_t)((s_u2_head + 1U) % U2_RING_SZ);
    if (nh != s_u2_tail) { s_u2_ring[s_u2_head] = b; s_u2_head = nh; }
}
static int u2_pop(uint8_t *b)
{
    if (s_u2_tail == s_u2_head) return 0;
    *b = s_u2_ring[s_u2_tail];
    s_u2_tail = (uint16_t)((s_u2_tail + 1U) % U2_RING_SZ);
    return 1;
}

/* ============================================================
 *  鏈路統計
 * ============================================================ */
typedef struct {
    uint32_t pkt_ok;
    uint32_t crc_err;
    int16_t  rssi_last, rssi_min, rssi_max;
    int32_t  rssi_sum;
    int16_t  snr_last, snr_min, snr_max;
    int32_t  snr_sum;
    uint32_t first_tick_ms;
    uint32_t last_tick_ms;
} GsLinkStats_t;

static GsLinkStats_t s_stat[2];   /* [0]=E22 433, [1]=E80 920 */

static void stats_reset(void)
{
    memset(s_stat, 0, sizeof(s_stat));
    for (int i = 0; i < 2; i++) {
        s_stat[i].rssi_min =  32767;
        s_stat[i].rssi_max = -32768;
        s_stat[i].snr_min  =  32767;
        s_stat[i].snr_max  = -32768;
    }
}

/* ============================================================
 *  目前 E80 RF 參數（可被命令修改）
 * ============================================================ */
static uint32_t s_e80_freq_hz  = 920000000UL;
static uint8_t  s_e80_sf       = 0x09U;  /* SF9 */
static uint8_t  s_e80_bw       = 0x04U;  /* 125kHz */
static uint8_t  s_e80_cr       = 0x01U;  /* 4/5 */
static int8_t   s_e80_pwr_dbm  = 22;
static uint16_t s_e80_preamble = 8U;
static uint32_t s_e22_freq_mhz = 432U;

/* BW index → kHz（for LDRO 計算與顯示） */
static uint32_t bw_idx_to_khz(uint8_t idx)
{
    switch (idx) {
        case 0x00: return 8;
        case 0x08: return 10;
        case 0x01: return 16;
        case 0x09: return 21;
        case 0x02: return 31;
        case 0x0A: return 42;
        case 0x03: return 63;
        case 0x04: return 125;
        case 0x05: return 250;
        case 0x06: return 500;
        default:   return 125;
    }
}

/* ============================================================
 *  命令處理
 * ============================================================ */
static void print_help(void)
{
    printf("\r\n=== LoRa 通訊測試命令（UART2 460800baud）===\r\n"
           "  help              顯示此說明\r\n"
           "  stats             顯示雙鏈路統計\r\n"
           "  stats reset       清除統計\r\n"
           "  e22 freq <mhz>    設 E22 頻率 (410-493)\r\n"
           "  e22 show          顯示 E22 目前頻率\r\n"
           "  e80 freq <hz>     設 E80 中心頻率 Hz (e.g. 915000000)\r\n"
           "  e80 sf   <7-12>   設 E80 展頻因子\r\n"
           "  e80 bw   <idx>    設 E80 頻寬 (0=7.8k 3=62.5k 4=125k 5=250k 6=500k)\r\n"
           "  e80 cr   <1-4>    設 E80 編碼率 (1=4/5 2=4/6 3=4/7 4=4/8)\r\n"
           "  e80 pwr  <dbm>    設 E80 發射功率 (-3~22 dBm)\r\n"
           "  e80 pre  <n>      設 E80 前導碼長度 (6~65535)\r\n"
           "  e80 show          顯示 E80 目前 RF 參數\r\n"
           "=============================================\r\n");
}

static void print_stats(void)
{
    const char *names[2] = {"E22-433", "E80-920"};
    for (int i = 0; i < 2; i++) {
        GsLinkStats_t *s = &s_stat[i];
        printf("\r\n[STATS] --- %s ---\r\n", names[i]);
        printf("[STATS] pkt_ok=%lu  crc_err=%lu\r\n",
               (unsigned long)s->pkt_ok, (unsigned long)s->crc_err);

        uint32_t elapsed_ms = (s->last_tick_ms > s->first_tick_ms)
                              ? (s->last_tick_ms - s->first_tick_ms) : 0;
        if (s->pkt_ok > 0 && elapsed_ms > 0) {
            /* 封包率 = pkt_ok * 1000 / elapsed_ms (pkt/s × 10 for one decimal) */
            uint32_t rate_x10 = s->pkt_ok * 10000U / elapsed_ms;
            printf("[STATS] rate=%lu.%lu pkt/s  elapsed=%lus\r\n",
                   (unsigned long)(rate_x10 / 10),
                   (unsigned long)(rate_x10 % 10),
                   (unsigned long)(elapsed_ms / 1000));
        } else {
            printf("[STATS] rate=-- pkt/s\r\n");
        }

        if (i == GS_LINK_920 && s->pkt_ok > 0) {
            int32_t rssi_avg = s->rssi_sum / (int32_t)s->pkt_ok;
            int32_t snr_avg  = s->snr_sum  / (int32_t)s->pkt_ok;
            printf("[STATS] RSSI: last=%d min=%d max=%d avg=%d dBm\r\n",
                   (int)s->rssi_last, (int)s->rssi_min, (int)s->rssi_max, (int)rssi_avg);
            printf("[STATS] SNR:  last=%d min=%d max=%d avg=%d (×0.25dB)\r\n",
                   (int)s->snr_last, (int)s->snr_min, (int)s->snr_max, (int)snr_avg);
        } else if (i == GS_LINK_433) {
            printf("[STATS] RSSI/SNR: N/A (E22 透傳模式無此資訊)\r\n");
        }
    }
    printf("\r\n");
}

static void print_e80_params(void)
{
    uint32_t bw_khz = bw_idx_to_khz(s_e80_bw);
    uint8_t  ldro   = ((1U << s_e80_sf) * 1000U / bw_khz >= 16U) ? 1U : 0U;
    printf("[E80] freq=%lu Hz  SF%u  BW%lu kHz (idx=%u)  CR 4/%u  pwr=%d dBm  pre=%u  ldro=%u\r\n",
           (unsigned long)s_e80_freq_hz,
           (unsigned)s_e80_sf,
           (unsigned long)bw_khz, (unsigned)s_e80_bw,
           (unsigned)(s_e80_cr + 4U),
           (int)s_e80_pwr_dbm,
           (unsigned)s_e80_preamble,
           (unsigned)ldro);
}

static void print_e22_params(void)
{
    printf("[E22] freq=%lu MHz  CH=%u\r\n",
           (unsigned long)s_e22_freq_mhz,
           (unsigned)(s_e22_freq_mhz - 410U));
}

static void apply_e80_reconfig(void)
{
    print_e80_params();
    HAL_StatusTypeDef st = LoRaE80_Reconfig(s_e80_freq_hz, s_e80_sf, s_e80_bw,
                                             s_e80_cr, s_e80_pwr_dbm, s_e80_preamble);
    if (st == HAL_OK)
        printf("[E80] reconfig OK → RX restarted\r\n");
    else
        printf("[E80] reconfig FAIL (st=%d)\r\n", (int)st);
}

/* 就地把字串轉小寫 */
static void str_tolower(char *s)
{
    for (; *s; s++) *s = (char)tolower((unsigned char)*s);
}

static void dispatch_cmd(char *line)
{
    str_tolower(line);

    /* tokenize: 最多 4 個 token */
    char *tok[4] = {NULL, NULL, NULL, NULL};
    int   n = 0;
    char *p = line;
    while (*p && n < 4) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }

    if (n == 0) return;

    if (strcmp(tok[0], "help") == 0) {
        print_help();

    } else if (strcmp(tok[0], "stats") == 0) {
        if (n >= 2 && strcmp(tok[1], "reset") == 0) {
            stats_reset();
            printf("[STATS] reset OK\r\n");
        } else {
            print_stats();
        }

    } else if (strcmp(tok[0], "e22") == 0 && n >= 2) {
        if (strcmp(tok[1], "show") == 0) {
            print_e22_params();

        } else if (strcmp(tok[1], "freq") == 0 && n >= 3) {
            uint32_t mhz = (uint32_t)strtoul(tok[2], NULL, 10);
            if (mhz < 410U || mhz > 493U) {
                printf("[E22] freq 範圍 410-493 MHz\r\n");
                return;
            }
            s_e22_freq_mhz = mhz;
            HAL_StatusTypeDef st = LoRaE22_SetFreqMHz(mhz);
            if (st == HAL_OK)
                printf("[E22] freq set to %lu MHz (CH=%u) OK\r\n",
                       (unsigned long)mhz, (unsigned)(mhz - 410U));
            else
                printf("[E22] freq set FAIL (st=%d)\r\n", (int)st);
        } else {
            printf("[E22] 未知子命令，輸入 help 查看說明\r\n");
        }

    } else if (strcmp(tok[0], "e80") == 0 && n >= 2) {
        if (strcmp(tok[1], "show") == 0) {
            print_e80_params();

        } else if (strcmp(tok[1], "freq") == 0 && n >= 3) {
            uint32_t hz = (uint32_t)strtoul(tok[2], NULL, 10);
            if (hz < 862000000UL || hz > 928000000UL) {
                printf("[E80] 頻率建議範圍 862-928 MHz\r\n");
                /* 仍允許設定（用戶可能測試其他頻段） */
            }
            s_e80_freq_hz = hz;
            apply_e80_reconfig();

        } else if (strcmp(tok[1], "sf") == 0 && n >= 3) {
            uint8_t sf = (uint8_t)atoi(tok[2]);
            if (sf < 7 || sf > 12) {
                printf("[E80] SF 範圍 7-12\r\n"); return;
            }
            s_e80_sf = sf;
            apply_e80_reconfig();

        } else if (strcmp(tok[1], "bw") == 0 && n >= 3) {
            uint8_t bw = (uint8_t)strtoul(tok[2], NULL, 10);
            /* 允許直接輸入 SX126x 的 BW index 值，或常用別名 */
            /* 合法值：0x00 0x08 0x01 0x09 0x02 0x0A 0x03 0x04 0x05 0x06 */
            static const uint8_t valid_bw[] = {0x00,0x08,0x01,0x09,0x02,0x0A,0x03,0x04,0x05,0x06};
            uint8_t ok = 0;
            for (uint8_t i = 0; i < sizeof(valid_bw); i++) {
                if (bw == valid_bw[i]) { ok = 1; break; }
            }
            if (!ok) {
                printf("[E80] BW idx 合法值: 0 1 2 3 4(125k) 5(250k) 6(500k) 8 9 10\r\n");
                return;
            }
            s_e80_bw = bw;
            apply_e80_reconfig();

        } else if (strcmp(tok[1], "cr") == 0 && n >= 3) {
            uint8_t cr = (uint8_t)atoi(tok[2]);
            if (cr < 1 || cr > 4) {
                printf("[E80] CR 範圍 1-4 (1=4/5 2=4/6 3=4/7 4=4/8)\r\n"); return;
            }
            s_e80_cr = cr;
            apply_e80_reconfig();

        } else if (strcmp(tok[1], "pwr") == 0 && n >= 3) {
            int8_t pwr = (int8_t)atoi(tok[2]);
            if (pwr < -3 || pwr > 22) {
                printf("[E80] pwr 範圍 -3~22 dBm\r\n"); return;
            }
            s_e80_pwr_dbm = pwr;
            apply_e80_reconfig();

        } else if (strcmp(tok[1], "pre") == 0 && n >= 3) {
            uint16_t pre = (uint16_t)atoi(tok[2]);
            if (pre < 6) {
                printf("[E80] preamble 最小 6\r\n"); return;
            }
            s_e80_preamble = pre;
            apply_e80_reconfig();

        } else {
            printf("[E80] 未知子命令，輸入 help 查看說明\r\n");
        }

    } else {
        printf("[TEST] 未知命令 '%s'，輸入 help\r\n", tok[0]);
    }
}

/* ============================================================
 *  公開 API
 * ============================================================ */

void GsLoraTest_Init(void)
{
    stats_reset();
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_u2_rxbuf, sizeof(s_u2_rxbuf));
    printf("[TEST] LoRa 通訊測試模組就緒，輸入 help 查看命令\r\n");
}

void GsLoraTest_OnUart2RxEvent(uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) u2_push(s_u2_rxbuf[i]);
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_u2_rxbuf, sizeof(s_u2_rxbuf));
}

void GsLoraTest_UpdateStats(uint8_t link, int16_t rssi_dbm, int16_t snr_q, uint8_t crc_ok)
{
    if (link > 1U) return;
    GsLinkStats_t *s = &s_stat[link];
    uint32_t now = HAL_GetTick();

    if (!crc_ok) { s->crc_err++; return; }

    if (s->pkt_ok == 0) s->first_tick_ms = now;
    s->pkt_ok++;
    s->last_tick_ms = now;

    if (link == GS_LINK_920 && rssi_dbm != GS_RSSI_NA) {
        s->rssi_last = rssi_dbm;
        s->rssi_sum += rssi_dbm;
        if (rssi_dbm < s->rssi_min) s->rssi_min = rssi_dbm;
        if (rssi_dbm > s->rssi_max) s->rssi_max = rssi_dbm;
    }
    if (link == GS_LINK_920 && snr_q != GS_SNR_NA) {
        s->snr_last = snr_q;
        s->snr_sum += snr_q;
        if (snr_q < s->snr_min) s->snr_min = snr_q;
        if (snr_q > s->snr_max) s->snr_max = snr_q;
    }
}

void GsLoraTest_Tick(void)
{
    /* 從 UART2 環形緩衝掃描命令（\n 結尾） */
    static char  s_cmd_buf[128];
    static uint8_t s_cmd_len = 0;

    uint8_t b;
    while (u2_pop(&b)) {
        if (b == '\r') continue;   /* 跳過 CR */
        if (b == '\n') {
            s_cmd_buf[s_cmd_len] = '\0';
            if (s_cmd_len > 0) dispatch_cmd(s_cmd_buf);
            s_cmd_len = 0;
        } else {
            if (s_cmd_len < (uint8_t)(sizeof(s_cmd_buf) - 1U))
                s_cmd_buf[s_cmd_len++] = (char)b;
        }
    }
}

#endif /* IS_GROUND */

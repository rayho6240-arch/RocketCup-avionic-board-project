/**
  ******************************************************************************
  * @file           : gps.c
  * @brief          : NMEA-0183 GPS driver (USART6, DMA-to-idle RX) — 實作
  *
  * GPS（NEO-M9N）實體掛載於 USART6（PC6=TX/PC7=RX），該埠已於 CubeMX 配置
  * 循環 DMA RX（DMA2_Stream1, ch5）＋ USART6 IDLE 中斷，故採 ReceiveToIdle_DMA：
  * DMA 於背景把位元組搬進 gps_dma_buf，IDLE/半傳輸/傳輸完成事件觸發
  * HAL_UARTEx_RxEventCallback，於其中把新位元組餵入 GPS_FeedByte 組句。
  ******************************************************************************
  */
#include "gps.h"
#include "rate_monitor.h"
#include <string.h>

/* NMEA 單句最長 82 字元（含 $ 與 CRLF）。緩衝留 96 充足餘量。 */
#define GPS_LINE_MAX   96U

/* --- 模組狀態 --- */
static UART_HandleTypeDef *gps_huart = NULL;

/* USART6_RX 循環 DMA 緩衝：DMA2 僅能存取主 SRAM（非 CCM），此 static 落於 .bss 主 SRAM。
 * 256 bytes ≈ 115200 baud 下 22 ms 連續資料；GPS 每秒一陣 NMEA，IDLE/HT/TC 事件可即時排空。 */
#define GPS_DMA_BUF_SIZE  256U
static uint8_t  gps_dma_buf[GPS_DMA_BUF_SIZE];
static uint16_t gps_dma_old_pos = 0;               /* 上次已處理到的 DMA 寫入位置（環形） */

static char     gps_asm[GPS_LINE_MAX];             /* ISR 內組裝中的句子 */
static uint16_t gps_asm_len = 0;
static char     gps_ready[GPS_LINE_MAX];           /* 已就緒、待 task 解析的整句 */
static volatile uint8_t gps_line_ready = 0;        /* 1 = gps_ready 有一句待解析 */
static volatile uint32_t gps_overrun_drops = 0;    /* task 還沒解析就被新句覆蓋的次數 */

static GPS_Data_t gps_data;                        /* 對外解析結果 */

/* ------------------------------------------------------------------ */
/* 低階：RX 中斷組裝整句                                               */
/* ------------------------------------------------------------------ */

/* 由 HAL_UARTEx_RxEventCallback（ISR context）對每個新收到的位元組呼叫。
 * 維持極短：只做字元累加，遇 '\n' 把整句搬到 ready buffer 並設旗標。 */
static void GPS_FeedByte(uint8_t b)
{
    if (b == '$') {                 /* 句首：重置組裝緩衝（容錯：句中遺漏 CRLF 也能重新對齊） */
        gps_asm_len = 0;
        gps_asm[gps_asm_len++] = (char)b;
        return;
    }

    if (b == '\r' || b == '\n') {   /* 句尾 */
        if (gps_asm_len > 0) {
            if (gps_line_ready) {
                /* 上一句還沒被 task 取走 → 覆蓋並計數（GPS 1Hz、主迴圈快，理論上不會發生） */
                gps_overrun_drops++;
            }
            uint16_t n = gps_asm_len;
            if (n >= GPS_LINE_MAX) n = GPS_LINE_MAX - 1;
            memcpy(gps_ready, gps_asm, n);
            gps_ready[n] = '\0';
            gps_line_ready = 1;
            gps_asm_len = 0;
        }
        return;
    }

    if (gps_asm_len < (GPS_LINE_MAX - 1)) {
        gps_asm[gps_asm_len++] = (char)b;
    } else {
        gps_asm_len = 0;            /* 溢位（雜訊/非 NMEA）→ 丟棄重來 */
    }
}

/* ------------------------------------------------------------------ */
/* NMEA 解析小工具                                                     */
/* ------------------------------------------------------------------ */

/* 驗證 "$....*HH" 的 XOR checksum。回傳 1 = 通過。 */
static uint8_t nmea_checksum_ok(const char *s)
{
    if (s[0] != '$') return 0;
    uint8_t sum = 0;
    const char *p = s + 1;
    while (*p && *p != '*') {
        sum ^= (uint8_t)(*p);
        p++;
    }
    if (*p != '*') return 0;                 /* 無 checksum 欄位 */
    /* 解析其後兩個 hex 字元 */
    uint8_t hi = 0, lo = 0;
    char c1 = p[1], c2 = p[2];
    if (c1 >= '0' && c1 <= '9') hi = c1 - '0';
    else if (c1 >= 'A' && c1 <= 'F') hi = c1 - 'A' + 10;
    else if (c1 >= 'a' && c1 <= 'f') hi = c1 - 'a' + 10;
    else return 0;
    if (c2 >= '0' && c2 <= '9') lo = c2 - '0';
    else if (c2 >= 'A' && c2 <= 'F') lo = c2 - 'A' + 10;
    else if (c2 >= 'a' && c2 <= 'f') lo = c2 - 'a' + 10;
    else return 0;
    return ((hi << 4) | lo) == sum;
}

/* 取第 n 個逗號分隔欄位（n=0 為句型 token，如 "$GPGGA"）。
 * 複製到 out（最多 cap-1 字元 + NUL）。回傳欄位長度（0 = 空欄或不存在）。 */
static int nmea_field(const char *s, int n, char *out, int cap)
{
    int field = 0;
    const char *p = s;
    /* 移到第 n 欄起點 */
    while (field < n && *p) {
        if (*p == ',') field++;
        p++;
    }
    if (field != n) { out[0] = '\0'; return 0; }
    int len = 0;
    while (*p && *p != ',' && *p != '*' && len < cap - 1) {
        out[len++] = *p++;
    }
    out[len] = '\0';
    return len;
}

/* 解析 NMEA 經緯度 "ddmm.mmmm" / "dddmm.mmmm" → 度 ×1e6 (int32)，依半球套用正負號。
 * 手動解析（不依賴 strtod），用 double 做最終縮放以保留 ~8 位有效數字。 */
static int32_t nmea_latlon_1e6(const char *f, char hemi)
{
    const char *dot = strchr(f, '.');
    if (!dot) return 0;
    int intlen = (int)(dot - f);
    if (intlen < 3) return 0;               /* 至少 "mm." 之前要有度+分 */

    long deg = 0;
    int i = 0;
    for (; i < intlen - 2; i++) {
        if (f[i] < '0' || f[i] > '9') return 0;
        deg = deg * 10 + (f[i] - '0');
    }
    long min_int = 0;
    for (; i < intlen; i++) {
        if (f[i] < '0' || f[i] > '9') return 0;
        min_int = min_int * 10 + (f[i] - '0');
    }
    long frac = 0, fdiv = 1;
    const char *p = dot + 1;
    while (*p >= '0' && *p <= '9' && fdiv < 1000000L) {
        frac = frac * 10 + (*p - '0');
        fdiv *= 10;
        p++;
    }
    double minutes = (double)min_int + (double)frac / (double)fdiv;
    double degrees = (double)deg + minutes / 60.0;
    int32_t out = (int32_t)(degrees * 1e6 + 0.5);
    if (hemi == 'S' || hemi == 'W') out = -out;
    return out;
}

/* 極簡 atof：可選正負號 + 整數 + 小數，無指數。空字串回傳 0。 */
static float nmea_atof(const char *s)
{
    float sign = 1.0f;
    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }
    float val = 0.0f;
    while (*s >= '0' && *s <= '9') { val = val * 10.0f + (float)(*s - '0'); s++; }
    if (*s == '.') {
        s++;
        float scale = 0.1f;
        while (*s >= '0' && *s <= '9') { val += (float)(*s - '0') * scale; scale *= 0.1f; s++; }
    }
    return sign * val;
}

static int nmea_atoi(const char *s)
{
    int v = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
    return v;
}

/* 比對句型（忽略 talker ID 前綴：GP/GN/GL/GA 等）。type 例如 "GGA"。 */
static uint8_t nmea_is_type(const char *s, const char *type)
{
    /* s 形如 "$GPGGA"；句型三碼位於索引 3..5 */
    return (s[0] == '$' && strncmp(s + 3, type, 3) == 0);
}

/* ------------------------------------------------------------------ */
/* 句型解析                                                            */
/* ------------------------------------------------------------------ */

/* $--GGA: 時間,緯度,N/S,經度,E/W,fix品質,衛星數,HDOP,海拔,M,水準面分離,M,... */
static void gps_parse_gga(const char *s)
{
    char buf[16], ns[2], ew[2];

    int q = 0;
    if (nmea_field(s, 6, buf, sizeof(buf)) > 0) q = nmea_atoi(buf);
    gps_data.fix_quality = (uint8_t)q;

    if (nmea_field(s, 7, buf, sizeof(buf)) > 0) gps_data.satellites = (uint8_t)nmea_atoi(buf);

    int latlen = nmea_field(s, 2, buf, sizeof(buf));
    nmea_field(s, 3, ns, sizeof(ns));
    if (latlen > 0 && ns[0]) {
        char latbuf[16];
        memcpy(latbuf, buf, sizeof(latbuf) > (size_t)latlen ? (size_t)latlen + 1 : sizeof(latbuf));
        gps_data.lat_1e6 = nmea_latlon_1e6(latbuf, ns[0]);
    }
    int lonlen = nmea_field(s, 4, buf, sizeof(buf));
    nmea_field(s, 5, ew, sizeof(ew));
    if (lonlen > 0 && ew[0]) {
        gps_data.lon_1e6 = nmea_latlon_1e6(buf, ew[0]);
    }

    if (nmea_field(s, 1, buf, sizeof(buf)) > 0) gps_data.utc_hhmmss = (uint32_t)nmea_atoi(buf);
    if (nmea_field(s, 9, buf, sizeof(buf)) > 0) gps_data.altitude_m = nmea_atof(buf);
    if (nmea_field(s, 11, buf, sizeof(buf)) > 0) gps_data.geoid_sep_m = nmea_atof(buf);

    if (q > 0) {
        gps_data.fix_valid = 1;
        gps_data.last_fix_tick = HAL_GetTick();
    } else {
        gps_data.fix_valid = 0;
    }
}

/* $--RMC: 時間,狀態(A/V),緯度,N/S,經度,E/W,地速(knots),航向,日期,... */
static void gps_parse_rmc(const char *s)
{
    char buf[16], st[2], ns[2], ew[2];

    nmea_field(s, 2, st, sizeof(st));
    uint8_t valid = (st[0] == 'A');

    if (nmea_field(s, 1, buf, sizeof(buf)) > 0) gps_data.utc_hhmmss = (uint32_t)nmea_atoi(buf);

    if (valid) {
        int latlen = nmea_field(s, 3, buf, sizeof(buf));
        nmea_field(s, 4, ns, sizeof(ns));
        if (latlen > 0 && ns[0]) gps_data.lat_1e6 = nmea_latlon_1e6(buf, ns[0]);

        int lonlen = nmea_field(s, 5, buf, sizeof(buf));
        nmea_field(s, 6, ew, sizeof(ew));
        if (lonlen > 0 && ew[0]) gps_data.lon_1e6 = nmea_latlon_1e6(buf, ew[0]);

        if (nmea_field(s, 7, buf, sizeof(buf)) > 0) gps_data.speed_mps = nmea_atof(buf) * 0.514444f; /* knots→m/s */
        if (nmea_field(s, 8, buf, sizeof(buf)) > 0) gps_data.course_deg = nmea_atof(buf);

        gps_data.fix_valid = 1;
        gps_data.last_fix_tick = HAL_GetTick();
    } else {
        gps_data.fix_valid = 0;
    }
}

/* ------------------------------------------------------------------ */
/* 公開 API                                                            */
/* ------------------------------------------------------------------ */

void GPS_Init(UART_HandleTypeDef *huart)
{
    gps_huart = huart;
    memset(&gps_data, 0, sizeof(gps_data));
    gps_asm_len = 0;
    gps_line_ready = 0;
    gps_dma_old_pos = 0;

    if (gps_huart) {
        /* 1. 定義 UBX 設置指令 */
        /* UBX-CFG-PRT: 設置 UART1 鮑率為 460800, 8N1, 輸入/輸出為 UBX+NMEA */
        const uint8_t UBX_CFG_PRT_460800[] = {
            0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 
            0xD0, 0x08, 0x00, 0x00, 0x00, 0x08, 0x07, 0x00, 0x07, 0x00, 
            0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xBC
        };
        
        /* UBX-CFG-RATE: 設置定位頻率為 10 Hz (100ms 測量週期) */
        const uint8_t UBX_CFG_RATE_10HZ[] = {
            0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 
            0x01, 0x00, 0x7A, 0x12
        };

        /* 2. 動態鮑率協商序列 (嘗試以多種鮑率發送設置命令，以相容各類初始狀態) */
        const uint32_t try_bauds[] = {38400, 115200, 9600};
        for (int i = 0; i < 3; i++) {
            /* 切換 MCU UART 至嘗試的鮑率 */
            HAL_UART_DeInit(gps_huart);
            gps_huart->Init.BaudRate = try_bauds[i];
            HAL_UART_Init(gps_huart);
            
            /* 發送變更鮑率命令 */
            HAL_UART_Transmit(gps_huart, (uint8_t*)UBX_CFG_PRT_460800, sizeof(UBX_CFG_PRT_460800), 50);
            HAL_Delay(10);
        }

        /* 3. 將 MCU UART 固定於最終目標鮑率 460800 */
        HAL_UART_DeInit(gps_huart);
        gps_huart->Init.BaudRate = 460800;
        HAL_UART_Init(gps_huart);
        HAL_Delay(20);

        /* 4. 在 460800 鮑率下，發送設置定位更新率為 10Hz 命明 */
        HAL_UART_Transmit(gps_huart, (uint8_t*)UBX_CFG_RATE_10HZ, sizeof(UBX_CFG_RATE_10HZ), 50);
        HAL_Delay(10);

        /* 5. 啟動 IDLE-line + 循環 DMA 接收 */
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, gps_dma_buf, GPS_DMA_BUF_SIZE);
    }
}

uint8_t GPS_Update(void)
{
    if (!gps_line_ready) return 0;

    /* 取出就緒句（短暫複製避免與 ISR 競爭：ISR 只在 ready→覆蓋時動 gps_ready，
     * 此處複製後立即清旗標，最壞情況是丟一句並由 gps_overrun_drops 計數）。 */
    char line[GPS_LINE_MAX];
    memcpy(line, gps_ready, GPS_LINE_MAX);
    gps_line_ready = 0;

    if (!nmea_checksum_ok(line)) {
        gps_data.sentences_err++;
        return 1;
    }

    if (nmea_is_type(line, "GGA")) {
        gps_parse_gga(line);
        gps_data.sentences_ok++;
        RATE_TICK_GPS();
    } else if (nmea_is_type(line, "RMC")) {
        gps_parse_rmc(line);
        gps_data.sentences_ok++;
        RATE_TICK_GPS();
    }
    /* 其他句型（GSV/GSA/VTG...）目前略過 */
    return 1;
}

const GPS_Data_t* GPS_GetData(void)
{
    return &gps_data;
}

uint8_t GPS_IsStale(uint32_t timeout_ms)
{
    if (!gps_data.fix_valid) return 1;
    return (HAL_GetTick() - gps_data.last_fix_tick) > timeout_ms;
}

/* ------------------------------------------------------------------ */
/* HAL UART 回呼（弱符號覆寫；以 instance 過濾僅處理 USART6）           */
/* ------------------------------------------------------------------ */

/* 循環 DMA 接收事件回呼：IDLE-line、半傳輸 (HT)、傳輸完成 (TC) 皆觸發。
 * Size = 自緩衝起點至目前 DMA 寫入位置的累計位元組數；以 gps_dma_old_pos 差分取出新位元組。
 * 於 ISR context 執行；僅做位元組搬移與旗標設定，不呼叫任何 FreeRTOS API（ISR-safe）。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (!gps_huart || huart->Instance != gps_huart->Instance) return;

    if (Size != gps_dma_old_pos) {
        if (Size > gps_dma_old_pos) {
            for (uint16_t i = gps_dma_old_pos; i < Size; i++) GPS_FeedByte(gps_dma_buf[i]);
        } else {
            /* 環形回繞：先處理 old_pos..尾端，再 0..Size */
            for (uint16_t i = gps_dma_old_pos; i < GPS_DMA_BUF_SIZE; i++) GPS_FeedByte(gps_dma_buf[i]);
            for (uint16_t i = 0; i < Size; i++) GPS_FeedByte(gps_dma_buf[i]);
        }
        gps_dma_old_pos = Size;
        if (gps_dma_old_pos >= GPS_DMA_BUF_SIZE) gps_dma_old_pos = 0;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (gps_huart && huart->Instance == gps_huart->Instance) {
        /* UART 錯誤（常見為 overrun）會中止 DMA 接收 → 清狀態並重啟，避免 GPS RX 死掉 */
        __HAL_UART_CLEAR_OREFLAG(gps_huart);
        gps_asm_len = 0;
        gps_dma_old_pos = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, gps_dma_buf, GPS_DMA_BUF_SIZE);
    }
}

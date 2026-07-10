/*
 * gs_timesync.h — 地面站時間戳自動對齊（純邏輯，host 可測）
 * ===========================================================================
 * 火箭下行封包帶的是「開機相對毫秒」tick_ms（HAL_GetTick）；地面站要把它自動轉成
 * GPS 紀律的牆鐘時間，讓 LoRa 紀錄能和地面 GPS 軌跡、絕對時間對齊。做法分兩層：
 *
 *   (1) 地面牆鐘：地面站自身 GPS 每次有效定位提供 UTC hhmmss（秒級）與當下本機 tick，
 *       以此錨定「當日 UTC 毫秒」；兩次定位之間用本機 tick 內插到毫秒解析度（GPS 10Hz
 *       會頻繁重錨）。跨日翻轉以一日毫秒數取模處理。
 *   (2) rocket↔ground 偏移：每筆 CRC 通過的封包，offset = 本機收包 tick − 封包 tick_ms，
 *       以 EMA 平滑吸收鏈路抖動。火箭事件的牆鐘時間 = 由 (rocket_tick + offset) 經 (1) 換算。
 *
 * 純 header-only（仿 gps_parse.h / crc16.h），不依賴 HAL / RTOS，可由
 * tests/test_gs_timesync.c 驗證。所有量為整數毫秒（遵專案黃金法則：避免浮點）。
 */
#ifndef GS_TIMESYNC_H
#define GS_TIMESYNC_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GS_DAY_MS  86400000UL   /* 一日毫秒數（24*60*60*1000） */

typedef struct {
    /* (1) 地面牆鐘錨點 */
    uint8_t  utc_valid;       /* 是否已取得過有效 GPS UTC 錨點 */
    uint32_t utc_anchor_ms;   /* 錨點「當日 UTC 毫秒」（hhmmss → ms-of-day；NMEA 無亞秒） */
    uint32_t tick_at_anchor;  /* 取得該 UTC 時的本機 tick (ms) */

    /* (2) rocket↔ground 偏移（EMA 平滑） */
    uint8_t  off_valid;       /* 是否已有偏移估計 */
    int32_t  offset_ms;       /* 本機 tick − 火箭 tick_ms（平滑後整數 ms，可正可負） */
} GsTimeSync_t;

static inline void GsTimeSync_Init(GsTimeSync_t *ts)
{
    memset(ts, 0, sizeof(*ts));
}

/* UTC hhmmss（整數，如 142530 = 14:25:30）→ 當日毫秒（秒級，*1000）。 */
static inline uint32_t gs_hhmmss_to_ms(uint32_t hhmmss)
{
    uint32_t hh = hhmmss / 10000U;
    uint32_t mm = (hhmmss / 100U) % 100U;
    uint32_t ss = hhmmss % 100U;
    return ((hh * 3600U) + (mm * 60U) + ss) * 1000U;
}

/* GPS 有效定位時呼叫：以 UTC + 當下本機 tick 設定/更新地面牆鐘錨點。 */
static inline void GsTimeSync_OnGpsFix(GsTimeSync_t *ts, uint32_t utc_hhmmss,
                                       uint32_t fix_tick_ms)
{
    ts->utc_anchor_ms  = gs_hhmmss_to_ms(utc_hhmmss);
    ts->tick_at_anchor = fix_tick_ms;
    ts->utc_valid      = 1U;
}

/* 取目前地面牆鐘「當日 UTC 毫秒」（以本機 tick 內插，跨日取模）。
 * 未取得 GPS 錨點時回傳 0（呼叫端應檢查 utc_valid）。 */
static inline uint32_t GsTimeSync_GroundUtcMs(const GsTimeSync_t *ts, uint32_t now_tick_ms)
{
    if (!ts->utc_valid) return 0U;
    uint32_t elapsed = now_tick_ms - ts->tick_at_anchor;   /* 本機 tick 單調遞增 → 非負 */
    return (uint32_t)(((uint64_t)ts->utc_anchor_ms + elapsed) % GS_DAY_MS);
}

/* 每筆 CRC 通過封包呼叫：更新 rocket↔ground 偏移 EMA。
 * @param rocket_tick_ms 封包內的火箭 tick_ms
 * @param rx_tick_ms     本機收到該封包時的 tick
 * @param ema_shift      EMA 係數（alpha = 1/2^shift；首筆直接設定）
 */
static inline void GsTimeSync_OnPacket(GsTimeSync_t *ts, uint32_t rocket_tick_ms,
                                       uint32_t rx_tick_ms, uint8_t ema_shift)
{
    int32_t raw = (int32_t)(rx_tick_ms - rocket_tick_ms);  /* uint32 環繞相減，再轉 signed */
    if (!ts->off_valid) {
        ts->offset_ms = raw;
        ts->off_valid = 1U;
    } else {
        ts->offset_ms += (raw - ts->offset_ms) >> ema_shift;  /* 算術右移（host/ARM 皆保號） */
    }
}

static inline int32_t GsTimeSync_Offset(const GsTimeSync_t *ts) { return ts->offset_ms; }

/* 火箭事件（rocket_tick_ms）對齊後的「當日 UTC 毫秒」。
 * 需 utc_valid 且 off_valid；否則回傳 0。 */
static inline uint32_t GsTimeSync_RocketAlignedUtcMs(const GsTimeSync_t *ts,
                                                     uint32_t rocket_tick_ms)
{
    if (!ts->utc_valid || !ts->off_valid) return 0U;
    /* 火箭事件 → 等效本機 tick → 經錨點換 UTC（事件可能早於錨點，用有號運算後取模） */
    int64_t ground_local = (int64_t)rocket_tick_ms + ts->offset_ms;
    int64_t utc = (int64_t)ts->utc_anchor_ms + (ground_local - (int64_t)ts->tick_at_anchor);
    utc %= (int64_t)GS_DAY_MS;
    if (utc < 0) utc += (int64_t)GS_DAY_MS;
    return (uint32_t)utc;
}

#ifdef __cplusplus
}
#endif

#endif /* GS_TIMESYNC_H */

/*
 * sensor_health.h — 感測器健康監測最小集（P0-D，純 header-only，host 可測）
 * ===========================================================================
 * 修復 改進計劃.md R4：原本沒有任何感測器失效偵測 —— 卡死/失流的感測器會
 * 持續把陳舊值餵進 EKF 與 FSM，零察覺（rate_monitor.h 僅供診斷顯示）。
 *
 * 偵測三種失效模式（每感測器一個 SensorMon_t）：
 *   STALE：超過 100ms 沒有新批次/樣本（正常 10ms 一批；含初始化失敗從未餵入）
 *   STUCK：raw 簽章連續 ≥1s 完全相同（ADC 噪聲使正常感測器幾乎不可能）
 *   RANGE：讀值持續 0.5s 超出物理合理範圍
 *
 * 消費端（偵測而不消費沒有意義）：
 *   1. BMP388 fault → EKF 樣本 has_baro=0（→ EKF_HB_BARO_TIMEOUT → FSM 降級鏈）
 *   2. FSM 輸入 sensor_bits（FSM_SB_BARO_FAULT：閘控 baro 交叉檢查/起飛冗餘）
 *   3. telemetry TELEM_FLAG_SENSOR_FAULT + 1Hz [HEALTH] 診斷行
 *
 * 純函式無 HAL 依賴，由 tests/test_sensor_health.c 驗證。
 */
#ifndef SENSOR_HEALTH_H
#define SENSOR_HEALTH_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === 閾值 === */
#define SH_STALE_MS   100U   /* 批次間隔上限（正常 10ms） */
#define SH_STUCK_MS   1000U  /* raw 簽章無變化上限 */
#define SH_RANGE_MS   500U   /* 範圍越界持續門檻 */

/* === sensor_mon_status() 回傳位元（0 = 健康） === */
#define SENSOR_MON_STALE  0x01U
#define SENSOR_MON_STUCK  0x02U
#define SENSOR_MON_RANGE  0x04U

/* === 系統層感測器故障位（main.c g_sensor_fault_bits / [HEALTH] 行） === */
#define SH_BIT_BMI088     0x01U
#define SH_BIT_ADXL375    0x02U
#define SH_BIT_BMP388     0x04U

typedef struct {
    uint32_t last_feed_ms;     /* 最後餵入時刻 */
    uint32_t last_change_ms;   /* raw 簽章最後變化時刻 */
    int32_t  last_sig;         /* 上次 raw 簽章 */
    uint32_t range_bad_since;  /* 範圍越界起始時刻（配合 range_bad 旗標） */
    uint8_t  range_bad;        /* 1 = 目前越界中 */
    uint8_t  inited;           /* 0 = 從未餵入（視同 STALE） */
} SensorMon_t;

static inline void sensor_mon_init(SensorMon_t *m)
{
    memset(m, 0, sizeof(*m));
}

/* 把多軸 raw 讀值壓成一個簽章：任一軸變化即簽章變化（碰撞機率可忽略） */
static inline int32_t sensor_sig3(int32_t a, int32_t b, int32_t c)
{
    return (a * 31 + b) * 31 + c;
}

/*
 * 每收到一筆新批次/樣本呼叫一次。
 *   sig      ：raw 簽章（sensor_sig3 或直接 raw 值；卡死偵測比對用）
 *   range_ok ：本筆讀值是否在物理合理範圍內
 */
static inline void sensor_mon_feed(SensorMon_t *m, uint32_t now_ms,
                                   int32_t sig, uint8_t range_ok)
{
    if (!m->inited) {
        m->inited         = 1U;
        m->last_sig       = sig;
        m->last_change_ms = now_ms;
    } else if (sig != m->last_sig) {
        m->last_sig       = sig;
        m->last_change_ms = now_ms;
    }

    if (range_ok) {
        m->range_bad = 0U;
    } else if (!m->range_bad) {
        m->range_bad       = 1U;
        m->range_bad_since = now_ms;
    }

    m->last_feed_ms = now_ms;
}

/* 查詢健康狀態（任意頻率呼叫）。回傳 SENSOR_MON_* 位元組合，0 = 健康。 */
static inline uint8_t sensor_mon_status(const SensorMon_t *m, uint32_t now_ms)
{
    if (!m->inited) {
        return SENSOR_MON_STALE;   /* 從未收到資料（含初始化失敗） */
    }
    uint8_t st = 0U;
    if ((now_ms - m->last_feed_ms) > SH_STALE_MS) {
        st |= SENSOR_MON_STALE;
    }
    if ((now_ms - m->last_change_ms) >= SH_STUCK_MS) {
        st |= SENSOR_MON_STUCK;
    }
    if (m->range_bad && (now_ms - m->range_bad_since) >= SH_RANGE_MS) {
        st |= SENSOR_MON_RANGE;
    }
    return st;
}

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_HEALTH_H */

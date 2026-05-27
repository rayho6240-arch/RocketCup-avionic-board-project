/**
 ******************************************************************************
 * @file    rate_monitor.h
 * @brief   輕量採樣率監測模組
 *
 * 使用方式：
 *   - 啟用：確保 #define RATE_MONITOR_ENABLE 未被註解（預設啟用）
 *   - 停用：把 #define RATE_MONITOR_ENABLE 這行註解掉
 *           → 所有巨集展開為空，零效能影響、零記憶體佔用
 *
 * 觀察方式（IDE Watch / Live Expressions）：
 *   加入 1 個變數：  g_sampling_rate
 *   展開後可看到：
 *     g_sampling_rate.bmi088.rate_hz    → BMI088  實測 Hz
 *     g_sampling_rate.adxl375.rate_hz   → ADXL375 實測 Hz
 *     g_sampling_rate.bmp388.rate_hz    → BMP388  實測 Hz
 ******************************************************************************
 */
#ifndef RATE_MONITOR_H
#define RATE_MONITOR_H

/* ============================================================
 *  ★ 控制開關：註解此行即可完全停用所有監測邏輯
 * ============================================================ */
#define RATE_MONITOR_ENABLE

/* ============================================================
 *  啟用路徑
 * ============================================================ */
#ifdef RATE_MONITOR_ENABLE

#include <stdint.h>
#include "stm32f4xx_hal.h"   /* for HAL_GetTick() */

/** 單一感測器的採樣率監測器 */
typedef struct {
    uint32_t count;        /**< 當前統計窗口內累積的呼叫次數 */
    uint32_t last_tick_ms; /**< 上次更新統計時的 HAL tick (ms) */
    float    rate_hz;      /**< 最近測量到的採樣率 (Hz)，每 ~1s 更新一次 */
} RateMonitor_t;

/**
 * @brief 所有感測器採樣率打包成一個結構體
 *        在 IDE Watch 視窗只需加入 g_sampling_rate 即可一次觀察全部
 */
typedef struct {
    RateMonitor_t bmi088_acc;  /**< BMI088 Accel 實測採樣率 */
    RateMonitor_t bmi088_gyro; /**< BMI088 Gyro 實測採樣率 */
    RateMonitor_t adxl375;     /**< ADXL375 (High-G Accel) 實測採樣率 */
    RateMonitor_t bmp388;      /**< BMP388  (Baro + Temp)  實測採樣率 */
} SamplingRateAll_t;

/** 全域實例 —— 宣告於 main.c，extern 供其他檔案參考 */
extern SamplingRateAll_t g_sampling_rate;

/* --- 底層 inline helper --- */
static inline void RateMonitor_Init(RateMonitor_t *mon)
{
    mon->count        = 0U;
    mon->last_tick_ms = HAL_GetTick();
    mon->rate_hz      = 0.0f;
}

static inline void RateMonitor_Tick(RateMonitor_t *mon)
{
    mon->count++;
    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - mon->last_tick_ms;
    if (elapsed >= 1000U) {
        mon->rate_hz      = (float)mon->count * 1000.0f / (float)elapsed;
        mon->count        = 0U;
        mon->last_tick_ms = now;
    }
}

/* --- 公開巨集 (啟用時展開為實際呼叫) --- */
/** 宣告全域 SamplingRateAll_t g_sampling_rate */
#define SAMPLING_RATE_DECL()      SamplingRateAll_t g_sampling_rate

/** 初始化所有監測器（在 task for 迴圈前呼叫） */
#define SAMPLING_RATE_INIT()      do { \
    RateMonitor_Init(&g_sampling_rate.bmi088_acc);  \
    RateMonitor_Init(&g_sampling_rate.bmi088_gyro); \
    RateMonitor_Init(&g_sampling_rate.adxl375); \
    RateMonitor_Init(&g_sampling_rate.bmp388);  \
} while(0)

/** 每次成功讀取後呼叫對應的 TICK */
#define RATE_TICK_BMI088_ACC()    RateMonitor_Tick(&g_sampling_rate.bmi088_acc)
#define RATE_TICK_BMI088_GYRO()   RateMonitor_Tick(&g_sampling_rate.bmi088_gyro)
#define RATE_TICK_BMI088()        RATE_TICK_BMI088_GYRO() /* Backward compatibility */
#define RATE_TICK_ADXL375()       RateMonitor_Tick(&g_sampling_rate.adxl375)
#define RATE_TICK_BMP388()        RateMonitor_Tick(&g_sampling_rate.bmp388)

/* ============================================================
 *  停用路徑：所有巨集展開為空，完全零開銷
 * ============================================================ */
#else  /* RATE_MONITOR_ENABLE not defined */

#define SAMPLING_RATE_DECL()      /* rate_monitor disabled */
#define SAMPLING_RATE_INIT()      /* rate_monitor disabled */
#define RATE_TICK_BMI088_ACC()    /* rate_monitor disabled */
#define RATE_TICK_BMI088_GYRO()   /* rate_monitor disabled */
#define RATE_TICK_BMI088()        /* rate_monitor disabled */
#define RATE_TICK_ADXL375()       /* rate_monitor disabled */
#define RATE_TICK_BMP388()        /* rate_monitor disabled */

#endif  /* RATE_MONITOR_ENABLE */

#endif  /* RATE_MONITOR_H */

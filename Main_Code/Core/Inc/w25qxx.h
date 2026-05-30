/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    w25qxx.h
 * @brief   W25Qxx SPI Flash 驅動程式標頭檔
 *          適用晶片: W25Q16 / W25Q32 / W25Q64 / W25Q128
 *          SPI Bus  : SPI1 (PA5=SCK, PA6=MISO, PA7=MOSI)
 *          CS Pin   : PA15 (FLASH_CSB)
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __W25QXX_H
#define __W25QXX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 *  硬體設定 — 若接線改變只需修改這裡
 * ============================================================ */
#define W25QXX_SPI_HANDLE       hspi3
#define W25QXX_CS_GPIO_PORT     GPIOA
#define W25QXX_CS_GPIO_PIN      GPIO_PIN_15
#define W25QXX_SPI_TIMEOUT_MS   100U

/* ============================================================
 *  W25Qxx 指令集 (Instruction Set)
 * ============================================================ */
#define W25QXX_CMD_WRITE_ENABLE         0x06U
#define W25QXX_CMD_WRITE_DISABLE        0x04U
#define W25QXX_CMD_READ_STATUS_REG1     0x05U
#define W25QXX_CMD_READ_STATUS_REG2     0x35U
#define W25QXX_CMD_WRITE_STATUS_REG     0x01U
#define W25QXX_CMD_READ_DATA            0x03U
#define W25QXX_CMD_FAST_READ            0x0BU
#define W25QXX_CMD_PAGE_PROGRAM         0x02U
#define W25QXX_CMD_SECTOR_ERASE_4KB    0x20U
#define W25QXX_CMD_BLOCK_ERASE_32KB    0x52U
#define W25QXX_CMD_BLOCK_ERASE_64KB    0xD8U
#define W25QXX_CMD_CHIP_ERASE           0xC7U
#define W25QXX_CMD_JEDEC_ID             0x9FU
#define W25QXX_CMD_READ_UID             0x4BU
#define W25QXX_CMD_POWER_DOWN           0xB9U
#define W25QXX_CMD_RELEASE_POWER_DOWN   0xABU

/* ============================================================
 *  Status Register 1 位元定義
 * ============================================================ */
#define W25QXX_SR1_BUSY     (1U << 0)   /* 1 = 正在執行寫入/擦除 */
#define W25QXX_SR1_WEL      (1U << 1)   /* 1 = Write Enable Latch 已設定 */

/* ============================================================
 *  記憶體規格常數
 * ============================================================ */
#define W25QXX_PAGE_SIZE        256U    /* 每頁 256 bytes */
#define W25QXX_SECTOR_SIZE      4096U   /* 每個 Sector 4 KB */
#define W25QXX_BLOCK_SIZE_32K   32768U  /* 32 KB Block */
#define W25QXX_BLOCK_SIZE_64K   65536U  /* 64 KB Block */

/* ============================================================
 *  JEDEC Manufacturer ID
 * ============================================================ */
#define W25QXX_MANUFACTURER_ID  0xEFU   /* Winbond */

/* ============================================================
 *  回傳狀態
 * ============================================================ */
typedef enum {
    W25QXX_OK           = 0,
    W25QXX_ERR_SPI      = 1,   /* SPI 傳輸失敗 */
    W25QXX_ERR_TIMEOUT  = 2,   /* 等待 BUSY 逾時 */
    W25QXX_ERR_ID       = 3,   /* JEDEC ID 不符 */
    W25QXX_ERR_PARAM    = 4,   /* 參數錯誤 */
} W25QXX_StatusTypeDef;

/* ============================================================
 *  Flash 裝置資訊結構
 * ============================================================ */
typedef struct {
    uint8_t  ManufacturerID;    /* 0xEF = Winbond */
    uint16_t DeviceID;          /* e.g. 0x4017 = W25Q64 */
    uint32_t Capacity_KB;       /* 容量 (KB) */
    uint32_t SectorCount;       /* Sector 總數 */
    bool     Initialized;       /* 初始化成功旗標 */
} W25QXX_InfoTypeDef;

/* ============================================================
 *  外部 SPI handle 宣告 (由 CubeMX spi.c 定義)
 * ============================================================ */
extern SPI_HandleTypeDef W25QXX_SPI_HANDLE;

/* ============================================================
 *  函式宣告
 * ============================================================ */

/**
 * @brief  初始化 Flash 並讀取 JEDEC ID
 * @param  info  指向 W25QXX_InfoTypeDef 結構，初始化後填入裝置資訊
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_Init(W25QXX_InfoTypeDef *info);

/**
 * @brief  讀取 JEDEC ID (3 bytes: Manufacturer + Device)
 * @param  manufacturer  輸出 Manufacturer ID
 * @param  deviceID      輸出 Device ID (2 bytes)
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_ReadJEDEC_ID(uint8_t *manufacturer, uint16_t *deviceID);

/**
 * @brief  從指定位址讀取資料
 * @param  addr    起始位址 (24-bit)
 * @param  buf     輸出緩衝區
 * @param  len     要讀取的 byte 數
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_ReadData(uint32_t addr, uint8_t *buf, uint32_t len);

/**
 * @brief  寫入一頁資料 (最多 256 bytes，不可跨頁)
 * @param  addr    起始位址 (需對齊或注意跨頁)
 * @param  buf     輸入資料
 * @param  len     長度 (1~256)
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_WritePage(uint32_t addr, const uint8_t *buf, uint16_t len);

/**
 * @brief  寫入任意長度資料 (自動處理跨頁)
 * @note   寫入前請確保目標區域已被擦除 (值為 0xFF)
 * @param  addr    起始位址
 * @param  buf     輸入資料
 * @param  len     長度
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_WriteData(uint32_t addr, const uint8_t *buf, uint32_t len);

/**
 * @brief  擦除一個 Sector (4 KB)
 * @param  sectorAddr  Sector 起始位址 (需為 4096 的倍數)
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_EraseSector(uint32_t sectorAddr);

/**
 * @brief  擦除整顆 Flash (時間較長，約 20~100 秒)
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_EraseChip(void);

/**
 * @brief  讀取 Status Register 1
 * @param  status  輸出 status byte
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_ReadStatusReg1(uint8_t *status);

/**
 * @brief  等待 Flash 完成操作 (輪詢 BUSY bit)
 * @param  timeout_ms  最長等待時間 (ms)
 * @retval W25QXX_OK 或 W25QXX_ERR_TIMEOUT
 */
W25QXX_StatusTypeDef W25QXX_WaitForReady(uint32_t timeout_ms);

/**
 * @brief  Flash 功能測試 — 在 UART2 輸出測試結果
 *         流程: 讀 JEDEC ID → Erase Sector 0 → 寫入測試資料 → 讀回比對
 */
void Flash_Test(void);

/* ============================================================
 *  Flash 記憶體分區位址（W25Q128JV, 16 MB）
 * ============================================================ */
#define FLASH_RINGBUF_ADDR       0x010000UL   /* Ring Buffer 起始（64KB 對齊） */
#define FLASH_RINGBUF_END        0xFFFFFFUL   /* Ring Buffer 結束 */
#define FLASH_RINGBUF_SIZE       0xFF0000UL   /* ~15.9 MB */
#define FLASH_RING_PACKET_SIZE   52UL         /* 每筆封包大小（bytes） */
#define FLASH_RING_PREERASE_N    10           /* 開機預擦 Sector 數量（10×4KB = 40KB） */

/* ============================================================
 *  飛行數據封包（52 bytes, packed）
 *  magic[0..1] = 0xAA 0x55 作為環形掃描標記
 *  CRC-16/CCITT 覆蓋 bytes [0..49]，存在 bytes [50..51]
 * ============================================================ */
typedef struct __attribute__((packed)) {
    uint8_t  magic[2];       /* [0..1]   0xAA 0x55                */
    uint16_t seq;            /* [2..3]   封包序號（滾動）          */
    uint32_t tick_ms;        /* [4..7]   HAL_GetTick() ms          */
    int16_t  bmi_ax;         /* [8..9]   BMI088 Accel X raw LSB    */
    int16_t  bmi_ay;         /* [10..11] BMI088 Accel Y raw        */
    int16_t  bmi_az;         /* [12..13] BMI088 Accel Z raw        */
    int16_t  bmi_gx;         /* [14..15] BMI088 Gyro X raw         */
    int16_t  bmi_gy;         /* [16..17] BMI088 Gyro Y raw         */
    int16_t  bmi_gz;         /* [18..19] BMI088 Gyro Z raw         */
    int16_t  adxl_x;         /* [20..21] ADXL375 X raw             */
    int16_t  adxl_y;         /* [22..23] ADXL375 Y raw             */
    int16_t  adxl_z;         /* [24..25] ADXL375 Z raw             */
    int32_t  temperature;    /* [26..29] BMP388 temp ×100 (°C)     */
    uint32_t pressure;       /* [30..33] BMP388 pressure (Pa)      */
    int32_t  altitude_cm;    /* [34..37] altitude ×100 (cm)        */
    int32_t  gps_lat;        /* [38..41] GPS lat ×1e6 (預留)       */
    int32_t  gps_lon;        /* [42..45] GPS lon ×1e6 (預留)       */
    uint8_t  fsm_state;      /* [46]     飛行狀態機 state           */
    uint8_t  flags;          /* [47]     雜旗標                    */
    uint16_t reserved;       /* [48..49] 保留                      */
    uint16_t crc16;          /* [50..51] CRC-16/CCITT [0..49]      */
} FlashRingPacket_t;

/* ============================================================
 *  環形緩衝區 API
 * ============================================================ */

/**
 * @brief  初始化環形緩衝區：掃描寫入頭、預擦 FLASH_RING_PREERASE_N 個 Sector
 *         輸出 [FLASH_RING] 訊息至 UART
 */
void FlashRing_Init(void);

/**
 * @brief  寫入一筆 52-byte 飛行數據封包至環形緩衝區
 *         自動計算 CRC，當接近預擦邊界時進行滾動擦除（~200ms）
 * @param  pkt  封包指標（seq、tick 等由呼叫方填入，crc 由本函式計算）
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef FlashRing_WritePacket(FlashRingPacket_t *pkt);
 
/**
 * @brief  邊界安全地提取環形緩衝區中最後一筆寫入的有效數據封包
 *         用於空中重開機時恢復飛行狀態機
 * @param  pkt  輸出封包結構體
 * @retval W25QXX_OK 成功提取, W25QXX_ERR_ID 數據無效或未寫入
 */
W25QXX_StatusTypeDef FlashRing_GetLastPacket(FlashRingPacket_t *pkt);

/** @brief 取得目前寫入地址 */
uint32_t FlashRing_GetWriteAddr(void);

/** @brief 取得累計寫入封包數 */
uint32_t FlashRing_GetPacketCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __W25QXX_H */

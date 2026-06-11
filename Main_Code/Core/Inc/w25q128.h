/**
  ******************************************************************************
  * @file           : w25q128.h
  * @brief          : Header for w25q128.c SPI Flash driver.
  *                   W25Q128JV (128 M-bit / 16 M-byte) SPI NOR Flash
  *                   掛載於 SPI1，CS 腳位為 PA15 (FLASH_CSB)
  ******************************************************************************
  */
#ifndef __W25Q128_H
#define __W25Q128_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* --- W25Q128 SPI Chip Select Port/Pin (PA15) --- */
#define W25Q128_CS_PORT    FLASH_CSB_GPIO_Port   /* GPIOA */
#define W25Q128_CS_PIN     FLASH_CSB_Pin          /* GPIO_PIN_15 */

/* --- W25Q128 Instruction Set --- */
#define W25Q128_CMD_WRITE_ENABLE      0x06
#define W25Q128_CMD_WRITE_DISABLE     0x04
#define W25Q128_CMD_READ_STATUS_REG1  0x05
#define W25Q128_CMD_READ_DATA         0x03
#define W25Q128_CMD_PAGE_PROGRAM      0x02
#define W25Q128_CMD_SECTOR_ERASE      0x20   /* 4KB Sector Erase */
#define W25Q128_CMD_BLOCK_ERASE_32K   0x52
#define W25Q128_CMD_BLOCK_ERASE_64K   0xD8
#define W25Q128_CMD_CHIP_ERASE        0xC7
#define W25Q128_CMD_JEDEC_ID          0x9F
#define W25Q128_CMD_POWER_DOWN        0xB9
#define W25Q128_CMD_RELEASE_PD        0xAB

/* --- Status Register Bit Masks --- */
#define W25Q128_SR_BUSY               0x01

/* --- Chip Parameters --- */
#define W25Q128_PAGE_SIZE             256               /* 每頁 256 bytes */
#define W25Q128_SECTOR_SIZE           4096              /* 每扇區 4096 bytes (4 KB) */
#define W25Q128_TOTAL_SIZE            (16 * 1024 * 1024)  /* 16 MB */

/* --- Timeout --- */
#define W25Q128_TIMEOUT_MS            3000              /* 最大等待超時 (sector erase 最壞情況 ~400ms) */

/* --- Expected JEDEC ID for W25Q128JV --- */
#define W25Q128_JEDEC_ID_EXPECTED     0xEF4018

/* ============================================================================
 * Flash Memory Map  (W25Q128JV, 16 MB)
 * ============================================================================
 *
 *  地址範圍                 大小     類型           用途
 *  0x000000 ~ 0x000FFF     4 KB    Static Offset  系統旗標 (System Flags)
 *  0x001000 ~ 0x00FFFF     60 KB   Static Offset  任務總結 (Mission Summary)
 *  0x010000 ~ 0xFFFFFF    ~15.9 MB Ring Buffer    關鍵飛行數據 (Flight Data)
 *
 * 環形緩衝區容量估算：
 *   每筆封包 52 bytes，記錄頻率 20 Hz
 *   15,925,248 bytes ÷ 52 ÷ 20 ≈ 15,312 秒 ≈ 255 分鐘 (> 3 小時)
 * ============================================================================ */

/* --------------------------------------------------------------------------
 * 區塊一：系統旗標區  (Sector 0)
 *   地址：0x000000 ~ 0x000FFF  (4 KB)
 *   性質：Static Offset，每次上電後由 FSM 讀取/更新
 *   擦除單位：整個 Sector 0 (4 KB)，更新前需先擦除
 * -------------------------------------------------------------------------- */
#define FLASH_SYSFLAGS_ADDR          0x000000UL  /* 系統旗標區起始地址 */
#define FLASH_SYSFLAGS_SIZE          0x001000UL  /* 4 KB */

#define FLASH_OFF_FSM_STATE          0x0000UL    /* 飛行狀態機 (FSM) 當前狀態，1 byte */
#define FLASH_OFF_DEPLOY_LOG         0x0004UL    /* 開傘事件紀錄：時間戳 + 級數，8 bytes */
#define FLASH_OFF_CALIB_PARAMS       0x0010UL    /* 感測器校準參數 (零偏/增益)，最多 64 bytes */

/* --------------------------------------------------------------------------
 * 區塊二：任務總結區  (Sectors 1~15)
 *   地址：0x001000 ~ 0x00FFFF  (60 KB)
 *   性質：Static Offset，飛行結束後由後處理任務一次性寫入
 *   每次飛行只寫一次，掉電不遺失
 * -------------------------------------------------------------------------- */
#define FLASH_SUMMARY_ADDR           0x001000UL  /* 任務總結區起始地址 */
#define FLASH_SUMMARY_SIZE           0x00F000UL  /* 60 KB */

#define FLASH_OFF_MAX_ALTITUDE       0x0000UL    /* 最高高度，float，4 bytes (單位 m) */
#define FLASH_OFF_MAX_VELOCITY       0x0004UL    /* 最大速度，float，4 bytes (單位 m/s) */
#define FLASH_OFF_STATE_TIMESTAMPS   0x0010UL    /* 各 FSM 狀態切換時間戳，uint32_t × 8 = 32 bytes */
#define FLASH_OFF_LAND_GPS_LAT       0x0040UL    /* 落地緯度，double，8 bytes */
#define FLASH_OFF_LAND_GPS_LON       0x0048UL    /* 落地經度，double，8 bytes */

/* --------------------------------------------------------------------------
 * 區塊三：飛行數據環形緩衝區  (Block 1 ~ end)
 *   地址：0x010000 ~ 0xFFFFFF  (~15.9 MB，64 KB 對齊起始)
 *   性質：Ring Buffer，從準備發射起持續寫入，滿後覆蓋最舊數據
 *   寫入時機：20 Hz，每筆 52 bytes
 *   封包內容：高度、GPS、姿態、FSM 狀態、CRC checksum
 * -------------------------------------------------------------------------- */
#define FLASH_RINGBUF_ADDR           0x010000UL  /* Ring Buffer 起始地址 (64 KB 對齊) */
#define FLASH_RINGBUF_END            0xFFFFFFUL  /* Ring Buffer 結束地址 */
#define FLASH_RINGBUF_SIZE           0xFF0000UL  /* ~15.9 MB */

#define FLASH_PACKET_SIZE            80UL        /* 每筆飛行數據封包大小 (bytes) */
#define FLASH_PACKET_RATE_HZ         20UL        /* 封包記錄頻率 (Hz) */
/* 最大記錄時長：0xFF0000 / 52 / 20 = 15,312 秒 ≈ 255 分鐘 */

/* --- Function Prototypes --- */
HAL_StatusTypeDef W25Q128_Init(SPI_HandleTypeDef *hspi);
uint32_t          W25Q128_ReadJEDEC_ID(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef W25Q128_EraseSector(SPI_HandleTypeDef *hspi, uint32_t sector_addr);
HAL_StatusTypeDef W25Q128_WritePage(SPI_HandleTypeDef *hspi, uint32_t addr, const uint8_t *data, uint16_t len);
HAL_StatusTypeDef W25Q128_ReadData(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len);
void              Flash_DumpAll(SPI_HandleTypeDef *hspi);   /* 開機完整 Dump 三分區至 USART2 */

#ifdef __cplusplus
}
#endif

#endif /* __W25Q128_H */

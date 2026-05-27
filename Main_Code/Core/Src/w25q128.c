/**
  ******************************************************************************
  * @file           : w25q128.c
  * @brief          : W25Q128JV SPI NOR Flash Driver for NCKU ISP Avionics Board
  *                   128 M-bit (16 M-byte), SPI Mode 0/3, 掛載於 SPI1, CS = PA15
  *
  *                   提供：JEDEC ID 讀取、4KB 扇區擦除、頁寫入 (≤256B)、連續讀取
  *                   所有操作採 HAL 阻塞傳輸 + WaitBusy 超時保護
  ******************************************************************************
  */

#include "w25q128.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* Flash_DumpAll 需要定期餵狗，直接存取 main.c 的 hiwdg（常見 STM32 嵌入式慣例） */
extern IWDG_HandleTypeDef hiwdg;

/* ========================== Internal Helpers =============================== */

/** @brief 拉低 CS 線 (選中 Flash) */
static inline void W25Q128_CS_Low(void)
{
    HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_RESET);
}

/** @brief 拉高 CS 線 (釋放 Flash) */
static inline void W25Q128_CS_High(void)
{
    HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_SET);
}

/**
  * @brief  等待 W25Q128 內部操作完成 (Busy 輪詢 + 超時保護)
  * @param  hspi  SPI 句柄
  * @retval HAL_OK = Flash 閒置; HAL_TIMEOUT = 超時仍忙碌
  */
static HAL_StatusTypeDef W25Q128_WaitBusy(SPI_HandleTypeDef *hspi)
{
    uint8_t cmd = W25Q128_CMD_READ_STATUS_REG1;
    uint8_t sr = 0xFF;
    uint32_t start = HAL_GetTick();

    while (1)
    {
        W25Q128_CS_Low();
        HAL_SPI_Transmit(hspi, &cmd, 1, 100);
        HAL_SPI_Receive(hspi, &sr, 1, 100);
        W25Q128_CS_High();

        if ((sr & W25Q128_SR_BUSY) == 0)
        {
            return HAL_OK;
        }

        if ((HAL_GetTick() - start) > W25Q128_TIMEOUT_MS)
        {
            return HAL_TIMEOUT;
        }
    }
}

/**
  * @brief  發送 Write Enable (WREN) 指令
  * @param  hspi  SPI 句柄
  */
static void W25Q128_WriteEnable(SPI_HandleTypeDef *hspi)
{
    uint8_t cmd = W25Q128_CMD_WRITE_ENABLE;
    W25Q128_CS_Low();
    HAL_SPI_Transmit(hspi, &cmd, 1, 100);
    W25Q128_CS_High();
}

/* ========================== Public API ===================================== */

/**
  * @brief  初始化 W25Q128：釋放 Power-Down 模式並驗證 JEDEC ID
  * @param  hspi  SPI 句柄 (hspi1)
  * @retval HAL_OK = 成功偵測到 W25Q128; HAL_ERROR = ID 不匹配
  */
HAL_StatusTypeDef W25Q128_Init(SPI_HandleTypeDef *hspi)
{
    /* 確保 CS 初始為高（未選中） */
    W25Q128_CS_High();
    HAL_Delay(1);

    /* 釋放 Power-Down 模式 (Release Power-down / Device ID) */
    uint8_t cmd = W25Q128_CMD_RELEASE_PD;
    W25Q128_CS_Low();
    HAL_SPI_Transmit(hspi, &cmd, 1, 100);
    W25Q128_CS_High();
    HAL_Delay(1);  /* tRES1 = 3µs, 用 1ms 安全餘裕 */

    /* 讀取並驗證 JEDEC ID */
    uint32_t id = W25Q128_ReadJEDEC_ID(hspi);
    if (id == W25Q128_JEDEC_ID_EXPECTED)
    {
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
  * @brief  讀取 W25Q128 JEDEC ID (Manufacturer + Memory Type + Capacity)
  * @param  hspi  SPI 句柄
  * @retval 24-bit JEDEC ID (e.g., 0xEF4018 for W25Q128JV)
  */
uint32_t W25Q128_ReadJEDEC_ID(SPI_HandleTypeDef *hspi)
{
    uint8_t cmd = W25Q128_CMD_JEDEC_ID;
    uint8_t id_buf[3] = {0};

    W25Q128_CS_Low();
    HAL_SPI_Transmit(hspi, &cmd, 1, 100);
    HAL_SPI_Receive(hspi, id_buf, 3, 100);
    W25Q128_CS_High();

    return ((uint32_t)id_buf[0] << 16) | ((uint32_t)id_buf[1] << 8) | id_buf[2];
}

/**
  * @brief  擦除 W25Q128 的一個 4KB 扇區 (Sector Erase, 0x20)
  * @param  hspi        SPI 句柄
  * @param  sector_addr 扇區起始地址 (必須為 4096 的倍數)
  * @retval HAL_OK / HAL_TIMEOUT / HAL_ERROR
  */
HAL_StatusTypeDef W25Q128_EraseSector(SPI_HandleTypeDef *hspi, uint32_t sector_addr)
{
    uint8_t cmd_buf[4];

    W25Q128_WriteEnable(hspi);

    cmd_buf[0] = W25Q128_CMD_SECTOR_ERASE;
    cmd_buf[1] = (uint8_t)(sector_addr >> 16);
    cmd_buf[2] = (uint8_t)(sector_addr >> 8);
    cmd_buf[3] = (uint8_t)(sector_addr);

    W25Q128_CS_Low();
    HAL_SPI_Transmit(hspi, cmd_buf, 4, 100);
    W25Q128_CS_High();

    return W25Q128_WaitBusy(hspi);
}

/**
  * @brief  寫入一頁數據到 W25Q128 (Page Program, 最大 256 bytes)
  * @param  hspi  SPI 句柄
  * @param  addr  寫入起始地址 (需在已擦除的區域)
  * @param  data  待寫入數據的指標
  * @param  len   寫入長度 (≤ 256, 不可跨頁邊界)
  * @retval HAL_OK / HAL_TIMEOUT / HAL_ERROR
  */
HAL_StatusTypeDef W25Q128_WritePage(SPI_HandleTypeDef *hspi, uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > W25Q128_PAGE_SIZE) return HAL_ERROR;

    uint8_t cmd_buf[4];

    W25Q128_WriteEnable(hspi);

    cmd_buf[0] = W25Q128_CMD_PAGE_PROGRAM;
    cmd_buf[1] = (uint8_t)(addr >> 16);
    cmd_buf[2] = (uint8_t)(addr >> 8);
    cmd_buf[3] = (uint8_t)(addr);

    W25Q128_CS_Low();
    HAL_SPI_Transmit(hspi, cmd_buf, 4, 100);
    HAL_SPI_Transmit(hspi, (uint8_t *)data, len, 500);
    W25Q128_CS_High();

    return W25Q128_WaitBusy(hspi);
}

/**
  * @brief  從 W25Q128 連續讀取數據 (Read Data, 0x03)
  * @param  hspi  SPI 句柄
  * @param  addr  讀取起始地址
  * @param  buf   接收緩衝區
  * @param  len   讀取長度
  * @retval HAL_OK / HAL_ERROR
  */
HAL_StatusTypeDef W25Q128_ReadData(SPI_HandleTypeDef *hspi, uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t cmd_buf[4];

    cmd_buf[0] = W25Q128_CMD_READ_DATA;
    cmd_buf[1] = (uint8_t)(addr >> 16);
    cmd_buf[2] = (uint8_t)(addr >> 8);
    cmd_buf[3] = (uint8_t)(addr);

    W25Q128_CS_Low();
    HAL_SPI_Transmit(hspi, cmd_buf, 4, 100);
    HAL_SPI_Receive(hspi, buf, len, 500);
    W25Q128_CS_High();

    return HAL_OK;
}

/* ========================== Flash Dump Utility ============================= */

/**
  * @brief  輸出單行標準 Hex Dump (最多 16 bytes) 至 USART2
  *         格式: XXXXXX: XX XX XX XX XX XX XX XX  XX XX XX XX XX XX XX XX  |................|
  */
static void flash_print_hex_line(uint32_t addr, const uint8_t *data, uint16_t len)
{
    printf("%06X: ", (unsigned int)addr);
    for (int i = 0; i < 16; i++) {
        if (i < (int)len) printf("%02X ", data[i]);
        else               printf("   ");
        if (i == 7)        printf(" ");   /* 中間空格分隔高低 8 bytes */
    }
    printf(" |");
    for (int i = 0; i < (int)len; i++)
        printf("%c", (data[i] >= 0x20 && data[i] < 0x7F) ? data[i] : '.');
    printf("|\r\n");
}

/**
  * @brief  完整讀取 W25Q128 三個記憶體分區並格式化輸出至 USART2
  *
  *   [1] 系統旗標區  (0x000000~0x000FFF, 4KB)   — 全量 Hex Dump + 欄位解析
  *   [2] 任務總結區  (0x001000~0x00FFFF, 60KB)  — 前 128 bytes Hex Dump + 欄位解析
  *   [3] Ring Buffer (0x010000~0xFFFFFF, ~15.9MB)— 前 3 封包 Hex Dump + 空滿判斷
  *
  *   每讀完一頁 (256 bytes) 自動餵狗，確保不觸發 2s IWDG 超時。
  *   系統旗標區全量輸出約需 1.7 秒 (115200 baud)。
  *
  * @param  hspi  Flash 所在的 SPI 句柄 (hspi3)
  */
void Flash_DumpAll(SPI_HandleTypeDef *hspi)
{
    uint8_t buf[256];

    printf("\r\n");
    printf("========================================================\r\n");
    printf("  W25Q128 Flash Memory Dump\r\n");
    printf("  Total 16MB  (0x000000 ~ 0xFFFFFF)\r\n");
    printf("========================================================\r\n");

    /* --- JEDEC ID 驗證，確認晶片正常就緒 --- */
    W25Q128_Init(hspi);
    uint32_t jedec = W25Q128_ReadJEDEC_ID(hspi);
    printf("[JEDEC] 0x%06X  %s\r\n", (unsigned int)jedec,
           (jedec == W25Q128_JEDEC_ID_EXPECTED) ? "W25Q128JV OK" : "ID MISMATCH!");
    if (jedec != W25Q128_JEDEC_ID_EXPECTED) {
        printf("[FLASH] Flash 晶片未就緒，中止讀取。\r\n\r\n");
        return;
    }

    /* ================================================================
     * [1] 系統旗標區  Sector 0  (0x000000 ~ 0x000FFF, 4KB)
     *     全量 Hex Dump：以 256 bytes/頁讀取，每頁餵狗一次
     *     FSM 狀態、開傘紀錄、校準參數
     * ================================================================ */
    printf("\r\n[1] 系統旗標區  0x%06X ~ 0x%06X  (%u bytes)\r\n",
           (unsigned int)FLASH_SYSFLAGS_ADDR,
           (unsigned int)(FLASH_SYSFLAGS_ADDR + FLASH_SYSFLAGS_SIZE - 1),
           (unsigned int)FLASH_SYSFLAGS_SIZE);
    printf("--------------------------------------------------------\r\n");

    for (uint32_t off = 0; off < FLASH_SYSFLAGS_SIZE; off += 256) {
        W25Q128_ReadData(hspi, FLASH_SYSFLAGS_ADDR + off, buf, 256);
        HAL_IWDG_Refresh(&hiwdg);          /* 每頁餵狗，防 2s IWDG 超時 */
        for (uint16_t ln = 0; ln < 256; ln += 16)
            flash_print_hex_line(FLASH_SYSFLAGS_ADDR + off + ln, buf + ln, 16);
    }

    /* 重新讀取前 80 bytes 進行欄位解析 */
    W25Q128_ReadData(hspi, FLASH_SYSFLAGS_ADDR, buf, 80);
    HAL_IWDG_Refresh(&hiwdg);
    printf("  [欄位解析]\r\n");
    printf("  FSM State    @ +0x%04X : 0x%02X\r\n",
           (unsigned int)FLASH_OFF_FSM_STATE,
           buf[FLASH_OFF_FSM_STATE]);
    printf("  Deploy Log   @ +0x%04X : %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
           (unsigned int)FLASH_OFF_DEPLOY_LOG,
           buf[FLASH_OFF_DEPLOY_LOG+0], buf[FLASH_OFF_DEPLOY_LOG+1],
           buf[FLASH_OFF_DEPLOY_LOG+2], buf[FLASH_OFF_DEPLOY_LOG+3],
           buf[FLASH_OFF_DEPLOY_LOG+4], buf[FLASH_OFF_DEPLOY_LOG+5],
           buf[FLASH_OFF_DEPLOY_LOG+6], buf[FLASH_OFF_DEPLOY_LOG+7]);
    printf("  Calib[0..3]  @ +0x%04X : %02X %02X %02X %02X ...\r\n",
           (unsigned int)FLASH_OFF_CALIB_PARAMS,
           buf[FLASH_OFF_CALIB_PARAMS+0], buf[FLASH_OFF_CALIB_PARAMS+1],
           buf[FLASH_OFF_CALIB_PARAMS+2], buf[FLASH_OFF_CALIB_PARAMS+3]);

    /* ================================================================
     * [2] 任務總結區  Sectors 1-15  (0x001000 ~ 0x00FFFF, 60KB)
     *     顯示前 128 bytes（涵蓋所有已定義欄位），其餘為預留空間
     *     最高高度、最大速度、狀態切換時間戳、落地 GPS 座標
     * ================================================================ */
    printf("\r\n[2] 任務總結區  0x%06X ~ 0x%06X  (%u bytes，顯示前 128 bytes)\r\n",
           (unsigned int)FLASH_SUMMARY_ADDR,
           (unsigned int)(FLASH_SUMMARY_ADDR + FLASH_SUMMARY_SIZE - 1),
           (unsigned int)FLASH_SUMMARY_SIZE);
    printf("--------------------------------------------------------\r\n");

    W25Q128_ReadData(hspi, FLASH_SUMMARY_ADDR, buf, 128);
    HAL_IWDG_Refresh(&hiwdg);
    for (uint16_t ln = 0; ln < 128; ln += 16)
        flash_print_hex_line(FLASH_SUMMARY_ADDR + ln, buf + ln, 16);

    printf("  [欄位解析]\r\n");
    printf("  Max Altitude  @ +0x%04X : %02X %02X %02X %02X  (float, little-endian)\r\n",
           (unsigned int)FLASH_OFF_MAX_ALTITUDE,
           buf[FLASH_OFF_MAX_ALTITUDE+0], buf[FLASH_OFF_MAX_ALTITUDE+1],
           buf[FLASH_OFF_MAX_ALTITUDE+2], buf[FLASH_OFF_MAX_ALTITUDE+3]);
    printf("  Max Velocity  @ +0x%04X : %02X %02X %02X %02X  (float, little-endian)\r\n",
           (unsigned int)FLASH_OFF_MAX_VELOCITY,
           buf[FLASH_OFF_MAX_VELOCITY+0], buf[FLASH_OFF_MAX_VELOCITY+1],
           buf[FLASH_OFF_MAX_VELOCITY+2], buf[FLASH_OFF_MAX_VELOCITY+3]);
    printf("  State Times   @ +0x%04X :\r\n", (unsigned int)FLASH_OFF_STATE_TIMESTAMPS);
    for (int i = 0; i < 8; i++) {
        /* 每個時間戳為 uint32_t (4 bytes, little-endian) */
        uint32_t ts = (uint32_t)buf[FLASH_OFF_STATE_TIMESTAMPS + i*4 + 0]
                    | (uint32_t)buf[FLASH_OFF_STATE_TIMESTAMPS + i*4 + 1] << 8
                    | (uint32_t)buf[FLASH_OFF_STATE_TIMESTAMPS + i*4 + 2] << 16
                    | (uint32_t)buf[FLASH_OFF_STATE_TIMESTAMPS + i*4 + 3] << 24;
        printf("    [%d] %u ms\r\n", i, (unsigned int)ts);
    }
    printf("  GPS Lat       @ +0x%04X : %02X %02X %02X %02X %02X %02X %02X %02X  (double)\r\n",
           (unsigned int)FLASH_OFF_LAND_GPS_LAT,
           buf[FLASH_OFF_LAND_GPS_LAT+0], buf[FLASH_OFF_LAND_GPS_LAT+1],
           buf[FLASH_OFF_LAND_GPS_LAT+2], buf[FLASH_OFF_LAND_GPS_LAT+3],
           buf[FLASH_OFF_LAND_GPS_LAT+4], buf[FLASH_OFF_LAND_GPS_LAT+5],
           buf[FLASH_OFF_LAND_GPS_LAT+6], buf[FLASH_OFF_LAND_GPS_LAT+7]);
    printf("  GPS Lon       @ +0x%04X : %02X %02X %02X %02X %02X %02X %02X %02X  (double)\r\n",
           (unsigned int)FLASH_OFF_LAND_GPS_LON,
           buf[FLASH_OFF_LAND_GPS_LON+0], buf[FLASH_OFF_LAND_GPS_LON+1],
           buf[FLASH_OFF_LAND_GPS_LON+2], buf[FLASH_OFF_LAND_GPS_LON+3],
           buf[FLASH_OFF_LAND_GPS_LON+4], buf[FLASH_OFF_LAND_GPS_LON+5],
           buf[FLASH_OFF_LAND_GPS_LON+6], buf[FLASH_OFF_LAND_GPS_LON+7]);

    /* ================================================================
     * [3] 飛行數據 Ring Buffer  (0x010000 ~ 0xFFFFFF, ~15.9MB)
     *     顯示前 3 筆封包 (3 × 52 = 156 bytes) 並判斷空滿狀態
     *     封包內容：高度、GPS、姿態、FSM 狀態、CRC checksum
     * ================================================================ */
    const uint16_t ring_preview = (uint16_t)(FLASH_PACKET_SIZE * 3);  /* 156 bytes */
    printf("\r\n[3] Ring Buffer  0x%06X ~ 0x%06X  (%u MB)\r\n",
           (unsigned int)FLASH_RINGBUF_ADDR,
           (unsigned int)FLASH_RINGBUF_END,
           (unsigned int)(FLASH_RINGBUF_SIZE >> 20));
    printf("    容量: %u 封包 / %u 分鐘  (%u Hz x %u bytes/packet)\r\n",
           (unsigned int)(FLASH_RINGBUF_SIZE / FLASH_PACKET_SIZE),
           (unsigned int)(FLASH_RINGBUF_SIZE / FLASH_PACKET_SIZE / FLASH_PACKET_RATE_HZ / 60),
           (unsigned int)FLASH_PACKET_RATE_HZ,
           (unsigned int)FLASH_PACKET_SIZE);
    printf("    顯示前 %u bytes (前 3 封包)\r\n", ring_preview);
    printf("--------------------------------------------------------\r\n");

    W25Q128_ReadData(hspi, FLASH_RINGBUF_ADDR, buf, ring_preview);
    HAL_IWDG_Refresh(&hiwdg);

    /* 判斷是否有有效資料（全 0xFF 表示尚未寫入任何封包） */
    uint8_t ring_has_data = 0;
    for (int i = 0; i < ring_preview; i++) {
        if (buf[i] != 0xFF) { ring_has_data = 1; break; }
    }
    printf("  狀態: %s\r\n", ring_has_data ? "有數據 (非全空白)" : "全空白 (0xFF，尚未寫入)");

    for (uint16_t ln = 0; ln < ring_preview; ln += 16) {
        uint16_t rem = ring_preview - ln;
        flash_print_hex_line(FLASH_RINGBUF_ADDR + ln, buf + ln,
                             rem >= 16 ? 16 : rem);
    }

    printf("\r\n[FLASH] Dump 完畢。\r\n");
    printf("========================================================\r\n\r\n");
    HAL_IWDG_Refresh(&hiwdg);
}

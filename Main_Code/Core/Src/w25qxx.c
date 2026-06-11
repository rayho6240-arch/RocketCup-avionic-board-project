/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    w25qxx.c
 * @brief   W25Qxx SPI Flash 驅動程式
 *          適用晶片: W25Q16 / W25Q32 / W25Q64 / W25Q128
 *
 *  接線:
 *    SPI1_SCK   → PA5
 *    SPI1_MISO  → PA6
 *    SPI1_MOSI  → PA7
 *    FLASH_CSB  → PA15  (軟體控制 CS)
 *
 *  Debug 輸出走 USART2 (PA2=TX, PA3=RX) → ESP32 TTL → Serial
 ******************************************************************************
 */
/* USER CODE END Header */

#include "w25qxx.h"
#include "spi3_bus.h"   /* SPI3 與 E80 920MHz LoRa 共用，CS 期間須持互斥鎖 */
#include "crc16.h"      /* P1：CRC-16/CCITT-FALSE 單一實作 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 *  私有巨集
 * ============================================================ */
/* CS_LOW 前取得 SPI3 匯流排、CS_HIGH 後釋放：保證 Flash 在 CS 拉低的整段交易期間
 * 獨佔 SPI3，與 E80 920MHz LoRa 互斥（見 spi3_bus.h）。兩 macro 在每個函式內成對使用，
 * 鎖為遞迴+優先級繼承；mutex 建立前 (pre-scheduler) Lock/Unlock 為 no-op。 */
#define CS_LOW()   do { SPI3_Bus_Lock(); \
                        HAL_GPIO_WritePin(W25QXX_CS_GPIO_PORT, W25QXX_CS_GPIO_PIN, GPIO_PIN_RESET); } while (0)
#define CS_HIGH()  do { HAL_GPIO_WritePin(W25QXX_CS_GPIO_PORT, W25QXX_CS_GPIO_PIN, GPIO_PIN_SET); \
                        SPI3_Bus_Unlock(); } while (0)

/* BUSY bit 等待逾時 (一般寫入) */
#define W25QXX_WRITE_TIMEOUT_MS     500U
/* Sector Erase 最長等待 */
#define W25QXX_SECTOR_ERASE_TIMEOUT 400U
/* Chip Erase 最長等待 (100 秒) */
#define W25QXX_CHIP_ERASE_TIMEOUT   100000U

/* ============================================================
 *  內部 Debug 輸出 (透過 printf retarget 到 USART2)
 * ============================================================ */
static void flash_debug(const char *msg)
{
    printf("%s", msg);
}

static void flash_debugf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

/* ============================================================
 *  私有: 傳送/接收單一 byte (CS 必須在外部控制)
 * ============================================================ */
static HAL_StatusTypeDef spi_transmit(const uint8_t *pTx, uint16_t len)
{
    return HAL_SPI_Transmit(&W25QXX_SPI_HANDLE, (uint8_t *)pTx, len, W25QXX_SPI_TIMEOUT_MS);
}

static HAL_StatusTypeDef spi_receive(uint8_t *pRx, uint16_t len)
{
    return HAL_SPI_Receive(&W25QXX_SPI_HANDLE, pRx, len, W25QXX_SPI_TIMEOUT_MS);
}

/* ============================================================
 *  私有: 傳送 Write Enable 指令
 * ============================================================ */
static W25QXX_StatusTypeDef send_write_enable(void)
{
    uint8_t cmd = W25QXX_CMD_WRITE_ENABLE;
    CS_LOW();
    HAL_StatusTypeDef ret = spi_transmit(&cmd, 1);
    CS_HIGH();
    return (ret == HAL_OK) ? W25QXX_OK : W25QXX_ERR_SPI;
}

/* ============================================================
 *  W25QXX_ReadStatusReg1
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_ReadStatusReg1(uint8_t *status)
{
    uint8_t cmd = W25QXX_CMD_READ_STATUS_REG1;
    CS_LOW();
    if (spi_transmit(&cmd, 1) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    if (spi_receive(status, 1) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    CS_HIGH();
    return W25QXX_OK;
}

/* ============================================================
 *  W25QXX_WaitForReady
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_WaitForReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t  status;

    while (1)
    {
        if (W25QXX_ReadStatusReg1(&status) != W25QXX_OK)
            return W25QXX_ERR_SPI;

        if (!(status & W25QXX_SR1_BUSY))
            return W25QXX_OK;  /* 不再 BUSY */

        if ((HAL_GetTick() - start) >= timeout_ms)
            return W25QXX_ERR_TIMEOUT;

        /* 短暫讓出 CPU，在 FreeRTOS 環境下可改為 osDelay(1) */
        HAL_Delay(1);
    }
}

/* ============================================================
 *  W25QXX_ReadJEDEC_ID
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_ReadJEDEC_ID(uint8_t *manufacturer, uint16_t *deviceID)
{
    uint8_t cmd = W25QXX_CMD_JEDEC_ID;
    uint8_t rx[3] = {0};

    CS_LOW();
    if (spi_transmit(&cmd, 1) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    if (spi_receive(rx, 3) != HAL_OK)    { CS_HIGH(); return W25QXX_ERR_SPI; }
    CS_HIGH();

    *manufacturer = rx[0];
    *deviceID     = ((uint16_t)rx[1] << 8) | rx[2];
    return W25QXX_OK;
}

/* ============================================================
 *  W25QXX_Init
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_Init(W25QXX_InfoTypeDef *info)
{
    if (info == NULL) return W25QXX_ERR_PARAM;

    memset(info, 0, sizeof(W25QXX_InfoTypeDef));

    /* 確保 CS 閒置為 HIGH */
    CS_HIGH();
    HAL_Delay(10);

    /* 喚醒 (Release Power Down) */
    uint8_t cmd = W25QXX_CMD_RELEASE_POWER_DOWN;
    CS_LOW();
    spi_transmit(&cmd, 1);
    CS_HIGH();
    HAL_Delay(1);

    /* 讀取 JEDEC ID */
    W25QXX_StatusTypeDef status = W25QXX_ReadJEDEC_ID(&info->ManufacturerID, &info->DeviceID);
    if (status != W25QXX_OK) return status;

    if (info->ManufacturerID != W25QXX_MANUFACTURER_ID)
    {
        flash_debugf("[FLASH] Init FAIL: MfgID=0x%02X (expected 0xEF)\r\n", info->ManufacturerID);
        return W25QXX_ERR_ID;
    }

    /* 根據 Device ID 推算容量 */
    uint8_t density = (uint8_t)(info->DeviceID & 0xFF);  /* 低 byte = density code */
    if (density >= 0x11 && density <= 0x1B)
    {
        /* 2^density KB, density=0x11 → 2MB (W25Q16), 0x17 → 128MB (W25Q128) */
        uint32_t capacity_bytes = (1UL << density) * 1024UL;  /* 實際是 2^density Mbit /8 */
        /* Winbond density code: 0x11=2MB, 0x12=4MB, 0x13=8MB, 0x14=16MB,
                                 0x15=32MB, 0x16=64MB, 0x17=128MB           */
        /* 正確計算: capacity = 2^density bits / 8 */
        info->Capacity_KB  = (1UL << (density - 3));   /* KB */
        info->SectorCount  = info->Capacity_KB / 4;    /* 每 Sector 4 KB */
        (void)capacity_bytes;
    }

    info->Initialized = true;

    flash_debugf("[FLASH] Init OK | MfgID=0x%02X DevID=0x%04X Cap=%lu KB Sectors=%lu\r\n",
                 info->ManufacturerID, info->DeviceID,
                 (unsigned long)info->Capacity_KB,
                 (unsigned long)info->SectorCount);

    return W25QXX_OK;
}

/* ============================================================
 *  W25QXX_ReadData
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_ReadData(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return W25QXX_ERR_PARAM;

    uint8_t cmd[4];
    cmd[0] = W25QXX_CMD_READ_DATA;
    cmd[1] = (uint8_t)((addr >> 16) & 0xFF);
    cmd[2] = (uint8_t)((addr >>  8) & 0xFF);
    cmd[3] = (uint8_t)( addr        & 0xFF);

    /* 確保 Flash 不在 BUSY 狀態 */
    if (W25QXX_WaitForReady(W25QXX_WRITE_TIMEOUT_MS) != W25QXX_OK)
        return W25QXX_ERR_TIMEOUT;

    CS_LOW();
    if (spi_transmit(cmd, 4) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }

    /* 分段接收 (HAL 單次最多 65535 bytes) */
    uint32_t remaining = len;
    uint8_t *ptr = buf;
    while (remaining > 0)
    {
        uint16_t chunk = (remaining > 65535U) ? 65535U : (uint16_t)remaining;
        if (spi_receive(ptr, chunk) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
        ptr       += chunk;
        remaining -= chunk;
    }
    CS_HIGH();
    return W25QXX_OK;
}

/* ============================================================
 *  W25QXX_WritePage  (最多 256 bytes，不可跨頁邊界)
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_WritePage(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0 || len > W25QXX_PAGE_SIZE) return W25QXX_ERR_PARAM;

    W25QXX_StatusTypeDef st;

    /* 等待 Flash 就緒 */
    st = W25QXX_WaitForReady(W25QXX_WRITE_TIMEOUT_MS);
    if (st != W25QXX_OK) return st;

    /* Write Enable */
    st = send_write_enable();
    if (st != W25QXX_OK) return st;

    uint8_t cmd[4];
    cmd[0] = W25QXX_CMD_PAGE_PROGRAM;
    cmd[1] = (uint8_t)((addr >> 16) & 0xFF);
    cmd[2] = (uint8_t)((addr >>  8) & 0xFF);
    cmd[3] = (uint8_t)( addr        & 0xFF);

    CS_LOW();
    if (spi_transmit(cmd, 4) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    if (spi_transmit(buf, len) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    CS_HIGH();

    /* 等待 Page Program 完成 */
    return W25QXX_WaitForReady(W25QXX_WRITE_TIMEOUT_MS);
}

/* ============================================================
 *  W25QXX_WriteData  (任意長度，自動分頁)
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_WriteData(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return W25QXX_ERR_PARAM;

    W25QXX_StatusTypeDef st;
    uint32_t remaining = len;
    uint32_t cur_addr  = addr;
    const uint8_t *ptr = buf;

    while (remaining > 0)
    {
        /* 計算到下一個頁面邊界還有多少 byte */
        uint32_t page_offset   = cur_addr % W25QXX_PAGE_SIZE;
        uint32_t space_in_page = W25QXX_PAGE_SIZE - page_offset;
        uint16_t write_len     = (uint16_t)((remaining < space_in_page) ? remaining : space_in_page);

        st = W25QXX_WritePage(cur_addr, ptr, write_len);
        if (st != W25QXX_OK) return st;

        cur_addr  += write_len;
        ptr       += write_len;
        remaining -= write_len;
    }
    return W25QXX_OK;
}

/* ============================================================
 *  W25QXX_EraseSector (4 KB)
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_EraseSector(uint32_t sectorAddr)
{
    W25QXX_StatusTypeDef st;

    st = W25QXX_WaitForReady(W25QXX_WRITE_TIMEOUT_MS);
    if (st != W25QXX_OK) return st;

    st = send_write_enable();
    if (st != W25QXX_OK) return st;

    uint8_t cmd[4];
    cmd[0] = W25QXX_CMD_SECTOR_ERASE_4KB;
    cmd[1] = (uint8_t)((sectorAddr >> 16) & 0xFF);
    cmd[2] = (uint8_t)((sectorAddr >>  8) & 0xFF);
    cmd[3] = (uint8_t)( sectorAddr        & 0xFF);

    CS_LOW();
    if (spi_transmit(cmd, 4) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    CS_HIGH();

    return W25QXX_WaitForReady(W25QXX_SECTOR_ERASE_TIMEOUT);
}

/* ============================================================
 *  W25QXX_EraseChip
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_EraseChip(void)
{
    W25QXX_StatusTypeDef st;

    st = W25QXX_WaitForReady(W25QXX_WRITE_TIMEOUT_MS);
    if (st != W25QXX_OK) return st;

    st = send_write_enable();
    if (st != W25QXX_OK) return st;

    uint8_t cmd = W25QXX_CMD_CHIP_ERASE;
    CS_LOW();
    if (spi_transmit(&cmd, 1) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    CS_HIGH();

    flash_debug("[FLASH] Chip Erase started, waiting...\r\n");
    return W25QXX_WaitForReady(W25QXX_CHIP_ERASE_TIMEOUT);
}

/* ============================================================
 *  Flash_Test — 完整讀寫驗證測試，結果輸出到 UART2
 * ============================================================ */
void Flash_Test(void)
{
    flash_debug("\r\n===== W25Qxx Flash Test Start =====\r\n");

    W25QXX_InfoTypeDef info;
    W25QXX_StatusTypeDef st;

    /* --- Step 1: Init --- */
    flash_debug("[TEST] Step 1: Init...\r\n");
    st = W25QXX_Init(&info);
    if (st != W25QXX_OK)
    {
        flash_debugf("[TEST] FAIL: Init error %d\r\n", st);
        return;
    }

    /* --- Step 2: JEDEC ID --- */
    flash_debug("[TEST] Step 2: Read JEDEC ID...\r\n");
    uint8_t  mfg;
    uint16_t devID;
    W25QXX_ReadJEDEC_ID(&mfg, &devID);
    flash_debugf("[TEST]   Manufacturer = 0x%02X  DeviceID = 0x%04X\r\n", mfg, devID);

    /* 常見 Device ID 對照 */
    const char *model = "Unknown";
    switch (devID)
    {
        case 0x4011: model = "W25Q10";  break;
        case 0x4012: model = "W25Q20";  break;
        case 0x4013: model = "W25Q40";  break;
        case 0x4014: model = "W25Q80";  break;
        case 0x4015: model = "W25Q16";  break;
        case 0x4016: model = "W25Q32";  break;
        case 0x4017: model = "W25Q64";  break;
        case 0x4018: model = "W25Q128"; break;
        case 0x4019: model = "W25Q256"; break;
    }
    flash_debugf("[TEST]   Model = %s  Capacity = %lu KB\r\n", model, (unsigned long)info.Capacity_KB);

    /* --- Step 3: Erase Sector 0 --- */
    flash_debug("[TEST] Step 3: Erase Sector 0 (addr=0x000000)...\r\n");
    st = W25QXX_EraseSector(0x000000);
    if (st != W25QXX_OK)
    {
        flash_debugf("[TEST] FAIL: Erase error %d\r\n", st);
        return;
    }
    flash_debug("[TEST]   Erase OK\r\n");

    /* --- Step 4: 確認擦除後全為 0xFF --- */
    flash_debug("[TEST] Step 4: Verify erase (first 16 bytes should be 0xFF)...\r\n");
    uint8_t readbuf[16];
    st = W25QXX_ReadData(0x000000, readbuf, sizeof(readbuf));
    if (st != W25QXX_OK)
    {
        flash_debugf("[TEST] FAIL: Read error %d\r\n", st);
        return;
    }
    bool erase_ok = true;
    for (int i = 0; i < 16; i++)
    {
        if (readbuf[i] != 0xFF) { erase_ok = false; break; }
    }
    flash_debugf("[TEST]   Erase verify: %s\r\n", erase_ok ? "PASS" : "FAIL");

    /* --- Step 5: 寫入測試資料 --- */
    flash_debug("[TEST] Step 5: Write test pattern (16 bytes)...\r\n");
    uint8_t writebuf[16];
    for (int i = 0; i < 16; i++) writebuf[i] = (uint8_t)(i * 0x11);
    /* writebuf = 0x00, 0x11, 0x22, ..., 0xFF */

    st = W25QXX_WriteData(0x000000, writebuf, sizeof(writebuf));
    if (st != W25QXX_OK)
    {
        flash_debugf("[TEST] FAIL: Write error %d\r\n", st);
        return;
    }
    flash_debug("[TEST]   Write OK\r\n");

    /* --- Step 6: 讀回比對 --- */
    flash_debug("[TEST] Step 6: Read back and verify...\r\n");
    memset(readbuf, 0, sizeof(readbuf));
    st = W25QXX_ReadData(0x000000, readbuf, sizeof(readbuf));
    if (st != W25QXX_OK)
    {
        flash_debugf("[TEST] FAIL: Read error %d\r\n", st);
        return;
    }

    bool verify_ok = (memcmp(writebuf, readbuf, sizeof(writebuf)) == 0);
    flash_debug("[TEST]   Read back hex: ");
    for (int i = 0; i < 16; i++)
    {
        char tmp[6];
        snprintf(tmp, sizeof(tmp), "%02X ", readbuf[i]);
        flash_debug(tmp);
    }
    flash_debug("\r\n");
    flash_debugf("[TEST]   Verify: %s\r\n", verify_ok ? "PASS" : "FAIL");

    /* --- 總結 --- */
    flash_debug("===========================\r\n");
    flash_debugf("[TEST] Overall: %s\r\n", verify_ok ? "ALL PASS" : "FAIL");
    flash_debug("===========================\r\n\r\n");
}

/* ============================================================
 *  環形緩衝區實作
 * ============================================================ */

extern IWDG_HandleTypeDef hiwdg;

static uint32_t s_ring_write_addr   = FLASH_RINGBUF_ADDR;
static uint32_t s_ring_erased_end   = FLASH_RINGBUF_ADDR;  /* 預擦區終止地址（exclusive） */
static uint32_t s_ring_packet_count = 0;
static uint16_t s_ring_seq          = 0;
static volatile uint8_t  s_ring_erase_allowed = 1;  /* P0-E：0 = 飛行中禁同步擦除 */
static volatile uint32_t s_ring_drop_count    = 0;  /* P0-E：池耗盡丟棄計數 */

/* P0-E：目前預擦池大小（bytes；exclusive-end 環形語意與 write/erase 指標一致） */
static uint32_t ring_pool_bytes(void)
{
    if (s_ring_erased_end >= s_ring_write_addr) {
        return s_ring_erased_end - s_ring_write_addr;
    }
    return (FLASH_RINGBUF_END + 1UL - s_ring_write_addr) +
           (s_ring_erased_end - FLASH_RINGBUF_ADDR);
}

uint16_t ring_crc16(const uint8_t *data, uint16_t len)
{
    return crc16_ccitt_false(data, len);   /* P1：統一至 crc16.h 單一實作（符號保留，多處引用） */
}

void FlashRing_Init(void)
{
    printf("[FLASH_RING] Init start...\r\n");

    /* --- 掃描寫入頭 ---
     * 策略：讀取 FLASH_RINGBUF_ADDR 的第一個 byte，
     *       若為 0xFF 則 Ring 為空，從頭開始；
     *       否則二分搜尋最後一個有資料的 Sector，再逐 slot 找首個空槽。 */
    uint8_t first_byte;
    W25QXX_ReadData(FLASH_RINGBUF_ADDR, &first_byte, 1);

    if (first_byte == 0xFF) {
        s_ring_write_addr = FLASH_RINGBUF_ADDR;
        printf("[FLASH_RING] Ring buffer empty, start from 0x%06lX\r\n", s_ring_write_addr);
    } else {
        /* 二分搜尋：找第一個內容全 0xFF 的 Sector */
        uint32_t lo_sec = 0;
        uint32_t hi_sec = FLASH_RINGBUF_SIZE / W25QXX_SECTOR_SIZE;  /* 3840 */
        while (lo_sec + 1 < hi_sec) {
            uint32_t mid = (lo_sec + hi_sec) / 2;
            W25QXX_ReadData(FLASH_RINGBUF_ADDR + mid * W25QXX_SECTOR_SIZE, &first_byte, 1);
            if (first_byte == 0xFF) hi_sec = mid;
            else                    lo_sec = mid;
        }
        /* 在 lo_sec 扇區內逐 slot 掃描 */
        uint32_t scan     = FLASH_RINGBUF_ADDR + lo_sec * W25QXX_SECTOR_SIZE;
        uint32_t scan_end = scan + W25QXX_SECTOR_SIZE;
        uint8_t  magic[2];
        s_ring_write_addr = scan;  /* fallback */
        while (scan + FLASH_RING_PACKET_SIZE <= scan_end) {
            W25QXX_ReadData(scan, magic, 2);
            if (magic[0] == 0xFF && magic[1] == 0xFF) {
                s_ring_write_addr = scan;
                break;
            }
            scan += FLASH_RING_PACKET_SIZE;
        }
        if (scan + FLASH_RING_PACKET_SIZE > scan_end) {
            /* 整個 Sector 已滿，從下一個 Sector 開始（滾動覆蓋） */
            uint32_t next = FLASH_RINGBUF_ADDR + (lo_sec + 1) * W25QXX_SECTOR_SIZE;
            s_ring_write_addr = (next <= FLASH_RINGBUF_END) ? next : FLASH_RINGBUF_ADDR;
        }
        printf("[FLASH_RING] Resumed from 0x%06lX\r\n", s_ring_write_addr);
    }

    /* --- 預擦 FLASH_RING_PREERASE_N 個 Sector --- */
    uint32_t erase_addr = s_ring_write_addr & ~((uint32_t)(W25QXX_SECTOR_SIZE - 1));
    for (int i = 0; i < FLASH_RING_PREERASE_N; i++) {
        W25QXX_EraseSector(erase_addr);
        HAL_IWDG_Refresh(&hiwdg);
        printf("[FLASH_RING] Pre-erase %2d/%d @ 0x%06lX\r\n", i + 1, FLASH_RING_PREERASE_N, erase_addr);
        erase_addr += W25QXX_SECTOR_SIZE;
        if (erase_addr > FLASH_RINGBUF_END) erase_addr = FLASH_RINGBUF_ADDR;
    }
    s_ring_erased_end   = erase_addr;
    s_ring_packet_count = 0;
    s_ring_seq          = 0;

    printf("[FLASH_RING] Ready. Write: 0x%06lX, Erased to: 0x%06lX\r\n",
           s_ring_write_addr, s_ring_erased_end);
}

W25QXX_StatusTypeDef FlashRing_WritePacket(FlashRingPacket_t *pkt)
{
    /* 若即將超出預擦區，滾動擦除下一個 Sector。
     * P0-E：飛行態（erase_allowed=0）禁止同步擦除 —— 最壞 ~400ms 阻塞主迴圈
     * （FSM 停擺、EKF 斷饋、持 SPI3 mutex），可能正落在頂點窗口。
     * 池耗盡時丟棄該筆並計數（PAD 期背景預擦 64 sectors ≈ 65s 飛行量，
     * 正常不應發生；發射檢核表須確認 [FLASH] pool 達標）。 */
    if (ring_pool_bytes() < FLASH_RING_PACKET_SIZE) {
        if (!s_ring_erase_allowed) {
            s_ring_drop_count++;
            if ((s_ring_drop_count % 100U) == 1U) {   /* 限流：首筆與每 100 筆印一次 */
                printf("[FLASH_RING] pool exhausted in flight, dropped=%lu\r\n",
                       (unsigned long)s_ring_drop_count);
            }
            return W25QXX_ERR_NO_POOL;
        }
        W25QXX_StatusTypeDef ret = W25QXX_EraseSector(s_ring_erased_end);
        HAL_IWDG_Refresh(&hiwdg);
        if (ret != W25QXX_OK) {
            printf("[FLASH_RING] Rolling erase FAILED @ 0x%06lX, err=%d\r\n",
                   s_ring_erased_end, (int)ret);
            return ret;
        }
        s_ring_erased_end += W25QXX_SECTOR_SIZE;
        if (s_ring_erased_end > FLASH_RINGBUF_END + 1) s_ring_erased_end = FLASH_RINGBUF_ADDR;
    }

    /* 填入欄位，計算 CRC（覆蓋 bytes [0..77]） */
    pkt->magic[0] = 0xAA;
    pkt->magic[1] = 0x55;
    pkt->seq      = s_ring_seq++;
    pkt->crc16    = ring_crc16((const uint8_t *)pkt, FLASH_RING_PACKET_SIZE - 2);

    /* 寫入 Flash（W25QXX_WriteData 自動處理跨 Page） */
    W25QXX_StatusTypeDef ret = W25QXX_WriteData(s_ring_write_addr,
                                                  (const uint8_t *)pkt,
                                                  FLASH_RING_PACKET_SIZE);
    if (ret != W25QXX_OK) {
        printf("[FLASH_RING] Write FAILED @ 0x%06lX, err=%d\r\n",
               s_ring_write_addr, (int)ret);
        return ret;
    }

    s_ring_write_addr += FLASH_RING_PACKET_SIZE;
    if (s_ring_write_addr + FLASH_RING_PACKET_SIZE > FLASH_RINGBUF_END + 1)
        s_ring_write_addr = FLASH_RINGBUF_ADDR;

    s_ring_packet_count++;

    /* 每 100 筆 log 一次進度 */
    if (s_ring_packet_count % 100 == 0) {
        printf("[FLASH_RING] PKT#%lu @ 0x%06lX\r\n",
               s_ring_packet_count, s_ring_write_addr);
    }

    return W25QXX_OK;
}

uint32_t FlashRing_GetWriteAddr(void)   { return s_ring_write_addr; }
uint32_t FlashRing_GetPacketCount(void) { return s_ring_packet_count; }

/* === P0-E：飛行中擦除禁令 + PAD 期背景預擦池 === */

void FlashRing_SetEraseAllowed(uint8_t allowed)
{
    s_ring_erase_allowed = (allowed != 0U);
}

uint8_t FlashRing_PreEraseOne(void)
{
    if (ring_pool_bytes() >= (uint32_t)FLASH_RING_PREERASE_TARGET * W25QXX_SECTOR_SIZE) {
        return 1U;   /* 池已達標 */
    }
    /* exclusive-end 語意：erased_end 可能停在 FLASH_RINGBUF_END+1，先迴繞再擦 */
    uint32_t target = (s_ring_erased_end > FLASH_RINGBUF_END)
                      ? FLASH_RINGBUF_ADDR : s_ring_erased_end;
    if (W25QXX_EraseSector(target) != W25QXX_OK) {
        return 0U;   /* 擦除失敗：下次再試 */
    }
    s_ring_erased_end = target + W25QXX_SECTOR_SIZE;
    return 0U;
}

uint32_t FlashRing_GetPoolSectors(void)
{
    return ring_pool_bytes() / W25QXX_SECTOR_SIZE;
}

uint32_t FlashRing_GetDropCount(void)
{
    return s_ring_drop_count;
}

W25QXX_StatusTypeDef FlashRing_GetLastPacket(FlashRingPacket_t *pkt)
{
    if (pkt == NULL) return W25QXX_ERR_PARAM;

    uint32_t last_write_addr = s_ring_write_addr;
    uint32_t last_packet_addr;

    // 處理環形邊界 wrap-around
    if (last_write_addr == FLASH_RINGBUF_ADDR) {
        last_packet_addr = FLASH_RINGBUF_END + 1 - FLASH_RING_PACKET_SIZE;
    } else {
        last_packet_addr = last_write_addr - FLASH_RING_PACKET_SIZE;
    }

    // 從 Flash 中讀取最後一個封包
    W25QXX_StatusTypeDef ret = W25QXX_ReadData(last_packet_addr, (uint8_t*)pkt, FLASH_RING_PACKET_SIZE);
    if (ret != W25QXX_OK) return ret;

    // 校報魔術字節與數據完整性 (CRC-16)
    if (pkt->magic[0] == 0xAA && pkt->magic[1] == 0x55) {
        uint16_t calc = ring_crc16((const uint8_t *)pkt, FLASH_RING_PACKET_SIZE - 2);
        if (calc == pkt->crc16) {
            return W25QXX_OK;
        }
    }

    return W25QXX_ERR_ID; // 若尚未有任何有效寫入，回傳 ID 錯誤
}

W25QXX_StatusTypeDef FlashRing_GetSecondLastPacket(FlashRingPacket_t *pkt)
{
    if (pkt == NULL) return W25QXX_ERR_PARAM;

    uint32_t last_write_addr = s_ring_write_addr;
    uint32_t last_packet_addr;

    // 處理環形邊界 wrap-around
    if (last_write_addr == FLASH_RINGBUF_ADDR) {
        last_packet_addr = FLASH_RINGBUF_END + 1 - FLASH_RING_PACKET_SIZE;
    } else {
        last_packet_addr = last_write_addr - FLASH_RING_PACKET_SIZE;
    }

    uint32_t second_last_packet_addr;
    if (last_packet_addr == FLASH_RINGBUF_ADDR) {
        second_last_packet_addr = FLASH_RINGBUF_END + 1 - FLASH_RING_PACKET_SIZE;
    } else {
        second_last_packet_addr = last_packet_addr - FLASH_RING_PACKET_SIZE;
    }

    // 從 Flash 中讀取倒數第二個封包
    W25QXX_StatusTypeDef ret = W25QXX_ReadData(second_last_packet_addr, (uint8_t*)pkt, FLASH_RING_PACKET_SIZE);
    if (ret != W25QXX_OK) return ret;

    // 校驗魔術字節與數據完整性 (CRC-16)
    if (pkt->magic[0] == 0xAA && pkt->magic[1] == 0x55) {
        uint16_t calc = ring_crc16((const uint8_t *)pkt, FLASH_RING_PACKET_SIZE - 2);
        if (calc == pkt->crc16) {
            return W25QXX_OK;
        }
    }

    return W25QXX_ERR_ID; // 若尚未有任何有效寫入，回傳 ID 錯誤
}

/* ============================================================
 *  靜態區 (Sector 0 / Sector 1-15) 讀寫實作
 * ============================================================ */

W25QXX_StatusTypeDef Flash_WriteSysFlags(FlashSysFlags_t *flags)
{
    if (flags == NULL) return W25QXX_ERR_PARAM;

    // 1. 計算並填充 CRC16 (覆蓋前 60 bytes)
    flags->crc16 = ring_crc16((const uint8_t *)flags, sizeof(FlashSysFlags_t) - 2);

    // 2. 擦除 Sector 0 (4KB) —— 警告：此操作會阻塞約 300ms
    W25QXX_StatusTypeDef ret = W25QXX_EraseSector(0x000000UL);
    if (ret != W25QXX_OK) return ret;

    // 3. 寫入資料
    return W25QXX_WriteData(0x000000UL, (const uint8_t *)flags, sizeof(FlashSysFlags_t));
}

W25QXX_StatusTypeDef Flash_ReadSysFlags(FlashSysFlags_t *flags)
{
    if (flags == NULL) return W25QXX_ERR_PARAM;

    // 1. 讀取 Sector 0 開頭資料
    W25QXX_StatusTypeDef ret = W25QXX_ReadData(0x000000UL, (uint8_t *)flags, sizeof(FlashSysFlags_t));
    if (ret != W25QXX_OK) return ret;

    // 2. 驗證 CRC16
    uint16_t calc = ring_crc16((const uint8_t *)flags, sizeof(FlashSysFlags_t) - 2);
    if (calc != flags->crc16) {
        return W25QXX_ERR_ID; // CRC 校驗失敗，可能尚未校準或資料損毀
    }

    return W25QXX_OK;
}

W25QXX_StatusTypeDef Flash_WriteMissionSummary(FlashMissionSummary_t *summary)
{
    if (summary == NULL) return W25QXX_ERR_PARAM;

    // 1. 計算並填充 CRC16 (覆蓋前 96 bytes)
    summary->crc16 = ring_crc16((const uint8_t *)summary, sizeof(FlashMissionSummary_t) - 2);

    // 2. 擦除 Sector 1 (4KB)
    W25QXX_StatusTypeDef ret = W25QXX_EraseSector(0x001000UL);
    if (ret != W25QXX_OK) return ret;

    // 3. 寫入資料
    return W25QXX_WriteData(0x001000UL, (const uint8_t *)summary, sizeof(FlashMissionSummary_t));
}

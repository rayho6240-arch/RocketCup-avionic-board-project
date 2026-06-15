/**
 ******************************************************************************
 * @file    lora_e80.c
 * @brief   E80-900M2213S 920MHz LoRa 驅動 (SX126x/LLCC68 命令相容, SPI3)
 ******************************************************************************
 */
#include "lora_e80.h"
#include "main.h"        /* LORA920_* / CSB_LORA920 腳位巨集 */
#include "spi3_bus.h"    /* SPI3 共用匯流排互斥鎖 */

/* ============================================================
 *  ★ RF 組態（上板 bring-up 須對照 E80-900M2213S 規格書與地面站逐項驗證）
 * ============================================================ */
#define E80_RF_FREQ_HZ        920000000UL /* 載波頻率 (Hz)，須符地面站與當地法規 */
#define E80_TX_POWER_DBM      22          /* 發射功率 (dBm)，E80-2213S 高功率 PA 上限 +22 */
#define E80_LORA_SF           0x09U       /* 展頻因子 SF9（LLCC68/SX1262 皆合法） */
#define E80_LORA_BW           0x04U       /* 頻寬 125kHz（0x04） */
#define E80_LORA_CR           0x01U       /* 編碼率 4/5（0x01） */
#define E80_LORA_LDRO         0x00U       /* Low Data Rate Optimize（SF9/BW125 關閉） */
#define E80_PREAMBLE_LEN      8U          /* 前導碼符號數 */
#define E80_SYNCWORD_MSB      0x14U       /* 私有網路 sync word 高位（0x1424 私有 / 0x3444 公有） */
#define E80_SYNCWORD_LSB      0x24U       /* 私有網路 sync word 低位 */
#define E80_USE_DIO2_RF_SW    0           /* 先關閉，確認 SPI 基本通訊後再開 */
#define E80_USE_TCXO          0           /* 先關閉，確認 SPI 基本通訊後再開；若模組為 XTAL 版永久保持 0 */
#define E80_TCXO_VOLTAGE      0x07U       /* DIO3→TCXO 電壓：0x07=3.3V（依模組） */
#define E80_TCXO_DELAY        320U        /* TCXO 啟動延遲（×15.625us ≈ 5ms） */

/* ============================================================
 *  SX126x opcode
 * ============================================================ */
#define SX_SET_STANDBY        0x80U
#define SX_SET_PACKET_TYPE    0x8AU
#define SX_SET_RF_FREQUENCY   0x86U
#define SX_SET_PA_CONFIG      0x95U
#define SX_SET_TX_PARAMS      0x8EU
#define SX_SET_BUFFER_BASE    0x8FU
#define SX_SET_MOD_PARAMS     0x8BU
#define SX_SET_PACKET_PARAMS  0x8CU
#define SX_SET_DIO_IRQ        0x08U
#define SX_SET_DIO2_RFSW      0x9DU
#define SX_SET_DIO3_TCXO      0x97U
#define SX_CALIBRATE          0x89U
#define SX_CLR_IRQ_STATUS     0x02U
#define SX_WRITE_REGISTER     0x0DU
#define SX_READ_REGISTER      0x1DU
#define SX_WRITE_BUFFER       0x0EU
#define SX_SET_TX             0x83U

#define SX_STANDBY_RC         0x00U
#define SX_PACKET_TYPE_LORA   0x01U
#define SX_REG_LORA_SYNCWORD  0x0740U   /* LoRa sync word 高位暫存器 */

#define SX_IRQ_TX_DONE        0x0001U
#define SX_IRQ_TIMEOUT        0x0200U

#define E80_BUSY_TIMEOUT_MS   20U     /* 等 BUSY 拉低逾時（命令處理一般 <ms） */
#define E80_SPI_TIMEOUT_MS    20U     /* SPI 傳輸逾時 */
#define E80_TX_TIMEOUT_MS     1500U   /* 一筆封包空中傳輸 + TxDone 上限（保守） */

/* ============================================================
 *  狀態
 * ============================================================ */
static SPI_HandleTypeDef *s_hspi = NULL;
static volatile uint8_t   s_tx_done = 0;
static uint8_t            s_tx_in_progress = 0;
static uint32_t           s_tx_start_tick = 0;
static uint8_t            s_inited = 0;
static uint8_t            s_disabled = 0;   /* 1 = 已 Shutdown 隔離：拒絕一切 SPI3 觸碰 */
/* 初始化診斷：存起來供週期性遙測輸出（開機太早、序列埠來不及接） */
static int     s_init_rd_st      = -1;
static uint8_t s_init_busy       = 0xFF;
static uint8_t s_init_rb[2]      = {0, 0};
static uint8_t s_get_status_byte = 0xFF;  /* GetStatus 回傳值：0x22=STBY_RC 正常 */

/* --- CS 控制 --- */
#define E80_CS_LOW()   HAL_GPIO_WritePin(CSB_LORA920_GPIO_Port, CSB_LORA920_Pin, GPIO_PIN_RESET)
#define E80_CS_HIGH()  HAL_GPIO_WritePin(CSB_LORA920_GPIO_Port, CSB_LORA920_Pin, GPIO_PIN_SET)

/* BUSY(PD6) HIGH=忙線；等待拉低，含逾時。不觸 SPI，故不需持鎖。 */
static HAL_StatusTypeDef e80_wait_busy(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LORA920_BUSY_GPIO_Port, LORA920_BUSY_Pin) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/* 送一筆命令 (opcode + params)。整段 CS 拉低期間持 SPI3 鎖。 */
static HAL_StatusTypeDef e80_cmd(uint8_t opcode, const uint8_t *params, uint16_t n)
{
    HAL_StatusTypeDef st;
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, &opcode, 1, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && n > 0) {
        st = HAL_SPI_Transmit(s_hspi, (uint8_t *)params, n, E80_SPI_TIMEOUT_MS);
    }
    E80_CS_HIGH();
    SPI3_Bus_Unlock();
    return st;
}

/* 寫暫存器 (opcode 0x0D + addr16 + data)。 */
static HAL_StatusTypeDef e80_write_reg(uint16_t addr, const uint8_t *data, uint16_t n)
{
    HAL_StatusTypeDef st;
    uint8_t hdr[3] = { SX_WRITE_REGISTER, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF) };
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, hdr, 3, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && n > 0) st = HAL_SPI_Transmit(s_hspi, (uint8_t *)data, n, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    SPI3_Bus_Unlock();
    return st;
}

/* 讀暫存器 (opcode 0x1D + addr16 + NOP + data)。 */
static HAL_StatusTypeDef e80_read_reg(uint16_t addr, uint8_t *data, uint16_t n)
{
    HAL_StatusTypeDef st;
    uint8_t hdr[4] = { SX_READ_REGISTER, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF), 0x00 };
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, hdr, 4, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && n > 0) st = HAL_SPI_Receive(s_hspi, data, n, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    SPI3_Bus_Unlock();
    return st;
}

/* 寫 TX FIFO (opcode 0x0E + offset + data)。 */
static HAL_StatusTypeDef e80_write_buffer(uint8_t offset, const uint8_t *data, uint8_t n)
{
    HAL_StatusTypeDef st;
    uint8_t hdr[2] = { SX_WRITE_BUFFER, offset };
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, hdr, 2, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && n > 0) st = HAL_SPI_Transmit(s_hspi, (uint8_t *)data, n, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    SPI3_Bus_Unlock();
    return st;
}

/* 硬體重置：RST(PD5) 拉低脈衝後等 BUSY 就緒。 */
static void e80_reset(void)
{
    HAL_GPIO_WritePin(LORA920_RST_GPIO_Port, LORA920_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(LORA920_RST_GPIO_Port, LORA920_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    (void)e80_wait_busy(100U);
}

/* 設定 LoRa 封包參數（含本次 payload 長度）。 */
static HAL_StatusTypeDef e80_set_packet_params(uint8_t payload_len)
{
    uint8_t pp[6] = {
        (uint8_t)(E80_PREAMBLE_LEN >> 8), (uint8_t)(E80_PREAMBLE_LEN & 0xFF),
        0x00,          /* 顯式表頭 (variable length) */
        payload_len,   /* payload 長度 */
        0x01,          /* CRC on */
        0x00           /* 標準 IQ */
    };
    return e80_cmd(SX_SET_PACKET_PARAMS, pp, 6);
}

HAL_StatusTypeDef LoRaE80_Init(SPI_HandleTypeDef *hspi)
{
    s_hspi           = hspi;
    s_tx_done        = 0;
    s_tx_in_progress = 0;
    s_inited         = 0;
    s_disabled       = 0;   /* 重新嘗試啟用：解除先前的隔離標記 */

    E80_CS_HIGH();
    e80_reset();

    /* 記錄 reset 後 BUSY 初始狀態（0=LOW=就緒；1=HIGH=仍忙/未回應） */
    s_init_busy = HAL_GPIO_ReadPin(LORA920_BUSY_GPIO_Port, LORA920_BUSY_Pin);

    /* GetStatus (0xC0)：不等 BUSY，直接問晶片狀態。
     * MISO[1]=status：0x22=STBY_RC(正常)；0x00=MISO 接地；0xFF=MISO 浮空；其他=異常 */
    {
        uint8_t gs_tx[2] = {0xC0, 0x00};
        uint8_t gs_rx[2] = {0x00, 0x00};
        SPI3_Bus_Lock();
        E80_CS_LOW();
        HAL_SPI_Transmit(s_hspi, &gs_tx[0], 1, 5);
        HAL_SPI_Receive (s_hspi, &gs_rx[1], 1, 5);
        E80_CS_HIGH();
        SPI3_Bus_Unlock();
        s_get_status_byte = gs_rx[1];  /* 0x22=STBY_RC 正常; 0x00=MISO接地; 0xFF=浮空 */
    }

    /* 早期 MISO 匯流排健康檢查：0x00=MISO 被拉接地、0xFF=MISO 浮空，兩者皆表示
     * E80 未正常驅動共用匯流排（晶片未上電/焊接不良/故障）。此時不再送後續設定
     * 命令（避免在壞匯流排上瞎跑、且每筆都觸碰 SPI3），直接回報失敗 —— 由 main
     * 接住後呼叫 LoRaE80_Shutdown() 把模組按在 reset，保護同匯流排的 Flash。 */
    if (s_get_status_byte == 0x00 || s_get_status_byte == 0xFF) {
        return HAL_ERROR;
    }

    /* Standby (RC) */
    uint8_t standby = SX_STANDBY_RC;
    e80_cmd(SX_SET_STANDBY, &standby, 1);

#if E80_USE_TCXO
    /* DIO3 供電 TCXO + 重新全區塊校準。
     * Calibrate(0x7F) 最多需要 ~50ms；必須等 BUSY 拉低才能繼續。
     * 若用固定 HAL_Delay 很容易不夠，這裡改等 BUSY 確實結束。 */
    uint8_t tcxo[4] = { E80_TCXO_VOLTAGE,
                        (uint8_t)((E80_TCXO_DELAY >> 16) & 0xFF),
                        (uint8_t)((E80_TCXO_DELAY >> 8) & 0xFF),
                        (uint8_t)(E80_TCXO_DELAY & 0xFF) };
    e80_cmd(SX_SET_DIO3_TCXO, tcxo, 4);
    (void)e80_wait_busy(20U);       /* 等 TCXO 啟動穩定 */
    uint8_t calib = 0x7F;           /* 校準全部區塊（RC64k/RC13M/PLL/ADC/Image） */
    e80_cmd(SX_CALIBRATE, &calib, 1);
    (void)e80_wait_busy(200U);      /* 全區塊校準最壞情況 ~50ms，給 200ms 充裕 */
#endif

#if E80_USE_DIO2_RF_SW
    /* DIO2 控制內部 RF 收發開關 */
    uint8_t rfsw = 0x01;
    e80_cmd(SX_SET_DIO2_RFSW, &rfsw, 1);
#endif

    /* LoRa 封包型態 */
    uint8_t ptype = SX_PACKET_TYPE_LORA;
    e80_cmd(SX_SET_PACKET_TYPE, &ptype, 1);

    /* RF 頻率：Frf = freq_hz * 2^25 / 32MHz */
    uint32_t frf = (uint32_t)(((uint64_t)E80_RF_FREQ_HZ << 25) / 32000000ULL);
    uint8_t freq[4] = { (uint8_t)(frf >> 24), (uint8_t)(frf >> 16), (uint8_t)(frf >> 8), (uint8_t)frf };
    e80_cmd(SX_SET_RF_FREQUENCY, freq, 4);

    /* PA 設定（SX1262/LLCC68 高功率：+22dBm） */
    uint8_t pa[4] = { 0x04, 0x07, 0x00, 0x01 };
    e80_cmd(SX_SET_PA_CONFIG, pa, 4);

    /* 發射功率 + ramp time 200us(0x04) */
    uint8_t txp[2] = { (uint8_t)E80_TX_POWER_DBM, 0x04 };
    e80_cmd(SX_SET_TX_PARAMS, txp, 2);

    /* TX/RX FIFO base address 皆 0 */
    uint8_t base[2] = { 0x00, 0x00 };
    e80_cmd(SX_SET_BUFFER_BASE, base, 2);

    /* 調變參數：SF / BW / CR / LDRO */
    uint8_t mod[4] = { E80_LORA_SF, E80_LORA_BW, E80_LORA_CR, E80_LORA_LDRO };
    e80_cmd(SX_SET_MOD_PARAMS, mod, 4);

    /* 預設封包參數（payload 先給 1，發送時再依實際長度重設） */
    e80_set_packet_params(1);

    /* sync word（私有網路），與地面站須一致 */
    uint8_t sw[2] = { E80_SYNCWORD_MSB, E80_SYNCWORD_LSB };
    e80_write_reg(SX_REG_LORA_SYNCWORD, sw, 2);

    /* DIO IRQ：TxDone | Timeout 對映到 DIO1 */
    uint16_t irq = SX_IRQ_TX_DONE | SX_IRQ_TIMEOUT;
    uint8_t dio[8] = { (uint8_t)(irq >> 8), (uint8_t)(irq & 0xFF),   /* IRQ mask */
                       (uint8_t)(irq >> 8), (uint8_t)(irq & 0xFF),   /* DIO1 mask */
                       0x00, 0x00, 0x00, 0x00 };                     /* DIO2/DIO3 mask = 0 */
    e80_cmd(SX_SET_DIO_IRQ, dio, 8);

    /* 驗活：讀回剛寫入的 sync word */
    uint8_t rb[2] = { 0, 0 };
    s_init_rd_st  = (int)e80_read_reg(SX_REG_LORA_SYNCWORD, rb, 2);
    s_init_busy   = HAL_GPIO_ReadPin(LORA920_BUSY_GPIO_Port, LORA920_BUSY_Pin);
    s_init_rb[0]  = rb[0];
    s_init_rb[1]  = rb[1];
    if (s_init_rd_st != HAL_OK) {
        return HAL_ERROR;
    }
    if (rb[0] != E80_SYNCWORD_MSB || rb[1] != E80_SYNCWORD_LSB) {
        return HAL_ERROR;
    }

    s_inited = 1;
    return HAL_OK;
}

void LoRaE80_Shutdown(void)
{
    /* 先釋放 CS，再把 RST 拉低並保持：SX126x NRST=低 → 全腳高阻，E80 自 SPI3
     * 共用線（SCK/MISO/MOSI）斷開，不再有機會驅動/污染 Flash 的讀寫交易。
     * 純 GPIO 操作（不碰 SPI/mutex），scheduler 啟動前亦可安全呼叫。 */
    E80_CS_HIGH();
    HAL_GPIO_WritePin(LORA920_RST_GPIO_Port, LORA920_RST_Pin, GPIO_PIN_RESET);

    s_inited         = 0;
    s_disabled       = 1;
    s_tx_in_progress = 0;
    s_tx_done        = 0;
}

uint8_t LoRaE80_IsReady(void)
{
    if (s_disabled || !s_inited) return 0U;   /* 已隔離或未初始化 → 不觸碰 SPI3 */
    if (s_tx_in_progress) return 0U;
    return (e80_wait_busy(E80_BUSY_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

HAL_StatusTypeDef LoRaE80_Send(const uint8_t *data, uint8_t len)
{
    uint8_t clr[2] = { 0xFF, 0xFF };

    if (s_disabled || !s_inited || s_hspi == NULL || data == NULL || len == 0) {
        return HAL_ERROR;   /* 已隔離（含 LORA920_ENABLE=0）→ 絕不觸碰 SPI3 */
    }

    /* 背壓：上一筆 TX 是否完成？ */
    if (s_tx_in_progress) {
        if (s_tx_done) {
            s_tx_in_progress = 0;
            e80_cmd(SX_CLR_IRQ_STATUS, clr, 2);   /* 收尾清 IRQ */
        } else if ((HAL_GetTick() - s_tx_start_tick) > E80_TX_TIMEOUT_MS) {
            s_tx_in_progress = 0;                 /* 逾時，放行重試 */
        } else {
            return HAL_BUSY;                      /* 仍在空中傳輸，本次跳過 */
        }
    }

    /* 啟動新一筆 TX */
    s_tx_done = 0;

    uint8_t standby = SX_STANDBY_RC;
    e80_cmd(SX_SET_STANDBY, &standby, 1);
    if (e80_set_packet_params(len) != HAL_OK)      return HAL_ERROR;
    e80_cmd(SX_CLR_IRQ_STATUS, clr, 2);
    if (e80_write_buffer(0x00, data, len) != HAL_OK) return HAL_ERROR;

    /* SetTx，timeout=0 → 單次發送，TxDone 後自動回 standby */
    uint8_t tx[3] = { 0x00, 0x00, 0x00 };
    if (e80_cmd(SX_SET_TX, tx, 3) != HAL_OK)       return HAL_ERROR;

    s_tx_in_progress = 1;
    s_tx_start_tick  = HAL_GetTick();
    /* 不在此空等 TxDone（不持鎖）；由下次呼叫或 IsReady 經 DIO1 旗標收尾 */
    return HAL_OK;
}

void LoRaE80_OnDio1IRQ(void)
{
    s_tx_done = 1;
}

/* DIO1(PD4/EXTI4) 上升緣中斷 → 設定 TxDone 旗標。
 * 覆寫 HAL 弱定義；專案其餘 EXTI 腳目前不使用回呼，故僅處理 LoRa DIO1。 */
void LoRaE80_GetInitDiag(int *rd_st, uint8_t *busy, uint8_t *rb0, uint8_t *rb1, uint8_t *gs)
{
    if (rd_st) *rd_st = s_init_rd_st;
    if (busy)  *busy  = s_init_busy;
    if (rb0)   *rb0   = s_init_rb[0];
    if (rb1)   *rb1   = s_init_rb[1];
    if (gs)    *gs    = s_get_status_byte;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LORA920_INT_Pin) {
        LoRaE80_OnDio1IRQ();
    }
}

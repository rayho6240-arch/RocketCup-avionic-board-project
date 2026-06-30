/**
 ******************************************************************************
 * @file    lora_e80.c
 * @brief   E80-900M2213S 920MHz LoRa 驅動 (Semtech LR1121, SPI3)
 *
 * ★★ 重要：E80-900M2213S 的核心是 Semtech **LR1121**（非 SX126x）。LR1121 的
 *    SPI 命令協定與 SX126x 截然不同：
 *      - opcode 為 16-bit（2 bytes），如 SetRfFrequency = 0x020B
 *      - 頻率直接以 Hz（4 bytes 大端）帶入，不用 SX126x 的 Frf=f·2^25/32e6 公式
 *      - 讀取命令回應走「獨立的第二次 SPI 交易」：先送 opcode，等 BUSY 拉低，
 *        再拉 CS 讀 [Stat1][資料…]
 *      - LoRa 封包型態 = 0x02、LoRa sync word 為「單一位元組」命令 0x022B
 *      - IRQ 遮罩為 32-bit；IRQ 狀態由 GetStatus(0x0100) 回應內含
 *      - PA 為 HP PA；模組內建 **RF 開關接在 LR1121 DIO5/DIO6**，必須以
 *        SetDioAsRfSwitch(0x0112) 設定，否則收發 RF 完全不通（最關鍵的上板項）
 *
 * 接線（連線和基本硬體規格表.md）：SPI3(SCK=PB3/MISO=PB4/MOSI=PB5),
 *   CS=PD7(CSB_LORA920), RST=PD5, IRQ(LR1121 DIO9)→PD4(EXTI4 rising), BUSY=PD6。
 * SPI3 與 W25Q128 Flash 共用 → 經 spi3_bus.h 的 SPI3_Bus_Lock/Unlock 互斥。
 *
 * ★ 下列 #define 為「板級」參數，務必於上板 bring-up 對照 E80-900M2213S 規格書
 *   逐項驗證（見 docs 操作手冊「上板驗證清單」）：
 *     - E80_RFSW_*：RF 開關真值表（DIO5=RFSW0 / DIO6=RFSW1），預設採 Semtech
 *       LR1121 參考設計，E80 接線若不同則收不到封包。
 *     - E80_USE_TCXO：E80 用 32MHz 主動式振盪器；若為自供電（clipped-sine 進 XTA）
 *       則保持 0；若 bring-up 顯示時鐘異常再開。
 *     - IRQ 對應之 LR1121 DIO：預設假設模組 INT 腳接 LR1121 DIO9（IRQ 走 dio1 遮罩）。
 ******************************************************************************
 */
#include "lora_e80.h"
#include "main.h"        /* LORA920_* / CSB_LORA920 腳位巨集 */
#include "spi3_bus.h"    /* SPI3 共用匯流排互斥鎖 */
#include "lora_calc.h"   /* 純換算：頻率位元組 / LDRO（與 host 測試共用） */
#include <stdio.h>
#include <string.h>

/* ============================================================
 *  ★ RF 組態（上板 bring-up 須對照 E80-900M2213S 規格書與地面站逐項驗證）
 * ============================================================ */
#define E80_RF_FREQ_HZ        920000000UL /* 載波頻率 (Hz)，回復為 920MHz 以匹配 E80 天線與硬體帶通濾波器 */
#define E80_TX_POWER_DBM      22          /* 發射功率 (dBm)，E80-2213S HP PA 上限 +22 */
#define E80_LORA_SF           0x09U       /* 展頻因子 SF9（LR1121 值 = SF 數字） */
#define E80_LORA_BW           0x05U       /* 頻寬 250kHz（0x05）加大頻寬提升資料率 */
#define E80_LORA_CR           0x01U       /* 編碼率 4/5（0x01） */
#define E80_PREAMBLE_LEN      8U          /* 前導碼符號數 */
#define E80_LORA_SYNCWORD     0x12U       /* LoRa sync word（單一位元組）：0x12 私有 / 0x34 公有，須與對端一致 */

#define E80_USE_TCXO          0           /* 0=自供電 XTA 振盪器（E80 屬此）；1=LR1121 供電 TCXO。
                                             ★實測：設 1 反而讓晶片在 SPI 上完全不回應(gs 0x98→0x00)，
                                             證實 E80 為自供電振盪器、非 LR1121-TCXO，故保持 0。 */
#define E80_TCXO_VOLTAGE      0x07U       /* TCXO 電壓：0x07=3.3V（依模組，僅 E80_USE_TCXO=1 生效） */
#define E80_TCXO_DELAY        0x000140UL  /* TCXO 啟動延遲（×30.52us，0x140=320≈9.8ms） */
#define E80_USE_DCDC          0           /* 0=LDO（保守）；1=DC-DC（須外部電感，E80 多含） */

/* ---- RF 開關真值表 ★依 Ebyte E80-xxxM2213S 使用手冊 v1.1 第 5 頁（DIO5/RFSW0=bit0, DIO6/RFSW1=bit1）----
 * ⚠ 手冊注3 明示：E80 的開關控制狀態「與 SEMTECH 官方 SDK 預設不同」（原驅動誤用 Semtech 預設值）：
 *     DIO5 DIO6  狀態
 *      0    0    RX
 *      0    1    TX Sub-GHz 低功率 (LP)
 *      1    0    TX Sub-GHz 高功率 (HP) ← 主航電下行主用
 *      1    1    TX 2.4GHz
 * enable: 哪些 DIO 充當 RF 開關；其餘為各模式下 DIO5/DIO6 的高低電平組合。 */
#define E80_RFSW_ENABLE       0x03U       /* DIO5+DIO6 皆作 RF 開關 */
#define E80_RFSW_STBY         0x00U       /* 待機：全低（= RX 路徑，安全） */
#define E80_RFSW_RX           0x00U       /* 接收：DIO5=0,DIO6=0（地面站主用） */
#define E80_RFSW_TX           0x02U       /* TX 低功率 LP：DIO5=0,DIO6=1 */
#define E80_RFSW_TX_HP        0x01U       /* TX 高功率 HP（+22dBm，下行主用）：DIO5=1,DIO6=0 */
#define E80_RFSW_TX_HF        0x03U       /* TX 2.4GHz：DIO5=1,DIO6=1（本專案不用） */
#define E80_RFSW_GNSS         0x00U
#define E80_RFSW_WIFI         0x00U

/* ============================================================
 *  LR1121 opcode（16-bit）
 * ============================================================ */
/* System */
#define LR_GET_STATUS         0x0100U
#define LR_GET_VERSION        0x0101U
#define LR_CALIBRATE          0x010FU
#define LR_SET_REGMODE        0x0110U
#define LR_SET_DIO_RFSW       0x0112U
#define LR_SET_DIO_IRQ        0x0113U
#define LR_CLEAR_IRQ          0x0114U
#define LR_SET_TCXO           0x0117U
#define LR_SET_STANDBY        0x011CU
/* RegMem（FIFO 緩衝） */
#define LR_WRITE_BUFFER       0x0109U
#define LR_READ_BUFFER        0x010AU
/* Radio */
#define LR_GET_RXBUF_STATUS   0x0203U
#define LR_GET_PKT_STATUS     0x0204U
#define LR_SET_TX_CW          0x0208U
#define LR_SET_RX             0x0209U
#define LR_SET_TX             0x020AU
#define LR_SET_RF_FREQ        0x020BU
#define LR_SET_PKT_TYPE       0x020EU
#define LR_SET_MOD_PARAMS     0x020FU
#define LR_SET_PKT_PARAMS     0x0210U
#define LR_SET_TX_PARAMS      0x0211U
#define LR_SET_PA_CFG         0x0215U
#define LR_SET_RX_BOOSTED     0x0227U
#define LR_SET_LORA_SYNCWORD  0x022BU

#define LR_STANDBY_RC         0x00U
#define LR_PKT_TYPE_LORA      0x02U

/* IRQ 位元（32-bit） */
#define LR_IRQ_TX_DONE        0x00000004UL
#define LR_IRQ_RX_DONE        0x00000008UL
#define LR_IRQ_PREAMBLE       0x00000010UL
#define LR_IRQ_HEADER_ERR     0x00000040UL
#define LR_IRQ_CRC_ERR        0x00000080UL
#define LR_IRQ_TIMEOUT        0x00000400UL

#define E80_BUSY_TIMEOUT_MS   20U     /* 等 BUSY 拉低逾時（命令處理一般 <ms） */
#define E80_SPI_TIMEOUT_MS    20U     /* SPI 傳輸逾時 */
#define E80_TX_TIMEOUT_MS     1500U   /* 一筆封包空中傳輸 + TxDone 上限（保守） */

/* ============================================================
 *  狀態
 * ============================================================ */
static SPI_HandleTypeDef *s_hspi = NULL;
static volatile uint8_t   s_tx_done = 0;
static volatile uint8_t   s_rx_event = 0;   /* DIO IRQ 觸發（地面站 RX：RxDone/CrcErr/Timeout） */
static uint8_t            s_tx_in_progress = 0;
static uint32_t           s_tx_start_tick = 0;
static uint8_t            s_inited = 0;
static uint8_t            s_disabled = 0;    /* 1 = 已 Shutdown 隔離：拒絕一切 SPI3 觸碰 */
static uint16_t           s_preamble = E80_PREAMBLE_LEN;  /* 目前前導碼（Reconfig 可改） */
static uint8_t            s_stat1 = 0xFF;    /* 最近一次讀回的 LR1121 Stat1 */

/* 初始化診斷：存起來供週期性遙測輸出（開機太早、序列埠來不及接） */
static int     s_init_rd_st      = -1;
static uint8_t s_init_busy       = 0xFF;
static uint8_t s_init_ver[2]     = {0, 0};  /* GetVersion: [0]=HW, [1]=Type(0x03=LR1121) */
static uint8_t s_get_status_byte = 0xFF;    /* GetStatus 的 Stat1 */

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

/* ============================================================
 *  LR1121 SPI 基本交易
 * ============================================================ */

/* 寫入型命令：CS 低 → [opcode_hi, opcode_lo, params…] → CS 高。整段持 SPI3 鎖。 */
static HAL_StatusTypeDef lr_cmd(uint16_t opcode, const uint8_t *params, uint16_t n)
{
    HAL_StatusTypeDef st;
    uint8_t hdr[2] = { (uint8_t)(opcode >> 8), (uint8_t)(opcode & 0xFF) };
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, hdr, 2, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && n > 0) st = HAL_SPI_Transmit(s_hspi, (uint8_t *)params, n, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    if (st == HAL_OK) {
        st = e80_wait_busy(E80_BUSY_TIMEOUT_MS);
    }
    SPI3_Bus_Unlock();
    return st;
}

/* 讀取型命令（LR1121 兩段式）：
 *   交易1：CS 低 → 送 opcode(+params) → CS 高；
 *   等 BUSY 拉低（晶片備妥回應）；
 *   交易2：CS 低 → 第一位元組為 Stat1（存 s_stat1），續讀 rn 位元組資料 → CS 高。 */
static HAL_StatusTypeDef lr_read(uint16_t opcode, const uint8_t *params, uint16_t pn,
                                 uint8_t *rdata, uint16_t rn)
{
    HAL_StatusTypeDef st;
    uint8_t hdr[2] = { (uint8_t)(opcode >> 8), (uint8_t)(opcode & 0xFF) };
    uint8_t stat1 = 0xFF;
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    /* 交易1：送命令 */
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, hdr, 2, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && pn > 0) st = HAL_SPI_Transmit(s_hspi, (uint8_t *)params, pn, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    if (st != HAL_OK) { SPI3_Bus_Unlock(); return st; }
    /* 等晶片備妥回應 */
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    /* 交易2：讀 Stat1 + 資料 */
    E80_CS_LOW();
    st = HAL_SPI_Receive(s_hspi, &stat1, 1, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && rn > 0) st = HAL_SPI_Receive(s_hspi, rdata, rn, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    SPI3_Bus_Unlock();
    s_stat1 = stat1;
    return st;
}

/* 寫 TX FIFO（LR1121 WriteBuffer8：opcode + data，無 offset，從緩衝起點寫整筆）。 */
static HAL_StatusTypeDef lr_write_buffer(const uint8_t *data, uint8_t n)
{
    HAL_StatusTypeDef st;
    uint8_t hdr[2] = { (uint8_t)(LR_WRITE_BUFFER >> 8), (uint8_t)(LR_WRITE_BUFFER & 0xFF) };
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, hdr, 2, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && n > 0) st = HAL_SPI_Transmit(s_hspi, (uint8_t *)data, n, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    SPI3_Bus_Unlock();
    return st;
}

/* 讀 RX FIFO（LR1121 ReadBuffer8：opcode + offset + len，等 BUSY 後讀 Stat1 + data）。 */
static HAL_StatusTypeDef lr_read_buffer(uint8_t offset, uint8_t *data, uint8_t n)
{
    HAL_StatusTypeDef st;
    uint8_t hdr[4] = { (uint8_t)(LR_READ_BUFFER >> 8), (uint8_t)(LR_READ_BUFFER & 0xFF), offset, n };
    uint8_t stat1 = 0xFF;
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, hdr, 4, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    if (st != HAL_OK) { SPI3_Bus_Unlock(); return st; }
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Receive(s_hspi, &stat1, 1, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && n > 0) st = HAL_SPI_Receive(s_hspi, data, n, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    SPI3_Bus_Unlock();
    s_stat1 = stat1;
    return st;
}

/* ============================================================
 *  共用設定片段
 * ============================================================ */

/* 硬體重置：RST(PD5) 拉低脈衝後等 BUSY 就緒。 */
static void e80_reset(void)
{
    HAL_GPIO_WritePin(LORA920_RST_GPIO_Port, LORA920_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LORA920_RST_GPIO_Port, LORA920_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    (void)e80_wait_busy(100U);
}

/* 設定 LoRa 封包參數（前導碼 s_preamble、顯式表頭、CRC on、標準 IQ、本次 payload 長度）。 */
static HAL_StatusTypeDef e80_set_packet_params(uint8_t payload_len)
{
    uint8_t pp[6] = {
        (uint8_t)(s_preamble >> 8), (uint8_t)(s_preamble & 0xFF),
        0x00,          /* 顯式表頭（variable length） */
        payload_len,   /* payload 長度 */
        0x01,          /* CRC on */
        0x00           /* 標準 IQ */
    };
    return lr_cmd(LR_SET_PKT_PARAMS, pp, 6);
}

/* 設定 RF 開關（★板級真值表）。LR1121 必須設定，否則收發 RF 不通。 */
static HAL_StatusTypeDef e80_set_rf_switch(void)
{
    uint8_t rfsw[8] = {
        E80_RFSW_ENABLE, E80_RFSW_STBY, E80_RFSW_RX, E80_RFSW_TX,
        E80_RFSW_TX_HP,  E80_RFSW_TX_HF, E80_RFSW_GNSS, E80_RFSW_WIFI
    };
    return lr_cmd(LR_SET_DIO_RFSW, rfsw, 8);
}

/* 設定 DIO IRQ 遮罩（32-bit ×2：dio1 / dio2）。事件放 dio1（假設模組 INT=LR1121 DIO9）。 */
static HAL_StatusTypeDef e80_set_dio_irq(uint32_t irq)
{
    uint8_t dio[8] = {
        (uint8_t)(irq >> 24), (uint8_t)(irq >> 16), (uint8_t)(irq >> 8), (uint8_t)irq, /* dio1 */
        0x00, 0x00, 0x00, 0x00                                                          /* dio2 */
    };
    return lr_cmd(LR_SET_DIO_IRQ, dio, 8);
}

static HAL_StatusTypeDef e80_clear_irq(void)
{
    uint8_t clr[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    return lr_cmd(LR_CLEAR_IRQ, clr, 4);
}

static HAL_StatusTypeDef e80_set_standby_rc(void)
{
    uint8_t sb = LR_STANDBY_RC;
    return lr_cmd(LR_SET_STANDBY, &sb, 1);
}

/* 套用調變參數（SF/BW/CR + 自動 LDRO）+ 載波頻率 + 發射功率。 */
static HAL_StatusTypeDef e80_apply_rf(uint32_t freq_hz, uint8_t sf, uint8_t bw,
                                      uint8_t cr, int8_t pwr_dbm)
{
    HAL_StatusTypeDef st;

    uint8_t fb[4];
    lr1121_freq_to_bytes(freq_hz, fb);
    st = lr_cmd(LR_SET_RF_FREQ, fb, 4);
    if (st != HAL_OK) return st;

    uint8_t mod[4] = { sf, bw, cr, lora_ldro_required(sf, bw) };
    st = lr_cmd(LR_SET_MOD_PARAMS, mod, 4);
    if (st != HAL_OK) return st;

    uint8_t txp[2] = { (uint8_t)pwr_dbm, 0x04 /* ramp 80us */ };
    return lr_cmd(LR_SET_TX_PARAMS, txp, 2);
}

/* 讀 GetStatus 的 32-bit IRQ 狀態（回應 = Stat1 + Stat2 + Irq[31:0]）。 */
static HAL_StatusTypeDef e80_get_irq(uint32_t *irq_out)
{
    uint8_t b[5] = {0};   /* [0]=Stat2, [1..4]=Irq 大端 */
    HAL_StatusTypeDef st = lr_read(LR_GET_STATUS, NULL, 0, b, 5);
    if (st != HAL_OK) return st;
    if (irq_out) {
        *irq_out = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16) |
                   ((uint32_t)b[3] << 8)  | (uint32_t)b[4];
    }
    s_get_status_byte = s_stat1;
    return HAL_OK;
}

/* ============================================================
 *  初始化
 * ============================================================ */
HAL_StatusTypeDef LoRaE80_Init(SPI_HandleTypeDef *hspi)
{
    s_hspi           = hspi;
    s_tx_done        = 0;
    s_tx_in_progress = 0;
    s_inited         = 1;
    s_disabled       = 0;
    s_preamble       = E80_PREAMBLE_LEN;

    E80_CS_HIGH();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);  /* 強制拉高 W25Q128 CS，防止 SPI3 匯流排干擾 */
    e80_reset();
    s_init_busy = HAL_GPIO_ReadPin(LORA920_BUSY_GPIO_Port, LORA920_BUSY_Pin);

    e80_set_standby_rc();

#if E80_USE_DCDC
    uint8_t reg = 0x01; lr_cmd(LR_SET_REGMODE, &reg, 1);   /* DC-DC */
#endif

#if E80_USE_TCXO
    /* DIO→TCXO 供電 + 重新校準（校準最長 ~50ms，等 BUSY 確實結束） */
    uint8_t tcxo[4] = { E80_TCXO_VOLTAGE,
                        (uint8_t)((E80_TCXO_DELAY >> 16) & 0xFF),
                        (uint8_t)((E80_TCXO_DELAY >> 8) & 0xFF),
                        (uint8_t)(E80_TCXO_DELAY & 0xFF) };
    lr_cmd(LR_SET_TCXO, tcxo, 4);
    (void)e80_wait_busy(20U);
    uint8_t calib = 0x3F;   /* 校準各區塊 */
    lr_cmd(LR_CALIBRATE, &calib, 1);
    (void)e80_wait_busy(200U);
#endif

    /* ★ RF 開關（板級；不設則收發不通） */
    e80_set_rf_switch();

    /* LoRa 封包型態 */
    uint8_t ptype = LR_PKT_TYPE_LORA;
    lr_cmd(LR_SET_PKT_TYPE, &ptype, 1);

    /* PA 設定（HP PA → +22dBm）。
     * ★ LR1121 datasheet SetPaConfig：HP PA(PaSel=0x01) 的 regPaSupply 必須為
     *   0x01（VBAT）；用 0x00（內部 LDO）餵 HP PA 規格不支援 → 輸出極弱/近乎無，
     *   正是「晶片 rdy 但 SDR 收不到」的典型主因。Semtech +22dBm 參考組合：
     *   PaSel=0x01, regPaSupply=0x01(VBAT), paDutyCycle=0x04, paHpSel=0x07。 */
    uint8_t pa[4] = { 0x01, 0x01, 0x04, 0x07 };
    lr_cmd(LR_SET_PA_CFG, pa, 4);

    /* 頻率 / 調變 / 發射功率 */
    e80_apply_rf(E80_RF_FREQ_HZ, E80_LORA_SF, E80_LORA_BW, E80_LORA_CR, E80_TX_POWER_DBM);

    /* 封包參數（payload 先給 1，發送/接收時再依實際長度重設） */
    e80_set_packet_params(1);

    /* LoRa sync word（單一位元組，須與對端一致） */
    uint8_t sw = E80_LORA_SYNCWORD;
    lr_cmd(LR_SET_LORA_SYNCWORD, &sw, 1);

    /* 預設 IRQ：TxDone | Timeout（主航電 TX 用；地面站 StartRx 會改為 RX 事件） */
    e80_set_dio_irq(LR_IRQ_TX_DONE | LR_IRQ_TIMEOUT);

    /* 驗活：GetVersion（Type=0x03 為 LR1121）+ MISO 健康檢查（全 0x00 接地 / 0xFF 浮空） */
    uint8_t ver[4] = {0, 0, 0, 0};   /* [0]=HW, [1]=Type, [2]=FW major, [3]=FW minor */
    s_init_rd_st  = (int)lr_read(LR_GET_VERSION, NULL, 0, ver, 4);
    s_init_ver[0] = ver[0];
    s_init_ver[1] = ver[1];

    /* GetStatus 的 Stat1（順帶清初始 IRQ） */
    uint32_t irq0 = 0;
    e80_get_irq(&irq0);
    e80_clear_irq();

    /* 在線判定（強化）：
     *  - GetVersion 全 0x00 / 全 0xFF → MISO 接地/浮空，晶片沒回應
     *  - GetStatus 的 Stat1（gs）為 0x00 / 0xFF → 同樣是 MISO 沒被晶片驅動
     *  - GetVersion 的 Type 必須為 0x03（LR1121 簽章；0x02=LR1120, 0x01=LR1110）
     * 任一不符即視為未偵測到 → 隔離 SPI3、回 HAL_ERROR（誠實回報，不謊報 rdy）。
     * 真正在線的 LR1121：Type=0x03、Stat1 為有效模式位元，皆會通過。 */
    uint8_t all_zero  = (ver[0] == 0x00 && ver[1] == 0x00 && ver[2] == 0x00 && ver[3] == 0x00);
    uint8_t all_ff    = (ver[0] == 0xFF && ver[1] == 0xFF && ver[2] == 0xFF && ver[3] == 0xFF);
    uint8_t gs_dead   = (s_get_status_byte == 0x00 || s_get_status_byte == 0xFF);
    uint8_t bad_type  = (ver[1] != 0x03);   /* Type 非 LR1121 */
    if (s_init_rd_st != (int)HAL_OK || all_zero || all_ff || gs_dead || bad_type) {
        LoRaE80_Shutdown();
        return HAL_ERROR;
    }

    s_inited   = 1;
    s_disabled = 0;
    return HAL_OK;
}

void LoRaE80_Shutdown(void)
{
    /* CS 釋放 + RST 拉低保持：LR1121 NRST=低時全腳高阻，自 SPI3 共用線斷開，
     * 不再驅動/污染 Flash 的讀寫交易。純 GPIO（不碰 SPI/mutex），可早於 scheduler 呼叫。 */
    E80_CS_HIGH();
    HAL_GPIO_WritePin(LORA920_RST_GPIO_Port, LORA920_RST_Pin, GPIO_PIN_RESET);

    s_inited         = 0;
    s_disabled       = 1;
    s_tx_in_progress = 0;
    s_tx_done        = 0;
}

uint8_t LoRaE80_IsReady(void)
{
    if (s_disabled || !s_inited) return 0U;
    if (s_tx_in_progress) return 0U;
    return (e80_wait_busy(E80_BUSY_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

/* ============================================================
 *  發送（主航電下行）
 * ============================================================ */
HAL_StatusTypeDef LoRaE80_Send(const uint8_t *data, uint8_t len)
{
    if (s_disabled || !s_inited || s_hspi == NULL || data == NULL || len == 0) {
        return HAL_ERROR;
    }

    /* 背壓：上一筆 TX 是否完成？ */
    if (s_tx_in_progress) {
        if (s_tx_done) {
            s_tx_in_progress = 0;
            e80_clear_irq();
        } else if ((HAL_GetTick() - s_tx_start_tick) > E80_TX_TIMEOUT_MS) {
            s_tx_in_progress = 0;   /* 逾時，放行重試 */
        } else {
            return HAL_BUSY;        /* 仍在空中傳輸，本次跳過 */
        }
    }

    s_tx_done = 0;

    e80_set_standby_rc();
    e80_set_rf_switch();   /* 強制切換天線開關至 TX 模式 */
    e80_set_dio_irq(LR_IRQ_TX_DONE | LR_IRQ_TIMEOUT);
    if (e80_set_packet_params(len) != HAL_OK)       return HAL_ERROR;
    e80_clear_irq();
    if (lr_write_buffer(data, len) != HAL_OK)       return HAL_ERROR;

    /* SetTx，timeout=0 → 單次發送，TxDone 後自動回 standby */
    uint8_t tx[3] = { 0x00, 0x00, 0x00 };
    if (lr_cmd(LR_SET_TX, tx, 3) != HAL_OK)         return HAL_ERROR;

    s_tx_in_progress = 1;
    s_tx_start_tick  = HAL_GetTick();
    return HAL_OK;
}

/* ============================================================
 *  接收（地面站）
 * ============================================================ */
HAL_StatusTypeDef LoRaE80_StartRx(void)
{
    if (s_disabled || !s_inited || s_hspi == NULL) return HAL_ERROR;

    e80_set_standby_rc();
    if (e80_set_packet_params(255) != HAL_OK) return HAL_ERROR;   /* RX payload 上限 */

    uint8_t boost = 0x01;
    lr_cmd(LR_SET_RX_BOOSTED, &boost, 1);   /* 提升接收靈敏度 */

    e80_set_dio_irq(LR_IRQ_RX_DONE | LR_IRQ_CRC_ERR | LR_IRQ_HEADER_ERR | LR_IRQ_TIMEOUT);
    e80_clear_irq();
    s_rx_event = 0;

    /* 連續接收：timeout = 0xFFFFFF（RxContinuous） */
    uint8_t rx[3] = { 0xFF, 0xFF, 0xFF };
    return lr_cmd(LR_SET_RX, rx, 3);
}

uint8_t LoRaE80_RxReady(void)
{
    if (s_disabled || !s_inited) return 0U;
    return s_rx_event ? 1U : 0U;
}

HAL_StatusTypeDef LoRaE80_ReadPacket(uint8_t *buf, uint8_t *len, int16_t *rssi_dbm, int16_t *snr_q)
{
    if (s_disabled || !s_inited || s_hspi == NULL || buf == NULL || len == NULL) {
        return HAL_ERROR;
    }
    s_rx_event = 0;

    /* IRQ 狀態（LR1121 含於 GetStatus 回應） */
    uint32_t irq = 0;
    if (e80_get_irq(&irq) != HAL_OK) return HAL_TIMEOUT;
    if (!(irq & LR_IRQ_RX_DONE)) return HAL_BUSY;      /* 尚無完整封包 */

    e80_clear_irq();                                   /* 連續 RX 維持 */

    if (irq & (LR_IRQ_CRC_ERR | LR_IRQ_HEADER_ERR)) return HAL_ERROR;   /* 壞包丟棄 */

    /* RX 緩衝狀態：[0]=payload 長度, [1]=起始指標 */
    uint8_t rbs[2] = {0, 0};
    if (lr_read(LR_GET_RXBUF_STATUS, NULL, 0, rbs, 2) != HAL_OK) return HAL_TIMEOUT;
    uint8_t plen = rbs[0];
    uint8_t pptr = rbs[1];
    if (plen == 0) return HAL_ERROR;

    if (lr_read_buffer(pptr, buf, plen) != HAL_OK) return HAL_TIMEOUT;
    *len = plen;

    /* 封包品質：[0]=rssi_pkt, [1]=snr_pkt, [2]=signal_rssi_pkt */
    uint8_t ps[3] = {0, 0, 0};
    if (lr_read(LR_GET_PKT_STATUS, NULL, 0, ps, 3) == HAL_OK) {
        if (rssi_dbm) *rssi_dbm = (int16_t)(-(int)ps[0] / 2);   /* RSSI(dBm) = -rssi_pkt/2 */
        if (snr_q)    *snr_q    = (int16_t)((int8_t)ps[1]);     /* SNR 原始值（dB = ÷4，沿用 gs_log 契約） */
    }
    return HAL_OK;
}

/* ============================================================
 *  動態重配置（地面站通訊測試）
 * ============================================================ */
HAL_StatusTypeDef LoRaE80_Reconfig(uint32_t freq_hz, uint8_t sf, uint8_t bw,
                                    uint8_t cr, int8_t pwr_dbm, uint16_t preamble)
{
    if (s_disabled || !s_inited || s_hspi == NULL) return HAL_ERROR;

    HAL_StatusTypeDef st = e80_set_standby_rc();
    if (st != HAL_OK) return st;

    s_preamble = preamble;

    st = e80_apply_rf(freq_hz, sf, bw, cr, pwr_dbm);
    if (st != HAL_OK) return st;

    st = e80_set_packet_params(255);   /* RX 模式上限 */
    if (st != HAL_OK) return st;

    /* 重新進入連續接收（保持地面站 RX 模式） */
    uint8_t boost = 0x01;
    lr_cmd(LR_SET_RX_BOOSTED, &boost, 1);
    e80_set_dio_irq(LR_IRQ_RX_DONE | LR_IRQ_CRC_ERR | LR_IRQ_HEADER_ERR | LR_IRQ_TIMEOUT);
    e80_clear_irq();
    s_rx_event = 0;

    uint8_t rx[3] = { 0xFF, 0xFF, 0xFF };
    return lr_cmd(LR_SET_RX, rx, 3);
}

/* ============================================================
 *  IRQ / 診斷
 * ============================================================ */
void LoRaE80_OnDio1IRQ(void)
{
    s_tx_done  = 1;   /* TX 路徑（主航電）：TxDone */
    s_rx_event = 1;   /* RX 路徑（地面站）：DIO 觸發，實際類型由 GetStatus IRQ 判 */
}

void LoRaE80_GetInitDiag(int *rd_st, uint8_t *busy, uint8_t *rb0, uint8_t *rb1, uint8_t *gs)
{
    if (rd_st) *rd_st = s_init_rd_st;
    if (busy)  *busy  = s_init_busy;
    if (rb0)   *rb0   = s_init_ver[0];   /* GetVersion HW */
    if (rb1)   *rb1   = s_init_ver[1];   /* GetVersion Type（0x03=LR1121） */
    if (gs)    *gs    = s_get_status_byte;
}

/* DIO IRQ（PD4/EXTI4）上升緣中斷 → 設定事件旗標。覆寫 HAL 弱定義。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LORA920_INT_Pin) {
        LoRaE80_OnDio1IRQ();
    }
}

void LoRaE80_PrintConfig(void)
{
    printf("[LORA 920MHz] E80-2213S   | Freq: %.3f MHz | Power: +%ddBm | BW: %ukHz | SF: %u | CR: 4/5 | SyncWord: 0x%02X\r\n",
           (double)E80_RF_FREQ_HZ / 1000000.0,
           E80_TX_POWER_DBM,
           (unsigned)lora_bw_to_khz(E80_LORA_BW),
           (unsigned)E80_LORA_SF,
           (unsigned)E80_LORA_SYNCWORD);
}



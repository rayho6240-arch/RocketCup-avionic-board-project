/**
 ******************************************************************************
 * @file    lora_e22.c
 * @brief   E22-400T30S 433MHz LoRa 透傳模式驅動 (UART3)
 ******************************************************************************
 */
#include "lora_e22.h"
#include "main.h"   /* LORA433_* 腳位巨集 */
#include <stdio.h>
#include <string.h>

#define LORA_E22_AUX_BOOT_TIMEOUT_MS  1000U  /* 開機/重置後等 AUX 拉高上限 */
#define LORA_E22_TX_TIMEOUT_MS        100U   /* UART 阻塞傳輸逾時 */

/* 頻率設定：E22-400T30S 頻率 = 410 + CH (MHz)，CH 寫入 EEPROM 暫存器 0x05 */
#define E22_FREQ_BASE_MHZ  410U
#define E22_TX_FREQ_MHZ    432U
#define E22_CH             ((uint8_t)(E22_TX_FREQ_MHZ - E22_FREQ_BASE_MHZ))

/* 發射功率等級（REG1(0x04) bit[1:0]）：0=30dBm 1=27dBm 2=24dBm 3=21dBm。
 * 本板 3V3 供電無法穩定驅動 30dBm（1W，突波電流 ~600mA 會把 3V3 拉垮→模組欠壓，
 * 表現為「發幾秒就斷」）。降到最低 21dBm 大幅減少突波電流。
 * 註：E22-400T30S 無 22dBm 檔位，21dBm 為最接近且最省電者。 */
#define E22_TX_POWER_LEVEL 3U   /* 21dBm */

/* 空中速率等級（REG0(0x03) bit[2:0]）：0=0.3k 1=1.2k 2=2.4k 3=4.8k 4=9.6k 5=19.2k 6=38.4k 7=62.5k。
 * 空中速率越低 → 靈敏度/鏈路餘裕越好、射程越遠、抗雜訊越強（代價是資料率低）。
 * 模組先前被(舊版)設成 62.5k → 台面測試就大量 CRC 錯（raw↑ 但 crc↑↑、ok 極少）。
 * 降到 2.4k 增加 ~15-20dB 餘裕；配合遙測降速 divider 不會塞爆緩衝。
 * ★兩端(火箭/地面站)必須相同才能通訊——此值兩板共用同一韌體巨集，保證一致。 */
#define E22_AIR_RATE       2U   /* 2.4k */

/* REG0(0x03)/REG1(0x04)/REG3(0x06) 暫存器位元定義（EByte E22-400T30S User Manual）：
 *   REG0 bit[7:5]=UART baud, bit[4:3]=parity, bit[2:0]=空中速率(air data rate)
 *   REG1 bit[7:6]=子封包長度, bit[5]=RSSI雜訊致能, bit[1:0]=發射功率
 *   REG3 bit[7]=RSSI byte致能, bit[6]=傳輸模式(0=透傳/1=定點)
 * 僅供 LoRaE22_PrintConfig() 解碼開機讀回的暫存器供人工比對雙端（地面站/主航電）
 * 是否一致；韌體不主動改寫這些暫存器（寫錯 UART baud 位會讓模組與 MCU 直接
 * 失聯、無法再讀回救援），一致性靠 bring-up 時比對雙端 log 人工確認。 */

static const char *const s_air_rate_str[8] = {
    "0.3k", "1.2k", "2.4k", "4.8k", "9.6k", "19.2k", "38.4k", "62.5k"
};
static const char *const s_uart_baud_str[8] = {
    "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"
};
static const uint32_t s_uart_baud_val[8] = {
    1200U, 2400U, 4800U, 9600U, 19200U, 38400U, 57600U, 115200U
};
static const char *const s_parity_str[4] = { "8N1", "8O1", "8E1", "8N1" };
static const char *const s_tx_power_str[4] = { "30dBm", "27dBm", "24dBm", "21dBm" };
static const char *const s_subpkt_str[4] = { "240B", "128B", "64B", "32B" };

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t             s_inited = 0;

/* AUX(PE11)：HIGH=空閒。等待拉高，含逾時保護。回傳 1=就緒, 0=逾時。 */
static uint8_t e22_wait_aux_high(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LORA433_BUSY_GPIO_Port, LORA433_BUSY_Pin) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return 0;
        }
    }
    return 1;
}

static uint8_t s_e22_cfg[7] = {0};
static uint8_t s_e22_cfg_valid = 0;

/* 透傳模式下 MCU↔E22 的 UART baud 必須等於模組 REG0(0x03) 設定值（只有「設定模式」才固定
 * 9600、與 REG0 無關）。開機 e22_probe() 已把 REG0 讀進 s_e22_cfg[3]；此處解出模組實際 baud，
 * 讓 MCU 跟著模組走，不再寫死 115200。
 *   —— 先前寫死 115200 對上模組實際 9600（開機 log 的 UART=9600bps），透傳位元框全錯：
 *      發射端灌 115200 → E22 以 9600 取樣得亂碼再上空；接收端 E22 吐 9600 → MCU 以 115200
 *      取樣得亂碼。偵測走設定模式固定 9600 故 log 報 OK，實際卻整條收不到。
 * 尚未 probe 到（模組未偵測）時回落 9600（E22 出廠預設）。 */
static uint32_t e22_transparent_baud(void)
{
    if (s_e22_cfg_valid) {
        return s_uart_baud_val[(s_e22_cfg[3] >> 5) & 0x07];
    }
    return 9600U;
}

/* 進入設定模式，寫入指定頻道後回透傳模式（帶 ch 參數版，供外部呼叫） */
static HAL_StatusTypeDef e22_write_channel(uint8_t ch)
{
    /* 切設定模式（M1=1, M0=0），等 AUX HIGH（模式切換完成） */
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    if (!e22_wait_aux_high(200)) {
        HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
        return HAL_TIMEOUT;
    }

    /* Config mode 固定 9600 baud */
    s_huart->Init.BaudRate = 9600;
    HAL_UART_Init(s_huart);

    /* 讀回 CH 暫存器，若已一致則跳過寫入 */
    uint8_t rd[3] = {0xC1, 0x05, 0x01};
    HAL_UART_Transmit(s_huart, rd, sizeof(rd), 50);
    uint8_t rd_resp[4] = {0};
    HAL_UART_Receive(s_huart, rd_resp, sizeof(rd_resp), 100);
    printf("[LORA433] CH read-back: %02X %02X %02X %02X (want C1 05 01 %02X)\r\n",
           rd_resp[0], rd_resp[1], rd_resp[2], rd_resp[3], ch);

    HAL_StatusTypeDef ret = HAL_OK;
    if (rd_resp[0] == 0xC1 && rd_resp[3] == ch) {
        printf("[LORA433] CH already %d (%u MHz), skip write\r\n",
               ch, (unsigned)(E22_FREQ_BASE_MHZ + ch));
    } else {
        uint8_t wr[4] = {0xC0, 0x05, 0x01, ch};
        HAL_UART_Transmit(s_huart, wr, sizeof(wr), 50);
        uint8_t wr_resp[4] = {0};
        HAL_UART_Receive(s_huart, wr_resp, sizeof(wr_resp), 100);
        printf("[LORA433] CH write resp: %02X %02X %02X %02X\r\n",
               wr_resp[0], wr_resp[1], wr_resp[2], wr_resp[3]);
        if (!e22_wait_aux_high(300)) ret = HAL_TIMEOUT;
        else {
            if (s_e22_cfg_valid) s_e22_cfg[5] = ch; // update cache
        }
    }

    /* 恢復透傳模式 baud（跟隨模組實際 REG0 設定，非寫死值） */
    s_huart->Init.BaudRate = e22_transparent_baud();
    HAL_UART_Init(s_huart);

    /* 回透傳模式（M1=0） */
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    return ret;
}

/* 寫入 REG1(0x04) 發射功率位元[1:0]，保留分包/雜訊等其他位元。
 * REG1 不含 UART baud（那在 REG0），故不影響 MCU↔E22 baud 一致性；
 * 也不含 RSSI-append（那在 REG3 bit7），故不影響地面站收包 framing。
 * 需先 probe 到 REG1（保留其他位元）才能寫；已是目標值則跳過（省一次寫入、加快後續開機）。 */
static HAL_StatusTypeDef e22_write_power(uint8_t pwr_level)
{
    if (!s_e22_cfg_valid) return HAL_ERROR;

    if ((s_e22_cfg[4] & 0x03) == (pwr_level & 0x03)) {
        printf("[LORA433] TX power already level %u (%s), skip write\r\n",
               (unsigned)(pwr_level & 0x03), s_tx_power_str[pwr_level & 0x03]);
        return HAL_OK;
    }

    /* 切設定模式（M1=1, M0=0），等 AUX HIGH */
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    if (!e22_wait_aux_high(200)) {
        HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
        return HAL_TIMEOUT;
    }

    /* Config mode 固定 9600 baud */
    s_huart->Init.BaudRate = 9600;
    HAL_UART_Init(s_huart);

    uint8_t reg1 = (uint8_t)((s_e22_cfg[4] & ~0x03) | (pwr_level & 0x03));
    uint8_t wr[4] = {0xC0, 0x04, 0x01, reg1};
    HAL_UART_Transmit(s_huart, wr, sizeof(wr), 50);
    uint8_t wr_resp[4] = {0};
    HAL_UART_Receive(s_huart, wr_resp, sizeof(wr_resp), 100);
    printf("[LORA433] REG1 power write resp: %02X %02X %02X %02X (set %s)\r\n",
           wr_resp[0], wr_resp[1], wr_resp[2], wr_resp[3], s_tx_power_str[pwr_level & 0x03]);

    HAL_StatusTypeDef ret = HAL_OK;
    if (!e22_wait_aux_high(300)) ret = HAL_TIMEOUT;
    else s_e22_cfg[4] = reg1;   /* 更新快取，使 LoRaE22_PrintConfig 顯示新功率 */

    /* 恢復透傳模式 baud（跟隨模組實際 REG0 設定）+ 回透傳模式（M1=0） */
    s_huart->Init.BaudRate = e22_transparent_baud();
    HAL_UART_Init(s_huart);
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    return ret;
}

/* 寫入 REG0(0x03) 空中速率位元[2:0]，保留 UART baud[7:5] 與 parity[4:3]。
 * 只動 air-rate 位、不動 baud 位 → 不影響 MCU↔E22 baud 一致性（無 bricking 風險）。
 * 兩端 air rate 必須相同才能通訊；已是目標值則跳過。
 * 註：改寫後 e22_transparent_baud() 仍讀 baud 位（未變），故還原 baud 正確。 */
static HAL_StatusTypeDef e22_write_airrate(uint8_t air_rate)
{
    if (!s_e22_cfg_valid) return HAL_ERROR;

    if ((s_e22_cfg[3] & 0x07) == (air_rate & 0x07)) {
        printf("[LORA433] AirRate already %s, skip write\r\n", s_air_rate_str[air_rate & 0x07]);
        return HAL_OK;
    }

    /* 切設定模式（M1=1, M0=0），等 AUX HIGH */
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    if (!e22_wait_aux_high(200)) {
        HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
        return HAL_TIMEOUT;
    }

    /* Config mode 固定 9600 baud */
    s_huart->Init.BaudRate = 9600;
    HAL_UART_Init(s_huart);

    uint8_t reg0 = (uint8_t)((s_e22_cfg[3] & ~0x07) | (air_rate & 0x07));   /* 保留 baud[7:5]+parity[4:3] */
    uint8_t wr[4] = {0xC0, 0x03, 0x01, reg0};
    HAL_UART_Transmit(s_huart, wr, sizeof(wr), 50);
    uint8_t wr_resp[4] = {0};
    HAL_UART_Receive(s_huart, wr_resp, sizeof(wr_resp), 100);
    printf("[LORA433] REG0 airrate write resp: %02X %02X %02X %02X (set %s)\r\n",
           wr_resp[0], wr_resp[1], wr_resp[2], wr_resp[3], s_air_rate_str[air_rate & 0x07]);

    HAL_StatusTypeDef ret = HAL_OK;
    if (!e22_wait_aux_high(300)) ret = HAL_TIMEOUT;
    else s_e22_cfg[3] = reg0;   /* 更新快取；baud 位未變，e22_transparent_baud 仍正確 */

    /* 恢復透傳模式 baud（baud 位未動，仍是原值）+ 回透傳模式（M1=0） */
    s_huart->Init.BaudRate = e22_transparent_baud();
    HAL_UART_Init(s_huart);
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    return ret;
}

/* 模組在線偵測：透傳模式無握手，故進設定模式回讀全部 7 個設定暫存器。
 * 在線 → 回 `C1 00 07 <7位元組>`；未接/接線錯/故障 → UART 無回應。回傳 1=在線, 0=無回應。 */
static uint8_t e22_probe(void)
{
    if (s_huart == NULL) return 0;

    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_SET);  /* 進設定模式 */
    HAL_Delay(20);
    (void)e22_wait_aux_high(100);

    s_huart->Init.BaudRate = 9600;     /* 設定模式固定 9600 */
    HAL_UART_Init(s_huart);

    uint8_t present = 0;
    for (int attempt = 0; attempt < 2 && !present; attempt++) {
        uint8_t rd[3] = {0xC1, 0x00, 0x07};         /* 讀全部 7 個設定暫存器 */
        uint8_t resp[10] = {0};
        HAL_UART_Transmit(s_huart, rd, sizeof(rd), 50);
        HAL_UART_Receive(s_huart, resp, sizeof(resp), 200);
        if (resp[0] == 0xC1 && resp[1] == 0x00 && resp[2] == 0x07) {
            present = 1;
            memcpy(s_e22_cfg, &resp[3], 7);
            s_e22_cfg_valid = 1;
        }
    }

    s_huart->Init.BaudRate = e22_transparent_baud();   /* 還原透傳 baud：跟隨模組回讀值 */
    HAL_UART_Init(s_huart);
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);  /* 回透傳模式 */
    HAL_Delay(20);
    return present;
}

HAL_StatusTypeDef LoRaE22_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    /* 透傳模式 M1=0, M0=0 */
    HAL_GPIO_WritePin(LORA433_M0_GPIO_Port, LORA433_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);

    /* 硬體重置脈衝：RST 拉低 ~10ms 再釋放（RST 低電平有效） */
    HAL_GPIO_WritePin(LORA433_RST_GPIO_Port, LORA433_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA433_RST_GPIO_Port, LORA433_RST_Pin, GPIO_PIN_SET);
    /* 等待 E22 模組開機完成（等待 AUX 腳位拉高，防開機太快回讀失敗） */
    (void)e22_wait_aux_high(LORA_E22_AUX_BOOT_TIMEOUT_MS);
    HAL_Delay(200);   /* 增加 200ms 穩定延時，給 E22 內部 MCU 充足開機時間 */

    s_inited = 1;   /* 鏈路恆標記可用（透傳模式發送靠 AUX 背壓）；偵測結果另由回傳值表示 */

    /* 設定模式回讀偵測模組是否真的在線（誠實回報；呼叫端據此印訊息 / 主航電每 10s 重試）。 */
    if (!e22_probe()) {
        return HAL_TIMEOUT;
    }

    /* probe 已把 MCU UART baud 對齊到模組實際 REG0 值（見 e22_transparent_baud）；
     * 此處只再「強制寫入固定頻道 E22_CH」，保證主航電/地面站兩端一定落在同一頻率，
     * 不依賴各模組 EEPROM 殘留的舊頻道。只寫 CH(REG2)，刻意不動 baud/air rate
     * （空中速率維持模組現值以保留射程；避免改寫 REG0 UART 位的失聯風險）。
     * 頻道寫入為 best-effort：即使逾時，模組仍在線、鏈路可用，故仍回 HAL_OK；
     * 實際生效頻道由呼叫端隨後的 LoRaE22_PrintConfig() 印出供人工核對。 */
    (void)e22_write_channel(E22_CH);
    /* 3V3 供電：把發射功率降到 21dBm（見 E22_TX_POWER_LEVEL 註解）。
     * 已是目標值時內部會跳過寫入，故穩態開機不增加時間。best-effort，同上仍回 HAL_OK。 */
    (void)e22_write_power(E22_TX_POWER_LEVEL);
    /* 把空中速率降到 2.4k（見 E22_AIR_RATE 註解）：增加鏈路餘裕、解 62.5k 大量 CRC 錯。
     * 只寫 REG0 air-rate 位、保留 baud 位。已是目標值時跳過。best-effort，仍回 HAL_OK。 */
    (void)e22_write_airrate(E22_AIR_RATE);
    return HAL_OK;
}

uint8_t LoRaE22_IsReady(void)
{
    return s_inited;
}

HAL_StatusTypeDef LoRaE22_Send(const uint8_t *data, uint16_t len)
{
    if (!s_inited || s_huart == NULL || data == NULL || len == 0) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit(s_huart, (uint8_t *)data, len, LORA_E22_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef LoRaE22_SetFreqMHz(uint32_t freq_mhz)
{
    if (!s_inited || s_huart == NULL) return HAL_ERROR;
    if (freq_mhz < E22_FREQ_BASE_MHZ || freq_mhz > (E22_FREQ_BASE_MHZ + 83U)) {
        return HAL_ERROR;
    }
    uint8_t ch = (uint8_t)(freq_mhz - E22_FREQ_BASE_MHZ);
    return e22_write_channel(ch);   /* 只改頻道(REG2)，不動 baud/air rate */
}

HAL_StatusTypeDef LoRaE22_SetPowerLevel(uint8_t pwr_level)
{
    if (!s_inited || s_huart == NULL) return HAL_ERROR;
    if (pwr_level > 3U) return HAL_ERROR;
    return e22_write_power(pwr_level);   /* 只改 REG1 功率位，保留分包/雜訊位 */
}

HAL_StatusTypeDef LoRaE22_SetAirRate(uint8_t air_rate)
{
    if (!s_inited || s_huart == NULL) return HAL_ERROR;
    if (air_rate > 7U) return HAL_ERROR;
    return e22_write_airrate(air_rate);  /* 只改 REG0 空速位，保留 baud/parity 位 */
}

void LoRaE22_PrintConfig(void)
{
    /* s_e22_cfg[] 是 e22_probe() 讀回位址 0x00~0x06 共 7 個暫存器的快取：
     *   [0]=ADDH [1]=ADDL [2]=NETID [3]=REG0 [4]=REG1 [5]=REG2(CH) [6]=REG3
     * 注意：發射功率/傳輸模式在 REG1([4])/REG3([6])，不是同一個位元組，
     * 曾經誤把兩者都讀成 [6]（把 REG3 當 REG1）算出來的功率是錯的，已修正。 */
    if (s_e22_cfg_valid) {
        uint32_t freq_mhz  = E22_FREQ_BASE_MHZ + s_e22_cfg[5];
        uint8_t  reg0      = s_e22_cfg[3];
        uint8_t  reg1      = s_e22_cfg[4];
        uint8_t  reg3      = s_e22_cfg[6];
        const char *air_rate = s_air_rate_str[reg0 & 0x07];
        const char *parity   = s_parity_str[(reg0 >> 3) & 0x03];
        const char *baud     = s_uart_baud_str[(reg0 >> 5) & 0x07];
        const char *tx_power = s_tx_power_str[reg1 & 0x03];
        const char *subpkt   = s_subpkt_str[(reg1 >> 6) & 0x03];
        uint8_t      rssi_noise_en = (reg1 >> 5) & 0x01;
        const char  *mode    = (reg3 & 0x40) ? "Fixed" : "Transparent";

        printf("[LORA433] E22-400T30S | Freq=%lu.000MHz(CH=%u) | Power=%s | AirRate=%s | "
               "UART=%sbps/%s | SubPkt=%s | RSSIen=%u | Mode=%s\r\n",
               (unsigned long)freq_mhz, (unsigned)s_e22_cfg[5], tx_power, air_rate,
               baud, parity, subpkt, (unsigned)rssi_noise_en, mode);
    } else {
        printf("[LORA433] E22-400T30S | Freq=%u.000MHz(CH=%u) | (未回讀到暫存器，顯示韌體預設值，非模組實際值)\r\n",
               (unsigned)E22_TX_FREQ_MHZ, (unsigned)E22_CH);
    }
}

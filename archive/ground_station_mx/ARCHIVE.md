# 📦 已封存：GroundStation_MX（地面站接收端 CubeMX 設定）

> **狀態：Archived（封存）** — 設計已被取代，僅保留作存檔參考。

## 這是什麼

地面站接收端 MCU 的 **STM32CubeMX 專案設定** —— **只有組態，沒有任何生成的原始碼**。

| 檔案 | 說明 |
| :--- | :--- |
| `GroundStation_MX.ioc` | CubeMX 設定檔（用來生成接收站主控代碼） |
| `.project` | STM32CubeIDE / Eclipse 專案檔 |

### 組態內容

- **MCU**：`STM32F103C8Tx`（Blue Pill，F1 系列）
- **USART1**：GPS（PA9 TX / PA10 RX）
- **USART2**：LoRa 模組（PA2 TX / PA3 RX / PA4 AUX）
- **SPI1**（PA5 SCK / PA6 MISO / PA7 MOSI）、**SPI2**
- **USB**：CDC（PA11 DM / PA12 DP），連接主機
- **SWD**：PA13 / PA14

## 為何封存

依 [`hardware/ground_station/備注.md`](../../hardware/ground_station/備注.md)：獨立設計的地面站 PCB
**已取消送印打樣**，改為直接使用一套組裝好的 **BigAvionic 四層疊板（STM32F407）** 作為地面站接收硬體。

因此這份 F103 的 CubeMX 組態已被取代，不再作為韌體開發起點。

## 相關位置

- 地面站主機軟體 → [`ground_station/`](../../ground_station/)
- 地面站硬體 KiCad PCB（同為存檔性質）→ [`hardware/ground_station/`](../../hardware/ground_station/)
- 主航電韌體（現行地面站接收硬體所用平台）→ [`firmware/main_flight_code/`](../../firmware/main_flight_code/)

## 封存紀錄

- **原始位置**：`firmware/ground_station_mx/`
- 更早位置：`GroundStation/GroundStation_MX/`（repo 重整前）

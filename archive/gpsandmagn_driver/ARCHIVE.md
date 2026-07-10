# 📦 已封存：GPSandMagn 驅動實驗

> **狀態：Archived（封存）** — 僅保留作技術參考，已停止開發，非正式飛行韌體。

## 這是什麼

STM32CubeIDE 專案，在 **STM32F103C8（Blue Pill）** 上做 **GPS + 磁力計驅動 bring-up**。
目的是先在小板上把驅動跑通，再整進主航電。

| 項目 | 位置 |
| :--- | :--- |
| GPS 驅動（USART1） | `Core/Src/gps.c` |
| 磁力計驅動（I2C） | `Core/Src/mag.c`、`Core/Src/bsp_i2c.c` |
| CubeMX 設定 | `GPSandMagn_driver.ioc` |
| 變更紀錄 | `change.md` |

## 為何封存

本專案是獨立於飛行韌體的試作。正式使用中的 GPS / 磁力計驅動位於主航電
[`firmware/main_flight_code/`](../../firmware/main_flight_code/)（STM32F407）：

- `Core/Src/gps.c`、`Core/Inc/gps_parse.h`
- `Core/Src/mmc5983.c`（MMC5983 磁力計）

此 F103 專案不再維護。

## 封存紀錄

- **原始位置**：`firmware/scratch_drivers/GPSandMagn_driver/GPSandMagn_driver/`
- 封存時移除了 Eclipse workspace 中繼資料（`GPSandMagn_driver_WS/.metadata/`）——該目錄僅含 IDE 產生的 log、索引與鎖檔，不具保存價值。
- 專案本體（`.ioc`、`.cproject`、`Core/`、`Drivers/`）完整保留，可直接用 STM32CubeIDE 開啟。

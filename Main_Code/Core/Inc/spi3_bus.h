/**
 ******************************************************************************
 * @file    spi3_bus.h
 * @brief   SPI3 共用匯流排互斥鎖（W25Q128 Flash 與 E80 920MHz LoRa 共用 SPI3）
 *
 * SPI3 同時掛載 W25Q128 Flash (CS=PA15) 與 E80-900M2213S LoRa (CS=PD7)。
 * 兩者共用 SCK/MISO/MOSI，必須互斥存取以免同時拉低各自 CS 造成匯流排衝突。
 *
 * g_spi3_mutex 與下列包裝函式定義於 main.c。採遞迴 + 優先級繼承屬性。
 * NULL-guard：在 mutex 建立前（scheduler 啟動前的初始化階段）呼叫安全
 * ——該階段為單執行緒、Flash 獨佔，無需上鎖。
 *
 * 規則：
 *   - Flash 端：在每個高階操作（FlashRing 系列 / Flash SysFlags / Flash_DumpAll）外圍上鎖。
 *   - E80 端：僅在 SPI 命令突發（CS 拉低期間）持鎖；空中 TX 等待期間不持鎖
 *     （靠 DIO1/EXTI4 TxDone 旗標非阻塞輪詢），把持鎖時間壓到最短。
 ******************************************************************************
 */
#ifndef __SPI3_BUS_H
#define __SPI3_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 取得 SPI3 匯流排（阻塞至取得；mutex 未建立時為 no-op）。 */
void SPI3_Bus_Lock(void);

/** @brief 釋放 SPI3 匯流排（mutex 未建立或非持有者時安全忽略）。 */
void SPI3_Bus_Unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI3_BUS_H */

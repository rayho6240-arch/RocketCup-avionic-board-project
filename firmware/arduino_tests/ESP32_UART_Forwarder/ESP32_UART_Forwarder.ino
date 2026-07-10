/**
 * ESP32-S (NodeMCU-32S) UART2-to-USB Serial Bridge
 * 
 * 該程式讓 ESP32 透過硬體 UART2（引腳 16 與 17）讀取來自 STM32 航電板的 460800 bps 遙測數據，
 * 並將其無縫轉發至主 USB 連接埠（PC），以便用於 3D EKF 實時可視化與 HIL 測試。
 * 
 * 接線說明 (Wiring Guide):
 *   STM32 UART2 TX (板載 TX)   <--->  ESP32 GPIO 16 (RX2)
 *   STM32 UART2 RX (板載 RX)   <--->  ESP32 GPIO 17 (TX2) [可選]
 *   STM32 GND (共地)          <--->  ESP32 GND
 * 
 * 💡 提示：在進行此接線前，請確保兩端供電電壓共地。
 */

#include <Arduino.h>

// 定義 ESP32 NodeMCU-32S 的硬體 UART2 預設引腳
#define RX2 16
#define TX2 17

// 設定鮑率 (Match upgraded STM32 460800 bps rate)
#define SERIAL_BAUD 460800

void setup() {
  // 1. 初始化與 PC 連接的主 USB 串口
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ; // 等待串口連接 (僅某些 Native USB 開發板需要)
  }
  
  Serial.println("\r\n==================================================");
  Serial.println("[BOOT] ESP32-S (NodeMCU-32S) UART2-to-USB Bridge Ready!");
  Serial.println("==================================================");
  Serial.printf("  PC Baud Rate   : %d bps\r\n", SERIAL_BAUD);
  Serial.printf("  STM32 Baud Rate: %d bps\r\n", SERIAL_BAUD);
  Serial.println("  Connection Pinouts:");
  Serial.println("    ESP32 GPIO 16 (RX2) <---> STM32 UART2 TX Pin");
  Serial.println("    ESP32 GPIO 17 (TX2) <---> STM32 UART2 RX Pin [Optional]");
  Serial.println("    ESP32 GND           <---> STM32 GND (COMMON GROUND)");
  Serial.println("==================================================\r\n");

  // 2. 初始化與 STM32 連接的硬體 UART2 (Serial2)
  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, RX2, TX2);
}

void loop() {
  // A. 從 STM32 讀取數據 (Serial2 RX) 並轉發給 PC (Serial TX)
  while (Serial2.available() > 0) {
    char ch = Serial2.read();
    Serial.write(ch);
  }

  // B. 從 PC 讀取數據 (Serial RX) 並轉發給 STM32 (Serial2 TX) [可選，用於指令傳輸]
  while (Serial.available() > 0) {
    char ch = Serial.read();
    Serial2.write(ch);
  }
}

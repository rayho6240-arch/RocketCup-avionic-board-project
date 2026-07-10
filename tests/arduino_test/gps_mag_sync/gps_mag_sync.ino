/*
 * ====================================================================
 * NCKU ISP Avionics - GPS & Magnetometer (MMC5983MA) Sync Test Code
 * ====================================================================
 * 
 * 📌 [測試狀態 (Test Status)]：
 * 🟢 測試成功 (Test Successful)！
 * 
 * 頂層接口板 (Top Layer) 已經成功與 Arduino 開發板 (以 ESP32 作為測試硬體) 
 * 進行整合，並完成了 I2C 磁力計數據讀取與 UART2 GPS 數據解析的雙重同步測試。
 * 目前硬體實體走線與通信介面一切正常，正處於「等待與 STM32 主控板韌體整合」的狀態。
 * 
 * ====================================================================
 */

#include <Wire.h>

// ---------------- Hardwares Pin Configuration ----------------
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define I2C_SDA    21
#define I2C_SCL    22

// ---------------- MMC5983MA Register Map ----------------
#define MMC5983MA_ADDR     0x30
#define MMC_XOUT_0         0x00  // X-axis output MSB
#define MMC_TOUT           0x07  // Temperature output
#define MMC_STATUS         0x08  // Status register
#define MMC_CONTROL_0      0x09  // Control register 0
#define MMC_CONTROL_1      0x0A  // Control register 1

// ---------------- Global Variables ----------------
String gpsBuffer = "";
unsigned long lastLogTime = 0;
const unsigned long logInterval = 200; // 每 200ms (5Hz) 刷新一次 Log 數據

// GPS 解析暫存
String gpsTime = "00:00:00", gpsStatus = "No Fix", satCount = "00";
String latitude = "0.00000", longitude = "0.00000", altitude = "0.0";

// 磁力計數據暫存 (單位: Gauss)
float magX = 0.0, magY = 0.0, magZ = 0.0;

// ---------------- Function Prototypes ----------------
void initMMC5983MA();
void readMMC5983MA();
void parseGPS();
void printLog();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n==============================================");
  Serial.println("  NCKU ISP Avionics - GPS & Magnetometer Sync ");
  Serial.println("  [Status: Top Layer & Arduino Integration OK] ");
  Serial.println("==============================================");

  // 1. 初始化 I2C 與磁力計
  Wire.begin(I2C_SDA, I2C_SCL);
  initMMC5983MA();

  // 2. 初始化 GPS (UART2, 38400 bps)
  Serial2.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[INFO] UART2 GPS Listener Initialized at 38400 bps.");
  
  Serial.println("[INFO] System Ready. Starting Data Stream...\n");
}

void loop() {
  // 非阻塞讀取 GPS 串口資料
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      parseGPS();
      gpsBuffer = "";
    } else if (c != '\r') {
      gpsBuffer += c;
    }
  }

  // 定期讀取磁力計並噴出完整 Log (不使用 delay 避免卡死 GPS 串口)
  if (millis() - lastLogTime >= logInterval) {
    lastLogTime = millis();
    readMMC5983MA();
    printLog();
  }
}

// ---------------- MMC5983MA 驅動實作 ----------------
void initMMC5983MA() {
  // 軟體重置晶片
  Wire.beginTransmission(MMC5983MA_ADDR);
  Wire.write(MMC_CONTROL_1);
  Wire.write(0x80); // Software Reset
  Wire.endTransmission();
  delay(20);

  // 配置控制暫存器 0：啟用自動連續測量模式 (如 100Hz)
  Wire.beginTransmission(MMC5983MA_ADDR);
  Wire.write(MMC_CONTROL_0);
  Wire.write(0x08); // Set Auto_SR_en (自動充放電消除偏移)
  Wire.endTransmission();

  Wire.beginTransmission(MMC5983MA_ADDR);
  Wire.write(MMC_CONTROL_1);
  Wire.write(0x03); // 自定義輸出頻率 (CM_Freq = 100Hz)
  Wire.endTransmission();
  
  Serial.println("[INFO] MMC5983MA Magnetometer Initialized (100Hz Auto Mode).");
}

void readMMC5983MA() {
  // 觸發一次量測 (如果不是純自動模式)
  Wire.beginTransmission(MMC5983MA_ADDR);
  Wire.write(MMC_CONTROL_0);
  Wire.write(0x01); // TM_M (Take Measurement)
  Wire.endTransmission();

  // 讀取 7 個位元組的資料 (X, Y, Z 的高低位 + 18-bit 的額外位元)
  Wire.beginTransmission(MMC5983MA_ADDR);
  Wire.write(MMC_XOUT_0);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MMC5983MA_ADDR, 7);
  if (Wire.available() >= 7) {
    uint32_t x_raw = (uint32_t)Wire.read() << 10;
    uint32_t x_l   = Wire.read();
    uint32_t y_raw = (uint32_t)Wire.read() << 10;
    uint32_t y_l   = Wire.read();
    uint32_t z_raw = (uint32_t)Wire.read() << 10;
    uint32_t z_l   = Wire.read();
    uint8_t  xyz_ex = Wire.read(); // 18-bit 擴展位元

    // 補齊 18-bit 原始數據
    x_raw |= (x_l << 2) | ((xyz_ex >> 6) & 0x03);
    y_raw |= (y_l << 2) | ((xyz_ex >> 4) & 0x03);
    z_raw |= (z_l << 2) | ((xyz_ex >> 2) & 0x03);

    // 根據說明書，18-bit 模式下的感應靈敏度與零點偏置換算
    // 0 高斯對應的值為 2^17 = 131072，每高斯增益為 16384 LSB/Gauss
    magX = ((float)x_raw - 131072.0) / 16384.0;
    magY = ((float)y_raw - 131072.0) / 16384.0;
    magZ = ((float)z_raw - 131072.0) / 16384.0;
  }
}

// ---------------- GPS NMEA 核心輕量解析 ----------------
void parseGPS() {
  // 我們只抓含有最完整定位資訊認的 $GNGGA 或 $GPGGA 字串
  if (gpsBuffer.startsWith("$GNGGA") || gpsBuffer.startsWith("$GPGGA")) {
    int count = 0;
    int lastIdx = 0;
    
    // 用逗號切碎 NMEA 字串
    for (int i = 0; i < gpsBuffer.length(); i++) {
      if (gpsBuffer.charAt(i) == ',' || i == gpsBuffer.length() - 1) {
        String val = gpsBuffer.substring(lastIdx, i);
        val.trim();
        
        switch(count) {
          case 1: // UTC 時間 (hhmmss.ss)
            if (val.length() >= 6) {
              int hh = val.substring(0,2).toInt() + 8; // 轉成台灣時間 UTC+8
              if (hh >= 24) hh -= 24;
              gpsTime = (hh < 10 ? "0" + String(hh) : String(hh)) + ":" + val.substring(2,4) + ":" + val.substring(4,6);
            }
            break;
          case 2: // 緯度
            if (val.length() > 0) latitude = val;
            break;
          case 4: // 經度
            if (val.length() > 0) longitude = val;
            break;
          case 6: // 定位狀態 (0=無效, 1=有效GPS, 2=DGPS)
            if (val == "0") gpsStatus = "No Fix";
            else if (val == "1" || val == "2") gpsStatus = "3D Fix";
            break;
          case 7: // 衛星數量
            if (val.length() > 0) satCount = (val.toInt() < 10 ? "0" + val : val);
            break;
          case 9: // 海拔高度
            if (val.length() > 0) altitude = val;
            break;
        }
        lastIdx = i + 1;
        count++;
      }
    }
  }
}

// ---------------- 完整且結構化的 Log 印出 ----------------
void printLog() {
  // 使用標準的 csv/標籤化排版，一眼就能看出所有航電狀態
  Serial.print("[SYS_TIME:"); Serial.print(millis()); Serial.print("] ");
  
  // GPS 區塊
  Serial.print("| GPS_TIME: "); Serial.print(gpsTime);
  Serial.print(" | STATUS: ");   Serial.print(gpsStatus);
  Serial.print(" | SATS: ");     Serial.print(satCount);
  Serial.print(" | LAT: ");      Serial.print(latitude);
  Serial.print(" | LON: ");      Serial.print(longitude);
  Serial.print(" | ALT: ");      Serial.print(altitude); Serial.print("m ");
  
  // 磁力計區塊 (Gauss 單位，取小數點後 4 位)
  Serial.print("| MAG_X: "); if(magX>=0) Serial.print(" "); Serial.print(magX, 4);
  Serial.print(" | MAG_Y: "); if(magY>=0) Serial.print(" "); Serial.print(magY, 4);
  Serial.print(" | MAG_Z: "); if(magZ>=0) Serial.print(" "); Serial.print(magZ, 4);
  
  Serial.println();
}

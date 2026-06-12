# 🚀 RocketCup 航電主控系統軟體架構與代碼結構規範書
## (Avionics Firmware Architecture & API Specification)

---

## 1. 系統時脈與感測器採樣率配置 (Sensor Sampling Rates)

為維持高可靠性之飛行狀態估算，系統實施了多匯流排隔離與非對稱高頻採樣。高頻 IMU 數據（BMI088 與 ADXL375）完全透過定時中斷與 Ping-Pong 雙緩衝進行背景資料收集，徹底免除飛控任務的等待時間。

```
+------------------+------------------+-------------------+--------------------------------------------+
| 感測器 (Sensor)  | 通訊匯流排 (Bus)  | 採樣頻率 (Rate)   | 中斷與資料讀取機制 (Trigger & Buffer)      |
+------------------+------------------+-------------------+--------------------------------------------+
| **ADXL375**      | SPI1 (Shared)    | **3200 Hz**       | TIM3 定時中斷觸發 SPI 讀取，推入 3.2kHz 雙緩衝區  |
| **BMI088 Accel** | SPI2 (Dedicated) | **1600 Hz**       | TIM6 定時中斷觸發 SPI 讀取，推入 1.6kHz 雙緩衝區  |
| **BMI088 Gyro**  | SPI2 (Dedicated) | **2000 Hz**       | TIM7 定時中斷觸發 SPI 讀取，推入 2.0kHz 雙緩衝區  |
| **BMP388**       | SPI1 (Shared)    | **200 Hz**        | TIM2 定時中斷觸發，具備 SPI1 總線屏蔽保護鎖  |
| **MMC5983MA**    | I2C1 (Shared)    | **100 Hz**        | PB7/PB8 I2C1 軟體定時任務背景查詢讀取       |
| **NEO-M9N GPS**  | UART6 (Dedicated)| **10 Hz**         | UART6 DMA 雙緩衝接收，空閒中斷 (IDLE) 背景解析|
+------------------+------------------+-------------------+--------------------------------------------+
```

### 1.1 SPI1 共享總線屏蔽鎖設計 (Bus Mutex Line Lock)
* **硬體背景**：ADXL375 (3.2kHz) 與 BMP388 (200Hz) 共享 SPI1。
* **衝突防範**：由於 ADXL375 之中斷頻率極高（每 312.5 微秒觸發一次），當 FlightControl_Task 正在讀取 BMP388（耗時約 250 微秒）或寫入 Flash 時，若 TIM3 中斷強行插入並發起 SPI1 交易，將直接導致 SPI 總線衝突與資料錯亂。
* **韌體實作**：在 FlightControl_Task 讀取 BMP388 暫存器期間，採用**硬體中斷暫時屏蔽線**：
  ```c
  void BMP388_Read_Safe(uint8_t reg_addr, uint8_t *data, uint16_t len) {
      __disable_irq(); // 暫時關閉全域中斷 (屏蔽 TIM3 插入)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // 拉低 Baro CS
      HAL_SPI_TransmitReceive(&hspi1, &reg_addr, data, len, 10);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // 拉高 Baro CS
      __enable_irq();  // 恢復全域中斷
  }
  ```

---

## 2. 卡爾曼濾波 (Kalman Filter) 融合與實時飛控時序

飛控核心任務 `FlightControl_Task` 以 **100 Hz (10ms 週期)** 穩定運行。在每一個 10ms 週期內，系統精確進行**「降採樣濾波」**與**「卡爾曼更新」**：

```
時序軸 (Time line - 10ms 週期)
|---------------------------------------------------------------------------> 10ms
TIM3 ISR (3.2kHz) [||||||||||||||||||||||||||||||||] -> 採樣 ADXL375 共 32 筆數據
TIM6 ISR (1.6kHz) [||||||||||||||||]                 -> 採樣 BMI_Acc 共 16 筆數據
TIM7 ISR (2.0kHz) [||||||||||||||||||||]             -> 採樣 BMI_Gyro 共 20 筆數據
TIM2 ISR (200Hz)  [|]                                -> 採樣 BMP388 共 2 筆數據

FlightControl_Task 喚醒 (10ms 邊界)
 ├── 1. 切換雙緩衝區 (Ping-Pong Switch)，保證中斷可繼續無鎖寫入下一緩衝區
 ├── 2. 對積累數據進行均值/積分處理，消除高頻隨機白噪聲
 ├── 3. 讀取最新 BMP388 高度與速度
 ├── 4. 二階卡爾曼狀態更新：
 │      狀態量 X = [高度 h, 垂直速度 v]^T，狀態方程融合 BMI088 加速度計積分
 ├── 5. FSM 有限狀態機轉移判定 (動態頂點預測與安全高度強制開傘判定)
 └── 6. 打包數據並推入 OS 佇列 (Non-blocking Queue Send)
```

---

## 3. SD 卡非同步「雙緩衝」儲存機制 (Non-blocking Double-Buffering)

為防範 SD 卡在寫入 Sector 邊界或進行內部垃圾回收 (GC) 時產生的隨機**毫秒級阻塞 (Latency Spikes, 最長可達 80-150ms)** 餓死飛控任務，本系統實施了 **FreeRTOS 佇列 + RAM 雙緩衝區** 的非同步寫入架構。

### 3.1 系統架構圖 (Architecture flow)
```
[FlightControl_Task] (100Hz)
       |
       | (打包成 96-Byte 二進制日誌包)
       v
  [xDataLoggingQueue] (FreeRTOS Queue - 長度 200，可快取 2.0 秒數據)
       |
       | (佇列讀取 - 背景非同步)
       v
[DataLogging_Task] (優先級: Normal)
 ├── 1. 從佇列中提取二進制日誌包，將其格式化為 CSV 字串
 ├── 2. 寫入 RAM 當前快取緩衝區 Buffer_A
 ├── 3. 判斷 Buffer_A 是否已滿 (4096 位元組)？
 └── 4. 若滿：
         ├── 切換目前寫入指標至 Buffer_B (無縫持續收集後續數據)
         └── 觸發寫入執行緒：調用 FatFs f_write(Buffer_A) 批量寫入 SD 卡
         └── 調用 f_sync() 強制硬碟同步，保證斷電時已寫入資料 100% 安全
```

---

## 4. SD 卡儲存數據完整格式規範 (SD Card CSV Format Specification)

### 4.1 檔案命名與生命週期規則
* **命名規則**：系統初始化時，掛載 SD 卡並從 `FLIGHT_00.CSV` 開始進行遞增檢測，直到找到第一個不存在的序號，建立全新的日誌檔案，避免覆蓋歷史飛行資料。
* **文件安全性**：當狀態轉移至 `STATE_RECOVERY`（落地成功）時，`DataLogging_Task` 將執行 `f_close()` 並釋放檔案控制權，蜂鳴器長鳴提示「數據寫入安全，可拔除 SD 卡」。

### 4.2 CSV 檔案首部 Metadata 定義 (File Header)
CSV 日誌檔案前 10 行以 `#` 起始，以便於 Python (Pandas) 或 MATLAB 數據分析軟體自動跳過並直接讀取系統運行參數：

```csv
# RocketCup Avionics Flight Data Log
# Flight ID: FLIGHT_03.CSV
# OS Start Time: 2026-05-27 14:10:00
# System Config: magic=0x53595343, drogue_delay=4.00s, force_drogue=300.00m, main_target=150.00m, main_delay=3.50s
# Takeoff Config: takeoff_accel=3.00g, takeoff_height=10.00m, watchdog=2.00s
# Calib Offsets: AccelOffset_Z=0.012g, GyroOffset_X=0.045dps, GyroOffset_Y=-0.012dps, GyroOffset_Z=0.008dps
# Columns: Timestamp_ms,FSM_State,Kalman_H,Kalman_V,Baro_H,Baro_P,Acc_X_G,Acc_Y_G,Acc_Z_G,Gyro_X_dps,Gyro_Y_dps,Gyro_Z_dps,HG_Acc_X_G,HG_Acc_Y_G,HG_Acc_Z_G,Mag_X_mG,Mag_Y_mG,Mag_Z_mG,GPS_Lat,GPS_Lon,GPS_Alt,GPS_Sats,GPS_Fix,V_Bat,FIRE_Ch,Servo_Ang,CRC16
```

### 4.3 CSV 資料欄位對齊與精確格式 (Columns & Precision)

系統在寫入 SD 卡前，將數據轉化為工整的 ASCII CSV 格式，資料列無任何多餘空白，欄位精確定義如下：

| 欄位序號 | 欄位名稱 (Column Name) | 資料類型 (Type) | 格式範例 (Format) | 物理單位 (Unit) | 職責與資料來源描述 |
| :---: | :--- | :---: | :--- | :---: | :--- |
| **1** | `Timestamp_ms` | `uint32_t` | `102450` | ms | 自系統上電以來的累計毫秒數 |
| **2** | `FSM_State` | `uint8_t` | `4` | - | 有限狀態機當前狀態代碼 (1 ~ 10) |
| **3** | `Kalman_H` | `float` | `245.32` | m | 卡爾曼濾波平滑輸出之實時海拔高度 |
| **4** | `Kalman_V` | `float` | `45.12` | m/s | 卡爾曼濾波平滑輸出之垂直爬升/下降速度 |
| **5** | `Baro_H` | `float` | `244.89` | m | BMP388 氣壓計原始海拔高度 (200Hz 採樣) |
| **6** | `Baro_P` | `float` | `984.12` | hPa | BMP388 氣壓計原始氣壓讀值 |
| **7** | `Acc_X_G` | `float` | `-0.012` | g | BMI088 低G加速度計 X 軸原始值 (±24g 量程) |
| **8** | `Acc_Y_G` | `float` | `0.024` | g | BMI088 低G加速度計 Y 軸原始值 |
| **9** | `Acc_Z_G` | `float` | `1.004` | g | BMI088 低G加速度計 Z 軸原始值 |
| **10**| `Gyro_X_dps` | `float` | `0.12` | dps | BMI088 陀螺儀 X 軸角速度 (±2000dps 量程) |
| **11**| `Gyro_Y_dps` | `float` | `-0.45` | dps | BMI088 陀螺儀 Y 軸角速度 |
| **12**| `Gyro_Z_dps` | `float` | `0.08` | dps | BMI088 陀螺儀 Z 軸角速度 |
| **13**| `HG_Acc_X_G` | `float` | `0.45` | g | ADXL375 高G加速度計 X 軸原始值 (±200g 量程) |
| **14**| `HG_Acc_Y_G` | `float` | `-1.23` | g | ADXL375 高G加速度計 Y 軸原始值 |
| **15**| `HG_Acc_Z_G` | `float` | `9.81` | g | ADXL375 高G加速度計 Z 軸原始值 |
| **16**| `Mag_X_mG` | `float` | `245.2` | mG | MMC5983MA 地磁計 X 軸磁感應強度 (±8 Gauss) |
| **17**| `Mag_Y_mG` | `float` | `-12.5` | mG | MMC5983MA 地磁計 Y 軸磁感應強度 |
| **18**| `Mag_Z_mG` | `float` | `412.0` | mG | MMC5983MA 地磁計 Z 軸磁感應強度 |
| **19**| `GPS_Lat` | `double` | `25.0214567` | deg | NEO-M9N GPS 緯度數據 (高精度雙精度浮點) |
| **20**| `GPS_Lon` | `double` | `121.5428901`| deg | NEO-M9N GPS 經度數據 (高精度雙精度浮點) |
| **21**| `GPS_Alt` | `float` | `254.3` | m | NEO-M9N GPS 高度數據 |
| **22**| `GPS_Sats` | `uint8_t` | `14` | - | GPS 定位可見衛星顆數 |
| **23**| `GPS_Fix` | `uint8_t` | `3` | - | GPS 定位類型 (0:無定位, 2:2D定位, 3:3D定位) |
| **24**| `V_Bat` | `float` | `11.85` | V | 板載 ADC1_IN10 採樣分壓後之實時電池電壓 |
| **25**| `FIRE_Ch` | `uint8_t` | `0` | - | 引傘點火通道狀態 (0: 未導通, 1: PD13 MOSFET 導通) |
| **26**| `Servo_Ang` | `uint16_t` | `90` | deg | 主傘釋放舵機當前控制角度值 (0 ~ 180 度) |
| **27**| `CRC16` | `uint16_t` | `45231` | - | 該行字串之 CRC16 校驗值，確保日誌離線讀取完整性 |

---

## 5. 記憶體開銷與堆疊安全防護

為防止 FreeRTOS 堆疊溢出 (Stack Overflow)，對 5 大任務進行了嚴格的靜態記憶體配置規劃：

* **FlightControl_Task (512 Words / 2048 Bytes)**:
  * *堆疊監控*：僅使用輕量級局部變數。卡爾曼濾波器與狀態暫存器皆為 `static` 全域分配，防止因深層計算導致堆疊崩潰。
* **DataLogging_Task (512 Words / 2048 Bytes + 8KB Heap)**:
  * *記憶體優化*：`Buffer_A` 與 `Buffer_B` (共計 8KB) 必須使用 `static uint8_t` 進行全域編譯期靜態分配，絕不能在任務執行中動態調用 `malloc()`，保障堆疊線路絕對安全。
  * *溢出防範*：開啟 FreeRTOS 堆疊溢出檢測鉤子函數 `vApplicationStackOverflowHook`，一旦發生溢出，立即強行拉低 `PD13` (FIRE) 並拉響蜂鳴器，以防硬體誤點火。

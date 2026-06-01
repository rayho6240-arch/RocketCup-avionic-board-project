# 設計筆記：A — ADXL375 融入 EKF 高 G 相位 ／ B — SPI1 並發保護

> **用途**：供「可上板 / HIL 測試」的 session 實作改善計畫的 🔴 高優先項目 A 與 B。
> **狀態**：**設計與程式碼草圖，本文件不含任何飛行碼變更**。所有 code 片段皆為 sketch，需於上板環境逐步驗證後才併入。
> **關聯**：改善計畫 `context-rosy-kahan.md` 項目 A、B、M；本次（2026-06-01）已完成 D/I/N/E/H。

---

## 0. 摘要與建議執行順序

A 與 B 互相牽連：A（融合 ADXL375）需要穩定、單位正確的 ADXL 資料；B（SPI1 並發）決定 ADXL 如何被讀取。建議順序：

1. **§2 前置阻斷項先解決** — 釐清並修正 EKF 加速度單位（g vs m/s²）。**這是 A 正確性的前提**，否則融合 ADXL 只是把另一個錯誤尺度的來源接進去。
2. **B 先於 A** — 先把 SPI1 存取整理乾淨（ADXL 讀取路徑穩定、BMP388 不被競爭），再動 EKF 融合。
3. **A 最後** — 在單位與資料路徑都確定後，加入感測器切換。每步都要對照既有 HIL CSV / Flash ring 資料驗證。

---

## 1. 現況（程式碼定位）

| 項目 | 位置 | 說明 |
|------|------|------|
| ADXL375 讀取 | `main.c:1900-1930`（`HAL_TIM_PeriodElapsedCallback`，TIM3 分支） | **在 ISR 內**直接阻塞讀 SPI1（~5.6 µs），填入 `g_adxl_batches[2][32]` ping-pong |
| ADXL375 轉換 | `adxl375.c:106-110` | `ax = x_raw * 0.049f` → **單位 g**（49 mg/LSB）。**未做軸向反向** |
| ADXL ping-pong 取用 | `main.c:1787-1799` | 主迴圈取最新一筆存入 `highg_data`（僅記錄，未進 EKF） |
| BMP388 讀取 | `main.c:1779-1787` | 主迴圈 200 Hz，讀取前 `__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE)`、讀完再 enable（暫時解，保護 SPI1） |
| BMI088 轉換 | `bmi088.c:162-164` | `ax = accel_x_raw * 24.0f / 32768.0f` → **單位 g**（±24 g 滿量程） |
| EKF 樣本組裝 | `main.c:1688-1721` | 內插出 1000 Hz，`p_sample->ax/ay/az`（BMI，g）、`gx/gy/gz`（rad/s） |
| EKF bias 校準 | `ekf.c:402-418` | 靜置 3000 筆平均：`accel_bias[0,1]=avg`、`accel_bias[2]=avg_az - GRAVITY` |
| EKF bias 修正 | `ekf.c:447-449` | `ax_corr = sample->ax - accel_bias[…]` |
| EKF 預測步 | `ekf.c:180-214` | `a_nav = R·a_corr`，z 軸再 `- GRAVITY(9.80665)`，積分為速度/位置 |

---

## 2. ⚠️ 前置阻斷項：EKF 加速度單位 (g vs m/s²)

**這是 A 的硬性前提，必須先在上板用 HIL 資料確認 / 修正。**

### 2.1 問題推導

- BMI088 樣本 `ax/ay/az` 單位是 **g**（`bmi088.c:162`，`×24/32768`）。
- 校準（`ekf.c:408`）：`accel_bias[2] = avg_az − GRAVITY`，GRAVITY = 9.80665。靜置時 `avg_az ≈ 1.0`（g），故 `bias_z ≈ 1.0 − 9.80665 = −8.80665`。
- 修正（`ekf.c:449`）：`az_corr = az − bias_z = az + 8.80665`。靜置時 `az_corr ≈ 1.0 + 8.80665 = 9.80665`。
- 預測（`ekf.c:202`）：`a_nav_z = (R·a_corr)_z − GRAVITY ≈ 9.80665 − 9.80665 = 0`。**靜置時完美抵銷** → ZUPT / 校準都正常，bench 靜態測試看不出問題。

但動態時：設真實 5 g 上升（z 軸朝上）：
- `az = 5.0`（g），`az_corr = 5.0 + 8.80665 = 13.80665`，`a_nav_z = 13.80665 − 9.80665 = 4.0`。
- 物理上淨加速度應為 `(5−1) g = 4 g = 39.2 m/s²`，但 EKF 得到 **4.0**。
- 一般式：`az_corr = GRAVITY + (az − avg_az)`，其中 `(az − avg_az)` 是**以 g 為單位**的動態偏差，卻被當成 m/s² 積分。**動態加速度被低估約 9.81×**（x/y 軸同理，`a_corr = Δa_g` 直接當 m/s²）。

> **結論**：靜態恆對、動態尺度錯約 9.81×。目前因 BMP388 量測更新主導高度、加速度只作短期預測，故 bench 上「看起來能動」，但速度估計（`ekf_vel_cms`）與動力段高度預測必然偏低。

### 2.2 為什麼這擋住 A

ADXL375 也是 **g**（`adxl375.c:108`）。若沿用「不縮放、用 bias 把靜態湊成 9.80665」的做法，切到 ADXL 時必須重現同一套 trick（ADXL 自己的 `avg_az` + GRAVITY），但這只能讓**靜態**對齊；高 G 動態段（正是要 ADXL 的場景）尺度仍錯 9.81×，融合等於無效。**所以單位問題必須先解。**

### 2.3 建議修法（上板驗證後再併入）

**方案 X（推薦）— 全程 SI 單位 m/s²：**

- 在 driver 或樣本組裝邊界，把 BMI088、ADXL375 的 `ax/ay/az` 都 `×9.80665f` 轉成 m/s²。
- 校準改為：`accel_bias[i] = avg_i`（純偏差，三軸一致），**不再**對 z 軸特別 `− GRAVITY`。靜置時 `avg_az ≈ 9.80665`（m/s²），`az_corr = az − avg_az ≈ 0`，預測 `a_nav_z = (R·a)_z − GRAVITY`；靜置 `(R·a)_z ≈ 9.80665`（因為 bias 只移除了「偏差」而非重力？）

  ⚠️ 注意：若 bias 改成純 `avg`（含重力），則 `az_corr` 把重力也減掉了，預測再 `− GRAVITY` 會變成 `−9.80665`。**重力要嘛在 bias 抵銷、要嘛在預測抵銷，不能兩次。** 兩種自洽寫法：
  - **(a) bias 只含感測器零偏，預測減重力**：校準需把重力從 `avg_az` 分離（如 `bias_z = avg_az − 9.80665`，但此式只有在 `avg` 為 m/s² 時才對）。等價於現況但單位改 m/s²。
  - **(b) bias 含重力（`bias=avg` 三軸），預測不減重力**：`a_nav = R·a_corr`，靜置 `a_corr≈0` → `a_nav≈0`。較簡潔，但要求載具升空前姿態與校準時一致（重力方向被「凍結」進 bias），翻滾後不再正確 → **不適合會翻滾的火箭**。

  → 採 **(a)**：全程 m/s²，校準 `bias_z = avg_az − GRAVITY`、`bias_xy = avg_xy`，預測維持 `− GRAVITY`。與現況唯一差別是 `ax/ay/az` 先 `×9.80665`，使動態尺度正確。

- **連帶影響**：`Q_pos/Q_vel/R_baro` 與起飛偵測門檻（`ekf.c:469` 的 `>5.0f`、落地 `ekf.c:515` 的 `0.8f`）的物理意義不變（那些是高度/重力單位，不受 accel 縮放影響），但 predict 的雜訊行為改變，需用 HIL 資料重新確認濾波是否仍穩定。

> **驗證**：拿一段既有 HIL CSV（含 `bmi_ax..az`），離線用 Python 跑修正前後的 EKF，比較動力段速度/高度是否更貼近 BMP388 積分與已知彈道。修正後再進 A。

---

## 3. Item A：ADXL375 融入 EKF 預測步

### 3.1 目標

BMI088 ±24 g 在動力段（高功率馬達可達 30–100 g）會**飽和**，飽和後 EKF 預測輸入被截斷 → 動力段速度/頂點高度低估。ADXL375 ±200 g 不飽和，應在高 G 段接手提供預測加速度。

### 3.2 切換邏輯（門檻 + 遲滯）

以 BMI088 加速度模長 `|a_bmi|` 為切換依據（單位統一為 m/s² 後）：

```
門檻（留飽和裕度，24 g ≈ 235.4 m/s²）：
  進入 ADXL：|a_bmi| ≥ 20 g (196.1 m/s²)
  回到 BMI ：|a_bmi| ≤ 18 g (176.5 m/s²)   ← 2 g 遲滯，避免門檻附近抖動
```

- BMI088 雜訊低、已校準，平時優先用。
- 只有逼近飽和才切 ADXL（ADXL 49 mg/LSB，雜訊大約是 BMI 的數十倍）。
- **硬切的風險**：兩感測器零偏/尺度不一致 → 切換瞬間注入加速度階躍 → 速度估計被踢一下。緩解：(1) ADXL 也做靜態 bias 校準（§3.3）；(2) 進階可在 18–22 g 帶內做線性混合 `a = (1−w)·a_bmi + w·a_adxl`，避免階躍（第一版可先硬切 + 遲滯，HIL 驗證階躍幅度後再決定是否加混合）。

### 3.3 ADXL375 靜態 bias 校準

在現有 3 秒靜置校準（`ekf.c:387-420`）期間，**同時**累積 ADXL 讀數求零偏：

- 新增 `EKF_adxl_sum[3]`、`EKF_adxl_bias[3]`（放 CCMRAM，與其他 bias 一致）。
- z 軸同樣 `adxl_bias[2] = avg_adxl_z − GRAVITY`（單位 m/s²），x/y 取平均。
- ADXL 噪聲大，3000 筆平均後零偏仍可用；必要時延長校準或加大平均窗。

### 3.4 單位與軸向對齊（關鍵）

切到 ADXL 時，餵進 EKF 的加速度必須與 BMI 路徑**完全同管線**：

1. **單位**：ADXL `ax = x_raw·0.049`（g）→ `×9.80665` 轉 m/s²（與 §2.3 BMI 一致）。
2. **軸向**：BMI088 PCB **X/Y 反向**已在資料路徑處理（計畫項目 M 指出 ADXL 反向「在 EKF 但因未融合而無效」）。融合 ADXL 前，**ADXL 必須套用與 BMI 相同的軸向變換**（建議在 `adxl375.c:106-110` driver 層直接做反向，與 BMI 對齊，順手了結項目 M）。
3. **bias**：用 §3.3 的 `EKF_adxl_bias`，**不可**用 `EKF_accel_bias`（那是 BMI 的）。

### 3.5 資料路徑：在樣本組裝點做切換

最乾淨的注入點是 EKF 樣本組裝（`main.c:1688-1721`）。在組好 BMI 的 `ax/ay/az` 後，判斷是否切 ADXL，並標記來源：

- 在 `EKF_Sample_t`（`ekf.h`）新增 `uint8_t accel_source;`（0=BMI，1=ADXL）。
- EKF 消費端依 `accel_source` 選用對應 bias（`ekf.c:447-449`）。

> 註：ADXL 在 TIM3 ISR @3.2 kHz、EKF 樣本 1000 Hz。組裝時取「當前最新」ADXL 值即可（不需逐筆對齊）；§4 整理 SPI1 後，最新 ADXL 值來源更穩定。

### 3.6 程式碼草圖（sketch，未驗證）

`ekf.h`：
```c
typedef struct {
    float ax, ay, az;      // m/s²（統一單位後）
    float gx, gy, gz;      // rad/s
    uint32_t timestamp_us;
    uint8_t  has_baro;
    float    baro_alt;
    uint8_t  accel_source; // 0 = BMI088, 1 = ADXL375   ← 新增
} EKF_Sample_t;
```

`main.c` 樣本組裝（接在 `main.c:1701` 算出 BMI `ax/ay/az` 之後）：
```c
/* --- Item A：高 G 飽和時切 ADXL375（單位已統一為 m/s²）--- */
float a_bmi_mag = sqrtf(ax*ax + ay*ay + az*az);
static uint8_t use_adxl = 0;                       // 遲滯狀態（單 producer，main task 內，安全）
if (!use_adxl && a_bmi_mag >= 196.1f) use_adxl = 1;       // ≥20 g 進
else if (use_adxl && a_bmi_mag <= 176.5f) use_adxl = 0;   // ≤18 g 回

if (use_adxl && adxl375_ok) {
    /* highg_data 已是最新 ADXL（main.c:1792-1797），單位 g；軸向需已於 driver 對齊 BMI */
    ax = highg_data.ax * 9.80665f;
    ay = highg_data.ay * 9.80665f;
    az = highg_data.az * 9.80665f;
    p_sample->accel_source = 1;
} else {
    p_sample->accel_source = 0;
}
p_sample->ax = ax; p_sample->ay = ay; p_sample->az = az;
```

`ekf.c` 消費端 bias 選擇（改 `ekf.c:447-449`）：
```c
const float *abias = (sample->accel_source == 1) ? EKF_adxl_bias : EKF_accel_bias;
float ax_corr = sample->ax - abias[0];
float ay_corr = sample->ay - abias[1];
float az_corr = sample->az - abias[2];
```

校準（`ekf.c` 靜置迴圈內，與現有 BMI 累積並列）：
```c
EKF_adxl_sum[0] += highg_g_x_in_mps2; /* 同步取一筆 ADXL（注意 SPI1 競爭，見 §4）*/
/* ... 3000 筆後：*/
EKF_adxl_bias[0] = EKF_adxl_sum[0] / 3000.0f;
EKF_adxl_bias[1] = EKF_adxl_sum[1] / 3000.0f;
EKF_adxl_bias[2] = EKF_adxl_sum[2] / 3000.0f - GRAVITY;
```

---

## 4. Item B：SPI1 並發保護

### 4.1 為什麼「SPI1 mutex」不能直接照計畫做

計畫原文是「建立 SPI1 FreeRTOS mutex」。**但 ADXL375 目前在 TIM3 ISR 內讀 SPI1（`main.c:1906`），而 FreeRTOS 互斥鎖（mutex）不能在 ISR 中取用**（`xSemaphoreTake` 不可從 ISR 呼叫；mutex 還涉及優先權繼承，更不可能）。所以不能把 ISR 那側包進 mutex。

現況的保護其實是「BMP388 讀取前 `__HAL_TIM_DISABLE_IT(TIM3)`」（`main.c:1781-1784`），本質是針對「3.2 kHz ISR ↔ 200 Hz task」這一對的**臨界區**，不是 mutex。要嘛**把 ADXL 移出 ISR**才能用 mutex，要嘛**把現有臨界區做法正規化**。

### 4.2 三種方案比較

| 方案 | 作法 | 優點 | 缺點 |
|------|------|------|------|
| **B1 正規化臨界區** | 維持 ADXL 在 ISR；把「disable TIM3 IT → 存取 SPI1 → enable」包成 `SPI1_Lock()/Unlock()` helper，所有 task-context 的 SPI1 使用者（BMP388、未來者）都呼叫 | 改動最小、不增 CPU 負擔、立即可上 | 仍非真 mutex；ADXL 讀取期間若 BMP388 正持鎖，ISR 不會觸發（漏採幾筆，可接受）；擴充性有限 |
| **B2 ADXL 移出 ISR → notify task** | TIM3 ISR 只 `vTaskNotifyGiveFromISR`；專責 task `ulTaskNotifyTake` 後取 SPI1 mutex 讀 ADXL | 真 mutex、可組合、ISR 極短 | **3.2 kHz → 每秒 3200 次 context switch**，CPU 開銷大，須實測；task 喚醒抖動可能影響 3.2 kHz 等間隔 |
| **B3 DMA 驅動 ADXL** | TIM3 觸發 SPI1 DMA 讀 ADXL（無 CPU）；BMP388 存取以 mutex + 檢查 DMA idle 協調 | 最省 CPU、最「正確」的嵌入式解 | 工作量最大；需重寫 ADXL 讀取為 DMA、處理半/全傳輸中斷與 CS 時序 |

### 4.3 建議：先 B1，長期評估 B3

- **本階段上板先做 B1**（風險最低、立即改善可維護性）：把現有 ad-hoc 的 disable/enable 收斂成單一 helper，避免未來新增 SPI1 寫入路徑時遺漏保護。
- A 的 ADXL 校準取樣（§3.6）也透過同一 helper 存取，避免與 ISR 競爭。
- **B2 不建議**用於 3.2 kHz（context switch 成本）；除非把 ADXL 降頻或改批次 DMA。
- **B3 列為長期**：當 SPI1 上再增設備、或要徹底卸載 CPU 時再做。

### 4.4 B1 程式碼草圖（sketch）

```c
/* main.c 或 spi_bus.c：SPI1 臨界區 helper。
 * 注意：僅供 task context 呼叫（會關 TIM3 IT）。ISR 端的 ADXL 讀取本身就是
 * 被這段臨界區排除的對象，不需自鎖。 */
static inline void SPI1_Lock(void)   { __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE); }
static inline void SPI1_Unlock(void) { __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE); }
```

BMP388 讀取（取代 `main.c:1781-1784`）：
```c
SPI1_Lock();
BMP388_ReadData(&hspi1, &baro_data);
SPI1_Unlock();
```

開機 T0 鎖定（Item H，`main.c` BMP388 自檢迴圈）與 A 的 ADXL 校準取樣同樣改用 `SPI1_Lock/Unlock()`，全專案 SPI1 task 端存取單一入口。

> 進一步可加一個 `volatile uint8_t spi1_owner` 旗標 + `configASSERT`，在 debug build 偵測巢狀/遺漏配對。

---

## 5. 上板驗證清單

| 項目 | 方法 | 通過標準 |
|------|------|----------|
| §2 單位修正 | 離線用既有 HIL CSV 重跑 EKF（Python `ekf_visualizer.py`） | 動力段速度/高度貼近 BMP388 積分與彈道，靜置仍 0±0.3 m |
| ADXL bias 校準 | 靜置開機，看 `[EKF]` 印出的 `adxl_bias` | x/y≈0、z≈0（已減 g）；數值穩定、無發散 |
| 軸向對齊 | 手動沿各軸傾斜，比較 BMI 與 ADXL 同號同軸 | 三軸方向一致 |
| 切換門檻/遲滯 | HIL 餵入掃過 18–22 g 的加速度序列 | 在 20 g 進、18 g 回，無高頻抖動切換 |
| 切換階躍 | 記錄 `accel_source` 與 `ekf_vel_cms`，看切換瞬間 | 速度估計無明顯跳階（>~?）；若大則加 §3.2 混合 |
| B1 SPI1 | 長時間跑，確認 BMP388 200 Hz 與 ADXL 3.2 kHz 採樣率（`g_sampling_rate`）穩定 | BMP≈200 Hz、ADXL≈3200 Hz，無掉速 |
| 整合迴歸 | 全程 HIL，FSM 各狀態轉換、頂點偵測 | 與融合前相比頂點/速度更準，FSM 不誤觸發 |

---

## 6. 風險

- **§2 單位修正**會改變 EKF 動態行為，是本計畫最高風險的一步；**務必先離線用 HIL 資料驗證**，不可盲改上飛。
- ADXL 雜訊大，融合後若門檻設太低（常切 ADXL）反而劣化靜飛段；門檻務必留在接近 BMI 飽和處。
- B1 的 disable IT 期間 ADXL 會漏採數筆（µs 級），對 3.2 kHz 記錄影響可忽略，但若未來 ADXL 也要進 EKF 高頻路徑需再評估。
- 切換瞬間階躍：硬切在兩感測器尺度/零偏不一致時會踢速度估計一下；HIL 量到的階躍幅度決定是否需要 §3.2 的線性混合。
```

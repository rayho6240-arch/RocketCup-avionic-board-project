# 🚀 RocketCup Avionic Board Project (成大 ISP 航電組)

歡迎來到成大 ISP 航電組的協作平台！這裡存放了我們為比賽設計的所有電路圖、PCB 檔案以及相關硬體參考資料。

為了方便大家協作、快速找到規格書，我們對專案資料夾進行了全面整合與邏輯分類。請大家開發前務必詳細閱讀以下指南。

---

## 📂 檔案結構說明 (去哪裡找你的檔案與資料？)

我們採用結構化、模組化的資料夾配置。請在您負責的子專案資料夾內開發，並參考對應的規格書：

### 1️⃣ 核心開發專案 (由 Git 進行版本控制)
* **[BigAvionic/](file:///Users/laizhiquan/coding/RocketCom/BigAvionic)**: 大火箭航電主板專區 (內附 `備注.md` 進度檔)。
* **[Deployment board/](file:///Users/laizhiquan/coding/RocketCom/Deployment%20board)**: 降落與開傘部署板專區 (內附 `備注.md` 進度檔)。
* **[GroundStation/](file:///Users/laizhiquan/coding/RocketCom/GroundStation)**: 地面接收站專區 (內附 `備注.md` 進度檔)。
* **[SmallAvionic/](file:///Users/laizhiquan/coding/RocketCom/SmallAvionic)**: 小火箭航電備份板專區 (包含 Joyce、Ray、Tank、Tony 的開發分支，內附 `備注.md` 進度檔)。
* **[Lib/](file:///Users/laizhiquan/coding/RocketCom/Lib)**: 共用的核心 KiCad 零件庫 (Symbol & Footprint)。

> 📌 **開發小筆記**：每個開發資料夾下都有一個 **`備注.md`**。請在裡面記錄您的設計要點、遇到的 Bug，並隨著開發勾選進度 Checkbox，讓其他組員知道目前的開發狀況！

### 2️⃣ 整理後的硬體參考資料 (建議在本地參考，不頻繁變動)
* **[Datasheets/](file:///Users/laizhiquan/coding/RocketCom/Datasheets)**: 收納了 13 個重命名後的 PDF 規格書與硬體指南，標註了晶片型號與硬體用途（如：`ADXL375_High_G_Accelerometer_Datasheet.pdf`），一目了然！
* **[Libraries/](file:///Users/laizhiquan/coding/RocketCom/Libraries)**: 收納了參考用的 KiCad 零件庫壓縮檔與解壓資料夾（如 `ul_*` 庫）。
* **[Documents/](file:///Users/laizhiquan/coding/RocketCom/Documents)**: 存放 PDR 簡報、競賽評分表以及系統繪圖檔。
* **[ReferenceSchematics/](file:///Users/laizhiquan/coding/RocketCom/ReferenceSchematics)**: 存放獨立的 `Telemetry_KiCAD_Project.kicad_sch` 參考電路圖。

---

## 文件導覽 (韌體 / 教學 / 歷史)

韌體、教學與專案歷史文件已整理歸檔如下：

* **[教學/教學_6_24.md](教學/教學_6_24.md)**：最新完整版開發教學（三角色架構、板間鏈路與不對稱備援、雙鏈路遙測協定、地面站、主機端測試）。
* **[PROJECT_HISTORY.md](PROJECT_HISTORY.md)**：專案演進史，記錄重大決策與轉折的「為什麼」。
* **[docs/](docs/README.md)**：工程／規劃文件索引（硬體規格表、改進計劃、進度、設計筆記、需求驗證、測試方法）。

> **韌體建置請以頂層 [`Makefile`](Makefile) 為準**：`make`（主航電）、`make build-backup`（備援）、`make build-ground`（地面站）、`make host-test`（純邏輯測試）。
> 工具鏈**必須使用 STM32CubeIDE 內建的 arm-none-eabi gcc**（Makefile 會自動偵測），**請勿使用 Homebrew 版**。
> 注意：航電專案目錄已由 `Main_AV_F407` 更名為 `Main_Code`；本文件下方早期撰寫的「VS Code / Homebrew」段落僅供歷史參考，現行建置一律以頂層 `Makefile` 與 `教學/教學_6_24.md` 第 11 節為準。

---

## ⚡ 每日開發流程 (SOP)

為了避免「悲劇」（檔案衝突或覆蓋），每次開工前請牢記口訣：**「先拉，再做，後推」**。

### 1️⃣ 開工第一件事：拉取 (Pull)
**只要你剛坐下準備開始畫圖，請先在 `RocketCom` 資料夾下執行：**
```bash
git pull origin main
```
* **為什麼？** 這能把雲端最新的進度同步到你的電腦。如果不拉就改，上傳時會發生嚴重冲突。

### 2️⃣ 進行開發
* 打開 KiCad，編輯您負責的原理圖或 PCB。
* 記得隨時存檔 (Ctrl+S)。
* **請更新您子專案目錄下的 `備注.md` 以更新開發進度！**

### 3️⃣ 收工上傳三部曲：加、存、推
當您完成一個階段的工作（例如：畫完電源線、擺完去耦電容），請依序執行：

**Step A: 加入變更 (Add)**
```bash
git add .
```

**Step B: 提交紀錄 (Commit)**
寫下您到底改了什麼。！！！絕對不要有啥都沒有描述的空 commit！！！
```bash
git commit -m "清楚的描述，例如：完成主航電板的 3.3V 電源佈線並更新備註"
```

**Step C: 推送到雲端 (Push)**
這一步才會把檔案真正上傳到 GitHub。
```bash
git push origin main
```

---

## 💣 KiCad 協作特別注意事項 (必讀！)

### 1. 關於 PCB 檔案 (`.kicad_pcb`)
Git 很擅長合併程式碼，但**非常不擅長合併二進位/圖形化的 PCB 檔**。
* ⚠️ **黃金法則**：同一時間，**只能有一個人**修改同一張 PCB 板。
* **開發前溝通**：要改 Layout 之前，請在群組喊一聲，確認目前沒人正在編輯再動手。

### 2. 優化後的 `.gitignore` 說明
我們已經全面升級了 `.gitignore`，能自動遞迴忽略以下不必要的檔案：
* KiCad 自動產生的原理圖與 PCB 備份檔 (`*.kicad_sch-bak`, `*.kicad_pcb-bak`)。
* 個人視窗配置檔 (`*.kicad_prl`，避免每個人螢幕大小不同導致衝突)。
* 作業系統垃圾檔（如 macOS 的 `.DS_Store` 與 Windows 的 `Thumbs.db`）。
* 開發快取檔（如 `fp-info-cache`）。
* *請勿強行上傳這些垃圾檔，以保持專案倉庫的輕量與整潔！*

---

## ❓ 常見問題救生圈

### Q1: 我輸入 `git push` 時出現 `error: failed to push some refs...` 怎麼辦？
代表有人在您工作的期間，已經先上傳了新版本。
**解法：**
1. 先輸入 `git pull origin main` (把對方的東西抓下來自動合併)。
2. 如果不幸出現衝突 (Conflict)，請找組長求救。
3. 合併成功後，再輸入一次 `git push origin main`。

### Q2: 為什麼有些備份檔 (`.bak`, `-bak`) 傳不上去？
這是正常的！我們在 `.gitignore` 中設定了過濾，這些是不需要的暫存備份檔。

## 🛠️ 輕量化 STM32 開發與編譯指南 (VS Code / Antigravity IDE)

日常編碼、編譯與燒錄**不需要每次打開重型的 STM32CubeIDE**：在 VS Code / Antigravity IDE 的終端機，直接呼叫**倉庫根目錄的頂層 [`Makefile`](Makefile)**，即可完成主控專案 [Main_Code](file:///Users/laizhiquan/coding/RocketCom/Main_Code/) 的極速編譯與燒錄。

### 1️⃣ 工具鏈 (Toolchain)：用 STM32CubeIDE 內建的，**不要**用 Homebrew
本專案**必須使用 STM32CubeIDE 內建的 `arm-none-eabi` gcc 與 `STM32_Programmer_CLI`**，**請勿**用 `brew install` 安裝任何 ARM GCC（例如 `gcc-arm-none-eabi`）——版本不同會造成連結或燒錄問題。

* 你**只需要安裝 STM32CubeIDE**（之後不必每次開啟它）；頂層 `Makefile` 第 12–15 行會自動從
  `…/STM32CubeIDE.app/…/plugins/` 偵測 gcc 與 `STM32_Programmer_CLI` 的路徑，編譯／燒錄時自動帶入 `PATH`。
* 若出現「STM32CubeIDE 工具鏈未找到」錯誤，代表 CubeIDE 未安裝或路徑不符，請先安裝 STM32CubeIDE。

> 📌 此為現行唯一正確的工具鏈規範，詳見 [`教學/教學_6_24.md`](教學/教學_6_24.md) 第 11 節與頂層 [`Makefile`](Makefile)。

### 2️⃣ 在 IDE 終端機進行極速編譯 (頂層 Makefile)
**請在倉庫根目錄 `RocketCom/` 下執行**（不要再 `cd` 進子資料夾）。一份程式碼可建置三種角色：

```bash
make               # 編譯主航電 (ROLE_PRIMARY，預設目標)
make build-backup  # 編譯備援航電 (ROLE_BACKUP)
make build-ground  # 編譯地面站   (ROLE_GROUND)
make host-test     # 純邏輯主機端測試（免上板）
make clean         # 清除建置產物
```

編譯成功後，產物在 `Main_Code/Debug/` 下（皆已被 `.gitignore` 忽略，不佔倉庫）：
* **`Main_AV_F407.elf`** — 主航電的除錯／燒錄檔（即頂層 `Makefile` 的 `ELF` 變數）
* 連帶產生 `Main_AV_F407.bin`、`Main_AV_F407.list`、`Main_AV_F407.map`
* `Main_AV_F407_backup.elf` / `Main_AV_F407_ground.elf` — 備援／地面站變體（由 `build-backup` / `build-ground` 產生）

### 3️⃣ 一鍵燒錄與序列埠監看
```bash
make flash          # 編譯 + 以 ST-Link SWD 燒錄主航電 .elf
make flash-backup   # 編譯並燒錄備援航電
make flash-ground   # 編譯並燒錄地面站
make monitor        # 開序列埠監看（自動偵測序列埠 @460800；可用 PORT=/dev/cu.xxx 覆寫，Ctrl+A 後按 K 離開）
make all            # 等同 flash + monitor
```

若偏好在 IDE 內單步除錯，可安裝 **Cortex-Debug** 擴充套件，並在 `.vscode/launch.json` 中配置 ST-Link 除錯器，即可按 **F5** 一鍵燒錄與中斷點除錯。

*(註：若你未來在 `.ioc` 中新增或修改了硬體引腳，請打開 `Main_Code/Main_AV_F407.ioc` 點擊儲存重新 Generate；其餘日常編碼、編譯與燒錄皆可在本 IDE 終端機以上述 `make` 指令快速完成。)*

---

## 🔧 環境設定檢查

* **KiCad 版本**：請大家統一使用 **KiCad 9.0.7**，避免原理圖或 PCB 檔案格式相容性出錯。
* **Git 個人化設定**：第一次使用請設定您的名字和 Email：
```bash
git config --global user.name "你的名字(英文)"
git config --global user.email "你的Email"
```
彩蛋： 唯一不是AIIIIIIII寫的

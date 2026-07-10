# docs/ — 專案文件索引

本資料夾收納專案的工程／規劃文件（純 Markdown）。設計資產（簡報、繪圖、評分表等二進位檔）
仍放在 `../Documents/`；硬體 KiCad 專案在各 `BigAvionic/`、`SmallAvionic/` 等資料夾。

> 約定：本專案文件遵循 `AI_output_requirement.md`——不使用 emoji、不截斷重要輸出。

## 規劃與進度
- [進度.md](進度.md) — 逐次 session 開發流水帳（最詳細的技術變更紀錄）。
- [改進計劃.md](改進計劃.md) — P0–P3 飛行安全改進計劃、風險清單 R1–R8 與上飛前檢查清單。
- [待確認、改進清單.md](待確認、改進清單.md) — 待硬體確認項與優化願望清單。

## 硬體與需求規格
- [連線和基本硬體規格表.md](連線和基本硬體規格表.md) — 接腳／匯流排「黃金真相」（含 I2C 腳位互換警告），自原理圖萃取。
- [需求_驗證.md](需求_驗證.md) — 系統需求（REQ-SYS-*）與驗證矩陣。

## 設計與架構
- [code_structure.md](code_structure.md) — 韌體架構與 API 規格（任務頻率、雙緩衝、CSV 格式等）。
- [設計_ADXL375_EKF融合與SPI1並發.md](設計_ADXL375_EKF融合與SPI1並發.md) — ADXL375 融入 EKF 與 SPI1 並發保護的上板設計筆記（含 g/m/s² 單位前置阻斷項）。

## 測試
- [測試方法.md](測試方法.md) — 測試環境、高頻中斷採樣與 HIL 程序。

## 規範與緣由（Meta）
- [AI_output_requirement.md](AI_output_requirement.md) — AI 輸出標準（不截斷 log、不用 emoji、純文字航太風格）。
- [ORIGINAL_REQUEST.md](ORIGINAL_REQUEST.md) — 教學文件的原始需求說明。

---

## 相關文件（不在本資料夾）
- [../教學/教學_6_24.md](../教學/教學_6_24.md) — 最新完整版開發教學（架構／協定／備援／地面站）。
- [../教學/教學_6_19.md](../教學/教學_6_19.md) — 6/19 版教學（+ 互動 HTML：`教學_6_19.html`）。
- [../教學/教學_6_12.md](../教學/教學_6_12.md) — 早期教學版本（原根目錄 `教學.md`）。
- [../PROJECT_HISTORY.md](../PROJECT_HISTORY.md) — 專案演進史與重大決策的「為什麼」。
- [../README.md](../README.md) — 團隊協作 SOP 與資料夾總覽。

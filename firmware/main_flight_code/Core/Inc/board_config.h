/*
 * board_config.h — 主/備雙航電編譯期角色與功能旗標（單一程式碼、雙板）
 * ===========================================================================
 * 同一份 firmware 以 BOARD_ROLE 切出三個 binary：
 *   - ROLE_PRIMARY（主航電）：完整功能（GPS / 磁力計 / LoRa 433+920）+ 板間鏈路。
 *   - ROLE_BACKUP （備援航電）：只跑 MCU-layer 飛控（IMU/baro/highG → EKF → FSM
 *     → 點火），關閉 GPS / 磁力計 / LoRa；其餘飛控、Flash 記錄、板間鏈路照常。
 *   - ROLE_GROUND （地面站）：硬體與航電板相同，只「接收」LoRa 下行遙測（E22 433 +
 *     E80 920）+ 讀自身 GPS，對齊時間戳後記錄到 SD/Flash 並以 USB-CDC 串流給 PC；
 *     關閉所有飛控感測器（IMU/baro/highG/磁力計）、EKF/FSM/點火與板間鏈路。
 *
 * 板間實體層（已選定）：兩塊「相同」的 MCU 板各自跑 USART2 硬體全雙工（PA2=TX/
 * PA3=RX），於板間排線把 TX/RX 交叉一次（A.PA2→B.PA3、A.PA3→B.PA2）即可對接。
 * 兩板 UART 程式完全相同，無軟體 bit-bang（F407 USART 無硬體 SWAP 位元，硬體腳位
 * 不可對調，故以排線交叉取代「韌體互換腳位」，換取硬體 UART 的可靠度）。
 *
 * 角色設定方式（擇一）：
 *   - STM32CubeIDE：新增 build configuration「Debug_Backup」，preprocessor 加
 *     BOARD_ROLE=ROLE_BACKUP（其餘與 Debug 相同）。
 *   - 命令列：頂層 `make build-backup`（自動切換下方 BOARD_ROLE 行後建置）。
 *
 * 本檔不依賴 HAL / CMSIS / FreeRTOS，純常數巨集，可被純邏輯模組與 host 測試 include。
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* === 版本號碼 === */
#define FIRMWARE_VERSION "v1.0.2"

/* === 角色 === */
#define ROLE_PRIMARY 0
#define ROLE_BACKUP  1
#define ROLE_GROUND  2   /* 地面站接收器：硬體與航電板相同，只收 LoRa 下行 + 自身 GPS */

#ifndef BOARD_ROLE
#define BOARD_ROLE   ROLE_PRIMARY   /* MAKE_ROLE_LINE */
#endif

#define IS_PRIMARY   (BOARD_ROLE == ROLE_PRIMARY)
#define IS_BACKUP    (BOARD_ROLE == ROLE_BACKUP)
#define IS_GROUND    (BOARD_ROLE == ROLE_GROUND)

/* === 功能閘（main.c 以 #if FEATURE_* 包住對應驅動 init / task） ===
 * 備板關閉 GPS(UART6) / 磁力計(I2C1) / LoRa(E22 UART3 + E80 SPI3)。
 * 地面站：開 GPS + LoRa(改 RX) + USB-CDC；關 磁力計 + 整個飛控管線(FEATURE_FLIGHT)。
 * 注意：W25Q128 Flash 與 E80 共用 SPI3（見 w25qxx.c / spi3_bus.h），故 SPI3 周邊本身
 * 不關閉（備板/地面站仍需 Flash 記錄）；備板只是不啟用 E80 驅動並按住其 RST 釋放匯流排，
 * 地面站則啟用 E80 但走 RX（讀 FIFO 與 Flash 同以 SPI3 互斥鎖序列化）。 */
#define FEATURE_GPS    (IS_PRIMARY || IS_GROUND)  /* NEO-M9N GPS（UART6）：主航電 + 地面站 */
#define FEATURE_MAG    IS_PRIMARY                 /* MMC5983MA 磁力計（I2C1）：僅主航電 */
#define FEATURE_LORA   (IS_PRIMARY || IS_GROUND)  /* LoRa 硬體 init：E22 433(UART3) + E80 920(SPI3) */
#define FEATURE_LORA_TX IS_PRIMARY                /* 下行遙測「發送」：僅主航電 */
#define FEATURE_LORA_RX IS_GROUND                 /* 下行遙測「接收」：僅地面站 */
#define FEATURE_FLASH   1                         /* 電梯場測：開啟記錄，無法全程接筆電監看時事後回放用 */
#ifndef FEATURE_HOTSTART
#define FEATURE_HOTSTART 0                        /* 暫時關閉空中熱啟動恢復機制 */
#endif
#define FEATURE_BUZZER  1                         /* 蜂鳴器開關（1=開啟，0=靜音以防測試時噪音過大） */

/* === 飛行 profile：場測（電梯）與真實飛行的門檻/降級鏈切換 ===
 * 電梯場測（垂直電梯井道，無法完整重現彈道）沿用真實飛行同一份 FSM，但頂樓僅
 * 30m、全程等速 1g、不具備真實加速度剖面，因此需要一組縮放後的門檻（見
 * fsm.h 內以本旗標分組的 #if/#else 常數表）並強制走純氣壓降級鏈（電梯無法讓
 * EKF 觀察到有意義的水平/垂直運動特徵，硬套用 EKF 路徑只會誤判）。
 *   0 = 飛行 profile（預設，真實彈道門檻 + EKF 路徑正常參與）
 *   1 = 電梯測試 profile（門檻縮放 + 強制 baro 降級鏈） */
#ifndef FLIGHT_PROFILE_ELEVATOR
#define FLIGHT_PROFILE_ELEVATOR 1
#endif

/* FEATURE_FORCE_BARO_ONLY 由 FLIGHT_PROFILE_ELEVATOR 推導：電梯 profile 全程強制
 * 狀態機使用純氣壓降級鏈（全靠 Baro，忽略 EKF 的高度/速度判定）；飛行 profile
 * 則讓 EKF 三路徑正常參與（P0-C 健康降級照常以 EKF_GetHealthBits() 判斷）。 */
#define FEATURE_FORCE_BARO_ONLY FLIGHT_PROFILE_ELEVATOR

/* === 垂直通道 Kalman 濾波器（vertical_filter.h，Schultz 火箭高度計架構） ===
 *   FEATURE_VFILTER      = 1：編譯並執行濾波器，1Hz 與 EKF 對照列印（供 A/B 比較）。
 *   FEATURE_VFILTER_FSM  = 1：【已啟用】VF 為開傘主估計器——FSM 的 h_est/v_est 直接
 *                             取自本濾波器，兩 profile 皆同；EKF 僅供姿態/遙測/記錄。
 *                             改用獨立垂直通道的動機：EKF 垂直速度靜置/地面實測會漂移。 */
#define FEATURE_VFILTER      1
#define FEATURE_VFILTER_FSM  1

/* 開傘（PWM 舵機）地面自測：設 1 → 每次重啟自動跑一次舵機釋放序列後停住，不進飛控。
 * 台面測試專用，飛行前務必設回 0。詳見 pyro_selftest.h（含刪除方式）。 */
#ifndef FEATURE_PYRO_SELFTEST
#define FEATURE_PYRO_SELFTEST 0
#endif

/* 上行手動開傘：地面站經 433 反向打命令，火箭在下行之外空出 1/10 時槽接收。
 * 僅主航電（有 E22 TX/RX + 飛控點火輸出）；地面站送命令端走 IS_GROUND 的 gs_lora_test。
 * 安全：兩段式 ARM→DEPLOY + ARM 逾時自動解除（見 uplink_cmd.c / uplink_proto.h）。 */
#define FEATURE_UPLINK_DEPLOY  IS_PRIMARY

/* 飛控管線（IMU/baro/highG → EKF → FSM → 點火/傘控）：主 + 備皆跑，地面站關閉省資源。 */
#define FEATURE_FLIGHT (!IS_GROUND)

/* 除錯用 USB-CDC printf 直通（台面測試開關，預設關閉）：
 * 設 1 → 不論角色，一律初始化 USB_DEVICE 並讓 printf 改走 USB 虛擬序列埠（見 main.c
 * _write），免接 SWD 除錯器、免佔用被板間鏈路用掉的 USART2，插 USB 線用序列埠工具
 * 即可看 log。best-effort（PC 未讀取即丟棄），不阻塞飛控路徑。
 * ★飛行前務必改回 0——純台面除錯用，不是飛行設計的一部分（會多出 USB 列舉與常駐
 * CPU 佔用，見下方 FEATURE_USB_CDC 註解）。 */
#ifndef FEATURE_USB_DEBUG_LOG
#define FEATURE_USB_DEBUG_LOG 0
#endif

/* USB 虛擬序列埠（CDC）：地面站固定啟用，串流接收到的遙測 + 自身 GPS 給 PC；
 * 其餘角色僅在上方 FEATURE_USB_DEBUG_LOG 開啟時（台面除錯）才啟用。
 * 飛行版預設不啟用（MX_USB_DEVICE_Init 被守住）→ 不列舉、不進 USB 中斷、不佔 CPU。 */
#define FEATURE_USB_CDC (IS_GROUND || FEATURE_USB_DEBUG_LOG)

/* 板間鏈路：主備皆需，地面站不參與（非配對飛控板）。啟用後 USART2 改跑二進制鏈路，
 * printf 改走 SWO/ITM 或 USB-CDC（見 FEATURE_USB_DEBUG_LOG / main.c _write 的優先序）。
 * 設 0 可回復「USART2 = printf 除錯橋」的單板開發建置（make monitor 可見，@460800）。 */
#ifndef FEATURE_LINK
#define FEATURE_LINK   (!IS_GROUND)   /* 雙板飛行版：主/備皆跑；地面站不參與 */
#endif

/* === 板間鏈路參數（USART2 硬體全雙工） === */
#define LINK_BAUD             38400U /* 主備兩板 USART2 同此值；短排線餘裕充足 */
#define LINK_TX_PERIOD_MS     50U    /* 自身狀態廣播週期（20 Hz；飛控 100 Hz 每 5 次送一次） */
#define LINK_PEER_TIMEOUT_MS  300U   /* 超過此值無有效封包 → 對端視為失聯（備板轉全自主） */

/* === 備援航電開傘偏壓（不對稱：主決策、備補位） ===
 * 備板獨立跑自己的 FSM；判到開傘條件後不立即輸出，先等 BACKUP_GRACE_MS 觀察主板是否
 * 送來開傘通知（flags 帶 DROGUE_FIRED / MAIN_DEPLOYED）。若收到 → 抑制（主板已點，
 * 二極體 OR 同一點火頭）；若 grace 到期仍未收到（主板漏點 / 失聯）→ 備板自行點火。 */
#define BACKUP_GRACE_MS       300U   /* 備板判到開傘後給主板的寬限期 */

/* === GPS-ONLY 隔離除錯開關 ════════════════════════════════════════════════
 * 1 = 關閉「除 GPS 外」的所有射頻/匯流排活動（LoRa 433+920、IMU/baro/highG 飛控管線、
 *     磁力計、Flash 記錄、USB、蜂鳴器、板間鏈路、上行開傘…），只留 GPS(USART6) 與
 *     printf 除錯輸出(USART2)。用途：判斷 GPS 在板上收 0 顆是否為飛控板自身 desense
 *     ——把最可能的干擾源(LoRa/SMPS 負載/SPI/I2C)全靜音後，若衛星數開始上來即坐實干擾。
 *     ★純測試用，量完務必改回 0，否則飛控/遙測/記錄全不會啟動！
 * 0 = 正常（依 BOARD_ROLE 決定各功能）。 */
#ifndef GPS_ONLY_DEBUG
#define GPS_ONLY_DEBUG 0
#endif
#if GPS_ONLY_DEBUG
  #undef  FEATURE_MAG
  #undef  FEATURE_LORA
  #undef  FEATURE_LORA_TX
  #undef  FEATURE_LORA_RX
  #undef  FEATURE_UPLINK_DEPLOY
  #undef  FEATURE_FLIGHT
  #undef  FEATURE_USB_DEBUG_LOG
  #undef  FEATURE_USB_CDC
  #undef  FEATURE_FLASH
  #undef  FEATURE_VFILTER
  #undef  FEATURE_VFILTER_FSM
  #undef  FEATURE_BUZZER
  #undef  FEATURE_PYRO_SELFTEST
  #undef  FEATURE_LINK
  #define FEATURE_MAG           0
  #define FEATURE_LORA          0   /* E22 433 + E80 920 完全不 init、不發射 */
  #define FEATURE_LORA_TX       0
  #define FEATURE_LORA_RX       0
  #define FEATURE_UPLINK_DEPLOY 0
  #define FEATURE_FLIGHT        0   /* 關整條 IMU/baro/highG→EKF→FSM 管線 */
  #define FEATURE_USB_DEBUG_LOG 0
  #define FEATURE_USB_CDC       0
  #define FEATURE_FLASH         0   /* 停 W25Q128 記錄，降低 SPI3 活動 */
  #define FEATURE_VFILTER       0
  #define FEATURE_VFILTER_FSM   0
  #define FEATURE_BUZZER        0
  #define FEATURE_PYRO_SELFTEST 0
  #define FEATURE_LINK          0   /* 板間鏈路本身也是匯流排活動，一併靜音 */
  /* FEATURE_GPS 保持開啟；FEATURE_LINK 強制 0 → printf 走 USART2@460800 可監看 [GPS_RAW] */
#endif

#endif /* BOARD_CONFIG_H */

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
航電板 3D 即時姿態儀與終端監控儀 (Premium Dark Mode Dashboard)

功能特色:
  1. 自動依賴安裝：啟動時自動檢測並安裝 pyserial, numpy, matplotlib 套件。
  2. 執行緒安全設計：背景執行緒負責 UART 高速收發（支援 460800 鮑率），透過 Thread-safe Queue 與 GUI 交互。
  3. 即時 3D 姿態渲染：解析 EKF [TELE] 的四元數 q:qw,qx,qy,qz，以 Matplotlib 3D 繪製火箭姿態（圓柱體身+圓錐體頭+4個穩定翼）。
  4. 雙窗格儀表板：
     - 左側：終端滾動日誌，支援重要關鍵字高亮（RATE、MAG、GPS、OK、ERROR）。
     - 右側：即時 3D 火箭姿態，支援視角旋轉與暫停/繼續渲染。
  5. 頂部狀態列：數位卡片顯示 BMI088、ADXL、BMP、MAG、GPS 最新採樣率及 Flash 寫入包數。
  6. 串口自動偵測：自動掃描 Mac 上的 USB 串口（支援 cu.usbserial 等）。
"""

import sys
import os
import time
import re
import queue
import threading
from datetime import datetime
from collections import deque

# ==================== 1. 自動安裝依賴套件 ====================
def install_and_import(package, import_name=None):
    if import_name is None:
        import_name = package
    try:
        __import__(import_name)
    except ImportError:
        print(f"[*] 偵測到未安裝 {package}，正在自動進行 pip 安裝...")
        try:
            subprocess = __import__("subprocess")
            subprocess.check_call([sys.executable, "-m", "pip", "install", package])
            print(f"[+] {package} 安裝成功！")
        except Exception as e:
            print(f"[-] 安裝 {package} 失敗，請手動執行: pip install {package}。錯誤: {e}")
            sys.exit(1)

install_and_import("pyserial", "serial")
install_and_import("numpy")
install_and_import("matplotlib")

# 選配：GPS 地圖元件（OpenStreetMap 圖磚，線上載入）。安裝失敗不擋主程式，
# GPS 分頁會自動退回「相對軌跡圖」（原點=第一筆定位，離線可用）。
try:
    import tkintermapview
    HAVE_MAPVIEW = True
except ImportError:
    try:
        __import__("subprocess").check_call([sys.executable, "-m", "pip", "install", "tkintermapview"])
        import tkintermapview
        HAVE_MAPVIEW = True
    except Exception:
        tkintermapview = None
        HAVE_MAPVIEW = False
        print("[*] tkintermapview 未安裝（無網路？），GPS 地圖改用相對軌跡圖。")

# 成功載入依賴套件
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import random

import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

# Add parent directory to sys.path to load serial_link
import sys
import os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import serial_link   # P3：port/baud 預設與自動偵測統一於此

# ==================== 2. 全域配置參數 ====================
DEFAULT_BAUD = serial_link.DEFAULT_BAUD
DEFAULT_PORT = serial_link.PREFERRED_PORT

# ==================== 3. 3D 幾何生成工具 ====================
def generate_rocket_geometry(r=0.25, h_body=1.8, h_nose=0.8):
    """
    在局部座標系（火箭長軸為 Z 軸）中生成圓柱體身（三分段）、圓錐體頭、黑色噴嘴、尾焰與 3D 穩定翼的頂點
    """
    theta = np.linspace(0, 2*np.pi, 24)
    
    # 1. 圓柱體身分段 (Cylinder segments)
    # Segment 1: 下段 (Lower body, Z: -h_body/2 -> -h_body/6)
    z_b1 = np.linspace(-h_body/2, -h_body/6, 6)
    theta_grid, z_grid_b1 = np.meshgrid(theta, z_b1)
    x_grid_b1 = r * np.cos(theta_grid)
    y_grid_b1 = r * np.sin(theta_grid)
    
    # Segment 2: 中段 (Mid stripe, Z: -h_body/6 -> h_body/6)
    z_b2 = np.linspace(-h_body/6, h_body/6, 6)
    theta_grid, z_grid_b2 = np.meshgrid(theta, z_b2)
    x_grid_b2 = r * np.cos(theta_grid)
    y_grid_b2 = r * np.sin(theta_grid)
    
    # Segment 3: 上段 (Upper body, Z: h_body/6 -> h_body/2)
    z_b3 = np.linspace(h_body/6, h_body/2, 6)
    theta_grid, z_grid_b3 = np.meshgrid(theta, z_b3)
    x_grid_b3 = r * np.cos(theta_grid)
    y_grid_b3 = r * np.sin(theta_grid)
    
    # 2. 圓錐體頭 (Nose Cone, Z: h_body/2 -> h_body/2 + h_nose)
    z_nose = np.linspace(h_body/2, h_body/2 + h_nose, 8)
    theta_grid_n, z_grid_n = np.meshgrid(theta, z_nose)
    r_taper = r * ( (h_body/2 + h_nose) - z_grid_n ) / h_nose
    x_grid_n = r_taper * np.cos(theta_grid_n)
    y_grid_n = r_taper * np.sin(theta_grid_n)
    
    # 3. 噴嘴 (Nozzle Cone, Z: -h_body/2 - 0.2 -> -h_body/2)
    z_nozzle = np.linspace(-h_body/2 - 0.2, -h_body/2, 5)
    theta_grid_noz, z_grid_noz = np.meshgrid(theta, z_nozzle)
    r_noz_taper = r * 0.75 + (r * 0.25) * (z_grid_noz - (-h_body/2 - 0.2)) / 0.2
    x_grid_noz = r_noz_taper * np.cos(theta_grid_noz)
    y_grid_noz = r_noz_taper * np.sin(theta_grid_noz)
    
    # 4. 引擎火光 (Thrust Plume Base, Z: -0.9 -> 0.0, top is 0.0 at nozzle base)
    z_flame = np.linspace(-0.9, 0.0, 8)
    theta_grid_fl, z_grid_fl = np.meshgrid(theta, z_flame)
    r_fl_taper = r * 0.65 * (z_grid_fl - (-0.9)) / 0.9
    x_grid_fl = r_fl_taper * np.cos(theta_grid_fl)
    y_grid_fl = r_fl_taper * np.sin(theta_grid_fl)
    
    # 5. 四個穩定翼 (Fins - 3D 梯形面，每個鰭翼由 4 個頂點定義的 3D 多邊形)
    fins = [
        # Fin 1: X+
        np.array([[r, 0, -h_body/2], [r + 0.4, 0, -h_body/2], [r + 0.3, 0, -h_body/2 + 0.45], [r, 0, -h_body/2 + 0.45]]),
        # Fin 2: X-
        np.array([[-r, 0, -h_body/2], [-r - 0.4, 0, -h_body/2], [-r - 0.3, 0, -h_body/2 + 0.45], [-r, 0, -h_body/2 + 0.45]]),
        # Fin 3: Y+
        np.array([[0, r, -h_body/2], [0, r + 0.4, -h_body/2], [0, r + 0.3, -h_body/2 + 0.45], [0, r, -h_body/2 + 0.45]]),
        # Fin 4: Y-
        np.array([[0, -r, -h_body/2], [0, -r - 0.4, -h_body/2], [0, -r - 0.3, -h_body/2 + 0.45], [0, -r, -h_body/2 + 0.45]])
    ]
    
    return (x_grid_b1, y_grid_b1, z_grid_b1,
            x_grid_b2, y_grid_b2, z_grid_b2,
            x_grid_b3, y_grid_b3, z_grid_b3,
            x_grid_n, y_grid_n, z_grid_n,
            x_grid_noz, y_grid_noz, z_grid_noz,
            x_grid_fl, y_grid_fl, z_grid_fl,
            fins)

def rotate_points(x, y, z, R):
    """將網格點套用 3D 旋轉矩陣 R"""
    orig_shape = x.shape
    pts = np.vstack((x.flatten(), y.flatten(), z.flatten()))
    rot_pts = R @ pts
    rx = rot_pts[0].reshape(orig_shape)
    ry = rot_pts[1].reshape(orig_shape)
    rz = rot_pts[2].reshape(orig_shape)
    return rx, ry, rz

def quaternion_to_matrix(q):
    """將四元數 [qw, qx, qy, qz] 轉換為旋轉矩陣 R (航空慣用基準)"""
    w, x, y, z = q
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-6:
        return np.eye(3)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    R = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])
    return R

def quaternion_to_euler(q):
    """
    將四元數 [qw, qx, qy, qz] 轉換為歐拉角 Roll, Pitch, Yaw (以度為單位)
    採用航太常用 Z-Y-X 順序 (Tait-Bryan angles)
    """
    w, x, y, z = q
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    # Roll (X-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (Y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    # Yaw (Z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    # 轉換成角度，並將 Yaw 映射到 0~360 度
    deg_yaw = np.degrees(yaw)
    if deg_yaw < 0:
        deg_yaw += 360.0
        
    return np.degrees(roll), np.degrees(pitch), deg_yaw

# ==================== 4. 主 GUI 應用程式 ====================
class RocketDashboardApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Antigravity Rocket Avionics 3D Real-Time Dashboard")
        # 四象限同屏（log/圖表/3D/地圖）需要較大視窗：依螢幕自適應，小螢幕縮到可用範圍
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.root.geometry(f"{min(1680, sw - 60)}x{min(1000, sh - 80)}")
        self.root.minsize(1150, 760)
        self.root.configure(bg="#151515")
        
        # 狀態配置
        self.serial_thread = None
        self.running = False
        self.paused = False
        self.data_queue = queue.Queue()
        self.fsm_state = "STATE_PAD"

        # 角色自動偵測（PRIMARY 主航電 / BACKUP 備援航電 / GROUND 地面站）
        # 被動：解析 [BOOT]/[ROLE]/[ROLE_ID]/[GS_*] 特徵行；主動：連線後送 'role' 命令。
        self.detected_role = None
        self.detected_fw = ""
        self.role_query_attempts = 0

        # LoRa 參數現值快取（由 [E80]/[E22]/[LORA433]/[LORA 920MHz] 回報行解析）
        self.lora_e80_state = {}   # freq_hz, sf, bw_idx, bw_khz, cr, pwr, pre
        self.lora_e22_state = {}   # freq_mhz, ch, power, air_rate

        # 飛行圖表時間序列：(t, 值)，t = time.monotonic() − chart_t0（秒）
        # 資料源：EKF=[TELE]/[GS_PKT]；原始=10Hz 裸 CSV 行(BMI/ADXL/Baro)、[GPS]、[GS_PKT] accel
        self.chart_t0 = time.monotonic()
        _N = 6000
        self.ts_alt_ekf  = deque(maxlen=_N)   # EKF 高度 m
        self.ts_alt_baro = deque(maxlen=_N)   # 氣壓高度 m（原始）
        self.ts_alt_gps  = deque(maxlen=_N)   # GPS 海拔 m（原始）
        self.ts_vz_ekf   = deque(maxlen=_N)   # EKF 垂直速度 m/s
        self.ts_spd_gps  = deque(maxlen=_N)   # GPS 地速 m/s（原始）
        self.ts_acc_bmi  = deque(maxlen=_N)   # BMI088 |a| g（原始）
        self.ts_acc_adxl = deque(maxlen=_N)   # ADXL375 |a| g（原始）
        self.chart_dirty = False
        self.chart_paused = False

        # GPS 地圖狀態
        self.gps_track = []       # 火箭軌跡 [(lat, lon)]
        self.gps_last = None      # 最新定位 {lat, lon, alt, spd, sats, fix}
        self.gs_own_pos = None    # 地面站自身 (lat, lon)（[GS_GPS]）
        self.map_home = None      # 第一筆定位（Home/原點）
        self.map_dirty = False
        self.map_zoomed = False   # 首筆定位時自動 zoom-in 一次
        
        # GPS 尋星與解包狀態
        self.gps_status_info = {
            "fix": 0,
            "sats": 0,
            "stale": 1,
            "ok": 0,
            "err": 0,
            "rate": 0.0
        }
        
        # 記錄檔配置
        self.log_file = None
        self.save_log_var = tk.BooleanVar(value=True)
        
        # 初始化 3D 幾何本地數據
        self.local_geom = generate_rocket_geometry()
        self.last_q = [1.0, 0.0, 0.0, 0.0]
        
        # 暫存最新感測器數據以進行一致性測試
        self.latest_imu = None
        self.latest_highg = None
        self.latest_mag = None
        
        # 地磁計校正與鎖定狀態
        self.board_mag_offsets = [131072.0, 131072.0, 131072.0]
        self.calib_x = []
        self.calib_y = []
        self.collecting_data = False
        self.new_ox = 131072.0
        self.new_oy = 131072.0
        
        # 設置 UI 樣式與結構
        self.setup_styles()
        self.create_widgets()
        
        # 開機自動掃描串口
        self.scan_ports()
        
        # 啟動 Tkinter 佇列定時輪詢
        self.root.after(10, self.poll_queue)
        
        # 關閉視窗處理
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        
        # 深色控制卡片風格
        style.configure("TFrame", background="#1e1e1e")
        style.configure("Card.TFrame", background="#222222", borderwidth=1, relief="ridge")
        style.configure("Header.TLabel", background="#151515", foreground="#ffffff", font=("Helvetica", 14, "bold"))
        style.configure("Card.TLabel", background="#222222", foreground="#aaaaaa", font=("Helvetica", 9))
        style.configure("CardVal.TLabel", background="#222222", foreground="#00d2ff", font=("Helvetica", 12, "bold"))
        
        # 按鈕風格
        style.configure("TButton", font=("Helvetica", 10, "bold"), background="#333333", foreground="#ffffff", borderwidth=0)
        style.map("TButton", background=[("active", "#00d2ff")], foreground=[("active", "#151515")])
        style.configure("Connect.TButton", font=("Helvetica", 10, "bold"), background="#28a745", foreground="#ffffff")
        style.configure("Disconnect.TButton", font=("Helvetica", 10, "bold"), background="#dc3545", foreground="#ffffff")

        # LoRa 設定面板分頁（深色 Notebook）
        style.configure("TNotebook", background="#1c1c1c", borderwidth=0)
        style.configure("TNotebook.Tab", background="#2a2a2a", foreground="#cccccc",
                        font=("Helvetica", 10, "bold"), padding=[14, 6])
        style.map("TNotebook.Tab",
                  background=[("selected", "#00435a")],
                  foreground=[("selected", "#00d2ff")])

    def create_widgets(self):
        # ------------------ 頂部標題與連接面板 ------------------
        top_bar = tk.Frame(self.root, bg="#151515", height=60)
        top_bar.pack(fill=tk.X, side=tk.TOP, padx=15, pady=8)
        
        title_label = tk.Label(top_bar, text="ROCKET AVIONICS 3D ATTITUDE DASHBOARD", bg="#151515", fg="#00d2ff", font=("Helvetica", 16, "bold"))
        title_label.pack(side=tk.LEFT, pady=5)

        # 角色徽章：自動偵測連接的是 主航電 / 備援航電 / 地面站
        self.lbl_role = tk.Label(top_bar, text="⚫ 未連線", bg="#151515", fg="#777777",
                                 font=("Helvetica", 12, "bold"))
        self.lbl_role.pack(side=tk.LEFT, padx=16, pady=5)

        # 主/備板間鏈路溝通狀態徽章（解析 [LINK] 行，見 update_link_status）。
        # 僅主/備航電會印 [LINK]（FEATURE_LINK，地面站不參與），未連線/未收到過該行時顯示灰色。
        self.lbl_link = tk.Label(top_bar, text="🔗 --", bg="#151515", fg="#555555",
                                 font=("Helvetica", 11, "bold"))
        self.lbl_link.pack(side=tk.LEFT, padx=4, pady=5)
        self.link_last_age_ms = None

        # 連接控制區
        conn_frame = tk.Frame(top_bar, bg="#151515")
        conn_frame.pack(side=tk.RIGHT, pady=5)
        
        tk.Label(conn_frame, text="串口:", bg="#151515", fg="#aaaaaa", font=("Helvetica", 10)).pack(side=tk.LEFT, padx=3)
        self.port_combo = ttk.Combobox(conn_frame, width=22, font=("Helvetica", 10))
        self.port_combo.pack(side=tk.LEFT, padx=3)
        
        btn_scan = ttk.Button(conn_frame, text="🔄", width=3, command=self.scan_ports)
        btn_scan.pack(side=tk.LEFT, padx=3)
        
        tk.Label(conn_frame, text="鮑率:", bg="#151515", fg="#aaaaaa", font=("Helvetica", 10)).pack(side=tk.LEFT, padx=3)
        self.baud_combo = ttk.Combobox(conn_frame, values=[9600, 38400, 115200, 460800, 921600], width=8, font=("Helvetica", 10))
        self.baud_combo.set(DEFAULT_BAUD)
        self.baud_combo.pack(side=tk.LEFT, padx=3)
        
        chk_log = tk.Checkbutton(conn_frame, text="同步日誌", variable=self.save_log_var, bg="#151515", fg="#00d2ff", selectcolor="#151515", font=("Helvetica", 9))
        chk_log.pack(side=tk.LEFT, padx=8)
        
        self.btn_connect = ttk.Button(conn_frame, text="連接 GPS / 航電", width=12, command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=5)

        # ------------------ 頂部數字感測卡片 ------------------
        self.cards_frame = tk.Frame(self.root, bg="#151515")
        self.cards_frame.pack(fill=tk.X, side=tk.TOP, padx=15, pady=5)
        
        self.cards = {}
        card_labels = [
            ("BMI088 A", "bmi_a", "0.00 Hz", "#00d2ff"),
            ("BMI088 G", "bmi_g", "0.00 Hz", "#00d2ff"),
            ("ADXL375", "adxl", "0.00 Hz", "#ffcc00"),
            ("BMP388", "bmp", "0.00 Hz", "#28a745"),
            ("MMC5983", "mag", "0.00 Hz", "#ff3b30"),
            ("GPS Update", "gps", "0.00 Hz", "#e03bfb"),
            ("Flash PKTs", "flash_pkt", "0 Pkts", "#ffffff"),
            ("ALT 高度", "alt", "-- m", "#00e676"),
            ("Vz 垂直速度", "vz", "-- m/s", "#00e676"),
            ("BATTERY 電池", "bat", "-- V", "#ffa500")
        ]
        
        for idx, (title, key, default, color) in enumerate(card_labels):
            self.cards_frame.columnconfigure(idx, weight=1, uniform="equal")
            card = ttk.Frame(self.cards_frame, style="Card.TFrame")
            card.grid(row=0, column=idx, padx=5, sticky="nsew")
            
            lbl_title = ttk.Label(card, text=title, style="Card.TLabel")
            lbl_title.pack(anchor="w", padx=8, pady=4)
            
            lbl_val = ttk.Label(card, text=default, font=("Helvetica", 13, "bold"), background="#222222", foreground=color)
            lbl_val.pack(anchor="w", padx=8, pady=4)
            
            self.cards[key] = lbl_val

        # ------------------ 中部四象限面板（log / 圖表 / 3D 姿態 / 地圖 同時可見） ------------------
        # 巢狀 PanedWindow：水平分左右兩欄，各欄再垂直分上下，四條分隔線皆可拖曳調整。
        main_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg="#151515",
                                   sashwidth=6, sashrelief="flat", bd=0)
        main_pane.pack(fill=tk.BOTH, expand=True, side=tk.TOP, padx=15, pady=10)

        left_pane = tk.PanedWindow(main_pane, orient=tk.VERTICAL, bg="#151515",
                                   sashwidth=6, sashrelief="flat", bd=0)
        right_pane = tk.PanedWindow(main_pane, orient=tk.VERTICAL, bg="#151515",
                                    sashwidth=6, sashrelief="flat", bd=0)
        main_pane.add(left_pane, width=660, minsize=380)
        main_pane.add(right_pane, minsize=420)

        # --- 左上: 終端日誌 (Terminal Console) ---
        left_frame = tk.Frame(left_pane, bg="#1e1e1e")

        # --- 指令/回應區：只顯示使用者指令與韌體對指令的回應（不含開機/系統/遙測）---
        tk.Label(left_frame, text=" ★ 指令 / 回應", bg="#1e1e1e",
                 fg="#ffcc00", font=("Monaco", 10, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        self.event_console = ScrolledText(left_frame, bg="#0d0d0d", fg="#e0e0e0",
                                          insertbackground="white", font=("Monaco", 9),
                                          borderwidth=0, highlightthickness=1,
                                          highlightbackground="#3a3a1a", height=9)
        self.event_console.pack(fill=tk.X, padx=5, pady=(2, 6))
        self.event_console.tag_config("cmd", foreground="#00d2ff")
        self.event_console.tag_config("resp", foreground="#33ff88")
        self.event_console.tag_config("boot", foreground="#e07bfb")
        self.event_console.tag_config("err", foreground="#ff5b5b", background="#2a0000")

        tk.Label(left_frame, text=" > TERMINAL TELEMETRY STREAM（完整資訊流）", bg="#1e1e1e", fg="#00d2ff", font=("Monaco", 10, "bold")).pack(anchor="w", padx=10, pady=5)

        # 終端底部按鈕區（先 pack 於底部，讓 console 撐滿剩餘空間）
        console_tools = tk.Frame(left_frame, bg="#1e1e1e")
        console_tools.pack(fill=tk.X, side=tk.BOTTOM, padx=5, pady=5)

        ttk.Button(console_tools, text="清除終端", width=10, command=self.clear_console).pack(side=tk.LEFT, padx=5)
        ttk.Button(console_tools, text="📡 LoRa 參數設定", width=15, command=self.open_lora_panel).pack(side=tk.LEFT, padx=5)
        ttk.Button(console_tools, text="🧭 軸向對齊測試", width=14, command=self.open_test_wizard).pack(side=tk.LEFT, padx=5)
        ttk.Button(console_tools, text="🧲 磁強計校正與鎖定", width=18, command=self.open_mag_calibration).pack(side=tk.LEFT, padx=5)
        self.lbl_drops = tk.Label(console_tools, text="EKF Queue Drops: 0", bg="#1e1e1e", fg="#aaaaaa", font=("Monaco", 9))
        self.lbl_drops.pack(side=tk.RIGHT, padx=10)

        # --- 手動指令列：自己打指令送到航電板（回應會隨遙測串流進終端）---
        cmd_bar = tk.Frame(left_frame, bg="#1e1e1e")
        cmd_bar.pack(fill=tk.X, side=tk.BOTTOM, padx=5, pady=(0, 4))
        tk.Label(cmd_bar, text="指令 ➤", bg="#1e1e1e", fg="#00d2ff",
                 font=("Monaco", 10, "bold")).pack(side=tk.LEFT, padx=(6, 4))
        self._cmd_history = []
        self._cmd_history_idx = 0
        self.cmd_entry = tk.Entry(cmd_bar, bg="#0e0e0e", fg="#33ff33",
                                  insertbackground="white", font=("Monaco", 10),
                                  relief="flat", highlightthickness=1,
                                  highlightbackground="#333333", highlightcolor="#00d2ff")
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.cmd_entry.bind("<Return>", self._on_manual_cmd)
        self.cmd_entry.bind("<Up>", self._manual_cmd_history_prev)
        self.cmd_entry.bind("<Down>", self._manual_cmd_history_next)
        ttk.Button(cmd_bar, text="送出", width=8, command=self._on_manual_cmd).pack(side=tk.LEFT, padx=4)
        tk.Button(cmd_bar, text="help", bg="#2a2a2a", fg="#cccccc", relief="flat",
                  font=("Monaco", 9), command=lambda: self.send_command("help")).pack(side=tk.LEFT, padx=(2, 4))

        # 終端文本框，設定為深色主題
        self.console = ScrolledText(left_frame, bg="#101010", fg="#33ff33", insertbackground="white", font=("Monaco", 9), borderwidth=0, highlightthickness=0)
        self.console.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 字體高亮色彩規則
        self.console.tag_config("rate", foreground="#00d2ff")
        self.console.tag_config("mag", foreground="#ff3b30")
        self.console.tag_config("gps", foreground="#ffcc00")
        self.console.tag_config("ok", foreground="#28a745")
        self.console.tag_config("err", foreground="#dc3545", background="#2a0000")
        self.console.tag_config("tele", foreground="#aaaaaa")
        self.console.tag_config("lora", foreground="#e07bfb")
        self.console.tag_config("link", foreground="#00e676")

        left_pane.add(left_frame, height=400, minsize=180)

        # --- 左下: 飛行圖表（高度 / 速度 / 加速度，原始+EKF） ---
        charts_frame = tk.Frame(left_pane, bg="#1e1e1e")
        tk.Label(charts_frame, text=" > FLIGHT CHARTS (RAW + EKF)", bg="#1e1e1e", fg="#00d2ff",
                 font=("Monaco", 10, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        self.build_charts_tab(charts_frame)
        left_pane.add(charts_frame, minsize=240)

        # --- 右上: 3D 姿態 ---
        att_frame = tk.Frame(right_pane, bg="#1e1e1e")
        title_3d_frame = tk.Frame(att_frame, bg="#1e1e1e")
        title_3d_frame.pack(fill=tk.X, side=tk.TOP, padx=10, pady=5)

        tk.Label(title_3d_frame, text=" > 3D ROCKET ATTITUDE (EKF ESTIMATE)", bg="#1e1e1e", fg="#00d2ff", font=("Monaco", 10, "bold")).pack(side=tk.LEFT)

        self.btn_pause = ttk.Button(title_3d_frame, text="暫停渲染", width=8, command=self.toggle_pause)
        self.btn_pause.pack(side=tk.RIGHT, padx=5)

        ttk.Button(title_3d_frame, text="重設視角", width=8, command=self.reset_view).pack(side=tk.RIGHT, padx=5)

        # Matplotlib 3D 畫布整合
        self.fig = plt.figure(facecolor="#1e1e1e")
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor("#1e1e1e")
        self.reset_view()

        self.canvas = FigureCanvasTkAgg(self.fig, master=att_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        right_pane.add(att_frame, height=430, minsize=240)

        # --- 右下: GPS 即時地圖 ---
        map_frame = tk.Frame(right_pane, bg="#1e1e1e")
        tk.Label(map_frame, text=" > GPS LIVE MAP", bg="#1e1e1e", fg="#00d2ff",
                 font=("Monaco", 10, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        self.build_map_tab(map_frame)
        right_pane.add(map_frame, minsize=220)

        # 初始化靜態 3D 火箭渲染
        self.update_3d_plot([1.0, 0.0, 0.0, 0.0])

        # 圖表 / 地圖獨立刷新迴圈（有新資料才重繪）
        self.root.after(400, self.charts_redraw_loop)
        self.root.after(1000, self.map_redraw_loop)

    # ------------------ UI 操作函數 ------------------
    def scan_ports(self):
        """自動偵測可用串口（偵測優先序統一在 serial_link.auto_port）"""
        ports = serial_link.list_candidate_ports()
        self.port_combo['values'] = ports
        self.port_combo.set(serial_link.auto_port() or DEFAULT_PORT)

    def clear_console(self):
        self.console.delete("1.0", tk.END)
        if hasattr(self, 'event_console'):
            self.event_console.delete("1.0", tk.END)

    def toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.config(text="繼續渲染" if self.paused else "暫停渲染")

    def reset_view(self):
        self.ax.view_init(elev=20, azim=45)
        if hasattr(self, 'canvas') and self.canvas:
            self.canvas.draw()

    def clear_rate_cards(self):
        defaults = {"flash_pkt": "0 Pkts", "alt": "-- m", "vz": "-- m/s", "bat": "-- V"}
        for key in self.cards:
            self.cards[key].config(text=defaults.get(key, "0.00 Hz"))
        self.lbl_drops.config(text="EKF Queue Drops: 0")

    # ==================== 飛行圖表 / GPS 地圖（與 3D、log 四象限同屏） ====================
    def now_t(self):
        return time.monotonic() - self.chart_t0

    def clear_chart_data(self):
        for dq in (self.ts_alt_ekf, self.ts_alt_baro, self.ts_alt_gps,
                   self.ts_vz_ekf, self.ts_spd_gps, self.ts_acc_bmi, self.ts_acc_adxl):
            dq.clear()
        self.chart_t0 = time.monotonic()
        self.chart_dirty = True

    def clear_gps_track(self):
        self.gps_track = []
        self.gps_last = None
        self.map_home = None
        self.map_zoomed = False
        self.map_dirty = True
        if HAVE_MAPVIEW and hasattr(self, 'map_widget'):
            try:
                if getattr(self, 'map_path', None):
                    self.map_path.delete()
                if getattr(self, 'map_marker', None):
                    self.map_marker.delete()
            except Exception:
                pass
            self.map_path = None
            self.map_marker = None

    # ---- 飛行圖表分頁 ----
    def build_charts_tab(self, tab):
        # 單列：大字即時讀數（左）+ 時間窗/暫停/清除 控制（右），節省四象限垂直空間
        bar = tk.Frame(tab, bg="#1e1e1e")
        bar.pack(fill=tk.X, padx=8, pady=(4, 2))
        self.lbl_big_alt = tk.Label(bar, text="ALT --.- m", bg="#1e1e1e", fg="#00e676",
                                    font=("Monaco", 13, "bold"))
        self.lbl_big_alt.pack(side=tk.LEFT, padx=(4, 12))
        self.lbl_big_vz = tk.Label(bar, text="Vz --.- m/s", bg="#1e1e1e", fg="#00e5ff",
                                   font=("Monaco", 13, "bold"))
        self.lbl_big_vz.pack(side=tk.LEFT, padx=(0, 12))
        self.lbl_big_acc = tk.Label(bar, text="|a| --.- g", bg="#1e1e1e", fg="#ffcc00",
                                    font=("Monaco", 13, "bold"))
        self.lbl_big_acc.pack(side=tk.LEFT)

        ttk.Button(bar, text="清除", width=5, command=self.clear_chart_data).pack(side=tk.RIGHT, padx=(4, 2))
        self.btn_chart_pause = ttk.Button(bar, text="暫停", width=5, command=self.toggle_chart_pause)
        self.btn_chart_pause.pack(side=tk.RIGHT, padx=4)
        self.chart_win_combo = ttk.Combobox(bar, values=["30 s", "60 s", "120 s", "300 s"],
                                            width=6, state="readonly", font=("Helvetica", 9))
        self.chart_win_combo.current(1)
        self.chart_win_combo.pack(side=tk.RIGHT)
        tk.Label(bar, text="時間窗:", bg="#1e1e1e", fg="#aaaaaa",
                 font=("Helvetica", 9)).pack(side=tk.RIGHT, padx=(8, 3))

        # 三聯圖：高度 / 速度 / 加速度（共用時間軸；標籤用 ASCII 避免 matplotlib 缺中文字型）
        self.chart_fig = plt.figure(facecolor="#101010")
        gs = self.chart_fig.add_gridspec(3, 1, hspace=0.32, left=0.11, right=0.97, top=0.97, bottom=0.08)
        self.ax_alt = self.chart_fig.add_subplot(gs[0])
        self.ax_vel = self.chart_fig.add_subplot(gs[1], sharex=self.ax_alt)
        self.ax_acc = self.chart_fig.add_subplot(gs[2], sharex=self.ax_alt)

        for ax, ylab in ((self.ax_alt, "Alt (m)"), (self.ax_vel, "Vel (m/s)"), (self.ax_acc, "Acc (g)")):
            ax.set_facecolor("#151515")
            ax.tick_params(colors="#888888", labelsize=8)
            for sp in ax.spines.values():
                sp.set_color("#333333")
            ax.grid(color="#2a2a2a", linewidth=0.5, alpha=0.6)
            ax.set_ylabel(ylab, color="#aaaaaa", fontsize=9)
        self.ax_acc.set_xlabel("t (s)", color="#aaaaaa", fontsize=9)

        self.ln_alt_ekf,  = self.ax_alt.plot([], [], color="#00e5ff", lw=1.6, label="EKF")
        self.ln_alt_baro, = self.ax_alt.plot([], [], color="#ff9800", lw=0.9, label="Baro raw")
        self.ln_alt_gps,  = self.ax_alt.plot([], [], color="#e03bfb", lw=0, marker=".", ms=3, label="GPS raw")

        self.ln_vz_ekf,   = self.ax_vel.plot([], [], color="#00e5ff", lw=1.6, label="EKF Vz")
        self.ln_spd_gps,  = self.ax_vel.plot([], [], color="#ffcc00", lw=0, marker=".", ms=3, label="GPS spd raw")

        self.ln_acc_bmi,  = self.ax_acc.plot([], [], color="#28d745", lw=0.9, label="BMI088 raw")
        self.ln_acc_adxl, = self.ax_acc.plot([], [], color="#ff3b30", lw=0.9, alpha=0.8, label="ADXL375 raw")
        self.ln_acc_ekf,  = self.ax_acc.plot([], [], color="#00e5ff", lw=1.4, ls="--", label="EKF dVz/dt")

        for ax in (self.ax_alt, self.ax_vel, self.ax_acc):
            leg = ax.legend(loc="upper left", fontsize=7, facecolor="#1c1c1c",
                            edgecolor="#333333", labelcolor="#cccccc", ncol=3)
            leg.get_frame().set_alpha(0.7)

        self.chart_canvas = FigureCanvasTkAgg(self.chart_fig, master=tab)
        self.chart_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def toggle_chart_pause(self):
        self.chart_paused = not self.chart_paused
        self.btn_chart_pause.config(text="繼續" if self.chart_paused else "暫停")

    @staticmethod
    def _series_window(dq, tmin):
        """deque[(t,v)] → 視窗內 (ts, vs) numpy 陣列"""
        if not dq:
            return np.array([]), np.array([])
        arr = np.array(dq, dtype=float)
        sel = arr[:, 0] >= tmin
        return arr[sel, 0], arr[sel, 1]

    def redraw_charts(self):
        win = float(self.chart_win_combo.get().split()[0])
        tmax = self.now_t()
        tmin = max(0.0, tmax - win)

        pairs = [
            (self.ln_alt_ekf, self.ts_alt_ekf), (self.ln_alt_baro, self.ts_alt_baro),
            (self.ln_alt_gps, self.ts_alt_gps),
            (self.ln_vz_ekf, self.ts_vz_ekf), (self.ln_spd_gps, self.ts_spd_gps),
            (self.ln_acc_bmi, self.ts_acc_bmi), (self.ln_acc_adxl, self.ts_acc_adxl),
        ]
        for ln, dq in pairs:
            ts, vs = self._series_window(dq, tmin)
            ln.set_data(ts, vs)

        # EKF 加速度 = EKF Vz 數值微分（中央差分，/9.81 轉 g）
        ts, vs = self._series_window(self.ts_vz_ekf, tmin)
        if len(ts) >= 3:
            self.ln_acc_ekf.set_data(ts, np.gradient(vs, ts) / 9.81)
        else:
            self.ln_acc_ekf.set_data([], [])

        for ax in (self.ax_alt, self.ax_vel, self.ax_acc):
            ax.set_xlim(tmin, max(tmax, tmin + 1.0))
            ax.relim(visible_only=True)
            ax.autoscale_view(scalex=False, scaley=True)
        self.chart_canvas.draw_idle()

        # 大字即時讀數
        if self.ts_alt_ekf:
            self.lbl_big_alt.config(text=f"ALT {self.ts_alt_ekf[-1][1]:.1f} m")
        elif self.ts_alt_baro:
            self.lbl_big_alt.config(text=f"ALT {self.ts_alt_baro[-1][1]:.1f} m (baro)")
        if self.ts_vz_ekf:
            self.lbl_big_vz.config(text=f"Vz {self.ts_vz_ekf[-1][1]:+.1f} m/s")
        if self.ts_acc_bmi:
            self.lbl_big_acc.config(text=f"|a| {self.ts_acc_bmi[-1][1]:.2f} g")

    def charts_redraw_loop(self):
        try:
            if not self.root.winfo_exists():
                return
            if (not self.chart_paused) and self.chart_dirty:
                self.redraw_charts()
                self.chart_dirty = False
            self.root.after(300, self.charts_redraw_loop)
        except tk.TclError:
            pass

    # ---- GPS 地圖分頁 ----
    def build_map_tab(self, tab):
        ctrl = tk.Frame(tab, bg="#1e1e1e")
        ctrl.pack(fill=tk.X, padx=8, pady=(6, 2))
        self.map_follow_var = tk.BooleanVar(value=True)
        tk.Checkbutton(ctrl, text="跟隨最新位置", variable=self.map_follow_var,
                       bg="#1e1e1e", fg="#00d2ff", selectcolor="#151515",
                       font=("Helvetica", 9)).pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrl, text="清除軌跡", width=8, command=self.clear_gps_track).pack(side=tk.LEFT, padx=6)
        mode = "OpenStreetMap 線上地圖" if HAVE_MAPVIEW else "相對軌跡圖（tkintermapview 未安裝/離線）"
        tk.Label(ctrl, text=mode, bg="#1e1e1e", fg="#777777",
                 font=("Helvetica", 9)).pack(side=tk.RIGHT, padx=6)

        self.lbl_gps_info = tk.Label(tab, text="等待 GPS 定位…（火箭=[GPS]/[GS_PKT]，地面站自身=[GS_GPS]）",
                                     bg="#101014", fg="#ffcc00", font=("Monaco", 10),
                                     anchor="w", justify=tk.LEFT, padx=10, pady=6)
        self.lbl_gps_info.pack(fill=tk.X, padx=8, pady=(0, 4))

        # 建立地圖與狀態欄的左右對齊容器
        container = tk.Frame(tab, bg="#1e1e1e")
        container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 左側地圖容器
        map_container = tk.Frame(container, bg="#1e1e1e")
        map_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 右側 GPS 尋星與封包解析狀態面板
        status_panel = tk.Frame(container, bg="#101014", width=180, highlightbackground="#2a2a30", highlightthickness=1)
        status_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(6, 0))
        status_panel.pack_propagate(False)

        # 狀態面板標題
        tk.Label(status_panel, text="📡 GPS 尋星狀態", bg="#101014", fg="#00d2ff",
                 font=("Helvetica", 9, "bold"), pady=6).pack(fill=tk.X)
        
        # 狀態面板網格
        grid = tk.Frame(status_panel, bg="#101014", padx=6, pady=4)
        grid.pack(fill=tk.BOTH, expand=True)
        
        # 1. 定位狀態
        tk.Label(grid, text="定位狀態:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=0, column=0, sticky="w", pady=4)
        self.lbl_gps_status_fix = tk.Label(grid, text="等待數據", bg="#101014", fg="#ffcc00", font=("Helvetica", 9, "bold"), anchor="e")
        self.lbl_gps_status_fix.grid(row=0, column=1, sticky="e", pady=4)
        
        # 2. 衛星數量
        tk.Label(grid, text="衛星數量:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=1, column=0, sticky="w", pady=4)
        self.lbl_gps_status_sats = tk.Label(grid, text="0", bg="#101014", fg="#ffffff", font=("Helvetica", 9, "bold"), anchor="e")
        self.lbl_gps_status_sats.grid(row=1, column=1, sticky="e", pady=4)
        
        # 3. 訊號延遲/Stale狀態
        tk.Label(grid, text="信號狀態:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=2, column=0, sticky="w", pady=4)
        self.lbl_gps_status_stale = tk.Label(grid, text="等待數據", bg="#101014", fg="#777777", font=("Helvetica", 9), anchor="e")
        self.lbl_gps_status_stale.grid(row=2, column=1, sticky="e", pady=4)
        
        # 4. 解析成功
        tk.Label(grid, text="解析成功:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=3, column=0, sticky="w", pady=4)
        self.lbl_gps_status_ok = tk.Label(grid, text="0", bg="#101014", fg="#00e676", font=("Helvetica", 9), anchor="e")
        self.lbl_gps_status_ok.grid(row=3, column=1, sticky="e", pady=4)
        
        # 5. 解析失敗
        tk.Label(grid, text="解析失敗:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=4, column=0, sticky="w", pady=4)
        self.lbl_gps_status_err = tk.Label(grid, text="0", bg="#101014", fg="#ffffff", font=("Helvetica", 9), anchor="e")
        self.lbl_gps_status_err.grid(row=4, column=1, sticky="e", pady=4)
        
        # 6. 更新頻率
        tk.Label(grid, text="更新頻率:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=5, column=0, sticky="w", pady=4)
        self.lbl_gps_status_rate = tk.Label(grid, text="0.00 Hz", bg="#101014", fg="#e03bfb", font=("Helvetica", 9), anchor="e")
        self.lbl_gps_status_rate.grid(row=5, column=1, sticky="e", pady=4)

        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        # 載入實體地圖或後備相對軌跡圖
        if HAVE_MAPVIEW:
            self.map_widget = tkintermapview.TkinterMapView(map_container, corner_radius=0)
            self.map_widget.pack(fill=tk.BOTH, expand=True)
            self.map_widget.set_position(23.97, 120.97)   # 預設台灣中心，待首筆定位跳轉
            self.map_widget.set_zoom(8)
            self.map_marker = None
            self.map_path = None
            self.gs_marker = None
        else:
            # 離線後備：E/N 相對軌跡（原點 = 第一筆定位 Home）
            self.trk_fig = plt.figure(facecolor="#101010")
            self.trk_ax = self.trk_fig.add_subplot(111)
            self.trk_ax.set_facecolor("#151515")
            self.trk_ax.tick_params(colors="#888888", labelsize=8)
            for sp in self.trk_ax.spines.values():
                sp.set_color("#333333")
            self.trk_ax.grid(color="#2a2a2a", linewidth=0.5, alpha=0.6)
            self.trk_ax.set_xlabel("East (m)", color="#aaaaaa", fontsize=9)
            self.trk_ax.set_ylabel("North (m)", color="#aaaaaa", fontsize=9)
            self.trk_ax.set_aspect("equal", adjustable="datalim")
            self.trk_line, = self.trk_ax.plot([], [], color="#00e5ff", lw=1.2)
            self.trk_pt,   = self.trk_ax.plot([], [], color="#ff1744", marker="o", ms=8, lw=0)
            self.trk_home, = self.trk_ax.plot([], [], color="#00e676", marker="^", ms=9, lw=0)
            self.trk_canvas = FigureCanvasTkAgg(self.trk_fig, master=map_container)
            self.trk_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.update_gps_status_panel()

    def update_gps_status_panel(self):
        """重新整理 UI 面板上的 GPS 尋星與解析狀態數據"""
        if not hasattr(self, 'lbl_gps_status_fix'):
            return
        
        info = self.gps_status_info
        
        # 1. 定位狀態標籤
        if info["fix"] == 0:
            self.lbl_gps_status_fix.config(text="未定位 (No Fix)", fg="#ff3366")
        elif info["fix"] == 1:
            self.lbl_gps_status_fix.config(text="已定位 (3D Fix)", fg="#00e676")
        elif info["fix"] == 2:
            self.lbl_gps_status_fix.config(text="差分定位 (DGPS)", fg="#00e676")
        else:
            self.lbl_gps_status_fix.config(text=f"定位中 ({info['fix']})", fg="#ffcc00")
            
        # 2. 衛星數量
        self.lbl_gps_status_sats.config(text=str(info["sats"]), fg="#ffffff" if info["sats"] >= 4 else "#ffcc00")
        
        # 3. 訊號延遲/Stale狀態
        if info["stale"] == 1:
            self.lbl_gps_status_stale.config(text="資料逾時 (Stale)", fg="#ff3366")
        else:
            self.lbl_gps_status_stale.config(text="即時更新 (Active)", fg="#00e676")
            
        # 4. 解析成功語句數
        self.lbl_gps_status_ok.config(text=str(info["ok"]))
        
        # 5. 解析失敗語句數
        err_fg = "#ffffff" if info["err"] == 0 else "#ff3366"
        self.lbl_gps_status_err.config(text=str(info["err"]), fg=err_fg)
        
        # 6. 更新頻率
        self.lbl_gps_status_rate.config(text=f"{info['rate']:.2f} Hz")

    @staticmethod
    def _latlon_to_en(lat, lon, lat0, lon0):
        """經緯度 → 相對 Home 的 East/North 公尺（等距圓柱近似，短距離足夠）"""
        k = 111320.0
        return (lon - lon0) * k * np.cos(np.radians(lat0)), (lat - lat0) * k

    def update_map(self):
        # 資訊列
        if self.gps_last:
            g = self.gps_last
            info = (f"🚀 lat={g['lat']:+.6f}  lon={g['lon']:+.6f}  alt={g.get('alt', 0)} m  "
                    f"spd={g.get('spd', 0.0):.1f} m/s  sats={g.get('sats', '?')}")
            if self.map_home:
                e, n = self._latlon_to_en(g['lat'], g['lon'], self.map_home[0], self.map_home[1])
                info += f"  距Home {float(np.hypot(e, n)):.0f} m"
            self.lbl_gps_info.config(text=info, fg="#00e676")
        elif self.gs_own_pos:
            self.lbl_gps_info.config(
                text=f"📡 地面站 lat={self.gs_own_pos[0]:+.6f} lon={self.gs_own_pos[1]:+.6f}（等待火箭 GPS…）",
                fg="#ffcc00")

        if HAVE_MAPVIEW:
            if self.gps_track:
                lat, lon = self.gps_track[-1]
                if self.map_marker is None:
                    self.map_marker = self.map_widget.set_marker(lat, lon, text="🚀")
                else:
                    self.map_marker.set_position(lat, lon)
                if len(self.gps_track) >= 2:
                    pts = self.gps_track[-600:]
                    try:
                        if self.map_path is None:
                            self.map_path = self.map_widget.set_path(pts)
                        else:
                            self.map_path.set_position_list(pts)
                    except Exception:   # 版本差異：退回重建路徑
                        try:
                            if self.map_path:
                                self.map_path.delete()
                        except Exception:
                            pass
                        self.map_path = self.map_widget.set_path(pts)
                if not self.map_zoomed:
                    self.map_widget.set_zoom(16)
                    self.map_zoomed = True
                if self.map_follow_var.get():
                    self.map_widget.set_position(lat, lon)
            if self.gs_own_pos:
                if self.gs_marker is None:
                    self.gs_marker = self.map_widget.set_marker(
                        self.gs_own_pos[0], self.gs_own_pos[1], text="📡GS")
                else:
                    self.gs_marker.set_position(self.gs_own_pos[0], self.gs_own_pos[1])
                if not self.gps_track and not self.map_zoomed:
                    self.map_widget.set_position(self.gs_own_pos[0], self.gs_own_pos[1])
                    self.map_widget.set_zoom(15)
                    self.map_zoomed = True
        else:
            if self.gps_track and self.map_home:
                lat0, lon0 = self.map_home
                arr = np.array(self.gps_track, dtype=float)
                e, n = self._latlon_to_en(arr[:, 0], arr[:, 1], lat0, lon0)
                self.trk_line.set_data(e, n)
                self.trk_pt.set_data([e[-1]], [n[-1]])
                self.trk_home.set_data([0.0], [0.0])
                self.trk_ax.relim()
                self.trk_ax.autoscale_view()
                self.trk_canvas.draw_idle()

    def map_redraw_loop(self):
        try:
            if not self.root.winfo_exists():
                return
            if self.map_dirty:
                self.update_map()
                self.map_dirty = False
            self.root.after(1000, self.map_redraw_loop)
        except tk.TclError:
            pass

    def on_gps_fix(self, lat, lon, alt_m=None, spd_ms=None, sats=None):
        """收到一筆有效火箭定位（來自 [GPS] 或 [GS_PKT] pos:）"""
        if self.map_home is None:
            self.map_home = (lat, lon)
        # 去抖：與上一點相同就不重複入軌跡
        if not self.gps_track or self.gps_track[-1] != (lat, lon):
            self.gps_track.append((lat, lon))
            if len(self.gps_track) > 5000:
                self.gps_track = self.gps_track[-4000:]
        g = self.gps_last or {}
        g.update({"lat": lat, "lon": lon})
        if alt_m is not None:
            g["alt"] = alt_m
        if spd_ms is not None:
            g["spd"] = spd_ms
        if sats is not None:
            g["sats"] = sats
        self.gps_last = g
        self.map_dirty = True

    # ------------------ 連線控制 ------------------
    def toggle_connection(self):
        if not self.running:
            # 建立連線
            port = self.port_combo.get().strip()
            try:
                baud = int(self.baud_combo.get())
            except ValueError:
                baud = DEFAULT_BAUD
                
            try:
                self.ser = serial_link.open_serial(port, baud, timeout=0.5)
            except Exception as e:
                messagebox.showerror("串口連線失敗", f"無法開啟 {port}，請檢查硬體接線！\n錯誤: {e}")
                return

            # 記住這次連的 port/baud，供斷線自動重連使用（板子重置＝USB-CDC 整個消失
            # 再重新列舉，見 serial_read_task 例外處理）。
            self.connect_port = port
            self.connect_baud = baud

            # 打開記錄檔
            if self.save_log_var.get():
                now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
                self.log_file = open(os.path.join(log_dir, f"gui_serial_continuous_{now_str}.log"), "w", encoding="utf-8")
                self.log_file.write(f"--- GUI CONTINUOUS MONITOR SESSION START AT {datetime.now()} ---\n")
                self.log_file.flush()
                
            self.running = True
            self.clear_console()
            self.clear_rate_cards()
            self.clear_chart_data()
            self.clear_gps_track()
            
            # 建立串列背景執行緒
            self.serial_thread = threading.Thread(target=self.serial_read_task, daemon=True)
            self.serial_thread.start()
            
            self.btn_connect.config(text="中斷連線", style="Disconnect.TButton")
            self.console.insert(tk.END, f"[SYSTEM] 🟢 已成功連接 {port} @ {baud} baud\n")

            # 角色偵測：重置後主動送 'role' 查詢（1.5s 後，讓開機/雜訊先過）
            self.set_role(None)
            self.role_query_attempts = 0
            self.root.after(1500, self.query_role)
        else:
            # 中斷連線
            self.running = False
            if hasattr(self, 'ser') and self.ser:
                try:
                    self.ser.close()
                except:
                    pass
            if self.log_file:
                try:
                    self.log_file.close()
                except:
                    pass
                self.log_file = None
                
            self.btn_connect.config(text="連接 GPS / 航電", style="TButton")
            self.console.insert(tk.END, "[SYSTEM] 🔴 串口已關閉，連線中止。\n")
            self.set_role(None)

    # ------------------ 背景執行緒：高速串口讀取 ------------------
    def serial_read_task(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    raw = self.ser.readline()
                    if not raw:
                        continue
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    # 放入佇列傳送至主執行緒
                    self.data_queue.put((ts, line))

                    # 同步寫入日誌
                    if self.log_file:
                        self.log_file.write(f"[{ts}] {line}\n")
                        self.log_file.flush()
                else:
                    time.sleep(0.002) # 減少 CPU 飆升
            except Exception as e:
                if not self.running:
                    break
                # 串口中斷不視為致命錯誤直接停止監控：MCU 重置時 USB-CDC 會整個消失、
                # 重新列舉成新裝置，若在此停手要求使用者自己按「連接」，等使用者反應過來
                # 開機/角色判斷等最早期的 log 早已錯過（USB best-effort、PC 沒接就丟，
                # 韌體端也不落地）。故改為原地快速輪詢重開同一個 port，搶開機最早窗口。
                self.data_queue.put(("SYS", f"[SYSTEM] ⚠️ 串口中斷（{e}），偵測是否為板子重置並自動重連…"))
                try:
                    self.ser.close()
                except Exception:
                    pass
                self._serial_reconnect_loop()
                # running 可能在等待期間被使用者按「中斷連線」清掉，迴圈條件會自然跳出

        # 清除連線狀態
        try:
            if self.root.winfo_exists():
                self.root.after(0, self.update_disconnect_ui)
        except tk.TclError:
            pass

    def _serial_reconnect_loop(self):
        """MCU 重置＝USB-CDC 整個消失再重新列舉；50ms 高頻輪詢重開同一個 port，
        搶開機最早期窗口（愈快接上，愈不會錯過 [BOOT]/角色判斷等只印一次的訊息）。
        直接呼叫 serial.Serial 而非 serial_link.open_serial：後者每次失敗都會 print，
        高頻輪詢會洗版終端機。"""
        t0 = time.monotonic()
        warned = False
        while self.running:
            try:
                ser = serial.Serial(self.connect_port, self.connect_baud, timeout=0.5)
                try:
                    ser.dtr = False
                    ser.rts = False
                except Exception:
                    pass
                ser.reset_input_buffer()
                self.ser = ser
                self.data_queue.put(("SYS", f"[SYSTEM] 🟢 已自動重新連接 {self.connect_port}"))
                return
            except Exception:
                if not warned and (time.monotonic() - t0) > 5.0:
                    self.data_queue.put(("SYS", f"[SYSTEM] ⏳ 仍在嘗試自動重連 {self.connect_port}…"))
                    warned = True
                time.sleep(0.05)

    def update_disconnect_ui(self):
        try:
            if hasattr(self, 'btn_connect') and self.btn_connect.winfo_exists():
                self.btn_connect.config(text="連接 GPS / 航電", style="TButton")
        except tk.TclError:
            pass
        if self.log_file:
            try: self.log_file.close()
            except: pass
            self.log_file = None

    # ------------------ 主執行緒：定時處理佇列 ------------------
    def poll_queue(self):
        # 批次處理佇列中的資料，避免界面阻塞
        max_lines_per_frame = 30
        lines_processed = 0
        
        while not self.data_queue.empty() and lines_processed < max_lines_per_frame:
            ts, line = self.data_queue.get()
            lines_processed += 1
            
            if ts == "ERR":
                self.console.insert(tk.END, f"{line}\n", "err")
                self.console.see(tk.END)
                continue

            if ts == "SYS":
                # 斷線/自動重連提示（不是錯誤，見 serial_read_task/_serial_reconnect_loop）
                self.console.insert(tk.END, f"{line}\n", "ok")
                self.console.see(tk.END)
                continue

            # 輸出滾動字元日誌，高亮重要字眼
            tag = "tele"
            if "[LINK]" in line:
                tag = "link"
            elif "[RATE]" in line:
                tag = "rate"
            elif "[MAG]" in line:
                tag = "mag"
            elif "[GPS]" in line or "[GS_GPS]" in line:
                tag = "gps"
            elif ("[E80]" in line or "[E22]" in line or "[LORA" in line
                  or "[STATS]" in line or "[ROLE_ID]" in line or "[BOOT]" in line):
                tag = "lora"
            elif "ok=" in line or "Ready" in line or "PASS" in line:
                tag = "ok"
            elif "err=" in line and "err:0" not in line and "err:0," not in line:
                tag = "err"
                
            self.console.insert(tk.END, f"[{ts}] {line}\n", tag)
            
            # 定期截斷終端，防記憶體飆升 (上限 2000 行)
            if float(self.console.index('end-1c')) > 2000.0:
                self.console.delete("1.0", "200.0")
                
            self.console.see(tk.END)
            
            # 指令/回應分流 + 解析（單行出錯不得中斷輪詢，否則 poll_queue 停止重排→GUI 凍結）
            try:
                if self._is_event_line(line):
                    self._event_log(f"[{ts}] {line}", self._event_tag(line))
                self.parse_telemetry(line)
            except Exception as e:
                try:
                    self.console.insert(tk.END, f"[GUI] ⚠ 處理行例外: {e}\n", "err")
                    self.console.see(tk.END)
                except Exception:
                    pass
            
        # 繼續定時輪詢
        try:
            if self.root.winfo_exists():
                self.root.after(10, self.poll_queue)
        except tk.TclError:
            pass

    # ------------------ 指令/回應區（只收 CMD 與指令回應） ------------------
    # 白名單：使用者送出的 [CMD] + 韌體對「指令」的回應標籤。開機/角色/系統/遙測一律不進。
    _EVENT_KEEP_TAGS = ("[CMD]", "[CAL]", "[E22]", "[E80]", "[LORA433]")

    def _is_event_line(self, line):
        """只有使用者指令與其回應才進「指令/回應」區。"""
        return bool(line) and any(tag in line for tag in self._EVENT_KEEP_TAGS)

    def _event_tag(self, line):
        if "[CMD]" in line:
            return "cmd"
        if "ERROR" in line or "FAIL" in line or "❌" in line:
            return "err"
        return "resp"

    def _event_log(self, text, tag="resp"):
        """寫入重要訊息區（若已建立）；截斷上限 800 行、自動捲到底。"""
        if not hasattr(self, 'event_console'):
            return
        try:
            self.event_console.insert(tk.END, text + "\n", tag)
            if float(self.event_console.index('end-1c')) > 800.0:
                self.event_console.delete("1.0", "100.0")
            self.event_console.see(tk.END)
        except tk.TclError:
            pass

    def parse_telemetry(self, line):
        """解析串口封包，更新 3D 姿態與頂部狀態數位卡片"""
        # 0. 角色偵測 + LoRa 參數回報（三角色通用）
        self.parse_role_and_lora(line)

        # 0b. 10Hz 裸 CSV 原始遙測行（飛行板）：
        #     bmi_ax,ay,az(mG), adxl_ax,ay,az(mG), temp(x100), press(Pa), baro_alt(cm)
        if line and (line[0] == '-' or line[0].isdigit()) and line.count(',') == 8:
            try:
                v = [int(x) for x in line.split(',')]
            except ValueError:
                v = None
            if v is not None:
                t = self.now_t()
                self.ts_acc_bmi.append((t, float(np.sqrt(v[0]**2 + v[1]**2 + v[2]**2)) / 1000.0))
                self.ts_acc_adxl.append((t, float(np.sqrt(v[3]**2 + v[4]**2 + v[5]**2)) / 1000.0))
                self.ts_alt_baro.append((t, v[8] / 100.0))
                self.chart_dirty = True
                return

        # A. 解析四元數 EKF 姿態封包 [TELE] pos:x,y,z vel:x,y,z q:qw,qx,qy,qz
        if "[TELE]" in line and "q:" in line:
            m = re.search(r"q:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                q = [float(m.group(i)) for i in range(1, 5)]
                self.last_q = q
                if not self.paused:
                    self.update_3d_plot(q)
            # EKF 高度（pos z）與垂直速度（vel z）→ 飛行圖表 + 即時卡片
            t = self.now_t()
            m = re.search(r"pos:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                alt = float(m.group(3))
                self.ts_alt_ekf.append((t, alt))
                self.cards["alt"].config(text=f"{alt:.1f} m")
                self.chart_dirty = True
            m = re.search(r"vel:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                vz = float(m.group(3))
                self.ts_vz_ekf.append((t, vz))
                self.cards["vz"].config(text=f"{vz:+.1f} m/s")
                self.chart_dirty = True
                    
        # B. 解析頻率封包 [RATE] BMI088_A:xxHz, ...
        elif "[RATE]" in line:
            # 提取採樣率數字
            m_a = re.search(r"BMI088_A:([\d\.]+)Hz", line)
            m_g = re.search(r"BMI088_G:([\d\.]+)Hz", line)
            m_x = re.search(r"ADXL375:([\d\.]+)Hz", line)
            m_b = re.search(r"BMP388:([\d\.]+)Hz", line)
            m_m = re.search(r"MMC5983:([\d\.]+)Hz", line)
            m_gps = re.search(r"GPS:([\d\.]+)Hz", line)
            m_drop = re.search(r"EKF_DROP:(\d+)", line)
            
            if m_a: self.cards["bmi_a"].config(text=f"{float(m_a.group(1)):.2f} Hz")
            if m_g: self.cards["bmi_g"].config(text=f"{float(m_g.group(1)):.2f} Hz")
            if m_x: self.cards["adxl"].config(text=f"{float(m_x.group(1)):.2f} Hz")
            if m_b: self.cards["bmp"].config(text=f"{float(m_b.group(1)):.2f} Hz")
            if m_m: self.cards["mag"].config(text=f"{float(m_m.group(1)):.2f} Hz")
            if m_gps:
                gps_rate = float(m_gps.group(1))
                self.cards["gps"].config(text=f"{gps_rate:.2f} Hz")
                self.gps_status_info["rate"] = gps_rate
                self.update_gps_status_panel()
            if m_drop: self.lbl_drops.config(text=f"EKF Queue Drops: {m_drop.group(1)}")
            
        # C. 解析 Flash 封包 [FLASH_RING] PKT_TOTAL:xx ADDR:xx
        elif "[FLASH_RING]" in line and "PKT_TOTAL:" in line:
            m_pkt = re.search(r"PKT_TOTAL:(\d+)", line)
            if m_pkt:
                self.cards["flash_pkt"].config(text=f"{m_pkt.group(1)} Pkts")

        # C2. 主/備板間鏈路溝通狀態 [LINK] self:.. peer:.. link:OK/STALE/NONE state:.. flags:.. age:..ms
        elif "[LINK]" in line:
            m = re.search(
                r"self=(\w+)\s+peer=(\w+)\s+link=(\w+)\s+state=(\w+)\s+flags=0x([0-9A-Fa-f]+)\s+age=(\d+)ms",
                line)
            if m:
                self_role, peer_role, link_ok, peer_state, flags_hex, age_ms = m.groups()
                self.update_link_status(self_role, peer_role, link_ok, peer_state, int(flags_hex, 16), int(age_ms))

        # D. 解析 FSM 狀態轉移以動態更新 HUD
        elif "[FSM]" in line:
            m_state = re.search(r"(STATE_[A-Z_]+)", line)
            if m_state:
                self.fsm_state = m_state.group(1)
            elif "LIFTOFF" in line:
                self.fsm_state = "STATE_BOOST"
            elif "BURNOUT" in line:
                self.fsm_state = "STATE_COAST"
            elif "APOGEE" in line:
                self.fsm_state = "STATE_RECOVERY"
            elif "LANDED" in line:
                self.fsm_state = "STATE_LANDED"
                
        # E. 解析 [IMU]
        elif "[IMU]" in line:
            m = re.search(r"a\[mG\]:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+) g\[dps\]:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                self.latest_imu = {
                    "ax": float(m.group(1)), "ay": float(m.group(2)), "az": float(m.group(3)),
                    "gx": float(m.group(4)), "gy": float(m.group(5)), "gz": float(m.group(6))
                }
                
        # F. 解析 [HIGHG]
        elif "[HIGHG]" in line:
            m = re.search(r"a\[mG\]:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                self.latest_highg = {
                    "ax": float(m.group(1)), "ay": float(m.group(2)), "az": float(m.group(3))
                }
                
        # G. 解析 [MAG]
        elif "[MAG] B[mG]" in line or ("[MAG]" in line and "B[mG]" in line):
            m = re.search(r"B\[mG\]:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)\s+hdg:(-?[\d\.]+)", line)
            if m:
                self.latest_mag = {
                    "mx": float(m.group(1)), "my": float(m.group(2)), "mz": float(m.group(3)),
                    "hdg": float(m.group(4))
                }
                if self.collecting_data:
                    self.calib_x.append(self.latest_mag["mx"])
                    self.calib_y.append(self.latest_mag["my"])

        # H. 解析 [PWR]（飛行板 1Hz 電池電壓）
        elif "[PWR]" in line and "bat:" in line:
            m = re.search(r"bat:(\d+)mV", line)
            if m:
                bat_v = float(m.group(1)) / 1000.0
                bat_color = "#4CAF50" if bat_v >= 7.4 else ("#FFC107" if bat_v >= 6.8 else "#E91E63")
                self.cards["bat"].config(text=f"{bat_v:.2f} V", foreground=bat_color)

        # I. 解析 [GPS]（飛行板 1Hz）：定位 → 地圖軌跡；海拔/地速 → 圖表原始序列
        elif "[GPS]" in line and "fix:" in line:
            m = re.search(r"fix:(\d+) q:\d+ sat:(\d+) ([+-])(\d+\.\d+),([+-])(\d+\.\d+) alt:(-?\d+)m spd:(-?\d+)cm/s(?: stale:(\d+) ok:(\d+) err:(\d+))?", line)
            if m:
                fix_val = int(m.group(1))
                sats_val = int(m.group(2))
                self.gps_status_info["fix"] = fix_val
                self.gps_status_info["sats"] = sats_val
                if m.group(9) is not None:
                    self.gps_status_info["stale"] = int(m.group(9))
                if m.group(10) is not None:
                    self.gps_status_info["ok"] = int(m.group(10))
                if m.group(11) is not None:
                    self.gps_status_info["err"] = int(m.group(11))
                self.update_gps_status_panel()
                
                if fix_val == 1:
                    lat = float(m.group(4)) * (1.0 if m.group(3) == '+' else -1.0)
                    lon = float(m.group(6)) * (1.0 if m.group(5) == '+' else -1.0)
                    if abs(lat) > 0.01 or abs(lon) > 0.01:   # 排除 0,0 假定位
                        alt = int(m.group(7))
                        spd = int(m.group(8)) / 100.0
                        t = self.now_t()
                        self.ts_alt_gps.append((t, float(alt)))
                        self.ts_spd_gps.append((t, spd))
                        self.on_gps_fix(lat, lon, alt_m=alt, spd_ms=spd, sats=sats_val)
                        self.chart_dirty = True

        # J. 解析 [GS_PKT]（地面站收到的火箭下行封包摘要）→ 圖表 + 地圖
        elif "[GS_PKT]" in line:
            t = self.now_t()
            m = re.search(r"alt:(-?\d+)cm", line)
            if m:
                alt = int(m.group(1)) / 100.0
                self.ts_alt_ekf.append((t, alt))
                self.cards["alt"].config(text=f"{alt:.1f} m")
            m = re.search(r"vz:(-?\d+)cms", line)
            if m:
                vz = int(m.group(1)) / 100.0
                self.ts_vz_ekf.append((t, vz))
                self.cards["vz"].config(text=f"{vz:+.1f} m/s")
            m = re.search(r"baro:(-?\d+)cm", line)
            if m:
                self.ts_alt_baro.append((t, int(m.group(1)) / 100.0))
            m = re.search(r"accel:(-?\d+),(-?\d+),(-?\d+)", line)
            if m:
                ax_, ay_, az_ = (int(m.group(i)) for i in range(1, 4))
                self.ts_acc_bmi.append((t, float(np.sqrt(ax_**2 + ay_**2 + az_**2)) / 1000.0))
            m = re.search(r"gps:(\d+)/(\d+)", line)
            sats = int(m.group(1)) if m else None
            fix = (m and m.group(2) == "1")
            if sats is not None:
                self.gps_status_info["sats"] = sats
                self.gps_status_info["fix"] = 1 if fix else 0
                self.update_gps_status_panel()
            m = re.search(r"pos:([+-])(\d+\.\d+),([+-])(\d+\.\d+)", line)
            if fix and m:
                lat = float(m.group(2)) * (1.0 if m.group(1) == '+' else -1.0)
                lon = float(m.group(4)) * (1.0 if m.group(3) == '+' else -1.0)
                if abs(lat) > 0.01 or abs(lon) > 0.01:
                    ma = re.search(r"galt:(-?\d+)m", line)
                    galt = int(ma.group(1)) if ma else None
                    if galt is not None:
                        self.ts_alt_gps.append((t, float(galt)))
                    self.on_gps_fix(lat, lon, alt_m=galt, sats=sats)
            self.chart_dirty = True

        # K. 解析 [GS_GPS]（地面站自身定位）→ 地圖 GS 標記
        elif "[GS_GPS]" in line and "FIX" in line:
            m = re.search(r"Pos:([+-])(\d+\.\d+),([+-])(\d+\.\d+)", line)
            if m:
                lat = float(m.group(2)) * (1.0 if m.group(1) == '+' else -1.0)
                lon = float(m.group(4)) * (1.0 if m.group(3) == '+' else -1.0)
                if abs(lat) > 0.01 or abs(lon) > 0.01:
                    self.gs_own_pos = (lat, lon)
                    self.map_dirty = True

        # H. 解析 MMC5983 初始或儲存的偏移量
        if "[MAG] MMC5983MA online. offset[X,Y,Z]=" in line:
            m = re.search(r"offset\[X,Y,Z\]=(-?\d+),(-?\d+),(-?\d+)", line)
            if m:
                self.board_mag_offsets = [float(m.group(1)), float(m.group(2)), float(m.group(3))]
        elif "[CAL] Mag hard-iron offset saved to Flash" in line:
            # 韌體格式（ekf.c:1303）：
            #   [CAL] Mag hard-iron offset saved to Flash (x10): <x*10>, <y*10>, <z*10>
            # 值是實際 counts 的 10 倍（整數傳輸保一位小數），需 ÷10 還原。
            m = re.search(r"saved to Flash \(x10\):\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)", line)
            if m:
                self.board_mag_offsets = [int(m.group(1)) / 10.0,
                                          int(m.group(2)) / 10.0,
                                          int(m.group(3)) / 10.0]
                self._on_mag_write_confirmed()
        elif "[CAL] EKF Mag Yaw Lock set to:" in line:
            m = re.search(r"Mag Yaw Lock set to:\s*(\d+)", line)
            if m:
                self._on_yaw_lock_confirmed(int(m.group(1)) != 0)
        elif "[CAL] ERROR" in line:
            self._on_mag_write_failed(line)

    # ------------------ 3D 繪圖更新 ------------------
    def update_3d_plot(self, q):
        """依據四元數旋轉 3D 火箭模型，繪製地平面與投影輔助線，並更新 HUD 歐拉角與姿態資訊"""
        # 1. 計算旋轉矩陣 R 與歐拉角（直接使用 EKF 輸出的四元數，body→nav ENU）
        # ZYX 分解中「roll」= 繞 body X(右軸) = 火箭 PITCH（前後仰俯）
        #                「pitch」= 繞 body Y(前軸) = 火箭 ROLL（左右翻滾）
        #                「yaw」  = 繞 body Z(上軸) = 航向
        R = quaternion_to_matrix(q)
        zyx_roll, zyx_pitch, yaw_corr = quaternion_to_euler(q)
        pitch_corr = zyx_roll   # 繞 X 軸 = 俯仰
        roll_corr  = zyx_pitch  # 繞 Y 軸 = 翻滾
        
        # 讀取局部座標點
        (xb1, yb1, zb1, xb2, yb2, zb2, xb3, yb3, zb3,
         xn, yn, zn, xnoz, ynoz, znoz, xfl, yfl, zfl, fins) = self.local_geom
         
        h_body = 1.8
        h_nose = 0.8
        
        # 計算 3D 空間中旋轉後的火箭各網格段點
        rxb1, ryb1, rzb1 = rotate_points(xb1, yb1, zb1, R)
        rxb2, ryb2, rzb2 = rotate_points(xb2, yb2, zb2, R)
        rxb3, ryb3, rzb3 = rotate_points(xb3, yb3, zb3, R)
        rxn, ryn, rzn = rotate_points(xn, yn, zn, R)
        rxnoz, rynoz, rznoz = rotate_points(xnoz, ynoz, znoz, R)
        
        # 引擎火光隨機抖動 (Flicker)
        flicker_factor = random.uniform(0.75, 1.25)
        
        # 外部大火焰 (橘紅色)
        zfl_outer = zfl * flicker_factor + (-h_body/2 - 0.2)
        rxfl, ryfl, rzfl = rotate_points(xfl, yfl, zfl_outer, R)
        
        # 內部核心小火焰 (黃色)
        zfl_inner = zfl * 0.65 * flicker_factor + (-h_body/2 - 0.2)
        rxfl_inner, ryfl_inner, rzfl_inner = rotate_points(xfl * 0.55, yfl * 0.55, zfl_inner, R)
        
        # 清除前一影格的 3D 繪圖，保留滑鼠拖曳視角
        elev, azim = self.ax.elev, self.ax.azim
        self.ax.clear()
        self.ax.set_facecolor("#101010") # 深邃太空黑背景
        self.ax.view_init(elev=elev, azim=azim)
        
        z_ground = -1.2
        
        # 1. 繪製圓形發射台平面 (Launchpad Disk)
        r_vals = np.linspace(0, 2.0, 15)
        theta_vals = np.linspace(0, 2*np.pi, 48)
        R_grid, T_grid = np.meshgrid(r_vals, theta_vals)
        xg = R_grid * np.cos(T_grid)
        yg = R_grid * np.sin(T_grid)
        zg = np.full_like(xg, z_ground)
        
        # 繪製暗碳灰色、帶有細緻格線的發射底座面
        self.ax.plot_surface(xg, yg, zg, color="#161616", alpha=0.6, edgecolor='#2c2c2c', linewidth=0.4, shade=False)
        
        # 繪製發射台霓虹邊緣外圈 (螢光青綠)
        cx = 2.0 * np.cos(theta_vals)
        cy = 2.0 * np.sin(theta_vals)
        cz = np.full_like(cx, z_ground)
        self.ax.plot(cx, cy, cz, color="#00ffcc", linestyle="-", linewidth=1.5, alpha=0.7)
        
        # 繪製內部同心雷達圈
        for r_c in [0.7, 1.4]:
            cx_c = r_c * np.cos(theta_vals)
            cy_c = r_c * np.sin(np.linspace(0, 2*np.pi, 48))
            cz_c = np.full_like(cx_c, z_ground)
            self.ax.plot(cx_c, cy_c, cz_c, color="#3e3e3e", linestyle="--", linewidth=0.8, alpha=0.5)
            
        # 繪製方位十字參考射線 (N E S W)
        for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
            rad = np.radians(angle)
            self.ax.plot([0, 2.0 * np.cos(rad)], [0, 2.0 * np.sin(rad)], [z_ground, z_ground], color="#333333", linestyle=":", linewidth=0.6)
            
        # 方位標籤
        self.ax.text(2.2, 0, z_ground, "E (90°)", color="#00ffcc", fontsize=8, fontweight="bold", ha="left", va="center")
        self.ax.text(-2.2, 0, z_ground, "W (270°)", color="#00ffcc", fontsize=8, fontweight="bold", ha="right", va="center")
        self.ax.text(0, 2.2, z_ground, "N (0°)", color="#00ffcc", fontsize=8, fontweight="bold", ha="center", va="bottom")
        self.ax.text(0, -2.2, z_ground, "S (180°)", color="#00ffcc", fontsize=8, fontweight="bold", ha="center", va="top")
        
        # 2. 繪製水平面參考系 (Z = 0.0 橫切面半透明全息環)
        th_h = np.linspace(0, 2*np.pi, 60)
        hx = 0.65 * np.cos(th_h)
        hy = 0.65 * np.sin(th_h)
        hz = np.zeros_like(th_h)
        self.ax.plot_surface(np.array([np.zeros_like(th_h), hx]), 
                             np.array([np.zeros_like(th_h), hy]), 
                             np.array([np.zeros_like(th_h), hz]), 
                             color="#00e5ff", alpha=0.12, shade=False)
        self.ax.plot(hx, hy, hz, color="#00e5ff", linestyle="-", linewidth=0.8, alpha=0.4)
        self.ax.plot([-0.65, 0.65], [0, 0], [0, 0], color="#00e5ff", linestyle=":", linewidth=0.5, alpha=0.4)
        self.ax.plot([0, 0], [-0.65, 0.65], [0, 0], color="#00e5ff", linestyle=":", linewidth=0.5, alpha=0.4)
        
        # 3. 計算火箭底部與頂部在 3D 空間中的旋轉位置
        p_base = R @ np.array([0, 0, -h_body/2])
        p_tip = R @ np.array([0, 0, h_body/2 + h_nose])
        
        # 4. 繪製天頂重力參考向量 (Zenith Vector - 螢光綠 arrow 指向正上方)
        self.ax.quiver(0, 0, 0, 0, 0, 1.2, color='#00e676', linewidth=1.5, arrow_length_ratio=0.15, alpha=0.6)
        self.ax.text(0, 0, 1.35, "ZENITH", color="#00e676", fontsize=7, fontweight="bold", ha="center")
        
        # 5. 繪製火箭鼻錐指向軸心向量 (Heading Vector - 鮮紅 arrow)
        dir_z = R[:, 2]  # 火箭 Z 軸在世界空間的向量
        self.ax.quiver(p_tip[0], p_tip[1], p_tip[2], dir_z[0]*0.5, dir_z[1]*0.5, dir_z[2]*0.5, color='#ff1744', linewidth=2.0, arrow_length_ratio=0.3, alpha=0.9)
        
        # 6. 繪製火箭對地平面的垂直輔助投影線 (垂足投影 guide)
        self.ax.plot([p_base[0], p_base[0]], [p_base[1], p_base[1]], [p_base[2], z_ground], color="#00e5ff", linestyle="--", linewidth=1.0, alpha=0.6)
        self.ax.plot([p_tip[0], p_tip[0]], [p_tip[1], p_tip[1]], [p_tip[2], z_ground], color="#ff1744", linestyle="--", linewidth=1.0, alpha=0.6)
        self.ax.plot([0, 0], [0, 0], [0, z_ground], color="#ffcc00", linestyle="-.", linewidth=0.8, alpha=0.4)
        
        # 7. 繪製地平面投影陰影向量 (Quiver Arrow 表示方位偏角)
        self.ax.quiver(p_base[0], p_base[1], z_ground, 
                       p_tip[0] - p_base[0], p_tip[1] - p_base[1], 0, 
                       color="#00e5ff", alpha=0.35, arrow_length_ratio=0.15, linewidth=2.5)
                       
        # 8. 繪製火箭主體曲面 (利用 plot_surface 本身陰影呈現 3D 質感)
        # 引擎噴嘴 (碳黑)
        self.ax.plot_surface(rxnoz, rynoz, rznoz, color="#212121", alpha=0.9, edgecolor='none', shade=True)
        # 下段船身 (鈦白)
        self.ax.plot_surface(rxb1, ryb1, rzb1, color="#eceff1", alpha=0.9, edgecolor='none', shade=True)
        # 中段條紋 (霓虹藍)
        self.ax.plot_surface(rxb2, ryb2, rzb2, color="#00e5ff", alpha=0.9, edgecolor='none', shade=True)
        # 上段船身 (鈦白)
        self.ax.plot_surface(rxb3, ryb3, rzb3, color="#eceff1", alpha=0.9, edgecolor='none', shade=True)
        # 彈頭 (紅色)
        self.ax.plot_surface(rxn, ryn, rzn, color="#ff1744", alpha=0.95, edgecolor='none', shade=True)
        
        # 9. 引擎火焰渲染 (雙層漸變噴射)
        self.ax.plot_surface(rxfl, ryfl, rzfl, color="#ff5722", alpha=0.4, edgecolor='none', shade=True)
        self.ax.plot_surface(rxfl_inner, ryfl_inner, rzfl_inner, color="#ffeb3b", alpha=0.7, edgecolor='none', shade=True)
        
        # 10. 繪製 3D 尾翼 (使用 Poly3DCollection 具備 solid 填充與 shading 特色)
        for f in fins:
            rf = f @ R.T
            poly = Poly3DCollection([rf], facecolors='#ffcc00', edgecolors='#e65100', linewidths=1.0, alpha=0.9, shade=True)
            self.ax.add_collection3d(poly)
            
        # 11. 設定 3D 限制
        self.ax.set_xlim([-1.8, 1.8])
        self.ax.set_ylim([-1.8, 1.8])
        self.ax.set_zlim([-1.5, 2.2]) # Z 的上限需要容納傾斜後的火箭鼻錐
        
        # 關閉原生的灰盒背景與格線，呈現極簡懸浮全息圖感
        self.ax.axis('off')
        
        # 12. 計算與垂直天頂的絕對偏角 (Tilt Angle from Vertical)
        tilt_cos = np.clip(dir_z[2], -1.0, 1.0)
        tilt_deg = np.degrees(np.arccos(tilt_cos))
        
        # 將 Yaw 顯示角度維持在 0~360 度範圍內
        yaw_disp = yaw_corr % 360.0
        
        # 13. 繪製 HUD 面板文字 (疊加於 3D 左上角 - 全息透明玻璃面板風)
        hud_text = (
            f"STATE: {self.fsm_state}\n"
            f"TILT:  {tilt_deg:.1f}°\n"
            f"PITCH: {pitch_corr:+.1f}°\n"
            f"YAW:   {yaw_disp:.1f}°\n"
            f"ROLL:  {roll_corr:+.1f}°"
        )
        self.ax.text2D(0.05, 0.95, hud_text, transform=self.ax.transAxes, color="#00e5ff", fontsize=10, fontweight="bold", fontname="Monaco", bbox=dict(facecolor="#0a0a0a", alpha=0.8, edgecolor="#00e5ff", boxstyle="round,pad=0.6", linewidth=1.2))
        
        # 更新畫布
        self.canvas.draw_idle()

    # ------------------ 視窗生命週期 ------------------
    def on_close(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser:
            try: self.ser.close()
            except: pass
        if self.log_file:
            try: self.log_file.close()
            except: pass
        # 關閉測試精靈視窗（若開啟）
        if hasattr(self, 'wizard_win') and self.wizard_win and self.wizard_win.winfo_exists():
            try: self.wizard_win.destroy()
            except: pass
        # 關閉 LoRa 參數面板（若開啟）
        if hasattr(self, 'lora_win') and self.lora_win and self.lora_win.winfo_exists():
            try: self.lora_win.destroy()
            except: pass
        self.root.destroy()
        
    # 板端命令台是 osPriorityLow 的 ~20ms 輪詢 + 硬體僅 1 byte 緩衝：整串 burst 送會
    # 掉字元（見 mag_calibrate.py 實測，30ms/字仍掉、100ms/字可靠）→ 命令組不起來、
    # 板子毫無回應。故改用背景執行緒逐字慢送，不阻塞 GUI 主迴圈。
    CMD_CHAR_GAP = 0.1   # 秒/字

    def send_command(self, cmd_str):
        if not self.running or not hasattr(self, 'ser') or not self.ser:
            messagebox.showwarning("警告", "串口未連接！請先連線。")
            return False
        if not cmd_str.endswith('\n'):
            cmd_str += '\n'
        # 前置換行：先終止板端命令緩衝 g_cmd_buf 內殘留的半截命令（來自過去 burst 送掉字
        # 的殘骸），讓本命令從乾淨狀態開始組裝，避免「垃圾+命令」黏在一起解析失敗。
        cmd_str = '\n' + cmd_str
        # 逐字慢送（背景執行緒）：避免板端命令台掉字元。
        if not hasattr(self, '_tx_queue'):
            self._tx_queue = queue.Queue()
        if getattr(self, '_tx_thread', None) is None or not self._tx_thread.is_alive():
            self._tx_thread = threading.Thread(target=self._tx_worker, daemon=True)
            self._tx_thread.start()
        self._tx_queue.put(cmd_str)
        # 本地回顯（指令/回應區 + 完整終端）立即顯示；實際位元由背景慢送（數秒）。
        self.console.insert(tk.END, f"[CMD] ➡️ {cmd_str.strip()}（背景慢送中…）\n", "rate")
        self.console.see(tk.END)
        self._event_log(f"[CMD] ➡️ {cmd_str.strip()}", "cmd")
        return True

    def _tx_worker(self):
        """背景逐字慢送佇列中的命令（CMD_CHAR_GAP 秒/字），遷就板端低優先權命令輪詢。"""
        while True:
            cmd = self._tx_queue.get()
            if cmd is None:
                return
            data = cmd.encode('utf-8')
            for i in range(len(data)):
                if not self.running or not getattr(self, 'ser', None):
                    break
                try:
                    self.ser.write(data[i:i + 1])
                    self.ser.flush()
                except Exception:
                    break
                time.sleep(self.CMD_CHAR_GAP)

    # ------------------ 手動指令列 ------------------
    def _on_manual_cmd(self, event=None):
        """送出手動輸入的原始指令；回應會隨遙測串流進終端。"""
        cmd = self.cmd_entry.get().strip()
        if not cmd:
            return "break"
        if self.send_command(cmd):
            if not self._cmd_history or self._cmd_history[-1] != cmd:
                self._cmd_history.append(cmd)
            self._cmd_history_idx = len(self._cmd_history)
            self.cmd_entry.delete(0, tk.END)
        return "break"

    def _manual_cmd_history_prev(self, event=None):
        """↑：回上一條送過的指令。"""
        if not self._cmd_history:
            return "break"
        self._cmd_history_idx = max(0, self._cmd_history_idx - 1)
        self.cmd_entry.delete(0, tk.END)
        self.cmd_entry.insert(0, self._cmd_history[self._cmd_history_idx])
        return "break"

    def _manual_cmd_history_next(self, event=None):
        """↓：往下一條；到底則清空輸入框。"""
        if not self._cmd_history:
            return "break"
        self._cmd_history_idx = min(len(self._cmd_history), self._cmd_history_idx + 1)
        self.cmd_entry.delete(0, tk.END)
        if self._cmd_history_idx < len(self._cmd_history):
            self.cmd_entry.insert(0, self._cmd_history[self._cmd_history_idx])
        return "break"

    # ==================== 角色自動偵測（主航電/備援航電/地面站） ====================
    ROLE_STYLES = {
        "PRIMARY": ("🚀 主航電 PRIMARY", "#00e676"),
        "BACKUP":  ("🛟 備援航電 BACKUP", "#ffcc00"),
        "GROUND":  ("📡 地面站 GROUND", "#e07bfb"),
        None:      ("⚫ 角色偵測中…", "#777777"),
    }

    def set_role(self, role, fw=""):
        """更新偵測到的角色徽章；role=None 表示未知/重置。"""
        if fw:
            self.detected_fw = fw
        changed = (role != self.detected_role)
        self.detected_role = role
        text, color = self.ROLE_STYLES.get(role, self.ROLE_STYLES[None])
        if role is None:
            self.detected_fw = ""
            if not self.running:
                text = "⚫ 未連線"
        elif self.detected_fw:
            text += f"  ({self.detected_fw})"
        self.lbl_role.config(text=text, fg=color)
        if changed and role:
            self.console.insert(tk.END, f"[SYSTEM] ✅ 偵測到裝置角色: {text}\n", "ok")
            self.console.see(tk.END)
        if role is None:
            self.lbl_link.config(text="🔗 --", fg="#555555")
            self.link_last_age_ms = None
        # 同步 LoRa 面板（若已開啟）
        if hasattr(self, 'lora_win') and self.lora_win and self.lora_win.winfo_exists():
            self.refresh_lora_panel_role()

    _LINK_PEER_LABEL = {"PRIMARY": "主航電", "BACKUP": "備航電", "NONE": "無"}

    def update_link_status(self, self_role, peer_role, link_ok, peer_state, flags, age_ms):
        """更新頂部鏈路徽章：本板↔對端的溝通狀態（來自 [LINK] 診斷行，1Hz）。
        link_ok: OK=收得到且新鮮 / STALE=曾收過但逾時 / NONE=從未收過對端封包。"""
        self.link_last_age_ms = age_ms
        peer_label = self._LINK_PEER_LABEL.get(peer_role, peer_role)
        if link_ok == "OK":
            text = f"🔗 {peer_label} OK ({peer_state}, {age_ms}ms)"
            color = "#00e676"
        elif link_ok == "STALE":
            text = f"🔗 {peer_label} 失聯 ({age_ms}ms)"
            color = "#ffcc00"
        else:
            text = f"🔗 {peer_label}（尚無回應）"
            color = "#ff3366"
        self.lbl_link.config(text=text, fg=color)

    def query_role(self):
        """主動送 'role' 命令查詢角色（三種角色 firmware 都會回 [ROLE_ID]）。
        最多重試 5 次；期間若被動解析（[BOOT]/[GS_*] 等）已判定則停止。"""
        if not self.running or self.detected_role is not None:
            return
        if self.role_query_attempts >= 5:
            self.console.insert(tk.END, "[SYSTEM] ⚠️ 角色查詢無回應（舊版 firmware 無 'role' 命令？"
                                        "仍可由 [BOOT]/遙測特徵行被動判定）\n", "err")
            self.console.see(tk.END)
            return
        self.role_query_attempts += 1
        try:
            self.ser.write(b"role\n")
            self.ser.flush()
        except Exception:
            return
        self.root.after(2500, self.query_role)

    def parse_role_and_lora(self, line):
        """解析角色特徵行與 LoRa 參數回報行（三角色輸出格式已於 firmware 端統一）。"""
        # --- 角色偵測（未判定時全面掃描；[ROLE_ID]/[BOOT] 永遠重判，支援重刷角色後熱插拔） ---
        if self.detected_role is None or "[ROLE_ID]" in line or "[BOOT]" in line:
            m = re.search(r"\[ROLE_ID\]\s+role=(PRIMARY|BACKUP|GROUND)(?:\s+fw=(\S+))?", line)
            if m:
                self.set_role(m.group(1), m.group(2) or "")
            else:
                m = re.search(r"\[BOOT\].*ROLE=(PRIMARY|BACKUP|GROUND)", line)
                if m:
                    mf = re.search(r"FW=(\S+)", line)
                    self.set_role(m.group(1), mf.group(1) if mf else "")
                elif re.search(r"\[ROLE\]\s+PRIMARY_AV", line):
                    self.set_role("PRIMARY")
                elif re.search(r"\[ROLE\]\s+BACKUP_AV", line):
                    self.set_role("BACKUP")
                elif ("LoRa 通訊測試模組就緒" in line or line.startswith("[GS_")
                      or "[GS_PKT]" in line or "[GS_STAT]" in line or "[GS_LORA_INIT]" in line):
                    self.set_role("GROUND")

        # --- E80 920MHz 參數回報（gs / 航電命令台同格式） ---
        m = re.search(r"\[E80\] freq=(\d+) Hz\s+SF(\d+)\s+BW([\d\.]+) kHz \(idx=(\d+)\)\s+"
                      r"CR 4/(\d)\s+pwr=(-?\d+) dBm\s+pre=(\d+)", line)
        if m:
            self.lora_e80_state = {
                "freq_hz": int(m.group(1)), "sf": int(m.group(2)),
                "bw_khz": float(m.group(3)), "bw_idx": int(m.group(4)),
                "cr": int(m.group(5)) - 4,   # 顯示為 4/5..4/8，換算回 firmware cr 值 1..4
                "pwr": int(m.group(6)), "pre": int(m.group(7)),
            }
            self.refresh_lora_panel_values()
            return

        # E80 開機組態行 [LORA 920MHz] ... Freq: 920.000 MHz | Power: +22dBm | BW: 250kHz | SF: 9 ...
        m = re.search(r"\[LORA 920MHz\].*Freq:\s*(\d+)\.(\d+)\s*MHz.*Power:\s*\+?(-?\d+)dBm.*"
                      r"BW:\s*(\d+)kHz.*SF:\s*(\d+)", line)
        if m:
            self.lora_e80_state.update({
                "freq_hz": int(m.group(1)) * 1000000 + int(m.group(2)) * 1000,
                "pwr": int(m.group(3)), "bw_khz": float(m.group(4)), "sf": int(m.group(5)),
            })
            self.refresh_lora_panel_values()
            return

        # --- E22 433MHz 參數回報 ---
        # 地面站格式: [E22] freq=432 MHz  CH=22
        m = re.search(r"\[E22\] freq=(\d+) MHz\s+CH=(\d+)", line)
        if m:
            self.lora_e22_state.update({"freq_mhz": int(m.group(1)), "ch": int(m.group(2))})
            self.refresh_lora_panel_values()
            return

        # 設定成功回報: [E22] freq set 435 MHz (CH=25) OK
        m = re.search(r"\[E22\] freq set (\d+) MHz \(CH=(\d+)\) OK", line)
        if m:
            self.lora_e22_state.update({"freq_mhz": int(m.group(1)), "ch": int(m.group(2))})
            self.refresh_lora_panel_values()
            return

        # 模組暫存器回讀: [LORA433] E22-400T30S | Freq=432.000MHz(CH=22) | Power=21dBm | AirRate=2.4k | ...
        m = re.search(r"\[LORA433\] E22-400T30S \| Freq=(\d+)\.000MHz\(CH=(\d+)\)"
                      r"(?: \| Power=(\S+) \| AirRate=(\S+))?", line)
        if m:
            self.lora_e22_state.update({"freq_mhz": int(m.group(1)), "ch": int(m.group(2))})
            if m.group(3):
                self.lora_e22_state["power"] = m.group(3)
            if m.group(4):
                self.lora_e22_state["air_rate"] = m.group(4)
            self.refresh_lora_panel_values()
            return

        # E22 功率設定成功回報: [E22] pwr set level 3 OK （韌體只回等級，換算成 dBm 顯示）
        m = re.search(r"\[E22\] pwr set level (\d+) OK", line)
        if m:
            lvl = int(m.group(1))
            if 0 <= lvl < len(self.E22_PWR_LEVELS):
                self.lora_e22_state["power"] = self.E22_PWR_LEVELS[lvl][0].split("=")[-1].strip()
            self.refresh_lora_panel_values()
            return

        # E22 空速設定成功回報: [E22] air rate set 2 OK（兩端須一致）
        m = re.search(r"\[E22\] air rate set (\d+) OK", line)
        if m:
            ar = int(m.group(1))
            if 0 <= ar < len(self.E22_AIR_RATES):
                self.lora_e22_state["air_rate"] = self.E22_AIR_RATES[ar][0].split("=")[-1].strip()
            self.refresh_lora_panel_values()
            return

    # ==================== LoRa 參數設定面板 ====================
    # 可調範圍（與 firmware 檢查一致；顯示於 GUI 並用於本地預檢）
    E22_FREQ_RANGE = (410, 493)          # MHz
    E80_FREQ_SUGGEST = (862000000, 928000000)  # Hz（僅警告，不硬擋）
    E80_SF_RANGE = (7, 12)
    E80_PWR_RANGE = (-9, 22)             # dBm
    E80_PRE_RANGE = (6, 65535)
    E22_PWR_LEVELS = [
        ("0 = 30 dBm (1W)", "⚠ 3V3 供電會欠壓斷線，勿用"),
        ("1 = 27 dBm", ""),
        ("2 = 24 dBm", ""),
        ("3 = 21 dBm", "✅ 建議（預設）"),
    ]
    E22_AIR_RATES = [
        ("0 = 0.3k bps", "最遠射程"),
        ("1 = 1.2k bps", ""),
        ("2 = 2.4k bps", "✅ 建議（預設）"),
        ("3 = 4.8k bps", ""),
        ("4 = 9.6k bps", ""),
        ("5 = 19.2k bps", ""),
        ("6 = 38.4k bps", ""),
        ("7 = 62.5k bps", "⚠ 台面實測大量 CRC 錯"),
    ]
    E80_BW_OPTIONS = [
        ("3 = 62.5 kHz", "靈敏度最好、最慢"),
        ("4 = 125 kHz", ""),
        ("5 = 250 kHz", "✅ 預設"),
        ("6 = 500 kHz", "最快、靈敏度 -3dB/檔"),
    ]
    E80_CR_OPTIONS = [
        ("1 = 4/5", "✅ 預設（開銷最小）"),
        ("2 = 4/6", ""),
        ("3 = 4/7", ""),
        ("4 = 4/8", "糾錯最強、有效資料率最低"),
    ]

    LORA_TUTORIAL = """📡 LoRa 可調參數教學與範圍速查
════════════════════════════════════════════

【通則 — 先讀這段】
• 下列參數「兩端（火箭主航電 ↔ 地面站）必須完全一致」才能通訊：
    E22：頻率、空中速率(air)
    E80：頻率、SF、BW、CR、前導碼、(SyncWord 固定 0x12 不開放調整)
• 發射功率兩端可以不同（只影響自己發出的訊號強度）。
• 建議流程：兩塊板分別接上 USB-TTL → 各自按「查詢目前設定」核對現值
  → 依序把兩端改成相同參數 → 在地面站用 stats 確認收包率/RSSI。
• 航電端改參數時遙測會暫停約 0.3 秒（防止資料流打斷模組設定），屬正常現象。
• 備援航電 BACKUP 無 LoRa 硬體，無參數可調。

【E22-400T30S — 433MHz UART 透傳模組】
• 頻率 freq：410 ~ 493 MHz（頻道 CH = 頻率 − 410，共 84 個頻道）。
  設定存入模組 EEPROM 掉電不遺失；但重新開機時 firmware 會強制寫回
  預設 432 MHz（保證兩端一致），要長期改頻率需改 firmware 巨集 E22_TX_FREQ_MHZ。
• 發射功率 pwr：0=30dBm 1=27dBm 2=24dBm 3=21dBm。
  ⚠ 本板 3V3 供電無法穩定驅動 30dBm（突波電流 ~600mA 拉垮 3V3 → 模組欠壓
  →「發幾秒就斷」），建議維持 3 (21dBm)。
• 空中速率 air：0=0.3k 1=1.2k 2=2.4k 3=4.8k 4=9.6k 5=19.2k 6=38.4k 7=62.5k bps。
  速率越低 → 靈敏度越好、射程越遠、抗干擾越強，但資料率越低。
  實測 62.5k 台面就大量 CRC 錯；2.4k 約多 15~20dB 鏈路餘裕（預設）。⚠ 兩端須一致。
• E22 為透傳模組：RSSI/SNR 無法回讀；UART baud/parity 位元 firmware
  刻意不開放（寫錯會與模組永久失聯）。

【E80-900M2213S — 920MHz LR1121 SPI 模組】
• 頻率 freq：以 Hz 輸入（例 920000000 = 920 MHz）。
  晶片支援 862~1020 MHz，但板上天線與帶通濾波器調在 920 MHz，
  偏離越遠衰減越大 → 建議只在 915~923 MHz 內微調。
• 展頻因子 SF：7 ~ 12。每 +1 → 靈敏度約 +2.5dB（射程更遠），
  但空中時間 ×2（資料率減半）。預設 SF9。
• 頻寬 BW（idx）：3=62.5k 4=125k 5=250k(預設) 6=500k。
  頻寬加倍 → 資料率加倍、靈敏度約 −3dB。
• 編碼率 CR：1=4/5(預設) 2=4/6 3=4/7 4=4/8。數字越大糾錯越強、有效資料率越低。
• 發射功率 pwr：−9 ~ +22 dBm（22 = HP PA 上限，預設）。
• 前導碼 pre：6 ~ 65535 符號。弱訊號時加長可提升同步成功率，一般 8 即可。
• LDRO（低資料率最佳化）由 firmware 依 SF/BW 自動計算，無需手動設。

【空中時間參考（77-byte 遙測封包）】
• SF7 / BW500k / CR4-5  ≈ 0.035 秒/包（台面除錯、快速刷新）
• SF9 / BW250k / CR4-5  ≈ 0.23 秒/包（目前預設，均衡）
• SF12 / BW125k / CR4-8 > 3 秒/包（極限射程；遙測 5Hz 完全跟不上，
  發送端會自動跳包 = 更新率大幅下降）

【角色差異】
• 主航電 PRIMARY：可調 E22 + E80（E80 為發射端）。
• 備援航電 BACKUP：無 LoRa 硬體。
• 地面站 GROUND：可調 E22 + E80（E80 為接收端），
  另有 stats 統計 / e80 init / e80 rxstart / airtime 估算工具。

【改完不通了怎麼辦】
• E80：對兩端各下一次「查詢」核對六個參數逐項一致；仍不通 → 地面站按
  「e80 init」重新初始化再「e80 rxstart」。
• E22：重開機兩端（firmware 開機會強制寫回預設頻道 432MHz、功率 21dBm、
  空速 2.4k），即可回到已知良好狀態。
"""

    def open_lora_panel(self):
        if not self.running:
            messagebox.showwarning("提示", "請先開啟串口連接裝置，再進行 LoRa 參數設定。")
            return
        if hasattr(self, 'lora_win') and self.lora_win and self.lora_win.winfo_exists():
            self.lora_win.lift()
            return

        self.lora_win = tk.Toplevel(self.root)
        self.lora_win.title("📡 LoRa 通訊參數設定（E22 433MHz / E80 920MHz）")
        self.lora_win.geometry("980x700")
        self.lora_win.configure(bg="#1c1c1c")
        self.lora_win.transient(self.root)   # 不 grab_set：保留主視窗終端可同時觀察回報

        self.lora_widgets_common = []   # BACKUP 時停用
        self.lora_widgets_ground = []   # 僅 GROUND 啟用

        # ---- 頂部：角色 + 提示列 ----
        self.lbl_lora_role = tk.Label(self.lora_win, text="", bg="#1c1c1c", fg="#00d2ff",
                                      font=("Helvetica", 12, "bold"), anchor="w")
        self.lbl_lora_role.pack(fill=tk.X, padx=16, pady=(12, 2))

        tk.Label(self.lora_win,
                 text="⚠ 頻率/SF/BW/CR/空速 兩端（火箭 ↔ 地面站）必須一致才能通訊；詳見「📖 參數教學」分頁",
                 bg="#1c1c1c", fg="#ffcc00", font=("Helvetica", 10), anchor="w").pack(fill=tk.X, padx=16, pady=(0, 8))

        nb = ttk.Notebook(self.lora_win)
        nb.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))

        tab_e22 = tk.Frame(nb, bg="#1c1c1c")
        tab_e80 = tk.Frame(nb, bg="#1c1c1c")
        tab_gs = tk.Frame(nb, bg="#1c1c1c")
        tab_doc = tk.Frame(nb, bg="#1c1c1c")
        nb.add(tab_e22, text=" E22 433MHz ")
        nb.add(tab_e80, text=" E80 920MHz ")
        nb.add(tab_gs, text=" 統計/工具（地面站） ")
        nb.add(tab_doc, text=" 📖 參數教學 ")

        self.build_e22_tab(tab_e22)
        self.build_e80_tab(tab_e80)
        self.build_gs_tab(tab_gs)
        self.build_doc_tab(tab_doc)

        self.refresh_lora_panel_role()
        self.refresh_lora_panel_values()

        # 開啟即查詢兩鏈路現值（錯開避免回報交錯）
        self.lora_send("e22 show")
        self.lora_win.after(700, lambda: self.lora_send("e80 show"))

    # ---- 小工具 ----
    def lora_send(self, cmd):
        """LoRa 面板統一發送入口（沿用 send_command 的 [CMD] 終端回顯）"""
        return self.send_command(cmd)

    def lora_confirm_link_break(self, what):
        return messagebox.askyesno(
            "確認變更",
            f"即將變更 {what}。\n\n此參數兩端必須一致：只改一端會令鏈路中斷，"
            f"需到另一塊板上設定相同值才能恢復通訊。\n\n確定要套用嗎？",
            parent=self.lora_win)

    def _lora_row(self, parent, row, label_text, hint_text):
        """網格列：左標籤 + 右提示；回傳中欄容器供放輸入元件"""
        tk.Label(parent, text=label_text, bg="#1c1c1c", fg="#ffffff",
                 font=("Helvetica", 10, "bold"), anchor="w", width=16).grid(
            row=row, column=0, sticky="w", padx=(14, 6), pady=6)
        holder = tk.Frame(parent, bg="#1c1c1c")
        holder.grid(row=row, column=1, sticky="w", pady=6)
        tk.Label(parent, text=hint_text, bg="#1c1c1c", fg="#8a8a8a",
                 font=("Helvetica", 9), anchor="w", justify=tk.LEFT).grid(
            row=row, column=2, sticky="w", padx=10, pady=6)
        return holder

    # ---- E22 433MHz 分頁 ----
    def build_e22_tab(self, tab):
        self.lbl_e22_cur = tk.Label(tab, text="目前板上設定：（按「查詢」讀取）", bg="#101014",
                                    fg="#00d2ff", font=("Monaco", 10), anchor="w",
                                    padx=10, pady=8, justify=tk.LEFT)
        self.lbl_e22_cur.pack(fill=tk.X, padx=14, pady=(12, 8))

        grid = tk.Frame(tab, bg="#1c1c1c")
        grid.pack(fill=tk.X, anchor="w")

        # 頻率
        h = self._lora_row(grid, 0, "頻率 (MHz)",
                           f"範圍 {self.E22_FREQ_RANGE[0]}–{self.E22_FREQ_RANGE[1]} MHz（CH=頻率−410）\n"
                           "開機會被 firmware 強制寫回預設 432 MHz")
        self.e22_freq_var = tk.StringVar(value="432")
        tk.Spinbox(h, from_=self.E22_FREQ_RANGE[0], to=self.E22_FREQ_RANGE[1],
                   textvariable=self.e22_freq_var, width=8, font=("Monaco", 11),
                   bg="#252525", fg="#ffffff", insertbackground="white").pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用頻率", width=9, command=self.apply_e22_freq)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # 發射功率
        h = self._lora_row(grid, 1, "發射功率",
                           "⚠ 本板 3V3 供電，30dBm 會欠壓斷線\n建議固定 3 (21dBm)")
        self.e22_pwr_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E22_PWR_LEVELS],
                                          width=30, state="readonly", font=("Helvetica", 10))
        self.e22_pwr_combo.current(3)
        self.e22_pwr_combo.pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用功率", width=9, command=self.apply_e22_pwr)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # 空中速率
        h = self._lora_row(grid, 2, "空中速率",
                           "越低 → 射程越遠/抗干擾越強、資料率越低\n⚠ 兩端必須一致")
        self.e22_air_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E22_AIR_RATES],
                                          width=30, state="readonly", font=("Helvetica", 10))
        self.e22_air_combo.current(2)
        self.e22_air_combo.pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用空速", width=9, command=self.apply_e22_air)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        tools = tk.Frame(tab, bg="#1c1c1c")
        tools.pack(fill=tk.X, padx=14, pady=12)
        b = ttk.Button(tools, text="🔍 查詢目前設定 (e22 show)", width=24,
                       command=lambda: self.lora_send("e22 show"))
        b.pack(side=tk.LEFT)
        self.lora_widgets_common.append(b)

        tk.Label(tab, text="說明：E22 為 UART 透傳模組，設定存入模組 EEPROM。RSSI/SNR 無法回讀；\n"
                           "UART baud/parity 不開放調整（寫錯會與模組永久失聯）。",
                 bg="#1c1c1c", fg="#8a8a8a", font=("Helvetica", 9), justify=tk.LEFT,
                 anchor="w").pack(fill=tk.X, padx=14, pady=(4, 0))

    def apply_e22_freq(self):
        try:
            mhz = int(self.e22_freq_var.get())
        except ValueError:
            messagebox.showerror("錯誤", "頻率須為整數 MHz", parent=self.lora_win)
            return
        lo, hi = self.E22_FREQ_RANGE
        if not (lo <= mhz <= hi):
            messagebox.showerror("超出範圍", f"E22 頻率範圍 {lo}–{hi} MHz", parent=self.lora_win)
            return
        if self.lora_confirm_link_break(f"E22 頻率 → {mhz} MHz"):
            self.lora_send(f"e22 freq {mhz}")

    def apply_e22_pwr(self):
        lvl = self.e22_pwr_combo.current()
        if lvl == 0 and not messagebox.askyesno(
                "高功率警告", "30dBm 需 1W 發射，3V3 供電會欠壓造成模組斷線！\n仍要套用嗎？",
                parent=self.lora_win):
            return
        self.lora_send(f"e22 pwr {lvl}")

    def apply_e22_air(self):
        ar = self.e22_air_combo.current()
        if self.lora_confirm_link_break(f"E22 空中速率 → {self.E22_AIR_RATES[ar][0]}"):
            self.lora_send(f"e22 air {ar}")

    # ---- E80 920MHz 分頁 ----
    def build_e80_tab(self, tab):
        self.lbl_e80_cur = tk.Label(tab, text="目前板上參數：（按「查詢」讀取）", bg="#101014",
                                    fg="#00d2ff", font=("Monaco", 10), anchor="w",
                                    padx=10, pady=8, justify=tk.LEFT)
        self.lbl_e80_cur.pack(fill=tk.X, padx=14, pady=(12, 8))

        grid = tk.Frame(tab, bg="#1c1c1c")
        grid.pack(fill=tk.X, anchor="w")

        # 頻率
        h = self._lora_row(grid, 0, "頻率 (Hz)",
                           "建議 862–928 MHz；天線/濾波器調在 920 MHz\n"
                           "偏離越遠訊號衰減越大（建議 915–923 內微調）")
        self.e80_freq_var = tk.StringVar(value="920000000")
        e = tk.Entry(h, textvariable=self.e80_freq_var, width=12, font=("Monaco", 11),
                     bg="#252525", fg="#ffffff", insertbackground="white")
        e.pack(side=tk.LEFT)
        self.lbl_e80_freq_mhz = tk.Label(h, text="= 920.000 MHz", bg="#1c1c1c", fg="#00e676",
                                         font=("Monaco", 10))
        self.lbl_e80_freq_mhz.pack(side=tk.LEFT, padx=6)
        self.e80_freq_var.trace_add("write", lambda *_: self._update_e80_mhz_hint())
        b = ttk.Button(h, text="套用頻率", width=9, command=self.apply_e80_freq)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # SF
        h = self._lora_row(grid, 1, "展頻因子 SF",
                           "7–12；每 +1 → 靈敏度 +~2.5dB、空中時間 ×2\n預設 SF9")
        self.e80_sf_var = tk.StringVar(value="9")
        tk.Spinbox(h, from_=self.E80_SF_RANGE[0], to=self.E80_SF_RANGE[1],
                   textvariable=self.e80_sf_var, width=5, font=("Monaco", 11),
                   bg="#252525", fg="#ffffff", insertbackground="white").pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用 SF", width=9, command=self.apply_e80_sf)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # BW
        h = self._lora_row(grid, 2, "頻寬 BW",
                           "加倍 → 資料率加倍、靈敏度 −~3dB\n預設 5 (250kHz)")
        self.e80_bw_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E80_BW_OPTIONS],
                                         width=28, state="readonly", font=("Helvetica", 10))
        self.e80_bw_combo.current(2)
        self.e80_bw_combo.pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用 BW", width=9, command=self.apply_e80_bw)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # CR
        h = self._lora_row(grid, 3, "編碼率 CR",
                           "越大 → 糾錯越強、有效資料率越低\n預設 1 (4/5)")
        self.e80_cr_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E80_CR_OPTIONS],
                                         width=28, state="readonly", font=("Helvetica", 10))
        self.e80_cr_combo.current(0)
        self.e80_cr_combo.pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用 CR", width=9, command=self.apply_e80_cr)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # 功率
        h = self._lora_row(grid, 4, "發射功率 (dBm)",
                           f"範圍 {self.E80_PWR_RANGE[0]} ~ +{self.E80_PWR_RANGE[1]} dBm\n"
                           "22 = HP PA 上限（預設）；兩端可不同")
        self.e80_pwr_var = tk.StringVar(value="22")
        tk.Spinbox(h, from_=self.E80_PWR_RANGE[0], to=self.E80_PWR_RANGE[1],
                   textvariable=self.e80_pwr_var, width=5, font=("Monaco", 11),
                   bg="#252525", fg="#ffffff", insertbackground="white").pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用功率", width=9, command=self.apply_e80_pwr)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # 前導碼
        h = self._lora_row(grid, 5, "前導碼 (符號)",
                           f"範圍 {self.E80_PRE_RANGE[0]}–{self.E80_PRE_RANGE[1]}；一般 8 即可\n"
                           "弱訊號時加長可提升同步成功率")
        self.e80_pre_var = tk.StringVar(value="8")
        e = tk.Entry(h, textvariable=self.e80_pre_var, width=7, font=("Monaco", 11),
                     bg="#252525", fg="#ffffff", insertbackground="white")
        e.pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用前導碼", width=9, command=self.apply_e80_pre)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        tools = tk.Frame(tab, bg="#1c1c1c")
        tools.pack(fill=tk.X, padx=14, pady=12)
        b = ttk.Button(tools, text="🔍 查詢目前參數 (e80 show)", width=24,
                       command=lambda: self.lora_send("e80 show"))
        b.pack(side=tk.LEFT)
        self.lora_widgets_common.append(b)

        tk.Label(tab, text="說明：SyncWord 固定 0x12（兩端 firmware 相同即一致）；LDRO 由 firmware 依 SF/BW 自動計算。\n"
                           "改任一參數 firmware 會整組重新配置並回報 [E80] 現值行（自動更新上方欄位）。",
                 bg="#1c1c1c", fg="#8a8a8a", font=("Helvetica", 9), justify=tk.LEFT,
                 anchor="w").pack(fill=tk.X, padx=14, pady=(4, 0))

    def _update_e80_mhz_hint(self):
        try:
            hz = int(self.e80_freq_var.get())
            self.lbl_e80_freq_mhz.config(text=f"= {hz/1e6:.3f} MHz")
        except (ValueError, tk.TclError):
            self.lbl_e80_freq_mhz.config(text="= ?")

    def apply_e80_freq(self):
        try:
            hz = int(self.e80_freq_var.get())
        except ValueError:
            messagebox.showerror("錯誤", "頻率須為整數 Hz（例 920000000）", parent=self.lora_win)
            return
        lo, hi = self.E80_FREQ_SUGGEST
        if not (lo <= hz <= hi):
            if not messagebox.askyesno("頻率超出建議範圍",
                                       f"{hz/1e6:.3f} MHz 超出建議 862–928 MHz。\n"
                                       "板上天線/濾波器調在 920 MHz，偏離會嚴重衰減。\n仍要套用嗎？",
                                       parent=self.lora_win):
                return
        if self.lora_confirm_link_break(f"E80 頻率 → {hz/1e6:.3f} MHz"):
            self.lora_send(f"e80 freq {hz}")

    def apply_e80_sf(self):
        try:
            sf = int(self.e80_sf_var.get())
        except ValueError:
            return
        lo, hi = self.E80_SF_RANGE
        if not (lo <= sf <= hi):
            messagebox.showerror("超出範圍", f"SF 範圍 {lo}–{hi}", parent=self.lora_win)
            return
        if self.lora_confirm_link_break(f"E80 SF → {sf}"):
            self.lora_send(f"e80 sf {sf}")

    def apply_e80_bw(self):
        idx = int(self.E80_BW_OPTIONS[self.e80_bw_combo.current()][0].split(" ")[0])
        if self.lora_confirm_link_break(f"E80 BW → idx {idx}"):
            self.lora_send(f"e80 bw {idx}")

    def apply_e80_cr(self):
        cr = int(self.E80_CR_OPTIONS[self.e80_cr_combo.current()][0].split(" ")[0])
        if self.lora_confirm_link_break(f"E80 CR → {cr}"):
            self.lora_send(f"e80 cr {cr}")

    def apply_e80_pwr(self):
        try:
            pwr = int(self.e80_pwr_var.get())
        except ValueError:
            return
        lo, hi = self.E80_PWR_RANGE
        if not (lo <= pwr <= hi):
            messagebox.showerror("超出範圍", f"功率範圍 {lo} ~ {hi} dBm", parent=self.lora_win)
            return
        self.lora_send(f"e80 pwr {pwr}")   # 功率不破壞鏈路，免確認

    def apply_e80_pre(self):
        try:
            pre = int(self.e80_pre_var.get())
        except ValueError:
            return
        lo, hi = self.E80_PRE_RANGE
        if not (lo <= pre <= hi):
            messagebox.showerror("超出範圍", f"前導碼範圍 {lo}–{hi}", parent=self.lora_win)
            return
        if self.lora_confirm_link_break(f"E80 前導碼 → {pre}"):
            self.lora_send(f"e80 pre {pre}")

    # ---- 統計/工具（地面站）分頁 ----
    def build_gs_tab(self, tab):
        tk.Label(tab, text="以下工具僅地面站 (GROUND) firmware 支援：", bg="#1c1c1c", fg="#ffcc00",
                 font=("Helvetica", 10, "bold"), anchor="w").pack(fill=tk.X, padx=14, pady=(12, 6))

        row1 = tk.Frame(tab, bg="#1c1c1c")
        row1.pack(fill=tk.X, padx=14, pady=4)
        for text, cmd in [("📊 顯示統計 (stats)", "stats"),
                          ("🧹 清除統計 (stats reset)", "stats reset"),
                          ("🛰 E80 版本診斷 (ver)", "ver")]:
            b = ttk.Button(row1, text=text, width=24, command=lambda c=cmd: self.lora_send(c))
            b.pack(side=tk.LEFT, padx=4)
            self.lora_widgets_ground.append(b)

        row2 = tk.Frame(tab, bg="#1c1c1c")
        row2.pack(fill=tk.X, padx=14, pady=4)
        tk.Label(row2, text="自動列印統計間隔(秒, 0=關閉):", bg="#1c1c1c", fg="#ffffff",
                 font=("Helvetica", 10)).pack(side=tk.LEFT)
        self.gs_auto_var = tk.StringVar(value="5")
        tk.Spinbox(row2, from_=0, to=3600, textvariable=self.gs_auto_var, width=6,
                   font=("Monaco", 11), bg="#252525", fg="#ffffff",
                   insertbackground="white").pack(side=tk.LEFT, padx=6)
        b = ttk.Button(row2, text="套用 (stats auto N)", width=18,
                       command=lambda: self.lora_send(f"stats auto {self.gs_auto_var.get()}"))
        b.pack(side=tk.LEFT, padx=4)
        self.lora_widgets_ground.append(b)

        row3 = tk.Frame(tab, bg="#1c1c1c")
        row3.pack(fill=tk.X, padx=14, pady=4)
        for text, cmd in [("♻️ E80 重新初始化+收 (e80 init)", "e80 init"),
                          ("▶️ 重新進入連續接收 (e80 rxstart)", "e80 rxstart")]:
            b = ttk.Button(row3, text=text, width=28, command=lambda c=cmd: self.lora_send(c))
            b.pack(side=tk.LEFT, padx=4)
            self.lora_widgets_ground.append(b)

        row4 = tk.Frame(tab, bg="#1c1c1c")
        row4.pack(fill=tk.X, padx=14, pady=4)
        tk.Label(row4, text="空中時間估算 payload 長度(B):", bg="#1c1c1c", fg="#ffffff",
                 font=("Helvetica", 10)).pack(side=tk.LEFT)
        self.gs_airtime_var = tk.StringVar(value="77")
        tk.Spinbox(row4, from_=1, to=255, textvariable=self.gs_airtime_var, width=5,
                   font=("Monaco", 11), bg="#252525", fg="#ffffff",
                   insertbackground="white").pack(side=tk.LEFT, padx=6)
        b = ttk.Button(row4, text="估算 (e80 airtime N)", width=18,
                       command=lambda: self.lora_send(f"e80 airtime {self.gs_airtime_var.get()}"))
        b.pack(side=tk.LEFT, padx=4)
        self.lora_widgets_ground.append(b)

        tk.Label(tab, text="統計輸出範例：[STATS] pkt_ok / crc_err / rate / RSSI / SNR（920 才有 RSSI/SNR，\n"
                           "E22 透傳模式無法回讀）。改完參數後看 rate 與 crc_err 判斷鏈路品質。",
                 bg="#1c1c1c", fg="#8a8a8a", font=("Helvetica", 9), justify=tk.LEFT,
                 anchor="w").pack(fill=tk.X, padx=14, pady=(10, 0))

    # ---- 教學分頁 ----
    def build_doc_tab(self, tab):
        doc = ScrolledText(tab, bg="#101010", fg="#d0d0d0", font=("Monaco", 10),
                           borderwidth=0, highlightthickness=0, wrap=tk.WORD)
        doc.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        doc.insert(tk.END, self.LORA_TUTORIAL)
        doc.config(state=tk.DISABLED)

    # ---- 面板刷新 ----
    def refresh_lora_panel_role(self):
        if not (hasattr(self, 'lora_win') and self.lora_win and self.lora_win.winfo_exists()):
            return
        role = self.detected_role
        if role == "BACKUP":
            header = "🛟 備援航電 BACKUP — 無 LoRa 硬體，無參數可調"
            color = "#ffcc00"
            common, ground = tk.DISABLED, tk.DISABLED
        elif role == "GROUND":
            header = "📡 地面站 GROUND — E80 為接收端；可用「統計/工具」分頁驗證收包"
            color = "#e07bfb"
            common, ground = tk.NORMAL, tk.NORMAL
        elif role == "PRIMARY":
            header = "🚀 主航電 PRIMARY — E80 為發射端；套用參數時遙測暫停約 0.3 秒"
            color = "#00e676"
            common, ground = tk.NORMAL, tk.DISABLED
        else:
            header = "⚫ 角色偵測中… 仍可手動操作（三種角色命令語法相同）"
            color = "#aaaaaa"
            common, ground = tk.NORMAL, tk.NORMAL
        self.lbl_lora_role.config(text=header, fg=color)
        for w in self.lora_widgets_common:
            try:
                w.config(state=common)
            except tk.TclError:
                pass
        for w in self.lora_widgets_ground:
            try:
                w.config(state=ground)
            except tk.TclError:
                pass

    def refresh_lora_panel_values(self):
        """把最近解析到的板上現值刷到面板（僅更新「目前設定」顯示，不覆寫使用者輸入框）"""
        if not (hasattr(self, 'lora_win') and self.lora_win and self.lora_win.winfo_exists()):
            return
        s = self.lora_e22_state
        if s:
            parts = []
            if "freq_mhz" in s:
                parts.append(f"頻率 {s['freq_mhz']} MHz (CH={s.get('ch', '?')})")
            if "power" in s:
                parts.append(f"功率 {s['power']}")
            if "air_rate" in s:
                parts.append(f"空速 {s['air_rate']}")
            self.lbl_e22_cur.config(text="目前板上設定：" + "  |  ".join(parts))
        s = self.lora_e80_state
        if s:
            parts = []
            if "freq_hz" in s:
                parts.append(f"頻率 {s['freq_hz']/1e6:.3f} MHz")
            if "sf" in s:
                parts.append(f"SF{s['sf']}")
            if "bw_khz" in s:
                idx = f" (idx={s['bw_idx']})" if "bw_idx" in s else ""
                parts.append(f"BW {s['bw_khz']:g} kHz{idx}")
            if "cr" in s:
                parts.append(f"CR 4/{s['cr'] + 4}")
            if "pwr" in s:
                parts.append(f"功率 {s['pwr']:+d} dBm")
            if "pre" in s:
                parts.append(f"前導碼 {s['pre']}")
            self.lbl_e80_cur.config(text="目前板上參數：" + "  |  ".join(parts))

    def open_mag_calibration(self):
        if not self.running:
            messagebox.showwarning("提示", "請先開啟串口連接航電板，再進行磁強計校正。")
            return
            
        # 若已開啟，提升焦點
        if hasattr(self, 'calib_win') and self.calib_win and self.calib_win.winfo_exists():
            self.calib_win.lift()
            return
            
        # 建立彈出視窗
        self.calib_win = tk.Toplevel(self.root)
        self.calib_win.title("🧲 地磁計硬鐵校正與航向鎖定")
        self.calib_win.geometry("900x520")
        self.calib_win.configure(bg="#1c1c1c")
        self.calib_win.transient(self.root)
        self.calib_win.grab_set()
        # 關窗時保證恢復 1Hz（否則板子會卡在 10Hz 提速模式）
        self.calib_win.protocol("WM_DELETE_WINDOW", self._close_mag_calibration)

        # 重設校正收集狀態
        self.calib_x = []
        self.calib_y = []
        self.collecting_data = False
        
        # 分割視窗為左側(繪圖)與右側(控制)
        left_frame = tk.Frame(self.calib_win, bg="#1c1c1c")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=15, pady=15)
        
        right_frame = tk.Frame(self.calib_win, bg="#222222", width=350)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=15, pady=15)
        right_frame.pack_propagate(False)
        
        tk.Label(left_frame, text="2D 地磁數據分佈 (Bx vs By)", bg="#1c1c1c", fg="#00d2ff", font=("Helvetica", 11, "bold")).pack(anchor="w", pady=(0,5))
        
        # 左側 Matplotlib 繪圖
        self.calib_fig = plt.figure(facecolor="#1c1c1c")
        self.calib_ax = self.calib_fig.add_subplot(111)
        self.calib_ax.set_facecolor("#151515")
        
        self.calib_ax.axhline(0, color="#555555", linestyle=":", linewidth=0.8)
        self.calib_ax.axvline(0, color="#555555", linestyle=":", linewidth=0.8)
        self.calib_ax.set_xlabel("Bx (mG)", color="#aaaaaa")
        self.calib_ax.set_ylabel("By (mG)", color="#aaaaaa")
        self.calib_ax.tick_params(colors="#aaaaaa")
        self.calib_ax.axis('equal')
        
        self.calib_canvas = FigureCanvasTkAgg(self.calib_fig, master=left_frame)
        self.calib_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 右側控制面板
        tk.Label(right_frame, text="磁力計控制面板", bg="#222222", fg="#00d2ff", font=("Helvetica", 12, "bold")).pack(anchor="w", padx=15, pady=15)
        
        # 當前偏置與即時強度
        self.lbl_board_offsets = tk.Label(right_frame, text=f"當前板載偏置 (Counts):\n  X={int(self.board_mag_offsets[0])}, Y={int(self.board_mag_offsets[1])}, Z={int(self.board_mag_offsets[2])}", bg="#222222", fg="#ffffff", font=("Monaco", 9), justify=tk.LEFT, anchor="w")
        self.lbl_board_offsets.pack(fill=tk.X, padx=15, pady=5)
        
        self.lbl_live_mag = tk.Label(right_frame, text="即時強度: mx=0.0, my=0.0, mz=0.0 mG\n即時航向: hdg=0.0°", bg="#222222", fg="#aaaaaa", font=("Monaco", 9), justify=tk.LEFT, anchor="w")
        self.lbl_live_mag.pack(fill=tk.X, padx=15, pady=5)
        
        # 數據收集按鈕
        btn_data_frame = tk.Frame(right_frame, bg="#222222")
        btn_data_frame.pack(fill=tk.X, padx=15, pady=10)
        
        self.btn_toggle_collect = ttk.Button(btn_data_frame, text="開始收集數據", command=self.toggle_collecting)
        self.btn_toggle_collect.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,5))
        
        btn_clear_collect = ttk.Button(btn_data_frame, text="清除數據", command=self.clear_calib_data)
        btn_clear_collect.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(5,0))
        
        # 擬合結果與按鈕
        self.lbl_fit_results = tk.Label(right_frame, text="擬合結果:\n  (無擬合數據)", bg="#222222", fg="#888888", font=("Helvetica", 9), justify=tk.LEFT, anchor="w")
        self.lbl_fit_results.pack(fill=tk.X, padx=15, pady=8)
        
        self.btn_fit = ttk.Button(right_frame, text="計算硬鐵校正", command=self.fit_mag_circle)
        self.btn_fit.pack(fill=tk.X, padx=15, pady=5)
        
        self.btn_write_calib = ttk.Button(right_frame, text="寫入偏置至 Flash", state=tk.DISABLED, command=self.write_mag_calibration)
        self.btn_write_calib.pack(fill=tk.X, padx=15, pady=5)
        
        tk.Frame(right_frame, height=2, bg="#3d3d3d").pack(fill=tk.X, padx=15, pady=10)
        
        # EKF 鎖定選項
        tk.Label(right_frame, text="EKF 絕對磁北航向鎖定", bg="#222222", fg="#00d2ff", font=("Helvetica", 10, "bold")).pack(anchor="w", padx=15, pady=2)

        # 目前鎖定狀態：韌體無查詢命令，開窗先顯示預設 (g_mag_yaw_lock=1)，收到 [CAL] 回傳才轉「已確認」
        self.lbl_yaw_lock = tk.Label(right_frame, text="磁北鎖定：🔒 開（預設，未確認）",
                                     bg="#222222", fg="#888888", font=("Helvetica", 9), anchor="w")
        self.lbl_yaw_lock.pack(anchor="w", padx=15, pady=(0, 4))

        btn_yaw_frame = tk.Frame(right_frame, bg="#222222")
        btn_yaw_frame.pack(fill=tk.X, padx=15, pady=5)
        
        ttk.Button(btn_yaw_frame, text="啟用磁北鎖定", command=lambda: self.toggle_mag_yaw_lock(True)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,5))
        ttk.Button(btn_yaw_frame, text="停用磁北鎖定", command=lambda: self.toggle_mag_yaw_lock(False)).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(5,0))
        
        ttk.Button(right_frame, text="重置硬鐵偏置 (預設 131072)", command=self.reset_mag_calibration).pack(fill=tk.X, padx=15, pady=10)
        
        # 啟動定時繪圖更新
        self.update_calib_plot()

        # 開窗即把 [MAG] 提速到 50Hz（CMD_MAG_CAL_START），整個校正過程都快；關窗恢復 1Hz。
        # ⚠ 需板子韌體已具備 fast-print（g_mag_cal_fast_print，見 main.c，目前僅在未 commit
        #   的工作區）。若板子燒的是舊韌體，此指令會被忽略，[MAG] 維持 1Hz。
        self.send_command("CMD_MAG_CAL_START")

    def toggle_collecting(self):
        # 印出提速（CMD_MAG_CAL_START/STOP）已改由開/關校正視窗管理；這裡只切換是否收點。
        self.collecting_data = not self.collecting_data
        if self.collecting_data:
            self.btn_toggle_collect.config(text="停止收集數據")
            self.calib_x = []
            self.calib_y = []
            self.lbl_fit_results.config(text="擬合結果:\n  (收集中，請旋轉航電板一整圈...)", fg="#ffcc00")
            self.btn_write_calib.config(state=tk.DISABLED)
        else:
            self.btn_toggle_collect.config(text="開始收集數據")
            self.lbl_fit_results.config(text=f"擬合結果:\n  (已停止收集，共 {len(self.calib_x)} 點數據)", fg="#ffffff")

    def clear_calib_data(self):
        self.calib_x = []
        self.calib_y = []
        self.collecting_data = False
        self.btn_toggle_collect.config(text="開始收集數據")
        self.lbl_fit_results.config(text="擬合結果:\n  (無擬合數據)", fg="#888888")
        self.btn_write_calib.config(state=tk.DISABLED)
        self.calib_ax.clear()
        self.calib_ax.set_facecolor("#151515")
        self.calib_ax.axhline(0, color="#555555", linestyle=":", linewidth=0.8)
        self.calib_ax.axvline(0, color="#555555", linestyle=":", linewidth=0.8)
        self.calib_ax.set_xlabel("Bx (mG)", color="#aaaaaa")
        self.calib_ax.set_ylabel("By (mG)", color="#aaaaaa")
        self.calib_ax.tick_params(colors="#aaaaaa")
        self.calib_canvas.draw()

    def _close_mag_calibration(self):
        """關閉校正視窗：恢復 [MAG] 1Hz（CMD_MAG_CAL_STOP），再關窗。"""
        self.collecting_data = False
        self.send_command("CMD_MAG_CAL_STOP")
        try:
            self.calib_win.grab_release()
        except Exception:
            pass
        self.calib_win.destroy()

    def fit_mag_circle(self):
        if len(self.calib_x) < 10:
            messagebox.showwarning("警告", "收集的數據點太少（至少需要10點以上，建議順時針/逆時針旋轉一整圈）。", parent=self.calib_win)
            return
            
        try:
            x = np.array(self.calib_x)
            y = np.array(self.calib_y)
            
            A = np.column_stack((x, y, np.ones_like(x)))
            B = x**2 + y**2
            res, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
            
            cx_fit = res[0] / 2.0
            cy_fit = res[1] / 2.0
            R_fit = np.sqrt(res[2] + cx_fit**2 + cy_fit**2)
            
            current_ox = self.board_mag_offsets[0]   # 晶片 X 軸 offset (counts)
            current_oy = self.board_mag_offsets[1]   # 晶片 Y 軸 offset (counts)

            # calib_x/y 收的是 body 軸 (mx_body, my_body)，但 offset[] 是晶片軸。
            # 依 sensor_axis.h：mx_body=+chipY、my_body=−chipX ⇒ body 圓心 (cx,cy)
            # 還原到晶片軸為 chipX_center=−cy、chipY_center=+cx；
            # 去硬鐵 new_offset = old_offset + center_counts（gauss=(raw−offset)/LSB）。
            # 16.384 = 16384 counts/Gauss ÷ 1000 = counts/mG。
            # ⚠ 方向依當前韌體映射推導，建議上板轉一圈看 heading 收斂方向再定案。
            new_ox = current_ox - cy_fit * 16.384   # 晶片 X ← −cy
            new_oy = current_oy + cx_fit * 16.384   # 晶片 Y ← +cx
            
            self.lbl_fit_results.config(
                text=f"擬合結果:\n"
                     f"  圓心偏移 (mG): cx={cx_fit:.1f}, cy={cy_fit:.1f}\n"
                     f"  圓半徑 (mG): R={R_fit:.1f}\n\n"
                     f"建議寫入偏置 (Counts):\n"
                     f"  X_offset = {int(new_ox)}\n"
                     f"  Y_offset = {int(new_oy)}",
                fg="#00d2ff"
            )
            
            self.draw_fit_circle(cx_fit, cy_fit, R_fit)
            
            self.new_ox = new_ox
            self.new_oy = new_oy
            self.btn_write_calib.config(state=tk.NORMAL)
            
        except Exception as e:
            messagebox.showerror("錯誤", f"擬合計算失敗: {e}", parent=self.calib_win)

    def draw_fit_circle(self, cx, cy, R):
        theta = np.linspace(0, 2*np.pi, 100)
        circle_x = cx + R * np.cos(theta)
        circle_y = cy + R * np.sin(theta)
        
        self.calib_ax.clear()
        self.calib_ax.set_facecolor("#151515")
        
        self.calib_ax.scatter(self.calib_x, self.calib_y, color="#00e5ff", s=10, label="收集數據")
        self.calib_ax.plot(cx, cy, "ro", markersize=6, label="擬合圓心")
        self.calib_ax.plot(circle_x, circle_y, "r--", linewidth=1.5, label="擬合圓")
        
        self.calib_ax.axhline(0, color="#555555", linestyle=":", linewidth=0.8)
        self.calib_ax.axvline(0, color="#555555", linestyle=":", linewidth=0.8)
        
        self.calib_ax.set_xlabel("Bx (mG)", color="#aaaaaa")
        self.calib_ax.set_ylabel("By (mG)", color="#aaaaaa")
        self.calib_ax.tick_params(colors="#aaaaaa")
        self.calib_ax.legend(facecolor="#222222", edgecolor="none", labelcolor="#ffffff")
        self.calib_ax.axis('equal')
        self.calib_canvas.draw()

    def write_mag_calibration(self):
        if not hasattr(self, 'new_ox') or not hasattr(self, 'new_oy'):
            return
            
        new_oz = self.board_mag_offsets[2]

        # 韌體 CMD_MAG_CAL 只接受「整數 raw ADC counts」（main.c:2399 sscanf %ld；
        # nano.specs 不支援 %f，送浮點會被拒收）。故一律取整數。
        ox, oy, oz = int(round(self.new_ox)), int(round(self.new_oy)), int(round(new_oz))
        cmd = f"CMD_MAG_CAL:{ox},{oy},{oz}"
        if self.send_command(cmd):
            # 不在此宣告成功：等韌體回傳 [CAL] ... saved to Flash (x10) 由序列解析
            # (_on_mag_write_confirmed) 才確認，避免「送出＝成功」的假象。
            self._pending_mag_write = (ox, oy, oz)
            self.lbl_board_offsets.config(
                text=f"當前板載偏置 (Counts):\n  X={ox}, Y={oy}, Z={oz}  ⏳ 等待韌體寫入確認…",
                fg="#ffcc00"
            )
            self._arm_mag_write_timeout()

    def toggle_mag_yaw_lock(self, enable):
        val = 1 if enable else 0
        cmd = f"CMD_MAG_YAW_LOCK:{val}"
        if self.send_command(cmd):
            # 不樂觀報成功：等韌體 [CAL] EKF Mag Yaw Lock set to: N 回傳再確認。
            self._pending_yaw_lock = enable
            if hasattr(self, 'lbl_yaw_lock') and self.lbl_yaw_lock.winfo_exists():
                status = "啟用" if enable else "停用"
                self.lbl_yaw_lock.config(text=f"磁北鎖定：⏳ 等待韌體確認{status}…", fg="#ffcc00")

    def _on_yaw_lock_confirmed(self, locked):
        """收到韌體 [CAL] EKF Mag Yaw Lock set to: N 確認：更新狀態顯示。"""
        if hasattr(self, 'lbl_yaw_lock') and self.lbl_yaw_lock.winfo_exists():
            if locked:
                self.lbl_yaw_lock.config(text="磁北鎖定：🔒 開（已確認）", fg="#28a745")
            else:
                self.lbl_yaw_lock.config(text="磁北鎖定：🔓 關（已確認）", fg="#ff9500")
        self.console.insert(tk.END, f"[CAL] ✅ 磁北鎖定已{'開啟' if locked else '關閉'}\n", "ok")
        self.console.see(tk.END)
        self._pending_yaw_lock = None

    def reset_mag_calibration(self):
        if messagebox.askyesno("確認", "是否確定要重置地磁偏置至預設值 (131072)？", parent=self.calib_win):
            cmd = "CMD_MAG_CAL:131072,131072,131072"
            if self.send_command(cmd):
                # 同 write：等韌體 [CAL] 回傳才確認（見 _on_mag_write_confirmed）。
                self._pending_mag_write = (131072, 131072, 131072)
                self.clear_calib_data()
                self.lbl_board_offsets.config(
                    text="當前板載偏置 (Counts):\n  X=131072, Y=131072, Z=131072  ⏳ 等待韌體寫入確認…",
                    fg="#ffcc00"
                )
                self._arm_mag_write_timeout()

    # ------------------ 地磁偏置寫入確認（由 poll_queue 序列回呼，主執行緒） ------------------
    # ⚠ 這些在序列處理迴圈中被呼叫，絕不可跳 modal messagebox：校正視窗持有 grab_set()，
    #   對話框會被蓋在後面又搶不到輸入 → 整個 GUI 死鎖。一律只更新非阻塞標籤。
    #   原始 [CAL] 行本身已由 poll_queue 分流顯示在「重要訊息」區，不需再彈窗。
    def _on_mag_write_confirmed(self):
        ox, oy, oz = (int(round(v)) for v in self.board_mag_offsets)
        if hasattr(self, 'lbl_board_offsets') and self.lbl_board_offsets.winfo_exists():
            self.lbl_board_offsets.config(
                text=f"當前板載偏置 (Counts):\n  X={ox}, Y={oy}, Z={oz}  ✅ 已寫入 Flash",
                fg="#28a745"
            )
        self._pending_mag_write = None

    def _on_mag_write_failed(self, line):
        if hasattr(self, 'lbl_board_offsets') and self.lbl_board_offsets.winfo_exists():
            self.lbl_board_offsets.config(text="⚠ 韌體回報寫入失敗，詳見重要訊息區", fg="#dc3545")
        self._pending_mag_write = None

    def _arm_mag_write_timeout(self):
        """3 秒未收到 [CAL] 寫入確認就提示（板子韌體可能較舊/指令被拒），不再永遠卡在 ⏳。"""
        self._mag_write_token = getattr(self, '_mag_write_token', 0) + 1
        tok = self._mag_write_token
        try:
            self.root.after(3000, lambda: self._check_mag_write_timeout(tok))
        except Exception:
            pass

    def _check_mag_write_timeout(self, tok):
        if tok == getattr(self, '_mag_write_token', 0) and getattr(self, '_pending_mag_write', None) is not None:
            self._pending_mag_write = None
            if hasattr(self, 'lbl_board_offsets') and self.lbl_board_offsets.winfo_exists():
                self.lbl_board_offsets.config(
                    text="⚠ 未收到韌體寫入確認\n（板子韌體可能較舊，或指令被拒，見重要訊息區）",
                    fg="#dc3545")

    def update_calib_plot(self):
        if not hasattr(self, 'calib_win') or not self.calib_win.winfo_exists():
            return
            
        if self.collecting_data and len(self.calib_x) > 0:
            self.calib_ax.clear()
            self.calib_ax.set_facecolor("#151515")
            self.calib_ax.scatter(self.calib_x, self.calib_y, color="#00e5ff", s=10, label="收集數據")
            
            self.calib_ax.axhline(0, color="#555555", linestyle=":", linewidth=0.8)
            self.calib_ax.axvline(0, color="#555555", linestyle=":", linewidth=0.8)
            
            self.calib_ax.set_xlabel("Bx (mG)", color="#aaaaaa")
            self.calib_ax.set_ylabel("By (mG)", color="#aaaaaa")
            self.calib_ax.tick_params(colors="#aaaaaa")
            self.calib_ax.axis('equal')
            self.calib_canvas.draw_idle()
            
        if self.latest_mag:
            self.lbl_live_mag.config(
                text=f"即時強度: mx={self.latest_mag['mx']:.1f}, my={self.latest_mag['my']:.1f}, mz={self.latest_mag['mz']:.1f} mG\n"
                     f"即時航向: hdg={self.latest_mag['hdg']:.1f}°"
            )
            
        self.calib_win.after(200, self.update_calib_plot)

    # ==================== 軸向對齊測試精靈 ====================
    def open_test_wizard(self):
        if not self.running:
            messagebox.showwarning("提示", "請先開啟串口連接航電板，再進行一致性測試。")
            return
        
        # 建立彈出視窗
        self.wizard_win = tk.Toplevel(self.root)
        self.wizard_win.title("🧭 航電板感測器軸向一致性測試精靈")
        self.wizard_win.geometry("580x460")
        self.wizard_win.configure(bg="#1c1c1c")
        self.wizard_win.transient(self.root) 
        self.wizard_win.grab_set()           
        
        self.current_step_idx = 0
        self.is_detecting = False
        
        # 建立 UI 組件
        self.setup_wizard_ui()
        
        # 啟動即時數據更新 loop
        self.update_wizard_live_feedback()

    def setup_wizard_ui(self):
        # 標題
        self.lbl_step_title = tk.Label(self.wizard_win, text="", bg="#1c1c1c", fg="#00d2ff", font=("Helvetica", 12, "bold"))
        self.lbl_step_title.pack(fill=tk.X, side=tk.TOP, pady=12)
        
        # 動作提示框 (全息邊框風格)
        prompt_frame = tk.Frame(self.wizard_win, bg="#252525", borderwidth=1, relief="solid")
        prompt_frame.pack(fill=tk.BOTH, expand=False, padx=20, pady=8, ipady=5)
        
        self.lbl_prompt = tk.Label(prompt_frame, text="", bg="#252525", fg="#ffffff", font=("Helvetica", 10), justify=tk.LEFT, wraplength=520)
        self.lbl_prompt.pack(fill=tk.BOTH, expand=True, padx=12, pady=10)
        
        # 即時數據反饋與檢測診斷框
        self.lbl_live_val = tk.Label(self.wizard_win, text="即時數據載入中...", bg="#101010", fg="#aaaaaa", font=("Monaco", 9), justify=tk.LEFT, anchor="nw", borderwidth=1, relief="sunken", height=8)
        self.lbl_live_val.pack(fill=tk.BOTH, expand=True, padx=20, pady=8)
        
        # 貼心狀態條
        self.lbl_status = tk.Label(self.wizard_win, text="等待開始...", bg="#1c1c1c", fg="#aaaaaa", font=("Helvetica", 10, "bold"))
        self.lbl_status.pack(fill=tk.X, side=tk.TOP, pady=8)
        
        # 控制按鈕區
        btn_frame = tk.Frame(self.wizard_win, bg="#1c1c1c")
        btn_frame.pack(fill=tk.X, side=tk.BOTTOM, pady=15, padx=20)
        
        self.btn_prev = ttk.Button(btn_frame, text="◀ 上一步", width=10, command=self.prev_wizard_step)
        self.btn_prev.pack(side=tk.LEFT, padx=5)
        
        self.btn_detect = ttk.Button(btn_frame, text="⚡ 開始檢測", width=12, command=self.run_wizard_detection)
        self.btn_detect.pack(side=tk.LEFT, padx=15)
        
        ttk.Button(btn_frame, text="關閉測試", width=10, command=self.wizard_win.destroy).pack(side=tk.RIGHT, padx=5)
        
        self.btn_next = ttk.Button(btn_frame, text="下一步 ▶", width=10, command=self.next_wizard_step)
        self.btn_next.pack(side=tk.RIGHT, padx=5)
        
        self.update_wizard_step()

    def update_wizard_step(self):
        step_info = TEST_STEPS[self.current_step_idx]
        self.lbl_step_title.config(text=step_info["title"])
        self.lbl_prompt.config(text=step_info["prompt"])
        self.lbl_status.config(text="等待檢測...", fg="#aaaaaa")
        
        # 設定按鈕可用性
        self.btn_prev.config(state=tk.DISABLED if self.current_step_idx == 0 else tk.NORMAL)
        self.btn_next.config(state=tk.DISABLED if self.current_step_idx == len(TEST_STEPS) - 1 else tk.NORMAL)

    def next_wizard_step(self):
        if self.current_step_idx < len(TEST_STEPS) - 1:
            self.current_step_idx += 1
            self.update_wizard_step()

    def prev_wizard_step(self):
        if self.current_step_idx > 0:
            self.current_step_idx -= 1
            self.update_wizard_step()

    def update_wizard_live_feedback(self):
        if not hasattr(self, 'wizard_win') or not self.wizard_win.winfo_exists():
            return
            
        step_info = TEST_STEPS[self.current_step_idx]
        sensor = step_info["sensor"]
        
        text = ""
        if not self.is_detecting:
            if sensor == "IMU" and self.latest_imu:
                text = (f"即時數據 (BMI088):\n"
                        f"  ax = {self.latest_imu['ax']:.1f} mG\n"
                        f"  ay = {self.latest_imu['ay']:.1f} mG\n"
                        f"  az = {self.latest_imu['az']:.1f} mG\n"
                        f"  gx = {self.latest_imu['gx']:.1f} dps\n"
                        f"  gy = {self.latest_imu['gy']:.1f} dps\n"
                        f"  gz = {self.latest_imu['gz']:.1f} dps")
            elif sensor == "HIGHG" and self.latest_highg:
                text = (f"即時數據 (ADXL375):\n"
                        f"  ax = {self.latest_highg['ax']:.1f} mG\n"
                        f"  ay = {self.latest_highg['ay']:.1f} mG\n"
                        f"  az = {self.latest_highg['az']:.1f} mG")
            elif sensor == "MAG" and self.latest_mag:
                text = (f"即時數據 (MMC5983):\n"
                        f"  mx = {self.latest_mag['mx']:.1f} mG\n"
                        f"  my = {self.latest_mag['my']:.1f} mG\n"
                        f"  mz = {self.latest_mag['mz']:.1f} mG\n"
                        f"  hdg = {self.latest_mag['hdg']:.1f}°")
            elif sensor == "BOTH" and self.latest_imu and self.latest_highg:
                text = (f"即時數據 (低G & 高G Z軸):\n"
                        f"  BMI088  az = {self.latest_imu['az']:.1f} mG\n"
                        f"  ADXL375 az = {self.latest_highg['az']:.1f} mG")
            else:
                text = "⏳ 等待資料串流中，請確認串口已連接且正輸出遙測..."
                
            try:
                if self.lbl_live_val.winfo_exists():
                    self.lbl_live_val.config(text=text, fg="#aaaaaa")
            except tk.TclError:
                pass
            
        try:
            if self.wizard_win.winfo_exists():
                self.wizard_win.after(100, self.update_wizard_live_feedback)
        except tk.TclError:
            pass

    def run_wizard_detection(self):
        if self.is_detecting:
            return
            
        step_info = TEST_STEPS[self.current_step_idx]
        self.is_detecting = True
        self.btn_detect.config(state=tk.DISABLED)
        self.lbl_status.config(text="🔍 正在檢測，請維持動作...", fg="#ffcc00")
        
        # 收集樣本
        samples = []
        duration = 1.0 if step_info["type"] == "static" else 2.0
        start_time = time.time()
        
        def sample_loop():
            if not hasattr(self, 'wizard_win') or not self.wizard_win.winfo_exists():
                self.is_detecting = False
                return
                
            elapsed = time.time() - start_time
            if elapsed < duration:
                # 採樣
                if step_info["type"] == "static":
                    if step_info["sensor"] == "IMU" and self.latest_imu:
                        samples.append(self.latest_imu.copy())
                    elif step_info["sensor"] == "HIGHG" and self.latest_highg:
                        samples.append(self.latest_highg.copy())
                    elif step_info["sensor"] == "MAG" and self.latest_mag:
                        samples.append(self.latest_mag.copy())
                    elif step_info["sensor"] == "BOTH" and self.latest_imu and self.latest_highg:
                        samples.append({"imu": self.latest_imu.copy(), "hg": self.latest_highg.copy()})
                else: # dynamic (peak recording)
                    axis = step_info["axis"]
                    if self.latest_imu:
                        samples.append(self.latest_imu[axis])
                        
                pct = int((elapsed / duration) * 100)
                self.lbl_status.config(text=f"⏳ 採樣中... {pct}%", fg="#ffcc00")
                self.root.after(50, sample_loop)
            else:
                self.evaluate_wizard_result(step_info, samples)
                
        sample_loop()

    def evaluate_wizard_result(self, step_info, samples):
        self.is_detecting = False
        self.btn_detect.config(state=tk.NORMAL)
        
        if not hasattr(self, 'wizard_win') or not self.wizard_win.winfo_exists():
            return
            
        if not samples:
            self.lbl_status.config(text="❌ [ERROR] 未收集到有效數據！", fg="#ff3b30")
            return
            
        if step_info["type"] == "static":
            if step_info["sensor"] == "BOTH":
                avg_imu_az = sum(s["imu"]["az"] for s in samples) / len(samples)
                avg_imu_ax = sum(s["imu"]["ax"] for s in samples) / len(samples)
                avg_imu_ay = sum(s["imu"]["ay"] for s in samples) / len(samples)
                avg_hg_az = sum(s["hg"]["az"] for s in samples) / len(samples)
                avg_hg_ax = sum(s["hg"]["ax"] for s in samples) / len(samples)
                avg_hg_ay = sum(s["hg"]["ay"] for s in samples) / len(samples)
                val = {
                    "imu": {"ax": avg_imu_ax, "ay": avg_imu_ay, "az": avg_imu_az},
                    "hg": {"ax": avg_hg_ax, "ay": avg_hg_ay, "az": avg_hg_az}
                }
            else:
                keys = samples[0].keys()
                val = {}
                for k in keys:
                    val[k] = sum(s[k] for s in samples) / len(samples)
                # 對於地磁計，直接平均 hdg 度數在 0°/360° 邊界（朝北）會產生嚴重的均值環繞錯誤
                # 改為先將 Cartesian 分量 mx, my 平均，再用 np.arctan2 重新計算均值 hdg
                if "mx" in val and "my" in val:
                    hdg_deg = float(np.degrees(np.arctan2(-val["mx"], val["my"])))
                    if hdg_deg < 0:
                        hdg_deg += 360.0
                    val["hdg"] = hdg_deg
                    
            ok = step_info["run"](val)
            msg = step_info["result_msg"](val, ok)
            
        else: # dynamic
            peak = max(samples, key=abs)
            ok = step_info["run"](peak)
            msg = step_info["result_msg"](peak, ok)
            
        if ok:
            self.lbl_status.config(text="✅ [PASS] 檢測通過！", fg="#28a745")
        else:
            self.lbl_status.config(text="❌ [FAIL] 檢測未通過！", fg="#ff3b30")
            
        self.lbl_live_val.config(text=msg, fg="#ffffff" if ok else "#ff4f4f")

# ==================== 測試項目資料結構 ====================
TEST_STEPS = [
    {
        "step": 1,
        "title": "【測試 1/9】BMI088 加速度計 Z 軸 (鼻錐朝上 / 靜態)",
        "prompt": "請將航電板【水平靜止放置於桌面，正面朝上】(Z 軸朝上，即鼻錐朝上)。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "IMU",
        "run": lambda val: 800.0 <= val["az"] <= 1200.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：Z 軸加速度 = {val['az']:.1f} mG\n" + (
            "✅ [PASS] Z 軸方向正確（鼻錐朝上）！" if ok else
            "❌ [FAIL] Z 軸方向異常！(預期 +800 ~ +1200 mG)\n💡 若接近 -1000 mG：代表 Z 軸被反置，需要在 main.c 中將 az 取負。"
        )
    },
    {
        "step": 2,
        "title": "【測試 2/9】BMI088 加速度計 X 軸 (右側朝上)",
        "prompt": "請將航電板【右側抬高約 45 度】(右緣朝上，body 右軸朝天)。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "IMU",
        "run": lambda val: val["ax"] > 200.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：X 軸加速度 = {val['ax']:.1f} mG\n" + (
            "✅ [PASS] 右側朝上時 ax 為正，X=右 對齊正確！" if ok else
            "❌ [FAIL] 右側朝上時 ax 不為正！\n💡 建議：晶片貼裝與 sensor_axis.h 表格不符，請核對 IMU 映射 (X->Y)。"
        )
    },
    {
        "step": 3,
        "title": "【測試 3/9】BMI088 加速度計 Y 軸 (前緣朝上)",
        "prompt": "請將航電板【前緣抬高約 45 度】(前緣朝上，body 前軸朝天)。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "IMU",
        "run": lambda val: val["ay"] > 200.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：Y 軸加速度 = {val['ay']:.1f} mG\n" + (
            "✅ [PASS] 前緣朝上時 ay 為正，Y=前 對齊正確！" if ok else
            "❌ [FAIL] 前緣朝上時 ay 不為正！\n💡 建議：晶片貼裝與 sensor_axis.h 表格不符，請核對 IMU 映射 (Y->X)。"
        )
    },
    {
        "step": 4,
        "title": "【測試 4/9】ADXL375 高 G 加速度計軸向對齊 (靜態)",
        "prompt": "請將航電板再次【水平靜置放於桌面，正面朝上】。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "BOTH",
        "run": lambda val: val["hg"]["az"] > 300.0,
        "result_msg": lambda val, ok: (
            f"📊 BMI088 (低G) 重力分量: [{val['imu']['ax']:.1f}, {val['imu']['ay']:.1f}, {val['imu']['az']:.1f}] mG\n"
            f"📊 ADXL375 (高G) 重力分量: [{val['hg']['ax']:.1f}, {val['hg']['ay']:.1f}, {val['hg']['az']:.1f}] mG\n"
            + ("✅ [PASS] ADXL375 Z 軸朝上，大於 +300 mG！" if ok else
               "❌ [FAIL] ADXL375 Z 軸讀值不足（< 300 mG）！\n💡 建議：在 main.c 高G替換邏輯中將 az 改為 +raw_adxl_az（去掉負號）。")
        )
    },
    {
        "step": 5,
        "title": "【測試 5/9】BMI088 陀螺儀 Z 軸 (Yaw/自旋)",
        "prompt": "請準備將航電板在水平桌面上【快速逆時針旋轉】。\n按下「開始檢測」後，請【立即開始逆時針快速旋轉約 2 秒】...",
        "type": "dynamic",
        "sensor": "IMU",
        "axis": "gz",
        "run": lambda peak: peak > 25.0,
        "result_msg": lambda peak, ok: f"📊 偵測結果：最大 Z 軸角速度 = {peak:.1f} dps\n" + (
            "✅ [PASS] 逆時針旋轉輸出為正，Yaw 軸向符合右手定則！" if ok else
            "❌ [FAIL] 逆時針旋轉輸出為負！Yaw 旋轉方向相反。\n💡 建議：您需要在 main.c 將 gz 取負。"
        )
    },
    {
        "step": 6,
        "title": "【測試 6/9】BMI088 陀螺儀 X 軸 (Pitch/俯仰，繞右軸)",
        "prompt": "請準備將航電板【快速抬頭/後仰】(前端/前緣朝上抬起)。\n按下「開始檢測」後，請【立即快速將板子前半部朝上抬起旋轉約 2 秒】...",
        "type": "dynamic",
        "sensor": "IMU",
        "axis": "gx",
        "run": lambda peak: peak > 25.0,
        "result_msg": lambda peak, ok: f"📊 偵測結果：最大 X 軸角速度 = {peak:.1f} dps\n" + (
            "✅ [PASS] 抬頭(後仰)旋轉為正，Pitch 繞 X(右)軸符合右手定則！" if ok else
            "❌ [FAIL] 抬頭旋轉為負！Pitch 旋轉方向相反。\n💡 建議：在 main.c 將 gx 來源 -raw_gy 改為 +raw_gy。"
        )
    },
    {
        "step": 7,
        "title": "【測試 7/9】BMI088 陀螺儀 Y 軸 (Roll/翻滾，繞前軸)",
        "prompt": "請準備將航電板【快速向右翻滾】(right side down)。\n按下「開始檢測」後，請【立即快速將板子向右翻滾】...",
        "type": "dynamic",
        "sensor": "IMU",
        "axis": "gy",
        "run": lambda peak: peak > 25.0,
        "result_msg": lambda peak, ok: f"📊 偵測結果：最大 Y 軸角速度 = {peak:.1f} dps\n" + (
            "✅ [PASS] 向右翻滾為正，Roll 繞 Y(前)軸符合右手定則！" if ok else
            "❌ [FAIL] 向右翻滾為負！Roll 旋轉方向相反。\n💡 建議：在 main.c 將 gy 來源 -raw_gx 改為 +raw_gx。"
        )
    },
    {
        "step": 8,
        "title": "【測試 8/9】MMC5983MA 地磁計航向角 (前緣朝北 ≈ 0°)",
        "prompt": "請將航電板水平靜置（Z軸朝天），並將【前緣/Y軸朝向正北方】。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "MAG",
        "run": lambda val: val["hdg"] <= 30.0 or val["hdg"] >= 330.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：地磁向量 B = [{val['mx']:.1f}, {val['my']:.1f}] mG, 航向角 hdg = {val['hdg']:.1f}°\n" + (
            f"✅ [PASS] 前緣朝北，航向角 {val['hdg']:.1f}° ≈ 0°，對齊正確！" if ok else
            f"❌ [FAIL] 前緣朝北，航向角 {val['hdg']:.1f}°，預期 ≈ 0°（±30°）。\n💡 建議：在 main.c 調整磁力計 mx_body/my_body 的來源軸。"
        )
    },
    {
        "step": 9,
        "title": "【測試 9/9】MMC5983MA 地磁計航向角收斂方向 (前緣朝東 ≈ 90°)",
        "prompt": "請將航電板水平靜置（Z軸朝天），並【順時針旋轉 90 度】(前緣朝向正東方)。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "MAG",
        "run": lambda val: 45.0 <= val["hdg"] <= 135.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：地磁 Y 分量 my = {val['my']:.1f} mG, 航向角 hdg = {val['hdg']:.1f}°\n" + (
            f"✅ [PASS] 轉向東方航向角為 {val['hdg']:.1f}°，收斂方向正確！" if ok else
            f"❌ [FAIL] 轉向東方航向角為 {val['hdg']:.1f}°！預期應為 ~90°\n💡 建議：如果航向角往反方向收斂（例如變成 270°），您需要在 main.c 調整磁力計軸向或 heading 計算公式。"
        )
    }
]

# ==================== 5. 主入口 ====================
if __name__ == "__main__":
    # 在 macOS 上強制使用 TkAgg 後端以適應 Tkinter 視窗整合
    import matplotlib
    matplotlib.use("TkAgg")
    
    root = tk.Tk()
    app = RocketDashboardApp(root)
    root.mainloop()

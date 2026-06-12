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
        self.root.geometry("1300x820")
        self.root.configure(bg="#151515")
        
        # 狀態配置
        self.serial_thread = None
        self.running = False
        self.paused = False
        self.data_queue = queue.Queue()
        self.fsm_state = "STATE_PAD"
        
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

    def create_widgets(self):
        # ------------------ 頂部標題與連接面板 ------------------
        top_bar = tk.Frame(self.root, bg="#151515", height=60)
        top_bar.pack(fill=tk.X, side=tk.TOP, padx=15, pady=8)
        
        title_label = tk.Label(top_bar, text="ROCKET AVIONICS 3D ATTITUDE DASHBOARD", bg="#151515", fg="#00d2ff", font=("Helvetica", 16, "bold"))
        title_label.pack(side=tk.LEFT, pady=5)
        
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
            ("Flash PKTs", "flash_pkt", "0 Pkts", "#ffffff")
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

        # ------------------ 中部雙視窗面板 (左右分欄) ------------------
        main_pane = tk.Frame(self.root, bg="#151515")
        main_pane.pack(fill=tk.BOTH, expand=True, side=tk.TOP, padx=15, pady=10)
        
        # 左側: 終端日誌 (Terminal Console)
        left_frame = tk.Frame(main_pane, bg="#1e1e1e")
        left_frame.pack(fill=tk.BOTH, expand=True, side=tk.LEFT, padx=(0, 8))
        
        tk.Label(left_frame, text=" > TERMINAL TELEMETRY STREAM", bg="#1e1e1e", fg="#00d2ff", font=("Monaco", 10, "bold")).pack(anchor="w", padx=10, pady=5)
        
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
        
        # 終端底部按鈕區
        console_tools = tk.Frame(left_frame, bg="#1e1e1e")
        console_tools.pack(fill=tk.X, side=tk.BOTTOM, padx=5, pady=5)
        
        ttk.Button(console_tools, text="清除終端", width=10, command=self.clear_console).pack(side=tk.LEFT, padx=5)
        ttk.Button(console_tools, text="🧭 軸向對齊測試", width=14, command=self.open_test_wizard).pack(side=tk.LEFT, padx=5)
        ttk.Button(console_tools, text="🧲 磁強計校正與鎖定", width=18, command=self.open_mag_calibration).pack(side=tk.LEFT, padx=5)
        self.lbl_drops = tk.Label(console_tools, text="EKF Queue Drops: 0", bg="#1e1e1e", fg="#aaaaaa", font=("Monaco", 9))
        self.lbl_drops.pack(side=tk.RIGHT, padx=10)

        # 右側: 3D 姿態渲染器 (3D Attitude Visualizer)
        right_frame = tk.Frame(main_pane, bg="#1e1e1e")
        right_frame.pack(fill=tk.BOTH, expand=True, side=tk.RIGHT, padx=(8, 0))
        
        # 3D 姿態控制區
        title_3d_frame = tk.Frame(right_frame, bg="#1e1e1e")
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
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 初始化靜態 3D 火箭渲染
        self.update_3d_plot([1.0, 0.0, 0.0, 0.0])

    # ------------------ UI 操作函數 ------------------
    def scan_ports(self):
        """自動偵測可用串口（偵測優先序統一在 serial_link.auto_port）"""
        ports = serial_link.list_candidate_ports()
        self.port_combo['values'] = ports
        self.port_combo.set(serial_link.auto_port() or DEFAULT_PORT)

    def clear_console(self):
        self.console.delete("1.0", tk.END)

    def toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.config(text="繼續渲染" if self.paused else "暫停渲染")

    def reset_view(self):
        self.ax.view_init(elev=20, azim=45)
        if hasattr(self, 'canvas') and self.canvas:
            self.canvas.draw()

    def clear_rate_cards(self):
        for key in self.cards:
            self.cards[key].config(text="0.00 Hz" if key != "flash_pkt" else "0 Pkts")
        self.lbl_drops.config(text="EKF Queue Drops: 0")

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
            
            # 打開記錄檔
            if self.save_log_var.get():
                now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.log_file = open(f"gui_serial_continuous_{now_str}.log", "w", encoding="utf-8")
                self.log_file.write(f"--- GUI CONTINUOUS MONITOR SESSION START AT {datetime.now()} ---\n")
                self.log_file.flush()
                
            self.running = True
            self.clear_console()
            self.clear_rate_cards()
            
            # 建立串列背景執行緒
            self.serial_thread = threading.Thread(target=self.serial_read_task, daemon=True)
            self.serial_thread.start()
            
            self.btn_connect.config(text="中斷連線", style="Disconnect.TButton")
            self.console.insert(tk.END, f"[SYSTEM] 🟢 已成功連接 {port} @ {baud} baud\n")
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
                # 串口異常中斷
                if self.running:
                    self.data_queue.put(("ERR", f"[SYSTEM] ⚠️ 串口通訊中斷，連線關閉: {e}"))
                    self.running = False
                break
                
        # 清除連線狀態
        try:
            if self.root.winfo_exists():
                self.root.after(0, self.update_disconnect_ui)
        except tk.TclError:
            pass

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
                
            # 輸出滾動字元日誌，高亮重要字眼
            tag = "tele"
            if "[RATE]" in line:
                tag = "rate"
            elif "[MAG]" in line:
                tag = "mag"
            elif "[GPS]" in line:
                tag = "gps"
            elif "ok=" in line or "Ready" in line or "PASS" in line:
                tag = "ok"
            elif "err=" in line and "err:0" not in line and "err:0," not in line:
                tag = "err"
                
            self.console.insert(tk.END, f"[{ts}] {line}\n", tag)
            
            # 定期截斷終端，防記憶體飆升 (上限 2000 行)
            if float(self.console.index('end-1c')) > 2000.0:
                self.console.delete("1.0", "200.0")
                
            self.console.see(tk.END)
            
            # 解析並處理特殊遙測數據
            self.parse_telemetry(line)
            
        # 繼續定時輪詢
        try:
            if self.root.winfo_exists():
                self.root.after(10, self.poll_queue)
        except tk.TclError:
            pass

    def parse_telemetry(self, line):
        """解析串口封包，更新 3D 姿態與頂部狀態數位卡片"""
        # A. 解析四元數 EKF 姿態封包 [TELE] pos:x,y,z vel:x,y,z q:qw,qx,qy,qz
        if "[TELE]" in line and "q:" in line:
            m = re.search(r"q:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                q = [float(m.group(i)) for i in range(1, 5)]
                self.last_q = q
                if not self.paused:
                    self.update_3d_plot(q)
                    
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
            if m_gps: self.cards["gps"].config(text=f"{float(m_gps.group(1)):.2f} Hz")
            if m_drop: self.lbl_drops.config(text=f"EKF Queue Drops: {m_drop.group(1)}")
            
        # C. 解析 Flash 封包 [FLASH_RING] PKT_TOTAL:xx ADDR:xx
        elif "[FLASH_RING]" in line and "PKT_TOTAL:" in line:
            m_pkt = re.search(r"PKT_TOTAL:(\d+)", line)
            if m_pkt:
                self.cards["flash_pkt"].config(text=f"{m_pkt.group(1)} Pkts")
                
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

        # H. 解析 MMC5983 初始或儲存的偏移量
        if "[MAG] MMC5983MA online. offset[X,Y,Z]=" in line:
            m = re.search(r"offset\[X,Y,Z\]=(-?\d+),(-?\d+),(-?\d+)", line)
            if m:
                self.board_mag_offsets = [float(m.group(1)), float(m.group(2)), float(m.group(3))]
        elif "[CAL] Mag hard-iron offset saved to Flash:" in line:
            m = re.search(r"saved to Flash:\s*(-?[\d\.]+),\s*(-?[\d\.]+),\s*(-?[\d\.]+)", line)
            if m:
                self.board_mag_offsets = [float(m.group(1)), float(m.group(2)), float(m.group(3))]

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
        self.root.destroy()
        
    def send_command(self, cmd_str):
        if not self.running or not hasattr(self, 'ser') or not self.ser:
            messagebox.showwarning("警告", "串口未連接！請先連線。")
            return False
        try:
            if not cmd_str.endswith('\n'):
                cmd_str += '\n'
            self.ser.write(cmd_str.encode('utf-8'))
            self.ser.flush()
            # 輸出至本地終端 console
            self.console.insert(tk.END, f"[CMD] ➡️ {cmd_str.strip()}\n", "rate")
            self.console.see(tk.END)
            return True
        except Exception as e:
            messagebox.showerror("錯誤", f"發送指令失敗: {e}")
            return False

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
        
        btn_yaw_frame = tk.Frame(right_frame, bg="#222222")
        btn_yaw_frame.pack(fill=tk.X, padx=15, pady=5)
        
        ttk.Button(btn_yaw_frame, text="啟用磁北鎖定", command=lambda: self.toggle_mag_yaw_lock(True)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,5))
        ttk.Button(btn_yaw_frame, text="停用磁北鎖定", command=lambda: self.toggle_mag_yaw_lock(False)).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(5,0))
        
        ttk.Button(right_frame, text="重置硬鐵偏置 (預設 131072)", command=self.reset_mag_calibration).pack(fill=tk.X, padx=15, pady=10)
        
        # 啟動定時繪圖更新
        self.update_calib_plot()

    def toggle_collecting(self):
        self.collecting_data = not self.collecting_data
        if self.collecting_data:
            self.btn_toggle_collect.config(text="停止收集數據")
            self.calib_x = []
            self.calib_y = []
            self.lbl_fit_results.config(text="擬合結果:\n  (正在收集數據，請旋轉航電板一整圈...)", fg="#ffcc00")
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

    def fit_mag_circle(self):
        if len(self.calib_x) < 10:
            messagebox.showwarning("警告", "收集的數據點太少（至少需要10點以上，建議順時針/逆時針旋轉一整圈）。")
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
            
            current_ox = self.board_mag_offsets[0]
            current_oy = self.board_mag_offsets[1]
            
            new_ox = current_ox + cx_fit * 16.384
            new_oy = current_oy - cy_fit * 16.384
            
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
            messagebox.showerror("錯誤", f"擬合計算失敗: {e}")

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
        
        cmd = f"CMD_MAG_CAL:{self.new_ox:.1f},{self.new_oy:.1f},{new_oz:.1f}"
        if self.send_command(cmd):
            messagebox.showinfo("成功", f"地磁偏置寫入指令已發送！\n偏置: X={int(self.new_ox)}, Y={int(self.new_oy)}, Z={int(new_oz)}")
            self.lbl_board_offsets.config(
                text=f"當前板載偏置 (Counts):\n  X={int(self.new_ox)}, Y={int(self.new_oy)}, Z={int(new_oz)}"
            )

    def toggle_mag_yaw_lock(self, enable):
        val = 1 if enable else 0
        cmd = f"CMD_MAG_YAW_LOCK:{val}"
        if self.send_command(cmd):
            status = "啟用" if enable else "停用"
            messagebox.showinfo("成功", f"EKF 絕對磁北鎖定{status}指令已發送！")

    def reset_mag_calibration(self):
        if messagebox.askyesno("確認", "是否確定要重置地磁偏置至預設值 (131072)？"):
            cmd = "CMD_MAG_CAL:131072,131072,131072"
            if self.send_command(cmd):
                self.board_mag_offsets = [131072.0, 131072.0, 131072.0]
                self.lbl_board_offsets.config(
                    text=f"當前板載偏置 (Counts):\n  X=131072, Y=131072, Z=131072"
                )
                self.clear_calib_data()
                messagebox.showinfo("成功", "已發送重置地磁偏置指令！")

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
        "title": "【測試 1/9】BMI088 加速度計 Z 軸 (靜態)",
        "prompt": "請將航電板【水平靜止放置於桌面，正面朝上】(Z 軸朝上)。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "IMU",
        "run": lambda val: 800.0 <= val["az"] <= 1200.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：Z 軸加速度 = {val['az']:.1f} mG\n" + (
            "✅ [PASS] Z 軸方向正確（朝上）！" if ok else
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
        "title": "【測試 3/9】BMI088 加速度計 Y 軸 (鼻錐朝上)",
        "prompt": "請將航電板【前緣/鼻錐抬高約 45 度】(前緣朝上，body 前軸朝天)。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "IMU",
        "run": lambda val: val["ay"] > 200.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：Y 軸加速度 = {val['ay']:.1f} mG\n" + (
            "✅ [PASS] 鼻錐朝上時 ay 為正，Y=前 對齊正確！" if ok else
            "❌ [FAIL] 鼻錐朝上時 ay 不為正！\n💡 建議：晶片貼裝與 sensor_axis.h 表格不符，請核對 IMU 映射 (Y->X)。"
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
        "prompt": "請準備將航電板【快速抬頭/後仰】(前端/鼻錐朝上抬起)。\n按下「開始檢測」後，請【立即快速將板子前半部朝上抬起旋轉約 2 秒】...",
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
        "prompt": "請準備將航電板【快速向右翻滾】(右側向下傾斜)。\n按下「開始檢測」後，請【立即快速將板子向右翻滾】...",
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
        "title": "【測試 8/9】MMC5983MA 地磁計航向角 (鼻錐朝北 ≈ 0°)",
        "prompt": "請將航電板水平靜置，並將【鼻錐朝向正北方】。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "MAG",
        "run": lambda val: val["hdg"] <= 30.0 or val["hdg"] >= 330.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：地磁向量 B = [{val['mx']:.1f}, {val['my']:.1f}] mG, 航向角 hdg = {val['hdg']:.1f}°\n" + (
            f"✅ [PASS] 鼻錐朝北，航向角 {val['hdg']:.1f}° ≈ 0°，對齊正確！" if ok else
            f"❌ [FAIL] 鼻錐朝北，航向角 {val['hdg']:.1f}°，預期 ≈ 0°（±30°）。\n💡 建議：在 main.c 調整磁力計 mx_body/my_body 的來源軸。"
        )
    },
    {
        "step": 9,
        "title": "【測試 9/9】MMC5983MA 地磁計航向角收斂方向",
        "prompt": "請將航電板【順時針旋轉 90 度】(鼻錐朝向正東方)。\n置妥後，請按「開始檢測」...",
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

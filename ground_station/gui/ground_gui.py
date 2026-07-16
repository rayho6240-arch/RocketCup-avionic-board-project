#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RocketCom Ground Station Real-time Telemetry Monitor GUI
- Dark Mode styling.
- Real-time updates of rocket EKF/baro altitude, 3-axis acceleration (with magnitude calculation), battery voltage, FSM flight state, and GPS fix.
- Ground station hardware diagnostics (Local GPS receiver status and LoRa 433/920 module statistics).
- Scrolled text console with keyword color highlighting.
- Thread-safe queue communication between serial reader thread and Tkinter main loop.
"""

import sys
import os
import time
import re
import queue
import threading
import math
from datetime import datetime

# GUI / Tkinter imports
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[*] 正在自動安裝 pyserial...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
    import serial
    import serial.tools.list_ports

# Add parent directory to sys.path to load serial_link
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import serial_link   # Unified serial utility

# FSM state code to string and colors (aligned with firmware FSM enum)
FSM_STATES = {
    0: ("INIT/BOOT", "#8E8D8A"),
    1: ("PREFLIGHT/PAD", "#9E9E9E"),
    2: ("BOOST", "#FF5722"),
    3: ("COAST", "#FFC107"),
    4: ("APOGEE/DROGUE", "#00BCD4"),
    5: ("DESCENT", "#A8DADC"),
    6: ("MAIN_DEPLOY", "#2196F3"),
    7: ("LANDED", "#4CAF50")
}

class GroundMonitorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RocketCom 地面監測與遙測分析儀")
        self.root.geometry("1100x750")
        self.root.configure(bg="#121212")

        # Serial state variables
        self.serial_port = tk.StringVar()
        self.baud_rate = tk.IntVar(value=460800)
        self.is_connected = False
        self.ser = None
        self.rx_thread = None
        self.data_queue = queue.Queue()

        # Device auto-detect state
        self.device_role = "AUTO"  # "AUTO", "GROUND", or "AVIONICS"
        self.cpu_main = "0.0"
        self.cpu_ekf = "0.0"
        self.sensor_health_bits = 0
        self.ekf_health_bits = 0

        # Telemetry parsing state
        self.rocket_state = {
            "seq": 0,
            "fsm": 0,
            "alt_m": 0.0,
            "baro_m": 0.0,
            "bat_v": 0.0,
            "gps_sats": 0,
            "gps_fix": 0,
            "ax_g": 0.0,
            "ay_g": 0.0,
            "az_g": 0.0,
            "accel_mag": 0.0,
            "rssi": 0,
            "snr": 0,
            "link": "433MHz",
            "last_pkt_time": None
        }

        self.gs_state = {
            "hw_433": "OFF",
            "hw_920": "OFF",
            "raw_433": 0,
            "ok_433": 0,
            "crc_433": 0,
            "rsync_433": 0,
            "ok_920": 0,
            "crc_920": 0,
            "rsync_920": 0,
            "pkts_433": 0,
            "pkts_920": 0,
            "gps_sats": 0,
            "gps_q": 0,
            "gps_ok": 0,
            "gps_err": 0,
            "gps_fix": False,
            "gps_lat": 0.0,
            "gps_lon": 0.0,
            "gps_alt": 0
        }

        # Apply dark theme styles
        self.setup_styles()
        self.build_ui()

        # Auto-detect serial ports on start
        self.refresh_ports()

        # Start periodic GUI queue polling
        self.root.after(100, self.poll_queue)

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        
        # Dark Theme Palette
        style.configure(".", bg="#121212", fg="#EEEEEE", fieldbackground="#1E1E1E")
        style.configure("TFrame", background="#121212")
        style.configure("Card.TFrame", background="#1E1E1E", relief="flat")
        
        style.configure("TLabel", background="#121212", foreground="#EEEEEE", font=("Arial", 10))
        style.configure("CardTitle.TLabel", background="#1E1E1E", foreground="#888888", font=("Arial", 9, "bold"))
        style.configure("CardValue.TLabel", background="#1E1E1E", foreground="#FFFFFF", font=("Arial", 22, "bold"))
        style.configure("CardSub.TLabel", background="#1E1E1E", foreground="#00ADB5", font=("Arial", 9))
        
        style.configure("TCombobox", background="#1E1E1E", foreground="#EEEEEE", fieldbackground="#1E1E1E", bordercolor="#2D2D2D")
        style.map("TCombobox", fieldbackground=[('readonly', '#1E1E1E')], foreground=[('readonly', '#EEEEEE')])

        style.configure("Connect.TButton", font=("Arial", 10, "bold"), background="#00ADB5", foreground="#FFFFFF")
        style.map("Connect.TButton", background=[('active', '#007A78')])

    def build_ui(self):
        # 1. 頂部控制欄 (Connection control)
        top_bar = ttk.Frame(self.root, padding=10)
        top_bar.pack(fill="x", side="top")

        ttk.Label(top_bar, text="序列埠:").pack(side="left", padx=5)
        self.cb_port = ttk.Combobox(top_bar, textvariable=self.serial_port, width=25, state="normal")
        self.cb_port.pack(side="left", padx=5)

        btn_refresh = tk.Button(top_bar, text="整理", bg="#2D2D2D", fg="#EEEEEE", relief="flat", command=self.refresh_ports)
        btn_refresh.pack(side="left", padx=5)

        ttk.Label(top_bar, text="波特率:").pack(side="left", padx=5)
        self.cb_baud = ttk.Combobox(top_bar, textvariable=self.baud_rate, values=[9600, 38400, 115200, 460800], width=8, state="readonly")
        self.cb_baud.pack(side="left", padx=5)

        self.btn_connect = tk.Button(top_bar, text="連線", bg="#00ADB5", fg="#FFFFFF", relief="flat", font=("Arial", 10, "bold"), width=10, command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=15)

        self.lbl_status = ttk.Label(top_bar, text="未連線", foreground="#E91E63", font=("Arial", 10, "bold"))
        self.lbl_status.pack(side="left", padx=5)

        self.lbl_role = ttk.Label(top_bar, text="連接設備: 自動偵測中...", foreground="#FFC107", font=("Arial", 10, "bold"))
        self.lbl_role.pack(side="left", padx=15)

        # 2. 中部儀表板卡片區
        dash_frame = ttk.Frame(self.root, padding=10)
        dash_frame.pack(fill="x", side="top")

        # 創造火箭遙測卡片 (Grid layout for cards)
        # Card 0: 飛行狀態機 FSM
        self.card_fsm = self.create_card(dash_frame, 0, 0, "火箭飛行階段 (FSM)")
        # Card 1: 融合高度/氣壓高度
        self.card_alt = self.create_card(dash_frame, 0, 1, "火箭高度")
        # Card 2: 3軸加速度/合力
        self.card_accel = self.create_card(dash_frame, 0, 2, "火箭加速度 (g)")
        # Card 3: 火箭電池電壓
        self.card_bat = self.create_card(dash_frame, 0, 3, "火箭電池電壓")
        # Card 4: 火箭 GPS 定位
        self.card_gps = self.create_card(dash_frame, 1, 0, "火箭 GPS 定位")
        # Card 5: LoRa 無線鏈路 RSSI
        self.card_link = self.create_card(dash_frame, 1, 1, "遙測收訊強度 (RSSI)")
        # Card 6: 地面站 GPS 狀態
        self.card_gs_gps = self.create_card(dash_frame, 1, 2, "地面站本機 GPS")
        # Card 7: 地面站 LoRa 硬體包統計
        self.card_gs_stat = self.create_card(dash_frame, 1, 3, "地面站通訊統計")

        self.update_ui_cards()

        # 3. 底部終端 log 欄
        log_frame = ttk.Frame(self.root, padding=10)
        log_frame.pack(fill="both", expand=True, side="bottom")
        
        ttk.Label(log_frame, text="地面站調試日誌 (調試串口輸出):", font=("Arial", 9, "bold")).pack(anchor="w", pady=2)
        
        self.txt_console = ScrolledText(log_frame, bg="#0E0E0E", fg="#CCCCCC", font=("Courier New", 10), insertbackground="white", relief="flat")
        self.txt_console.pack(fill="both", expand=True)

        # Setup Tag Highlight colors
        self.txt_console.tag_config("GS_PKT", foreground="#4CAF50")      # Green
        self.txt_console.tag_config("GS_PKT_BAD", foreground="#FF5722")  # Orange-red (CRC_BAD)
        self.txt_console.tag_config("GS_STAT", foreground="#00BCD4")     # Cyan
        self.txt_console.tag_config("GS_GPS", foreground="#FFC107")      # Yellow
        self.txt_console.tag_config("INFO", foreground="#888888")        # Muted grey

    def create_card(self, parent, row, col, title):
        card = ttk.Frame(parent, style="Card.TFrame", padding=10)
        card.grid(row=row, column=col, padx=6, pady=6, sticky="nsew")
        parent.grid_columnconfigure(col, weight=1)

        lbl_title = ttk.Label(card, text=title, style="CardTitle.TLabel")
        lbl_title.pack(anchor="w", pady=2)

        lbl_val = ttk.Label(card, text="--", style="CardValue.TLabel")
        lbl_val.pack(anchor="w", pady=2)

        lbl_sub = ttk.Label(card, text="--", style="CardSub.TLabel")
        lbl_sub.pack(anchor="w", pady=2)

        return {"frame": card, "title": lbl_title, "val": lbl_val, "sub": lbl_sub}

    def refresh_ports(self):
        ports = serial_link.list_candidate_ports()
        self.cb_port['values'] = ports
        if ports:
            # Prefer USB Serial ports
            pref = [p for p in ports if "usbserial" in p or "usbmodem" in p or "tty.usb" in p]
            if pref:
                self.serial_port.set(pref[0])
            else:
                self.serial_port.set(ports[0])
        else:
            self.serial_port.set("")

    def toggle_connection(self):
        if not self.is_connected:
            port = self.serial_port.get()
            if not port:
                messagebox.showerror("錯誤", "未偵測到任何可用序列埠！")
                return
            baud = self.baud_rate.get()
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.is_connected = True
                self.btn_connect.config(text="中斷", bg="#E91E63")
                self.lbl_status.config(text=f"已連線: {os.path.basename(port)}", foreground="#4CAF50")
                self.txt_console.insert(tk.END, f"[{datetime.now().strftime('%H:%M:%S')}] 連線開啟 {port} @ {baud} baud\n", "INFO")
                self.txt_console.see(tk.END)

                # Start background receiver thread
                self.rx_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
                self.rx_thread.start()
            except Exception as e:
                messagebox.showerror("連線失敗", f"無法開啟 {port}: {e}")
        else:
            self.disconnect_serial()

    def disconnect_serial(self):
        self.is_connected = False
        self.device_role = "AUTO"
        self.lbl_role.config(text="連接設備: 自動偵測中...", foreground="#FFC107")
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.btn_connect.config(text="連線", bg="#00ADB5")
        self.lbl_status.config(text="未連線", foreground="#E91E63")
        self.txt_console.insert(tk.END, f"[{datetime.now().strftime('%H:%M:%S')}] 序列埠已中斷關閉\n", "INFO")
        self.txt_console.see(tk.END)

    def serial_read_loop(self):
        while self.is_connected:
            try:
                if self.ser.in_waiting:
                    raw_line = self.ser.readline()
                    if raw_line:
                        line = raw_line.decode('utf-8', errors='replace').strip()
                        if line:
                            self.data_queue.put(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                # If error, close port and post error
                self.data_queue.put(f"[ERR] 讀取異常中斷: {e}")
                break

    def poll_queue(self):
        """Consume lines from queue, parse them, and print to text console"""
        # Limit lines processed per tick to keep GUI responsive
        for _ in range(50):
            if self.data_queue.empty():
                break
            line = self.data_queue.get()
            if line.startswith("[ERR]"):
                self.disconnect_serial()
                messagebox.showerror("錯誤", line)
                break
            
            # Format timestamp
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            log_line = f"[{ts}] {line}\n"

            # Parse line by pattern matching
            tag = None
            
            # --- Device Role Auto-detection ---
            if self.device_role == "AUTO":
                if any(x in line for x in ["[GS_PKT]", "[GS_STAT]", "[GS_GPS]", "ROLE=ROLE_GROUND"]):
                    self.device_role = "GROUND"
                    self.lbl_role.config(text="連接設備: 地面站接收端", foreground="#00ADB5")
                elif any(x in line for x in ["[TELE]", "[RATE]", "[CPU]", "[GPS]", "[HEALTH]", "ROLE=ROLE_PRIMARY", "ROLE=ROLE_BACKUP"]):
                    self.device_role = "AVIONICS"
                    self.lbl_role.config(text="連接設備: 航電板 (直連)", foreground="#4CAF50")

            # --- Ground Station Mode parsing ---
            if "[GS_PKT]" in line:
                tag = "GS_PKT"
                self.parse_gs_pkt(line)
            elif "CRC_BAD" in line or "[GS_PKT?]" in line:
                tag = "GS_PKT_BAD"
                self.parse_gs_pkt_bad(line)
            elif "[GS_STAT]" in line:
                tag = "GS_STAT"
                self.parse_gs_stat(line)
            elif "[GS_GPS]" in line:
                tag = "GS_GPS"
                self.parse_gs_gps(line)

            # --- Direct Avionics Mode parsing ---
            elif "[TELE]" in line:
                tag = "GS_PKT"
                m_pos = re.search(r"pos:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
                if m_pos:
                    self.rocket_state["alt_m"] = float(m_pos.group(3))
                    self.rocket_state["last_pkt_time"] = time.time()
            elif "[GPS]" in line:
                tag = "GS_GPS"
                m_gps = re.search(r"fix:(\d+)\s+q:\d+\s+sat:(\d+)", line)
                if m_gps:
                    self.rocket_state["gps_fix"] = int(m_gps.group(1))
                    self.rocket_state["gps_sats"] = int(m_gps.group(2))
                    self.rocket_state["last_pkt_time"] = time.time()
            elif "[IMU]" in line:
                tag = "INFO"
                if self.device_role == "AVIONICS":
                    m_imu = re.search(r"a\[mG\]:(-?\d+),(-?\d+),(-?\d+)", line)
                    if m_imu:
                        self.rocket_state["ax_g"] = float(m_imu.group(1)) / 1000.0
                        self.rocket_state["ay_g"] = float(m_imu.group(2)) / 1000.0
                        self.rocket_state["az_g"] = float(m_imu.group(3)) / 1000.0
                        self.rocket_state["accel_mag"] = math.sqrt(self.rocket_state["ax_g"]**2 + self.rocket_state["ay_g"]**2 + self.rocket_state["az_g"]**2)
                        self.rocket_state["last_pkt_time"] = time.time()
            elif "[HIGHG]" in line:
                tag = "INFO"
                if self.device_role == "AVIONICS":
                    m_hg = re.search(r"a\[mG\]:(-?\d+),(-?\d+),(-?\d+)", line)
                    if m_hg:
                        self.rocket_state["ax_g"] = float(m_hg.group(1)) / 1000.0
                        self.rocket_state["ay_g"] = float(m_hg.group(2)) / 1000.0
                        self.rocket_state["az_g"] = float(m_hg.group(3)) / 1000.0
                        self.rocket_state["accel_mag"] = math.sqrt(self.rocket_state["ax_g"]**2 + self.rocket_state["ay_g"]**2 + self.rocket_state["az_g"]**2)
                        self.rocket_state["last_pkt_time"] = time.time()
            elif "[HEALTH]" in line:
                tag = "GS_STAT"
                m_fsm = re.search(r"fsm=(\d+)", line)
                if m_fsm:
                    self.rocket_state["fsm"] = int(m_fsm.group(1))
                    self.rocket_state["last_pkt_time"] = time.time()
                m_health = re.search(r"sens=0x([0-9A-Fa-f]+)\s+ekf=0x([0-9A-Fa-f]+)", line)
                if m_health:
                    self.sensor_health_bits = int(m_health.group(1), 16)
                    self.ekf_health_bits = int(m_health.group(2), 16)
            elif "[FSM]" in line:
                tag = "INFO"
                m_state = re.search(r"Entering (STATE_[A-Z_]+)", line)
                if m_state:
                    state_name = m_state.group(1)
                    name_to_code = {
                        "STATE_INIT": 0, "STATE_PAD": 1, "STATE_BOOST": 2, "STATE_COAST": 3,
                        "STATE_APOGEE": 4, "STATE_DESCENT": 5, "STATE_MAIN_DEPLOY": 6, "STATE_LANDED": 7
                    }
                    if state_name in name_to_code:
                        self.rocket_state["fsm"] = name_to_code[state_name]
                        self.rocket_state["last_pkt_time"] = time.time()
                elif "LIFTOFF" in line:
                    self.rocket_state["fsm"] = 2
                    self.rocket_state["last_pkt_time"] = time.time()
                elif "BURNOUT" in line:
                    self.rocket_state["fsm"] = 3
                    self.rocket_state["last_pkt_time"] = time.time()
                elif "APOGEE" in line or "DROGUE" in line:
                    self.rocket_state["fsm"] = 5  # DESCENT
                    self.rocket_state["last_pkt_time"] = time.time()
                elif "MAIN" in line and "deployed" in line:
                    self.rocket_state["fsm"] = 6
                    self.rocket_state["last_pkt_time"] = time.time()
                elif "TOUCHDOWN" in line:
                    self.rocket_state["fsm"] = 7
                    self.rocket_state["last_pkt_time"] = time.time()
            elif "[CPU]" in line:
                tag = "INFO"
                m_cpu = re.search(r"MainTask\+ISR:([\d\.]+)%,\s*EKFTask:([\d\.]+)%", line)
                if m_cpu:
                    self.cpu_main = m_cpu.group(1)
                    self.cpu_ekf = m_cpu.group(2)
            elif "," in line and not line.startswith("["):
                parts = line.split(',')
                if len(parts) == 9:
                    try:
                        self.rocket_state["baro_m"] = float(parts[8]) / 100.0
                        self.rocket_state["last_pkt_time"] = time.time()
                    except ValueError:
                        pass

            # Insert into terminal scrolling box
            self.txt_console.insert(tk.END, log_line, tag)
            
            # Auto scroll and limit history (e.g. max 500 lines)
            num_lines = float(self.txt_console.index('end-1c').split('.')[0])
            if num_lines > 500:
                self.txt_console.delete('1.0', '2.0')
            self.txt_console.see(tk.END)

        # Periodically refresh telemetry cards
        self.update_ui_cards()
        self.root.after(50, self.poll_queue)

    def parse_gs_pkt(self, line):
        # Format: [GS_PKT] link:433MHz rssi:-32 snr:-32768 seq:108 fsm:1 alt:-65cm baro:3426cm bat:7419mV gps:0/0 accel:0,0,0
        m_link = re.search(r"link:(\d+MHz)", line)
        m_rssi = re.search(r"rssi:(-?\d+)", line)
        m_snr  = re.search(r"snr:(-?\d+)", line)
        m_seq  = re.search(r"seq:(\d+)", line)
        m_fsm  = re.search(r"fsm:(\d+)", line)
        m_alt  = re.search(r"alt:(-?\d+)cm", line)
        m_baro = re.search(r"baro:(-?\d+)cm", line)
        m_bat  = re.search(r"bat:(\d+)mV", line)
        
        # New P1 parameters
        m_gps   = re.search(r"gps:(\d+)/(\d+)", line)
        m_accel = re.search(r"accel:(-?\d+),(-?\d+),(-?\d+)", line)

        if m_link: self.rocket_state["link"] = m_link.group(1)
        if m_rssi: self.rocket_state["rssi"] = int(m_rssi.group(1))
        if m_snr:  self.rocket_state["snr"]  = int(m_snr.group(1))
        if m_seq:  self.rocket_state["seq"]  = int(m_seq.group(1))
        if m_fsm:  self.rocket_state["fsm"]  = int(m_fsm.group(1))
        
        if m_alt:  self.rocket_state["alt_m"] = float(m_alt.group(1)) / 100.0
        if m_baro: self.rocket_state["baro_m"] = float(m_baro.group(1)) / 100.0
        if m_bat:  self.rocket_state["bat_v"] = float(m_bat.group(1)) / 100.0

        if m_gps:
            self.rocket_state["gps_sats"] = int(m_gps.group(1))
            self.rocket_state["gps_fix"]  = int(m_gps.group(2))
        
        if m_accel:
            ax = float(m_accel.group(1)) / 1000.0
            ay = float(m_accel.group(2)) / 1000.0
            az = float(m_accel.group(3)) / 1000.0
            self.rocket_state["ax_g"] = ax
            self.rocket_state["ay_g"] = ay
            self.rocket_state["az_g"] = az
            self.rocket_state["accel_mag"] = math.sqrt(ax**2 + ay**2 + az**2)

        self.rocket_state["last_pkt_time"] = time.time()

    def parse_gs_pkt_bad(self, line):
        pass

    def parse_gs_gps(self, line):
        # SEARCHING format: [GS_GPS] SEARCHING sats=0 q=0 ok=0 err=0
        # FIX format: [GS_GPS] FIX sats=5 Pos:+25.123456,+121.567890 Alt:45m
        if "SEARCHING" in line:
            m = re.search(r"sats=(\d+)\s+q=(\d+)\s+ok=(\d+)\s+err=(\d+)", line)
            if m:
                self.gs_state["gps_sats"] = int(m.group(1))
                self.gs_state["gps_q"] = int(m.group(2))
                self.gs_state["gps_ok"] = int(m.group(3))
                self.gs_state["gps_err"] = int(m.group(4))
                self.gs_state["gps_fix"] = False
        elif "FIX" in line:
            m = re.search(r"sats=(\d+)\s+Pos:([+-]?[\d\.]+),([+-]?[\d\.]+)\s+Alt:(\d+)m", line)
            if m:
                self.gs_state["gps_sats"] = int(m.group(1))
                self.gs_state["gps_lat"] = float(m.group(2))
                self.gs_state["gps_lon"] = float(m.group(3))
                self.gs_state["gps_alt"] = int(m.group(4))
                self.gs_state["gps_fix"] = True

    def parse_gs_stat(self, line):
        # Format: [GS_STAT] HW:433=OK 920=OFF | 433 raw=5742 ok=53 crc=17 rsync=0 | 920 ok=0 crc=0 ...
        m_hw433 = re.search(r"HW:433=(\w+)", line)
        m_hw920 = re.search(r"920=(\w+)", line)
        
        m_433 = re.search(r"433 raw=(\d+)\s+ok=(\d+)\s+crc=(\d+)\s+rsync=(\d+)", line)
        m_920 = re.search(r"920 ok=(\d+)\s+crc=(\d+)\s+rsync=(\d+)", line)

        m_pkts = re.search(r"pkts 433=(\d+)\s+920=(\d+)", line)

        if m_hw433: self.gs_state["hw_433"] = m_hw433.group(1)
        if m_hw920: self.gs_state["hw_920"] = m_hw920.group(1)

        if m_433:
            self.gs_state["raw_433"]   = int(m_433.group(1))
            self.gs_state["ok_433"]    = int(m_433.group(2))
            self.gs_state["crc_433"]   = int(m_433.group(3))
            self.gs_state["rsync_433"] = int(m_433.group(4))

        if m_920:
            self.gs_state["ok_920"]    = int(m_920.group(1))
            self.gs_state["crc_920"]   = int(m_920.group(2))
            self.gs_state["rsync_920"] = int(m_920.group(3))

        if m_pkts:
            self.gs_state["pkts_433"] = int(m_pkts.group(1))
            self.gs_state["pkts_920"] = int(m_pkts.group(2))

    def update_ui_cards(self):
        # 1. Rocket FSM State
        state_code = self.rocket_state["fsm"]
        state_str, state_color = FSM_STATES.get(state_code, ("UNKNOWN", "#8E8D8A"))
        
        # Check rocket heartbeat timeout
        if self.rocket_state["last_pkt_time"] is None:
            self.card_fsm["val"].config(text="未偵測到訊號", foreground="#888888")
            self.card_fsm["sub"].config(text="等待首個 telemetry 封包...")
        elif time.time() - self.rocket_state["last_pkt_time"] > 2.0:
            self.card_fsm["val"].config(text="遙測失聯", foreground="#E91E63")
            self.card_fsm["sub"].config(text=f"最後接收: {int(time.time() - self.rocket_state['last_pkt_time'])} 秒前")
        else:
            self.card_fsm["val"].config(text=state_str, foreground=state_color)
            self.card_fsm["sub"].config(text=f"封包序號 seq={self.rocket_state['seq']}")

        # 2. Rocket Altitude
        if self.rocket_state["last_pkt_time"] and time.time() - self.rocket_state["last_pkt_time"] <= 2.0:
            self.card_alt["val"].config(text=f"{self.rocket_state['alt_m']:.2f} m", foreground="#00ADB5")
            self.card_alt["sub"].config(text=f"氣壓計高度: {self.rocket_state['baro_m']:.2f} m")
        else:
            self.card_alt["val"].config(text="--", foreground="#888888")
            self.card_alt["sub"].config(text="氣壓計高度: --")

        # 3. Rocket Acceleration
        if self.rocket_state["last_pkt_time"] and time.time() - self.rocket_state["last_pkt_time"] <= 2.0:
            # Display Accel magnitude and components
            self.card_accel["val"].config(text=f"{self.rocket_state['accel_mag']:.2f} g", foreground="#4CAF50")
            self.card_accel["sub"].config(text=f"X:{self.rocket_state['ax_g']:+.2f}  Y:{self.rocket_state['ay_g']:+.2f}  Z:{self.rocket_state['az_g']:+.2f}")
        else:
            self.card_accel["val"].config(text="--", foreground="#888888")
            self.card_accel["sub"].config(text="X:--  Y:--  Z:--")

        # 4. Rocket Battery
        if self.rocket_state["last_pkt_time"] and time.time() - self.rocket_state["last_pkt_time"] <= 2.0:
            bat_v = self.rocket_state["bat_v"]
            bat_color = "#4CAF50" if bat_v >= 7.4 else ("#FFC107" if bat_v >= 6.8 else "#E91E63")
            self.card_bat["val"].config(text=f"{bat_v:.2f} V", foreground=bat_color)
            self.card_bat["sub"].config(text="2S LiPo 狀態")
        else:
            self.card_bat["val"].config(text="--", foreground="#888888")
            self.card_bat["sub"].config(text="--")

        # 5. Rocket GPS
        if self.rocket_state["last_pkt_time"] and time.time() - self.rocket_state["last_pkt_time"] <= 2.0:
            fix_str = "● 已定位" if self.rocket_state["gps_fix"] else "○ 搜星中"
            fix_color = "#4CAF50" if self.rocket_state["gps_fix"] else "#FFC107"
            self.card_gps["val"].config(text=fix_str, foreground=fix_color)
            self.card_gps["sub"].config(text=f"衛星數量: {self.rocket_state['gps_sats']} sats")
        else:
            self.card_gps["val"].config(text="--", foreground="#888888")
            self.card_gps["sub"].config(text="衛星數量: --")

        # 6. LoRa Link quality
        if self.device_role == "AVIONICS":
            self.card_link["val"].config(text="N/A", foreground="#888888")
            self.card_link["sub"].config(text="工作鏈路: 直連偵錯串口")
        elif self.rocket_state["last_pkt_time"] and time.time() - self.rocket_state["last_pkt_time"] <= 2.0:
            rssi = self.rocket_state["rssi"]
            link_color = "#4CAF50" if rssi >= -70 else ("#FFC107" if rssi >= -95 else "#E91E63")
            self.card_link["val"].config(text=f"{rssi} dBm", foreground=link_color)
            self.card_link["sub"].config(text=f"工作鏈路: {self.rocket_state['link']}")
        else:
            self.card_link["val"].config(text="--", foreground="#888888")
            self.card_link["sub"].config(text="--")

        # 7. Ground Station Local GPS or Avionics CPU Load
        if self.device_role == "AVIONICS":
            self.card_gs_gps["title"].config(text="航電板 CPU 負載")
            self.card_gs_gps["val"].config(text=f"Main: {self.cpu_main}%", foreground="#00ADB5")
            self.card_gs_gps["sub"].config(text=f"EKF Task: {self.cpu_ekf}%")
        else:
            self.card_gs_gps["title"].config(text="地面站本機 GPS")
            if self.gs_state["gps_fix"]:
                self.card_gs_gps["val"].config(text="● 已定位", foreground="#4CAF50")
                self.card_gs_gps["sub"].config(text=f"sats:{self.gs_state['gps_sats']}  H:{self.gs_state['gps_alt']}m  GPS時鐘同步中")
            else:
                gs_gps_text = "○ 搜星中" if self.gs_state["gps_sats"] > 0 else "○ 無訊號"
                # If ok=0 and err=0, meaning no serial packets at all
                if self.gs_state["gps_ok"] == 0 and self.gs_state["gps_err"] == 0:
                    gs_gps_text = "⚠ 硬體斷線"
                    self.card_gs_gps["val"].config(text=gs_gps_text, foreground="#E91E63")
                    self.card_gs_gps["sub"].config(text="PC7 RX 未接線或無供電")
                else:
                    self.card_gs_gps["val"].config(text=gs_gps_text, foreground="#FFC107")
                    self.card_gs_gps["sub"].config(text=f"sats:{self.gs_state['gps_sats']}  ok:{self.gs_state['gps_ok']}  err:{self.gs_state['gps_err']}")

        # 8. Ground Station Communication Stats or Avionics Health Bits
        if self.device_role == "AVIONICS":
            self.card_gs_stat["title"].config(text="航電板健康狀態")
            sens_color = "#4CAF50" if self.sensor_health_bits == 0 else "#E91E63"
            self.card_gs_stat["val"].config(text=f"Sens: 0x{self.sensor_health_bits:02X}", foreground=sens_color)
            ekf_color = "#4CAF50" if self.ekf_health_bits == 0 else "#FFC107"
            self.card_gs_stat["sub"].config(text=f"EKF健康碼: 0x{self.ekf_health_bits:02X}")
        else:
            self.card_gs_stat["title"].config(text="地面站通訊統計")
            hw_text = f"433:{self.gs_state['hw_433']} 920:{self.gs_state['hw_920']}"
            self.card_gs_stat["val"].config(text=f"{self.gs_state['ok_433']} Pkts", foreground="#00BCD4")
            self.card_gs_stat["sub"].config(text=f"硬體:{hw_text} | CRC錯誤:{self.gs_state['crc_433']}")

def main():
    import argparse
    parser = argparse.ArgumentParser(description="RocketCom Ground Station GUI")
    parser.add_argument("--port", type=str, default=None, help="Serial port to open")
    parser.add_argument("--baud", type=int, default=460800, help="Serial baud rate")
    args = parser.parse_args()

    root = tk.Tk()
    app = GroundMonitorGUI(root)
    
    if args.port:
        app.serial_port.set(args.port)
        app.baud_rate.set(args.baud)
        # Auto-connect if port is provided via CLI
        app.root.after(500, app.toggle_connection)
    
    # Graceful exit on window close
    def on_closing():
        app.disconnect_serial()
        root.destroy()
        sys.exit(0)
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    root.mainloop()

if __name__ == "__main__":
    main()

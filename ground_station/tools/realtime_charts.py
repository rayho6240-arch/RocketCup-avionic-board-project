#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RocketCom Comprehensive Real-time Avionics Dashboard (Matplotlib HIL Visualizer)
===========================================================================
Displays:
  1. Sensor Raw Data: 3-axis gyro (dps) and Z-axis accelerations (g)
  2. EKF State: Altitude (m) vs Baro Altitude (m), Vertical Velocity (m/s)
  3. FSM State: Flight State Machine transitions & current state
  4. System Stats: Battery voltage, GPS status, Sensor/EKF health bits
"""

import sys
import os
import time
import re
import queue
import threading
import math
from collections import deque

# Check dependencies
try:
    import serial
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import numpy as np
except ImportError:
    print("[*] 偵測到缺失套件，正在自動安裝 pyserial 與 matplotlib...")
    import subprocess
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial", "matplotlib", "numpy"])
        import serial
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
        import numpy as np
    except Exception as e:
        print(f"[-] 安裝失敗，請手動執行: pip install pyserial matplotlib numpy (錯誤: {e})")
        sys.exit(1)

# Add parent directory to sys.path to load serial_link
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import serial_link   # Auto-detect serial ports and default baudrate

# Global Configuration
MAX_POINTS = 150     # History window size for scrolling plot
BAUD_RATE = 460800   # Avionics default baud rate

# Regex patterns for parsing UART data stream
tele_pattern = re.compile(
    r"\[TELE\] pos:([\d\.-]+),([\d\.-]+),([\d\.-]+) "
    r"vel:([\d\.-]+),([\d\.-]+),([\d\.-]+) "
    r"q:([\d\.-]+),([\d\.-]+),([\d\.-]+),([\d\.-]+)"
)
imu_pattern = re.compile(
    r"\[IMU\] a\[mG\]:(-?\d+),(-?\d+),(-?\d+) g\[dps\]:(-?\d+),(-?\d+),(-?\d+)"
)
pwr_pattern = re.compile(r"\[PWR\] bat:(\d+)mV")
gps_pattern = re.compile(r"\[GPS\] fix:(\d+) q:\d+ sat:(\d+)")
health_pattern = re.compile(r"\[HEALTH\] sens=(0x[0-9a-fA-F]+|\d+)\s+ekf=(0x[0-9a-fA-F]+|\d+)")

# Thread-safe queue for serial reading thread -> main plotting thread
data_queue = queue.Queue()

# Deques to store history data (scrolling live view)
time_history = deque(maxlen=MAX_POINTS)
ekf_alt_history = deque(maxlen=MAX_POINTS)
baro_alt_history = deque(maxlen=MAX_POINTS)
ekf_vel_z_history = deque(maxlen=MAX_POINTS)
ekf_vel_mag_history = deque(maxlen=MAX_POINTS)
raw_accel_low_history = deque(maxlen=MAX_POINTS)
raw_accel_high_history = deque(maxlen=MAX_POINTS)
gyro_x_history = deque(maxlen=MAX_POINTS)
gyro_y_history = deque(maxlen=MAX_POINTS)
gyro_z_history = deque(maxlen=MAX_POINTS)

# Lists to store the ENTIRE run history for saving on exit
full_time = []
full_ekf_alt = []
full_baro_alt = []
full_ekf_vel_z = []
full_ekf_vel_mag = []
full_accel_low = []
full_accel_high = []
full_gyro_x = []
full_gyro_y = []
full_gyro_z = []
full_fsm_state = []

# Latest state variables
latest_ekf_alt = 0.0
latest_baro_alt = 0.0
latest_ekf_vel_z = 0.0
latest_ekf_vel_mag = 0.0
latest_accel_low = 1.0   # BMI088 Z-axis (default ~1g)
latest_accel_high = 1.0  # ADXL375 Z-axis (default ~1g)
latest_gyro_x = 0.0
latest_gyro_y = 0.0
latest_gyro_z = 0.0

# System status
current_fsm_state = "STATE_INIT"
battery_mv = 0
gps_sats = 0
gps_fix = 0
sensor_health_bits = 0x00
ekf_health_bits = 0x00

running = True
report_saved = False
baro_baseline = None

def serial_reader_task(port):
    """Reads serial, parses data lines, and inserts snapshot to the queue"""
    global running, baro_baseline
    global latest_ekf_alt, latest_baro_alt, latest_ekf_vel_z, latest_ekf_vel_mag
    global latest_accel_low, latest_accel_high, latest_gyro_x, latest_gyro_y, latest_gyro_z
    global current_fsm_state, battery_mv, gps_sats, gps_fix, sensor_health_bits, ekf_health_bits
    
    print(f"📡 正在開啟串口: {port} @ {BAUD_RATE} ...")
    try:
        ser = serial_link.open_serial(port, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"❌ 錯誤: 無法開啟 {port}: {e}")
        running = False
        return
        
    start_time = time.time()
    print("🚀 串口已連接，等待資料流... (Ctrl+C 關閉視窗或在終端機結束都會存檔)")
    
    while running:
        try:
            if ser.in_waiting:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                # 印出所有非高頻遙測的 MCU 調試/開機日誌
                parts = line.split(',')
                is_csv = (len(parts) == 9 and all(p.lstrip('-').isdigit() for p in parts))
                if not is_csv and not line.startswith("[TELE]"):
                    print(line)
                
                # Check for board boot/reset print
                if "[BOOT]" in line:
                    print("\r\n🔄 [RESET] 偵測到主航電板重置/重新開機！重設時間軸與繪圖快取...")
                    data_queue.put("RESET")
                    start_time = time.time()
                    baro_baseline = None
                    latest_ekf_alt = 0.0
                    latest_baro_alt = 0.0
                    latest_ekf_vel_z = 0.0
                    latest_ekf_vel_mag = 0.0
                    latest_accel_low = 1.0
                    latest_accel_high = 1.0
                    latest_gyro_x = 0.0
                    latest_gyro_y = 0.0
                    latest_gyro_z = 0.0
                    current_fsm_state = "STATE_INIT"
                    continue
                
                # A. Parse EKF state [TELE]
                m_tele = tele_pattern.search(line)
                if m_tele:
                    latest_ekf_alt = float(m_tele.group(3)) # pos_z
                    vx = float(m_tele.group(4))
                    vy = float(m_tele.group(5))
                    vz = float(m_tele.group(6))
                    latest_ekf_vel_z = vz
                    latest_ekf_vel_mag = math.sqrt(vx**2 + vy**2 + vz**2)
                    
                    # Push time snapshot
                    t_elapsed = time.time() - start_time
                    data_queue.put((
                        t_elapsed, latest_ekf_alt, latest_baro_alt,
                        latest_ekf_vel_z, latest_ekf_vel_mag,
                        latest_accel_low, latest_accel_high,
                        latest_gyro_x, latest_gyro_y, latest_gyro_z,
                        current_fsm_state, battery_mv, gps_sats, gps_fix,
                        sensor_health_bits, ekf_health_bits
                    ))
                
                # B. Parse raw sensor data (9 integers comma-separated line)
                # Format: bmi_ax,bmi_ay,bmi_az,adxl_ax,adxl_ay,adxl_az,temp,press,alt
                parts = line.split(',')
                if len(parts) == 9 and all(p.lstrip('-').isdigit() for p in parts):
                    latest_accel_low = int(parts[2]) / 1000.0   # bmi_az (mg -> g)
                    latest_accel_high = int(parts[5]) / 1000.0  # adxl_az (mg -> g)
                    raw_baro = int(parts[8]) / 100.0            # alt(cm -> m)
                    if baro_baseline is None:
                        baro_baseline = raw_baro
                        print(f"📍 偵測氣壓計起點基準 (Baseline): {baro_baseline:.2f} m")
                    latest_baro_alt = raw_baro - baro_baseline
                
                # C. Parse Gyro from [IMU]
                m_imu = imu_pattern.search(line)
                if m_imu:
                    latest_gyro_x = float(m_imu.group(4))
                    latest_gyro_y = float(m_imu.group(5))
                    latest_gyro_z = float(m_imu.group(6))
                
                # D. Parse battery voltage [PWR]
                m_pwr = pwr_pattern.search(line)
                if m_pwr:
                    battery_mv = int(m_pwr.group(1))
                
                # E. Parse GPS state [GPS]
                m_gps = gps_pattern.search(line)
                if m_gps:
                    gps_fix = int(m_gps.group(1))
                    gps_sats = int(m_gps.group(2))
                
                # F. Parse Health status [HEALTH]
                m_health = health_pattern.search(line)
                if m_health:
                    sensor_health_bits = int(m_health.group(1), 16) if 'x' in m_health.group(1) else int(m_health.group(1))
                    ekf_health_bits = int(m_health.group(2), 16) if 'x' in m_health.group(2) else int(m_health.group(2))
                
                # G. Parse FSM state [FSM]
                if "[FSM]" in line:
                    m_state = re.search(r"(STATE_[A-Z_]+)", line)
                    if m_state:
                        current_fsm_state = m_state.group(1)
                    elif "LIFTOFF" in line:
                        current_fsm_state = "STATE_BOOST"
                    elif "BURNOUT" in line:
                        current_fsm_state = "STATE_COAST"
                    elif "DEPLOY_DROGUE" in line:
                        current_fsm_state = "STATE_DEPLOY_DROGUE"
                    elif "APOGEE" in line:
                        current_fsm_state = "STATE_APOGEE"
                    elif "LANDED" in line:
                        current_fsm_state = "STATE_LANDED"
            else:
                time.sleep(0.002)
        except Exception as e:
            # Silence decode issues
            pass
            
    try:
        ser.close()
    except:
        pass


def save_full_report():
    """Generates a high-quality light-themed plot of the entire recorded data on exit"""
    global report_saved
    if report_saved:
        return
        
    if not full_time:
        print("\r\n[INFO] 沒有收集到任何數據，跳過存檔。")
        return
        
    # Save to simulation_and_data/flight_plots/
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(script_dir, "..", "..", "simulation_and_data", "flight_plots", f"flight_record_{time.strftime('%Y%m%d_%H%M%S')}.png")
    print(f"\r\n💾 正在產生完整飛行數據分析圖 (點數: {len(full_time)})...")
    report_saved = True
    
    # Save a clean, print-friendly light background chart for reports
    plt.style.use('default')
    fig_full, axs = plt.subplots(2, 2, figsize=(15, 10))
    fig_full.suptitle(f"RocketCom Complete Flight Analysis Report\r\nSaved to: {filename}", fontsize=15, fontweight='bold')
    
    ax_alt, ax_vel = axs[0, 0], axs[0, 1]
    ax_acc, ax_gyro = axs[1, 0], axs[1, 1]
    
    # Colors for printing
    ax_alt.plot(full_time, full_ekf_alt, '#0077b6', lw=2, label='EKF Altitude ($h_{ekf}$)')
    ax_alt.plot(full_time, full_baro_alt, '#38b000', lw=1.2, ls='--', label='Baro Altitude ($h_{baro}$)')
    ax_alt.set_ylabel("Altitude (m)", fontsize=10, fontweight='bold')
    ax_alt.set_title("Altitude Curve (m)", fontsize=11, fontweight='bold')
    ax_alt.grid(True, linestyle=':')
    ax_alt.legend(loc="upper left")
    
    ax_vel.plot(full_time, full_ekf_vel_z, '#e63946', lw=2, label='Vertical Velocity ($v_z$)')
    ax_vel.plot(full_time, full_ekf_vel_mag, '#7209b7', lw=1.5, ls='--', label='Speed Magnitude ($|v|$)')
    ax_vel.set_ylabel("Velocity (m/s)", fontsize=10, fontweight='bold')
    ax_vel.set_title("Velocity Curve (m/s)", fontsize=11, fontweight='bold')
    ax_vel.grid(True, linestyle=':')
    ax_vel.legend(loc="upper left")
    
    ax_acc.plot(full_time, full_accel_low, '#fca311', lw=1.5, label='Low-G BMI088 Z')
    ax_acc.plot(full_time, full_accel_high, '#d90429', lw=1.0, ls='--', label='High-G ADXL375 Z')
    ax_acc.set_ylabel("Acceleration (g)", fontsize=10, fontweight='bold')
    ax_acc.set_xlabel("Time (s)", fontsize=10)
    ax_acc.set_title("Z-Axis Acceleration Raw (g)", fontsize=11, fontweight='bold')
    ax_acc.grid(True, linestyle=':')
    ax_acc.legend(loc="upper left")
    
    ax_gyro.plot(full_time, full_gyro_x, '#ef233c', lw=0.8, label='Gyro X')
    ax_gyro.plot(full_time, full_gyro_y, '#38b000', lw=0.8, label='Gyro Y')
    ax_gyro.plot(full_time, full_gyro_z, '#0077b6', lw=1.2, label='Gyro Z')
    ax_gyro.set_ylabel("Angular Rate (dps)", fontsize=10, fontweight='bold')
    ax_gyro.set_xlabel("Time (s)", fontsize=10)
    ax_gyro.set_title("3-Axis Gyro Raw (dps)", fontsize=11, fontweight='bold')
    ax_gyro.grid(True, linestyle=':')
    ax_gyro.legend(loc="upper left")
    
    # Draw vertical indicator lines for FSM state transitions on altitude and velocity plots
    last_state = None
    for idx, (t, alt, state) in enumerate(zip(full_time, full_ekf_alt, full_fsm_state)):
        if state != last_state:
            # Skip drawing line for init state
            if state != "STATE_INIT":
                for ax in [ax_alt, ax_vel]:
                    ax.axvline(x=t, color='#d90429', linestyle=':', alpha=0.7)
                # Label the state on the altitude plot with name and timestamp
                state_name = state.replace("STATE_", "")
                label_text = f"{state_name}\r\n({t:.2f}s)"
                ax_alt.text(t, alt + max(full_ekf_alt)*0.03 + 2.0, label_text, 
                            rotation=0, fontsize=8, fontweight='bold', color='#d90429',
                            ha='center', va='bottom')
            last_state = state
            
    plt.tight_layout()
    fig_full.savefig(filename, dpi=150)
    plt.close(fig_full)
    print(f"✅ 飛行分析圖表已成功儲存為：{os.path.abspath(filename)}")


def main():
    global running
    
    # 1. Resolve serial port
    port = serial_link.auto_port()
    if not port:
        ports = serial_link.list_candidate_ports()
        if not ports:
            print("❌ 系統未檢測到任何可用串口！請插上 USB-TTL 模組後重試。")
            sys.exit(1)
        port = serial_link.prompt_select_port()
        if not port:
            sys.exit(1)
            
    # 2. Launch serial thread
    thread = threading.Thread(target=serial_reader_task, args=(port,), daemon=True)
    thread.start()
    
    # 3. Setup Matplotlib figure & subplots grid (2x2)
    plt.style.use('dark_background')
    fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    fig.canvas.manager.set_window_title("RocketCom Live Dashboard — Sensor + EKF + FSM")
    fig.patch.set_facecolor('#121212')
    
    ax_alt, ax_vel = axs[0, 0], axs[0, 1]
    ax_acc, ax_gyro = axs[1, 0], axs[1, 1]
    
    # Face colors
    for ax in [ax_alt, ax_vel, ax_acc, ax_gyro]:
        ax.set_facecolor('#1c1c1c')
        ax.grid(True, color='#444444', linestyle=':')
    
    # Subplot 1: Altitude (EKF vs Baro)
    line_ekf_alt, = ax_alt.plot([], [], '#00d2ff', lw=2, label='EKF Altitude ($h_{ekf}$)')
    line_baro_alt, = ax_alt.plot([], [], '#28a745', lw=1.2, ls='--', label='Baro Altitude ($h_{baro}$)')
    ax_alt.set_ylabel("Altitude (m)", fontsize=10, fontweight='bold')
    ax_alt.set_title("Altitude (EKF vs Baro)", color='white', fontsize=11, fontweight='bold')
    ax_alt.legend(loc="upper left")
    
    # Subplot 2: Velocity (EKF Vz & Speed)
    line_vel_z, = ax_vel.plot([], [], 'cyan', lw=2, label='Vertical Velocity ($v_z$)')
    line_vel_mag, = ax_vel.plot([], [], '#a88beb', lw=1.5, ls='--', label='EKF Speed ($|v|$)')
    ax_vel.set_ylabel("Velocity (m/s)", fontsize=10, fontweight='bold')
    ax_vel.set_title("EKF Estimation Velocity", color='white', fontsize=11, fontweight='bold')
    ax_vel.legend(loc="upper left")
    
    # Subplot 3: Z-axis Acceleration (BMI088 vs ADXL375)
    line_acc_low, = ax_acc.plot([], [], '#ffaa00', lw=1.8, label='Low-G BMI088 ($a_{z}$)')
    line_acc_high, = ax_acc.plot([], [], '#ff3b30', lw=1.2, ls='--', label='High-G ADXL375 ($a_{z}$)')
    ax_acc.set_ylabel("Acceleration (g)", fontsize=10, fontweight='bold')
    ax_acc.set_xlabel("Elapsed Time (s)", fontsize=10)
    ax_acc.set_title("Z-Axis Acceleration Raw Data", color='white', fontsize=11, fontweight='bold')
    ax_acc.legend(loc="upper left")
    
    # Subplot 4: Gyro (3-axis angular rates)
    line_gyro_x, = ax_gyro.plot([], [], '#ff5555', lw=1.2, label='Gyro X')
    line_gyro_y, = ax_gyro.plot([], [], '#55ff55', lw=1.2, label='Gyro Y')
    line_gyro_z, = ax_gyro.plot([], [], '#5555ff', lw=1.5, label='Gyro Z')
    ax_gyro.set_ylabel("Angular Rate (dps)", fontsize=10, fontweight='bold')
    ax_gyro.set_xlabel("Elapsed Time (s)", fontsize=10)
    ax_gyro.set_title("3-Axis Gyro Raw Data", color='white', fontsize=11, fontweight='bold')
    ax_gyro.legend(loc="upper left")
    
    # Bounding box banner for system status
    status_text = fig.text(
        0.5, 0.95, "FSM: STATE_INIT | BAT: 0 mV | GPS: 0 Sats (No Fix) | Health: Sensor OK / EKF OK",
        ha='center', va='center', color='white', fontsize=11, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.6', facecolor='#222222', edgecolor='#00d2ff', alpha=0.9)
    )

    # Dictionary to keep state variables inside closure
    fsm_state_ref = {"last": "STATE_INIT"}
    fsm_artists = []

    def init():
        line_ekf_alt.set_data([], [])
        line_baro_alt.set_data([], [])
        line_vel_z.set_data([], [])
        line_vel_mag.set_data([], [])
        line_acc_low.set_data([], [])
        line_acc_high.set_data([], [])
        line_gyro_x.set_data([], [])
        line_gyro_y.set_data([], [])
        line_gyro_z.set_data([], [])
        return line_ekf_alt, line_baro_alt, line_vel_z, line_vel_mag, line_acc_low, line_acc_high, line_gyro_x, line_gyro_y, line_gyro_z

    def animate(frame):
        new_data = False
        fsm_lbl = "STATE_INIT"
        bat = 0
        sats = 0
        fix = 0
        sens_hb = 0
        ekf_hb = 0
        
        while not data_queue.empty():
            item = data_queue.get()
            if item == "RESET":
                # Clear all live deques
                time_history.clear()
                ekf_alt_history.clear()
                baro_alt_history.clear()
                ekf_vel_z_history.clear()
                ekf_vel_mag_history.clear()
                raw_accel_low_history.clear()
                raw_accel_high_history.clear()
                gyro_x_history.clear()
                gyro_y_history.clear()
                gyro_z_history.clear()
                # Clear full lists
                full_time.clear()
                full_ekf_alt.clear()
                full_baro_alt.clear()
                full_ekf_vel_z.clear()
                full_ekf_vel_mag.clear()
                full_accel_low.clear()
                full_accel_high.clear()
                full_gyro_x.clear()
                full_gyro_y.clear()
                full_gyro_z.clear()
                full_fsm_state.clear()
                
                # Clear previous FSM vertical lines and text from charts
                for artist in fsm_artists:
                    try:
                        artist.remove()
                    except:
                        pass
                fsm_artists.clear()
                fsm_state_ref["last"] = "STATE_INIT"
                
                new_data = True
                continue
                
            # Unpack normal sample
            (t, ealt, balt, vz, vmag, alow, ahigh, gx, gy, gz,
             fsm_lbl, bat, sats, fix, sens_hb, ekf_hb) = item
            
            time_history.append(t)
            ekf_alt_history.append(ealt)
            baro_alt_history.append(balt)
            ekf_vel_z_history.append(vz)
            ekf_vel_mag_history.append(vmag)
            raw_accel_low_history.append(alow)
            raw_accel_high_history.append(ahigh)
            gyro_x_history.append(gx)
            gyro_y_history.append(gy)
            gyro_z_history.append(gz)
            
            # Store in full history
            full_time.append(t)
            full_ekf_alt.append(ealt)
            full_baro_alt.append(balt)
            full_ekf_vel_z.append(vz)
            full_ekf_vel_mag.append(vmag)
            full_accel_low.append(alow)
            full_accel_high.append(ahigh)
            full_gyro_x.append(gx)
            full_gyro_y.append(gy)
            full_gyro_z.append(gz)
            full_fsm_state.append(fsm_lbl)
            new_data = True
            
            # Check for FSM State Transitions to draw vertical marks in real-time
            if fsm_lbl != fsm_state_ref["last"]:
                if fsm_lbl != "STATE_INIT" and len(time_history) > 0:
                    t_trans = time_history[-1]
                    alt_trans = ekf_alt_history[-1]
                    state_name = fsm_lbl.replace("STATE_", "")
                    label_text = f"{state_name}\n({t_trans:.1f}s)"
                    
                    # Draw vertical line on Altitude and Velocity subplots
                    v1 = ax_alt.axvline(x=t_trans, color='#ef233c', linestyle=':', alpha=0.7)
                    v2 = ax_vel.axvline(x=t_trans, color='#ef233c', linestyle=':', alpha=0.7)
                    fsm_artists.append(v1)
                    fsm_artists.append(v2)
                    
                    # Draw text label above the altitude point
                    t1 = ax_alt.text(t_trans, alt_trans + 3.0, label_text, 
                                     color='#ffcc00', fontsize=8, fontweight='bold',
                                     ha='center', va='bottom')
                    fsm_artists.append(t1)
                fsm_state_ref["last"] = fsm_lbl
            
        if new_data and len(time_history) > 0:
            t_list = list(time_history)
            
            # Update Altitudes
            line_ekf_alt.set_data(t_list, list(ekf_alt_history))
            line_baro_alt.set_data(t_list, list(baro_alt_history))
            
            # Update Velocities
            line_vel_z.set_data(t_list, list(ekf_vel_z_history))
            line_vel_mag.set_data(t_list, list(ekf_vel_mag_history))
            
            # Update Accelerations
            line_acc_low.set_data(t_list, list(raw_accel_low_history))
            line_acc_high.set_data(t_list, list(raw_accel_high_history))
            
            # Update Gyro rates
            line_gyro_x.set_data(t_list, list(gyro_x_history))
            line_gyro_y.set_data(t_list, list(gyro_y_history))
            line_gyro_z.set_data(t_list, list(gyro_z_history))
            
            # Adjust X bounds on bottom plots
            x_min = t_list[0]
            x_max = t_list[-1]
            ax_acc.set_xlim(x_min, max(x_max, x_min + 5.0))
            ax_gyro.set_xlim(x_min, max(x_max, x_min + 5.0))
            
            # Dynamic limits with margins
            # Alt
            alts_arr = np.concatenate([list(ekf_alt_history), list(baro_alt_history)])
            alt_min, alt_max = np.min(alts_arr), np.max(alts_arr)
            alt_pad = max(abs(alt_max - alt_min) * 0.1, 5.0)
            ax_alt.set_ylim(alt_min - alt_pad, alt_max + alt_pad)
            
            # Vel
            vels_arr = np.concatenate([list(ekf_vel_z_history), list(ekf_vel_mag_history)])
            vel_min, vel_max = np.min(vels_arr), np.max(vels_arr)
            vel_pad = max(abs(vel_max - vel_min) * 0.1, 2.0)
            ax_vel.set_ylim(vel_min - vel_pad, vel_max + vel_pad)
            
            # Acc
            acc_arr = np.concatenate([list(raw_accel_low_history), list(raw_accel_high_history)])
            acc_min, acc_max = np.min(acc_arr), np.max(acc_arr)
            acc_pad = max(abs(acc_max - acc_min) * 0.1, 0.5)
            ax_acc.set_ylim(acc_min - acc_pad, acc_max + acc_pad)
            
            # Gyro
            gyr_arr = np.concatenate([list(gyro_x_history), list(gyro_y_history), list(gyro_z_history)])
            gyr_min, gyr_max = np.min(gyr_arr), np.max(gyr_arr)
            gyr_pad = max(abs(gyr_max - gyr_min) * 0.1, 50.0)
            ax_gyro.set_ylim(gyr_min - gyr_pad, gyr_max + gyr_pad)
            
            # Update Banner text & color based on status
            gps_status = f"{sats} Sats (Fix OK)" if fix else f"{sats} Sats (No Fix)"
            health_str = []
            if sens_hb == 0: health_str.append("Sensor OK")
            else: health_str.append(f"Sensor ERR (0x{sens_hb:02X})")
            if ekf_hb == 0: health_str.append("EKF OK")
            else: health_str.append(f"EKF ERR (0x{ekf_hb:02X})")
            
            banner_msg = f"FSM: {fsm_lbl}  |  BAT: {bat} mV  |  GPS: {gps_status}  |  Health: {' / '.join(health_str)}"
            status_text.set_text(banner_msg)
            
            # Change banner color based on state
            if fsm_lbl == "STATE_PAD":
                status_text.get_bbox_patch().set_edgecolor('#00d2ff')
            elif fsm_lbl == "STATE_BOOST":
                status_text.get_bbox_patch().set_edgecolor('#ff3b30')
            elif fsm_lbl == "STATE_COAST":
                status_text.get_bbox_patch().set_edgecolor('#ffcc00')
            elif fsm_lbl in ["STATE_DEPLOY_DROGUE", "STATE_APOGEE", "STATE_DESCENT", "STATE_MAIN_DEPLOY"]:
                status_text.get_bbox_patch().set_edgecolor('#28a745')
            elif fsm_lbl == "STATE_LANDED":
                status_text.get_bbox_patch().set_edgecolor('#ffffff')

        return line_ekf_alt, line_baro_alt, line_vel_z, line_vel_mag, line_acc_low, line_acc_high, line_gyro_x, line_gyro_y, line_gyro_z, status_text

    # Dynamic animation handler
    ani = animation.FuncAnimation(
        fig, animate, init_func=init, interval=40, blit=False, cache_frame_data=False
    )
    
    plt.subplots_adjust(top=0.88, bottom=0.08, left=0.08, right=0.95, hspace=0.25, wspace=0.20)
    
    def on_close(event):
        global running
        running = False
        save_full_report()
        
    fig.canvas.mpl_connect('close_event', on_close)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\r\n[INFO] 偵測到 Ctrl+C 中斷，正在結束程式並儲存報告...")
    finally:
        running = False
        save_full_report()

if __name__ == '__main__':
    main()

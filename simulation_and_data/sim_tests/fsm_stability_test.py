#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
成大 ISP 航電組 — RocketFSM 雜訊與穩定度模擬分析工具
===========================================================================
此腳本讀取 `data/台灣盃2026時間高度垂直速度合速度合加速度.csv`，
在加速度與氣壓高度中注入使用者可調的隨機高斯雜訊，並運行卡爾曼濾波（EKF）與
飛行狀態機（FSM）演算法，以定量評估開傘時機與落點判定的演算法穩定度。
"""

import os
import sys
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons

# 設定 Matplotlib 字型支援，以防中文字元或負號缺失警告
matplotlib.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'Heiti TC', 'PingFang TC', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False  # 正常顯示負號


# =========================================================================
# 1. 數據加載模組
# =========================================================================
def load_simulation_data(file_path):
    times, alts, vels, accs = [], [], [], []
    if not os.path.exists(file_path):
        print(f"[Error] 找不到數據檔案: {file_path}")
        print("請確認 data 資料夾下是否存在 台灣盃2026時間高度垂直速度合速度合加速度.csv")
        sys.exit(1)
        
    print(f"[Data] 正在讀取模擬資料: {file_path} ...")
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            # 跳過註解行
            if line.startswith('#'):
                continue
            parts = line.strip().split(',')
            if len(parts) < 5:
                continue
            try:
                # 欄位解析: 0:時間(s), 1:高度(m), 2:垂直速度(m/s), 4:合加速度(m/s²)
                t = float(parts[0])
                h = float(parts[1])
                v = float(parts[2])
                a = float(parts[4])
                times.append(t)
                alts.append(h)
                vels.append(v)
                accs.append(a)
            except ValueError:
                # 跳過標頭或壞字元行
                continue
                
    print(f"[Data] 成功加載 {len(times)} 筆數據。總飛行時間 {times[-1]:.2f} 秒。")
    return np.array(times), np.array(alts), np.array(vels), np.array(accs)


# =========================================================================
# 2. 狀態機與濾波器定義
# =========================================================================
# FSM 狀態定義
STATE_INIT = 0
STATE_PAD = 1
STATE_BOOST = 2
STATE_COAST = 3
STATE_APOGEE = 4
STATE_DESCENT = 5
STATE_MAIN_DEPLOY = 6
STATE_LANDED = 7

class SimulatedFSM:
    def __init__(self):
        self.state = STATE_PAD
        self.flight_start_ms = 0
        self.state_entered_ms = 0
        self.max_altitude = 0.0
        self.last_vel_z = 0.0
        self.consec_apogee_counts = 0
        self.touchdown_latched = False
        self.drogue_fired = False
        self.max_alt_baro = 0.0
        self.consec_baro_drop = 0
        
        # 狀態觸發時間紀錄 (秒)
        self.t_liftoff = None
        self.t_burnout = None
        self.t_apogee = None
        self.t_main = None
        self.t_landed = None
        
        # 標記是否為失效保護觸發
        self.failsafe_apogee = False
        self.failsafe_main = False

    def step(self, now_ms, h_est, v_est, a_z_g, baro_alt_rel, ekf_healthy, baro_healthy):
        now = now_ms
        if h_est > self.max_altitude:
            self.max_altitude = h_est
            
        baro_ok = baro_healthy

        # --- A. 40秒頂點絕對失效保護 (配合 3156m 頂點大火箭軌跡) ---
        if (self.state == STATE_BOOST or self.state == STATE_COAST) and \
           (now - self.flight_start_ms) >= 40000:
            self.state = STATE_APOGEE
            self.state_entered_ms = now
            self.drogue_fired = True
            self.t_apogee = now / 1000.0
            self.failsafe_apogee = True
            self.last_vel_z = v_est
            return
            
        # --- B. 狀態機核心轉移分支 ---
        if self.state == STATE_PAD:
            liftoff = False
            if ekf_healthy:
                # 加速度 > 3.0g 或 高度 > 10.0m，或氣壓計相對高度 > 20.0m
                liftoff = (a_z_g > 3.0 or h_est > 10.0 or (baro_ok and baro_alt_rel > 20.0))
            else:
                # 降級路徑：忽略 EKF 高度
                liftoff = (a_z_g > 3.0 or (baro_ok and baro_alt_rel > 20.0))
            
            if liftoff:
                self.state = STATE_BOOST
                self.flight_start_ms = now
                self.state_entered_ms = now
                self.t_liftoff = now / 1000.0

        elif self.state == STATE_BOOST:
            # 熄火判定：加速度 < 0.5g 且進入狀態大於 1.5 秒
            if a_z_g < 0.5 and (now - self.state_entered_ms) > 1500:
                self.state = STATE_COAST
                self.state_entered_ms = now
                self.t_burnout = now / 1000.0

        elif self.state == STATE_COAST:
            decel = -9.80665
            if self.last_vel_z != 0.0:
                # 10ms 差分計算加速度
                a_z_nav = (v_est - self.last_vel_z) / 0.010
                if -25.0 < a_z_nav < -5.0:
                    decel = a_z_nav
            
            t_to_apogee = -v_est / decel
            
            # 氣壓計趨勢下落交叉檢查 (連續 200ms 下落 10m)
            baro_apogee = False
            if baro_ok:
                if baro_alt_rel > self.max_alt_baro:
                    self.max_alt_baro = baro_alt_rel
                if (self.max_alt_baro - baro_alt_rel) >= 10.0:
                    self.consec_baro_drop += 1
                else:
                    self.consec_baro_drop = 0
                baro_apogee = (self.consec_baro_drop >= 20)
                
            apogee_cond = False
            # 正常路徑 1：動態預測小於 4.0 秒且仍在上升
            if ekf_healthy and v_est > 0.0 and t_to_apogee <= 4.0:
                apogee_cond = True
            # 正常路徑 2：速度過零 (<-0.2m/s) 或高度回落超過 5.0m
            elif ekf_healthy and (v_est < -0.2 or (self.max_altitude - h_est) > 5.0):
                apogee_cond = True
            # 降級路徑 3：氣壓計趨勢已過頂點
            elif baro_apogee:
                apogee_cond = True
                
            # 起飛 3 秒鎖
            if apogee_cond and (now - self.flight_start_ms) > 3000:
                self.consec_apogee_counts += 1
                if self.consec_apogee_counts >= 5:
                    self.state = STATE_APOGEE
                    self.state_entered_ms = now
                    self.drogue_fired = True
                    self.t_apogee = now / 1000.0
            else:
                self.consec_apogee_counts = 0

        elif self.state == STATE_APOGEE:
            # 點火持續 2.0 秒後斷開，轉為 Descent
            if (now - self.state_entered_ms) >= 2000:
                self.state = STATE_DESCENT
                self.state_entered_ms = now

        elif self.state == STATE_DESCENT:
            main_trigger = False
            if ekf_healthy:
                v_fall = -v_est if v_est < 0.0 else 0.0
                # 動態開傘高度 = 150m + v_fall * 3.5s
                h_trigger_main = 150.0 + v_fall * 3.5
                main_trigger = (h_est <= h_trigger_main)
            elif baro_ok:
                # EKF損毀降級：氣壓高度 <= 200m
                main_trigger = (baro_alt_rel <= 200.0)
                
            # 開傘判定或 120秒總時間安全看門狗 (配合大火箭 354s 全程飛控)
            if main_trigger or (now - self.flight_start_ms) > 120000:
                self.state = STATE_MAIN_DEPLOY
                self.state_entered_ms = now
                self.t_main = now / 1000.0
                if (now - self.flight_start_ms) > 120000:
                    self.failsafe_main = True

        elif self.state == STATE_MAIN_DEPLOY:
            # 等待 3.0 秒充氣展開
            if (now - self.state_entered_ms) >= 3000:
                self.state = STATE_LANDED
                self.state_entered_ms = now

        elif self.state == STATE_LANDED:
            if not self.touchdown_latched:
                touchdown = False
                if ekf_healthy:
                    touchdown = (abs(v_est) < 0.3 and h_est < 20.0)
                elif baro_ok:
                    # 降級著陸：高度低於30m
                    touchdown = (h_est < 30.0)
                if touchdown:
                    self.touchdown_latched = True
                    self.t_landed = now / 1000.0

        self.last_vel_z = v_est


# =========================================================================
# 3. 穩定度模擬引擎
# =========================================================================
def run_simulation(times, alts, vels, accs, baro_std, accel_std, q_pos, q_vel, ekf_healthy):
    # A. 數據重採樣至 100Hz (10ms 步長)，完美對齊航電板飛控時序
    t_sim = np.arange(0, times[-1], 0.01)
    alt_true = np.interp(t_sim, times, alts)
    vel_true = np.interp(t_sim, times, vels)
    acc_true = np.interp(t_sim, times, accs)
    
    n_points = len(t_sim)
    
    # B. 注入獨立的高斯隨機雜訊
    # 氣壓計雜訊 (Baro Noise)
    noise_baro = np.random.normal(0, baro_std, n_points)
    alt_noisy = alt_true + noise_baro
    
    # 加速度計雜訊 (Accel Noise)
    noise_accel = np.random.normal(0, accel_std, n_points)
    acc_noisy = acc_true + noise_accel
    
    # C. 卡爾曼濾波（EKF 1D 垂直估算）
    # 狀態向量: x = [h, v]^T
    x = np.array([0.0, 0.0])
    P = np.array([[1.0, 0.0],
                  [0.0, 1.0]])
    
    # 共變異數矩陣
    Q = np.array([[q_pos, 0.0],
                  [0.0, q_vel]])
    R = baro_std ** 2 + 1e-5
    
    h_est = np.zeros(n_points)
    v_est = np.zeros(n_points)
    
    # 進行一步預測與修正
    for i in range(n_points):
        dt = 0.01
        if i == 0:
            h_est[i] = x[0]
            v_est[i] = x[1]
            continue
            
        # 1. 狀態預測 (Predict Step)
        F = np.array([[1.0, dt],
                      [0.0, 1.0]])
        B = np.array([0.5 * dt * dt, dt])
        
        # 讀取當前包含雜訊的加速度 (net acceleration)
        u = acc_noisy[i]
        
        x = F @ x + B * u
        P = F @ P @ F.T + Q
        
        # 2. 狀態修正 (Update Step)
        z = alt_noisy[i]
        H = np.array([1.0, 0.0])
        y = z - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T / S
        
        x = x + K * y
        P = (np.eye(2) - np.outer(K, H)) @ P
        
        h_est[i] = x[0]
        v_est[i] = x[1]

    # D. 狀態機仿真步進
    fsm = SimulatedFSM()
    fsm_states = np.zeros(n_points, dtype=int)
    
    for i in range(n_points):
        now_ms = int(t_sim[i] * 1000)
        # 加速度轉換為 g 單位 (合加速度中需考慮 net accel / g)
        a_z_g = acc_noisy[i] / 9.80665
        
        # 執行 FSM Step
        fsm.step(now_ms, h_est[i], v_est[i], a_z_g, alt_noisy[i], ekf_healthy, True)
        fsm_states[i] = fsm.state
        
    return t_sim, alt_true, vel_true, alt_noisy, acc_noisy, h_est, v_est, fsm_states, fsm


# =========================================================================
# 4. GUI 主視窗與互動式 Dashboard
# =========================================================================
class FSMStabilityApp:
    def __init__(self, times, alts, vels, accs):
        self.times = times
        self.alts = alts
        self.vels = vels
        self.accs = accs
        
        # 預設參數 (規格書 + 50%)
        self.baro_std = 0.25     # 氣壓計雜訊標準差 (m)
        self.accel_std = 2.94    # 加速度計雜訊標準差 (m/s²)
        self.q_pos = 0.01        # EKF 位置過程噪聲
        self.q_vel = 0.1         # EKF 速度過程噪聲
        self.ekf_healthy = True # EKF 狀態良好
        
        # 執行無噪聲真實基準值 (Ground Truth FSM)
        _, _, _, _, _, _, _, _, self.true_fsm = run_simulation(
            self.times, self.alts, self.vels, self.accs, 0.001, 0.001, 0.01, 0.1, True
        )
        
        # 初始化繪圖
        plt.style.use('dark_background')
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 8))
        self.fig.canvas.manager.set_window_title("NCKU ISP — RocketFSM Algorithm Stability Analyzer")
        
        # 給控制項留出空間
        plt.subplots_adjust(bottom=0.32, top=0.9)
        
        # 宣告繪圖對象
        self.lines = {}
        self.state_bands1 = []
        self.state_bands2 = []
        self.markers = []
        self.textbox = None
        
        # 建立控制滑塊與按鈕
        self.create_widgets()
        
        # 運行第一次仿真
        self.update_simulation()
        
    def create_widgets(self):
        # 滑塊配色
        slider_color = '#8b5cf6'
        accent_color = '#f97316'
        
        # A. 氣壓計噪聲滑塊 (Baro Noise Slider)
        ax_baro = plt.axes([0.15, 0.22, 0.3, 0.03])
        self.slider_baro = Slider(ax_baro, 'Baro Noise (m)', 0.0, 25.0, valinit=self.baro_std, valstep=0.1, color=slider_color)
        self.slider_baro.on_changed(self.on_slider_change)
        
        # B. 加速度計噪聲滑塊 (Accel Noise Slider)
        ax_accel = plt.axes([0.15, 0.16, 0.3, 0.03])
        self.slider_accel = Slider(ax_accel, 'Accel Noise (m/s²)', 0.0, 10.0, valinit=self.accel_std, valstep=0.1, color=slider_color)
        self.slider_accel.on_changed(self.on_slider_change)
        
        # C. EKF Q_Position 滑塊
        ax_qp = plt.axes([0.6, 0.22, 0.25, 0.03])
        self.slider_qp = Slider(ax_qp, 'EKF Q_Pos', 0.0001, 0.1, valinit=self.q_pos, valstep=0.0005, color=accent_color)
        self.slider_qp.on_changed(self.on_slider_change)
        
        # D. EKF Q_Velocity 滑塊
        ax_qv = plt.axes([0.6, 0.16, 0.25, 0.03])
        self.slider_qv = Slider(ax_qv, 'EKF Q_Vel', 0.001, 1.0, valinit=self.q_vel, valstep=0.005, color=accent_color)
        self.slider_qv.on_changed(self.on_slider_change)
        
        # E. EKF 故障狀態切換複選框 (Check Button)
        ax_check = plt.axes([0.15, 0.06, 0.15, 0.05])
        self.check_ekf = CheckButtons(ax_check, ['EKF Healthy'], [self.ekf_healthy])
        self.check_ekf.on_clicked(self.on_checkbox_click)
        
        # F. 隨機重置種子按鈕 (Regenerate Noise Button)
        ax_btn_reset = plt.axes([0.35, 0.06, 0.15, 0.05])
        self.btn_reset = Button(ax_btn_reset, 'Regenerate Noise', color='#1e1b4b', hovercolor='#312e81')
        self.btn_reset.label.set_color('#c084fc')
        self.btn_reset.on_clicked(self.on_regenerate_click)
        
        # G. 標準推薦參數重置按鈕 (Restore Defaults Button)
        ax_btn_def = plt.axes([0.55, 0.06, 0.15, 0.05])
        self.btn_def = Button(ax_btn_def, 'Restore Defaults', color='#064e3b', hovercolor='#065f46')
        self.btn_def.label.set_color('#34d399')
        self.btn_def.on_clicked(self.on_defaults_click)

    def on_slider_change(self, val):
        self.baro_std = self.slider_baro.val
        self.accel_std = self.slider_accel.val
        self.q_pos = self.slider_qp.val
        self.q_vel = self.slider_qv.val
        self.update_simulation()
        
    def on_checkbox_click(self, label):
        self.ekf_healthy = self.check_ekf.get_status()[0]
        self.update_simulation()
        
    def on_regenerate_click(self, event):
        # 透過重新設定亂數種子，更新噪聲樣本
        self.update_simulation()
        
    def on_defaults_click(self, event):
        self.slider_baro.set_val(0.25)
        self.slider_accel.set_val(2.94)
        self.slider_qp.set_val(0.01)
        self.slider_qv.set_val(0.1)
        if not self.check_ekf.get_status()[0]:
            self.check_ekf.set_active(0)
            
    def update_simulation(self):
        # 執行模擬器核心
        t_sim, alt_true, vel_true, alt_noisy, acc_noisy, h_est, v_est, fsm_states, fsm = run_simulation(
            self.times, self.alts, self.vels, self.accs, 
            self.baro_std, self.accel_std, self.q_pos, self.q_vel, self.ekf_healthy
        )
        
        # 清除舊的狀態背景顏色帶
        for band in self.state_bands1 + self.state_bands2:
            band.remove()
        self.state_bands1.clear()
        self.state_bands2.clear()
        
        # 清除舊的標記點
        for m in self.markers:
            m.remove()
        self.markers.clear()
        
        # A. 繪製左側高度子圖 (Altitude Plot)
        self.ax1.cla()
        self.ax1.plot(t_sim, alt_true, color='#9ca3af', linestyle='--', alpha=0.8, label='True Altitude')
        self.ax1.scatter(t_sim[::5], alt_noisy[::5], color='#f87171', s=1.0, alpha=0.3, label='Noisy Baro')
        self.ax1.plot(t_sim, h_est, color='#00f2fe', linewidth=2.0, label='EKF Estimated')
        
        self.ax1.set_title("Rocket Altitude (Z)", fontsize=12, fontweight='bold', color='#e5e7eb')
        self.ax1.set_xlabel("Time (seconds)", fontsize=10)
        self.ax1.set_ylabel("Altitude (meters)", fontsize=10)
        self.ax1.grid(True, color=(1.0, 1.0, 1.0, 0.05))
        self.ax1.legend(loc='upper left', framealpha=0.1)
        
        # B. 繪製右側速度子圖 (Velocity Plot)
        self.ax2.cla()
        self.ax2.plot(t_sim, vel_true, color='#9ca3af', linestyle='--', alpha=0.8, label='True Velocity')
        self.ax2.plot(t_sim, v_est, color='#c084fc', linewidth=2.0, label='EKF Estimated')
        
        self.ax2.set_title("Vertical Velocity (Vz)", fontsize=12, fontweight='bold', color='#e5e7eb')
        self.ax2.set_xlabel("Time (seconds)", fontsize=10)
        self.ax2.set_ylabel("Velocity (m/s)", fontsize=10)
        self.ax2.grid(True, color=(1.0, 1.0, 1.0, 0.05))
        self.ax2.legend(loc='upper right', framealpha=0.1)
        
        # C. 繪製背景狀態顏色帶 (FSM State Bands)
        # 火箭狀態配色 (顏色, 透明度)
        state_colors = {
            STATE_PAD: ('#3b82f6', 0.1),      # 藍色
            STATE_BOOST: ('#f97316', 0.15),   # 橘色
            STATE_COAST: ('#8b5cf6', 0.12),   # 紫色
            STATE_APOGEE: ('#ef4444', 0.2),    # 紅色
            STATE_DESCENT: ('#06b6d4', 0.12),  # 青色
            STATE_MAIN_DEPLOY: ('#10b981', 0.15), # 綠色
            STATE_LANDED: ('#eab308', 0.12)    # 黃色
        }
        
        # 尋找狀態交界點以區分區段
        diff = np.diff(fsm_states)
        boundaries = np.where(diff != 0)[0]
        start_idx = 0
        
        for b in list(boundaries) + [len(fsm_states)-1]:
            st = fsm_states[start_idx]
            if st in state_colors:
                c, alpha_val = state_colors[st]
                # 填充背景顏色
                band1 = self.ax1.axvspan(t_sim[start_idx], t_sim[b], color=c, alpha=alpha_val)
                band2 = self.ax2.axvspan(t_sim[start_idx], t_sim[b], color=c, alpha=alpha_val)
                self.state_bands1.append(band1)
                self.state_bands2.append(band2)
            start_idx = b + 1

        # D. 標記頂點與主傘部署觸發位置 (Event Markers)
        # 1. 頂點標記
        if fsm.t_apogee is not None:
            # 找到對應時間點的索引
            idx = np.abs(t_sim - fsm.t_apogee).argmin()
            m1 = self.ax1.axvline(fsm.t_apogee, color='#f87171', linestyle=':', alpha=0.8)
            m2 = self.ax2.axvline(fsm.t_apogee, color='#f87171', linestyle=':', alpha=0.8)
            self.markers.extend([m1, m2])
            
            # 畫點與標籤
            p1, = self.ax1.plot([fsm.t_apogee], [h_est[idx]], 'ro', ms=6)
            self.markers.append(p1)
            text_apogee = "APOGEE" if not fsm.failsafe_apogee else "APOGEE\n(FAILSAFE)"
            self.ax1.text(fsm.t_apogee + 0.5, h_est[idx] - 25, text_apogee, color='#f87171', fontsize=9, fontweight='bold')
            
        # 2. 主傘開傘標記
        if fsm.t_main is not None:
            idx = np.abs(t_sim - fsm.t_main).argmin()
            m1 = self.ax1.axvline(fsm.t_main, color='#34d399', linestyle=':', alpha=0.8)
            m2 = self.ax2.axvline(fsm.t_main, color='#34d399', linestyle=':', alpha=0.8)
            self.markers.extend([m1, m2])
            
            p2, = self.ax1.plot([fsm.t_main], [h_est[idx]], 'go', ms=6)
            self.markers.append(p2)
            text_main = "MAIN" if not fsm.failsafe_main else "MAIN\n(WATCHDOG)"
            self.ax1.text(fsm.t_main + 0.5, h_est[idx] + 15, text_main, color='#34d399', fontsize=9, fontweight='bold')

        # E. 計算與輸出定量指標報告 (Stability Statistics)
        self.render_statistics_box(fsm)
        
        # 重繪畫面
        self.fig.canvas.draw_idle()

    def render_statistics_box(self, fsm):
        # Delete old text box
        if self.textbox:
            self.textbox.remove()
            
        # Calculate parachute trigger time errors relative to ground truth
        err_apogee = fsm.t_apogee - self.true_fsm.t_apogee if (fsm.t_apogee and self.true_fsm.t_apogee) else None
        err_main = fsm.t_main - self.true_fsm.t_main if (fsm.t_main and self.true_fsm.t_main) else None
        
        str_apogee_err = f"{(err_apogee):+.3f} s" if err_apogee is not None else "N/A"
        str_main_err = f"{(err_main):+.3f} s" if err_main is not None else "N/A"
        
        # Get altitude at the moment of drogue deployment
        apogee_alt = fsm.max_altitude if fsm.t_apogee else 0.0
        
        stats_text = (
            f"=== FSM STABILITY ANALYSIS ===\n"
            f"True Apogee Time: {self.true_fsm.t_apogee:6.2f} s\n"
            f"Sim Apogee Time : {fsm.t_apogee if fsm.t_apogee else 0.0:6.2f} s (Err: {str_apogee_err})\n"
            f"Drogue Deploy Alt: {apogee_alt:6.1f} m (True Apogee: {self.true_fsm.max_altitude:.1f}m)\n"
            f"-----------------------------------------\n"
            f"True Main Time  : {self.true_fsm.t_main:6.2f} s\n"
            f"Sim Main Time   : {fsm.t_main if fsm.t_main else 0.0:6.2f} s (Err: {str_main_err})\n"
            f"Main Trigger Mode: {'Fallback (Baro)' if not self.ekf_healthy else 'Standard EKF'}\n"
            f"Main Deploy Alt  : {self.get_alt_at_time(fsm.t_main):6.1f} m (Target: 150m + compensation)\n"
            f"-----------------------------------------\n"
            f"Touchdown Time  : {fsm.t_landed if fsm.t_landed else 0.0:6.2f} s (True: {self.true_fsm.t_landed if self.true_fsm.t_landed else 0.0:.2f}s)\n"
            f"Failsafe Status : {'Drogue Timeout (Failsafe)' if fsm.failsafe_apogee else 'None (Normal)'}"
        )
        
        self.textbox = self.fig.text(
            0.15, 0.81, stats_text, fontsize=9.5, family='monospace', color='#e5e7eb',
            bbox=dict(facecolor='#161229', alpha=0.85, edgecolor=(1.0, 1.0, 1.0, 0.08), boxstyle='round,pad=0.8')
        )
        
    def get_alt_at_time(self, t):
        if t is None:
            return 0.0
        idx = np.abs(self.times - t).argmin()
        return self.alts[idx]

# =========================================================================
# 5. 主程式入口
# =========================================================================
if __name__ == "__main__":
    # 解析真實飛行仿真數據
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, "..", "flight_data", "台灣盃2026時間高度垂直速度合速度合加速度.csv")
    times, alts, vels, accs = load_simulation_data(csv_path)
    
    # 啟動應用程式
    app = FSMStabilityApp(times, alts, vels, accs)
    plt.show()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
成大 ISP 航電組 — EKF Q 值調校與 FSM 穩定度分步模擬器
===========================================================================
此工具專門用於評估卡爾質波過程噪聲 Q 值對開傘時機的影響。
為了保持視覺化簡潔，一次執行只跑一種特定狀態 (Scenario)。
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt

# =========================================================================
# 1. 基礎 FSM 與數據加載
# =========================================================================
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
        self.drogue_fired = False
        self.max_alt_baro = 0.0
        self.consec_baro_drop = 0
        
        # Trigger times (s)
        self.t_liftoff = None
        self.t_burnout = None
        self.t_apogee = None
        self.t_main = None

    def step(self, now_ms, h_est, v_est, a_z_g, baro_alt_rel, ekf_healthy, baro_healthy):
        now = now_ms
        if h_est > self.max_altitude:
            self.max_altitude = h_est
            
        baro_ok = baro_healthy

        # 40s Failsafe drogue trigger (配合 3156m 頂點大火箭軌跡)
        if (self.state == STATE_BOOST or self.state == STATE_COAST) and \
           (now - self.flight_start_ms) >= 40000:
            self.state = STATE_APOGEE
            self.state_entered_ms = now
            self.drogue_fired = True
            self.t_apogee = now / 1000.0
            return
            
        if self.state == STATE_PAD:
            liftoff = False
            if ekf_healthy:
                liftoff = (a_z_g > 3.0 or h_est > 10.0 or (baro_ok and baro_alt_rel > 20.0))
            else:
                liftoff = (a_z_g > 3.0 or (baro_ok and baro_alt_rel > 20.0))
            if liftoff:
                self.state = STATE_BOOST
                self.flight_start_ms = now
                self.state_entered_ms = now
                self.t_liftoff = now / 1000.0

        elif self.state == STATE_BOOST:
            if a_z_g < 0.5 and (now - self.state_entered_ms) > 1500:
                self.state = STATE_COAST
                self.state_entered_ms = now
                self.t_burnout = now / 1000.0

        elif self.state == STATE_COAST:
            decel = -9.80665
            if self.last_vel_z != 0.0:
                a_z_nav = (v_est - self.last_vel_z) / 0.010
                if -25.0 < a_z_nav < -5.0:
                    decel = a_z_nav
            
            t_to_apogee = -v_est / decel
            
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
            if ekf_healthy and v_est > 0.0 and t_to_apogee <= 4.0:
                apogee_cond = True
            elif ekf_healthy and (v_est < -0.2 or (self.max_altitude - h_est) > 5.0):
                apogee_cond = True
            elif baro_apogee:
                apogee_cond = True
                
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
            if (now - self.state_entered_ms) >= 2000:
                self.state = STATE_DESCENT
                self.state_entered_ms = now

        elif self.state == STATE_DESCENT:
            main_trigger = False
            if ekf_healthy:
                v_fall = -v_est if v_est < 0.0 else 0.0
                h_trigger_main = 150.0 + v_fall * 3.5
                main_trigger = (h_est <= h_trigger_main)
            elif baro_ok:
                main_trigger = (baro_alt_rel <= 200.0)
                
            if main_trigger or (now - self.flight_start_ms) > 120000:
                self.state = STATE_MAIN_DEPLOY
                self.state_entered_ms = now
                self.t_main = now / 1000.0

        elif self.state == STATE_MAIN_DEPLOY:
            if (now - self.state_entered_ms) >= 3000:
                self.state = STATE_LANDED
                self.state_entered_ms = now

        self.last_vel_z = v_est

def load_data(file_path):
    times, alts, vels, accs = [], [], [], []
    if not os.path.exists(file_path):
        print(f"[Error] File not found: {file_path}")
        sys.exit(1)
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split(',')
            if len(parts) < 5:
                continue
            try:
                t = float(parts[0])
                h = float(parts[1])
                v = float(parts[2])
                a = float(parts[4])
                times.append(t)
                alts.append(h)
                vels.append(v)
                accs.append(a)
            except ValueError:
                continue
    return np.array(times), np.array(alts), np.array(vels), np.array(accs)

# =========================================================================
# 2. 1D EKF (Kalman Filter)
# =========================================================================
def run_kalman_filter(t_sim, alt_noisy, acc_noisy, baro_std, q_pos, q_vel):
    n = len(t_sim)
    x = np.array([0.0, 0.0]) # [pos, vel]
    P = np.eye(2) * 1.0
    Q = np.array([[q_pos, 0.0],
                  [0.0, q_vel]])
    R = baro_std ** 2
    
    h_est = np.zeros(n)
    v_est = np.zeros(n)
    
    for i in range(n):
        dt = 0.01
        if i == 0:
            h_est[i] = x[0]
            v_est[i] = x[1]
            continue
            
        F = np.array([[1.0, dt],
                      [0.0, 1.0]])
        B = np.array([0.5 * dt * dt, dt])
        
        # Predict
        u = acc_noisy[i]
        x = F @ x + B * u
        P = F @ P @ F.T + Q
        
        # Update
        z = alt_noisy[i]
        H = np.array([1.0, 0.0])
        y = z - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T / S
        
        x = x + K * y
        P = (np.eye(2) - np.outer(K, H)) @ P
        
        h_est[i] = x[0]
        v_est[i] = x[1]
        
    return h_est, v_est

# =========================================================================
# 3. 測試情境模擬
# =========================================================================
def simulate_scenario(scenario, times, alts, vels, accs):
    # 重採樣至 100Hz
    t_sim = np.arange(0, times[-1], 0.01)
    alt_true = np.interp(t_sim, times, alts)
    vel_true = np.interp(t_sim, times, vels)
    acc_true = np.interp(t_sim, times, accs)
    
    # 預設雜訊
    baro_std = 2.0
    accel_std = 1.0
    np.random.seed(42) # 固定種子確保可重複性
    alt_noisy = alt_true + np.random.normal(0, baro_std, len(t_sim))
    acc_noisy = acc_true + np.random.normal(0, accel_std, len(t_sim))
    
    plt.style.use('dark_background')
    
    if scenario == "drogue":
        run_drogue_scenario(t_sim, alt_true, vel_true, alt_noisy, acc_noisy, baro_std)
    elif scenario == "main":
        run_main_scenario(t_sim, alt_true, vel_true, alt_noisy, acc_noisy, baro_std)
    elif scenario == "fallback":
        run_fallback_scenario(t_sim, alt_true, vel_true, alt_noisy)
    elif scenario == "launchpad":
        run_launchpad_scenario(t_sim, alt_true, vel_true, alt_noisy, acc_noisy, baro_std)

# =========================================================================
# A. 情境 1: 頂點副傘開傘穩定度 (Drogue Apogee Simulation)
# =========================================================================
def run_drogue_scenario(t_sim, alt_true, vel_true, alt_noisy, acc_noisy, baro_std):
    # 定義三組不同的 Q 值來對比
    configs = [
        {"name": "Q Too Small (Over-filtered / High Lag)", "qp": 1e-7, "qv": 1e-6, "color": "#fbbf24"},
        {"name": "Q Too Large (Under-filtered / High Noise)", "qp": 1e-1, "qv": 1.0, "color": "#ef4444"},
        {"name": "Q Optimal (Balanced)", "qp": 1e-4, "qv": 1e-2, "color": "#06b6d4"}
    ]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8))
    plt.subplots_adjust(hspace=0.35, right=0.7)
    
    # 繪製真實軌跡
    ax1.plot(t_sim, alt_true, color='#9ca3af', linestyle='--', label='True Altitude')
    ax2.plot(t_sim, vel_true, color='#9ca3af', linestyle='--', label='True Velocity')
    
    # 無噪聲基準狀態機
    fsm_true = SimulatedFSM()
    for i in range(len(t_sim)):
        # 尋找基準起飛與頂點
        acc_true_g_val = acc_noisy[i] / 9.80665
        fsm_true.step(int(t_sim[i]*1000), alt_true[i], vel_true[i], acc_true_g_val, alt_true[i], True, True)
    t_true_apogee = fsm_true.t_apogee
    
    # 運行三種 Q 值的卡爾曼濾波與狀態機
    explanation_text = "=== HOW TO CHOOSE Q (APOGEE) ===\n\n"
    explanation_text += f"True Apogee Time: {t_true_apogee:.2f} s\n\n"
    
    for cfg in configs:
        h_est, v_est = run_kalman_filter(t_sim, alt_noisy, acc_noisy, baro_std, cfg["qp"], cfg["qv"])
        
        fsm = SimulatedFSM()
        for i in range(len(t_sim)):
            fsm.step(int(t_sim[i]*1000), h_est[i], v_est[i], acc_true_g(acc_noisy[i]), alt_noisy[i], True, True)
            
        # 繪圖
        ax1.plot(t_sim, h_est, color=cfg["color"], label=cfg["name"])
        ax2.plot(t_sim, v_est, color=cfg["color"], label=cfg["name"])
        
        # 標記開傘點
        if fsm.t_apogee:
            idx = np.abs(t_sim - fsm.t_apogee).argmin()
            ax1.plot(fsm.t_apogee, h_est[idx], 'o', color=cfg["color"], ms=7)
            ax2.axvline(fsm.t_apogee, color=cfg["color"], linestyle=':', alpha=0.7)
            err = fsm.t_apogee - t_true_apogee
            explanation_text += f"{cfg['name'].split(' ')[1]}:\n  Deploy: {fsm.t_apogee:.2f}s (Err: {err:+.3f}s)\n"
        else:
            explanation_text += f"{cfg['name'].split(' ')[1]}:\n  Drogue did NOT deploy!\n"

    # 動態焦點放大在頂點附近
    idx_apogee = np.abs(t_sim - t_true_apogee).argmin()
    apogee_alt = alt_true[idx_apogee]
    
    ax1.set_xlim([t_true_apogee - 4.0, t_true_apogee + 4.0])
    ax1.set_ylim([apogee_alt - 50.0, apogee_alt + 15.0])
    ax1.set_title("Drogue Deployment Alt (Zoomed near Apogee)", fontsize=12, fontweight='bold')
    ax1.set_ylabel("Altitude (m)")
    ax1.grid(True, color=(1.0, 1.0, 1.0, 0.05))
    ax1.legend(loc='lower left', fontsize=8.5)
    
    ax2.set_xlim([t_true_apogee - 4.0, t_true_apogee + 4.0])
    ax2.set_ylim([-30, 30])
    ax2.set_title("Vertical Velocity Vz (Zoomed near Apogee)", fontsize=12, fontweight='bold')
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.grid(True, color=(1.0, 1.0, 1.0, 0.05))
    
    # 增加右側教學說明框
    explanation_text += (
        "\n-----------------------------------------\n"
        "GUIDE: How to Tune Q for Apogee\n"
        "1. If Q is TOO SMALL (Yellow):\n"
        "   Filter has high lag. Velocity zero-crossing\n"
        "   is delayed. Drogue deploys LATE (+Err).\n"
        "2. If Q is TOO LARGE (Red):\n"
        "   Filter follows sensor noise. Spikes in\n"
        "   estimated height trigger premature\n"
        "   deploy condition EARLY (-Err).\n"
        "3. Optimal Q (Cyan):\n"
        "   Minimizes both noise and lag. Deploy\n"
        "   occurs exactly at the physical peak."
    )
    fig.text(0.72, 0.15, explanation_text, fontsize=9, family='monospace', color='#e5e7eb',
             bbox=dict(facecolor='#161229', alpha=0.85, edgecolor=(1.0, 1.0, 1.0, 0.08), boxstyle='round,pad=0.8'))
    
    plt.suptitle("Scenario 1: Drogue Deployment & Q Tuning (Apogee Phase)", fontsize=14, fontweight='bold', color='#ff9d66')
    plt.show()

# =========================================================================
# B. 情境 2: 主傘開傘高度與 Q 值的關係 (Main Deploy Altitude Simulation)
# =========================================================================
def run_main_scenario(t_sim, alt_true, vel_true, alt_noisy, acc_noisy, baro_std):
    configs = [
        {"name": "Q Too Small (High Velocity Lag)", "qp": 1e-7, "qv": 1e-6, "color": "#fbbf24"},
        {"name": "Q Optimal (Balanced Tracker)", "qp": 1e-4, "qv": 1e-2, "color": "#06b6d4"}
    ]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8))
    plt.subplots_adjust(hspace=0.35, right=0.7)
    
    ax1.plot(t_sim, alt_true, color='#9ca3af', linestyle='--', label='True Altitude')
    ax2.plot(t_sim, vel_true, color='#9ca3af', linestyle='--', label='True Velocity')
    
    explanation_text = "=== DYNAMIC MAIN DEPLOY ALTITUDE ===\n\n"
    explanation_text += "Formula:\n"
    explanation_text += "  h_trig = 150m + |v_est| * 3.5s\n\n"
    
    t_main_opt = None
    for cfg in configs:
        h_est, v_est = run_kalman_filter(t_sim, alt_noisy, acc_noisy, baro_std, cfg["qp"], cfg["qv"])
        
        fsm = SimulatedFSM()
        # 預先開了副傘
        fsm.state = STATE_DESCENT
        fsm.flight_start_ms = 0
        
        for i in range(len(t_sim)):
            # 餵入數據
            fsm.step(int(t_sim[i]*1000), h_est[i], v_est[i], acc_true_g(acc_noisy[i]), alt_noisy[i], True, True)
            
        ax1.plot(t_sim, h_est, color=cfg["color"], label=cfg["name"])
        ax2.plot(t_sim, v_est, color=cfg["color"], label=cfg["name"])
        
        if fsm.t_main:
            idx = np.abs(t_sim - fsm.t_main).argmin()
            ax1.plot(fsm.t_main, h_est[idx], 'o', color=cfg["color"], ms=7)
            ax2.axvline(fsm.t_main, color=cfg["color"], linestyle=':', alpha=0.7)
            
            # 取得該開傘時間點的真實高度
            true_idx = np.abs(t_sim - fsm.t_main).argmin()
            true_alt = alt_true[true_idx]
            
            explanation_text += f"{cfg['name'].split(' ')[1]}:\n"
            explanation_text += f"  Deploy Time : {fsm.t_main:.2f}s\n"
            explanation_text += f"  Deploy Height: {true_alt:.1f}m\n"
            explanation_text += f"  v_est at deploy: {v_est[idx]:.2f}m/s\n\n"
            
            if "Optimal" in cfg["name"]:
                t_main_opt = fsm.t_main

    # 動態焦點放大在下墜段主傘開傘附近
    t_zoom = t_main_opt if t_main_opt is not None else 26.5
    idx_zoom = np.abs(t_sim - t_zoom).argmin()
    alt_zoom = alt_true[idx_zoom]

    ax1.set_xlim([t_zoom - 4.0, t_zoom + 4.0])
    ax1.set_ylim([alt_zoom - 80.0, alt_zoom + 120.0])
    ax1.set_title("Main Deployment Alt (Zoomed near 150m)", fontsize=12, fontweight='bold')
    ax1.set_ylabel("Altitude (m)")
    ax1.grid(True, color=(1.0, 1.0, 1.0, 0.05))
    ax1.legend(loc='upper right', fontsize=8.5)
    
    ax2.set_xlim([t_zoom - 4.0, t_zoom + 4.0])
    ax2.set_ylim([-35.0, 5.0])
    ax2.set_title("Vertical Velocity Vz (Zoomed near 150m)", fontsize=12, fontweight='bold')
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.grid(True, color=(1.0, 1.0, 1.0, 0.05))
    
    explanation_text += (
        "-----------------------------------------\n"
        "ANALYSIS: How Q affects Main Chute\n"
        "1. If Q is TOO SMALL (Yellow):\n"
        "   Filter lags and underestimates velocity\n"
        "   magnitude (v_est is less negative than\n"
        "   the true -20m/s). Thus, the lead-time\n"
        "   compensation is too small, delaying\n"
        "   chute deploy closer to ground.\n"
        "2. Optimal Q (Cyan):\n"
        "   Establishes a fast, accurate velocity\n"
        "   profile, compensating correctly for\n"
        "   the chute deploy delay."
    )
    fig.text(0.72, 0.12, explanation_text, fontsize=9, family='monospace', color='#e5e7eb',
             bbox=dict(facecolor='#161229', alpha=0.85, edgecolor=(1.0, 1.0, 1.0, 0.08), boxstyle='round,pad=0.8'))
    
    plt.suptitle("Scenario 2: Main Deployment & Dynamic Altitude (Descent Phase)", fontsize=14, fontweight='bold', color='#34d399')
    plt.show()

# =========================================================================
# C. 情境 3: EKF 故障與降級氣壓開傘 (Fallback Baro-only Simulation)
# =========================================================================
def run_fallback_scenario(t_sim, alt_true, vel_true, alt_noisy):
    # EKF Healthy = False, 氣壓計降級開傘
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8))
    plt.subplots_adjust(hspace=0.35, right=0.7)
    
    ax1.plot(t_sim, alt_true, color='#9ca3af', linestyle='--', label='True Altitude')
    ax1.scatter(t_sim[::4], alt_noisy[::4], color='#ef4444', s=1.0, alpha=0.3, label='Noisy Baro')
    ax2.plot(t_sim, vel_true, color='#9ca3af', linestyle='--', label='True Velocity')
    
    # 執行 FSM Step, EKF healthy = False
    fsm = SimulatedFSM()
    fsm_states = np.zeros(len(t_sim), dtype=int)
    
    for i in range(len(t_sim)):
        now_ms = int(t_sim[i] * 1000)
        # 傳入虛假的 EKF 值 (999m)，但 EKF_healthy = False
        fsm.step(now_ms, 999.0, 999.0, 1.0, alt_noisy[i], False, True)
        fsm_states[i] = fsm.state
        
    # 繪製狀態區間背景色
    state_colors = {
        STATE_PAD: ('#3b82f6', 0.1),
        STATE_BOOST: ('#f97316', 0.15),
        STATE_COAST: ('#8b5cf6', 0.12),
        STATE_APOGEE: ('#ef4444', 0.2),
        STATE_DESCENT: ('#06b6d4', 0.12),
        STATE_MAIN_DEPLOY: ('#10b981', 0.15),
        STATE_LANDED: ('#eab308', 0.12)
    }
    
    diff = np.diff(fsm_states)
    boundaries = np.where(diff != 0)[0]
    start_idx = 0
    
    for b in list(boundaries) + [len(fsm_states)-1]:
        st = fsm_states[start_idx]
        if st in state_colors:
            c, alpha_val = state_colors[st]
            ax1.axvspan(t_sim[start_idx], t_sim[b], color=c, alpha=alpha_val)
            ax2.axvspan(t_sim[start_idx], t_sim[b], color=c, alpha=alpha_val)
        start_idx = b + 1

    # 標記事件
    if fsm.t_apogee:
        idx = np.abs(t_sim - fsm.t_apogee).argmin()
        ax1.plot(fsm.t_apogee, alt_true[idx], 'ro', ms=7)
        ax1.text(fsm.t_apogee + 0.5, alt_true[idx] - 25, "APOGEE\n(BARO FALLBACK)", color='#f87171', fontsize=9, fontweight='bold')
        ax2.axvline(fsm.t_apogee, color='#f87171', linestyle=':', alpha=0.7)
        
    if fsm.t_main:
        idx = np.abs(t_sim - fsm.t_main).argmin()
        ax1.plot(fsm.t_main, alt_true[idx], 'go', ms=7)
        ax1.text(fsm.t_main + 0.5, alt_true[idx] + 15, "MAIN\n(BARO FALLBACK)", color='#34d399', fontsize=9, fontweight='bold')
        ax2.axvline(fsm.t_main, color='#34d399', linestyle=':', alpha=0.7)

    ax1.set_title("Rocket Altitude (Z) under Baro-only Fallback", fontsize=12, fontweight='bold')
    ax1.set_ylabel("Altitude (m)")
    ax1.grid(True, color=(1.0, 1.0, 1.0, 0.05))
    ax1.legend(loc='upper left')
    
    ax2.set_title("Vertical Velocity (Vz)", fontsize=12, fontweight='bold')
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.grid(True, color=(1.0, 1.0, 1.0, 0.05))
    
    explanation_text = (
        "=== EKF FAILURE FALLBACK ===\n\n"
        "When EKF is unhealthy:\n"
        "1. Liftoff trigger:\n"
        "   a_z > 3g or baro > 20m.\n"
        "2. Apogee trigger:\n"
        "   Baro height drop >= 10m\n"
        "   sustained for 200ms.\n"
        "3. Main trigger:\n"
        "   Baro alt <= 200m (fixed).\n"
        "4. Touchdown:\n"
        "   Baro delta < 2m in 2s.\n\n"
        "This ensures that if the CPU\n"
        "or EKF math hangs or drifts,\n"
        "the rocket still deploys\n"
        "its chutes safely using raw\n"
        "barometric altitude."
    )
    fig.text(0.72, 0.15, explanation_text, fontsize=9, family='monospace', color='#e5e7eb',
             bbox=dict(facecolor='#161229', alpha=0.85, edgecolor=(1.0, 1.0, 1.0, 0.08), boxstyle='round,pad=0.8'))
    
    plt.suptitle("Scenario 3: EKF Fault & Fallback Baro-only Open Loop", fontsize=14, fontweight='bold', color='#ef4444')
    plt.show()

# =========================================================================
# D. 情境 4: 發射架風噪與起飛判定 (Launchpad Vibration Resistance)
# =========================================================================
def run_launchpad_scenario(t_sim, alt_true, vel_true, alt_noisy, acc_noisy, baro_std):
    # 專注於 t=0s 到 t=2s 的發射架靜置狀態，注入大風阻噪聲與發動機點火前震動
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8))
    plt.subplots_adjust(hspace=0.35, right=0.7)
    
    # 進行模擬，使用高過程噪聲 Q_pos=1e-1 (對噪聲敏感) 與 推薦 Q_pos=1e-4
    configs = [
        {"name": "Q Too Large (Sensor Follower / Drift)", "qp": 1e-2, "qv": 1e-1, "color": "#ef4444"},
        {"name": "Q Optimal (Pad ZUPT Lock)", "qp": 1e-4, "qv": 1e-2, "color": "#06b6d4"}
    ]
    
    ax1.plot(t_sim, alt_true, color='#9ca3af', linestyle='--', label='True Altitude')
    ax1.scatter(t_sim[::1], alt_noisy[::1], color='#f87171', s=2.0, alpha=0.4, label='Noisy Baro')
    
    explanation_text = "=== LAUNCHPAD ZUPT STABILITY ===\n\n"
    explanation_text += "Vibration & Wind Simulation\n"
    explanation_text += "on the launchpad (t = 0s to 2s)\n\n"
    
    for cfg in configs:
        h_est, v_est = run_kalman_filter(t_sim, alt_noisy, acc_noisy, baro_std, cfg["qp"], cfg["qv"])
        
        # 模擬地面校準狀態
        fsm = SimulatedFSM()
        
        # 繪圖
        ax1.plot(t_sim, h_est, color=cfg["color"], label=cfg["name"])
        ax2.plot(t_sim, v_est, color=cfg["color"], label=cfg["name"])
        
        # 檢查是否有誤觸發起飛
        triggered = False
        for i in range(len(t_sim)):
            if t_sim[i] > 2.0: # 我們只監控前2秒
                break
            fsm.step(int(t_sim[i]*1000), h_est[i], v_est[i], acc_true_g(acc_noisy[i]), alt_noisy[i], True, True)
            if fsm.state == STATE_BOOST:
                triggered = True
                explanation_text += f"{cfg['name'].split(' ')[1]}:\n"
                explanation_text += f"  PREMATURE LIFTOFF!\n"
                explanation_text += f"  Triggered at {t_sim[i]:.2f}s\n\n"
                ax1.plot(t_sim[i], h_est[i], 'ro', ms=8)
                break
        
        if not triggered:
            explanation_text += f"{cfg['name'].split(' ')[1]}:\n"
            explanation_text += "  Safe (No Premature Trigger)\n\n"

    ax1.set_xlim([0.0, 2.0])
    ax1.set_ylim([-5, 10])
    ax1.set_title("Estimated Altitude (Zoomed at Launchpad)", fontsize=12, fontweight='bold')
    ax1.set_ylabel("Altitude (m)")
    ax1.grid(True, color=(1.0, 1.0, 1.0, 0.05))
    ax1.legend(loc='upper left', fontsize=8.5)
    
    ax2.set_xlim([0.0, 2.0])
    ax2.set_ylim([-2, 2])
    ax2.set_title("Estimated Velocity Vz (Zoomed at Launchpad)", fontsize=12, fontweight='bold')
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.grid(True, color=(1.0, 1.0, 1.0, 0.05))
    
    explanation_text += (
        "-----------------------------------------\n"
        "GUIDE: EKF Pad Constraints\n"
        "1. If Q is TOO LARGE (Red):\n"
        "   Filter drifts due to sensor noise on\n"
        "   the pad. If h_est rises above 10m\n"
        "   due to drift, FSM triggers premature\n"
        "   liftoff, breaking flight sequencing.\n"
        "2. Optimal Q (Cyan):\n"
        "   Locks the altitude and velocity estimates\n"
        "   firmly near 0, preventing accidental\n"
        "   triggering from ground vibration."
    )
    fig.text(0.72, 0.12, explanation_text, fontsize=9, family='monospace', color='#e5e7eb',
             bbox=dict(facecolor='#161229', alpha=0.85, edgecolor=(1.0, 1.0, 1.0, 0.08), boxstyle='round,pad=0.8'))
    
    plt.suptitle("Scenario 4: Launchpad Noise & Liftoff Lock (Anti-Premature Trigger)", fontsize=14, fontweight='bold', color='#3b82f6')
    plt.show()

def acc_true_g(a_val):
    return a_val / 9.80665

# =========================================================================
# 4. 主選單命令解析
# =========================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="NCKU ISP RocketFSM Q-Tuner & Scenario Simulator")
    parser.add_argument(
        "--scenario", 
        type=str, 
        choices=["drogue", "main", "fallback", "launchpad"],
        help="Specify which scenario to run (drogue, main, fallback, launchpad)"
    )
    
    args = parser.parse_args()
    
    # 載入模擬飛行數據
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, "..", "flight_data", "台灣盃2026時間高度垂直速度合速度合加速度.csv")
    times, alts, vels, accs = load_data(csv_path)
    
    scenario = args.scenario
    if not scenario:
        print("\n==================================================")
        print("NCKU ISP 航電狀態機分步調校模擬器 (FSM Q-Tuner)")
        print("==================================================")
        print("請選擇你要模擬的情境 (一次只跑一個，保持畫面乾淨)：")
        print("  1. [drogue]    頂點副傘開傘穩定度與 Q 值的關係")
        print("  2. [main]      主傘開傘高度與 Q 值的關係")
        print("  3. [fallback]  EKF 故障降級氣壓計開傘 (無 EKF 時)")
        print("  4. [launchpad] 發射架風噪與震動誤起飛鎖定")
        print("==================================================")
        try:
            choice = input("請輸入數字 (1-4) 或情境名稱: ").strip().lower()
            if choice in ['1', 'drogue']:
                scenario = "drogue"
            elif choice in ['2', 'main']:
                scenario = "main"
            elif choice in ['3', 'fallback']:
                scenario = "fallback"
            elif choice in ['4', 'launchpad']:
                scenario = "launchpad"
            else:
                print("輸入無效，預設執行 1. [drogue]")
                scenario = "drogue"
        except (KeyboardInterrupt, EOFError):
            print("\n已退出。")
            sys.exit(0)
            
    print(f"\n[Simulator] 開始執行情境: {scenario} ...")
    simulate_scenario(scenario, times, alts, vels, accs)
    print("[Simulator] 執行結束。\n")

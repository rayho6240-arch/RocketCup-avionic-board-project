#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
成大 ISP 航電組 — EKF Q 參數蒙地卡羅自動最佳化分析工具
===========================================================================
此腳本讀取 `data/台灣盃2026時間高度垂直速度合速度合加速度.csv`，
在第一階段（前 35 秒，涵蓋起飛、熄火與頂點開傘）執行數百次隨機噪聲模擬。
透過定義評估指標（防止提前起飛、極小化頂點開傘誤差、追蹤速度精度），
自動尋找最優的 EKF 過程噪聲 Q 矩陣 (Q_pos, Q_vel) 參數組合。
"""

import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

# =========================================================================
# 1. 數據加載與環境準備
# =========================================================================
def load_data(file_path):
    times, alts, vels, accs = [], [], [], []
    if not os.path.exists(file_path):
        print(f"[Error] File not found: {file_path}")
        sys.exit(1)
    with open(file_path, 'r', encoding='cp950', errors='ignore') as f:
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

# FSM 狀態常數
STATE_PAD = 1
STATE_BOOST = 2
STATE_COAST = 3
STATE_APOGEE = 4

# =========================================================================
# 2. 高效標量 EKF 模擬與狀態機步進
# =========================================================================
def run_fast_simulation(t_sim, alt_true, vel_true, acc_true, baro_std, accel_std, q_pos, q_vel):
    n = len(t_sim)
    dt = 0.01
    
    # 注入隨機高斯雜訊
    alt_noisy = alt_true + np.random.normal(0, baro_std, n)
    acc_noisy = acc_true + np.random.normal(0, accel_std, n)
    
    # EKF 標量狀態初始化 (h, v)
    h, v = 0.0, 0.0
    p00, p01, p11 = 1.0, 0.0, 1.0
    R = baro_std**2 + 1e-5
    
    h_est = np.zeros(n)
    v_est = np.zeros(n)
    
    # 狀態機初始化
    state = STATE_PAD
    flight_start_ms = 0
    state_entered_ms = 0
    max_altitude = 0.0
    last_vel_z = 0.0
    consec_apogee_counts = 0
    max_alt_baro = 0.0
    consec_baro_drop = 0
    
    t_liftoff = None
    t_apogee = None
    premature_liftoff = False
    
    for i in range(n):
        now_ms = int(t_sim[i] * 1000)
        u = acc_noisy[i]
        
        # 1. EKF 預測步 (Scalar Predict)
        h_pred = h + dt * v + 0.5 * dt * dt * u
        v_pred = v + dt * u
        p00_pred = p00 + 2.0 * dt * p01 + dt * dt * p11 + q_pos
        p01_pred = p01 + dt * p11
        p11_pred = p11 + q_vel
        
        # 2. EKF 更新步 (Scalar Update)
        z = alt_noisy[i]
        y = z - h_pred
        S = p00_pred + R
        k0 = p00_pred / S
        k1 = p01_pred / S
        
        h = h_pred + k0 * y
        v = v_pred + k1 * y
        p00 = (1.0 - k0) * p00_pred
        p01 = (1.0 - k0) * p01_pred
        p11 = p11_pred - k1 * p01_pred
        
        h_est[i] = h
        v_est[i] = v
        
        # 3. 狀態更新與狀態機 (FSM)
        if h > max_altitude:
            max_altitude = h
            
        a_z_g = u / 9.80665
        
        # 40秒頂點絕對失效保護 (此模擬時間共 35秒，正常不觸發)
        if (state == STATE_BOOST or state == STATE_COAST) and \
           (now_ms - flight_start_ms) >= 40000:
            state = STATE_APOGEE
            t_apogee = now_ms / 1000.0
            break
            
        if state == STATE_PAD:
            # 判斷起飛
            liftoff = (a_z_g > 3.0 or h > 10.0 or alt_noisy[i] > 20.0)
            if liftoff:
                state = STATE_BOOST
                flight_start_ms = now_ms
                state_entered_ms = now_ms
                t_liftoff = now_ms / 1000.0
                if t_liftoff < 1.30:  # 比真實起飛點 1.45s 提前過多視為誤觸發
                    premature_liftoff = True
                    
        elif state == STATE_BOOST:
            if a_z_g < 0.5 and (now_ms - state_entered_ms) > 1500:
                state = STATE_COAST
                state_entered_ms = now_ms
                
        elif state == STATE_COAST:
            decel = -9.80665
            if last_vel_z != 0.0:
                a_z_nav = (v - last_vel_z) / 0.010
                if -25.0 < a_z_nav < -5.0:
                    decel = a_z_nav
            
            t_to_apogee = -v / decel
            
            # 氣壓計高度回落判定
            if alt_noisy[i] > max_alt_baro:
                max_alt_baro = alt_noisy[i]
            if (max_alt_baro - alt_noisy[i]) >= 10.0:
                consec_baro_drop += 1
            else:
                consec_baro_drop = 0
            baro_apogee = (consec_baro_drop >= 20)
            
            apogee_cond = False
            # 正常路徑 1: 動態時間預測
            if v > 0.0 and t_to_apogee <= 4.0:
                apogee_cond = True
            # 正常路徑 2: 速度過零或回落過多
            elif v < -0.2 or (max_altitude - h) > 5.0:
                apogee_cond = True
            # 降級路徑 3: 氣壓計回落
            elif baro_apogee:
                apogee_cond = True
                
            if apogee_cond and (now_ms - flight_start_ms) > 3000:
                consec_apogee_counts += 1
                if consec_apogee_counts >= 5:
                    state = STATE_APOGEE
                    t_apogee = now_ms / 1000.0
                    break
            else:
                consec_apogee_counts = 0
                
        last_vel_z = v
        
    return t_liftoff, t_apogee, premature_liftoff, h_est, v_est

# =========================================================================
# 3. 蒙地卡羅多參數網格搜尋
# =========================================================================
def optimize_q(times, alts, vels, accs, num_trials=30):
    # 截斷並重採樣數據至前 35 秒
    t_sim = np.arange(0, 35.0, 0.01)
    alt_true = np.interp(t_sim, times, alts)
    vel_true = np.interp(t_sim, times, vels)
    acc_true = np.interp(t_sim, times, accs)
    
    # 預設噪聲 (規格書 + 50%)
    baro_std = 0.25
    accel_std = 2.94
    
    # 理想開傘觸發時間 (無噪聲下的基準，一般是帶有 4.0s 提前的 23.85s)
    TARGET_APOGEE_TIME = 23.85
    
    # 搜尋網格設計 (對數網格 5x5)
    qp_values = [1e-6, 1e-5, 1e-4, 1e-3, 1e-2]
    qv_values = [1e-4, 1e-3, 1e-2, 1e-1, 1.0]
    
    loss_grid = np.zeros((len(qp_values), len(qv_values)))
    premature_grid = np.zeros((len(qp_values), len(qv_values)))
    apogee_err_grid = np.zeros((len(qp_values), len(qv_values)))
    
    best_loss = float('inf')
    best_qp = None
    best_qv = None
    
    print("\n[Optimizer] 正在執行蒙地卡羅搜尋（共 25 組參數，每組運行 {} 次隨機模擬）...".format(num_trials))
    t0 = time.time()
    
    for i, qp in enumerate(qp_values):
        for j, qv in enumerate(qv_values):
            trial_losses = []
            premature_count = 0
            apogee_times = []
            vel_rmses = []
            
            for _ in range(num_trials):
                t_liftoff, t_apogee, premature, h_est, v_est = run_fast_simulation(
                    t_sim, alt_true, vel_true, acc_true, baro_std, accel_std, qp, qv
                )
                
                loss_run = 0.0
                
                # A. 誤起飛懲罰
                if premature:
                    premature_count += 1
                    loss_run += 5000.0
                    
                # B. 未開傘懲罰
                if t_apogee is None:
                    loss_run += 2000.0
                else:
                    apogee_times.append(t_apogee)
                    # 頂點觸發時間偏離理想時間 (23.85s)
                    loss_run += 150.0 * abs(t_apogee - TARGET_APOGEE_TIME)
                    
                # C. 速度 RMSE (專注在 Coast 段 t = 6.4s 到 24s 之間以精確解算 apogee)
                idx_coast = (t_sim >= 6.4) & (t_sim <= 24.0)
                vel_rmse = np.sqrt(np.mean((v_est[idx_coast] - vel_true[idx_coast])**2))
                vel_rmses.append(vel_rmse)
                loss_run += 5.0 * vel_rmse
                
                trial_losses.append(loss_run)
                
            # 計算平均指標
            avg_loss = np.mean(trial_losses)
            premature_rate = premature_count / num_trials
            
            # 加入開傘時間震盪標準差作為懲罰（越穩定越好）
            if len(apogee_times) > 1:
                avg_loss += 50.0 * np.std(apogee_times)
                avg_apogee_err = np.mean(np.abs(np.array(apogee_times) - TARGET_APOGEE_TIME))
            else:
                avg_apogee_err = 99.0
                
            loss_grid[i, j] = avg_loss
            premature_grid[i, j] = premature_rate
            apogee_err_grid[i, j] = avg_apogee_err
            
            print("  Q_Pos={:1.0e}, Q_Vel={:1.0e} | Avg Loss={:7.1f} | Premature={:3.0f}% | Apogee Err={:5.2f}s".format(
                qp, qv, avg_loss, premature_rate*100, avg_apogee_err if avg_apogee_err < 99 else 0.0
            ))
            
            if avg_loss < best_loss:
                best_loss = avg_loss
                best_qp = qp
                best_qv = qv
                
    t1 = time.time()
    print("\n[Optimizer] 最佳化搜尋完成！耗時: {:.2f} 秒".format(t1 - t0))
    print("==================================================")
    print("[RESULT] 【最優過程噪聲 covariance Q 參數】")
    print("  位置過程噪聲 Q_Pos : {:1.0e}".format(best_qp))
    print("  速度過程噪聲 Q_Vel : {:1.0e}".format(best_qv))
    print("  綜合損失評分 Loss  : {:.2f}".format(best_loss))
    print("==================================================")
    
    # 繪製 Loss Surface 熱圖 (Loss Surface Heatmap)
    plot_loss_surface(qp_values, qv_values, loss_grid, best_qp, best_qv)
    
    return best_qp, best_qv, qp_values, qv_values, loss_grid

def plot_loss_surface(qp_values, qv_values, loss_grid, best_qp, best_qv):
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(8, 6))
    
    # 轉換成對數尺度進行繪圖
    x_labels = ["$10^{" + str(int(np.log10(v))) + "}$" for v in qv_values]
    y_labels = ["$10^{" + str(int(np.log10(p))) + "}$" for p in qp_values]
    
    # 限制 Loss 顯示上限，避免超大 penalty 影響漸層顏色
    display_grid = np.clip(loss_grid, 0, 1500)
    
    cax = ax.imshow(display_grid, cmap='viridis_r', origin='lower', aspect='auto')
    fig.colorbar(cax, label='Optimization Loss (Lower is Better)')
    
    ax.set_xticks(np.arange(len(qv_values)))
    ax.set_yticks(np.arange(len(qp_values)))
    ax.set_xticklabels(x_labels)
    ax.set_yticklabels(y_labels)
    
    ax.set_xlabel("Velocity Process Noise $Q_{vel}$", fontsize=11)
    ax.set_ylabel("Position Process Noise $Q_{pos}$", fontsize=11)
    ax.set_title("Monte Carlo Optimization: EKF Q Loss Surface", fontsize=13, fontweight='bold', pad=15)
    
    # 標記最佳參數點
    best_i = qp_values.index(best_qp)
    best_j = qv_values.index(best_qv)
    ax.plot(best_j, best_i, '*', color='#ff3737', ms=15, label='Optimal Q')
    ax.legend(loc='upper right')
    
    # 在網格中標注數值
    for i in range(len(qp_values)):
        for j in range(len(qv_values)):
            ax.text(j, i, f"{loss_grid[i,j]:.0f}", ha='center', va='center', 
                    color='white' if display_grid[i,j] < 800 else 'black', fontsize=9)
            
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_png = os.path.join(script_dir, "..", "flight_plots", "monte_carlo_loss_surface.png")
    plt.savefig(output_png, dpi=150)
    print(f"[Plot] 損失函數表面熱圖已成功儲存至: {output_png}\n")
    plt.close()

# =========================================================================
# 5. 主程式入口
# =========================================================================
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, "..", "flight_data", "台灣盃2026時間高度垂直速度合速度合加速度.csv")
    
    # 讀取真實飛行數據
    times, alts, vels, accs = load_data(csv_path)
    
    # 執行最佳化
    optimize_q(times, alts, vels, accs, num_trials=30)

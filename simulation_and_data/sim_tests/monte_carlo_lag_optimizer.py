#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
成大 ISP 航電組 — EKF Q 參數與動態 R 矩陣蒙地卡羅最佳化分析工具（考慮氣壓計滯後）
===========================================================================
此腳本讀取 `data/台灣盃2026時間高度垂直速度合速度合加速度.csv`，
在模擬中導入非線性氣壓計滯後模型（基於通氣閥孔數 N=2, 4, 6, 8 進行建模），
並在數據中加入雜訊（氣壓計高斯雜訊 0.25m，加速度計高斯雜訊 2.94 m/s²）。
針對每個 N，在 EKF 中運行：
  Case A: 靜態 R = 0.25^2 = 0.0625 m²
  Case B: 動態 R = R_base + alpha * (v_est)^2  （其中 alpha = (0.738 / N)^2）
透過蒙地卡羅多參數網格搜尋，找出最優的 EKF 過程噪聲 Q_pos 與 Q_vel。
"""

import os
import sys
import time
import numpy as np

# =========================================================================
# 1. 數據加載與環境準備
# =========================================================================
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

# FSM 狀態常數
STATE_PAD = 1
# EKF 1D 垂直狀態機
STATE_BOOST = 2
STATE_COAST = 3
STATE_APOGEE = 4

# 大氣物理常數
def alt_to_press(h):
    # h (m) -> P (mbar)
    # Clamp to prevent negative or complex values
    ratio = 1.0 - 0.0065 * h / 288.15
    ratio = np.clip(ratio, 0.01, 1.5)
    return 1013.25 * (ratio ** 5.255877)

def press_to_alt(P):
    # P (mbar) -> h (m)
    ratio = np.clip(P / 1013.25, 0.001, 2.0)
    return 44330.7692 * (1.0 - (ratio ** 0.190263))

# =========================================================================
# 2. 氣壓計滯後模擬
# =========================================================================
def simulate_baro_lag(t_sim, alt_true, N):
    """
    依據閥門螺絲數量 N 模擬氣壓滯後，求解 ODE：
    dPin/dt = - (Pin / V) * N * Q_spec * ((Pin - Pout) / 70.0)
    """
    V = 9.8e-5 # m^3
    Q_spec = 9.1667e-6 # m^3/s
    dt = 0.01
    n = len(t_sim)
    
    P_out = alt_to_press(alt_true)
    P_in = np.zeros(n)
    P_in[0] = P_out[0] # 起始處於平衡狀態
    
    # ODE: dP/dt = g(P, P_ext)
    # g(P, P_ext) = - (P / V) * N * Q_spec * ((P - P_ext) / 70.0)
    # 可簡化為: dP/dt = - C * P * (P - P_ext)
    # C = (N * Q_spec) / (V * 70.0)
    C = (N * Q_spec) / (V * 70.0)
    
    for i in range(n - 1):
        P_curr = P_in[i]
        P_ext_curr = P_out[i]
        P_ext_next = P_out[i+1]
        P_ext_mid = 0.5 * (P_ext_curr + P_ext_next)
        
        # RK4 數值積分
        k1 = - C * P_curr * (P_curr - P_ext_curr)
        
        P_half1 = P_curr + 0.5 * dt * k1
        k2 = - C * P_half1 * (P_half1 - P_ext_mid)
        
        P_half2 = P_curr + 0.5 * dt * k2
        k3 = - C * P_half2 * (P_half2 - P_ext_mid)
        
        P_next = P_curr + dt * k3
        k4 = - C * P_next * (P_next - P_ext_next)
        
        P_in[i+1] = P_curr + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        
    alt_perc = press_to_alt(P_in)
    return alt_perc

# =========================================================================
# 3. 高效 EKF + FSM 模擬步進
# =========================================================================
def run_fast_simulation(t_sim, alt_true, vel_true, acc_true, alt_lagged, 
                         baro_std, accel_std, q_pos, q_vel, use_dynamic_R, alpha):
    n = len(t_sim)
    dt = 0.01
    
    # 注入高斯隨機雜訊
    alt_noisy = alt_lagged + np.random.normal(0, baro_std, n)
    acc_noisy = acc_true + np.random.normal(0, accel_std, n)
    
    # EKF 標量狀態初始化 (h, v)
    h, v = 0.0, 0.0
    p00, p01, p11 = 1.0, 0.0, 1.0
    R_base = baro_std**2
    
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
        
        # R_k 調適
        if use_dynamic_R:
            R = R_base + alpha * (v**2)
        else:
            R = R_base
            
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
        
        # 3. FSM 邏輯步進
        if h > max_altitude:
            max_altitude = h
            
        a_z_g = u / 9.80665
        
        # 40秒安全時間保護
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
                if t_liftoff < 1.30:  # 比真實起飛點 1.45s 提前過多視為誤起飛
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
            
            # 氣壓計相對落差計數
            if alt_noisy[i] > max_alt_baro:
                max_alt_baro = alt_noisy[i]
            if (max_alt_baro - alt_noisy[i]) >= 10.0:
                consec_baro_drop += 1
            else:
                consec_baro_drop = 0
            baro_apogee = (consec_baro_drop >= 20)
            
            apogee_cond = False
            # 正常路徑 1: 動態時間預測 (剩餘 4.0秒內頂點預測)
            if v > 0.0 and t_to_apogee <= 4.0:
                apogee_cond = True
            # 正常路徑 2: 速度過零 (<-0.2m/s) 或高度回落超過 5.0m
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
# 4. 網格搜尋優化核心
# =========================================================================
def run_optimization(t_sim, alt_true, vel_true, acc_true, alt_lagged, 
                     baro_std, accel_std, use_dynamic_R, alpha, num_trials, target_apogee_time):
    # 網格空間設計
    qp_values = [1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 5e-2]
    qv_values = [1e-4, 1e-3, 1e-2, 1e-1, 5e-1, 1.0]
    
    best_loss = float('inf')
    best_qp = None
    best_qv = None
    best_apogee_err = 99.0
    best_apogee_std = 99.0
    best_vel_rmse = 99.0
    best_premature_rate = 1.0
    
    for qp in qp_values:
        for qv in qv_values:
            trial_losses = []
            premature_count = 0
            apogee_times = []
            vel_rmses = []
            
            for _ in range(num_trials):
                t_liftoff, t_apogee, premature, h_est, v_est = run_fast_simulation(
                    t_sim, alt_true, vel_true, acc_true, alt_lagged,
                    baro_std, accel_std, qp, qv, use_dynamic_R, alpha
                )
                
                loss_run = 0.0
                
                # A. 誤起飛懲罰
                if premature:
                    premature_count += 1
                    loss_run += 5000.0
                    
                # B. 未開傘或嚴重延誤開傘懲罰
                if t_apogee is None:
                    loss_run += 2000.0
                else:
                    apogee_times.append(t_apogee)
                    # 偏離無噪聲基準開傘時間的誤差
                    loss_run += 150.0 * abs(t_apogee - target_apogee_time)
                    
                # C. 速度估計精度 RMSE (主要在 Coast 段 t = 6.4s ~ 24s)
                idx_coast = (t_sim >= 6.4) & (t_sim <= 24.0)
                vel_rmse = np.sqrt(np.mean((v_est[idx_coast] - vel_true[idx_coast])**2))
                vel_rmses.append(vel_rmse)
                loss_run += 5.0 * vel_rmse
                
                trial_losses.append(loss_run)
                
            avg_loss = np.mean(trial_losses)
            premature_rate = premature_count / num_trials
            
            # 加入開傘時間的標準差作為穩定性懲罰
            if len(apogee_times) > 1:
                apogee_std = np.std(apogee_times)
                avg_loss += 50.0 * apogee_std
                avg_apogee_err = np.mean(np.abs(np.array(apogee_times) - target_apogee_time))
            else:
                apogee_std = 99.0
                avg_apogee_err = 99.0
                
            if avg_loss < best_loss:
                best_loss = avg_loss
                best_qp = qp
                best_qv = qv
                best_apogee_err = avg_apogee_err
                best_apogee_std = apogee_std
                best_vel_rmse = np.mean(vel_rmses)
                best_premature_rate = premature_rate
                
    return best_qp, best_qv, best_loss, best_apogee_err, best_apogee_std, best_vel_rmse, best_premature_rate

# =========================================================================
# 5. 主程式
# =========================================================================
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, "..", "flight_data", "台灣盃2026時間高度垂直速度合速度合加速度.csv")
    
    times, alts, vels, accs = load_data(csv_path)
    
    # 截斷並重採樣前 35 秒數據 (涵蓋升空到頂點後)
    t_sim = np.arange(0, 35.0, 0.01)
    alt_true = np.interp(t_sim, times, alts)
    vel_true = np.interp(t_sim, times, vels)
    acc_true = np.interp(t_sim, times, accs)
    
    # 雜訊設定
    baro_std = 0.25     # m
    accel_std = 2.94    # m/s^2
    num_trials = 40     # 每個網格點運行 40 次蒙地卡羅
    
    print("=======================================================================")
    print("NCKU ISP Avionics — EKF Q-Matrix & Dynamic R Optimization")
    print("=======================================================================")
    print(f"Data Source   : {os.path.basename(csv_path)}")
    print(f"Baro Std Dev  : {baro_std} m  (Datasheet + 50%)")
    print(f"Accel Std Dev : {accel_std} m/s^2 (Datasheet + 50%)")
    print(f"Monte Carlo   : {num_trials} trials per grid point")
    print("=======================================================================\n")
    
    results = []
    
    # 對不同的通氣孔數 N = 2, 4, 6, 8 進行分析
    for N in [2, 4, 6, 8]:
        print(f"[STEP] 正在分析 N = {N} 個排氣孔...")
        
        # 1. 產生該 N 下的氣壓滯後高度
        alt_lagged = simulate_baro_lag(t_sim, alt_true, N)
        
        # 2. 計算無雜訊下的 FSM 基準開傘時間
        # 在無雜訊、無滯後下跑一次 FSM 得到基準時間 (代表理想完美 EKF 下的觸發時間)
        _, t_apogee_ideal, _, _, _ = run_fast_simulation(
            t_sim, alt_true, vel_true, acc_true, alt_true, 
            0.001, 0.001, 0.01, 0.1, use_dynamic_R=False, alpha=0.0
        )
        if t_apogee_ideal is None:
            t_apogee_ideal = 23.85 # fallback
        
        # 計算此 N 下的 R_k 調適係數 alpha
        # 根據物理推導，時間常數 tau = 0.738 / N， alpha = tau^2 = (0.738/N)^2
        alpha = (0.738 / N) ** 2
        
        # 3. 優化 Case A: 靜態 R
        print(f"  [Case A] 正在最佳化 靜態 R...")
        qp_a, qv_a, loss_a, err_a, std_a, rmse_a, prem_a = run_optimization(
            t_sim, alt_true, vel_true, acc_true, alt_lagged,
            baro_std, accel_std, use_dynamic_R=False, alpha=0.0,
            num_trials=num_trials, target_apogee_time=t_apogee_ideal
        )
        print(f"    最佳參數: Q_pos={qp_a:1.0e}, Q_vel={qv_a:1.0e} | Avg Loss={loss_a:.1f} | Apogee Err={err_a:.2f}s | Vel RMSE={rmse_a:.2f} m/s")
        
        # 4. 優化 Case B: 動態 R
        print(f"  [Case B] 正在最佳化 動態 R (alpha={alpha:.5f})...")
        qp_b, qv_b, loss_b, err_b, std_b, rmse_b, prem_b = run_optimization(
            t_sim, alt_true, vel_true, acc_true, alt_lagged,
            baro_std, accel_std, use_dynamic_R=True, alpha=alpha,
            num_trials=num_trials, target_apogee_time=t_apogee_ideal
        )
        print(f"    最佳參數: Q_pos={qp_b:1.0e}, Q_vel={qv_b:1.0e} | Avg Loss={loss_b:.1f} | Apogee Err={err_b:.2f}s | Vel RMSE={rmse_b:.2f} m/s")
        
        results.append({
            'N': N,
            'ideal_time': t_apogee_ideal,
            'alpha': alpha,
            # Case A
            'qp_a': qp_a, 'qv_a': qv_a, 'loss_a': loss_a, 'err_a': err_a, 'std_a': std_a, 'rmse_a': rmse_a, 'prem_a': prem_a,
            # Case B
            'qp_b': qp_b, 'qv_b': qv_b, 'loss_b': loss_b, 'err_b': err_b, 'std_b': std_b, 'rmse_b': rmse_b, 'prem_b': prem_b
        })
        print()
        
    # =========================================================================
    # 6. 輸出優化對比報告
    # =========================================================================
    print("=============================================================================================")
    print("[TABLE] EKF 優化對比總表 (氣壓滯後滯後 vs 動態/靜態 R)")
    print("=============================================================================================")
    print(" N |  R 模式 |  Q_pos  |  Q_vel  |  對比 Loss | 開傘誤差 (s) | 速度 RMSE (m/s) | 誤起飛率")
    print("---------------------------------------------------------------------------------------------")
    for r in results:
        N = r['N']
        # Case A
        print(f" {N} |  Static  | {r['qp_a']:7.1e} | {r['qv_a']:7.1e} |   {r['loss_a']:7.1f}  |   {r['err_a']:8.2f}   |    {r['rmse_a']:9.2f}    |  {r['prem_a']*100:6.1f}%")
        # Case B
        print(f" {N} |  Dynamic | {r['qp_b']:7.1e} | {r['qv_b']:7.1e} |   {r['loss_b']:7.1f}  |   {r['err_b']:8.2f}   |    {r['rmse_b']:9.2f}    |  {r['prem_b']*100:6.1f}%")
        print("---------------------------------------------------------------------------------------------")
    
    print("\n[NOTE] 最佳化結論分析：")
    for r in results:
        N = r['N']
        improvement = (r['loss_a'] - r['loss_b']) / r['loss_a'] * 100
        if improvement > 0:
            print(f"- [N = {N}] 動態 R 將綜合 Loss 降低了 {improvement:.1f}%。開傘時間誤差從 {r['err_a']:.2f}s 降至 {r['err_b']:.2f}s。")
        else:
            print(f"- [N = {N}] 動態 R 與靜態 R 表現相當。最佳參數為 Q_pos={r['qp_b']:1.0e}, Q_vel={r['qv_b']:1.0e}。")
    print("=============================================================================================")

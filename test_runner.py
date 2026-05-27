#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
航電板自動化硬體迴路測試 (HIL) 腳本 (Mac 版)
此腳本可自動執行以下步驟：
1. 自動編譯 (make)
2. 自動燒錄 (STM32_Programmer_CLI)
3. 自動監聽串口 (Serial) 並對傳回數據進行斷言分析 (Assertion & Verification)
"""

import os
import sys
import time
import subprocess
import re

try:
    import serial
except ImportError:
    print("正在自動安裝所需的 pyserial 套件...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
    import serial

# ==================== 配置參數 ====================
# 1. 編譯配置
COMPILER_PATH = "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.macos64_1.0.0.202411102158/tools/bin"
DEBUG_DIR = "/Users/laizhiquan/coding/RocketCom/Main_Code/Debug"
ELF_FILE = f"{DEBUG_DIR}/Main_AV_F407.elf"

# 2. 燒錄配置 (使用 Mac 上偵測到的唯一 ST-Link SN: 35FF67065057393437231143)
PROGRAMMER_CLI = "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macos64_2.2.200.202503041107/tools/bin/STM32_Programmer_CLI"
ST_LINK_SN = "35FF67065057393437231143"

# 3. 串口配置 (使用偵測到的 CH340 晶片串口 /dev/cu.usbserial-110)
SERIAL_PORT = "/dev/cu.usbserial-110"
BAUD_RATE = 115200
TEST_DURATION = 30  # 測試監聽時間 (秒)，涵蓋 Flash_Test + SD 卡掛載 + 10 秒 CSV 寫入週期

# 4. 預期指標判定標準 (HIL 驗證)
EXPECTED_MIN_BMI088_A = 1550.0  # Hz (TIM6 實測約 1600 Hz)
EXPECTED_MIN_BMI088_G = 1950.0  # Hz (TIM7 實測約 2000 Hz)
EXPECTED_MIN_ADXL375 = 3100.0   # Hz (TIM3 實測約 3200 Hz)
EXPECTED_MIN_BMP388 = 180.0     # Hz (實測約 187-200 Hz)

# 5. Flash Ring Buffer 驗收標準
EXPECTED_MIN_RING_PACKETS = 400  # 30s × 20Hz = 600 筆，允許初始化延遲，要求至少 400 筆
# ==================================================

def run_command(cmd, cwd=None, env=None):
    """執行命令並即時輸出日誌"""
    print(f"执行命令: {cmd}")
    process = subprocess.Popen(
        cmd,
        shell=True,
        cwd=cwd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )
    
    output_lines = []
    while True:
        line = process.stdout.readline()
        if not line and process.poll() is not None:
            break
        if line:
            print(f"  {line.strip()}")
            output_lines.append(line)
            
    return process.returncode, "".join(output_lines)

def build_project():
    """1. 編譯專案"""
    print("\n" + "="*50)
    print("【第一步：自動編譯專案】")
    print("="*50)
    
    # 設置編譯環境變數 PATH 引入 Arm 工具鏈
    env = os.environ.copy()
    env["PATH"] = f"{COMPILER_PATH}:{env.get('PATH', '')}"
    
    cmd = "make clean && make -j4 Main_AV_F407.elf"
    ret, out = run_command(cmd, cwd=DEBUG_DIR, env=env)
    
    if ret != 0:
        print("\n❌ 編譯失敗！請檢查編譯錯誤。")
        return False
    
    print("\n✅ 編譯成功！產出檔案：", ELF_FILE)
    return True

def flash_mcu():
    """2. 燒錄韌體"""
    print("\n" + "="*50)
    print("【第二步：自動燒錄韌體】")
    print("="*50)
    
    if not os.path.exists(ELF_FILE):
        print(f"❌ 找不到 ELF 檔案：{ELF_FILE}")
        return False
        
    cmd = f'"{PROGRAMMER_CLI}" -c port=SWD sn={ST_LINK_SN} mode=UR -d "{ELF_FILE}" -v -rst'
    ret, out = run_command(cmd)
    
    if "Download verified successfully" not in out:
        print("\n❌ 燒錄或驗證失敗！")
        return False
        
    print("\n✅ 韌體燒錄成功，並已重新啟動航電板 (MCU Reset)！")
    return True

def monitor_serial_and_verify():
    """3. 監聽串口並進行 HIL 斷言分析"""
    print("\n" + "="*50)
    print(f"【第三步：監聽串口數據 & HIL 自動化測試】")
    print(f"串口: {SERIAL_PORT} @ {BAUD_RATE} Baud | 監聽時長: {TEST_DURATION} 秒")
    print("="*50)
    
    # 用於儲存收集到的採樣率
    bmi_acc_rates = []
    bmi_gyro_rates = []
    adxl_rates = []
    bmp_rates = []

    # Flash Ring Buffer 監控
    flash_ring_ready = False
    flash_ring_final_pkt = 0
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        # 不 reset buffer：MCU 已在跑，buffer 裡可能有有效資料
        
        start_time = time.time()
        print("  正在從航電板接收即時遥測數據...")
        
        while time.time() - start_time < TEST_DURATION:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                # 打印原始遙測數據或採樣率報告
                print(f"  [Tele]: {line}")
                
                # 匹配 [RATE] BMI088_A:1600Hz, BMI088_G:2000Hz, ADXL375:3200Hz, BMP388:196Hz
                match = re.search(r"\[RATE\]\s*BMI088_A:(\d+)Hz,\s*BMI088_G:(\d+)Hz,\s*ADXL375:(\d+)Hz,\s*BMP388:(\d+)Hz", line)
                if match:
                    bmi_a_r = float(match.group(1))
                    bmi_g_r = float(match.group(2))
                    adxl_r = float(match.group(3))
                    bmp_r = float(match.group(4))

                    # 跳過暖機期：任一感測器為 0 表示 timer 尚未完整計數
                    if bmi_a_r == 0 or bmi_g_r == 0 or adxl_r == 0 or bmp_r == 0:
                        print(f"  ⏭️ 【跳過暖機期數據（含 0Hz）】")
                        continue

                    bmi_acc_rates.append(bmi_a_r)
                    bmi_gyro_rates.append(bmi_g_r)
                    adxl_rates.append(adxl_r)
                    bmp_rates.append(bmp_r)

                    print(f"  📢 【捕獲採樣率】：BMI088_A={bmi_a_r}Hz | BMI088_G={bmi_g_r}Hz | ADXL375={adxl_r}Hz | BMP388={bmp_r}Hz")

                # Flash Ring Buffer 監控
                if "[FLASH_RING] Ready." in line:
                    flash_ring_ready = True
                    print(f"  ✅ 【Flash Ring Buffer 初始化成功】")

                ring_total_match = re.search(r"\[FLASH_RING\] PKT_TOTAL:(\d+)", line)
                if ring_total_match:
                    flash_ring_final_pkt = int(ring_total_match.group(1))

        ser.close()
    except Exception as e:
        print(f"\n❌ 串口連線或讀取失敗: {e}")
        return False

    # 4. HIL 測試斷言與分析報告
    print("\n" + "="*50)
    print("【第四步：HIL 測試自動化斷言報告】")
    print("="*50)
    
    if not bmi_acc_rates:
        print("❌ HIL 失敗：未在接收的數據中檢測到任何 [RATE] 採樣率資訊。")
        return False
        
    avg_bmi_a = sum(bmi_acc_rates) / len(bmi_acc_rates)
    avg_bmi_g = sum(bmi_gyro_rates) / len(bmi_gyro_rates)
    avg_adxl = sum(adxl_rates) / len(adxl_rates)
    avg_bmp = sum(bmp_rates) / len(bmp_rates)
    
    print(f"📊 平均採樣率統計結果：")
    print(f"  - BMI088 (Accel)     : {avg_bmi_a:.1f} Hz (目標門檻: >={EXPECTED_MIN_BMI088_A} Hz)")
    print(f"  - BMI088 (Gyro)      : {avg_bmi_g:.1f} Hz (目標門檻: >={EXPECTED_MIN_BMI088_G} Hz)")
    print(f"  - ADXL375 (High-G)   : {avg_adxl:.1f} Hz (目標門檻: >={EXPECTED_MIN_ADXL375} Hz)")
    print(f"  - BMP388 (Baro)      : {avg_bmp:.1f} Hz (目標門檻: >={EXPECTED_MIN_BMP388} Hz)")
    
    # 斷言檢查
    failures = []
    if avg_bmi_a < EXPECTED_MIN_BMI088_A:
        failures.append(f"BMI088 Accel 採樣率過低 ({avg_bmi_a:.1f} Hz)")
    if avg_bmi_g < EXPECTED_MIN_BMI088_G:
        failures.append(f"BMI088 Gyro 採樣率過低 ({avg_bmi_g:.1f} Hz)")
    if avg_adxl < EXPECTED_MIN_ADXL375:
        failures.append(f"ADXL375 採樣率過低 ({avg_adxl:.1f} Hz)")
    if avg_bmp < EXPECTED_MIN_BMP388:
        failures.append(f"BMP388 採樣率過低 ({avg_bmp:.1f} Hz)")
        
    # Flash Ring Buffer 斷言
    print(f"\n📦 Flash Ring Buffer 統計：")
    print(f"  - 初始化成功 : {'✅ 是' if flash_ring_ready else '❌ 否（未收到 Ready 訊息）'}")
    print(f"  - 寫入封包數 : {flash_ring_final_pkt} 筆 (目標門檻: >={EXPECTED_MIN_RING_PACKETS} 筆)")

    if not flash_ring_ready:
        failures.append("Flash Ring Buffer 初始化失敗（未收到 [FLASH_RING] Ready. 訊息）")
    if flash_ring_final_pkt < EXPECTED_MIN_RING_PACKETS:
        failures.append(f"Flash Ring Buffer 寫入封包不足 ({flash_ring_final_pkt} 筆 < {EXPECTED_MIN_RING_PACKETS})")

    if failures:
        print("\n❌ 測試結果：【不通過 FAILED】")
        for f in failures:
            print(f"  ⚠️ {f}")
        return False
    else:
        print("\n🎉🎉 測試結果：【通過 PASSED】🎉🎉")
        print("  所有感測器皆穩定運作於官方設計與規範之最高工作頻率！")
        print(f"  Flash Ring Buffer 成功寫入 {flash_ring_final_pkt} 筆飛行數據至 W25Q128。")
        return True

if __name__ == "__main__":
    print("🚀 啟動航電板自動化硬體迴路測試 (HIL)...")
    
    if not build_project():
        sys.exit(1)
        
    if not flash_mcu():
        sys.exit(1)
        
    if not monitor_serial_and_verify():
        sys.exit(2)
        
    sys.exit(0)

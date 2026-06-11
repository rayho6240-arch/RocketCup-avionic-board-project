#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
航電板自動化硬體迴路測試 (HIL) 腳本 (Mac 版)

使用方式:
  python3 test_runner.py           # 完整流程：編譯 → 燒錄 → 監聽 → 斷言
  python3 test_runner.py --monitor # 只監聽 serial（跳過編譯燒錄，適合即時 debug）
  python3 test_runner.py --flash   # 燒錄後監聽（跳過編譯）
"""

import os
import sys
import time
import subprocess
import re
import argparse
from datetime import datetime

try:
    import serial
except ImportError:
    print("正在自動安裝所需的 pyserial 套件...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
    import serial

# ==================== 配置參數 ====================
COMPILER_PATH = "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.macos64_1.0.0.202411102158/tools/bin"
DEBUG_DIR = "/Users/laizhiquan/coding/RocketCom/Main_Code/Debug"
ELF_FILE = f"{DEBUG_DIR}/Main_AV_F407.elf"

PROGRAMMER_CLI = "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macos64_2.2.200.202503041107/tools/bin/STM32_Programmer_CLI"
ST_LINK_SN = "35FF67065057393437231143"

SERIAL_PORT = "/dev/cu.usbserial-110"
BAUD_RATE = 460800
TEST_DURATION = 30  # 完整 HIL 測試監聽時間 (秒)
MONITOR_DURATION = 0   # --monitor 模式持續時間 (秒，0 = 無限)

# HIL 斷言門檻
EXPECTED_MIN_BMI088_A = 1550.0
EXPECTED_MIN_BMI088_G = 1950.0
EXPECTED_MIN_ADXL375  = 3100.0
EXPECTED_MIN_BMP388   = 140.0
EXPECTED_MIN_MMC5983  = 80.0
EXPECTED_MIN_GPS      = 1.0
EXPECTED_MIN_RING_PACKETS = 300
# ==================================================

def run_command(cmd, cwd=None, env=None):
    process = subprocess.Popen(cmd, shell=True, cwd=cwd, env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, bufsize=1)
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
    print("\n" + "="*60)
    print("【第一步：自動編譯專案】")
    print("="*60)
    env = os.environ.copy()
    env["PATH"] = f"{COMPILER_PATH}:{env.get('PATH', '')}"
    ret, out = run_command("make clean && make -j4 Main_AV_F407.elf", cwd=DEBUG_DIR, env=env)
    if ret != 0:
        print("\n❌ 編譯失敗！")
        return False
    print("\n✅ 編譯成功！")
    return True

def flash_mcu():
    print("\n" + "="*60)
    print("【自動燒錄韌體】")
    print("="*60)
    if not os.path.exists(ELF_FILE):
        print(f"❌ 找不到 ELF 檔案：{ELF_FILE}")
        return False
    cmd = f'"{PROGRAMMER_CLI}" -c port=SWD sn={ST_LINK_SN} mode=UR -d "{ELF_FILE}" -v -rst'
    ret, out = run_command(cmd)
    if "Download verified successfully" not in out:
        print("\n❌ 燒錄或驗證失敗！")
        return False
    print("\n✅ 韌體燒錄成功，MCU 已重啟！")
    return True

# ─────────────────────────────────────────────────────────────
#  顏色輸出 (ANSI)
# ─────────────────────────────────────────────────────────────
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def col(color, text): return f"{color}{text}{RESET}"

# ─────────────────────────────────────────────────────────────
#  Debug 監控模式
# ─────────────────────────────────────────────────────────────
def monitor_debug(duration_s=0):
    """
    即時監聽 serial，專注診斷 GPS 和 Mag 狀態。
    duration_s = 0 → 無限迴圈直到 Ctrl-C
    """
    print("\n" + "="*60)
    print(col(BOLD, f"【Debug 監聽模式】{SERIAL_PORT} @ {BAUD_RATE} baud"))
    print(f"  持續時間: {'無限 (Ctrl-C 停止)' if duration_s == 0 else f'{duration_s} 秒'}")
    print("="*60)

    log_filename = "serial_continuous.log"
    print(col(GREEN, f"  📝 所有讀取數據將同步追加寫入 {log_filename}\n"))

    # ── 狀態追蹤 ──
    state = {
        # Mag
        "mag_init": None,          # None / "online" / "not_detected"
        "mag_offset": None,
        "mag_reads_ok": 0,
        "mag_reads_err": 0,
        "mag_last_B": None,        # (Bx, By, Bz) mG
        "mag_last_hdg": None,

        # GPS
        "gps_fix": 0,
        "gps_sats": 0,
        "gps_sentences_ok": 0,
        "gps_sentences_err": 0,
        "gps_last_lat": None,
        "gps_last_lon": None,
        "gps_seen_any": False,

        # IMU/Baro rates
        "bmi_a": 0, "bmi_g": 0, "adxl": 0, "bmp": 0,

        # EKF
        "ekf_calib": False,
        "ekf_drop": 0,

        # System
        "flash_ring_ready": False,
        "flash_pkt": 0,
    }

    # LED 預測
    def predict_leds():
        stat1 = state["gps_sentences_ok"] > 0
        stat2 = state["mag_init"] == "online"
        fix   = state["gps_fix"] == 1

        stat1_str = (col(GREEN, "● 常亮 (有fix)") if (stat1 and fix) else
                     col(YELLOW, "◎ 閃爍 (有NMEA無fix)") if stat1 else
                     col(RED, "○ 熄滅"))
        stat2_str = col(GREEN, "● 常亮") if stat2 else col(RED, "○ 熄滅")
        print(f"\n  {col(BOLD,'LED 預測')}  STAT1(GPS)={stat1_str}  STAT2(MAG)={stat2_str}")

    def print_separator():
        print(col(CYAN, "  " + "─"*56))

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        print(f"  串口已開啟。等待 MCU 資料...\n")
        start = time.time()
        last_summary = time.time()

        while True:
            if duration_s > 0 and (time.time() - start) > duration_s:
                break

            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='replace').strip()
            if not line:
                continue

            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            # 寫入持續日誌檔
            with open(log_filename, "a", encoding="utf-8") as f_log:
                f_log.write(f"[{ts}] {line}\n")

            # ── Mag 初始化 ──
            if "[MAG] MMC5983MA online" in line:
                state["mag_init"] = "online"
                m = re.search(r"offset\[X,Y,Z\]=(-?\d+),(-?\d+),(-?\d+)", line)
                if m:
                    state["mag_offset"] = tuple(int(m.group(i)) for i in range(1,4))
                print(col(GREEN, f"  [{ts}] ✅ MAG ONLINE  offset={state['mag_offset']}"))

            elif "[MAG] MMC5983MA NOT detected" in line:
                state["mag_init"] = "not_detected"
                print(col(RED, f"  [{ts}] ❌ MAG NOT DETECTED — 請確認 I2C1 接線 (SDA=PB8, SCL=PB7) 及 3V3 供電"))

            # ── Mag 週期遙測  [MAG] B[mG]:x,y,z hdg:d ok:n err:n ──
            elif line.startswith("[MAG]") and "B[mG]" in line:
                m = re.search(r"B\[mG\]:(-?\d+),(-?\d+),(-?\d+).*hdg:(\d+).*ok:(\d+).*err:(\d+)", line)
                if m:
                    bx,by,bz = int(m.group(1)),int(m.group(2)),int(m.group(3))
                    hdg = int(m.group(4))
                    state["mag_last_B"]    = (bx,by,bz)
                    state["mag_last_hdg"]  = hdg
                    state["mag_reads_ok"]  = int(m.group(5))
                    state["mag_reads_err"] = int(m.group(6))
                    print(col(GREEN,
                        f"  [{ts}] MAG  B=({bx:+5d},{by:+5d},{bz:+5d}) mG  "
                        f"hdg={hdg}°  ok={state['mag_reads_ok']}  err={state['mag_reads_err']}"))

            # ── GPS 週期遙測  [GPS] fix:x q:x sat:x ... ok:n err:n ──
            elif line.startswith("[GPS]"):
                state["gps_seen_any"] = True
                mf  = re.search(r"fix:(\d)", line)
                ms  = re.search(r"sat:(\d+)", line)
                mok = re.search(r"ok:(\d+)", line)
                mer = re.search(r"err:(\d+)", line)
                mla = re.search(r"([NS])(\d+)\.(\d+)", line)
                mlo = re.search(r"([EW])(\d+)\.(\d+)", line)

                if mf:  state["gps_fix"]  = int(mf.group(1))
                if ms:  state["gps_sats"] = int(ms.group(1))
                if mok: state["gps_sentences_ok"]  = int(mok.group(1))
                if mer: state["gps_sentences_err"] = int(mer.group(1))

                fix_col = col(GREEN,"✅ FIX") if state["gps_fix"] else col(YELLOW,"⏳ 搜星中")
                sat_col = col(GREEN if state["gps_sats"]>=4 else YELLOW, f"sats={state['gps_sats']}")
                ok_col  = col(GREEN if state["gps_sentences_ok"]>0 else RED,
                              f"ok={state['gps_sentences_ok']}")
                err_col = col(RED if state["gps_sentences_err"]>0 else RESET,
                              f"err={state['gps_sentences_err']}")

                print(f"  [{ts}] GPS  {fix_col}  {sat_col}  {ok_col}  {err_col}")

                if state["gps_sentences_ok"] == 0 and state["gps_sentences_err"] > 10:
                    print(col(RED,
                        "  ⚠️  GPS 有資料進來但 checksum 全失敗 → "
                        "可能 baud rate 不符（NEO-M9N 預設 38400，板設 115200）"))

            # ── RATE 報告 ──
            elif line.startswith("[RATE]"):
                m = re.search(r"BMI088_A:([\d\.]+)Hz.*BMI088_G:([\d\.]+)Hz.*ADXL375:([\d\.]+)Hz.*BMP388:([\d\.]+)Hz", line)
                if m:
                    state["bmi_a"],state["bmi_g"],state["adxl"],state["bmp"] = \
                        float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))
                mmc_rate = re.search(r"MMC5983:([\d\.]+)Hz", line)
                gps_rate = re.search(r"GPS:([\d\.]+)Hz", line)
                mag_rate_val = float(mmc_rate.group(1)) if mmc_rate else 0.0
                gps_rate_val = float(gps_rate.group(1)) if gps_rate else 0.0

                ekf_drop = re.search(r"EKF_DROP:(\d+)", line)
                if ekf_drop: state["ekf_drop"] = int(ekf_drop.group(1))
                print(f"  [{ts}] RATE  "
                      f"BMI-A={state['bmi_a']:.2f}Hz  BMI-G={state['bmi_g']:.2f}Hz  "
                      f"ADXL={state['adxl']:.2f}Hz  BMP={state['bmp']:.2f}Hz  "
                      f"MAG={mag_rate_val:.2f}Hz  GPS={gps_rate_val:.2f}Hz  "
                      f"EKF_DROP={state['ekf_drop']}")

            # ── EKF 校準完成 ──
            elif "Calibration Done" in line or "EKF_calibrated" in line:
                state["ekf_calib"] = True
                print(col(GREEN, f"  [{ts}] ✅ EKF 校準完成"))

            # ── Flash Ring ──
            elif "[FLASH_RING] Ready" in line:
                state["flash_ring_ready"] = True
                print(col(GREEN, f"  [{ts}] ✅ Flash Ring Buffer 就緒"))
            else:
                m_pkt = re.search(r"PKT_TOTAL:(\d+)", line)
                if m_pkt: state["flash_pkt"] = int(m_pkt.group(1))
                # 其他行原樣印出（EKF TELE、系統訊息等）
                if any(line.startswith(p) for p in ("[TELE]","[EKF]","[SD]","[FLASH]","[IMU]")):
                    print(f"  [{ts}] {line}")

            # ── 每 10 秒打印一次摘要 ──
            if time.time() - last_summary >= 10:
                last_summary = time.time()
                print_separator()
                print(f"  {col(BOLD,'─── 10s 診斷摘要 ───')}")

                # Mag
                if state["mag_init"] is None:
                    print(col(YELLOW,"  MAG: 尚未收到初始化訊息（MCU 可能還在啟動中）"))
                elif state["mag_init"] == "not_detected":
                    print(col(RED,"  MAG: ❌ init 失敗 — 硬體未連接或 I2C 接線錯誤"))
                    print(col(RED,"       ↳ 確認 SDA=PB8 / SCL=PB7 / 3V3 / 4.7kΩ pull-up"))
                else:
                    print(col(GREEN,f"  MAG: ✅ online  最新 hdg={state['mag_last_hdg']}°  "
                               f"reads_ok={state['mag_reads_ok']}  err={state['mag_reads_err']}"))

                # GPS
                if not state["gps_seen_any"]:
                    print(col(RED,"  GPS: ❌ 完全沒收到 [GPS] 行"))
                    print(col(RED,"       ↳ 可能原因: GPS TX 未接 PC7, 或 baud rate 錯誤"))
                    print(col(RED,"       ↳ NEO-M9N 預設 baud=38400，板設 115200，若不一致需用 U-Center 設定"))
                elif state["gps_sentences_ok"] == 0:
                    print(col(YELLOW,f"  GPS: ⚠️  有收到遙測但 sentences_ok=0  err={state['gps_sentences_err']}"))
                    print(col(YELLOW,"       ↳ 可能 baud rate 不符 → NMEA checksum 全失敗"))
                else:
                    fix_s = "有 fix ✅" if state["gps_fix"] else "搜星中 ⏳"
                    print(col(GREEN,f"  GPS: ✅ sentences_ok={state['gps_sentences_ok']}  "
                               f"sats={state['gps_sats']}  {fix_s}"))

                predict_leds()
                print_separator()

        ser.close()

    except serial.SerialException as e:
        print(col(RED, f"\n❌ 串口連線失敗: {e}"))
        print(col(YELLOW, f"   請確認 {SERIAL_PORT} 是否正確，或改用 --port /dev/cu.xxx"))
        return False
    except KeyboardInterrupt:
        print("\n\n  [使用者中斷]")

    # 最終報告
    print("\n" + "="*60)
    print(col(BOLD, "【最終診斷報告】"))
    print("="*60)

    ok = True

    mag_result = (state["mag_init"] == "online")
    print(f"  MAG  : {col(GREEN,'✅ online') if mag_result else col(RED,'❌ NOT detected')}")
    if not mag_result:
        ok = False
        print(col(RED,"         → 確認 I2C1 (PB8=SDA, PB7=SCL), 3V3, 4.7kΩ pull-up"))

    gps_alive = state["gps_sentences_ok"] > 0
    gps_fix   = state["gps_fix"] == 1
    if gps_alive:
        print(f"  GPS  : {col(GREEN,'✅ NMEA ok')}  sats={state['gps_sats']}  "
              f"fix={'yes ✅' if gps_fix else 'no (需戶外)'}")
    else:
        ok = False
        if state["gps_seen_any"]:
            print(col(YELLOW,f"  GPS  : ⚠️  有遙測但 sentences_ok=0 err={state['gps_sentences_err']}"))
            print(col(YELLOW,"         → baud rate 不符? NEO-M9N 預設 38400，需 U-Center 改 115200"))
        else:
            print(col(RED,"  GPS  : ❌ 完全沒收到 [GPS] 行"))
            print(col(RED,"         → GPS TX 未接 PC7? 或模組未供電?"))

    print(f"\n  STAT1 預測: {col(GREEN,'● 閃爍/常亮') if gps_alive else col(RED,'○ 熄滅')}")
    print(f"  STAT2 預測: {col(GREEN,'● 常亮') if mag_result else col(RED,'○ 熄滅')}")

    return ok

# ─────────────────────────────────────────────────────────────
#  完整 HIL 測試模式（原有功能保留）
# ─────────────────────────────────────────────────────────────
def monitor_serial_and_verify():
    print("\n" + "="*60)
    print(f"【HIL 監聽 & 自動斷言】| 時長: {TEST_DURATION}s")
    print("="*60)

    bmi_acc_rates, bmi_gyro_rates, adxl_rates, bmp_rates = [], [], [], []
    mmc_rates, gps_rates = [], []
    flash_ring_ready = False
    flash_ring_final_pkt = 0

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        start_time = time.time()
        print("  接收遙測中...")

        while time.time() - start_time < TEST_DURATION:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue
                print(f"  [Tele]: {line}")

                m = re.search(r"\[RATE\]\s*BMI088_A:([\d\.]+)Hz,\s*BMI088_G:([\d\.]+)Hz,\s*ADXL375:([\d\.]+)Hz,\s*BMP388:([\d\.]+)Hz,\s*MMC5983:([\d\.]+)Hz,\s*GPS:([\d\.]+)Hz", line)
                if m:
                    a,g,x,b = float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))
                    m_rate, g_rate = float(m.group(5)), float(m.group(6))
                    if 0.0 in (a,g,x,b):
                        print("  ⏭️ 暖機期，跳過")
                        continue
                    bmi_acc_rates.append(a); bmi_gyro_rates.append(g)
                    adxl_rates.append(x);    bmp_rates.append(b)
                    mmc_rates.append(m_rate); gps_rates.append(g_rate)
                    print(f"  📢 RATE BMI-A={a:.2f}Hz G={g:.2f}Hz ADXL={x:.2f}Hz BMP={b:.2f}Hz MAG={m_rate:.2f}Hz GPS={g_rate:.2f}Hz")

                if "[FLASH_RING] Ready." in line:
                    flash_ring_ready = True
                    start_time = time.time()
                    print("  ✅ Flash Ring 就緒，重置計時")

                m2 = re.search(r"\[FLASH_RING\] PKT_TOTAL:(\d+)", line)
                if m2: flash_ring_final_pkt = int(m2.group(1))

        ser.close()
    except Exception as e:
        print(f"\n❌ 串口錯誤: {e}")
        return False

    print("\n" + "="*60)
    print("【HIL 斷言結果】")
    print("="*60)

    if not bmi_acc_rates:
        print("❌ 未收到 [RATE] 資料")
        return False

    avg = lambda lst: sum(lst)/len(lst)
    results = {
        "BMI088 Accel": (avg(bmi_acc_rates), EXPECTED_MIN_BMI088_A),
        "BMI088 Gyro" : (avg(bmi_gyro_rates), EXPECTED_MIN_BMI088_G),
        "ADXL375"     : (avg(adxl_rates),     EXPECTED_MIN_ADXL375),
        "BMP388"      : (avg(bmp_rates),       EXPECTED_MIN_BMP388),
        "MMC5983MA"   : (avg(mmc_rates),       EXPECTED_MIN_MMC5983),
        "GPS"         : (avg(gps_rates),       EXPECTED_MIN_GPS),
    }
    failures = []
    for name, (val, thr) in results.items():
        ok = val >= thr
        sym = "✅" if ok else "❌"
        print(f"  {sym} {name}: {val:.1f} Hz (>= {thr})")
        if not ok: failures.append(f"{name} 過低 ({val:.1f}Hz)")

    print(f"\n  Flash Ring: {'✅' if flash_ring_ready else '❌'} ready | pkt={flash_ring_final_pkt} (>= {EXPECTED_MIN_RING_PACKETS})")
    if not flash_ring_ready: failures.append("Flash Ring 未就緒")
    if flash_ring_final_pkt < EXPECTED_MIN_RING_PACKETS:
        failures.append(f"Flash pkt 不足 ({flash_ring_final_pkt})")

    if failures:
        print(col(RED, "\n❌ FAILED"))
        for f in failures: print(f"  ⚠️ {f}")
        return False
    print(col(GREEN, f"\n🎉 PASSED — 所有感測器正常，Flash 寫入 {flash_ring_final_pkt} 筆"))
    return True

# ─────────────────────────────────────────────────────────────
#  入口
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="航電板 HIL 測試工具")
    parser.add_argument("--monitor", action="store_true", help="只監聽 serial（跳過編譯燒錄）")
    parser.add_argument("--flash",   action="store_true", help="燒錄後監聽（跳過編譯）")
    parser.add_argument("--port",    default=SERIAL_PORT,  help=f"串口路徑 (預設 {SERIAL_PORT})")
    parser.add_argument("--duration",type=int, default=MONITOR_DURATION, help="監聽秒數 (0=無限)")
    args = parser.parse_args()

    SERIAL_PORT = args.port  # 允許命令列覆蓋

    if args.monitor:
        print("🔍 Debug 監聽模式（不編譯不燒錄）")
        monitor_debug(args.duration)
        sys.exit(0)

    if args.flash:
        print("⚡ 燒錄 + 監聽模式")
        if not flash_mcu(): sys.exit(1)
        time.sleep(2)
        monitor_debug(args.duration)
        sys.exit(0)

    # 完整 HIL 流程
    print("🚀 完整 HIL 測試流程（編譯 → 燒錄 → 斷言）")
    if not build_project(): sys.exit(1)
    if not flash_mcu():     sys.exit(1)
    if not monitor_serial_and_verify(): sys.exit(2)
    sys.exit(0)

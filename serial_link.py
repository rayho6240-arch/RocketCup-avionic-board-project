"""
serial_link.py — RocketCom 串口共用模組（P3）
===========================================================================
統一 test_runner.py / gui_monitor.py / ekf_visualizer.py 的 port/baud 預設、
自動偵測與開埠錯誤處理（原本三份腳本各有一套寫死預設與互異的偵測邏輯）。

GroundStation/telemetry_decoder.py 刻意不依賴本模組 —— 地面站筆電可能只
複製那一支檔案，保持自包含是功能不是重複。

用法：
    import serial_link
    ser = serial_link.open_serial()                  # 全自動
    ser = serial_link.open_serial("/dev/cu.x", 460800, timeout=0.5)
"""
import sys

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial：pip3 install -r requirements.txt")
    sys.exit(1)

DEFAULT_BAUD   = 460800                      # printf 重定向 USART2 的固定鮑率
PREFERRED_PORT = "/dev/cu.usbserial-110"     # 實測過的 USB-TTL 橋接器（macOS）

# USB-to-UART 橋接器關鍵字（排除藍牙等虛擬埠）
_USB_KEYWORDS = ("usbserial", "usbmodem", "ch34", "cp210", "ftdi", "uart",
                 "ttyusb", "ttyacm")


def list_candidate_ports():
    """回傳所有串口裝置路徑（macOS 一律轉 cu.*，避免 tty.* 阻塞 open）。"""
    out = []
    for p in list_ports.comports():
        dev = p.device
        if sys.platform == "darwin" and "tty." in dev:
            dev = dev.replace("tty.", "cu.")
        out.append(dev)
    return out


def auto_port():
    """自動偵測：優先實測過的橋接器，其次 USB 關鍵字符合者，找不到回 None。
    刻意不退回「清單第一個」—— 藍牙 / wlan-debug 等虛擬埠絕不可能是航電。"""
    import os
    if os.path.exists(PREFERRED_PORT):
        return PREFERRED_PORT
    for dev in list_candidate_ports():
        if any(kw in dev.lower() for kw in _USB_KEYWORDS):
            return dev
    return None


def resolve_port(port=None):
    """port 為 None / "" / "AUTO"（不分大小寫）→ 自動偵測；否則原樣回傳。"""
    if port and str(port).upper() != "AUTO":
        return port
    return auto_port() or PREFERRED_PORT


def open_serial(port=None, baud=DEFAULT_BAUD, timeout=1.0):
    """解析 port 後開埠並清空輸入緩衝。失敗印出統一診斷（含可用串口清單）
    並重拋 serial.SerialException，由呼叫端決定收尾方式。"""
    port = resolve_port(port)
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        avail = ", ".join(list_candidate_ports()) or "(無)"
        print(f"❌ 無法開啟串口 {port}: {e}")
        print(f"   可用串口: {avail}")
        raise

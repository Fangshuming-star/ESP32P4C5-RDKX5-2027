# -*- coding: utf-8 -*-
"""
ESP32-P4C5 无线调试工具（图形界面）

功能：
  1. 串口监视（RX 区实时显示，自动提取 P4 IP）
  2. TCP 通讯（TX 区记录发送，RX 区记录接收）
  3. OTA 无线升级（含下载进度条 + 成功/失败弹窗）
  4. 固件烧录（C5 / P4 / 自定义芯片，通过 esptool）
  5. 中英文界面切换

运行：
  双击 启动P4工具.bat
  或 python p4_tool.py
"""
import os
import sys
import json
import queue
import socket
import threading
import subprocess
import re
import time
import functools
import http.server
import socketserver
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
from tkinter import simpledialog

# Windows 下隐藏 subprocess 弹出的控制台窗口（构建/烧录不再弹黑框）
CREATE_NO_WINDOW = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    serial = None

# ---------------- 路径 ----------------
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(TOOLS_DIR, "tool_config.json")

IDF_PATH = os.environ.get("IDF_PATH", r"D:\ESP32-IDF\Espressif\frameworks\esp-idf-v5.5.5")
IDF_TOOLS_PATH = os.environ.get("IDF_TOOLS_PATH", r"D:\ESP32-IDF\Espressif")
IDF_PYTHON_ENV_PATH = os.environ.get(
    "IDF_PYTHON_ENV_PATH",
    os.path.join(IDF_TOOLS_PATH, "python_env", "idf5.5_py3.11_env"))
IDF_TMP_DIR = r"C:\esp_tmp"
ESPTOOL = os.path.join(IDF_PYTHON_ENV_PATH, "Scripts", "esptool.exe")

TCP_PORT = 3333
HTTP_PORT = 8000
BAUDRATE = 115200


def _find_dir_with_file(start, filename, max_depth=6):
    """从 start 向上查找包含 filename 的目录。"""
    d = os.path.abspath(start)
    for _ in range(max_depth):
        if os.path.isfile(os.path.join(d, filename)):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


def _find_p4_project():
    """定位 P4 项目根（含 sdkconfig.defaults 的目录，即 p4_wifi）。"""
    # 脚本自身所在目录可能就是项目根
    if os.path.isfile(os.path.join(TOOLS_DIR, "sdkconfig.defaults")):
        return TOOLS_DIR
    # 向上查找
    found = _find_dir_with_file(TOOLS_DIR, "sdkconfig.defaults")
    if found:
        return found
    # 兜底：脚本上级
    return os.path.dirname(TOOLS_DIR)


def _find_c5_prebuilt():
    """定位 C5 预编译固件目录（c5_firmware/prebuilt）。"""
    d = os.path.abspath(TOOLS_DIR)
    for _ in range(6):
        # 从脚本位置向上查找 c5_firmware/prebuilt
        p = os.path.join(d, "c5_firmware", "prebuilt")
        if os.path.isdir(p):
            return p
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    # 兜底：从 P4 项目根找归档目录
    p4 = _find_p4_project()
    p = os.path.join(p4, "esp32p4c5_wireless parameter tuning and code download",
                     "c5_firmware", "prebuilt")
    if os.path.isdir(p):
        return p
    return os.path.join(TOOLS_DIR, "c5_firmware", "prebuilt")


# 默认路径：P4 项目根 = p4_wifi（含 sdkconfig.defaults），C5 固件 = 归档目录
DEFAULT_P4_PROJECT = _find_p4_project()
DEFAULT_C5_PREBUILT = _find_c5_prebuilt()

# OTA 超时（秒）
OTA_DOWNLOAD_TIMEOUT = 180   # 等待 P4 下载固件的超时
OTA_REBOOT_TIMEOUT = 180     # 等待 P4 重启的超时

IP_RE = re.compile(r"got ip[:\s]*([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)")
ANSI_RE = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")


def strip_ansi(s):
    return ANSI_RE.sub("", s)


# ---------------- i18n ----------------
TEXTS = {
    "zh": {
        "title": "ESP32-P4C5 无线调试工具",
        "conn_settings": "连接设置",
        "serial": "串口",
        "baud": "波特率",
        "connect_serial": "连接串口",
        "disconnect_serial": "断开串口",
        "refresh": "刷新",
        "p4_ip": "P4 IP",
        "ping": "Ping",
        "language": "语言",
        "ready": "就绪",
        "wifi_ssid": "WiFi 名称",
        "wifi_password": "WiFi 密码",
        "wifi_save": "保存 WiFi 配置",
        "wifi_saved": "✅ 已保存 WiFi 配置: {ssid}",
        "wifi_ssid_ascii_title": "WiFi 名称不支持中文",
        "wifi_ssid_ascii_msg": "WiFi 名称（SSID）只能使用英文字母、数字和常用符号。\n\n请将 WiFi 名称改为英文后重试。\n例如: REDMI Turbo 4\n\n当前输入: {ssid}",
        "rx_area": "接收区 (RX)",
        "tx_area": "发送区 (TX)",
        "serial_rx_area": "有线串口接收",
        "net_rx_area": "无线网络接收",
        "build_log_area": "构建日志",
        "show_serial_rx": "显示串口接收",
        "show_net_rx": "显示网络接收",
        "clear_logs": "清空日志",
        "tab_tcp": "TCP 通讯",
        "tab_ota": "OTA 升级",
        "tab_flash": "固件烧录",
        "tcp_msg": "消息",
        "tcp_send": "发送",
        "tcp_send_ota_manual": "发送 ota 指令(手动)",
        "ota_fw": "固件",
        "ota_browse": "浏览",
        "ota_build": "1. 构建固件",
        "ota_upgrade": "2. OTA 无线升级",
        "ota_hint": "(升级前请先连接串口并 Ping 通)",
        "ota_progress": "下载进度",
        "build_progress": "构建进度",
        "flash_chip": "芯片",
        "flash_files": "烧录文件 (偏移地址 + 文件)",
        "flash_add": "添加文件",
        "flash_del": "删除",
        "flash_clear": "清空",
        "flash_load_c5": "载入 C5 默认固件",
        "flash_load_p4": "载入 P4 默认固件",
        "flash_start": "开始烧录",
        "flash_port": "串口",
        "flash_baud": "烧录波特率",
        "flash_progress": "烧录进度",
        "serial_connected": "已连接 {port} @ {baud}",
        "serial_disconnected": "已断开",
        "serial_fail": "串口打开失败",
        "serial_no_pyserial": "未安装 pyserial，请先安装：pip install pyserial",
        "ping_ok": "可达 ✅",
        "ping_fail": "不可达 ❌",
        "build_start": "开始构建固件...",
        "build_success": "✅ 构建成功",
        "build_fail": "❌ 构建失败 (code={code})",
        "ota_start": "OTA 固件:",
        "ota_url": "OTA URL:",
        "ota_http_up": "HTTP 服务器已启动 (端口 {port})",
        "ota_triggered": "已触发，正在等待 P4 下载固件...",
        "ota_download_ok": "✅ P4 已成功下载固件",
        "ota_download_fail": "❌ 下载超时：未检测到固件下载",
        "ota_serial_ok": "✅ 串口检测到升级成功标志，P4 即将/正在重启",
        "ota_success": "✅✅ 升级成功！P4 已重启并运行新固件。",
        "ota_dl_no_reboot": "⚠️ 固件已下载，但未确认重启成功，请查看串口日志。",
        "ota_fail": "❌ 升级失败：P4 未下载固件。请检查防火墙与网络。",
        "ota_trigger_fail": "❌ 触发失败：无法连接 P4。请检查 IP 与网络。",
        "msgbox_ota_success": "✅ 升级成功！\nP4 已重启并运行新固件。",
        "msgbox_ota_warn": "⚠️ 固件已下载，但未确认重启成功。\n请查看串口日志确认。",
        "msgbox_ota_error": "❌ 升级失败：P4 未下载固件。\n请检查防火墙与网络连通性。",
        "msgbox_ota_trigger": "触发失败：无法连接 P4 ({ip})。\n请检查 IP 与网络。",
        "flash_starting": "开始烧录 {chip} 到 {port} ...",
        "flash_ok": "✅ 烧录成功",
        "flash_fail": "❌ 烧录失败 (code={code})",
        "flash_no_esptool": "找不到 esptool，请检查 ESP-IDF 安装。",
        "flash_no_files": "请先添加要烧录的文件。",
        "flash_no_port": "请先选择串口。",
        "flash_manual_boot_title": "烧录前准备（手动进入下载模式）",
        "flash_manual_boot_msg": "即将烧录 {chip} 到串口 {port}。\n\n"
            "请现在【按住 BOOT 键不放】，然后点击「是」开始烧录。\n\n"
            "esptool 启动后会显示 \"Connecting...\"，此时：\n"
            "  · BOOT 键仍然保持按住\n"
            "  · 再点按一下 RST 键（按下后立即松开）\n"
            "  · 保持按住 BOOT，直到看到 \"Chip is ...\" 或下载进度条\n\n"
            "看到进度条后再松开 BOOT 键。\n\n"
            "现在已按住 BOOT 键了吗？\n"
            "点击「是」开始烧录，「否」取消。",
        "sel_files": "选择固件文件",
        "all_files": "所有文件",
        "input_offset": "输入偏移地址",
        "offset_prompt": "输入 {file} 的偏移地址 (hex 或 decimal):",
        "auto_ip": "[自动识别] P4 IP = {ip}",
        "ping_start": "[Ping] {ip} ...",
        "serial_avail": "[串口] 可用端口: {ports}",
        "http_req": "[HTTP] {msg}",
        "tcp_tx": "[TCP] → {ip}:{port}  {msg}",
        "tcp_rx": "[TCP] ← {msg}",
        "tcp_fail": "[TCP] 失败: {e}",
    },
    "en": {
        "title": "ESP32-P4C5 Wireless Debug Tool",
        "conn_settings": "Connection",
        "serial": "Port",
        "baud": "Baud",
        "connect_serial": "Connect",
        "disconnect_serial": "Disconnect",
        "refresh": "Refresh",
        "p4_ip": "P4 IP",
        "ping": "Ping",
        "language": "Language",
        "ready": "Ready",
        "wifi_ssid": "WiFi SSID",
        "wifi_password": "WiFi Password",
        "wifi_save": "Save WiFi config",
        "wifi_saved": "✅ WiFi config saved: {ssid}",
        "wifi_ssid_ascii_title": "Non-ASCII SSID not supported",
        "wifi_ssid_ascii_msg": "WiFi name (SSID) may only contain ASCII letters, digits and common symbols.\n\nPlease use an English WiFi name and retry.\nExample: REDMI Turbo 4\n\nCurrent input: {ssid}",
        "rx_area": "Receive (RX)",
        "tx_area": "Transmit (TX)",
        "serial_rx_area": "Wired Serial RX",
        "net_rx_area": "Wireless RX",
        "build_log_area": "Build Log",
        "show_serial_rx": "Show serial RX",
        "show_net_rx": "Show network RX",
        "clear_logs": "Clear logs",
        "tab_tcp": "TCP",
        "tab_ota": "OTA Update",
        "tab_flash": "Flash Firmware",
        "tcp_msg": "Message",
        "tcp_send": "Send",
        "tcp_send_ota_manual": "Send 'ota' cmd (manual)",
        "ota_fw": "Firmware",
        "ota_browse": "Browse",
        "ota_build": "1. Build",
        "ota_upgrade": "2. OTA Update",
        "ota_hint": "(Connect serial & ping first)",
        "ota_progress": "Download progress",
        "build_progress": "Build progress",
        "flash_chip": "Chip",
        "flash_files": "Files (offset + file)",
        "flash_add": "Add file",
        "flash_del": "Delete",
        "flash_clear": "Clear",
        "flash_load_c5": "Load C5 defaults",
        "flash_load_p4": "Load P4 defaults",
        "flash_start": "Start Flash",
        "flash_port": "Port",
        "flash_baud": "Flash baud",
        "flash_progress": "Flash progress",
        "serial_connected": "Connected {port} @ {baud}",
        "serial_disconnected": "Disconnected",
        "serial_fail": "Serial open failed",
        "serial_no_pyserial": "pyserial not installed. Run: pip install pyserial",
        "ping_ok": "reachable ✅",
        "ping_fail": "unreachable ❌",
        "build_start": "Building firmware...",
        "build_success": "✅ Build succeeded",
        "build_fail": "❌ Build failed (code={code})",
        "ota_start": "OTA firmware:",
        "ota_url": "OTA URL:",
        "ota_http_up": "HTTP server started (port {port})",
        "ota_triggered": "Triggered, waiting for P4 to download...",
        "ota_download_ok": "✅ P4 downloaded firmware",
        "ota_download_fail": "❌ Download timeout: no download detected",
        "ota_serial_ok": "✅ Upgrade success marker seen on serial, P4 rebooting",
        "ota_success": "✅✅ Upgrade success! P4 rebooted with new firmware.",
        "ota_dl_no_reboot": "⚠️ Downloaded but reboot not confirmed. Check serial log.",
        "ota_fail": "❌ Upgrade failed: P4 did not download. Check firewall/network.",
        "ota_trigger_fail": "❌ Trigger failed: cannot reach P4. Check IP/network.",
        "msgbox_ota_success": "✅ Upgrade success!\nP4 rebooted with new firmware.",
        "msgbox_ota_warn": "⚠️ Downloaded but reboot not confirmed.\nCheck serial log.",
        "msgbox_ota_error": "❌ Upgrade failed: P4 did not download.\nCheck firewall/network.",
        "msgbox_ota_trigger": "Trigger failed: cannot reach P4 ({ip}).\nCheck IP/network.",
        "flash_starting": "Flashing {chip} to {port} ...",
        "flash_ok": "✅ Flash succeeded",
        "flash_fail": "❌ Flash failed (code={code})",
        "flash_no_esptool": "esptool not found. Check ESP-IDF install.",
        "flash_no_files": "Add at least one file to flash.",
        "flash_no_port": "Select a serial port first.",
        "flash_manual_boot_title": "Prepare before flashing (manual download mode)",
        "flash_manual_boot_msg": "About to flash {chip} to port {port}.\n\n"
            "Now HOLD DOWN the BOOT button, then click Yes to start.\n\n"
            "When esptool shows \"Connecting...\":\n"
            "  · Keep holding BOOT\n"
            "  · Tap the RST button once (press and release)\n"
            "  · Keep holding BOOT until you see \"Chip is ...\" or the progress bar\n\n"
            "Release BOOT after the progress bar appears.\n\n"
            "Holding BOOT now?\n"
            "Click Yes to start flashing, No to cancel.",
        "sel_files": "Select firmware files",
        "all_files": "All files",
        "input_offset": "Enter offset",
        "offset_prompt": "Enter offset for {file} (hex or decimal):",
        "auto_ip": "[Auto] P4 IP = {ip}",
        "ping_start": "[Ping] {ip} ...",
        "serial_avail": "[Serial] available ports: {ports}",
        "http_req": "[HTTP] {msg}",
        "tcp_tx": "[TCP] → {ip}:{port}  {msg}",
        "tcp_rx": "[TCP] ← {msg}",
        "tcp_fail": "[TCP] failed: {e}",
    },
}

# 芯片预设（烧录）
CHIP_PRESETS = {
    "esp32c5": {"chip": "esp32c5", "flash_size": "4MB", "flash_mode": "dio", "flash_freq": "80m"},
    "esp32p4": {"chip": "esp32p4", "flash_size": "16MB", "flash_mode": "dio", "flash_freq": "80m"},
}


def load_config():
    cfg = {
        "language": "zh",
        "ip": "10.251.210.193",
        "port": str(BAUDRATE),
        "p4_project": DEFAULT_P4_PROJECT,
        "c5_prebuilt": DEFAULT_C5_PREBUILT,
        "flash_chip": "esp32c5",
        "flash_baud": "460800",
    }
    try:
        if os.path.isfile(CONFIG_PATH):
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                cfg.update(json.load(f))
    except Exception:
        pass
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
    except Exception:
        pass


class OTAHandler(http.server.SimpleHTTPRequestHandler):
    """带下载进度回调的 HTTP 文件服务器。"""
    app = None

    def log_message(self, fmt, *args):
        if OTAHandler.app is not None:
            OTAHandler.app.on_http_request(self.path, fmt % args)
        else:
            super().log_message(fmt, *args)

    def copyfile(self, source, outputfile):
        app = OTAHandler.app
        try:
            while True:
                buf = source.read(1024 * 64)
                if not buf:
                    break
                outputfile.write(buf)
                if app is not None:
                    app.on_ota_chunk(len(buf))
        except Exception:
            pass


class P4ToolApp:
    def __init__(self, root):
        self.root = root
        self.cfg = load_config()
        self.lang = self.cfg.get("language", "zh")

        # 窗口尺寸：初始较大并设最小尺寸，避免内容被裁切
        self.root.geometry("1180x880")
        self.root.minsize(920, 720)

        self.q = queue.Queue()
        self.ser = None
        self.serial_open = False
        self.http_server = None
        self.building = False
        self.ota_running = False
        self.flashing = False

        self.ota_download_event = threading.Event()
        self.ota_serial_success_event = threading.Event()
        self.ota_current_bin_name = ""
        self.ota_total_size = 0
        self.ota_sent_bytes = 0

        # 接收区开关
        self.show_serial_rx = tk.BooleanVar(value=True)
        self.show_net_rx = tk.BooleanVar(value=True)

        self._build_ui()
        self._apply_language()
        self._refresh_ports()
        self._load_flash_defaults("esp32c5")
        self._load_wifi_from_defaults()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(80, self._poll_queue)

    # ---------- i18n ----------
    def t(self, key, **kwargs):
        s = TEXTS.get(self.lang, TEXTS["zh"]).get(key, key)
        if kwargs:
            try:
                s = s.format(**kwargs)
            except Exception:
                pass
        return s

    def _apply_language(self):
        self.root.title(self.t("title"))
        self.conn_frame.configure(text=self.t("conn_settings"))
        self.lbl_serial.configure(text=self.t("serial"))
        self.lbl_baud.configure(text=self.t("baud"))
        self.lbl_lang.configure(text=self.t("language"))
        self.lbl_ip.configure(text=self.t("p4_ip"))
        self.ping_btn.configure(text=self.t("ping"))
        self.serial_btn.configure(
            text=self.t("disconnect_serial") if self.serial_open else self.t("connect_serial"))
        self.refresh_btn.configure(text=self.t("refresh"))
        self.lbl_wifi_ssid.configure(text=self.t("wifi_ssid"))
        self.lbl_wifi_password.configure(text=self.t("wifi_password"))
        self.wifi_save_btn.configure(text=self.t("wifi_save"))
        self.rx_container.configure(text=self.t("rx_area"))
        self.serial_rx_frame.configure(text=self.t("serial_rx_area"))
        self.net_rx_frame.configure(text=self.t("net_rx_area"))
        self.tx_frame.configure(text=self.t("tx_area"))
        self.serial_rx_check.configure(text=self.t("show_serial_rx"))
        self.net_rx_check.configure(text=self.t("show_net_rx"))
        self.build_frame.configure(text=self.t("build_log_area"))
        self.build_clear_btn.configure(text=self.t("clear_logs"))
        self.nb.tab(0, text=self.t("tab_tcp"))
        self.nb.tab(1, text=self.t("tab_ota"))
        self.nb.tab(2, text=self.t("tab_flash"))

        # TCP 页
        self.lbl_msg.configure(text=self.t("tcp_msg"))
        self.tcp_send_btn.configure(text=self.t("tcp_send"))
        self.tcp_ota_btn.configure(text=self.t("tcp_send_ota_manual"))

        # OTA 页
        self.lbl_fw.configure(text=self.t("ota_fw"))
        self.ota_browse_btn.configure(text=self.t("ota_browse"))
        self.build_btn.configure(text=self.t("ota_build"))
        self.ota_btn.configure(text=self.t("ota_upgrade"))
        self.ota_hint_lbl.configure(text=self.t("ota_hint"))
        self.lbl_progress.configure(text=self.t("ota_progress"))
        self.lbl_build_progress.configure(text=self.t("build_progress"))

        # Flash 页
        self.lbl_chip.configure(text=self.t("flash_chip"))
        self.flash_frame_files.configure(text=self.t("flash_files"))
        self.flash_add_btn.configure(text=self.t("flash_add"))
        self.flash_del_btn.configure(text=self.t("flash_del"))
        self.flash_clear_btn.configure(text=self.t("flash_clear"))
        self.flash_load_c5_btn.configure(text=self.t("flash_load_c5"))
        self.flash_load_p4_btn.configure(text=self.t("flash_load_p4"))
        self.lbl_flash_progress.configure(text=self.t("flash_progress"))
        self.flash_start_btn.configure(text=self.t("flash_start"))
        self.lbl_flash_port.configure(text=self.t("flash_port"))
        self.lbl_flash_baud.configure(text=self.t("flash_baud"))

        if self.status_var.get() in ("就绪", "Ready"):
            self.status_var.set(self.t("ready"))

    # ---------- UI ----------
    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}

        # 根容器用 grid 布局：接收区(row1)与构建日志区(row4)可伸缩，其余固定
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=3)   # 接收区（主要伸缩）
        self.root.rowconfigure(4, weight=1)   # 构建日志区（次要伸缩）

        # 顶部：连接设置
        self.conn_frame = ttk.LabelFrame(self.root)
        self.conn_frame.grid(row=0, column=0, sticky="ew", padx=6, pady=4)
        row1 = ttk.Frame(self.conn_frame)
        row1.pack(fill="x", padx=6, pady=4)

        self.lbl_serial = ttk.Label(row1)
        self.lbl_serial.pack(side="left")
        self.port_var = tk.StringVar(value="COM16")
        self.port_combo = ttk.Combobox(row1, textvariable=self.port_var, width=12)
        self.port_combo.pack(side="left", padx=(0, 6))

        self.lbl_baud = ttk.Label(row1)
        self.lbl_baud.pack(side="left")
        self.baud_var = tk.StringVar(value=self.cfg.get("port", str(BAUDRATE)))
        ttk.Combobox(row1, textvariable=self.baud_var, width=9,
                     values=["115200", "460800", "921600"]).pack(side="left", padx=(0, 6))

        self.serial_btn = ttk.Button(row1, command=self._toggle_serial)
        self.serial_btn.pack(side="left", padx=(0, 6))
        self.refresh_btn = ttk.Button(row1, command=self._refresh_ports)
        self.refresh_btn.pack(side="left", padx=(0, 12))

        self.lbl_ip = ttk.Label(row1)
        self.lbl_ip.pack(side="left")
        self.ip_var = tk.StringVar(value=self.cfg.get("ip", "10.251.210.193"))
        ttk.Entry(row1, textvariable=self.ip_var, width=16).pack(side="left", padx=(0, 6))
        self.ping_btn = ttk.Button(row1, command=self._ping)
        self.ping_btn.pack(side="left", padx=(0, 12))

        self.lbl_lang = ttk.Label(row1)
        self.lbl_lang.pack(side="left")
        self.lang_var = tk.StringVar(value="中文" if self.lang == "zh" else "English")
        lang_combo = ttk.Combobox(row1, textvariable=self.lang_var, width=8,
                                  values=["中文", "English"], state="readonly")
        lang_combo.pack(side="left", padx=(0, 12))
        lang_combo.bind("<<ComboboxSelected>>", self._on_lang_change)

        self.status_var = tk.StringVar(value="")
        ttk.Label(row1, textvariable=self.status_var, foreground="gray").pack(side="left")

        # 第二行：WiFi 设置
        row2 = ttk.Frame(self.conn_frame)
        row2.pack(fill="x", padx=6, pady=4)
        self.lbl_wifi_ssid = ttk.Label(row2)
        self.lbl_wifi_ssid.pack(side="left")
        self.wifi_ssid_var = tk.StringVar(value=self.cfg.get("wifi_ssid", ""))
        ttk.Entry(row2, textvariable=self.wifi_ssid_var, width=22).pack(side="left", padx=(0, 12))

        self.lbl_wifi_password = ttk.Label(row2)
        self.lbl_wifi_password.pack(side="left")
        self.wifi_pass_var = tk.StringVar(value=self.cfg.get("wifi_password", ""))
        ttk.Entry(row2, textvariable=self.wifi_pass_var, width=22, show="*").pack(
            side="left", padx=(0, 12))

        self.wifi_save_btn = ttk.Button(row2, command=self._save_wifi)
        self.wifi_save_btn.pack(side="left")

        # 中部：接收区（有线串口 + 无线网络，可开关）
        self.rx_container = ttk.LabelFrame(self.root)
        self.rx_container.grid(row=1, column=0, sticky="nsew", padx=6, pady=4)

        rx_toggle = ttk.Frame(self.rx_container)
        rx_toggle.pack(fill="x", padx=4, pady=2)
        self.serial_rx_check = ttk.Checkbutton(rx_toggle, variable=self.show_serial_rx)
        self.serial_rx_check.pack(side="left", padx=6)
        self.net_rx_check = ttk.Checkbutton(rx_toggle, variable=self.show_net_rx)
        self.net_rx_check.pack(side="left", padx=6)

        rx_cols = ttk.Frame(self.rx_container)
        rx_cols.pack(fill="both", expand=True, padx=4, pady=4)
        rx_cols.columnconfigure(0, weight=1)
        rx_cols.columnconfigure(1, weight=1)
        rx_cols.rowconfigure(0, weight=1)

        self.serial_rx_frame = ttk.LabelFrame(rx_cols)
        self.serial_rx_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 3))
        self.serial_rx_text = scrolledtext.ScrolledText(
            self.serial_rx_frame, wrap="word", font=("Consolas", 10), state="disabled")
        self.serial_rx_text.pack(fill="both", expand=True, padx=4, pady=4)

        self.net_rx_frame = ttk.LabelFrame(rx_cols)
        self.net_rx_frame.grid(row=0, column=1, sticky="nsew", padx=(3, 0))
        self.net_rx_text = scrolledtext.ScrolledText(
            self.net_rx_frame, wrap="word", font=("Consolas", 10), state="disabled")
        self.net_rx_text.pack(fill="both", expand=True, padx=4, pady=4)

        # 发送区
        self.tx_frame = ttk.LabelFrame(self.root)
        self.tx_frame.grid(row=2, column=0, sticky="ew", padx=6, pady=4)
        self.tx_text = scrolledtext.ScrolledText(
            self.tx_frame, wrap="word", height=5, font=("Consolas", 10), state="disabled")
        self.tx_text.pack(fill="both", expand=True, padx=4, pady=4)

        # 功能页
        self.nb = ttk.Notebook(self.root)
        self.nb.grid(row=3, column=0, sticky="ew", padx=6, pady=4)

        # --- TCP 页 ---
        tcp_frame = ttk.Frame(self.nb)
        self.nb.add(tcp_frame)
        tcp_row = ttk.Frame(tcp_frame)
        tcp_row.pack(fill="x", padx=6, pady=6)
        self.lbl_msg = ttk.Label(tcp_row)
        self.lbl_msg.pack(side="left")
        self.msg_var = tk.StringVar(value="hello p4")
        ttk.Entry(tcp_row, textvariable=self.msg_var, width=40).pack(side="left", padx=6)
        self.tcp_send_btn = ttk.Button(tcp_row, command=self._tcp_send)
        self.tcp_send_btn.pack(side="left", padx=2)
        self.tcp_ota_btn = ttk.Button(tcp_row, command=self._tcp_send_ota)
        self.tcp_ota_btn.pack(side="left", padx=2)

        # --- OTA 页 ---
        ota_frame = ttk.Frame(self.nb)
        self.nb.add(ota_frame)
        ota_row1 = ttk.Frame(ota_frame)
        ota_row1.pack(fill="x", padx=6, pady=6)
        self.lbl_fw = ttk.Label(ota_row1)
        self.lbl_fw.pack(side="left")
        self.bin_var = tk.StringVar(value=os.path.join(
            self.cfg.get("p4_project", DEFAULT_P4_PROJECT), "build", "p4_host_ota.bin"))
        ttk.Entry(ota_row1, textvariable=self.bin_var, width=52).pack(side="left", padx=6)
        self.ota_browse_btn = ttk.Button(ota_row1, command=self._browse_bin)
        self.ota_browse_btn.pack(side="left", padx=2)

        ota_row2 = ttk.Frame(ota_frame)
        ota_row2.pack(fill="x", padx=6, pady=6)
        self.build_btn = ttk.Button(ota_row2, command=self._build)
        self.build_btn.pack(side="left", padx=4)
        self.ota_btn = ttk.Button(ota_row2, command=self._ota_start)
        self.ota_btn.pack(side="left", padx=4)
        self.ota_hint_lbl = ttk.Label(ota_row2)
        self.ota_hint_lbl.pack(side="left", padx=8)

        ota_row4 = ttk.Frame(ota_frame)
        ota_row4.pack(fill="x", padx=6, pady=4)
        self.lbl_progress = ttk.Label(ota_row4)
        self.lbl_progress.pack(side="left", padx=(0, 6))
        self.ota_progress = ttk.Progressbar(ota_row4, length=400, mode="determinate")
        self.ota_progress.pack(side="left", padx=4, fill="x", expand=True)
        self.ota_progress_lbl = ttk.Label(ota_row4, text="0%")
        self.ota_progress_lbl.pack(side="left", padx=6)

        # --- Flash 页 ---
        flash_frame = ttk.Frame(self.nb)
        self.nb.add(flash_frame)

        flash_row1 = ttk.Frame(flash_frame)
        flash_row1.pack(fill="x", padx=6, pady=4)
        self.lbl_chip = ttk.Label(flash_row1)
        self.lbl_chip.pack(side="left")
        self.flash_chip_var = tk.StringVar(value=self.cfg.get("flash_chip", "esp32c5"))
        self.flash_chip_combo = ttk.Combobox(
            flash_row1, textvariable=self.flash_chip_var, width=12, state="readonly",
            values=list(CHIP_PRESETS.keys()))
        self.flash_chip_combo.pack(side="left", padx=(0, 12))
        self.flash_chip_combo.bind("<<ComboboxSelected>>", self._on_chip_change)

        self.lbl_flash_port = ttk.Label(flash_row1)
        self.lbl_flash_port.pack(side="left")
        self.flash_port_var = tk.StringVar(value="COM16")
        self.flash_port_combo = ttk.Combobox(flash_row1, textvariable=self.flash_port_var, width=10,
                                             values=[])
        self.flash_port_combo.pack(side="left", padx=(0, 12))

        self.lbl_flash_baud = ttk.Label(flash_row1)
        self.lbl_flash_baud.pack(side="left")
        self.flash_baud_var = tk.StringVar(value=self.cfg.get("flash_baud", "460800"))
        ttk.Combobox(flash_row1, textvariable=self.flash_baud_var, width=9,
                     values=["460800", "921600", "115200", "230400"]).pack(side="left")

        self.flash_frame_files = ttk.LabelFrame(flash_frame)
        self.flash_frame_files.pack(fill="both", expand=True, padx=6, pady=4)
        cols = ("offset", "path")
        self.flash_tree = ttk.Treeview(self.flash_frame_files, columns=cols,
                                       show="headings", height=4)
        self.flash_tree.heading("offset", text="Offset")
        self.flash_tree.heading("path", text="File")
        self.flash_tree.column("offset", width=100, anchor="center")
        self.flash_tree.column("path", width=560, anchor="w")
        self.flash_tree.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        sb = ttk.Scrollbar(self.flash_frame_files, orient="vertical",
                           command=self.flash_tree.yview)
        sb.pack(side="right", fill="y")
        self.flash_tree.configure(yscrollcommand=sb.set)

        flash_btns = ttk.Frame(flash_frame)
        flash_btns.pack(fill="x", padx=6, pady=4)
        self.flash_add_btn = ttk.Button(flash_btns, command=self._flash_add_file)
        self.flash_add_btn.pack(side="left", padx=2)
        self.flash_del_btn = ttk.Button(flash_btns, command=self._flash_del_file)
        self.flash_del_btn.pack(side="left", padx=2)
        self.flash_clear_btn = ttk.Button(flash_btns, command=self._flash_clear_files)
        self.flash_clear_btn.pack(side="left", padx=2)
        self.flash_load_c5_btn = ttk.Button(flash_btns, command=lambda: self._load_flash_defaults("esp32c5"))
        self.flash_load_c5_btn.pack(side="left", padx=8)
        self.flash_load_p4_btn = ttk.Button(flash_btns, command=lambda: self._load_flash_defaults("esp32p4"))
        self.flash_load_p4_btn.pack(side="left", padx=2)
        self.flash_start_btn = ttk.Button(flash_btns, command=self._flash_start)
        self.flash_start_btn.pack(side="left", padx=12)

        # 烧录进度条
        flash_progress_row = ttk.Frame(flash_frame)
        flash_progress_row.pack(fill="x", padx=6, pady=4)
        self.lbl_flash_progress = ttk.Label(flash_progress_row)
        self.lbl_flash_progress.pack(side="left", padx=(0, 6))
        self.flash_progress = ttk.Progressbar(flash_progress_row, length=400, mode="determinate")
        self.flash_progress.pack(side="left", padx=4, fill="x", expand=True)
        self.flash_progress_lbl = ttk.Label(flash_progress_row, text="0%")
        self.flash_progress_lbl.pack(side="left", padx=6)

        # 最底部：构建日志区 + 构建进度条
        self.build_frame = ttk.LabelFrame(self.root)
        self.build_frame.grid(row=4, column=0, sticky="nsew", padx=6, pady=4)
        build_toggle = ttk.Frame(self.build_frame)
        build_toggle.pack(fill="x", padx=4, pady=2)
        self.lbl_build_progress = ttk.Label(build_toggle)
        self.lbl_build_progress.pack(side="left", padx=4)
        self.build_progress = ttk.Progressbar(build_toggle, length=300, mode="determinate")
        self.build_progress.pack(side="left", padx=4)
        self.build_progress_lbl = ttk.Label(build_toggle, text="0%")
        self.build_progress_lbl.pack(side="left", padx=4)
        self.build_clear_btn = ttk.Button(build_toggle, command=self._clear_build_log)
        self.build_clear_btn.pack(side="right", padx=4)

        self.build_log_text = scrolledtext.ScrolledText(
            self.build_frame, wrap="word", height=8, font=("Consolas", 10), state="disabled")
        self.build_log_text.pack(fill="both", expand=True, padx=4, pady=4)

    # ---------- 日志 ----------
    def _log_rx(self, text):
        """无线网络接收区日志（TCP/OTA/ping/flash 反馈等）。"""
        self.q.put(("rx", text))

    def _log_serial_rx(self, text):
        """有线串口接收区日志。"""
        self.q.put(("serial_rx", text))

    def _log_tx(self, text):
        self.q.put(("tx", text))

    def _log_build(self, text):
        """构建日志区。"""
        self.q.put(("build", text))

    def _clear_build_log(self):
        self.build_log_text.configure(state="normal")
        self.build_log_text.delete("1.0", "end")
        self.build_log_text.configure(state="disabled")

    def _append_text(self, widget, text):
        widget.configure(state="normal")
        widget.insert("end", text)
        widget.see("end")
        widget.configure(state="disabled")

    def _poll_queue(self):
        try:
            while True:
                item = self.q.get_nowait()
                kind = item[0]
                if kind == "rx":
                    if self.show_net_rx.get():
                        self._append_text(self.net_rx_text, item[1])
                elif kind == "tx":
                    self._append_text(self.tx_text, item[1])
                elif kind == "serial":
                    data = item[1]
                    text = data.decode("utf-8", errors="replace")
                    if self.show_serial_rx.get():
                        self._append_text(self.serial_rx_text, text)
                    self._extract_ip(data)
                    self._check_ota_markers(text)
                elif kind == "serial_rx":
                    if self.show_serial_rx.get():
                        self._append_text(self.serial_rx_text, item[1])
                elif kind == "build":
                    self._append_text(self.build_log_text, item[1])
                elif kind == "status":
                    self.status_var.set(item[1])
                elif kind == "progress":
                    val = max(0.0, min(100.0, item[1]))
                    self.ota_progress["value"] = val
                    self.ota_progress_lbl.configure(text=f"{val:.0f}%")
                elif kind == "build_progress":
                    val = max(0.0, min(100.0, item[1]))
                    self.build_progress["value"] = val
                    self.build_progress_lbl.configure(text=f"{val:.0f}%")
                elif kind == "flash_progress":
                    val = max(0.0, min(100.0, item[1]))
                    self.flash_progress["value"] = val
                    self.flash_progress_lbl.configure(text=f"{val:.0f}%")
                elif kind == "ui":
                    # 主线程安全执行后台线程请求的 UI 回调
                    try:
                        item[1]()
                    except Exception as e:
                        self._append_text(self.net_rx_text, f"[UI error] {e}\n")
        except queue.Empty:
            pass
        self.root.after(80, self._poll_queue)

    def _ui_safe(self, callback):
        """后台线程用它把 UI 操作安全地派发到主线程。"""
        self.q.put(("ui", callback))

    def _extract_ip(self, data: bytes):
        text = data.decode("utf-8", errors="replace")
        m = IP_RE.search(text)
        if m:
            ip = m.group(1)
            if ip != self.ip_var.get():
                self.ip_var.set(ip)
                self._log_serial_rx(self.t("auto_ip", ip=ip) + "\n")

    def _check_ota_markers(self, text):
        if "OTA succeeded" in text or "OTA finished" in text or "rebooting" in text:
            if not self.ota_serial_success_event.is_set():
                self.ota_serial_success_event.set()
                self._log_serial_rx(self.t("ota_serial_ok") + "\n")

    # ---------- 语言切换 ----------
    def _on_lang_change(self, event=None):
        self.lang = "zh" if self.lang_var.get() == "中文" else "en"
        self.cfg["language"] = self.lang
        save_config(self.cfg)
        self._apply_language()

    # ---------- 串口 ----------
    def _refresh_ports(self):
        ports = []
        if HAS_SERIAL:
            ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        self.flash_port_combo["values"] = ports
        if ports:
            self.port_var.set(ports[0])
            self.flash_port_var.set(ports[0])
            self._log_serial_rx(self.t("serial_avail", ports=", ".join(ports)) + "\n")

    def _toggle_serial(self):
        if self.serial_open:
            self._close_serial()
        else:
            self._open_serial()

    def _open_serial(self):
        if not HAS_SERIAL:
            messagebox.showerror(self.t("serial_fail"), self.t("serial_no_pyserial"))
            return
        port = self.port_var.get().strip()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = BAUDRATE
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            messagebox.showerror(self.t("serial_fail"), f"{port}\n{e}")
            return
        self.serial_open = True
        self.serial_btn.configure(text=self.t("disconnect_serial"))
        self._log_serial_rx(self.t("serial_connected", port=port, baud=baud) + "\n")
        threading.Thread(target=self._serial_reader, daemon=True).start()

    def _close_serial(self):
        self.serial_open = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.serial_btn.configure(text=self.t("connect_serial"))
        self._log_serial_rx(self.t("serial_disconnected") + "\n")

    def _serial_reader(self):
        while self.serial_open and self.ser:
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n if n else 1)
                if data:
                    self.q.put(("serial", data))
            except Exception as e:
                self._log_serial_rx(f"\n[串口读取错误] {e}\n")
                break

    # ---------- Ping ----------
    def _ping(self):
        ip = self.ip_var.get().strip()
        if not ip:
            return
        self._log_tx(self.t("ping_start", ip=ip) + "\n")

        def run():
            try:
                proc = subprocess.run(
                    ["ping", "-n", "2", ip], capture_output=True, text=True, timeout=15,
                    creationflags=CREATE_NO_WINDOW)
                out = proc.stdout
                lines = [l for l in out.splitlines()
                         if "来自" in l or "Reply" in l or "丢失" in l or "Lost" in l
                         or "找不到" in l or "could not" in l]
                self._log_rx("\n".join(lines) + "\n" if lines else out)
                if proc.returncode == 0:
                    self._log_rx(f"[Ping] {ip} {self.t('ping_ok')}\n")
                else:
                    self._log_rx(f"[Ping] {ip} {self.t('ping_fail')}\n")
            except Exception as e:
                self._log_rx(f"[Ping] {e}\n")

        threading.Thread(target=run, daemon=True).start()

    def _ping_once(self, ip):
        try:
            r = subprocess.run(["ping", "-n", "1", "-w", "1000", ip],
                               capture_output=True, timeout=5,
                               creationflags=CREATE_NO_WINDOW)
            return r.returncode == 0
        except Exception:
            return False

    # ---------- TCP ----------
    def _tcp_send(self):
        self._tcp_send_raw(self.msg_var.get())

    def _tcp_send_ota(self):
        url = f"http://{self._local_ip()}:{HTTP_PORT}/{os.path.basename(self.bin_var.get())}"
        self._tcp_send_raw(f"ota {url}")

    def _tcp_send_raw(self, message):
        ip = self.ip_var.get().strip()
        if not ip or not message:
            return
        self._log_tx(self.t("tcp_tx", ip=ip, port=TCP_PORT, msg=message) + "\n")

        def run():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5)
                s.connect((ip, TCP_PORT))
                s.sendall(message.encode("utf-8"))
                try:
                    data = s.recv(4096)
                    if data:
                        self._log_rx(self.t("tcp_rx", msg=data.decode("utf-8", errors="replace")) + "\n")
                except socket.timeout:
                    self._log_rx("[TCP] ← (no reply)\n")
                s.close()
            except Exception as e:
                self._log_rx(self.t("tcp_fail", e=e) + "\n")

        threading.Thread(target=run, daemon=True).start()

    def _local_ip(self):
        ip = self.ip_var.get().strip() or "8.8.8.8"
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((ip, 80))
            return s.getsockname()[0]
        finally:
            s.close()

    # ---------- WiFi 配置 ----------
    def _sdkconfig_defaults_path(self):
        return os.path.join(self.cfg.get("p4_project", DEFAULT_P4_PROJECT), "sdkconfig.defaults")

    def _load_wifi_from_defaults(self):
        path = self._sdkconfig_defaults_path()
        ssid = ""
        password = ""
        try:
            if os.path.isfile(path):
                with open(path, "r", encoding="utf-8", errors="replace") as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith("CONFIG_ESP_WIFI_SSID="):
                            ssid = line.split("=", 1)[1].strip().strip('"')
                        elif line.startswith("CONFIG_ESP_WIFI_PASSWORD="):
                            password = line.split("=", 1)[1].strip().strip('"')
        except Exception:
            pass
        if ssid:
            self.wifi_ssid_var.set(ssid)
            self.cfg["wifi_ssid"] = ssid
        if password:
            self.wifi_pass_var.set(password)
            self.cfg["wifi_password"] = password

    def _save_wifi(self):
        ssid = self.wifi_ssid_var.get().strip()
        password = self.wifi_pass_var.get()
        if not ssid:
            messagebox.showwarning("WiFi", self.t("wifi_ssid"))
            return False
        # 强制英文 WiFi 名：SSID 必须为 ASCII 可打印字符
        try:
            ssid.encode("ascii")
        except UnicodeEncodeError:
            messagebox.showerror(
                self.t("wifi_ssid_ascii_title"),
                self.t("wifi_ssid_ascii_msg", ssid=ssid))
            return False
        path = self._sdkconfig_defaults_path()
        if not os.path.isfile(path):
            messagebox.showerror("WiFi", f"sdkconfig.defaults not found:\n{path}")
            return False

        try:
            with open(path, "r", encoding="utf-8") as f:
                lines = f.readlines()
            out = []
            has_ssid = False
            has_pass = False
            for line in lines:
                stripped = line.strip()
                if stripped.startswith("CONFIG_ESP_WIFI_SSID="):
                    out.append(f"CONFIG_ESP_WIFI_SSID={json.dumps(ssid, ensure_ascii=False)}\n")
                    has_ssid = True
                elif stripped.startswith("CONFIG_ESP_WIFI_PASSWORD="):
                    out.append(f"CONFIG_ESP_WIFI_PASSWORD={json.dumps(password, ensure_ascii=False)}\n")
                    has_pass = True
                else:
                    out.append(line)
            if not has_ssid:
                out.append(f"CONFIG_ESP_WIFI_SSID={json.dumps(ssid, ensure_ascii=False)}\n")
            if not has_pass:
                out.append(f"CONFIG_ESP_WIFI_PASSWORD={json.dumps(password, ensure_ascii=False)}\n")
            with open(path, "w", encoding="utf-8") as f:
                f.writelines(out)
        except Exception as e:
            messagebox.showerror("WiFi", f"{e}")
            return False

        self.cfg["wifi_ssid"] = ssid
        self.cfg["wifi_password"] = password
        save_config(self.cfg)
        self._log_rx(self.t("wifi_saved", ssid=ssid) + "\n")
        return True

    # ---------- 构建 ----------
    def _find_app_bin(self, project_dir):
        """在 build 目录找 app 固件（排除 bootloader/partition/ota_data）。"""
        build = os.path.join(project_dir, "build")
        if not os.path.isdir(build):
            return None
        cands = []
        for f in os.listdir(build):
            if f.endswith(".bin") and f not in (
                    "bootloader.bin", "partition-table.bin", "ota_data_initial.bin"):
                cands.append(os.path.join(build, f))
        if cands:
            return max(cands, key=os.path.getsize)
        return None

    def _build(self):
        if self.building:
            return
        self.building = True
        self.build_btn.configure(state="disabled")
        self._log_build(self.t("build_start") + "\n")
        self.build_progress["value"] = 0
        self.build_progress_lbl.configure(text="0%")

        # 构建前保存 WiFi 配置到 sdkconfig.defaults（校验失败则中止构建）
        if not self._save_wifi():
            self._log_build("⚠️ 构建中止：WiFi 配置无效\n")
            self.building = False
            self.build_btn.configure(state="normal")
            return

        project_dir = self.cfg.get("p4_project", DEFAULT_P4_PROJECT)
        export_bat = os.path.join(IDF_PATH, "export.bat")
        cmd = f'call "{export_bat}" >nul 2>&1 && idf.py build'

        os.makedirs(IDF_TMP_DIR, exist_ok=True)
        build_env = os.environ.copy()
        build_env["TMP"] = IDF_TMP_DIR
        build_env["TEMP"] = IDF_TMP_DIR
        build_env["IDF_PYTHON_ENV_PATH"] = IDF_PYTHON_ENV_PATH

        # ninja 进度格式: [N/TOTAL] Building/Linking ...
        ninja_re = re.compile(r"\[(\d+)/(\d+)\]")

        def run():
            try:
                proc = subprocess.Popen(
                    cmd, shell=True, cwd=project_dir, env=build_env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding="utf-8", errors="replace",
                    creationflags=CREATE_NO_WINDOW)
                for line in proc.stdout:
                    line = strip_ansi(line.rstrip("\n"))
                    if line.strip():
                        self._log_build(line + "\n")
                        m = ninja_re.search(line)
                        if m:
                            cur = int(m.group(1))
                            total = int(m.group(2))
                            pct = (cur / total) * 100.0 if total > 0 else 0.0
                            self.q.put(("build_progress", pct))
                proc.wait()
                if proc.returncode == 0:
                    self._log_build(self.t("build_success") + "\n")
                    self.q.put(("build_progress", 100.0))
                    # 自动定位新固件
                    bin_path = self._find_app_bin(project_dir)
                    if bin_path:
                        def _set_bin():
                            self.bin_var.set(bin_path)
                        self._ui_safe(_set_bin)
                        self._log_build(f"[构建] app bin: {bin_path}\n")
                else:
                    self._log_build(self.t("build_fail", code=proc.returncode) + "\n")
                    self.q.put(("build_progress", 0.0))
            except Exception as e:
                self._log_build(f"[构建] {e}\n")
            finally:
                self.building = False
                self._ui_safe(lambda: self.build_btn.configure(state="normal"))

        threading.Thread(target=run, daemon=True).start()

    # ---------- OTA ----------
    def _browse_bin(self):
        initial = os.path.join(self.cfg.get("p4_project", DEFAULT_P4_PROJECT), "build")
        path = filedialog.askopenfilename(
            initialdir=initial, filetypes=[("*.bin", "*.bin"), (self.t("all_files"), "*.*")])
        if path:
            self.bin_var.set(path)

    def on_http_request(self, path, msg):
        self._log_rx(self.t("http_req", msg=msg) + "\n")
        bin_name = self.ota_current_bin_name
        if bin_name and " 200 " in msg and (path == "/" + bin_name or path.endswith("/" + bin_name)):
            if not self.ota_download_event.is_set():
                self.ota_download_event.set()
                self._log_rx(self.t("ota_download_ok") + "\n")

    def on_ota_chunk(self, n):
        if self.ota_total_size <= 0:
            return
        self.ota_sent_bytes += n
        pct = (self.ota_sent_bytes / self.ota_total_size) * 100.0
        self.q.put(("progress", pct))

    def _ota_start(self):
        if self.ota_running:
            return
        bin_path = self.bin_var.get().strip()
        if not os.path.isfile(bin_path):
            messagebox.showerror("Error", f"Firmware not found:\n{bin_path}\n\nPlease build first.")
            return
        ip = self.ip_var.get().strip()
        if not ip:
            return

        self.ota_running = True
        self.ota_btn.configure(state="disabled")
        self._log_rx(self.t("ota_start") + f" {bin_path}\n")

        serve_dir = os.path.dirname(bin_path)
        bin_name = os.path.basename(bin_path)
        local_ip = self._local_ip()
        url = f"http://{local_ip}:{HTTP_PORT}/{bin_name}"
        self._log_rx(self.t("ota_url") + f" {url}\n")

        self.ota_current_bin_name = bin_name
        self.ota_download_event.clear()
        self.ota_serial_success_event.clear()
        self.ota_total_size = os.path.getsize(bin_path)
        self.ota_sent_bytes = 0
        self.ota_progress["value"] = 0
        self.ota_progress_lbl.configure(text="0%")

        def run():
            # 1) HTTP 服务器
            try:
                OTAHandler.app = self
                handler = functools.partial(OTAHandler, directory=serve_dir)
                self.http_server = socketserver.TCPServer(("0.0.0.0", HTTP_PORT), handler)
                threading.Thread(target=self.http_server.serve_forever, daemon=True).start()
                self._log_rx(self.t("ota_http_up", port=HTTP_PORT) + "\n")
            except Exception as e:
                self._log_rx(f"[OTA] HTTP server fail: {e}\n")
                self._finish_ota()
                return

            # 2) TCP 触发（带重试：P4 单连接 + backlog=1，偶发拒绝时自动重试）
            triggered = False
            last_err = None
            for attempt in range(3):
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(5)
                    s.connect((ip, TCP_PORT))
                    s.sendall(f"ota {url}\n".encode("utf-8"))
                    try:
                        reply = s.recv(4096).decode("utf-8", errors="replace")
                        self._log_rx(f"[OTA] P4: {reply}\n")
                    except socket.timeout:
                        self._log_rx("[OTA] P4 no reply (download may start)\n")
                    s.close()
                    triggered = True
                    break
                except Exception as e:
                    last_err = e
                    self._log_rx(f"[OTA] 触发第 {attempt + 1} 次失败: {e}，重试...\n")
                    time.sleep(2)

            if not triggered:
                self._log_rx(self.t("ota_trigger_fail") + f" ({last_err})\n")
                self._stop_http_server()
                self._ui_safe(lambda: messagebox.showerror(
                    "OTA", self.t("msgbox_ota_trigger", ip=ip)))
                self._finish_ota()
                return

            self._log_tx(f"[OTA] → {url}\n")
            self._log_rx(self.t("ota_triggered") + "\n")

            # 3) 等待下载
            download_ok = self.ota_download_event.wait(OTA_DOWNLOAD_TIMEOUT)
            if not download_ok:
                self._log_rx(self.t("ota_download_fail") + "\n")

            # 4) 等待重启
            success = False
            if download_ok:
                deadline = time.time() + OTA_REBOOT_TIMEOUT
                saw_down = False
                while time.time() < deadline:
                    if self.ota_serial_success_event.is_set():
                        success = True
                        break
                    up = self._ping_once(ip)
                    if not up:
                        saw_down = True
                    elif saw_down:
                        success = True
                        break
                    time.sleep(2)

            # 5) 结论
            self._stop_http_server()
            self.q.put(("progress", 100.0 if download_ok else 0.0))

            if success:
                self._log_rx(self.t("ota_success") + "\n")
                self._ui_safe(lambda: messagebox.showinfo("OTA", self.t("msgbox_ota_success")))
            elif download_ok:
                self._log_rx(self.t("ota_dl_no_reboot") + "\n")
                self._ui_safe(lambda: messagebox.showwarning("OTA", self.t("msgbox_ota_warn")))
            else:
                self._log_rx(self.t("ota_fail") + "\n")
                self._ui_safe(lambda: messagebox.showerror("OTA", self.t("msgbox_ota_error")))

            self._finish_ota()

        threading.Thread(target=run, daemon=True).start()

    def _stop_http_server(self):
        if self.http_server:
            try:
                self.http_server.shutdown()
                self.http_server.server_close()
            except Exception:
                pass
            self.http_server = None

    def _finish_ota(self):
        self.ota_running = False
        self._ui_safe(lambda: self.ota_btn.configure(state="normal"))

    # ---------- 烧录 ----------
    def _on_chip_change(self, event=None):
        self._load_flash_defaults(self.flash_chip_var.get())

    def _load_flash_defaults(self, chip):
        self.flash_chip_var.set(chip)
        self._flash_clear_files()
        if chip == "esp32c5":
            prebuilt = self.cfg.get("c5_prebuilt", DEFAULT_C5_PREBUILT)
            files = [
                ("0x2000", os.path.join(prebuilt, "bootloader.bin")),
                ("0x8000", os.path.join(prebuilt, "partition-table.bin")),
                ("0xd000", os.path.join(prebuilt, "ota_data_initial.bin")),
                ("0x10000", os.path.join(prebuilt, "slave.bin")),
            ]
            for off, path in files:
                if os.path.isfile(path):
                    self.flash_tree.insert("", "end", values=(off, path))
        elif chip == "esp32p4":
            proj = self.cfg.get("p4_project", DEFAULT_P4_PROJECT)
            build = os.path.join(proj, "build")
            files = [
                ("0x2000", os.path.join(build, "bootloader", "bootloader.bin")),
                ("0x8000", os.path.join(build, "partition_table", "partition-table.bin")),
                ("0xd000", os.path.join(build, "ota_data_initial.bin")),
            ]
            app = self._find_app_bin(proj)
            if app:
                files.append(("0x10000", app))
            for off, path in files:
                if os.path.isfile(path):
                    self.flash_tree.insert("", "end", values=(off, path))

    def _flash_add_file(self):
        paths = filedialog.askopenfilenames(
            title=self.t("sel_files"),
            filetypes=[("*.bin", "*.bin"), ("*.hex", "*.hex"), (self.t("all_files"), "*.*")])
        for path in paths:
            off = tk.simpledialog.askstring(
                self.t("input_offset"), self.t("offset_prompt", file=os.path.basename(path)),
                initialvalue="0x10000")
            if off is not None:
                self.flash_tree.insert("", "end", values=(off.strip(), path))

    def _flash_del_file(self):
        sel = self.flash_tree.selection()
        for item in sel:
            self.flash_tree.delete(item)

    def _flash_clear_files(self):
        for item in self.flash_tree.get_children():
            self.flash_tree.delete(item)

    def _flash_start(self):
        if self.flashing:
            return
        items = self.flash_tree.get_children()
        if not items:
            messagebox.showwarning("Flash", self.t("flash_no_files"))
            return
        port = self.flash_port_var.get().strip()
        if not port:
            messagebox.showwarning("Flash", self.t("flash_no_port"))
            return
        if not os.path.isfile(ESPTOOL):
            messagebox.showerror("Flash", self.t("flash_no_esptool"))
            return

        # 烧录前自动断开串口监视，否则 esptool 无法打开串口（COM busy）
        if self.serial_open:
            self._log_serial_rx("[烧录] 自动断开串口监视以释放串口\n")
            self._close_serial()

        chip = self.flash_chip_var.get()
        preset = CHIP_PRESETS.get(chip, {"chip": chip, "flash_size": "4MB",
                                         "flash_mode": "dio", "flash_freq": "80m"})
        baud = self.flash_baud_var.get().strip() or "460800"

        args = [
            ESPTOOL, "--chip", preset["chip"], "-p", port, "-b", baud,
            "--before", "default_reset", "--after", "hard_reset",
            "--connect-attempts", "50",
            "write_flash",
            "--flash_mode", preset["flash_mode"],
            "--flash_freq", preset["flash_freq"],
            "--flash_size", preset["flash_size"],
        ]
        for item in items:
            vals = self.flash_tree.item(item, "values")
            args.append(vals[0])
            args.append(vals[1])

        self.flashing = True
        self.flash_start_btn.configure(state="disabled")
        self._log_tx(f"[FLASH] {' '.join(args)}\n")
        self.flash_progress["value"] = 0
        self.flash_progress_lbl.configure(text="0%")

        # 烧录前引导：仅 C5 需要手动按住 BOOT 进入下载模式
        # （P4 会自动复位进下载模式，无需按键，直接烧录）
        if preset["chip"] == "esp32c5":
            if not messagebox.askyesno(
                    self.t("flash_manual_boot_title"),
                    self.t("flash_manual_boot_msg", chip=preset["chip"], port=port)):
                self._log_serial_rx("⚠️ 用户取消烧录\n")
                self.flashing = False
                self.flash_start_btn.configure(state="normal")
                return

        self._log_serial_rx(self.t("flash_starting", chip=preset["chip"], port=port) + "\n")

        # esptool 写入进度格式: "Writing at 0x00002000... (100 %)" 或 "(3 %)"
        write_re = re.compile(r"Writing at .+?\((\d+) %\)")

        def run():
            try:
                proc = subprocess.Popen(
                    args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding="utf-8", errors="replace",
                    creationflags=CREATE_NO_WINDOW)
                for line in proc.stdout:
                    line = strip_ansi(line.rstrip("\n"))
                    if line.strip():
                        self._log_serial_rx(line + "\n")
                        m = write_re.search(line)
                        if m:
                            pct = int(m.group(1))
                            self.q.put(("flash_progress", float(pct)))
                proc.wait()
                if proc.returncode == 0:
                    self.q.put(("flash_progress", 100.0))
                    self._log_serial_rx(self.t("flash_ok") + "\n")
                    self._ui_safe(lambda: messagebox.showinfo("Flash", self.t("flash_ok")))
                else:
                    self._log_serial_rx(self.t("flash_fail", code=proc.returncode) + "\n")
                    self._ui_safe(lambda: messagebox.showerror(
                        "Flash", self.t("flash_fail", code=proc.returncode)))
            except Exception as e:
                self._log_serial_rx(f"[FLASH] {e}\n")
            finally:
                self.flashing = False
                self._ui_safe(lambda: self.flash_start_btn.configure(state="normal"))

        threading.Thread(target=run, daemon=True).start()

    # ---------- 关闭 ----------
    def _on_close(self):
        self.cfg["ip"] = self.ip_var.get()
        self.cfg["port"] = self.baud_var.get()
        self.cfg["flash_chip"] = self.flash_chip_var.get()
        self.cfg["flash_baud"] = self.flash_baud_var.get()
        save_config(self.cfg)
        self._close_serial()
        self._stop_http_server()
        self.root.destroy()


def main():
    root = tk.Tk()
    P4ToolApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()

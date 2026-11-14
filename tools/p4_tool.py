# -*- coding: utf-8 -*-
"""
ESP32-P4C5 无线调试工具（图形界面）v2

功能：
  1. 双通道：有线串口 / 无线 TCP，各自独立的接收区与发送区，可单独开关
  2. 串口自动识别（按 VID/PID 找 ESP32），不再固定 COM16
  3. P4 IP 自动识别（串口日志 / TCP 探测 / 局域网扫描）
  4. 无线下载：TCP 推送（推荐，不怕防火墙）与 HTTP 下载两种方式
  5. 构建 / 烧录（C5、P4 预设固定偏移，路径自动定位）
  6. 所有路径与选项自动记忆，下次打开保持上次的选择
  7. 一键流程：保存 WiFi -> 构建 -> 等 IP -> 无线升级

运行：
  双击 启动P4工具.bat
  或 python p4_tool.py
"""
import os
import re
import sys
import json
import time
import queue
import socket
import functools
import subprocess
import threading
import http.server
import socketserver
from concurrent.futures import ThreadPoolExecutor, as_completed

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext, simpledialog

CREATE_NO_WINDOW = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    serial = None

# ---------------- 路径与常量 ----------------
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(TOOLS_DIR, "tool_config.json")

IDF_PATH = os.environ.get("IDF_PATH", r"D:\ESP32-IDF\Espressif\frameworks\esp-idf-v5.5.5")
IDF_TOOLS_PATH = os.environ.get("IDF_TOOLS_PATH", r"D:\ESP32-IDF\Espressif")
IDF_PYTHON_ENV_PATH = os.environ.get(
    "IDF_PYTHON_ENV_PATH",
    os.path.join(IDF_TOOLS_PATH, "python_env", "idf5.5_py3.11_env"))
IDF_TMP_DIR = os.path.join(IDF_TOOLS_PATH, "esp_tmp")
ESPTOOL = os.path.join(IDF_PYTHON_ENV_PATH, "Scripts", "esptool.exe")

TCP_PORT = 3333
HTTP_PORT = 8000
BAUDRATE = 115200

# ESP32 常见 USB 转串口 VID（用于自动识别板子所在的串口）
ESP_VIDS = {
    0x303A: "Espressif USB",
    0x10C4: "CP210x",
    0x1A86: "CH34x",
    0x0403: "FTDI",
}

CHIP_PRESETS = {
    "esp32c5": {"chip": "esp32c5", "flash_size": "4MB", "flash_mode": "dio", "flash_freq": "80m"},
    "esp32p4": {"chip": "esp32p4", "flash_size": "16MB", "flash_mode": "dio", "flash_freq": "80m"},
}

IP_PATTERNS = [
    re.compile(r"got ip[:\s]*([0-9]{1,3}(?:\.[0-9]{1,3}){3})", re.I),
    re.compile(r"\bIP\s*[=:]\s*([0-9]{1,3}(?:\.[0-9]{1,3}){3})"),
    re.compile(r"ip_info\.ip\s*[=:]\s*([0-9]{1,3}(?:\.[0-9]{1,3}){3})"),
]
ANSI_RE = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")


def strip_ansi(s):
    return ANSI_RE.sub("", s)


# ---------------- 路径探测 ----------------
def _walk_up_find(start, predicate, max_depth=5):
    d = os.path.abspath(start)
    for _ in range(max_depth):
        try:
            if predicate(d):
                return d
        except Exception:
            pass
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


def looks_like_idf_project(path):
    return (os.path.isfile(os.path.join(path, "CMakeLists.txt"))
            and os.path.isfile(os.path.join(path, "sdkconfig.defaults")))


def find_default_project():
    """自动定位本机上的 ESP-IDF 工程根目录。"""
    # 1) 工具目录本身 / 上层目录
    found = _walk_up_find(TOOLS_DIR, looks_like_idf_project, 6)
    if found:
        return found
    # 2) 常见工作目录下扫描（sdkconfig.defaults + main/）
    for base in (r"D:\ESP32-IDF\code", os.path.expanduser("~")):
        if not os.path.isdir(base):
            continue
        cands = []
        try:
            for name in os.listdir(base):
                p = os.path.join(base, name)
                if os.path.isdir(p) and looks_like_idf_project(p):
                    cands.append(p)
        except Exception:
            continue
        if cands:
            cands.sort(key=lambda p: os.path.getmtime(
                os.path.join(p, "sdkconfig.defaults")), reverse=True)
            return cands[0]
    return os.path.dirname(TOOLS_DIR)


def find_c5_prebuilt(project_dir):
    """C5 预编译固件目录（c5_firmware/prebuilt）。"""
    for start in (TOOLS_DIR, project_dir):
        p = _walk_up_find(
            start,
            lambda d: os.path.isdir(os.path.join(d, "c5_firmware", "prebuilt")),
            6)
        if p:
            return os.path.join(p, "c5_firmware", "prebuilt")
    return os.path.join(os.path.dirname(TOOLS_DIR), "c5_firmware", "prebuilt")


# ---------------- i18n ----------------
TEXTS = {
    "zh": {
        "title": "ESP32-P4C5 无线调试工具",
        "conn": "连接设置",
        "serial": "串口",
        "baud": "波特率",
        "serial_connect": "连接串口",
        "serial_disconnect": "断开串口",
        "scan": "扫描",
        "auto_detect_port": "自动识别串口",
        "p4_ip": "P4 IP",
        "auto_ip": "自动识别 IP",
        "ping": "Ping",
        "net_connect": "连接无线",
        "net_disconnect": "断开无线",
        "status": "状态",
        "wifi_ssid": "WiFi 名称",
        "wifi_password": "WiFi 密码",
        "wifi_save": "保存 WiFi 到工程",
        "project_dir": "P4 工程目录",
        "c5_dir": "C5 固件目录",
        "browse": "浏览",
        "channels": "通道（每个通道都有独立收发区，可单独关闭）",
        "ch_serial": "有线串口",
        "ch_net": "无线 TCP",
        "send": "发送",
        "newline": "换行",
        "hex": "HEX",
        "clear": "清空",
        "close": "关闭",
        "open": "打开",
        "not_connected": "未连接",
        "connected": "已连接",
        "send_hint": "输入要发送的内容…",
        "tab_tcp": "TCP 调试",
        "tab_ota": "无线下载",
        "tab_flash": "固件烧录",
        "tab_auto": "自动化",
        "ota_mode": "下载方式",
        "ota_push": "TCP 推送（推荐）",
        "ota_http": "HTTP 下载",
        "ota_fw": "固件",
        "ota_server": "HTTP 服务器地址",
        "build": "1. 构建固件",
        "upgrade": "2. 开始无线下载",
        "selfcheck": "HTTP 自检",
        "progress": "进度",
        "flash_chip": "芯片",
        "flash_port": "串口",
        "flash_baud": "烧录波特率",
        "flash_files": "烧录文件（偏移 + 文件，C5/P4 为固定预设）",
        "add": "添加文件",
        "del": "删除",
        "clear_all": "清空",
        "load_c5": "载入 C5 预设",
        "load_p4": "载入 P4 预设",
        "start_flash": "开始烧录",
        "pipeline": "一键流程：保存 WiFi → 构建 → 等 IP → 无线下载",
        "run_pipeline": "开始一键流程",
        "auto_on_start": "启动时自动识别串口与 IP",
        "auto_reconnect": "无线断线自动重连",
        "log_area": "日志（构建 / 流程）",
        "save_log": "保存日志",
        "ready": "就绪",
        "lang": "语言",
        "rx_auto_nl": "接收自动换行",
        "rx_ts": "接收区加时间戳",
    },
    "en": {
        "title": "ESP32-P4C5 Wireless Debug Tool",
        "conn": "Connection",
        "serial": "Port",
        "baud": "Baud",
        "serial_connect": "Connect serial",
        "serial_disconnect": "Disconnect serial",
        "scan": "Scan",
        "auto_detect_port": "Auto-detect port",
        "p4_ip": "P4 IP",
        "auto_ip": "Auto-detect IP",
        "ping": "Ping",
        "net_connect": "Connect TCP",
        "net_disconnect": "Disconnect TCP",
        "status": "Status",
        "wifi_ssid": "WiFi SSID",
        "wifi_password": "WiFi Password",
        "wifi_save": "Save WiFi to project",
        "project_dir": "P4 project",
        "c5_dir": "C5 firmware",
        "browse": "Browse",
        "channels": "Channels (each has its own RX/TX, closable)",
        "ch_serial": "Wired serial",
        "ch_net": "Wireless TCP",
        "send": "Send",
        "newline": "Newline",
        "hex": "HEX",
        "clear": "Clear",
        "close": "Close",
        "open": "Open",
        "not_connected": "Not connected",
        "connected": "Connected",
        "send_hint": "Text to send…",
        "tab_tcp": "TCP",
        "tab_ota": "Wireless OTA",
        "tab_flash": "Flash",
        "tab_auto": "Automation",
        "ota_mode": "Mode",
        "ota_push": "TCP push (recommended)",
        "ota_http": "HTTP download",
        "ota_fw": "Firmware",
        "ota_server": "HTTP server host",
        "build": "1. Build",
        "upgrade": "2. Start OTA",
        "selfcheck": "HTTP self-check",
        "progress": "Progress",
        "flash_chip": "Chip",
        "flash_port": "Port",
        "flash_baud": "Flash baud",
        "flash_files": "Files (offset + file; C5/P4 presets are fixed)",
        "add": "Add file",
        "del": "Delete",
        "clear_all": "Clear",
        "load_c5": "Load C5 preset",
        "load_p4": "Load P4 preset",
        "start_flash": "Start flashing",
        "pipeline": "Pipeline: save WiFi → build → wait IP → OTA",
        "run_pipeline": "Run pipeline",
        "auto_on_start": "Auto-detect port and IP on start",
        "auto_reconnect": "Auto-reconnect TCP",
        "log_area": "Log (build / pipeline)",
        "save_log": "Save log",
        "ready": "Ready",
        "lang": "Language",
        "rx_auto_nl": "RX auto newline",
        "rx_ts": "Timestamp in RX",
    },
}


# ---------------- 配置 ----------------
def load_config():
    cfg = {
        "language": "zh",
        "serial_port": "",
        "serial_baud": str(BAUDRATE),
        "flash_port": "",
        "flash_baud": "460800",
        "flash_chip": "esp32p4",
        "ip": "",
        "http_host": "",
        "http_port": str(HTTP_PORT),
        "ota_mode": "push",
        "ota_dir": "",
        "wifi_ssid": "",
        "wifi_password": "",
        "auto_on_start": True,
        "auto_reconnect": True,
    }
    try:
        if os.path.isfile(CONFIG_PATH):
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                cfg.update(json.load(f))
    except Exception:
        pass
    if not cfg.get("p4_project") or not os.path.isdir(cfg["p4_project"]):
        cfg["p4_project"] = find_default_project()
    if not cfg.get("c5_prebuilt") or not os.path.isdir(cfg["c5_prebuilt"]):
        cfg["c5_prebuilt"] = find_c5_prebuilt(cfg["p4_project"])
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
    except Exception:
        pass


def list_serial_ports():
    """返回 [(device, description, vid, pid)]"""
    out = []
    if not HAS_SERIAL:
        return out
    for p in serial.tools.list_ports.comports():
        out.append((p.device, p.description or "", p.vid or 0, p.pid or 0))
    out.sort(key=lambda x: x[0])
    return out


def guess_esp_port(ports):
    """按 VID 优先挑 ESP 相关串口，其次第一个。"""
    for dev, desc, vid, pid in ports:
        if vid in ESP_VIDS:
            return dev
    for dev, desc, vid, pid in ports:
        d = (desc or "").lower()
        if "esp" in d or "cp210" in d or "ch34" in d or "usb-serial" in d:
            return dev
    return ports[0][0] if ports else ""


# ---------------- HTTP 服务器（带进度） ----------------
class OTAHandler(http.server.SimpleHTTPRequestHandler):
    app = None

    def log_message(self, fmt, *args):
        msg = fmt % args
        if OTAHandler.app is not None:
            OTAHandler.app.on_http_request(self.path, msg)

    def copyfile(self, source, outputfile):
        app = OTAHandler.app
        try:
            while True:
                buf = source.read(64 * 1024)
                if not buf:
                    break
                outputfile.write(buf)
                if app is not None:
                    app.on_ota_chunk(len(buf))
        except Exception:
            pass


class ThreadingHTTP(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


# ---------------- 通道面板 ----------------
class ChannelPanel:
    """一个通道 = 标题栏 + 接收区 + 发送区；可整体隐藏/关闭。"""

    def __init__(self, parent, title, on_send, on_close, on_clear,
                 with_hex=False, on_connect=None, ts_provider=None,
                 rx_nl_text="接收自动换行"):
        self.frame = ttk.LabelFrame(parent, text=title)
        self.title = title
        self.on_send = on_send
        self.on_connect = on_connect
        self.ts_provider = ts_provider
        self._pend = ""
        self._pend_ts = 0.0

        head = ttk.Frame(self.frame)
        head.pack(fill="x", padx=4, pady=2)
        self.status_lbl = ttk.Label(head, text="", foreground="gray")
        self.status_lbl.pack(side="left", padx=4)
        ttk.Button(head, text="✕", width=3, command=on_close).pack(side="right", padx=2)
        self.clear_btn = ttk.Button(head, text=TEXTS["zh"]["clear"], width=6,
                                    command=on_clear)
        self.clear_btn.pack(side="right", padx=2)
        self.rx_nl_var = tk.BooleanVar(value=True)
        self.rx_nl_btn = ttk.Checkbutton(head, text=rx_nl_text,
                                         variable=self.rx_nl_var)
        self.rx_nl_btn.pack(side="right", padx=6)

        self.text = scrolledtext.ScrolledText(
            self.frame, wrap="word", font=("Consolas", 10), height=12, state="disabled")
        self.text.pack(fill="both", expand=True, padx=4, pady=(0, 2))

        tx = ttk.Frame(self.frame)
        tx.pack(fill="x", padx=4, pady=(0, 4))
        self.send_btn = ttk.Button(tx, text="发送", width=6, command=self._send)
        self.send_btn.pack(side="right", padx=(4, 0))
        self.entry = ttk.Entry(tx)
        self.entry.pack(side="left", fill="x", expand=True)
        self.entry.bind("<Return>", lambda e: self._send())

        self.nl_var = tk.BooleanVar(value=True)
        self.nl_btn = ttk.Checkbutton(tx, text="换行", variable=self.nl_var)
        self.nl_btn.pack(side="right", padx=4)
        self.hex_var = tk.BooleanVar(value=False)
        self.hex_btn = None
        if with_hex:
            self.hex_btn = ttk.Checkbutton(tx, text="HEX", variable=self.hex_var)
            self.hex_btn.pack(side="right", padx=4)

    def _send(self):
        data = self.entry.get()
        if self.on_send:
            self.on_send(data, self.nl_var.get(), self.hex_var.get())

    def _insert(self, text):
        prefix = ""
        if self.ts_provider is not None:
            try:
                if self.ts_provider():
                    prefix = time.strftime("[%H:%M:%S] ")
            except Exception:
                prefix = ""
        self.text.configure(state="normal")
        self.text.insert("end", prefix + text)
        if float(self.text.index("end-1c").split(".")[0]) > 5000:
            self.text.delete("1.0", "500.0")
        self.text.see("end")
        self.text.configure(state="disabled")

    def append(self, text):
        """接收数据。勾选“自动换行”时按行落盘，无换行的数据也会在空闲后补一行，
        避免无线回显把多行粘成 `hello p4statushello p4...`。"""
        if not text:
            return
        if not self.rx_nl_var.get():
            self._insert(text)
            return
        self._pend += text
        self._pend_ts = time.time()
        while True:
            idx = self._pend.find("\n")
            if idx < 0:
                break
            line = self._pend[:idx].rstrip("\r")
            self._pend = self._pend[idx + 1:]
            if line:
                self._insert(line + "\n")
        if len(self._pend) > 512:
            self._insert(self._pend + "\n")
            self._pend = ""

    def flush_pending(self, idle=0.25):
        if self._pend and (time.time() - self._pend_ts) >= idle:
            self._insert(self._pend + "\n")
            self._pend = ""

    def clear(self):
        self._pend = ""
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.configure(state="disabled")

    def set_status(self, text, color="gray"):
        self.status_lbl.configure(text=text, foreground=color)

    def retitle(self, title, labels):
        self.frame.configure(text=title)
        self.clear_btn.configure(text=labels["clear"])
        self.send_btn.configure(text=labels["send"])
        self.nl_btn.configure(text=labels["newline"])
        self.rx_nl_btn.configure(text=labels.get("rx_auto_nl", "接收自动换行"))
        if self.hex_btn is not None:
            self.hex_btn.configure(text=labels["hex"])


class P4ToolApp:
    def __init__(self, root):
        self.root = root
        self.cfg = load_config()
        self.lang = self.cfg.get("language", "zh")

        self.root.geometry("1280x900")
        self.root.minsize(1000, 720)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.q = queue.Queue()

        # 串口
        self.ser = None
        self.serial_open = False
        self.serial_ports = []
        self.serial_rx_cache = ""   # 最近日志，用于提取 IP

        # 无线
        self.net_sock = None
        self.net_lock = threading.Lock()
        self.net_alive = False

        # 任务状态
        self.building = False
        self.build_ok = False
        self.ota_running = False
        self.ota_ok = False
        self.flashing = False
        self.flash_ok = False
        self.http_server = None
        self.http_thread = None
        self.ota_http_event = threading.Event()
        self.ota_serial_event = threading.Event()
        self.ota_current_bin = ""
        self.ota_total = 0
        self.ota_sent = 0
        self.pipeline_running = False

        # 接收区开关
        self.show_serial_var = tk.BooleanVar(value=True)
        self.show_net_var = tk.BooleanVar(value=True)
        # 接收时间戳（全局开关）
        self.ts_var = tk.BooleanVar(value=bool(self.cfg.get("rx_timestamp", False)))

        self._build_ui()
        self._apply_language()
        self._refresh_ports(verbose=False)
        self._restore_from_cfg()

        self.root.after(80, self._poll_queue)
        if self.cfg.get("auto_on_start", True):
            self.root.after(400, lambda: self._auto_start_probe())

    # ---------------- i18n ----------------
    def t(self, key, **kw):
        s = TEXTS.get(self.lang, TEXTS["zh"]).get(key, key)
        if kw:
            try:
                s = s.format(**kw)
            except Exception:
                pass
        return s

    def L(self):
        return TEXTS.get(self.lang, TEXTS["zh"])

    # ---------------- UI ----------------
    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=3)
        self.root.rowconfigure(5, weight=1)

        # ---- 连接设置 ----
        self.conn = ttk.LabelFrame(self.root)
        self.conn.grid(row=0, column=0, sticky="ew", padx=6, pady=4)

        r1 = ttk.Frame(self.conn)
        r1.pack(fill="x", padx=6, pady=3)
        self.lbl_serial = ttk.Label(r1)
        self.lbl_serial.pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(r1, textvariable=self.port_var, width=30)
        self.port_combo.pack(side="left", padx=4)
        self.scan_btn = ttk.Button(r1, command=lambda: self._refresh_ports(True))
        self.scan_btn.pack(side="left", padx=2)
        self.auto_port_btn = ttk.Button(r1, command=self._auto_detect_port)
        self.auto_port_btn.pack(side="left", padx=2)

        self.lbl_baud = ttk.Label(r1)
        self.lbl_baud.pack(side="left", padx=(10, 0))
        self.baud_var = tk.StringVar(value=self.cfg.get("serial_baud", str(BAUDRATE)))
        ttk.Combobox(r1, textvariable=self.baud_var, width=8,
                     values=["115200", "230400", "460800", "921600"]).pack(side="left", padx=4)

        self.serial_btn = ttk.Button(r1, command=self._toggle_serial)
        self.serial_btn.pack(side="left", padx=6)

        # ---- 第二行：IP ----
        r2 = ttk.Frame(self.conn)
        r2.pack(fill="x", padx=6, pady=3)
        self.lbl_ip = ttk.Label(r2)
        self.lbl_ip.pack(side="left")
        self.ip_var = tk.StringVar(value=self.cfg.get("ip", ""))
        ttk.Entry(r2, textvariable=self.ip_var, width=16).pack(side="left", padx=4)
        self.auto_ip_btn = ttk.Button(r2, command=self._auto_detect_ip)
        self.auto_ip_btn.pack(side="left", padx=2)
        self.ping_btn = ttk.Button(r2, command=self._ping)
        self.ping_btn.pack(side="left", padx=2)
        self.net_btn = ttk.Button(r2, command=self._toggle_net)
        self.net_btn.pack(side="left", padx=6)
        self.status_var = tk.StringVar(value="")
        ttk.Label(r2, textvariable=self.status_var, foreground="#0a7").pack(side="left", padx=10)

        # ---- 第三行：WiFi ----
        r3 = ttk.Frame(self.conn)
        r3.pack(fill="x", padx=6, pady=3)
        self.lbl_ssid = ttk.Label(r3)
        self.lbl_ssid.pack(side="left")
        self.wifi_ssid_var = tk.StringVar(value=self.cfg.get("wifi_ssid", ""))
        ttk.Entry(r3, textvariable=self.wifi_ssid_var, width=22).pack(side="left", padx=4)
        self.lbl_pass = ttk.Label(r3)
        self.lbl_pass.pack(side="left", padx=(10, 0))
        self.wifi_pass_var = tk.StringVar(value=self.cfg.get("wifi_password", ""))
        ttk.Entry(r3, textvariable=self.wifi_pass_var, width=20, show="*").pack(side="left", padx=4)
        self.wifi_btn = ttk.Button(r3, command=self._save_wifi)
        self.wifi_btn.pack(side="left", padx=6)

        # ---- 第四行：路径 ----
        r4 = ttk.Frame(self.conn)
        r4.pack(fill="x", padx=6, pady=3)
        self.lbl_proj = ttk.Label(r4)
        self.lbl_proj.pack(side="left")
        self.proj_var = tk.StringVar(value=self.cfg.get("p4_project", ""))
        ttk.Entry(r4, textvariable=self.proj_var, width=46).pack(side="left", padx=4)
        self.proj_btn = ttk.Button(r4, command=self._browse_project)
        self.proj_btn.pack(side="left", padx=2)

        self.lbl_c5 = ttk.Label(r4)
        self.lbl_c5.pack(side="left", padx=(10, 0))
        self.c5_var = tk.StringVar(value=self.cfg.get("c5_prebuilt", ""))
        ttk.Entry(r4, textvariable=self.c5_var, width=36).pack(side="left", padx=4)
        self.c5_btn = ttk.Button(r4, command=self._browse_c5)
        self.c5_btn.pack(side="left", padx=2)

        # ---- 第 2 行：通道容器 ----
        self.chan_frame = ttk.LabelFrame(self.root)
        self.chan_frame.grid(row=2, column=0, sticky="nsew", padx=6, pady=4)
        self.chan_frame.columnconfigure(0, weight=1)
        self.chan_frame.columnconfigure(1, weight=1)
        self.chan_frame.rowconfigure(1, weight=1)

        self.serial_chk = ttk.Checkbutton(self.chan_frame, variable=self.show_serial_var,
                                          command=self._apply_channel_layout)
        self.serial_chk.grid(row=0, column=0, sticky="w", padx=6, pady=2)
        self.net_chk = ttk.Checkbutton(self.chan_frame, variable=self.show_net_var,
                                       command=self._apply_channel_layout)
        self.net_chk.grid(row=0, column=1, sticky="w", padx=6, pady=2)

        self.serial_ch = ChannelPanel(
            self.chan_frame, self.t("ch_serial"), self._serial_send,
            lambda: self._hide_channel("serial"), self._serial_clear,
            with_hex=True, ts_provider=lambda: self.ts_var.get(),
            rx_nl_text=self.t("rx_auto_nl"))
        self.serial_ch.frame.grid(row=1, column=0, sticky="nsew", padx=(4, 2), pady=4)

        self.net_ch = ChannelPanel(
            self.chan_frame, self.t("ch_net"), self._net_send,
            lambda: self._hide_channel("net"), self._net_clear,
            ts_provider=lambda: self.ts_var.get(),
            rx_nl_text=self.t("rx_auto_nl"))
        self.net_ch.frame.grid(row=1, column=1, sticky="nsew", padx=(2, 4), pady=4)

        self.serial_ch.set_status(self.t("not_connected"))
        self.net_ch.set_status(self.t("not_connected"))

        # ---- 功能页 ----
        self.nb = ttk.Notebook(self.root)
        self.nb.grid(row=3, column=0, sticky="ew", padx=6, pady=4)

        self._build_tcp_tab()
        self._build_ota_tab()
        self._build_flash_tab()
        self._build_auto_tab()

        # ---- 日志区 ----
        self.log_frame = ttk.LabelFrame(self.root)
        self.log_frame.grid(row=5, column=0, sticky="nsew", padx=6, pady=4)
        logbar = ttk.Frame(self.log_frame)
        logbar.pack(fill="x", padx=4, pady=2)
        self.save_log_btn = ttk.Button(logbar, command=self._save_log)
        self.save_log_btn.pack(side="right", padx=2)
        self.log_clear_btn = ttk.Button(logbar, command=self._clear_log)
        self.log_clear_btn.pack(side="right", padx=2)
        self.log_text = scrolledtext.ScrolledText(
            self.log_frame, wrap="word", height=8, font=("Consolas", 10), state="disabled")
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

    def _build_tcp_tab(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f)
        row = ttk.Frame(f)
        row.pack(fill="x", padx=6, pady=8)
        ttk.Label(row, text="命令").pack(side="left")
        self.cmd_var = tk.StringVar(value="hello p4")
        ttk.Entry(row, textvariable=self.cmd_var, width=44).pack(side="left", padx=6)
        ttk.Button(row, text="发送到无线通道", command=self._tcp_send_text).pack(side="left", padx=3)
        ttk.Button(row, text="hello p4", command=lambda: self._tcp_send_raw("hello p4")).pack(side="left", padx=3)
        ttk.Button(row, text="status", command=lambda: self._tcp_send_raw("status")).pack(side="left", padx=3)
        ttk.Button(row, text="reboot 重启设备",
                   command=self._tcp_reboot).pack(side="left", padx=3)
        row2 = ttk.Frame(f)
        row2.pack(fill="x", padx=6, pady=6)
        ttk.Label(row2, text="常用：hello p4 / status / reboot / ota-push（工具自动发送）").pack(side="left")

    def _build_ota_tab(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f)

        r1 = ttk.Frame(f)
        r1.pack(fill="x", padx=6, pady=6)
        self.lbl_ota_mode = ttk.Label(r1)
        self.lbl_ota_mode.pack(side="left")
        self.ota_mode_var = tk.StringVar(value=self.cfg.get("ota_mode", "push"))
        ttk.Radiobutton(r1, text=self.t("ota_push"), value="push",
                        variable=self.ota_mode_var).pack(side="left", padx=6)
        ttk.Radiobutton(r1, text=self.t("ota_http"), value="http",
                        variable=self.ota_mode_var).pack(side="left", padx=6)

        r2 = ttk.Frame(f)
        r2.pack(fill="x", padx=6, pady=6)
        self.lbl_fw = ttk.Label(r2)
        self.lbl_fw.pack(side="left")
        default_dir = self.cfg.get("ota_dir") or os.path.join(
            self.cfg.get("p4_project", ""), "build")
        self.bin_var = tk.StringVar(value=self.cfg.get("ota_bin") or "")
        ttk.Entry(r2, textvariable=self.bin_var, width=64).pack(side="left", padx=4)
        self.browse_btn = ttk.Button(r2, command=self._browse_bin)
        self.browse_btn.pack(side="left", padx=2)
        self._ota_default_dir = default_dir

        r3 = ttk.Frame(f)
        r3.pack(fill="x", padx=6, pady=6)
        self.lbl_srv = ttk.Label(r3)
        self.lbl_srv.pack(side="left")
        self.http_host_var = tk.StringVar(value=self.cfg.get("http_host", ""))
        ttk.Entry(r3, textvariable=self.http_host_var, width=18).pack(side="left", padx=4)
        ttk.Label(r3, text=":").pack(side="left")
        self.http_port_var = tk.StringVar(value=self.cfg.get("http_port", str(HTTP_PORT)))
        ttk.Entry(r3, textvariable=self.http_port_var, width=7).pack(side="left", padx=2)
        self.selfcheck_btn = ttk.Button(r3, command=self._http_selfcheck)
        self.selfcheck_btn.pack(side="left", padx=8)
        self.fw_btn = ttk.Button(r3, text="防火墙放行", command=self._add_firewall_rule)
        self.fw_btn.pack(side="left", padx=2)

        r_hint = ttk.Frame(f)
        r_hint.pack(fill="x", padx=6, pady=(0, 2))
        ttk.Label(r_hint, foreground="gray",
                  text="无线升级只写 app 分区；bootloader / 分区表请用「固件烧录」页。"
                       "TCP 推送无需端口放行，HTTP 需要。").pack(side="left")

        r4 = ttk.Frame(f)
        r4.pack(fill="x", padx=6, pady=6)
        self.build_btn = ttk.Button(r4, command=self._build)
        self.build_btn.pack(side="left", padx=3)
        self.ota_btn = ttk.Button(r4, command=self._ota_start)
        self.ota_btn.pack(side="left", padx=3)

        r5 = ttk.Frame(f)
        r5.pack(fill="x", padx=6, pady=4)
        self.lbl_prog = ttk.Label(r5)
        self.lbl_prog.pack(side="left")
        self.ota_progress = ttk.Progressbar(r5, length=420, mode="determinate")
        self.ota_progress.pack(side="left", padx=4, fill="x", expand=True)
        self.ota_progress_lbl = ttk.Label(r5, text="0%")
        self.ota_progress_lbl.pack(side="left", padx=6)

    def _build_flash_tab(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f)

        r1 = ttk.Frame(f)
        r1.pack(fill="x", padx=6, pady=4)
        self.lbl_chip = ttk.Label(r1)
        self.lbl_chip.pack(side="left")
        self.flash_chip_var = tk.StringVar(value=self.cfg.get("flash_chip", "esp32p4"))
        self.chip_combo = ttk.Combobox(r1, textvariable=self.flash_chip_var, width=10,
                                       state="readonly", values=list(CHIP_PRESETS.keys()))
        self.chip_combo.pack(side="left", padx=4)
        self.chip_combo.bind("<<ComboboxSelected>>", lambda e: self._load_flash_preset())

        self.lbl_fport = ttk.Label(r1)
        self.lbl_fport.pack(side="left", padx=(10, 0))
        self.flash_port_var = tk.StringVar(value=self.cfg.get("flash_port", ""))
        self.flash_port_combo = ttk.Combobox(r1, textvariable=self.flash_port_var, width=30)
        self.flash_port_combo.pack(side="left", padx=4)

        self.lbl_fbaud = ttk.Label(r1)
        self.lbl_fbaud.pack(side="left", padx=(10, 0))
        self.flash_baud_var = tk.StringVar(value=self.cfg.get("flash_baud", "460800"))
        ttk.Combobox(r1, textvariable=self.flash_baud_var, width=8,
                     values=["460800", "921600", "115200", "230400"]).pack(side="left", padx=4)

        self.files_frame = ttk.LabelFrame(f)
        self.files_frame.pack(fill="both", expand=True, padx=6, pady=4)
        self.flash_tree = ttk.Treeview(self.files_frame, columns=("offset", "path"),
                                       show="headings", height=5)
        self.flash_tree.heading("offset", text="Offset")
        self.flash_tree.heading("path", text="File")
        self.flash_tree.column("offset", width=110, anchor="center")
        self.flash_tree.column("path", width=760, anchor="w")
        self.flash_tree.pack(fill="both", expand=True, padx=4, pady=4)

        bar = ttk.Frame(f)
        bar.pack(fill="x", padx=6, pady=4)
        self.add_btn = ttk.Button(bar, command=self._flash_add)
        self.add_btn.pack(side="left", padx=2)
        self.del_btn = ttk.Button(bar, command=self._flash_del)
        self.del_btn.pack(side="left", padx=2)
        self.clear_btn = ttk.Button(bar, command=self._flash_clear)
        self.clear_btn.pack(side="left", padx=2)
        self.c5_btn2 = ttk.Button(bar, command=lambda: self._load_flash_preset("esp32c5"))
        self.c5_btn2.pack(side="left", padx=8)
        self.p4_btn2 = ttk.Button(bar, command=lambda: self._load_flash_preset("esp32p4"))
        self.p4_btn2.pack(side="left", padx=2)
        self.flash_btn = ttk.Button(bar, command=self._flash_start)
        self.flash_btn.pack(side="left", padx=12)

        r3 = ttk.Frame(f)
        r3.pack(fill="x", padx=6, pady=4)
        self.flash_progress = ttk.Progressbar(r3, length=420, mode="determinate")
        self.flash_progress.pack(side="left", padx=4, fill="x", expand=True)
        self.flash_progress_lbl = ttk.Label(r3, text="0%")
        self.flash_progress_lbl.pack(side="left", padx=6)

    def _build_auto_tab(self):
        f = ttk.Frame(self.nb)
        self.nb.add(f)
        r1 = ttk.Frame(f)
        r1.pack(fill="x", padx=6, pady=8)
        self.auto_start_var = tk.BooleanVar(value=bool(self.cfg.get("auto_on_start", True)))
        ttk.Checkbutton(r1, text=self.t("auto_on_start"),
                        variable=self.auto_start_var).pack(side="left", padx=6)
        self.auto_reconn_var = tk.BooleanVar(value=bool(self.cfg.get("auto_reconnect", True)))
        ttk.Checkbutton(r1, text=self.t("auto_reconnect"),
                        variable=self.auto_reconn_var).pack(side="left", padx=6)
        ttk.Checkbutton(r1, text=self.t("rx_ts"),
                        variable=self.ts_var).pack(side="left", padx=6)

        r2 = ttk.Frame(f)
        r2.pack(fill="x", padx=6, pady=8)
        self.pipeline_btn = ttk.Button(r2, command=self._run_pipeline)
        self.pipeline_btn.pack(side="left", padx=4)
        self.pipeline_flash_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(r2, text="流程开始时先烧录当前预设固件",
                        variable=self.pipeline_flash_var).pack(side="left", padx=8)

        r3 = ttk.Frame(f)
        r3.pack(fill="x", padx=6, pady=6)
        ttk.Label(r3, text=self.t("pipeline")).pack(side="left")

    def _apply_language(self):
        self.root.title(self.t("title"))
        self.conn.configure(text=self.t("conn"))
        self.chan_frame.configure(text=self.t("channels"))
        self.lbl_serial.configure(text=self.t("serial"))
        self.lbl_baud.configure(text=self.t("baud"))
        self.lbl_ip.configure(text=self.t("p4_ip"))
        self.lbl_ssid.configure(text=self.t("wifi_ssid"))
        self.lbl_pass.configure(text=self.t("wifi_password"))
        self.lbl_proj.configure(text=self.t("project_dir"))
        self.lbl_c5.configure(text=self.t("c5_dir"))
        self.scan_btn.configure(text=self.t("scan"))
        self.auto_port_btn.configure(text=self.t("auto_detect_port"))
        self.auto_ip_btn.configure(text=self.t("auto_ip"))
        self.ping_btn.configure(text=self.t("ping"))
        self.wifi_btn.configure(text=self.t("wifi_save"))
        self.proj_btn.configure(text=self.t("browse"))
        self.c5_btn.configure(text=self.t("browse"))
        self.serial_btn.configure(text=self.t("serial_disconnect" if self.serial_open
                                              else "serial_connect"))
        self.net_btn.configure(text=self.t("net_disconnect" if self.net_alive
                                           else "net_connect"))
        self.serial_chk.configure(text=self.t("ch_serial"))
        self.net_chk.configure(text=self.t("ch_net"))
        labels = self.L()
        self.serial_ch.retitle(self.t("ch_serial"), labels)
        self.net_ch.retitle(self.t("ch_net"), labels)
        self.nb.tab(0, text=self.t("tab_tcp"))
        self.nb.tab(1, text=self.t("tab_ota"))
        self.nb.tab(2, text=self.t("tab_flash"))
        self.nb.tab(3, text=self.t("tab_auto"))
        self.lbl_ota_mode.configure(text=self.t("ota_mode"))
        self.lbl_fw.configure(text=self.t("ota_fw"))
        self.lbl_srv.configure(text=self.t("ota_server"))
        self.browse_btn.configure(text=self.t("browse"))
        self.build_btn.configure(text=self.t("build"))
        self.ota_btn.configure(text=self.t("upgrade"))
        self.selfcheck_btn.configure(text=self.t("selfcheck"))
        self.lbl_prog.configure(text=self.t("progress"))
        self.lbl_chip.configure(text=self.t("flash_chip"))
        self.lbl_fport.configure(text=self.t("flash_port"))
        self.lbl_fbaud.configure(text=self.t("flash_baud"))
        self.files_frame.configure(text=self.t("flash_files"))
        self.add_btn.configure(text=self.t("add"))
        self.del_btn.configure(text=self.t("del"))
        self.clear_btn.configure(text=self.t("clear_all"))
        self.c5_btn2.configure(text=self.t("load_c5"))
        self.p4_btn2.configure(text=self.t("load_p4"))
        self.flash_btn.configure(text=self.t("start_flash"))
        self.pipeline_btn.configure(text=self.t("run_pipeline"))
        self.log_frame.configure(text=self.t("log_area"))
        self.save_log_btn.configure(text=self.t("save_log"))
        self.log_clear_btn.configure(text=self.t("clear"))

    # ---------------- 队列 ----------------
    def _push(self, kind, data):
        self.q.put((kind, data))

    def _poll_queue(self):
        try:
            while True:
                kind, data = self.q.get_nowait()
                if kind == "serial_rx":
                    self.serial_ch.append(data)
                    self._remember_serial_text(data)
                elif kind == "net_rx":
                    self.net_ch.append(data)
                    self._remember_serial_text(data)
                elif kind == "log":
                    self._append_log(data)
                elif kind == "status":
                    self.status_var.set(data)
                elif kind == "serial_status":
                    self.serial_ch.set_status(data[0], data[1])
                elif kind == "net_status":
                    self.net_ch.set_status(data[0], data[1])
                elif kind == "prog":
                    which, val = data
                    self._set_progress(which, val)
                elif kind == "ui":
                    try:
                        data()
                    except Exception as e:
                        self._append_log(f"[UI] {e}\n")
        except queue.Empty:
            pass
        # 没有换行的数据（例如设备原样回显）在空闲后补一行
        try:
            self.serial_ch.flush_pending()
            self.net_ch.flush_pending()
        except Exception:
            pass
        self.root.after(60, self._poll_queue)

    def _set_progress(self, which, val):
        val = max(0.0, min(100.0, val))
        if which == "ota":
            self.ota_progress["value"] = val
            self.ota_progress_lbl.configure(text=f"{val:.0f}%")
        elif which == "flash":
            self.flash_progress["value"] = val
            self.flash_progress_lbl.configure(text=f"{val:.0f}%")

    def _append_log(self, text):
        # 后台线程不能直接改 Tk 控件，统一转成队列消息
        if threading.current_thread() is not threading.main_thread():
            self.q.put(("log", text))
            return
        self.log_text.configure(state="normal")
        self.log_text.insert("end", strip_ansi(text))
        if float(self.log_text.index("end-1c").split(".")[0]) > 5000:
            self.log_text.delete("1.0", "500.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _save_log(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".txt", filetypes=[("log", "*.txt"), ("all", "*.*")],
            initialfile=time.strftime("p4tool_%Y%m%d_%H%M%S.txt"))
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.serial_ch.text.get("1.0", "end"))
                f.write("\n----- wireless -----\n")
                f.write(self.net_ch.text.get("1.0", "end"))
                f.write("\n----- log -----\n")
                f.write(self.log_text.get("1.0", "end"))
            self._append_log(f"[日志] 已保存: {path}\n")
        except Exception as e:
            messagebox.showerror("Log", str(e))

    # ---------------- 通道布局 ----------------
    def _apply_channel_layout(self):
        show_s = self.show_serial_var.get()
        show_n = self.show_net_var.get()
        for col, ch, show in ((0, self.serial_ch, show_s), (1, self.net_ch, show_n)):
            if show:
                ch.frame.grid(row=1, column=col, sticky="nsew",
                              padx=(4, 2) if col == 0 else (2, 4), pady=4)
                self.chan_frame.columnconfigure(col, weight=1)
            else:
                ch.frame.grid_remove()
                self.chan_frame.columnconfigure(col, weight=0)

    def _hide_channel(self, which):
        """✕ 关闭：连底层连接一起断开，并隐藏整个面板。"""
        if which == "serial":
            self.show_serial_var.set(False)
            self._close_serial()
        else:
            self.show_net_var.set(False)
            self._close_net()
        self._apply_channel_layout()

    # ---------------- 串口 ----------------
    def _refresh_ports(self, verbose=True):
        self.serial_ports = list_serial_ports()
        items = [f"{dev}  |  {desc}" for dev, desc, _, _ in self.serial_ports]
        self.port_combo["values"] = items
        self.flash_port_combo["values"] = items
        desc_map = dict((d, s) for d, s, _, _ in self.serial_ports)
        devs = list(desc_map.keys())

        def ensure(var, saved):
            """优先保留上次选择（记忆），其次当前有效值，最后自动识别。"""
            cur = (var.get() or "").split("  |")[0].strip()
            for cand in (saved, cur):
                if cand and cand in devs:
                    var.set(f"{cand}  |  {desc_map.get(cand, '')}")
                    return
            if devs:
                pick = guess_esp_port(self.serial_ports)
                var.set(f"{pick}  |  {desc_map.get(pick, '')}")

        ensure(self.port_var, self.cfg.get("serial_port"))
        ensure(self.flash_port_var, self.cfg.get("flash_port"))

        if verbose:
            if not HAS_SERIAL:
                self._append_log("[串口] 未安装 pyserial: pip install pyserial\n")
            elif not devs:
                self._append_log("[串口] 未发现任何串口设备\n")
            else:
                lines = [f"[串口] {d} | {s}" +
                         (f" | VID={v:04X} PID={p:04X}" if v else "")
                         for d, s, v, p in self.serial_ports]
                self._append_log("\n".join(lines) + "\n")

    def _port_name(self, var):
        return (var.get() or "").split("  |")[0].strip()

    def _auto_detect_port(self):
        self._refresh_ports(verbose=False)
        if not self.serial_ports:
            self._append_log("[串口] 没有可用串口\n")
            return
        pick = guess_esp_port(self.serial_ports)
        desc = dict((d, s) for d, s, _, _ in self.serial_ports).get(pick, "")
        self.port_var.set(f"{pick}  |  {desc}")
        self.flash_port_var.set(f"{pick}  |  {desc}")
        self._append_log(f"[串口] 自动识别 -> {pick} ({desc})\n")

    def _toggle_serial(self):
        if self.serial_open:
            self._close_serial()
        else:
            self._open_serial()

    def _open_serial(self):
        if not HAS_SERIAL:
            messagebox.showerror("Serial", "未安装 pyserial：pip install pyserial")
            return
        port = self._port_name(self.port_var)
        if not port:
            messagebox.showwarning("Serial", "请选择串口")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = BAUDRATE
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            messagebox.showerror("Serial", f"{port}\n{e}")
            return
        self.serial_open = True
        self.show_serial_var.set(True)
        self._apply_channel_layout()
        self.serial_btn.configure(text=self.t("serial_disconnect"))
        self.serial_ch.set_status(f"{self.t('connected')} {port} @ {baud}", "#0a7")
        self._append_log(f"[串口] 已连接 {port} @ {baud}\n")
        threading.Thread(target=self._serial_reader, daemon=True).start()

    def _close_serial(self):
        self.serial_open = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.serial_btn.configure(text=self.t("serial_connect"))
        self.serial_ch.set_status(self.t("not_connected"))

    def _serial_reader(self):
        while self.serial_open and self.ser:
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n if n else 1)
                if data:
                    self._push("serial_rx", data.decode("utf-8", errors="replace"))
                    self._check_serial_markers(data.decode("utf-8", errors="replace"))
                    self._auto_ip_from_text(data.decode("utf-8", errors="replace"))
            except Exception as e:
                self._push("log", f"[串口] 读取错误: {e}\n")
                break

    def _serial_clear(self):
        self.serial_ch.clear()
        self.serial_rx_cache = ""

    def _serial_send(self, text, newline, use_hex):
        if not self.serial_open or not self.ser:
            messagebox.showwarning("Serial", "串口未连接")
            return
        try:
            if use_hex:
                raw = bytes.fromhex(text.replace(" ", "").replace(",", ""))
            else:
                raw = text.encode("utf-8")
                if newline:
                    raw += b"\r\n"
            self.ser.write(raw)
            self._append_log(f"[串口 TX] {text}\n")
        except Exception as e:
            messagebox.showerror("Serial", str(e))

    # ---------------- 无线 TCP ----------------
    def _toggle_net(self):
        if self.net_alive:
            self._close_net()
        else:
            self._open_net()

    def _open_net(self):
        ip = self.ip_var.get().strip()
        if not ip:
            messagebox.showwarning("TCP", "请先填写或自动识别 P4 IP")
            return
        try:
            sock = socket.create_connection((ip, TCP_PORT), timeout=5)
            sock.settimeout(0.5)
        except Exception as e:
            messagebox.showerror("TCP", f"{ip}:{TCP_PORT}\n{e}")
            return
        self.net_sock = sock
        self.net_alive = True
        self.show_net_var.set(True)
        self._apply_channel_layout()
        self.net_btn.configure(text=self.t("net_disconnect"))
        self.net_ch.set_status(f"{self.t('connected')} {ip}:{TCP_PORT}", "#0a7")
        self._append_log(f"[TCP] 已连接 {ip}:{TCP_PORT}（接收 P4 日志镜像）\n")
        threading.Thread(target=self._net_reader, daemon=True).start()

    def _close_net(self):
        self.net_alive = False
        sock, self.net_sock = self.net_sock, None
        if sock:
            try:
                sock.close()
            except Exception:
                pass
        self.net_btn.configure(text=self.t("net_connect"))
        self.net_ch.set_status(self.t("not_connected"))

    def _net_reader(self):
        sock = self.net_sock
        while self.net_alive and sock is not None and sock is self.net_sock:
            try:
                data = sock.recv(4096)
                if not data:
                    break
                text = data.decode("utf-8", errors="replace")
                self._push("net_rx", text)
                self._check_serial_markers(text)
                self._auto_ip_from_text(text)
            except socket.timeout:
                continue
            except Exception:
                break
        if sock is self.net_sock:
            self.net_alive = False
            self._push("ui", lambda: self.net_btn.configure(text=self.t("net_connect")))
            self._push("net_status", (self.t("not_connected"), "#c00"))
            self._append_log("[TCP] 连接已断开\n")
            if self.auto_reconn_var.get():
                self._push("ui", lambda: self.root.after(2000, self._try_reconnect_net))

    def _try_reconnect_net(self):
        if self.net_alive or not self.auto_reconn_var.get():
            return
        ip = self.ip_var.get().strip()
        if not ip:
            return
        try:
            sock = socket.create_connection((ip, TCP_PORT), timeout=3)
            sock.settimeout(0.5)
        except Exception:
            self.root.after(3000, self._try_reconnect_net)
            return
        self.net_sock = sock
        self.net_alive = True
        self.net_btn.configure(text=self.t("net_disconnect"))
        self.net_ch.set_status(f"{self.t('connected')} {ip}:{TCP_PORT}", "#0a7")
        self._append_log("[TCP] 已自动重连\n")
        threading.Thread(target=self._net_reader, daemon=True).start()

    def _net_clear(self):
        self.net_ch.clear()

    def _net_send(self, text, newline, use_hex):
        if not self.net_alive or self.net_sock is None:
            self._open_net()
            if not self.net_alive:
                return
        payload = (text + ("\r\n" if newline else "")).encode("utf-8")
        try:
            with self.net_lock:
                self.net_sock.sendall(payload)
            self._append_log(f"[TCP TX] {text}\n")
        except Exception as e:
            messagebox.showerror("TCP", str(e))

    def _tcp_send_text(self):
        self._net_send(self.cmd_var.get(), True, False)

    def _tcp_send_raw(self, text):
        self._net_send(text, True, False)

    def _tcp_reboot(self):
        if not messagebox.askyesno("reboot", "确定重启 ESP32-P4？"):
            return
        self._append_log("[命令] reboot\n")
        self._net_send("reboot", True, False)

    # ---------------- IP 自动识别 ----------------
    def _remember_serial_text(self, text):
        self.serial_rx_cache = (self.serial_rx_cache + text)[-4000:]

    def _ip_from_text(self, text):
        for pat in IP_PATTERNS:
            m = pat.search(text)
            if m:
                ip = m.group(1)
                # 过滤明显无效的地址
                if ip.count(".") == 3 and not ip.startswith("0."):
                    return ip
        return None

    def _auto_ip_from_text(self, text):
        ip = self._ip_from_text(text)
        # 后台线程里不要读 Tk 变量，用普通属性比较即可
        if ip and ip != self.cfg.get("ip"):
            self.cfg["ip"] = ip
            self._set_ip(ip, "日志")

    def _local_ipv4_list(self):
        ips = set()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.2)
            s.connect(("8.8.8.8", 80))
            ips.add(s.getsockname()[0])
            s.close()
        except Exception:
            pass
        try:
            for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
                ip = info[4][0]
                if not ip.startswith("127."):
                    ips.add(ip)
        except Exception:
            pass
        return [ip for ip in ips if ip and not ip.startswith("127.")]

    def _probe(self, ip, timeout=0.7):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        try:
            if s.connect_ex((ip, TCP_PORT)) == 0:
                s.settimeout(max(0.6, timeout))
                try:
                    s.sendall(b"ping")
                    if s.recv(64):
                        return ip
                except Exception:
                    pass
                return ip
        except Exception:
            pass
        finally:
            try:
                s.close()
            except Exception:
                pass
        return None

    def _scan_subnet(self):
        bases = sorted({ip.rsplit(".", 1)[0] for ip in self._local_ipv4_list()})
        if not bases:
            return None
        self._append_log("[IP] 扫描 %s 端口 %d ...\n"
                         % (", ".join(b + ".0/24" for b in bases), TCP_PORT))
        found = []

        def probe(ip):
            if found:
                return None
            return self._probe(ip)

        with ThreadPoolExecutor(max_workers=128) as ex:
            futs = [ex.submit(probe, f"{b}.{i}") for b in bases for i in range(1, 255)]
            for fut in as_completed(futs):
                try:
                    r = fut.result()
                except Exception:
                    r = None
                if r:
                    found.append(r)
                    for f in futs:
                        f.cancel()
                    break
        return found[0] if found else None

    def _auto_detect_ip(self):
        self._append_log("[IP] 开始自动识别...\n")

        def run():
            # 1) 已连接的 TCP
            if self.net_alive and self.net_sock:
                try:
                    ip = self.net_sock.getpeername()[0]
                    self._set_ip(ip, "TCP 连接")
                    return
                except Exception:
                    pass
            # 2) 串口日志缓存
            ip = self._ip_from_text(self.serial_rx_cache)
            if ip:
                self._set_ip(ip, "串口日志")
                return
            # 3) 局域网扫描
            ip = self._scan_subnet()
            if ip:
                self._set_ip(ip, "局域网扫描")
            else:
                self._push("log", "[IP] 未找到 P4：确认已连同一 WiFi / 已烧新固件\n")

        threading.Thread(target=run, daemon=True).start()

    def _set_ip(self, ip, source):
        def apply():
            self.ip_var.set(ip)
            self.cfg["ip"] = ip
            self._append_log(f"[IP] 自动识别({source}) -> {ip}\n")
        self._push("ui", apply)

    def _auto_start_probe(self):
        self._auto_detect_port()
        self._auto_detect_ip()

    # ---------------- 标记检测 ----------------
    def _check_serial_markers(self, text):
        if ("OTA succeeded" in text or "Restarting to apply" in text
                or "OTA finished" in text):
            if not self.ota_serial_event.is_set():
                self.ota_serial_event.set()
                self._append_log("[OTA] 设备报告升级成功\n")
        for bad in ("ota end failed", "OTA incomplete", "http open failed",
                    "ota write failed", "首字节"):
            if bad in text:
                self._append_log(f"[OTA] 设备报错: {bad}\n")

    # ---------------- Ping ----------------
    def _ping(self):
        ip = self.ip_var.get().strip()
        if not ip:
            return
        self._append_log(f"[Ping] {ip} ...\n")

        def run():
            try:
                r = subprocess.run(["ping", "-n", "2", "-w", "1000", ip],
                                   capture_output=True, text=True, timeout=15,
                                   creationflags=CREATE_NO_WINDOW)
                ok = r.returncode == 0
                self._append_log(f"[Ping] {ip} {'可达 ✅' if ok else '不可达 ❌'}\n")
            except Exception as e:
                self._append_log(f"[Ping] {e}\n")

        threading.Thread(target=run, daemon=True).start()

    def _ping_once(self, ip, wait_ms=1000):
        try:
            r = subprocess.run(["ping", "-n", "1", "-w", str(wait_ms), ip],
                               capture_output=True, timeout=5,
                               creationflags=CREATE_NO_WINDOW)
            return r.returncode == 0
        except Exception:
            return False

    # ---------------- WiFi 保存 ----------------
    def _save_wifi(self, silent=False):
        return self._save_wifi_values(self.wifi_ssid_var.get().strip(),
                                      self.wifi_pass_var.get(),
                                      self.proj_var.get().strip(), silent)

    def _save_wifi_values(self, ssid, password, proj, silent=False):
        """纯逻辑版本：不触碰任何 Tk 变量，可安全地在后台线程调用。"""
        if not ssid:
            if not silent:
                messagebox.showwarning("WiFi", "WiFi 名称不能为空")
            return False
        try:
            ssid.encode("ascii")
        except UnicodeEncodeError:
            if not silent:
                messagebox.showerror("WiFi", "WiFi 名称仅支持英文（ASCII）")
            return False
        if password and len(password) < 8:
            if not silent:
                messagebox.showwarning("WiFi", "WPA/WPA2 密码至少 8 位")
            return False

        defaults = os.path.join(proj, "sdkconfig.defaults")
        sdkconfig = os.path.join(proj, "sdkconfig")
        if not os.path.isfile(defaults):
            if not silent:
                messagebox.showerror("WiFi", f"未找到 sdkconfig.defaults:\n{defaults}")
            return False

        values = {
            "CONFIG_ESP_WIFI_SSID": json.dumps(ssid, ensure_ascii=False),
            "CONFIG_ESP_WIFI_PASSWORD": json.dumps(password, ensure_ascii=False),
        }
        try:
            for path in (defaults, sdkconfig):
                if not os.path.isfile(path):
                    continue
                with open(path, "r", encoding="utf-8", errors="replace") as f:
                    lines = f.readlines()
                out = []
                seen = set()
                for line in lines:
                    stripped = line.strip()
                    key = stripped.split("=", 1)[0].strip() if "=" in stripped else ""
                    if key in values:
                        if key not in seen:
                            out.append(f"{key}={values[key]}\n")
                            seen.add(key)
                    else:
                        out.append(line)
                for key, val in values.items():
                    if key not in seen:
                        out.append(f"{key}={val}\n")
                with open(path, "w", encoding="utf-8") as f:
                    f.writelines(out)
        except Exception as e:
            if not silent:
                messagebox.showerror("WiFi", str(e))
            return False

        self.cfg["wifi_ssid"] = ssid
        self.cfg["wifi_password"] = password
        save_config(self.cfg)
        self._append_log(f"[WiFi] 已写入工程: {ssid}\n")
        return True

    # ---------------- 路径选择 ----------------
    def _browse_project(self):
        path = filedialog.askdirectory(initialdir=self.proj_var.get() or TOOLS_DIR)
        if path:
            self.proj_var.set(path)
            self.cfg["p4_project"] = path
            self.cfg["c5_prebuilt"] = find_c5_prebuilt(path)
            self.c5_var.set(self.cfg["c5_prebuilt"])
            save_config(self.cfg)

    def _browse_c5(self):
        path = filedialog.askdirectory(initialdir=self.c5_var.get() or TOOLS_DIR)
        if path:
            self.c5_var.set(path)
            self.cfg["c5_prebuilt"] = path
            save_config(self.cfg)

    def _restore_from_cfg(self):
        self.proj_var.set(self.cfg.get("p4_project", ""))
        self.c5_var.set(self.cfg.get("c5_prebuilt", ""))
        if not self.ip_var.get():
            self.ip_var.set(self.cfg.get("ip", ""))
        # 通道显示状态也记住
        self.show_serial_var.set(bool(self.cfg.get("show_serial", True)))
        self.show_net_var.set(bool(self.cfg.get("show_net", True)))
        self._load_flash_preset()
        self._apply_channel_layout()

    # ---------------- 构建 ----------------
    def _find_app_bin(self, project_dir):
        build = os.path.join(project_dir, "build")
        if not os.path.isdir(build):
            return None
        cands = [os.path.join(build, f) for f in os.listdir(build)
                 if f.endswith(".bin") and f not in (
                     "bootloader.bin", "partition-table.bin", "ota_data_initial.bin")]
        return max(cands, key=os.path.getsize) if cands else None

    def _build(self):
        if self.building:
            self._append_log("[构建] 已在构建中\n")
            return
        proj = self.proj_var.get().strip()
        if not os.path.isdir(proj):
            messagebox.showerror("Build", f"工程目录不存在:\n{proj}")
            return
        if not self._save_wifi(silent=True):
            self._append_log("[构建] ⚠️ WiFi 配置无效，继续构建（沿用现有配置）\n")

        self.building = True
        self.build_ok = False
        self.build_btn.configure(state="disabled")
        self._set_progress("ota", 0)
        self._append_log("[构建] 开始 idf.py build ...\n")

        export_bat = os.path.join(IDF_PATH, "export.bat")
        cmd = f'call "{export_bat}" >nul 2>&1 && idf.py build'
        os.makedirs(IDF_TMP_DIR, exist_ok=True)
        env = os.environ.copy()
        env["TMP"] = IDF_TMP_DIR
        env["TEMP"] = IDF_TMP_DIR
        env["IDF_PYTHON_ENV_PATH"] = IDF_PYTHON_ENV_PATH
        ninja_re = re.compile(r"\[(\d+)/(\d+)\]")

        def run():
            try:
                proc = subprocess.Popen(cmd, shell=True, cwd=proj, env=env,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, encoding="utf-8", errors="replace",
                                        creationflags=CREATE_NO_WINDOW)
                for line in proc.stdout:
                    line = strip_ansi(line.rstrip("\n"))
                    if not line.strip():
                        continue
                    self._push("log", line + "\n")
                    m = ninja_re.search(line)
                    if m and int(m.group(2)) > 0:
                        self._push("prog", ("ota", int(m.group(1)) / int(m.group(2)) * 100))
                proc.wait()
                self.build_ok = (proc.returncode == 0)
                if self.build_ok:
                    self._push("prog", ("ota", 100.0))
                    bin_path = self._find_app_bin(proj)
                    if bin_path:
                        self._push("ui", lambda p=bin_path: self.bin_var.set(p))
                        self._push("log", f"[构建] ✅ 成功，固件: {bin_path}\n")
                    else:
                        self._push("log", "[构建] ✅ 成功，但未找到 app bin\n")
                else:
                    self._push("log", f"[构建] ❌ 失败 (code={proc.returncode})\n")
            except Exception as e:
                self._push("log", f"[构建] 异常: {e}\n")
            finally:
                self.building = False
                self._push("ui", lambda: self.build_btn.configure(state="normal"))

        threading.Thread(target=run, daemon=True).start()

    def _browse_bin(self):
        initial = self._ota_default_dir or TOOLS_DIR
        path = filedialog.askopenfilename(
            initialdir=initial, filetypes=[("firmware", "*.bin"), ("all", "*.*")])
        if path:
            self.bin_var.set(path)
            self.cfg["ota_bin"] = path
            self.cfg["ota_dir"] = os.path.dirname(path)
            save_config(self.cfg)

    def _validate_firmware(self, path):
        if not os.path.isfile(path):
            messagebox.showerror("OTA", f"固件不存在:\n{path}")
            return None
        size = os.path.getsize(path)
        try:
            with open(path, "rb") as f:
                head = f.read(0x30)
        except Exception as e:
            messagebox.showerror("OTA", str(e))
            return None
        if not head or head[0] != 0xE9:
            messagebox.showerror(
                "OTA", "所选文件不是 ESP 应用镜像（首字节应为 0xE9）。\n\n"
                       "请选择 build 目录下的 app bin（如 DIpances.bin），"
                       "不要选 bootloader / partition-table / 合并包。")
            return None
        # app 镜像在 0x20 处有 esp_app_desc_t，magic = 0xABCD5432。
        # bootloader.bin 首字节同样是 0xE9，靠这一步才能区分。
        if len(head) >= 0x24:
            magic = int.from_bytes(head[0x20:0x24], "little")
            if magic != 0xABCD5432:
                messagebox.showerror(
                    "OTA", "这个 bin 不是应用固件（缺少 app 描述符）：\n"
                           f"{os.path.basename(path)}\n\n"
                           "无线升级只能写 app 分区，不能更新 bootloader / 分区表。\n"
                           "需要更新这两者请用「固件烧录」页（支持多文件、固定偏移）。")
                return None
        return size

    # ---------------- HTTP OTA ----------------
    def on_http_request(self, path, msg):
        self._push("log", f"[HTTP] {msg}\n")
        bin_name = self.ota_current_bin
        if " 200 " in msg and bin_name and path.endswith("/" + bin_name):
            if not self.ota_http_event.is_set():
                self.ota_http_event.set()
                self._push("log", "[HTTP] P4 已开始下载固件\n")

    def on_ota_chunk(self, n):
        if self.ota_total <= 0:
            return
        self.ota_sent += n
        self._push("prog", ("ota", self.ota_sent / self.ota_total * 100))

    def _http_host(self):
        host = self.http_host_var.get().strip()
        if host:
            return host
        ips = self._local_ipv4_list()
        return ips[0] if ips else "127.0.0.1"

    def _http_port(self):
        try:
            return int(self.http_port_var.get())
        except ValueError:
            return HTTP_PORT

    def _start_http(self, directory, port=None):
        port = port or self._http_port()
        try:
            OTAHandler.app = self
            handler = functools.partial(OTAHandler, directory=directory)
            self.http_server = ThreadingHTTP(("0.0.0.0", port), handler)
        except Exception as e:
            return f"HTTP 服务器启动失败: {e}（端口被占用？）"
        self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()
        return None

    def _stop_http(self):
        if self.http_server:
            try:
                self.http_server.shutdown()
                self.http_server.server_close()
            except Exception:
                pass
            self.http_server = None

    def _http_selfcheck(self):
        path = self.bin_var.get().strip()
        if not path or not os.path.isfile(path):
            messagebox.showwarning("HTTP", "请先选择固件")
            return
        err = self._start_http(os.path.dirname(path))
        if err:
            messagebox.showerror("HTTP", err)
            return
        url = f"http://{self._http_host()}:{self._http_port()}/{os.path.basename(path)}"
        self._append_log(f"[HTTP] 自检 {url}\n")

        def run():
            try:
                import urllib.request
                with urllib.request.urlopen(url, timeout=5) as r:
                    n = len(r.read())
                self._append_log(f"[HTTP] ✅ 自检通过，读取 {n} 字节\n")
            except Exception as e:
                self._append_log(f"[HTTP] ❌ 自检失败: {e}\n"
                                 "[HTTP] 检查电脑防火墙是否放行该端口\n")
            finally:
                self._stop_http()

        threading.Thread(target=run, daemon=True).start()

    def _ota_http_worker(self, bin_path, ip, fast_fail=False, host=None, port=None):
        """HTTP 方式下载。返回 True=成功, False=失败, "blocked"=设备根本没来下载
        （多半是电脑防火墙拦了入站连接，可自动回退到 TCP 推送）。"""
        serve_dir = os.path.dirname(bin_path)
        bin_name = os.path.basename(bin_path)
        # host/port 必须由主线程传入：Tk 变量不能在后台线程读取
        host = host or "127.0.0.1"
        port = port or HTTP_PORT
        url = f"http://{host}:{port}/{bin_name}"

        err = self._start_http(serve_dir, port)
        if err:
            self._push("log", f"[OTA] {err}\n")
            return False
        self._push("log", f"[OTA] HTTP 就绪: {url}\n")

        self.ota_current_bin = bin_name
        self.ota_http_event.clear()
        self.ota_serial_event.clear()
        self.ota_total = os.path.getsize(bin_path)
        self.ota_sent = 0

        ok = self._tcp_send_sync(ip, f"ota {url}\n", timeout=6)
        if not ok:
            self._push("log", f"[OTA] ❌ 触发失败：无法连接 {ip}:{TCP_PORT}\n")
            self._stop_http()
            return False
        self._push("log", "[OTA] 已触发，等待设备下载...\n")

        # fast_fail：设备 25s 还没开始下载，几乎必然是入站被拦
        wait_s = 25 if fast_fail else 180
        download = self.ota_http_event.wait(wait_s)
        if not download:
            self._push("log", "[OTA] ❌ 设备没有发起下载（电脑防火墙拦住 8000 端口？）\n")
            self._stop_http()
            return "blocked" if fast_fail else False

        self._push("log", "[OTA] 下载完成，等待设备校验并重启...\n")
        success = self.ota_serial_event.wait(120)
        if not success:
            self._push("log", "[OTA] 未收到串口成功标志，通过连通性判断...\n")
            success = self._wait_reboot(ip)
        self._stop_http()
        return success

    def _add_firewall_rule(self):
        """尝试为 HTTP OTA 端口添加 Windows 入站放行规则。"""
        port = self._http_port()
        cmd = ["netsh", "advfirewall", "firewall", "add", "rule",
               f"name=P4Tool OTA {port}", "dir=in", "action=allow",
               "protocol=TCP", f"localport={port}"]
        self._append_log("[防火墙] " + " ".join(cmd) + "\n")
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=20,
                               creationflags=CREATE_NO_WINDOW)
            if r.returncode == 0:
                self._append_log(f"[防火墙] ✅ 已放行 TCP {port}\n")
            else:
                self._append_log("[防火墙] ❌ 需要管理员权限。请用管理员身份运行本工具，"
                                 f"或在管理员 CMD 里手动执行上面的命令\n{r.stdout}{r.stderr}\n")
        except Exception as e:
            self._append_log(f"[防火墙] ❌ {e}\n")

    # ---------------- TCP 推送 OTA ----------------
    def _recv_line(self, sock, timeout):
        sock.settimeout(timeout)
        data = b""
        while True:
            try:
                ch = sock.recv(1)
            except socket.timeout:
                return data, False
            if not ch:
                return data, False
            data += ch
            if ch in (b"\n", b"\r"):
                return data.strip(), True

    def _wait_for_token(self, sock, prefixes, timeout):
        """读取到指定前缀的应答为止；中间夹带的日志行只做展示，不算应答。
        设备侧的日志镜像可能先于 READY 到达，必须跳过，否则会误判失败。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            line, ok = self._recv_line(sock, max(0.5, deadline - time.time()))
            text = line.decode("utf-8", errors="replace").strip()
            if not text:
                if not ok:
                    return ""
                continue
            for pre in prefixes:
                if text.upper().startswith(pre.upper()):
                    return text
            self._push("net_rx", text + "\n")
        return ""

    def _tcp_send_sync(self, ip, text, timeout=5, retries=3):
        for attempt in range(retries):
            try:
                s = socket.create_connection((ip, TCP_PORT), timeout=timeout)
                s.sendall(text.encode("utf-8"))
                s.close()
                return True
            except Exception:
                if attempt + 1 < retries:
                    time.sleep(1.5)
        return False

    def _preflight_device(self, ip):
        """OTA 前先确认设备在线并判断固件新旧。
        旧固件（p4_host 那版）只允许一个 TCP 客户端，且不认 ota-push，
        这里会明确提示，避免用户一直卡在“无线烧录失败”。"""
        try:
            s = socket.create_connection((ip, TCP_PORT), timeout=5)
        except Exception as e:
            self._push("log", f"[检查] ❌ 无法连接 {ip}:{TCP_PORT} ({e})。"
                              "确认与 P4 在同一网络、IP 正确\n")
            return "none"
        text = ""
        try:
            s.settimeout(2.0)
            s.sendall(b"hello p4\n")
            data = b""
            deadline = time.time() + 2.0
            # 固件回包以 "\r\n[P4] ..." 开头，不能一见到换行就停
            while time.time() < deadline and len(data) < 256:
                try:
                    chunk = s.recv(128)
                except socket.timeout:
                    break
                if not chunk:
                    break
                data += chunk
                if b"\n" in data.strip() and len(data) > 4:
                    break
            text = data.decode("utf-8", errors="replace").strip()
        except Exception:
            pass
        finally:
            try:
                s.close()
            except Exception:
                pass

        if "[P4]" in text:
            self._push("log", f"[检查] ✅ 设备在线（新固件）: {text.splitlines()[0][:90]}\n")
            return "new"
        if text:
            self._push("log", f"[检查] ⚠️ 设备回复 \"{text[:48]}\"，这只是原样回显 —— "
                              "说明板子里还是旧固件（不支持 ota-push / hello / status）。\n"
                              "[检查] 请先用 USB 烧录一次本项目固件（build/DIpances.bin），"
                              "之后无线升级才可用。\n")
            return "old"
        self._push("log", "[检查] ⚠️ 设备无应答（可能被其他客户端占用）\n")
        return "unknown"

    def _ota_push_worker(self, bin_path, ip):
        size = os.path.getsize(bin_path)
        self._push("log", f"[推送] 连接 {ip}:{TCP_PORT} ，固件 {size} 字节\n")
        try:
            sock = socket.create_connection((ip, TCP_PORT), timeout=8)
        except Exception as e:
            self._push("log", f"[推送] ❌ 无法连接: {e}\n")
            return False

        try:
            sock.sendall(f"ota-push {size}\n".encode())
            reply = self._wait_for_token(sock, ("READY", "ERR"), 10)
            if not reply:
                self._push("log", "[推送] ❌ 设备无应答。若固件较旧（不识别 ota-push），"
                                  "请先用 USB 烧录一次新固件，或改用 HTTP 方式\n")
                return False
            if not reply.upper().startswith("READY"):
                self._push("log", f"[推送] ❌ 设备拒绝: {reply}\n")
                return False
            self._push("log", "[推送] 设备就绪，开始传输...\n")

            sock.settimeout(30)
            sent = 0
            with open(bin_path, "rb") as f:
                while True:
                    chunk = f.read(8192)
                    if not chunk:
                        break
                    sock.sendall(chunk)
                    sent += len(chunk)
                    if sent % (64 * 1024) < 8192 or sent == size:
                        self._push("prog", ("ota", sent / size * 100))
            self._push("prog", ("ota", 100.0))
            self._push("log", f"[推送] 传输完成 {sent}/{size} 字节，等待设备校验...\n")

            reply = self._wait_for_token(sock, ("OK", "ERR"), 60)
            if reply.upper().startswith("OK"):
                self._push("log", f"[推送] ✅ 设备校验通过: {reply}\n")
                return True
            self._push("log", f"[推送] ❌ 设备返回: {reply or '(无响应)'}\n")
            return False
        except Exception as e:
            self._push("log", f"[推送] ❌ 传输异常: {e}\n")
            return False
        finally:
            try:
                sock.close()
            except Exception:
                pass

    def _wait_reboot(self, ip, timeout=90):
        deadline = time.time() + timeout
        saw_down = False
        while time.time() < deadline:
            if self.ota_serial_event.is_set():
                return True
            up = self._ping_once(ip, 800)
            if not up:
                saw_down = True
            elif saw_down:
                return True
            time.sleep(1.5)
        return False

    # ---------------- OTA 入口 ----------------
    def _ota_start(self):
        if self.ota_running:
            return
        bin_path = self.bin_var.get().strip()
        if not bin_path:
            messagebox.showwarning("OTA", "请先选择固件（可先点「构建固件」）")
            return
        if self._validate_firmware(bin_path) is None:
            return
        ip = self.ip_var.get().strip()
        if not ip:
            messagebox.showwarning("OTA", "请先填写或自动识别 P4 IP")
            return

        mode = self.ota_mode_var.get()
        self.ota_running = True
        self.ota_ok = False
        self.ota_btn.configure(state="disabled")
        self._set_progress("ota", 0)
        self._append_log(f"[OTA] 方式={mode} 固件={bin_path}\n")

        # 旧固件只支持一个 TCP 客户端：无线通道常驻会占住唯一的连接，
        # 导致 OTA 触发连接被拒。这里先让出连接，结束后自动重连。
        was_net_alive = self.net_alive
        if was_net_alive:
            self._append_log("[OTA] 临时断开无线接收通道（让出设备的唯一 TCP 连接）\n")
            self._close_net()

        # 所有 Tk 变量在主线程取值，后台线程只用普通 Python 变量
        http_host = self._http_host()
        http_port = self._http_port()

        def run():
            try:
                state = self._preflight_device(ip)
                if state == "none":
                    self._append_log("[OTA] ❌ 设备不可达，升级终止\n")
                    self.ota_ok = False
                    return
                if state == "old" and mode == "push":
                    self._append_log("[OTA] ❌ 设备固件过旧，不支持 TCP 推送。"
                                     "请改用 HTTP 方式，或先 USB 烧录新固件\n")
                    self.ota_ok = False
                    return

                if mode == "http":
                    result = self._ota_http_worker(bin_path, ip,
                                                   fast_fail=(state == "new"),
                                                   host=http_host, port=http_port)
                    if result == "blocked" and state == "new":
                        self._append_log("[OTA] HTTP 入站被拦，自动改用 TCP 推送重试"
                                         "（不用开任何端口）\n")
                        self._push("prog", ("ota", 0))
                        result = self._ota_push_worker(bin_path, ip)
                    ok = (result is True)
                else:
                    result = self._ota_push_worker(bin_path, ip)
                    ok = (result is True)
                    if not ok and state == "new":
                        self._append_log("[OTA] 提示：也可改用 HTTP 方式"
                                         "（需先点「防火墙放行」）\n")
                if ok:
                    self._append_log("[OTA] 等待设备重启...\n")
                    self._wait_reboot(ip)
                    self._append_log("[OTA] ✅ 升级成功\n")
                else:
                    self._append_log("[OTA] ❌ 升级失败（上方日志有具体原因）\n")
                self.ota_ok = ok
                self._push("ui", lambda: messagebox.showinfo(
                    "OTA", "✅ 升级成功" if ok else "❌ 升级失败，详见日志"))
            finally:
                self.ota_running = False
                self._push("ui", lambda: self.ota_btn.configure(state="normal"))
                if was_net_alive:
                    self._append_log("[OTA] 恢复无线接收通道...\n")
                    self._push("ui", lambda: self.root.after(1500, self._try_reconnect_net))

        threading.Thread(target=run, daemon=True).start()

    # ---------------- 烧录 ----------------
    def _load_flash_preset(self, chip=None):
        chip = chip or self.flash_chip_var.get() or "esp32p4"
        self.flash_chip_var.set(chip)
        for item in self.flash_tree.get_children():
            self.flash_tree.delete(item)
        if chip == "esp32c5":
            base = self.c5_var.get().strip()
            files = [
                ("0x2000", os.path.join(base, "bootloader.bin")),
                ("0x8000", os.path.join(base, "partition-table.bin")),
                ("0xd000", os.path.join(base, "ota_data_initial.bin")),
                ("0x10000", os.path.join(base, "slave.bin")),
            ]
        else:
            proj = self.proj_var.get().strip()
            build = os.path.join(proj, "build")
            files = [
                ("0x2000", os.path.join(build, "bootloader", "bootloader.bin")),
                ("0x8000", os.path.join(build, "partition_table", "partition-table.bin")),
                ("0xd000", os.path.join(build, "ota_data_initial.bin")),
            ]
            app = self._find_app_bin(proj)
            if app:
                files.append(("0x10000", app))
        missing = []
        for off, path in files:
            if os.path.isfile(path):
                self.flash_tree.insert("", "end", values=(off, path))
            else:
                missing.append(path)
        if missing:
            self._append_log("[烧录] 缺少文件（请先构建/确认目录）:\n" +
                             "\n".join(f"  {m}" for m in missing) + "\n")

    def _flash_add(self):
        paths = filedialog.askopenfilenames(filetypes=[("bin", "*.bin"), ("all", "*.*")])
        for p in paths:
            off = simpledialog.askstring("Offset", f"{os.path.basename(p)} 的偏移地址:",
                                         initialvalue="0x10000")
            if off:
                self.flash_tree.insert("", "end", values=(off.strip(), p))

    def _flash_del(self):
        for item in self.flash_tree.selection():
            self.flash_tree.delete(item)

    def _flash_clear(self):
        for item in self.flash_tree.get_children():
            self.flash_tree.delete(item)

    def _flash_start(self):
        if self.flashing:
            return
        items = self.flash_tree.get_children()
        if not items:
            messagebox.showwarning("Flash", "请先载入或添加要烧录的文件")
            return
        port = self._port_name(self.flash_port_var)
        if not port:
            messagebox.showwarning("Flash", "请选择串口")
            return
        if not os.path.isfile(ESPTOOL):
            messagebox.showerror("Flash", f"找不到 esptool:\n{ESPTOOL}")
            return

        if self.serial_open:
            self._append_log("[烧录] 自动断开串口监视以释放串口\n")
            self._close_serial()

        chip = self.flash_chip_var.get()
        preset = CHIP_PRESETS.get(chip, {"chip": chip, "flash_size": "4MB",
                                         "flash_mode": "dio", "flash_freq": "80m"})
        baud = self.flash_baud_var.get().strip() or "460800"
        args = [ESPTOOL, "--chip", preset["chip"], "-p", port, "-b", baud,
                "--before", "default_reset", "--after", "hard_reset",
                "--connect-attempts", "50", "write_flash",
                "--flash_mode", preset["flash_mode"],
                "--flash_freq", preset["flash_freq"],
                "--flash_size", preset["flash_size"]]
        for item in items:
            off, path = self.flash_tree.item(item, "values")
            args += [off, path]

        if preset["chip"] == "esp32c5":
            if not messagebox.askyesno(
                    "C5 烧录", f"即将烧录 C5 到 {port}。\n\n"
                              "请按住 BOOT 键，点「是」开始，然后点一下 RST。\n"
                              "看到进度条后再松开 BOOT。"):
                return

        self.flashing = True
        self.flash_ok = False
        self.flash_btn.configure(state="disabled")
        self._set_progress("flash", 0)
        self._append_log(f"[烧录] {preset['chip']} -> {port} @ {baud}\n")
        write_re = re.compile(r"\((\d+)\s*%\)")

        def run():
            try:
                proc = subprocess.Popen(args, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True,
                                        encoding="utf-8", errors="replace",
                                        creationflags=CREATE_NO_WINDOW)
                for line in proc.stdout:
                    line = strip_ansi(line.rstrip("\n"))
                    if line.strip():
                        self._push("log", line + "\n")
                        m = write_re.search(line)
                        if m:
                            self._push("prog", ("flash", float(m.group(1))))
                proc.wait()
                self.flash_ok = (proc.returncode == 0)
                self._push("prog", ("flash", 100.0 if self.flash_ok else 0.0))
                self._push("log", "[烧录] ✅ 成功\n" if self.flash_ok
                           else f"[烧录] ❌ 失败 (code={proc.returncode})\n")
            except Exception as e:
                self._push("log", f"[烧录] 异常: {e}\n")
            finally:
                self.flashing = False
                self._push("ui", lambda: self.flash_btn.configure(state="normal"))

        threading.Thread(target=run, daemon=True).start()

    # ---------------- 一键流程 ----------------
    def _run_pipeline(self):
        if self.pipeline_running:
            return
        self.pipeline_running = True
        self.pipeline_btn.configure(state="disabled")
        self._append_log("\n========== 一键流程开始 ==========\n")
        do_flash = self.pipeline_flash_var.get()
        # 主线程先取值，避免后台线程访问 Tk 变量
        p_ssid = self.wifi_ssid_var.get().strip()
        p_pass = self.wifi_pass_var.get()
        p_proj = self.proj_var.get().strip()
        p_chip = self.flash_chip_var.get()

        def wait_flag(getter, timeout):
            t0 = time.time()
            while time.time() - t0 < timeout:
                if not getter[0]():
                    return True
                time.sleep(0.3)
            return False

        def run():
            ok = True
            try:
                # 1) 保存 WiFi
                if not self._save_wifi_values(p_ssid, p_pass, p_proj, silent=True):
                    self._push("log", "[流程] ⚠️ WiFi 配置未写入，继续\n")

                # 2) 烧录（可选，固定预设）
                if do_flash:
                    self._append_log("[流程] 载入烧录预设并烧录...\n")
                    self._push("ui", lambda c=p_chip: self._load_flash_preset(c))
                    time.sleep(0.5)
                    self._push("ui", self._flash_start)
                    time.sleep(0.5)
                    wait_flag((lambda: self.flashing,), 300)
                    if not self.flash_ok:
                        self._push("log", "[流程] ❌ 烧录失败，终止\n")
                        ok = False
                        return

                # 3) 构建
                self._push("ui", self._build)
                time.sleep(0.5)
                wait_flag((lambda: self.building,), 900)
                if not self.build_ok:
                    self._push("log", "[流程] ❌ 构建失败，终止\n")
                    ok = False
                    return

                # 4) 等 IP：先试上次的 IP / 串口日志，再扫网段
                ip = ""
                cands = []
                last_ip = self.cfg.get("ip", "")
                if last_ip:
                    cands.append(last_ip)
                deadline = time.time() + 60
                while time.time() < deadline and not ip:
                    cand = self._ip_from_text(self.serial_rx_cache)
                    if cand and cand not in cands:
                        cands.insert(0, cand)
                    for c in cands:
                        if self._ping_once(c, 800):
                            ip = c
                            break
                    if not ip:
                        time.sleep(2)
                if ip:
                    self._set_ip(ip, "流程")
                else:
                    self._push("log", "[流程] ⚠️ 未自动获取 IP，尝试局域网扫描...\n")
                    ip = self._scan_subnet() or ""
                    if ip:
                        self._set_ip(ip, "局域网扫描")
                if not ip:
                    self._push("log", "[流程] ❌ 找不到设备 IP，终止\n")
                    ok = False
                    return

                # 5) OTA
                time.sleep(1.0)
                self._push("ui", self._ota_start)
                time.sleep(0.5)
                wait_flag((lambda: self.ota_running,), 400)
                ok = self.ota_ok
            finally:
                self._append_log("========== 一键流程结束: %s ==========\n"
                                 % ("成功 ✅" if ok else "失败 ❌"))
                self.pipeline_running = False
                self._push("ui", lambda: self.pipeline_btn.configure(state="normal"))

        threading.Thread(target=run, daemon=True).start()

    # ---------------- 关闭 ----------------
    def _on_close(self):
        self.cfg.update({
            "language": self.lang,
            "serial_port": self._port_name(self.port_var),
            "serial_baud": self.baud_var.get(),
            "flash_port": self._port_name(self.flash_port_var),
            "flash_baud": self.flash_baud_var.get(),
            "flash_chip": self.flash_chip_var.get(),
            "ip": self.ip_var.get().strip(),
            "http_host": self.http_host_var.get().strip(),
            "http_port": self.http_port_var.get().strip(),
            "ota_mode": self.ota_mode_var.get(),
            "ota_bin": self.bin_var.get().strip(),
            "ota_dir": os.path.dirname(self.bin_var.get().strip())
                        if self.bin_var.get().strip() else "",
            "p4_project": self.proj_var.get().strip(),
            "c5_prebuilt": self.c5_var.get().strip(),
            "wifi_ssid": self.wifi_ssid_var.get().strip(),
            "wifi_password": self.wifi_pass_var.get(),
            "auto_on_start": self.auto_start_var.get(),
            "auto_reconnect": self.auto_reconn_var.get(),
            "rx_timestamp": self.ts_var.get(),
            "show_serial": self.show_serial_var.get(),
            "show_net": self.show_net_var.get(),
        })
        save_config(self.cfg)
        self._close_serial()
        self._close_net()
        self._stop_http()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        root.call("tk", "scaling", 1.2)
    except Exception:
        pass
    P4ToolApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()

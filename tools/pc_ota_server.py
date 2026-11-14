# -*- coding: utf-8 -*-
"""
电脑端 OTA 一键脚本：
  1) 在本机启动一个 HTTP 文件服务器，提供编译好的固件 p4_host_ota.bin
  2) 通过 TCP 向 P4 发送 "ota <url>" 命令，触发 P4 无线下载固件并升级

用法：
    python pc_ota_server.py <P4的IP> [--bin 固件路径] [--http-port 8000] [--tcp-port 3333]

示例：
    python pc_ota_server.py 10.251.210.193
    python pc_ota_server.py 10.251.210.193 --bin build/p4_host_ota.bin
"""
import argparse
import functools
import http.server
import os
import socket
import socketserver
import sys
import threading
import time


def get_local_ip(target_ip):
    """获取本机用来访问 target_ip 的那块网卡的 IP（和 P4 同网段的真实地址）。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((target_ip, 80))
        return s.getsockname()[0]
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("p4_ip", help="P4 的 IP，从串口日志 got ip 那行读取")
    ap.add_argument("--bin", default=os.path.join("build", "p4_host_ota.bin"),
                    help="要烧录的固件路径，默认 build/p4_host_ota.bin")
    ap.add_argument("--tcp-port", type=int, default=3333)
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--wait", type=int, default=120,
                    help="OTA 触发后 HTTP 服务器保持运行的秒数")
    args = ap.parse_args()

    bin_path = os.path.abspath(args.bin)
    if not os.path.isfile(bin_path):
        print(f"[ERROR] 找不到固件: {bin_path}")
        sys.exit(1)

    serve_dir = os.path.dirname(bin_path)
    bin_name = os.path.basename(bin_path)
    local_ip = get_local_ip(args.p4_ip)

    # 1) 启动 HTTP 文件服务器
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=serve_dir)
    httpd = socketserver.TCPServer(("0.0.0.0", args.http_port), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    url = f"http://{local_ip}:{args.http_port}/{bin_name}"
    print(f"[INFO] 正在提供固件: {bin_path}")
    print(f"[INFO] OTA URL: {url}")

    # 2) 通过 TCP 触发 P4 的 OTA
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)
        s.connect((args.p4_ip, args.tcp_port))
        s.sendall(f"ota {url}\n".encode("utf-8"))
        try:
            print("[INFO] P4 回复:", s.recv(1024).decode("utf-8", errors="replace").strip())
        except socket.timeout:
            print("[INFO] P4 未回复（正常，它可能已开始下载）")
        s.close()
    except Exception as e:
        print(f"[ERROR] 无法连接 P4 {args.p4_ip}:{args.tcp_port}: {e}")
        httpd.shutdown()
        httpd.server_close()
        sys.exit(1)

    print(f"[INFO] OTA 已触发，HTTP 服务器继续运行 {args.wait} 秒")
    print("[INFO] 请观察 P4 串口日志，等待 'OTA succeeded, rebooting...'")

    try:
        time.sleep(args.wait)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        httpd.server_close()
        print("[INFO] HTTP 服务器已关闭")


if __name__ == "__main__":
    main()

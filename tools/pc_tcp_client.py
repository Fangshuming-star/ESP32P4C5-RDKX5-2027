# -*- coding: utf-8 -*-
"""
电脑端 TCP 测试客户端：连接 ESP32-P4 上的回显服务器。
用法：
    python pc_tcp_client.py <P4的IP> [端口] [要发送的消息]

示例：
    python pc_tcp_client.py 192.168.1.100
    python pc_tcp_client.py 192.168.1.100 3333 "hello p4"
"""
import socket
import sys


def main():
    if len(sys.argv) < 2:
        print("用法: python pc_tcp_client.py <P4的IP> [端口] [消息]")
        sys.exit(1)

    ip = sys.argv[1]
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 3333
    message = sys.argv[3] if len(sys.argv) > 3 else "hello p4"

    print(f"连接 {ip}:{port} ...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((ip, port))

    s.sendall(message.encode("utf-8"))
    data = s.recv(1024)
    print(f"收到回显 ({len(data)} 字节): {data.decode('utf-8', errors='replace')}")

    s.close()
    print("连接已关闭")


if __name__ == "__main__":
    main()

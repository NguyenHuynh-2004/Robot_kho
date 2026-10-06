#!/usr/bin/env python3
"""Xem so lieu BNO055 qua Wi-Fi (sketch firmware/bno055_test) va go lenh.

    python3 tools/bno055_wifi.py [IP_XE] [CONG]      # mac dinh 192.168.1.150 2323

Go mot chu roi Enter: h p z d r m c (xem huong dan tren xe). Ctrl+C de thoat.
Mat ket noi thi tu ket noi lai.
"""
import socket
import sys
import threading
import time

host = sys.argv[1] if len(sys.argv) > 1 else '192.168.1.150'
port = int(sys.argv[2]) if len(sys.argv) > 2 else 2323
sock = None


def reader():
    global sock
    while True:
        try:
            s = socket.create_connection((host, port), timeout=3)
            s.settimeout(None)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock = s
            print(f'--- da ket noi {host}:{port} ---', flush=True)
            while True:
                data = s.recv(4096)
                if not data:
                    break
                sys.stdout.write(data.decode('utf-8', 'replace'))
                sys.stdout.flush()
        except OSError as e:
            print(f'--- chua ket noi duoc {host}:{port} ({e}), thu lai... ---', flush=True)
        sock = None
        time.sleep(2)


threading.Thread(target=reader, daemon=True).start()
try:
    for line in sys.stdin:
        cmd = line.strip()[:1]
        if not cmd:
            continue
        if sock is None:
            print('--- chua ket noi, lenh bi bo qua ---', flush=True)
            continue
        try:
            sock.sendall(cmd.encode())
        except OSError:
            print('--- gui lenh loi ---', flush=True)
except KeyboardInterrupt:
    pass

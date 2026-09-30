#!/usr/bin/env python3
"""Doc tay cam tren WINDOWS -> gui UDP sang WSL.

Chay tren Windows, khong phai trong WSL:
    python tools\\windows_gamepad_bridge.py

Tai sao phai lam vong nay: kernel WSL2 cua Microsoft khong bat
CONFIG_INPUT_JOYSTICK, nen KHONG co driver joystick nao ca (khong xpad, khong
joydev) va cung khong co duoi dang module. Forward USB bang usbipd van vao
duoc WSL nhung khong driver nao nhan, /dev/input/event* khong xuat hien, va
goi joy cua ROS thay con so khong. Cach duy nhat khong phai build lai kernel
la doc tay cam o phia Windows roi day du lieu qua.

Chi dung thu vien chuan cua Python (ctypes + socket), khong can pip install gi.
Doc qua XInput nen dung cho tay cam Xbox va moi tay cam chay che do XInput.

Phia WSL: chay tools/joy_udp_bridge.py de nhan va phat ra topic /joy.
"""

import ctypes
import json
import socket
import sys
import time

# 127.0.0.1 chay duoc vi .wslconfig dat networkingMode=mirrored.
# Neu ban doi sang NAT thi thay bang IP cua WSL (lay bang: wsl hostname -I).
DEST_HOST = "127.0.0.1"
DEST_PORT = 9002
SEND_HZ = 50.0

# ---------------------------------------------------------------- XInput
class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ("wButtons", ctypes.c_ushort),
        ("bLeftTrigger", ctypes.c_ubyte),
        ("bRightTrigger", ctypes.c_ubyte),
        ("sThumbLX", ctypes.c_short),
        ("sThumbLY", ctypes.c_short),
        ("sThumbRX", ctypes.c_short),
        ("sThumbRY", ctypes.c_short),
    ]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [("dwPacketNumber", ctypes.c_ulong), ("Gamepad", XINPUT_GAMEPAD)]


def load_xinput():
    for name in ("XInput1_4.dll", "XInput1_3.dll", "XInput9_1_0.dll"):
        try:
            return ctypes.windll.LoadLibrary(name), name
        except OSError:
            continue
    return None, None


# Mat na nut cua XInput
BTN = {
    "DPAD_UP": 0x0001, "DPAD_DOWN": 0x0002, "DPAD_LEFT": 0x0004, "DPAD_RIGHT": 0x0008,
    "START": 0x0010, "BACK": 0x0020, "LTHUMB": 0x0040, "RTHUMB": 0x0080,
    "LB": 0x0100, "RB": 0x0200,
    "A": 0x1000, "B": 0x2000, "X": 0x4000, "Y": 0x8000,
}

# Thu tu nut sap xep GIONG SDL2/Xbox, de config joy_teleop.yaml dung nguyen
# chi so quen thuoc: buttons[4] = LB (nut chet), buttons[5] = RB (turbo).
BUTTON_ORDER = ["A", "B", "X", "Y", "LB", "RB", "BACK", "START",
                "LTHUMB", "RTHUMB", "DPAD_UP", "DPAD_DOWN", "DPAD_LEFT", "DPAD_RIGHT"]


def axis(value):
    """Chuan hoa truc -32768..32767 thanh -1.0..1.0."""
    return max(-1.0, value / 32767.0)


def main():
    dll, dll_name = load_xinput()
    if dll is None:
        sys.exit("LOI: khong nap duoc XInput DLL. May nay co phai Windows khong?")
    print(f"Da nap {dll_name}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state = XINPUT_STATE()
    period = 1.0 / SEND_HZ

    pad_index = None
    last_report = 0.0
    sent = 0

    print(f"Gui UDP toi {DEST_HOST}:{DEST_PORT} o {SEND_HZ:.0f} Hz. Ctrl-C de dung.\n")

    while True:
        loop_start = time.perf_counter()

        # Tim tay cam dang cam. Lam lai moi vong de rut/cam lai van chay tiep.
        if pad_index is None or dll.XInputGetState(pad_index, ctypes.byref(state)) != 0:
            pad_index = None
            for i in range(4):
                if dll.XInputGetState(i, ctypes.byref(state)) == 0:
                    pad_index = i
                    print(f"Thay tay cam o khe {i}")
                    break

        if pad_index is None:
            # Khong co tay cam -> van gui goi rong. Node ben WSL se phat /joy
            # toan 0, teleop giu xe dung. Im lang la nguy hiem hon.
            payload = {"axes": [0.0] * 6, "buttons": [0] * len(BUTTON_ORDER), "pad": -1}
        else:
            g = state.Gamepad
            # ROS quy uoc: truc ngang duong la sang TRAI, con XInput duong la
            # sang PHAI -> phai doi dau hai truc ngang.
            payload = {
                "axes": [
                    -axis(g.sThumbLX),          # 0 can trai ngang  (re)
                    axis(g.sThumbLY),           # 1 can trai doc    (tien/lui)
                    g.bLeftTrigger / 255.0,     # 2 cò trai
                    -axis(g.sThumbRX),          # 3 can phai ngang
                    axis(g.sThumbRY),           # 4 can phai doc
                    g.bRightTrigger / 255.0,    # 5 cò phai
                ],
                "buttons": [1 if (g.wButtons & BTN[n]) else 0 for n in BUTTON_ORDER],
                "pad": pad_index,
            }

        try:
            sock.sendto(json.dumps(payload).encode("ascii"), (DEST_HOST, DEST_PORT))
            sent += 1
        except OSError as e:
            print(f"Loi gui UDP: {e}")

        now = time.perf_counter()
        if now - last_report > 2.0:
            last_report = now
            if pad_index is None:
                print("Chua thay tay cam nao (van gui goi rong de xe dung)")
            else:
                a = payload["axes"]
                print(f"goi da gui: {sent:6d} | re={a[0]:+.2f} tien={a[1]:+.2f} "
                      f"| LB={payload['buttons'][4]} RB={payload['buttons'][5]}")

        sleep_left = period - (time.perf_counter() - loop_start)
        if sleep_left > 0:
            time.sleep(sleep_left)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDung.")

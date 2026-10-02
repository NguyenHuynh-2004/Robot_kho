#!/bin/bash
# Sua dong ho WSL chay sai toc do. Chay MOT lan (can mat khau):
#
#     sudo bash ~/robot_ws_3/tools/fix_wsl_clock.sh
#
# DA DO TREN MAY NAY: dong ho Ubuntu chay CHAM 8.3% (kernel tick = 9167 thay
# vi 10000), cu ~35 s lai bi Hyper-V giat len ~2.4 s. Hau qua voi xe:
#   - moi dau thoi gian tren laptop lech toi +-1.2 s, nhay 2.4 s
#   - joy_udp_bridge tuong mat tay cam moi lan nhay -> phat lenh dung
#   - ESP32 dong bo dong ho luc dung yen mat 1.2-1.7 s moi phut
#
# NGUYEN NHAN: systemd-timesyncd (dong bo gio qua NTP) chay trong WSL2, trong
# khi Hyper-V cung tu dong bo gio cho may ao. Hyper-V giat dong ho len, timesyncd
# tuong dong ho chay nhanh nen ham lai bang tick; dong ho cang cham, Hyper-V
# cang giat - hai ben cai nhau mai. WSL2 khong can timesyncd.
#
# LAM GI: tat timesyncd (ca lan khoi dong sau), dua tick/freq cua kernel ve
# chuan. Khong dong vao gio Windows.
set -e
if [ "$(id -u)" != 0 ]; then
  echo "Can chay voi sudo:  sudo bash $0"; exit 1
fi

show() {
python3 - <<'PY'
import ctypes, ctypes.util
class timex(ctypes.Structure):
    _fields_ = [("modes", ctypes.c_uint), ("offset", ctypes.c_long), ("freq", ctypes.c_long),
                ("maxerror", ctypes.c_long), ("esterror", ctypes.c_long), ("status", ctypes.c_int),
                ("constant", ctypes.c_long), ("precision", ctypes.c_long), ("tolerance", ctypes.c_long),
                ("time_sec", ctypes.c_long), ("time_usec", ctypes.c_long), ("tick", ctypes.c_long),
                ("ppsfreq", ctypes.c_long), ("jitter", ctypes.c_long), ("shift", ctypes.c_int),
                ("stabil", ctypes.c_long), ("jitcnt", ctypes.c_long), ("calcnt", ctypes.c_long),
                ("errcnt", ctypes.c_long), ("stbcnt", ctypes.c_long), ("tai", ctypes.c_int),
                ("pad", ctypes.c_int * 11)]
libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
t = timex(); t.modes = 0
libc.adjtimex(ctypes.byref(t))
err = (t.tick - 10000) / 100 + t.freq / 65536 / 1e4
print(f"  tick={t.tick} (chuan 10000), freq={t.freq/65536:+.1f} ppm -> dong ho lech {err:+.2f} %")
PY
}

echo "Truoc:"; show

echo "Tat systemd-timesyncd (ca lan khoi dong sau)..."
timedatectl set-ntp false 2>/dev/null || true
systemctl disable --now systemd-timesyncd 2>/dev/null || true

echo "Dua tick/freq ve chuan..."
python3 - <<'PY'
import ctypes, ctypes.util, os
class timex(ctypes.Structure):
    _fields_ = [("modes", ctypes.c_uint), ("offset", ctypes.c_long), ("freq", ctypes.c_long),
                ("maxerror", ctypes.c_long), ("esterror", ctypes.c_long), ("status", ctypes.c_int),
                ("constant", ctypes.c_long), ("precision", ctypes.c_long), ("tolerance", ctypes.c_long),
                ("time_sec", ctypes.c_long), ("time_usec", ctypes.c_long), ("tick", ctypes.c_long),
                ("ppsfreq", ctypes.c_long), ("jitter", ctypes.c_long), ("shift", ctypes.c_int),
                ("stabil", ctypes.c_long), ("jitcnt", ctypes.c_long), ("calcnt", ctypes.c_long),
                ("errcnt", ctypes.c_long), ("stbcnt", ctypes.c_long), ("tai", ctypes.c_int),
                ("pad", ctypes.c_int * 11)]
ADJ_FREQUENCY, ADJ_TICK = 0x0002, 0x4000
libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
t = timex(); t.modes = ADJ_FREQUENCY | ADJ_TICK; t.freq = 0; t.tick = 10000
if libc.adjtimex(ctypes.byref(t)) < 0:
    raise SystemExit(f"adjtimex loi: {os.strerror(ctypes.get_errno())}")
PY

echo "Sau:"; show
echo "Xong. Hyper-V tu dong bo gio cho WSL, khong can them gi."

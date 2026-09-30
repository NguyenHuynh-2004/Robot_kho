#!/usr/bin/env python3
"""Do nhip va do tre cua du lieu ESP32 gui ve.

    cd ~/robot_ws_3 && python3 tools/check_esp32_rates.py

VI SAO CAN FILE NAY thay vi `ros2 topic hz`:
  Phep do nay rat de tu lam sai. Ban dau toi do bang mot node subscribe ca 4
  topic voi depth=100, va no bao /odom chi ve 14 Hz, stamp tre 798 ms. Ca hai
  so deu sai: subscriber Python doc khong kip nen message nam cho trong hang
  doi, va chinh thoi gian nam cho do bi cong vao "do tre". Dau hieu nhan ra:
  khoang giua hai lan NHAN chi 10.7 ms trong khi firmware phat moi 20 ms -
  tuc dang doc tu mot dong ton, khong phai do thuc.

  Nen file nay chi subscribe MOT topic mot luc, va dung depth=1 de rmw luon bo
  message cu. Do lai bang cach nay: 19.7 ms/message, dung 50 Hz.

Dung 100% thu dong (khong publish gi) nen an toan khi xe dang de tren san.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

# Firmware dung best_effort. Subscriber phai khop, khong thi khong nhan gi.
TOPICS = {
    'scan':     ('/scan',     LaserScan, 8.7),
    'odom':     ('/odom',     Odometry,  50.0),
    'imu/data': ('/imu/data', Imu,       50.0),
    'tf':       ('/tf',       TFMessage, 50.0),
}


class One(Node):
    """Do mot topic. Callback khong lam gi ngoai ghi 2 con so."""

    def __init__(self, name, topic, msg_type):
        super().__init__('check_rate')
        self.recv = []
        self.lag = []
        self.create_subscription(
            msg_type, topic, self.cb,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))

    def cb(self, m):
        now = time.time()
        self.recv.append(now)
        hdr = getattr(m, 'header', None)
        if hdr is None and getattr(m, 'transforms', None):
            hdr = m.transforms[0].header
        if hdr is not None:
            self.lag.append(now - (hdr.stamp.sec + hdr.stamp.nanosec * 1e-9))


def measure(name, topic, msg_type, want, dur, settle):
    node = One(name, topic, msg_type)
    end = time.time() + settle          # discovery trong WSL kha cham
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.recv.clear()
    node.lag.clear()

    t0 = time.time()
    end = t0 + dur
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    dt = time.time() - t0

    print(f"\n--- {topic} ---")
    if not node.recv:
        print("  KHONG NHAN DUOC GI. Kiem tra: micro_ros_agent dang chay? "
              "ESP32 in 'Da thay agent!'? ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST?")
        node.destroy_node()
        return

    print(f"  {len(node.recv)} msg = {len(node.recv)/dt:.1f} Hz   "
          f"(firmware phat {want})")
    if node.lag:
        lag = sorted(node.lag)
        print(f"  lech stamp so voi dong ho may: trung vi {lag[len(lag)//2]*1000:+.1f} ms"
              f"  (nho nhat {lag[0]*1000:+.1f} / lon nhat {lag[-1]*1000:+.1f})")
        print("    so AM = stamp nhanh hon dong ho may. Am vai chuc ms la binh thuong")
        print("    va la phia lanh; duong vai tram ms moi la van de.")
    gaps = sorted(b - a for a, b in zip(node.recv, node.recv[1:]))
    if gaps:
        print(f"  khoang giua 2 lan nhan: trung vi {gaps[len(gaps)//2]*1000:.1f} ms"
              f" (dat: {1000.0/want:.1f}), lon nhat {gaps[-1]*1000:.1f} ms")
        n_big = sum(1 for g in gaps if g > 0.2)
        print(f"  so khoang > 200 ms: {n_big}"
              + ("   <- xe dung yen thi day la cac lan ping_agent chan"
                 if n_big else ""))
    node.destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', choices=list(TOPICS) + ['all'], default='all')
    ap.add_argument('--duration', type=float, default=15.0)
    ap.add_argument('--settle', type=float, default=8.0,
                    help='thoi gian cho discovery truoc khi bat dau dem')
    a = ap.parse_args()

    rclpy.init()
    names = list(TOPICS) if a.topic == 'all' else [a.topic]
    for n in names:
        topic, msg_type, want = TOPICS[n]
        measure(n, topic, msg_type, want, a.duration, a.settle)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

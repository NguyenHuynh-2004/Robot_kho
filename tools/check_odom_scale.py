#!/usr/bin/env python3
"""Hieu chinh WHEEL_RADIUS_M bang mot doan chay thang do bang thuoc.
Day la buoc A3 trong Phu luc A cua firmware, chua ai lam.

    cd ~/robot_ws_3 && python3 tools/check_odom_scale.py --distance 2.0

CACH LAM:
  1. Danh dau vach xuat phat tren san, do truoc quang duong se chay (vd 2.00 m)
     va danh dau vach dich.
  2. Dat xe o vach xuat phat, chay file nay, bam Enter.
  3. Lai xe chay THANG toi vach dich. Khong re.
  4. Bam Enter lan nua.

Cong cu tra loi hai cau mot luc:
  - DAU cua quang duong: am tuc odometry van bao nguoc -> firmware chua nap
    ban da dao chieu dong co.
  - DO LON: ti so giua quang duong odom bao va quang duong that chinh la ti so
    can nhan vao WHEEL_RADIUS_M.

Thuan doc, khong publish gi."""
import argparse
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry

WHEEL_RADIUS_NOW = 0.03225      # gia tri dang nam trong firmware


class O(Node):
    def __init__(self):
        super().__init__('check_odom_scale')
        self.last = None
        self.create_subscription(
            Odometry, '/odom', self.cb,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=10))

    def cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * (q.z * q.z))
        self.last = (p.x, p.y, yaw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--distance', type=float, required=True,
                    help='quang duong THAT se chay, met (vd 2.0)')
    ap.add_argument('--radius', type=float, default=WHEEL_RADIUS_NOW,
                    help='WHEEL_RADIUS_M dang nam trong firmware')
    a = ap.parse_args()

    rclpy.init()
    n = O()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    t0 = time.time()
    while n.last is None and time.time() - t0 < 15:
        time.sleep(0.1)
    if n.last is None:
        print("Khong nhan duoc /odom. micro_ros_agent va ESP32 co chay khong?")
        rclpy.shutdown()
        return

    input(f"\nDat xe o vach xuat phat roi bam Enter "
          f"(se do doan {a.distance:.2f} m)... ")
    x0, y0, th0 = n.last
    input("Lai xe chay THANG toi vach dich, roi bam Enter... ")
    x1, y1, th1 = n.last

    dx, dy = x1 - x0, y1 - y0
    # Chieu dai doan di duoc, co dau theo huong xe luc xuat phat
    along = dx * math.cos(th0) + dy * math.sin(th0)
    lateral = -dx * math.sin(th0) + dy * math.cos(th0)
    dyaw = math.degrees((th1 - th0 + math.pi) % (2 * math.pi) - math.pi)

    print(f"\n=== odometry bao ===")
    print(f"  doc theo huong xuat phat : {along:+.3f} m")
    print(f"  lech ngang               : {lateral:+.3f} m")
    print(f"  doi huong                : {dyaw:+.1f} do")
    print(f"  quang duong that (ban do): {a.distance:.3f} m")

    print("\n=== ket luan ===")
    if abs(along) < 0.05:
        print("  Odom gan nhu khong doi. Xe co thuc su chay khong? Hoac")
        print("  /odom khong cap nhat - chay tools/check_esp32_rates.py.")
        rclpy.shutdown()
        return

    if along < 0:
        print("  QUANG DUONG AM: odometry bao LUI trong khi xe chay TOI.")
        print("  -> Firmware tren chip CHUA phai ban da dao chieu dong co.")
        print("     Nap lai firmware roi do lai. Khong chinh WHEEL_RADIUS_M")
        print("     luc nay, vi do voi dau sai thi vo nghia.")
        rclpy.shutdown()
        return

    ratio = a.distance / along
    new_r = a.radius * ratio
    err = (along - a.distance) / a.distance * 100

    print(f"  Dau DUNG (odom bao toi, xe chay toi).")
    print(f"  Odom bao {along:.3f} m cho {a.distance:.3f} m that "
          f"-> sai {err:+.1f}%")
    if abs(err) < 2.0:
        print(f"  Duoi 2% - DAT, khong can sua WHEEL_RADIUS_M.")
    else:
        print(f"\n  Sua trong firmware:")
        print(f"    WHEEL_RADIUS_M  {a.radius:.5f}f  ->  {new_r:.5f}f")
        print(f"  (= {a.radius:.5f} * {a.distance:.3f}/{along:.3f})")
        print(f"  Nap lai roi do lai cho den khi sai so < 2%.")

    if abs(lateral) > 0.1 or abs(dyaw) > 10:
        print(f"\n  LUU Y: doan chay khong thang (lech ngang {lateral:+.2f} m,")
        print(f"  doi huong {dyaw:+.1f} do). Ket qua tren kem tin cay - chay lai")
        print(f"  cho that thang.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()

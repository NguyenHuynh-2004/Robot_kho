#!/usr/bin/env python3
"""Hieu chinh ban kinh banh bang mot doan chay thang do bang thuoc.
Tuong ung buoc A3 trong Phu luc A cua firmware.

Tu khi odometry chuyen len laptop, ket qua sua vao THAM SO wheel_radius cua
bringup_real.launch.py, KHONG phai vao WHEEL_RADIUS_M trong firmware. Khong
phai nap chip nua.

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
  - DO LON: ti so giua quang duong that va quang duong odom bao chinh la he so
    can nhan vao wheel_radius.

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


def read_live_radius(node, target):
    """Hoi wheel_radius tu node odometry dang chay. None neu khong hoi duoc."""
    try:
        from rclpy.parameter_client import AsyncParameterClient
        cli = AsyncParameterClient(node, target)
        if not cli.wait_for_services(timeout_sec=8.0):
            return None
        fut = cli.get_parameters(['wheel_radius'])
        end = time.time() + 8.0
        while not fut.done() and time.time() < end:
            time.sleep(0.05)
        if not fut.done():
            return None
        vals = fut.result().values
        return float(vals[0].double_value) if vals else None
    except Exception:
        return None


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
    ap.add_argument('--radius', type=float, default=None,
                    help='chi dung khi khong hoi duoc node dang chay')
    ap.add_argument('--node', default='wheel_odometry',
                    help='ten node odometry de hoi tham so')
    a = ap.parse_args()

    rclpy.init()
    n = O()

    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    # HOI THANG node dang chay thay vi tin vao --radius.
    # Da mat mot phep do vi cho nay: default trong launch da doi thanh 0.03300
    # nhung node van dang chay voi 0.03225 tu truoc do (chua khoi dong lai), ma
    # cong cu thi tinh he so tren 0.03300 -> ra mot con so vo nghia.
    #
    # PHAI goi SAU khi executor da chay: AsyncParameterClient tra ve future, ma
    # future chi hoan tat khi node duoc spin.
    live_r = read_live_radius(n, a.node)
    if live_r is not None:
        if a.radius is not None and abs(a.radius - live_r) > 1e-9:
            print(f"\n  LUU Y: ban truyen --radius {a.radius:.5f} nhung node "
                  f"/{a.node} dang chay voi {live_r:.5f}.")
            print(f"  Dung gia tri DANG CHAY. Neu vua sua launch thi phai khoi "
                  f"dong lai no.")
        a.radius = live_r
        print(f"\nwheel_radius dang dung (hoi tu /{a.node}): {a.radius:.5f} m")
    else:
        if a.radius is None:
            a.radius = WHEEL_RADIUS_NOW
        print(f"\n  CANH BAO: khong hoi duoc tham so cua /{a.node}. "
              f"Dung {a.radius:.5f} m.")
        print(f"  Neu node dang chay voi gia tri khac thi ket qua se sai.")

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
          f"-> sai {err:+.1f}%  (he so can nhan: {ratio:.4f})")
    if abs(err) < 2.0:
        print(f"  Duoi 2% - DAT, khong can sua gi.")
    else:
        print(f"\n  Sua tren LAPTOP, KHONG phai trong firmware:")
        print(f"    ros2 launch my_bot_bringup bringup_real.launch.py \\")
        print(f"        wheel_radius:={new_r:.5f}")
        print(f"  (= {a.radius:.5f} * {a.distance:.3f}/{along:.3f})")
        print(f"  Muon co dinh thi sua default cua wheel_radius trong")
        print(f"  launch/bringup_real.launch.py. Khoi dong lai node la xong,")
        print(f"  khong phai nap chip - odometry da chuyen len laptop.")
        print(f"\n  LUU Y khi do lai: truyen --radius la gia tri DANG DUNG,")
        print(f"  neu khong cong cu se tinh he so tren mot moc sai.")
        print(f"      python3 tools/check_odom_scale.py --distance 2.0 "
              f"--radius {new_r:.5f}")
        print(f"\n  He so {ratio:.4f} nay gop CA HAI nguon sai lech, khong tach")
        print(f"  duoc bang mot phep do: ban kinh LAN cua banh, va")
        print(f"  ENC_PULSES_PER_REV (dang la 1200, lay tu sketch goc, chua ai")
        print(f"  kiem). Tach bang buoc A1 trong Phu luc A cua firmware: nhac xe")
        print(f"  len, quay tay mot banh dung 10 vong, count phai doi ~12000.")

    if abs(lateral) > 0.1 or abs(dyaw) > 10:
        print(f"\n  LUU Y: doan chay khong thang (lech ngang {lateral:+.2f} m,")
        print(f"  doi huong {dyaw:+.1f} do). Ket qua tren kem tin cay - chay lai")
        print(f"  cho that thang.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()

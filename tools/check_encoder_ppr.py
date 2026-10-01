#!/usr/bin/env python3
"""Kiem ENC_PULSES_PER_REV bang cach quay tay banh xe mot so vong da biet.

    cd ~/robot_ws_3 && python3 tools/check_encoder_ppr.py --turns 10

CACH LAM:
  1. NHAC XE LEN khoi mat san (hoac ke banh len) de banh quay tu do.
  2. Chay file nay, bam Enter.
  3. Quay TAY mot banh dung so vong da khai bao, theo chieu TOI.
  4. Bam Enter.

VI SAO CAN:
  check_odom_scale.py cho mot he so hieu chinh duy nhat, nhung he so do gop
  HAI nguon sai lech nhan vao cung mot cho:
      quang duong = (so xung / ENC_PULSES_PER_REV) * 2*pi * wheel_radius
  Mot phep chay thang khong tach duoc chung. Phep nay tach: quay dung 10 vong
  thi goc phai doi dung 10*2*pi = 62.832 rad, khong dinh gi den ban kinh.

  Dieu dang nghi: hieu chinh tren san cho wheel_radius 0.03300, LON HON ban
  kinh hinh hoc 0.03225, trong khi lop bi nen thi phai cho ban kinh NHO hon.
  Neu phep nay cho thay encoder lech ~2%, thi thu pham la
  ENC_PULSES_PER_REV chu khong phai lop, va nen sua dung cho.

Thuan doc, khong publish gi."""
import argparse
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

# ENC_PULSES_PER_REV dang nam trong firmware. Do duoc bang
# firmware/encoder_test/encoder_test.ino (doc xung THO): 10 vong cho
# TRAI 21614 -> 2161.4 va PHAI 21375 -> 2137.5, lay trung binh 2150.
# So cu 1200 chep tu sketch goc va sai 78%.
PPR_NOW = 2150.0


class E(Node):
    def __init__(self):
        super().__init__('check_encoder_ppr')
        self.pos = {}
        self.create_subscription(
            JointState, 'joint_states', self.cb,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=10))

    def cb(self, m):
        for n, p in zip(m.name, m.position):
            self.pos[n] = p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--turns', type=float, default=10.0,
                    help='so vong se quay tay (mac dinh 10)')
    ap.add_argument('--ppr', type=float, default=PPR_NOW,
                    help='ENC_PULSES_PER_REV dang nam trong firmware')
    a = ap.parse_args()

    rclpy.init()
    n = E()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    t0 = time.time()
    while not n.pos and time.time() - t0 < 15:
        time.sleep(0.1)
    if not n.pos:
        print('Khong nhan duoc /joint_states. ESP32 va agent co chay khong?')
        rclpy.shutdown()
        return

    print(f"\nCac khop thay duoc: {', '.join(n.pos)}")
    input(f"\nNhac xe len. Bam Enter, roi quay TAY mot banh dung "
          f"{a.turns:.0f} vong theo chieu TOI... ")
    before = dict(n.pos)
    input("Quay xong thi bam Enter... ")
    after = dict(n.pos)

    want = a.turns * 2.0 * math.pi
    print(f"\n=== goc phai doi: {want:.3f} rad ({a.turns:.0f} vong) ===")
    any_moved = False
    for name in sorted(before):
        d = after.get(name, 0.0) - before[name]
        if abs(d) < 0.5:
            print(f"  {name:20s} {d:+8.3f} rad   (khong quay banh nay)")
            continue
        any_moved = True
        ratio = abs(d) / want
        new_ppr = a.ppr * ratio
        err = (abs(d) - want) / want * 100
        print(f"  {name:20s} {d:+8.3f} rad   = {abs(d)/(2*math.pi):.3f} vong"
              f"   sai {err:+.1f}%")
        if abs(err) < 1.0:
            print(f"  {'':20s} -> ENC_PULSES_PER_REV = {a.ppr:.0f} DUNG, "
                  f"khong sua.")
        else:
            print(f"  {'':20s} -> ENC_PULSES_PER_REV {a.ppr:.0f} "
                  f"-> {new_ppr:.0f}  (phai NAP LAI chip)")
            print(f"  {'':20s}    Sua xong thi do lai wheel_radius bang")
            print(f"  {'':20s}    check_odom_scale.py, vi hai cai bu cho nhau.")
        if d < 0:
            print(f"  {'':20s} CANH BAO: goc GIAM khi quay toi -> xem lai "
                  f"INVERT_ENC_x")

    if not any_moved:
        print("\n  Khong banh nao quay. Co quay dung banh khong?")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

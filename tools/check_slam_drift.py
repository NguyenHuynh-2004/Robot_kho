#!/usr/bin/env python3
"""Theo doi luong hieu chinh ma slam_toolbox dang bu cho odometry.

    cd ~/robot_ws_3 && python3 tools/check_slam_drift.py --duration 60

Chay trong luc dang lai xe ve ban do.

DOC KET QUA:
  map -> odom chinh la phan slam_toolbox phai BU vi odometry sai. Theo doi no
  theo thoi gian thi biet hong o dau:

    tang DAN va deu      -> odometry troi that, nhung khop scan van bam kip.
                            Ban do van dung, chi la odom kem.
    DUNG YEN khi xe chay -> khop scan DA MAT DAU. Tu luc do moi vong quet duoc
                            ve o vi tri odom doan, va no lech dan khoi tuong.
                            Day la trieu chung "lidar khong con bam vat can".
    nhay GIAT tung nac    -> khop scan dang vat lon, co the do anh quet nhoe khi
                            quay nhanh hoac phong qua trong tru.

  Cung in cac canh bao cua slam_toolbox tren /rosout - no co noi khi khong khop
  duoc, nhung dong do de troi mat giua rung log.

Thuan doc, khong publish gi."""
import argparse
import math
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rcl_interfaces.msg import Log
from nav_msgs.msg import Odometry
import tf2_ros

LEVEL = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}


class D(Node):
    def __init__(self):
        super().__init__('check_slam_drift')
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        tf2_ros.TransformListener(self.buf, self)
        self.logs = []
        self.create_subscription(
            Log, '/rosout', self.cb_log,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=500))

        # Toc do quay DINH giua hai lan in. Can de tra loi cau hoi: cac lan slam
        # phai sua nhieu co trung voi luc xe quay nhanh khong? LiDAR mat 0.116 s
        # mot vong va firmware khong bu chuyen dong, nen quay 1.2 rad/s la anh
        # quet nhoe 8 do - khop mot anh nhoe voi ban do sac net thi ra goc lech.
        self.w_peak = 0.0
        self.v_peak = 0.0
        self.create_subscription(
            Odometry, '/odom', self.cb_odom,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=20))

    def cb_log(self, m):
        if m.name.startswith('slam_toolbox') and m.level >= 30:
            self.logs.append((LEVEL.get(m.level, m.level), m.msg))

    def cb_odom(self, m):
        self.w_peak = max(self.w_peak, abs(m.twist.twist.angular.z))
        self.v_peak = max(self.v_peak, abs(m.twist.twist.linear.x))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--every', type=float, default=3.0)
    a = ap.parse_args()

    rclpy.init()
    n = D()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(8.0)                     # discovery trong WSL cham

    from rclpy.time import Time
    print(f"\nTheo doi {a.duration:.0f} s. HAY LAI XE trong luc nay.\n")
    print(f"{'t(s)':>5} {'map->odom x':>11} {'y':>8} {'yaw(do)':>9} "
          f"{'slam sua':>12} {'xe di':>12} {'v dinh':>8} {'w dinh':>8}")

    t0 = time.time()
    prev = prev_pose = None
    rows = []
    while time.time() - t0 < a.duration:
        try:
            tf = n.buf.lookup_transform('map', 'odom', Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            th = math.degrees(yaw_of(tf.transform.rotation))

            # PHAI biet xe co CHAY khong, neu khong se ket luan nham: khi xe do
            # thi map->odom dung yen la DUNG (minimum_travel_distance 0.2 nghia
            # la slam chi xu ly vong quet moi sau khi xe di 0.2 m).
            # Da mac loi nay o ban dau: bao "khop scan mat dau" trong khi xe
            # chi dang do.
            bp = n.buf.lookup_transform('odom', 'base_footprint', Time())
            px, py = bp.transform.translation.x, bp.transform.translation.y

            chg = moved = ''
            if prev is not None:
                chg = f"{math.hypot(x-prev[0], y-prev[1])*1000:6.0f} mm"
                dm = math.hypot(px-prev_pose[0], py-prev_pose[1])
                moved = f"{dm*1000:6.0f} mm"
            wp, vp = n.w_peak, n.v_peak
            n.w_peak = n.v_peak = 0.0
            print(f"{time.time()-t0:5.0f} {x:11.3f} {y:8.3f} {th:9.2f} "
                  f"{chg:>12} {moved:>12} {vp:8.2f} {wp:8.2f}")
            rows.append((x, y, th, px, py, wp))
            prev = (x, y)
            prev_pose = (px, py)
        except Exception as e:
            print(f"{time.time()-t0:5.0f}   khong tra cuu duoc TF: "
                  f"{type(e).__name__}")
        time.sleep(a.every)

    print("\n=== canh bao cua slam_toolbox ===")
    if not n.logs:
        print("  khong co canh bao nao (muc WARN tro len)")
    else:
        seen = {}
        for lvl, msg in n.logs:
            k = (lvl, msg[:120])
            seen[k] = seen.get(k, 0) + 1
        for (lvl, msg), c in seen.items():
            print(f"  [{lvl}] {msg}" + (f"   (x{c})" if c > 1 else ""))

    if len(rows) > 2:
        corr = math.hypot(rows[-1][0] - rows[0][0], rows[-1][1] - rows[0][1])
        dth = rows[-1][2] - rows[0][2]
        # Quang duong XE THUC SU di, cong don tung doan
        drove = sum(math.hypot(b[3]-a_[3], b[4]-a_[4])
                    for a_, b in zip(rows, rows[1:]))
        n_corr = sum(1 for a_, b in zip(rows, rows[1:])
                     if math.hypot(b[0]-a_[0], b[1]-a_[1]) > 0.001)

        print(f"\n=== tong ket ===")
        print(f"  xe di duoc           : {drove:.2f} m")
        print(f"  map->odom doi        : {corr*1000:.0f} mm, {dth:+.2f} do")
        print(f"  so lan slam sua pose : {n_corr}/{len(rows)-1}")

        if drove < 0.3:
            print("\n  XE GAN NHU KHONG DI CHUYEN -> phep do nay khong noi len")
            print("  dieu gi. slam chi xu ly vong quet moi sau khi xe di 0.2 m")
            print("  (minimum_travel_distance), nen map->odom dung yen la DUNG.")
            print("  Chay lai VA LAI XE trong suot thoi gian do.")
        elif n_corr == 0:
            print("\n  Xe co di ma slam KHONG sua pose lan nao -> khop scan da")
            print("  mat dau. Tu do moi vong quet duoc ve o vi tri odom doan,")
            print("  va no lech dan khoi tuong tren ban do.")
        elif corr > 0.3 or abs(dth) > 5.0:
            print("\n  slam van dang sua, nhung phai sua NHIEU -> odometry troi")
            print("  manh. Ban do con dung nhung bien an toan dang mong.")
        else:
            print("\n  slam sua deu va luong sua nho -> odometry va khop scan")
            print("  deu lanh.")

        # Cac lan sua lon co trung voi luc quay nhanh khong?
        pairs = [(math.hypot(b[0]-a_[0], b[1]-a_[1]), b[5])
                 for a_, b in zip(rows, rows[1:])]
        big = [w for c, w in pairs if c > 0.15]
        small = [w for c, w in pairs if c <= 0.15]
        if big and small:
            mb = sum(big) / len(big)
            ms = sum(small) / len(small)
            print(f"\n  toc do quay dinh, trung binh theo nhom:")
            print(f"    luc slam sua NHIEU (>150 mm): {mb:.2f} rad/s "
                  f"({len(big)} lan)")
            print(f"    luc slam sua it   (<150 mm): {ms:.2f} rad/s "
                  f"({len(small)} lan)")
            if mb > ms * 1.4:
                print("    -> sua nhieu TRUNG voi quay nhanh. LiDAR mat 0.116 s")
                print("       mot vong va khong duoc bu chuyen dong, nen quay")
                print("       nhanh la anh quet nhoe. Ha scale_angular xuong,")
                print("       hoac quay cham tay khi ve ban do.")
            else:
                print("    -> khong trung. Nguyen nhan nam o cho khac, khong")
                print("       phai anh quet nhoe khi quay.")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

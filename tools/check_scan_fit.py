#!/usr/bin/env python3
"""Do bang SO cai ma ban dang nhin bang MAT trong RViz: anh quet lech khoi
tuong tren ban do bao nhieu met.

    cd ~/robot_ws_3 && python3 tools/check_scan_fit.py --duration 60

Voi moi tia hop le: doi sang he toa do map qua TF (map->odom->base->lidar), roi
tim o TUONG gan nhat tren /map. Khoang cach do chinh la do lech ban thay tren
man hinh - nhung ra thanh con so thay vi uoc luong bang mat.

DOC KET QUA
  trung vi < 0.05 m   anh quet trung tuong. Lanh.
  0.05 - 0.15 m       lech thay duoc nhung trong gioi han o luoi (0.05 m).
  > 0.15 m            lech that su. Xem cot "khi dung yen" va "khi dang chay":
                      chi lech luc dang chay -> do tre giua hai lan khop scan,
                                                ban do van dung
                      lech ca luc dung yen   -> uoc luong vi tri that su sai

Thuan doc, khong publish gi."""
import argparse
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros

OCCUPIED = 65          # gia tri tren /map tro len thi coi la tuong
MOVING = 0.03          # m/s - nhanh hon the nay thi coi la dang chay


class F(Node):
    def __init__(self):
        super().__init__('check_scan_fit')
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        tf2_ros.TransformListener(self.buf, self)
        self.grid = None
        self.scan = None
        self.speed = 0.0

        self.create_subscription(
            OccupancyGrid, '/map', self.cb_map,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(
            LaserScan, '/scan', self.cb_scan,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=2))
        self.create_subscription(
            Odometry, '/odom', self.cb_odom,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=5))

    def cb_map(self, m):
        self.grid = m

    def cb_scan(self, m):
        self.scan = m

    def cb_odom(self, m):
        self.speed = math.hypot(m.twist.twist.linear.x,
                                m.twist.twist.angular.z * 0.1)


def wall_points(grid):
    """Toa do (x, y) trong he map cua moi o tuong."""
    w, h = grid.info.width, grid.info.height
    a = np.array(grid.data, dtype=np.int16).reshape(h, w)
    ys, xs = np.nonzero(a >= OCCUPIED)
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    return np.stack([ox + (xs + 0.5) * res, oy + (ys + 0.5) * res], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--every', type=float, default=2.0)
    a = ap.parse_args()

    rclpy.init()
    n = F()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    end = time.time() + 25
    while time.time() < end and (n.grid is None or n.scan is None):
        time.sleep(0.2)
    if n.grid is None:
        print('Khong nhan duoc /map. slam_toolbox co dang chay khong?')
        rclpy.shutdown(); return
    if n.scan is None:
        print('Khong nhan duoc /scan.')
        rclpy.shutdown(); return

    print(f"\n{'t(s)':>5} {'tia':>5} {'lech trung vi':>14} {'90%':>8} "
          f"{'trang thai':>12}")
    walls = None
    wall_stamp = None
    still, moving = [], []
    t0 = time.time()
    while time.time() - t0 < a.duration:
        time.sleep(a.every)
        g, s = n.grid, n.scan
        if g is None or s is None:
            continue
        # Chi dung lai danh sach o tuong khi /map thay doi - no to va cham
        if wall_stamp != (g.header.stamp.sec, g.header.stamp.nanosec):
            walls = wall_points(g)
            wall_stamp = (g.header.stamp.sec, g.header.stamp.nanosec)
        if walls is None or len(walls) < 10:
            continue

        try:
            tf = n.buf.lookup_transform('map', s.header.frame_id, Time())
        except Exception as e:
            print(f"{time.time()-t0:5.0f}   khong tra cuu duoc TF: "
                  f"{type(e).__name__}")
            continue
        q = tf.transform.rotation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y),
                        1 - 2 * (q.y * q.y + q.z * q.z))
        tx, ty = tf.transform.translation.x, tf.transform.translation.y

        r = np.array(s.ranges, dtype=np.float64)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = (r > s.range_min) & (r < s.range_max) & np.isfinite(r)
        if ok.sum() < 20:
            continue
        r, ang = r[ok], ang[ok]
        # Diem trong he lidar -> he map
        px = tx + r * np.cos(ang + th)
        py = ty + r * np.sin(ang + th)

        # Khoang cach toi o tuong gan nhat. Chia nho de khong ngon bo nho.
        d = np.empty(len(px))
        for i in range(0, len(px), 64):
            sl = slice(i, i + 64)
            dd = np.hypot(walls[None, :, 0] - px[sl, None],
                          walls[None, :, 1] - py[sl, None])
            d[sl] = dd.min(axis=1)

        med = float(np.median(d))
        p90 = float(np.percentile(d, 90))
        run = n.speed > MOVING
        (moving if run else still).append(med)
        print(f"{time.time()-t0:5.0f} {len(r):5d} {med:11.3f} m {p90:7.3f} "
              f"{'dang chay' if run else 'dung yen':>12}")

    print(f"\n=== tong ket ===")
    for ten, arr in (('dung yen', still), ('dang chay', moving)):
        if arr:
            print(f"  {ten:10s}: lech trung vi {np.median(arr):.3f} m "
                  f"({len(arr)} lan do)")
        else:
            print(f"  {ten:10s}: khong co lan do nao")
    if still and moving:
        a_s, a_m = float(np.median(still)), float(np.median(moving))
        print()
        if a_s < 0.08 and a_m > a_s * 1.5:
            print("  Chi lech khi DANG CHAY -> do tre giua hai lan khop scan.")
            print("  Ban do van dung; cai ban thay trong RViz la vi tri odom")
            print("  doan giua hai lan slam sua.")
        elif a_s > 0.15:
            print("  Lech CA KHI DUNG YEN -> uoc luong vi tri that su sai,")
            print("  khong phai do tre hien thi.")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

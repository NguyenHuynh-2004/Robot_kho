#!/usr/bin/env python3
"""Dinh vi lai tung xe bang ICP (khop anh quet voi /map) - dung khi chay 2 xe.

    python3 tools/fleet_localize.py                 # ca agv1 va agv2
    python3 tools/fleet_localize.py agv2            # mot xe
    python3 tools/fleet_localize.py --dry-run       # chi in, khong gui AMCL

Diem xuat phat = vi tri AMCL hien tai (fleet_real.sh dat AMCL o cho ban khai
bao khi bat, nen xe dat dung o la du gan). ICP tinh tien + xoay de anh quet
trung tuong nhat, roi gui /<xe>/initialpose. Khong lam xe chay.

Khac tools/auto_initial_pose.py (do toan ban do, can >= 60% tia trung tuong):
cong cu nay chi tinh chinh quanh vi tri dang co nen van dung duoc khi phong
co them do dac (da gap: xe 1 o cot 1 chi 58%).
"""
import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

LIDAR_X = 0.014        # LiDAR (tam than) truoc base_footprint (tam truc banh), URDF
MIN_GOOD = 0.40        # it hon 40% tia trong 3 cm -> khong tin, khong gui


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Loc(Node):
    def __init__(self, cars):
        super().__init__('fleet_localize')
        self.map = None
        self.scans = {c: [] for c in cars}
        self.amcl = {}
        self.pubs = {}
        self.create_subscription(
            OccupancyGrid, '/map', lambda m: setattr(self, 'map', m),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        for c in cars:
            self.create_subscription(LaserScan, f'/{c}/scan',
                                     lambda m, c=c: self.scans[c].append(m), qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, f'/{c}/amcl_pose',
                                     lambda m, c=c: self.amcl.__setitem__(c, m),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.pubs[c] = self.create_publisher(PoseWithCovarianceStamped, f'/{c}/initialpose', 1)


def icp(tree, M, RA, x0, y0, th0):
    def pts(p):
        return np.c_[p[0] + RA[:, 0] * np.cos(RA[:, 1] + p[2]),
                     p[1] + RA[:, 0] * np.sin(RA[:, 1] + p[2])]
    best = None
    for dth in np.radians([-6, -3, 0, 3, 6]):
        p = np.array([x0, y0, th0 + dth])
        for it in range(30):
            d, idx = tree.query(pts(p))
            keep = d < max(0.30 * 0.85 ** it, 0.05)
            if keep.sum() < 20:
                break
            T = M[idx[keep]]
            p = least_squares(lambda q: (pts(q)[keep] - T).ravel(), p,
                              loss='huber', f_scale=0.03).x
        d, _ = tree.query(pts(p))
        score = (float(np.median(d)), float(np.mean(d < 0.03)))
        if best is None or score[0] < best[1][0]:
            best = (p, score)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cars', nargs='*', default=['agv1', 'agv2'])
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    n = Loc(a.cars)
    t0 = time.time()
    while time.time() - t0 < 10 and (n.map is None or len(n.amcl) < len(a.cars)
                                      or min(len(v) for v in n.scans.values()) < 10):
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.map is None:
        print('Khong nhan duoc /map - tools/fleet_real.sh up da chay chua?')
        return
    m = n.map
    W, H, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    occ = np.argwhere(np.array(m.data).reshape(H, W) > 50)
    M = np.c_[ox + (occ[:, 1] + 0.5) * res, oy + (occ[:, 0] + 0.5) * res]
    tree = cKDTree(M)

    for c in a.cars:
        if c not in n.amcl or len(n.scans[c]) < 5:
            print(f'{c}: thieu AMCL hoac /scan - xe da ket noi chua?')
            continue
        p0 = n.amcl[c].pose.pose
        th = yaw_of(p0.orientation)
        lx, ly = p0.position.x + LIDAR_X * math.cos(th), p0.position.y + LIDAR_X * math.sin(th)
        RA = []
        for s in n.scans[c][-10:]:
            ang = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
            rr = np.array(s.ranges)
            ok = np.isfinite(rr) & (rr > 0.15) & (rr < 6)
            RA.append(np.c_[rr[ok], ang[ok]])
        RA = np.vstack(RA)
        p, (med, good) = icp(tree, M, RA, lx, ly, th)
        bx, by = p[0] - LIDAR_X * math.cos(p[2]), p[1] - LIDAR_X * math.sin(p[2])
        moved = math.hypot(bx - p0.position.x, by - p0.position.y) * 100
        print(f'{c}: AMCL ({p0.position.x:+.3f}, {p0.position.y:+.3f}, {math.degrees(th):+.1f} do) -> '
              f'ICP ({bx:+.3f}, {by:+.3f}, {math.degrees(p[2]):+.1f} do), doi {moved:.1f} cm | '
              f'trung vi cach tuong {med * 100:.1f} cm, {good * 100:.0f}% tia trong 3 cm')
        if good < MIN_GOOD:
            print(f'  KHONG GUI: anh quet khop kem (< {MIN_GOOD * 100:.0f}%) - xe dat sai o, hoac '
                  'quanh xe co nhieu do dac khong co tren ban do. Dat lai xe roi chay lai.')
            continue
        if a.dry_run:
            continue
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(bx), float(by)
        msg.pose.pose.orientation.z = math.sin(p[2] / 2)
        msg.pose.pose.orientation.w = math.cos(p[2] / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02 ** 2
        msg.pose.covariance[35] = math.radians(1.5) ** 2
        t1 = time.time()
        while n.pubs[c].get_subscription_count() == 0 and time.time() - t1 < 5:
            rclpy.spin_once(n, timeout_sec=0.1)
        n.pubs[c].publish(msg)
        time.sleep(0.3)
        print('  da gui cho AMCL')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

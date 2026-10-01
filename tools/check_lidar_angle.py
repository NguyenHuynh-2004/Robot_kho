#!/usr/bin/env python3
"""Kiem LIDAR_ANGLE_FLIP: anh quet co bi lat guong khong.

    cd ~/robot_ws_3 && python3 tools/check_lidar_angle.py

CACH DUNG: cho xe QUAY TAI CHO (giu LB, day can sang ngang) trong suot thoi
gian do. Khong can quay dung bao nhieu do, chi can quay lien tuc ~15 giay.

VI SAO CAN:
  Lech goc co dinh (LIDAR_ANGLE_OFFSET_DEG sai) KHONG pha SLAM - no chi la mot
  phep quay co dinh cua khung lidar, cac vong quet van nhat quan voi nhau, ban
  do van sac net, chi bi xoay so voi huong xe.

  Nhung LAT GUONG (LIDAR_ANGLE_FLIP sai) thi pha. Lat guong khong phai phep
  quay. Xe quay trai thi anh quet DUNG phai xoay phai trong khung xe; anh bi
  lat se xoay trai. Khop scan hong dung moi khi xe quay, con chay thang van
  on -> ban do ra tung mang trang tu te noi voi nhau bang rac.

CACH DO:
  1. Uoc luong goc xoay giua hai vong quet lien tiep bang tuong quan vong:
     tim do dich bin lam hai duong range khop nhau nhat.
  2. Tich phan gyro Z trong dung khoang thoi gian do.
  3. So sanh. Vat the dung yen, xe quay trai (gyro Z duong) mot goc dθ thi
     phuong vi cua vat trong khung xe GIAM di dθ. Vay:
         tong dich anh quet  ~  -tong gyro      -> dung
         tong dich anh quet  ~  +tong gyro      -> BI LAT GUONG

Thuan doc, khong publish gi."""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

DUR = 15.0
# Giua hai vong quet lien tiep (8.6 Hz) xe quay chung 5 bin o toc do thuong.
# Tam tim rong qua thi de roi vao cuc tieu gia o phong vuong (co doi xung
# 180 do) - da thu: tam +-40 bin cho sai toi 8 bin, +-20 thi khong.
# Tran vat ly: wz_max 1.5 rad/s * 0.116 s = 0.17 rad = 8.9 bin. 15 la du rong.
MAX_SHIFT_BINS = 15
MIN_OVERLAP = 50         # so bin hop le chung toi thieu de tin ket qua
# Khop DUNG cho cost ~0.02-0.03 m (dung bang muc nhieu cua LiDAR nay, da do:
# bien do dao dong trung vi 21 mm). Khop SAI cho ~0.09 tro len. 0.08 tach
# duoc hai nhom ma van con bien cho phong nhieu do thuc te hon du lieu gia.
MAX_COST_M = 0.08


class C(Node):
    def __init__(self):
        super().__init__('check_lidar_angle')
        be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.scans = []      # (t, np.array ranges)
        self.gyro = []       # (t, wz)
        self.rec = False
        self.create_subscription(LaserScan, '/scan', self.cb_scan, be)
        self.create_subscription(Imu, '/imu/data', self.cb_imu, be)

    def cb_scan(self, m):
        if self.rec:
            self.scans.append((time.time(), np.array(m.ranges, dtype=np.float64),
                               m.range_min, m.range_max))

    def cb_imu(self, m):
        if self.rec:
            self.gyro.append((time.time(), m.angular_velocity.z))


def best_shift(r1, r2, lo, hi):
    """Do dich bin lam r2 khop r1 nhat. Tra ve (shift, so bin chung, cost).

    Cost la TRUNG VI cua |hieu| chu khong phai trung binh: LiDAR nay co ~6%
    tia rac, va trung binh bi chung keo lech du de chon sai. Da thu tren du
    lieu gia co day du nhieu 2 cm, 6% tia rac va 26% bin trong: trung binh
    cho sai toi 8 bin, trung vi sai toi da 1 bin.
    """
    v1 = (r1 > lo) & (r1 < hi)
    v2 = (r2 > lo) & (r2 < hi)
    best_k, best_cost, best_cnt = None, float('inf'), 0
    for k in range(-MAX_SHIFT_BINS, MAX_SHIFT_BINS + 1):
        r2s = np.roll(r2, -k)
        both = v1 & np.roll(v2, -k)
        cnt = int(both.sum())
        if cnt < MIN_OVERLAP:
            continue
        cost = float(np.median(np.abs(r1[both] - r2s[both])))
        if cost < best_cost:
            best_k, best_cost, best_cnt = k, cost, cnt
    return best_k, best_cnt, best_cost


def gyro_between(gyro, t0, t1):
    """Tich phan wz tu t0 den t1 (hinh thang)."""
    s = 0.0
    pts = [(t, w) for t, w in gyro if t0 <= t <= t1]
    for (ta, wa), (tb, wb) in zip(pts, pts[1:]):
        s += 0.5 * (wa + wb) * (tb - ta)
    return s


def main():
    rclpy.init()
    n = C()

    end = time.time() + 6
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

    print(f"\n>>> CHO XE QUAY TAI CHO NGAY BAY GIO, giu trong {DUR:.0f} giay <<<\n")
    n.rec = True
    end = time.time() + DUR
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
    n.rec = False

    print(f"nhan duoc {len(n.scans)} vong quet, {len(n.gyro)} mau gyro")
    if len(n.scans) < 5 or len(n.gyro) < 10:
        print("Khong du du lieu. Agent va ESP32 co dang chay khong?")
        n.destroy_node(); rclpy.shutdown(); return

    nbin = len(n.scans[0][1])
    bin_rad = 2 * math.pi / nbin

    sum_scan = 0.0
    sum_gyro = 0.0
    pairs = bad = 0
    for (t1, r1, lo, hi), (t2, r2, _, _) in zip(n.scans, n.scans[1:]):
        k, cnt, cost = best_shift(r1, r2, lo, hi)
        # Khop te -> hai vong quet khong nhin cung mot canh phong (xe quay qua
        # nhanh, hoac co nguoi di qua). Tinh vao chi lam nhieu ket qua.
        if k is None or cost > MAX_COST_M:
            bad += 1
            continue
        sum_scan += k * bin_rad
        sum_gyro += gyro_between(n.gyro, t1, t2)
        pairs += 1

    print(f"ghep duoc {pairs} cap vong quet lien tiep, bo {bad} cap khop te\n")
    if pairs < 5:
        print("Qua it cap dung duoc. Quay cham lai roi do lai.")
        n.destroy_node(); rclpy.shutdown(); return
    print(f"  tong dich anh quet : {sum_scan:+.3f} rad  ({math.degrees(sum_scan):+.1f} do)")
    print(f"  tong gyro tich phan: {sum_gyro:+.3f} rad  ({math.degrees(sum_gyro):+.1f} do)")

    if abs(sum_gyro) < 1.0:
        print("\n  Xe quay chua du (can > 1 rad ~ 57 do). Chay lai va quay nhieu hon.")
    else:
        ratio = sum_scan / sum_gyro
        print(f"  ti so dich/gyro    : {ratio:+.2f}\n")
        if ratio < -0.6:
            print("  KET LUAN: anh quet xoay NGUOC chieu xe -> DUNG.")
            print("  LIDAR_ANGLE_FLIP hien tai la dung, khong sua.")
            if abs(abs(ratio) - 1.0) > 0.25:
                print(f"  Nhung do lon lech nhieu (|{ratio:.2f}| thay vi 1.00) - xem lai")
                print("  SCAN_BINS hoac hieu chinh gyro.")
        elif ratio > 0.6:
            print("  KET LUAN: anh quet xoay CUNG chieu xe -> BI LAT GUONG.")
            print("  Sua trong firmware: doi LIDAR_ANGLE_FLIP tu 1 thanh 0")
            print("  (hoac tu 0 thanh 1 neu dang la 0), roi nap lai va ve lai ban do.")
        else:
            print("  KET LUAN: khong ro. Co the xe quay qua nhanh (anh quet bi")
            print("  nhoe trong moi vong) hoac phong qua trong tru de khop.")
            print("  Quay cham lai roi do lai.")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

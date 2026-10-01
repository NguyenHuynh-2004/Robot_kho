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
import bisect
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
        self.quat = []       # (t, yaw tuyet doi tu quaternion BNO055)
        self.rec = False
        self.create_subscription(LaserScan, '/scan', self.cb_scan, be)
        self.create_subscription(Imu, '/imu/data', self.cb_imu, be)

    def cb_scan(self, m):
        if self.rec:
            self.scans.append((time.time(), np.array(m.ranges, dtype=np.float64),
                               m.range_min, m.range_max))

    def cb_imu(self, m):
        if self.rec:
            now = time.time()
            self.gyro.append((now, m.angular_velocity.z))
            q = m.orientation
            self.quat.append((now, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                              1 - 2 * (q.y * q.y + q.z * q.z))))


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
    """Tich phan wz tu t0 den t1 (hinh thang), co NOI SUY o hai bien.

    Ban dau ham nay chi cong cac mau nam han trong [t0, t1] va bo hai doan cut
    o hai dau. Mau gyro cach nhau ~0.021 s con khoang giua hai vong quet
    ~0.113 s, nen moi lan bo nhu vay thieu toi ~18%. Do la ly do lan do dau
    tien ra ti so -1.13 thay vi -1.00: xe khong sai 13%, cong cu sai.
    """
    s = 0.0
    for (ta, wa), (tb, wb) in zip(gyro, gyro[1:]):
        if tb <= t0 or ta >= t1:
            continue
        # Cat doan [ta,tb] ve trong [t0,t1], noi suy tuyen tinh wz o cho cat
        lo, hi = max(ta, t0), min(tb, t1)
        if hi <= lo:
            continue
        span = tb - ta
        if span <= 0:
            continue
        wlo = wa + (wb - wa) * (lo - ta) / span
        whi = wa + (wb - wa) * (hi - ta) / span
        s += 0.5 * (wlo + whi) * (hi - lo)
    return s


def quat_between(quat, t0, t1):
    """Goc quaternion doi tu t0 den t1, lay mau gan nhat moi dau (50 Hz,
    sai lech <= 10 ms - nho so voi khoang 113 ms giua hai vong quet)."""
    if not quat:
        return 0.0
    ts = [t for t, _ in quat]

    def at(t):
        i = bisect.bisect_left(ts, t)
        if i <= 0:
            return quat[0][1]
        if i >= len(ts):
            return quat[-1][1]
        return quat[i][1] if ts[i] - t < t - ts[i - 1] else quat[i - 1][1]

    d = at(t1) - at(t0)
    return (d + math.pi) % (2 * math.pi) - math.pi


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
    sum_quat = 0.0
    res_g, res_q = [], []
    pairs = bad = 0
    for (t1, r1, lo, hi), (t2, r2, _, _) in zip(n.scans, n.scans[1:]):
        k, cnt, cost = best_shift(r1, r2, lo, hi)
        # Khop te -> hai vong quet khong nhin cung mot canh phong (xe quay qua
        # nhanh, hoac co nguoi di qua). Tinh vao chi lam nhieu ket qua.
        if k is None or cost > MAX_COST_M:
            bad += 1
            continue
        s = k * bin_rad
        g = gyro_between(n.gyro, t1, t2)
        qd = quat_between(n.quat, t1, t2)
        sum_scan += s
        sum_gyro += g
        sum_quat += qd
        # Anh quet xoay NGUOC chieu xe, nen s + nguon = 0 neu nguon dung.
        res_g.append(s + g)
        res_q.append(s + qd)
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

    # ------------------------------------------------------------------
    # NGUON GOC NAO DUNG HON? Lay anh quet LiDAR lam chuan doi chieu.
    # Quaternion BNO055 dung tu ke de giu huong tuyet doi; neu tu ke chua hieu
    # chinh hard-iron (nam cham dong co nam cach vai cm) thi sai so huong THAY
    # DOI THEO HUONG xe dang quay mat - dung yen mot huong thi on dinh, quay
    # quanh phong thi dao dong. Gyro mien nhiem chuyen do.
    if pairs >= 5 and abs(sum_gyro) > 1.0:
        def rms(a):
            return math.sqrt(sum(x * x for x in a) / len(a))
        rg, rq = math.degrees(rms(res_g)), math.degrees(rms(res_q))
        print("\n=== nguon goc nao khop anh quet hon ===")
        print(f"  {'nguon':16s} {'tong':>10} {'ti so':>8} {'lech moi cap (RMS)':>20}")
        print(f"  {'anh quet LiDAR':16s} {math.degrees(sum_scan):+9.1f}d")
        for ten, tot, r in (('gyro', sum_gyro, rg), ('quaternion', sum_quat, rq)):
            print(f"  {ten:16s} {math.degrees(tot):+9.1f}d "
                  f"{sum_scan / tot:+8.2f} {r:17.2f} do")
        print()
        if rq > rg * 1.5:
            print("  -> GYRO khop anh quet tot hon han. Quaternion lech theo huong")
            print("     xe dang quay mat - dau hieu tu ke bi nam cham dong co keo.")
            print("     Dung:  bringup_real.launch.py yaw_source:=imu_gyro")
        elif rg > rq * 1.5:
            print("  -> QUATERNION khop tot hon. Giu yaw_source:=imu_quaternion.")
        else:
            print("  -> Hai nguon ngang nhau. Lech cua slam khong den tu goc")
            print("     odometry; tim o cho khac.")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

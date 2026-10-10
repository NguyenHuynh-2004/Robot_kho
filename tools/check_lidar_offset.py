#!/usr/bin/env python3
"""Do goc lech giua anh quet LiDAR va huong truoc cua xe.

    cd ~/robot_ws_3 && python3 tools/check_lidar_offset.py [--offset 180]

--offset = LIDAR_ANGLE_OFFSET_DEG dang nap trong xe (moi xe mot so, xem khoi
"#if XE == 1" trong firmware). Mac dinh 180.

CACH LAM:
  1. De xe dung yen tren san.
  2. Dat MOT vat (hop giay, chai nuoc, quyen sach dung) ngay TRUOC MUI XE,
     cach 0.4 - 1.0 m, cang dung truoc cang tot.
  3. Don trong xung quanh: trong ban kinh ~1.5 m khong duoc co gi khac gan hon
     vat do. Tuong o xa thi khong sao.
  4. Chay file nay.

VI SAO CAN:
  LiDAR gan len xe theo mot huong bat ky. Firmware bu bang
  LIDAR_ANGLE_OFFSET_DEG, nhung so do tu truoc den nay la so DAT TAM (90 do),
  chua ai do.

  Lech goc co dinh KHONG lam ban do bi meo - ban do va anh quet van nhat quan
  voi nhau. Nhung no lam moi thu suy ra tu LiDAR bi dat lech so voi xe: Nav2
  bao "vat can phia truoc" trong khi vat can thuc ra nam lech 79 do sang ben.
  Chay Nav2 voi loi nay la dam.

Thuan doc, khong publish gi."""
import math
import statistics
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

N_SCAN = 25
FIRMWARE_OFFSET_NOW = 180.0     # LIDAR_ANGLE_OFFSET_DEG dang nap (doi bang --offset)


class S(Node):
    def __init__(self):
        super().__init__('check_lidar_offset')
        self.scans = []
        self.create_subscription(
            LaserScan, '/scan', self.cb,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=5))

    def cb(self, m):
        if len(self.scans) < N_SCAN:
            self.scans.append((np.array(m.ranges, dtype=np.float64),
                               m.angle_min, m.angle_increment,
                               m.range_min, m.range_max))


WIN = 2          # cua so trung vi truot: +-2 bin = 5 bin = 5.6 do


def nearest_bearing(ranges, amin, ainc, lo, hi):
    """Phuong vi cua VAT gan nhat, do rong (so bin), va khoang cach.

    KHONG lay tia gan nhat. LiDAR nay co ~6% tia rac va co tia chi 0.24 m, nen
    argmin tho se bam vao rac chu khong vao vat: da thu tren du lieu gia, vat
    dat o 15 do bi do thanh 100.8 do.
    Lam tron bang TRUNG VI TRUOT +-2 bin truoc: mot tia rac le loi trong cua so
    5 bin khong keo noi trung vi, con vat that rong hang chuc bin nen khong bi
    lam tron mat.
    """
    n = len(ranges)
    v = (ranges > lo) & (ranges < hi)
    if v.sum() < 10:
        return None, 0, None

    sm = np.full(n, np.inf)
    for i in range(n):
        vals = [ranges[(i + k) % n] for k in range(-WIN, WIN + 1)
                if v[(i + k) % n]]
        if len(vals) >= 3:
            sm[i] = float(np.median(vals))
    if not np.isfinite(sm).any():
        return None, 0, None

    idx = int(np.argmin(sm))
    d = sm[idx]
    width = int((sm <= d + 0.10).sum())
    return math.degrees(amin + idx * ainc), width, d


def main():
    global FIRMWARE_OFFSET_NOW
    if "--offset" in sys.argv:
        FIRMWARE_OFFSET_NOW = float(sys.argv[sys.argv.index("--offset") + 1])
    rclpy.init()
    n = S()
    end = time.time() + 25
    while time.time() < end and len(n.scans) < N_SCAN:
        rclpy.spin_once(n, timeout_sec=0.05)

    if len(n.scans) < 5:
        print(f"Chi nhan duoc {len(n.scans)} vong quet. "
              "micro_ros_agent va ESP32 co chay khong?")
        n.destroy_node(); rclpy.shutdown(); return

    res = [nearest_bearing(*s) for s in n.scans]
    good = [(b, w, d) for b, w, d in res if b is not None]
    if not good:
        print("Khong tia nao hop le. Kiem tra LiDAR co quay khong.")
        n.destroy_node(); rclpy.shutdown(); return

    bearings = [b for b, _, _ in good]
    widths = [w for _, w, _ in good]
    dists = [d for _, _, d in good]

    # Trung vi theo huong (vong tron) - tranh chuyen 359 va 1 do trung binh ra 180
    s = statistics.median([math.sin(math.radians(b)) for b in bearings])
    c = statistics.median([math.cos(math.radians(b)) for b in bearings])
    bm = math.degrees(math.atan2(s, c)) % 360.0
    spread = max(bearings) - min(bearings) if len(bearings) > 1 else 0.0

    print(f"\n=== {len(good)} vong quet ===")
    print(f"  diem gan nhat o phuong vi : {bm:.1f} do")
    print(f"  khoang cach               : {statistics.median(dists):.2f} m")
    print(f"  do rong vat               : {statistics.median(widths):.0f} tia "
          f"(~{statistics.median(widths) * 1.125:.0f} do)")
    print(f"  dao dong giua cac vong    : {spread:.1f} do")

    if statistics.median(widths) > 60:
        print("\n  CANH BAO: vat chiem qua nhieu tia - co le dang do mot buc")
        print("  tuong chu khong phai vat ban dat. Ket qua khong dung duoc.")

    # Vat nam dung truoc mui xe -> dang le phai cho phuong vi 0.
    # Do duoc bm, tuc anh quet dang bi xoay du +bm. Trong firmware:
    #     bearing = (360 - OFFSET) - angle_lidar
    # nen tang OFFSET len bm do la keo bearing xuong dung bm do.
    new_off = (FIRMWARE_OFFSET_NOW + bm) % 360.0
    urdf_yaw = math.radians(-bm)

    print("\n=== cach sua ===")
    if min(bm, 360 - bm) < 3.0:
        print(f"  Lech {min(bm, 360-bm):.1f} do - khong dang ke, khong can sua.")
    else:
        print(f"  Anh quet dang bi xoay du {bm:.1f} do. Chon MOT trong hai:")
        print()
        print(f"  (a) Trong URDF - KHONG can nap lai firmware:")
        print(f"      robot.urdf.xacro, joint lidar_joint, doi rpy thanh")
        print(f"          rpy=\"0 0 {urdf_yaw:.4f}\"")
        print(f"      roi colcon build. Day la cho DUNG de mo ta huong gan")
        print(f"      cam bien, va thay duoc ngay trong TF.")
        print()
        print(f"  (b) Trong firmware - phai nap lai (khoi cua DUNG xe, #if XE == ...):")
        print(f"      LIDAR_ANGLE_OFFSET_DEG  {FIRMWARE_OFFSET_NOW:.1f}f "
              f"-> {new_off:.1f}f")
        print()
        print("  Chi lam MOT trong hai, lam ca hai la bu thua gap doi.")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

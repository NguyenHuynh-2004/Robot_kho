#!/usr/bin/env python3
"""Phan tich /scan de tra loi: cac cham den roi rac tren ban do la nhieu LiDAR,
hay la loi khac.

Ba phep do doc lap:
  1. DIEM CO DON  - tia co khoang cach lech han ca hai tia ben canh. Vat the
     that (tuong, chan ban) chiem nhieu tia lien nhau; mot tia don doc lech
     hang met la gia.
  2. KHONG ON DINH THEO THOI GIAN - xe dung yen thi cung mot huong phai cho
     cung mot khoang cach. Bin nao nhay lung tung giua cac vong quet la nhieu.
  3. TI LE BIN TRONG - firmware chia 360 do thanh 320 bin va giu diem GAN
     NHAT trong moi bin. Neu nhieu bin trong thi do phan giai goc dang bi phi.

CHAY KHI XE DANG DUNG YEN. Thuan doc, khong publish gi.
"""
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

N_SCAN = 40
JUMP = 0.30          # m - lech bao nhieu thi coi la "khac han" hai tia ben canh


class S(Node):
    def __init__(self):
        super().__init__('scan_noise')
        self.scans = []
        self.meta = None
        self.create_subscription(
            LaserScan, '/scan', self.cb,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=5))

    def cb(self, m):
        if len(self.scans) < N_SCAN:
            self.scans.append(list(m.ranges))
            self.meta = (m.range_min, m.range_max, len(m.ranges))


def valid(r, lo, hi):
    return lo < r < hi and r != float('inf')


def main():
    rclpy.init()
    n = S()
    end = time.time() + 60
    while time.time() < end and len(n.scans) < N_SCAN:
        rclpy.spin_once(n, timeout_sec=0.1)

    if len(n.scans) < 5:
        print(f"chi nhan duoc {len(n.scans)} vong quet, khong du de phan tich")
        n.destroy_node()
        rclpy.shutdown()
        return

    lo, hi, nbin = n.meta
    print(f"=== {len(n.scans)} vong quet, {nbin} bin, range {lo}-{hi} m ===\n")

    # --- 1. bin trong ---
    empties = [sum(1 for r in s if not valid(r, lo, hi)) for s in n.scans]
    print(f"1. BIN TRONG: trung binh {statistics.mean(empties):.0f}/{nbin} "
          f"= {statistics.mean(empties)/nbin*100:.0f}% moi vong")

    # --- 2. diem co don ---
    lone_tot = pts_tot = 0
    lone_dist = []
    for s in n.scans:
        for i in range(nbin):
            r = s[i]
            if not valid(r, lo, hi):
                continue
            pts_tot += 1
            a, b = s[(i - 1) % nbin], s[(i + 1) % nbin]
            # co don = ca hai ben hoac trong, hoac lech > JUMP
            la = (not valid(a, lo, hi)) or abs(a - r) > JUMP
            lb = (not valid(b, lo, hi)) or abs(b - r) > JUMP
            if la and lb:
                lone_tot += 1
                lone_dist.append(r)
    print(f"2. DIEM CO DON: {lone_tot}/{pts_tot} = {lone_tot/max(pts_tot,1)*100:.1f}% "
          f"so tia hop le")
    if lone_dist:
        d = sorted(lone_dist)
        print(f"   khoang cach cua chung: trung vi {d[len(d)//2]:.2f} m "
              f"(min {d[0]:.2f} / max {d[-1]:.2f})")

    # --- 3. on dinh theo thoi gian ---
    unstable = 0
    considered = 0
    spreads = []
    for i in range(nbin):
        vals = [s[i] for s in n.scans if valid(s[i], lo, hi)]
        if len(vals) < len(n.scans) * 0.5:      # bin hiem khi co diem -> bo qua
            continue
        considered += 1
        spread = max(vals) - min(vals)
        spreads.append(spread)
        if spread > JUMP:
            unstable += 1
    if considered:
        print(f"3. ON DINH: {unstable}/{considered} bin co bien do dao dong "
              f"> {JUMP} m giua cac vong ({unstable/considered*100:.1f}%)")
        sp = sorted(spreads)
        print(f"   bien do dao dong: trung vi {sp[len(sp)//2]*1000:.0f} mm, "
              f"90% duoi {sp[int(len(sp)*0.9)]*1000:.0f} mm, "
              f"lon nhat {sp[-1]*1000:.0f} mm")

    # --- 4. nhieu THEO KHOANG CACH ---
    # Nguoi dung thay diem xa hon ~2.5 m nhay loan. Do bang so de chon nguong
    # max_laser_range cho slam thay vi doan.
    # Do do lech chuan BEN VUNG (MAD * 1.4826) chu khong dung max-min: max-min
    # tang theo so vong quet va bi mot tia rac keo len.
    # Cot "luong tu goc" la sai lech ngang do dat diem vao GIUA bin 1.125 do:
    # d * 0.0098 m. Tu 2.55 m tro di no vuot mot o luoi 5 cm.
    buckets = [(0.0, 1.0), (1.0, 2.0), (2.0, 2.5), (2.5, 3.0),
               (3.0, 4.0), (4.0, 5.0), (5.0, 8.0)]
    rows = {b: [] for b in buckets}
    for i in range(nbin):
        vals = sorted(s[i] for s in n.scans if valid(s[i], lo, hi))
        if len(vals) < len(n.scans) * 0.5:
            continue
        med = vals[len(vals) // 2]
        mad = sorted(abs(v - med) for v in vals)[len(vals) // 2]
        sigma = 1.4826 * mad
        for b in buckets:
            if b[0] <= med < b[1]:
                rows[b].append((sigma, med))
                break
    print("\n4. NHIEU THEO KHOANG CACH (xe phai dung yen)")
    print(f"   {'khoang (m)':>10} {'so bin':>7} {'nhieu doc tia':>14} "
          f"{'% khoang cach':>14} {'luong tu goc':>13}")
    for b in buckets:
        r = rows[b]
        if not r:
            continue
        sg = sorted(x for x, _ in r)[len(r) // 2]
        dm = sorted(d for _, d in r)[len(r) // 2]
        print(f"   {b[0]:4.1f}-{b[1]:<4.1f}  {len(r):7d} {sg*1000:11.0f} mm "
              f"{sg/dm*100:12.1f} % {dm*0.0098*1000:10.0f} mm")

    print("\nDoc ket qua:")
    print("  diem co don < 2%  va  dao dong trung vi < 30 mm  -> LiDAR sach,")
    print("     cham den tren ban do den tu nguyen nhan khac (odometry/goc lap).")
    print("  diem co don > 5%  hoac dao dong lon -> dung la nhieu LiDAR.")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

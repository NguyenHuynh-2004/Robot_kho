#!/usr/bin/env python3
"""Can sa ban theo VACH THAT tren san: dat xe len vai giao diem vach, doc vi
tri AMCL, roi tinh tam / goc / canh o khop nhat.

    cd ~/robot_ws_3 && python3 tools/calib_grid_points.py            # chi do + in
    python3 tools/calib_grid_points.py --write                       # ghi grid_real.yaml
    python3 tools/calib_grid_points.py --write --fit-spacing         # ghi ca canh o

Moi lan: dat TAM XE dung giao diem vach, de yen, nhap "hang cot" (vd: 3 3).
Nen do it nhat 3 diem xa nhau (vd 3 3, 3 5, 5 3, 1 1). Do cung mot diem voi
hai huong xe nguoc nhau (vd dau xe sang Dong roi sang Tay) thi biet them lidar
co gan lech tam xe khong. Enter trong de ket thuc va tinh.

Ket qua cho biet:
  - moi diem do lech diem luoi HIEN TAI bao nhieu (chinh la do lech ban thay
    ngoai san), tach thanh phan tinh tien chung va phan con lai;
  - tam / goc / canh o khop nhat voi cac diem da do, va sai so con lai.
Can navigation.launch.py dang chay va AMCL da dinh vi dung (auto_initial_pose).
"""
import argparse
import math
import os
import re
import sys
import threading
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from grid_board import Board, config_path   # noqa: E402


class PoseReader:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        import tf2_ros
        rclpy.init()
        self.rclpy = rclpy
        self.node = Node('calib_grid_points')
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self.node)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def average(self, seconds=2.0):
        """Trung binh TF map -> base_footprint trong `seconds` giay.
        Tra ve (x, y, yaw_rad, dao_dong_m) hoac None."""
        from rclpy.time import Time
        poses, t_first, end = [], None, time.time() + 6.0 + seconds
        while time.time() < end:
            try:
                t = self.buf.lookup_transform('map', 'base_footprint', Time())
                q = t.transform.rotation
                poses.append((t.transform.translation.x, t.transform.translation.y,
                              math.atan2(2 * (q.w * q.z + q.x * q.y),
                                         1 - 2 * (q.y * q.y + q.z * q.z))))
                t_first = t_first or time.time()
                if time.time() - t_first >= seconds:
                    break
            except Exception:
                pass
            time.sleep(0.1)
        if not poses:
            return None
        p = np.array(poses)
        yaw = math.atan2(np.sin(p[:, 2]).mean(), np.cos(p[:, 2]).mean())
        spread = max(np.ptp(p[:, 0]), np.ptp(p[:, 1]))
        return p[:, 0].mean(), p[:, 1].mean(), yaw, spread

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def fit(board, pts, fit_spacing):
    """pts: [(r, c, x, y)] (r, c tu 0). Tim tam (cx, cy), goc, canh o sao cho
    diem luoi trung diem do nhat (binh phuong nho nhat).
    1 diem: chi tinh tien (giu goc + canh o cu)."""
    G = np.array([[(c - (board.center_col - 1)), (r - (board.center_row - 1))]
                  for r, c, _, _ in pts], float)            # toa do luoi (o)
    P = np.array([[x, y] for _, _, x, y in pts], float)
    if len(pts) == 1 or np.ptp(G, axis=0).max() == 0:
        th, s = board.yaw, board.spacing
    else:
        g0, p0 = G - G.mean(0), P - P.mean(0)
        # Procrustes 2D: goc tu tong tich cheo / tich vo huong
        a = (g0 * p0).sum()
        b = (g0[:, 0] * p0[:, 1] - g0[:, 1] * p0[:, 0]).sum()
        th = math.atan2(b, a)
        if fit_spacing:
            s = (a * math.cos(th) + b * math.sin(th)) / (g0 ** 2).sum()
        else:
            s = board.spacing
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    C = (P - s * G @ R.T).mean(0)
    pred = C + s * G @ R.T
    return C[0], C[1], th, s, P - pred


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=config_path(True))
    ap.add_argument('--write', action='store_true', help='ghi ket qua vao file sa ban')
    ap.add_argument('--fit-spacing', action='store_true',
                    help='tinh ca canh o (mac dinh giu spacing trong file)')
    a = ap.parse_args()

    board = Board.load(a.file)
    print(f"Sa ban hien tai: tam ({board.center_x:+.3f}, {board.center_y:+.3f}), "
          f"goc {math.degrees(board.yaw):+.1f} do, o {board.spacing*100:.1f} cm")
    reader = PoseReader()
    pts = []          # (r, c, x, y, yaw)
    try:
        while True:
            s = input("\nDat tam xe dung giao diem vach, nhap 'hang cot' (Enter trong = tinh): ").strip()
            if not s:
                break
            try:
                r, c = (int(v) for v in s.replace(',', ' ').split())
                if not (1 <= r <= board.rows and 1 <= c <= board.cols):
                    raise ValueError
            except ValueError:
                print(f"  Nhap hai so 1..{board.rows}, vd: 3 3")
                continue
            p = reader.average()
            if p is None:
                print("  Khong doc duoc TF map -> base_footprint (AMCL chay chua?)")
                continue
            x, y, yaw, spread = p
            if spread > 0.02:
                print(f"  Vi tri dao dong {spread*100:.1f} cm - de xe yen roi nhap lai.")
                continue
            gx, gy = board.xy(r - 1, c - 1)
            print(f"  ({r},{c}): AMCL ({x:+.3f}, {y:+.3f}) huong {math.degrees(yaw):+.0f} do"
                  f" | lech diem luoi hien tai {math.hypot(x-gx, y-gy)*100:.1f} cm"
                  f" (dx {(x-gx)*100:+.1f}, dy {(y-gy)*100:+.1f})")
            pts.append((r - 1, c - 1, x, y, yaw))
    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        reader.close()

    if not pts:
        return
    # Lech so voi sa ban hien tai, tach phan tinh tien chung
    d = np.array([[x - board.xy(r, c)[0], y - board.xy(r, c)[1]] for r, c, x, y, _ in pts])
    m = d.mean(0)
    print(f"\n=== So voi sa ban hien tai ({len(pts)} lan do) ===")
    print(f"Tinh tien chung: dx {m[0]*100:+.1f} cm, dy {m[1]*100:+.1f} cm "
          f"(= {np.hypot(*m)*100:.1f} cm)")
    print(f"Con lai sau khi bo tinh tien: lon nhat {np.hypot(*(d - m).T).max()*100:.1f} cm")

    # Cung mot diem, hai huong khac nhau -> lidar lech tam xe
    by = {}
    for r, c, x, y, yaw in pts:
        by.setdefault((r, c), []).append((x, y, yaw))
    for (r, c), L in by.items():
        for i in range(len(L)):
            for j in range(i + 1, len(L)):
                (x1, y1, w1), (x2, y2, w2) = L[i], L[j]
                dw = abs(math.degrees(math.atan2(math.sin(w1 - w2), math.cos(w1 - w2))))
                if dw > 120:
                    print(f"({r+1},{c+1}) do 2 huong nguoc nhau: hai vi tri cach "
                          f"{math.hypot(x1-x2, y1-y2)*100:.1f} cm -> lidar/tam xe lech "
                          f"~{math.hypot(x1-x2, y1-y2)*50:.1f} cm (khong tinh lan dat xe lech tay)")

    cx, cy, th, s, res = fit(board, [(r, c, x, y) for r, c, x, y, _ in pts], a.fit_spacing)
    print(f"\n=== Sa ban khop nhat ===")
    print(f"tam ({cx:+.3f}, {cy:+.3f})  [cu ({board.center_x:+.3f}, {board.center_y:+.3f}), "
          f"doi {math.hypot(cx-board.center_x, cy-board.center_y)*100:.1f} cm]")
    print(f"goc {math.degrees(th):+.1f} do  [cu {math.degrees(board.yaw):+.1f}]"
          + ("" if len(by) > 1 else "  (1 diem: giu goc cu)"))
    print(f"canh o {s*100:.1f} cm" + ("" if a.fit_spacing else " (giu nguyen, --fit-spacing de tinh)"))
    for (r, c, x, y, _), e in zip(pts, res):
        print(f"  ({r+1},{c+1}) con lech {np.hypot(*e)*100:.1f} cm")

    if not a.write:
        print("\nChua ghi gi. Them --write de ghi vao", a.file)
        return
    text = open(a.file).read()
    vals = [('center_x', f"{cx:.3f}"), ('center_y', f"{cy:.3f}")]
    if len(by) > 1:
        vals.append(('yaw_deg', f"{math.degrees(th):.1f}"))
    if a.fit_spacing:
        vals.append(('spacing', f"{s:.3f}"))
    for key, val in vals:
        text, n = re.subn(rf'^({key}:\s*)[-+0-9.eE]+', lambda mm: f"{mm.group(1)}{val}", text, flags=re.M)
        if n == 0 and key == 'yaw_deg':
            text, n = re.subn(r'^(center_y:.*\n)', lambda mm: mm.group(1) + f"yaw_deg: {val}\n",
                              text, count=1, flags=re.M)
        if n != 1:
            sys.exit(f'Khong ghi duoc "{key}" vao {a.file} - khong ghi gi.')
    open(a.file, 'w').write(text)
    Board.load(a.file)
    print(f"Da ghi {a.file}. Chay lai grid_navigator.py --real.")


if __name__ == '__main__':
    main()

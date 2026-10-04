#!/usr/bin/env python3
"""Doi tam sa ban xe that sang vi tri xe HIEN TAI (hoac toa do nhap tay).

    cd ~/robot_ws_3 && python3 tools/set_grid_center.py           # theo xe
    python3 tools/set_grid_center.py --x -0.55 --y 0.04           # nhap tay
    python3 tools/set_grid_center.py --dry-run                    # chi xem
    python3 tools/set_grid_center.py --yaw-walls                  # + xoay luoi theo tuong
    python3 tools/set_grid_center.py --x -0.55 --y 0.04 --yaw -5  # + goc nhap tay (do)

Goc luoi (yaw_deg): ban do SLAM co huong tuy y nen tuong / vach gach that
thuong lech vai do so voi truc x/y ban do. --yaw-walls do goc tuong tren
real_room_map (vach gach song song tuong) va xoay sa ban cho khop.

Theo xe: doc TF map -> base_footprint (can localization.launch.py hoac
navigation.launch.py dang chay va AMCL da dung cho), lay trung binh 2 giay.
Ghi center_x / center_y vao config/grid_real.yaml (giu nguyen chu thich),
roi ve lai docs/sa_ban_5x5_preview.png.

Sa ban CO DINH: sau khi doi, cac diem nam co dinh tren ban do cho toi lan doi
sau - xe di dau cung khong keo sa ban theo.
"""
import argparse
import math
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from grid_board import Board, config_path   # noqa: E402


def robot_pose(seconds=2.0):
    import threading
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    import tf2_ros
    rclpy.init()
    n = Node('set_grid_center')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    poses, end = [], time.time() + 8.0 + seconds
    t_first = None
    while time.time() < end:
        try:
            t = buf.lookup_transform('map', 'base_footprint', Time())
            q = t.transform.rotation
            poses.append((t.transform.translation.x, t.transform.translation.y,
                          math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
            t_first = t_first or time.time()
            if time.time() - t_first >= seconds:
                break
        except Exception:
            pass
        time.sleep(0.1)
    rclpy.shutdown()
    if not poses:
        return None
    xs, ys, ths = zip(*poses)
    spread = max(max(xs) - min(xs), max(ys) - min(ys))
    return sum(xs) / len(xs), sum(ys) / len(ys), math.degrees(ths[-1]), spread


def wall_angle(map_yaml):
    """Goc (do, -45..45) cua tuong tren ban do so voi truc x: goc lam hinh
    chieu cac o tuong len hai truc vuong goc sac net nhat."""
    import numpy as np
    import yaml
    from PIL import Image
    m = yaml.safe_load(open(map_yaml))
    img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), m['image'])))
    res = m['resolution']
    ys, xs = np.nonzero(img < 50)
    X, Y = xs * res, -ys * res

    def score(a):
        c, s = np.cos(a), np.sin(a)
        tot = 0.0
        for p in (X * c + Y * s, -X * s + Y * c):
            h = np.histogram(p, bins=np.arange(p.min(), p.max() + 2 * res, res))[0]
            tot += float((h.astype(float) ** 2).sum())
        return tot

    angs = np.radians(np.arange(-45, 45, 0.1))
    sc = [score(a) for a in angs]
    k = int(np.argmax(sc))
    return math.degrees(angs[k]), sc[k] / max(score(0.0), 1e-9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float)
    ap.add_argument('--y', type=float)
    ap.add_argument('--yaw', type=float, help='goc luoi (do) so voi truc x ban do')
    ap.add_argument('--yaw-walls', action='store_true',
                    help='do goc luoi tu tuong tren config/real_room_map.yaml')
    ap.add_argument('--keep-center', action='store_true', help='chi doi goc, giu tam cu')
    ap.add_argument('--file', default=config_path(True))
    ap.add_argument('--dry-run', action='store_true', help='chi in, khong ghi file')
    a = ap.parse_args()

    old = Board.load(a.file)
    yaw = None
    if (a.x is None) != (a.y is None):
        sys.exit('Nhap ca --x va --y, hoac khong nhap gi de lay theo xe.')
    grid_yaw = a.yaw
    if a.yaw_walls:
        map_yaml = os.path.join(ROOT, 'src', 'my_bot_bringup', 'config', 'real_room_map.yaml')
        grid_yaw, gain = wall_angle(map_yaml)
        print(f"Tuong tren ban do lech {grid_yaw:+.1f} do so voi truc x "
              f"(khop hon goc 0 do {gain:.2f} lan)")
    if a.keep_center:
        x, y = old.center_x, old.center_y
    elif a.x is not None:
        x, y = a.x, a.y
    else:
        p = robot_pose()
        if p is None:
            sys.exit('Khong doc duoc TF map -> base_footprint. Chay localization.launch.py '
                     'use_sim_time:=false truoc va dat vi tri ban dau cho AMCL '
                     '(tools/auto_initial_pose.py).')
        x, y, yaw, spread = p
        if spread > 0.05:
            sys.exit(f'Vi tri xe dao dong {spread * 100:.0f} cm trong 2 giay - de xe dung yen roi chay lai.')
        print(f"Xe dang o ({x:+.3f}, {y:+.3f}), huong {yaw:+.0f} do")

    print(f"Tam cu: ({old.center_x:+.3f}, {old.center_y:+.3f})  ->  tam moi: ({x:+.3f}, {y:+.3f})")
    if grid_yaw is not None:
        print(f"Goc luoi cu: {math.degrees(old.yaw):+.1f} do  ->  moi: {grid_yaw:+.1f} do")
    if a.dry_run:
        return
    text = open(a.file).read()
    for key, val in (('center_x', x), ('center_y', y)):
        text, n = re.subn(rf'^({key}:\s*)[-+0-9.eE]+', lambda m: f"{m.group(1)}{val:.3f}", text, flags=re.M)
        if n != 1:
            sys.exit(f'Khong tim thay dong "{key}:" trong {a.file} - khong ghi gi.')
    if grid_yaw is not None:
        text, n = re.subn(r'^(yaw_deg:\s*)[-+0-9.eE]+', lambda m: f"{m.group(1)}{grid_yaw:.1f}",
                          text, flags=re.M)
        if n == 0:
            text, n = re.subn(r'^(center_y:.*\n)',
                              lambda m: m.group(1) + "# Goc truc sa ban so voi truc x ban do (do), "
                              f"tools/set_grid_center.py --yaw-walls\nyaw_deg: {grid_yaw:.1f}\n",
                              text, count=1, flags=re.M)
        if n != 1:
            sys.exit(f'Khong ghi duoc yaw_deg vao {a.file}.')
    open(a.file, 'w').write(text)
    Board.load(a.file)          # doc lai de chac file van hop le
    print(f"Da ghi {a.file}")

    cmd = [sys.executable, os.path.join(ROOT, 'tools', 'preview_grid.py'), '--grid', a.file]
    if os.path.abspath(a.file) != os.path.abspath(config_path(True)):
        # File khac file mac dinh: dung de anh cua no de len anh trong docs/
        cmd += ['-o', os.path.splitext(a.file)[0] + '_preview.png']
    if yaw is not None:
        cmd += ['--robot', f'{x}', f'{y}', f'{yaw}']
    subprocess.run(cmd, check=False)
    print("Chay lai grid_navigator.py --real de dung sa ban moi.")


if __name__ == '__main__':
    main()

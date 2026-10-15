#!/usr/bin/env python3
"""Do goc XOAY cua camera so voi truc xe (camera lap khong thang) -> `yaw` trong
camera_<xe>.yaml. Camera xoay 3-4 do lam vach LiDAR lech ~5 cm so voi hop trong anh
va do sai be rong (2026-10-15, xe 2: rong 22.5 thay vi 18.5 cm).

    python3 tools/cam_extrinsic.py agv2              # can he dang chay (scan + anh camera)
    python3 tools/cam_extrinsic.py agv2 --dry-run    # chi in, khong ghi file
    python3 tools/cam_extrinsic.py agv2 --height 0.320   # them goc NGOC (pitch) tu hop biet cao

Chuan bi: xe DUNG YEN, mot hop CAO HON tia LiDAR (> ~25 cm) dat THANG, CHINH GIUA truoc xe,
cach 0.6-0.9 m, nen phia sau tron. Script lay 5 lan (moi lan 10 vong quet + 1 anh),
lay trung vi, ghi `yaw:` (do, + = camera quay sang trai).
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cargo_measure as cm   # noqa: E402


def write_yaw(car, yaw, pitch=None):
    path = cm.calib_path(car)
    lines = open(path).read().splitlines() if os.path.exists(path) else []
    drop = ('yaw:',) + (('pitch:',) if pitch is not None else ())
    lines = [ln for ln in lines if not ln.startswith(drop) and 'tools/cam_extrinsic.py' not in ln]
    lines.append(f'# Goc xoay (yaw, + = trai) / ngoc (pitch, + = len) camera, do - tools/cam_extrinsic.py '
                 f'{time.strftime("%Y-%m-%d %H:%M")}')
    lines.append(f'yaw: {yaw:.2f}')
    if pitch is not None:
        lines.append(f'pitch: {pitch:.2f}')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('car')
    ap.add_argument('--n', type=int, default=5)
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--height', type=float, help='chieu cao THAT cua hop (m) -> do them goc ngoc')
    a = ap.parse_args()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    from tf2_msgs.msg import TFMessage  # noqa: F401  (lidar_x lay tu cargo.yaml)
    cfg = cm.car_cfg(cm.load_cfg(), a.car)
    rclpy.init()
    node = rclpy.create_node('cam_extrinsic')
    scans, img = [], {}
    node.create_subscription(LaserScan, f'/{a.car}/scan',
                             lambda m: scans.append((time.time(), (m.angle_min, m.angle_increment, list(m.ranges)))),
                             qos_profile_sensor_data)
    node.create_subscription(Image, f'/{a.car}/camera/image_raw', lambda m: img.__setitem__('m', (time.time(), m)), 2)
    print(f'{a.car}: camera hien tai yaw {cfg["camera"].get("yaw", 0.0)} do. Lay {a.n} lan...')
    res = []
    for k in range(a.n):
        t0 = time.time()
        while time.time() - t0 < 15:
            rclpy.spin_once(node, timeout_sec=0.05)
            fresh = [x for t, x in scans if t > t0]
            if len(fresh) >= cfg['lidar']['scans'] and 'm' in img and img['m'][0] > t0:
                break
        fresh = [x for t, x in scans if t > t0][-cfg['lidar']['scans']:]
        if 'm' not in img or len(fresh) < 5:
            sys.exit('Khong du anh quet / anh camera - he (fleet_real.sh up) dang chay chua?')
        m = img['m'][1]
        frame = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        try:
            r = cm.estimate_yaw(frame, fresh, cfg)
        except ValueError as e:
            print(f'  lan {k + 1}: LOI {e}')
            continue
        r['frame'], r['scans'] = frame, fresh
        res.append(r)
        print(f'  lan {k + 1}: yaw {r["yaw_deg"]:+.2f} do (hop cach {r["dist"] * 100:.0f} cm, mat lech '
              f'{r["face_yaw_deg"]:+.1f} do, giua hop: LiDAR u={r["u_lidar"]:.0f} / anh u={r["u_cam"]:.0f})')
    node.destroy_node()
    rclpy.try_shutdown()
    if len(res) < 3:
        sys.exit('Duoi 3 lan do duoc - kiem tra hop (cao hon LiDAR, chinh giua, nen tron)')
    yaws = np.array([r['yaw_deg'] for r in res])
    yaw = float(np.median(yaws))
    print(f'\nyaw = {yaw:+.2f} do (lech giua cac lan {yaws.max() - yaws.min():.2f} do)')
    if abs(np.median([r['face_yaw_deg'] for r in res])) > 5:
        print('CANH BAO: mat hop lech > 5 do so voi xe - dat hop thang lai cho chinh xac hon')
    pitch = None
    if a.height:
        # mep tren hop (da bu yaw, pitch 0) -> goc ngoc de tia qua mep tren cat mat hop dung o
        # do cao that: z = cam_z + t*(h cos p + f sin p) ... giai bang chia doi
        import math
        pitches = []
        for r in res:
            c = dict(cfg)
            c['camera'] = dict(cfg['camera'], yaw=yaw)
            c['camera']['pitch'] = 0.0     # che do cao toi san: chi can mep tren, bo qua mep duoi (bong san)
            try:
                m, _ = cm.measure(r['frame'], r['scans'], c)
            except ValueError as e:
                print(f'  pitch: bo mot lan ({e})')
                continue
            ua, ub, va, _ = m['edges_px']
            n, d = np.array(m['face_n']), m['face_d']
            lo, hi = -10.0, 10.0
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                c['camera']['pitch'] = mid
                z = cm.Camera(c).ray_plane(0.5 * (ua + ub), va, n, d)[2]
                if z < a.height:
                    lo = mid      # tia qua mep tren con thap -> camera phai ngoc len hon
                else:
                    hi = mid
            pitches.append(0.5 * (lo + hi))
            print(f'  pitch: {pitches[-1]:+.2f} do (cao do duoc khi pitch 0: {m["height"] * 100:.1f} cm)')
        if len(pitches) >= 3:
            pitch = float(np.median(pitches))
            print(f'pitch = {pitch:+.2f} do (lech giua cac lan {max(pitches) - min(pitches):.2f} do)')
        else:
            print('Khong du lan do pitch - chi ghi yaw')
    if a.dry_run:
        return
    print(f'Da ghi {write_yaw(a.car, yaw, pitch)}')


if __name__ == '__main__':
    main()

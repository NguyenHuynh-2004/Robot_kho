#!/usr/bin/env python3
"""Hieu chuan camera ESP32-CAM cua mot xe bang ban co -> camera_<xe>.yaml.

    python3 tools/cam_calib.py agv1                      # can tools/esp32_cam_node.py dang chay
    python3 tools/cam_calib.py agv1 --board 9x6 --square 0.025 --n 15
    python3 tools/cam_calib.py agv1 --dir ~/agv_logs/calib/agv1     # tinh lai tu anh da luu
    python3 tools/cam_calib.py agv1 --ost /tmp/calibrationdata.tar.gz # nhap ket qua cua
        ros2 run camera_calibration cameracalibrator (giao dien ROS, nut SAVE -> ost.yaml)

Ban co: in trang "9x6 inner corners" (10x7 o vuong) len A4, dan phang len bia
cung, DO canh o vuong bang thuoc (--square, m). Cam ban co truoc camera, xoay /
nghieng / dua ra cac goc anh; moi lan ban co dung yen va KHAC vi tri truoc do ro
rang thi script tu lay mot anh (luu ~/agv_logs/calib/<xe>/). Du --n anh thi tinh
K + he so meo, ghi src/my_bot_bringup/config/camera_<xe>.yaml (de len camera:
trong cargo.yaml cho rieng xe do). Sai so (RMS) nen < 0.5 px; > 1 px thi chup lai.

Vi tri lap camera (x truoc truc banh, y trai, z cao - m) KHONG hieu chuan duoc:
do bang thuoc roi sua tay trong camera_<xe>.yaml (script giu nguyen neu da co).
"""
import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cargo_measure as cm   # noqa: E402


def detect(img, size):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, c = cv2.findChessboardCorners(gray, size, cv2.CALIB_CB_ADAPTIVE_THRESH +
                                      cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)
    if not ok:
        return None
    return cv2.cornerSubPix(gray, c, (5, 5), (-1, -1),
                            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))


def calibrate(views, size, square, wh):
    obj = np.zeros((size[0] * size[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2) * square
    rms, K, D, rv, tv = cv2.calibrateCamera([obj] * len(views), views, wh, None, None)
    errs = []
    for c, r, t in zip(views, rv, tv):
        p, _ = cv2.projectPoints(obj, r, t, K, D)
        errs.append(float(np.sqrt(np.mean(np.sum((p - c) ** 2, axis=2)))))
    return rms, K, D.ravel(), errs


def read_ost(path):
    """ost.yaml cua camera_calibration (file .yaml hoac /tmp/calibrationdata.tar.gz)
    -> (K 3x3, D, (w, h))."""
    path = os.path.expanduser(path)
    if path.endswith(('.tar.gz', '.tgz')):
        import tarfile
        with tarfile.open(path) as t:
            name = next((m for m in t.getnames() if m.endswith('ost.yaml')), None)
            if name is None:
                sys.exit(f'{path}: khong co ost.yaml (da bam CALIBRATE roi SAVE chua?)')
            d = yaml.safe_load(t.extractfile(name).read())
    else:
        with open(path) as f:
            d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], float).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], float)
    if d.get('distortion_model', 'plumb_bob') != 'plumb_bob':
        sys.exit(f'distortion_model {d.get("distortion_model")} khong ho tro (can plumb_bob)')
    return K, D, (int(d['image_width']), int(d['image_height']))


def write_yaml(car, K, D, wh, rms, n):
    path = cm.calib_path(car)
    old, text = {}, ''
    if os.path.exists(path):
        with open(path) as f:
            text = f.read()
        old = yaml.safe_load(text) or {}
    base = cm.load_cfg()['camera']
    x, y, z = (old.get(k, base[k]) for k in ('x', 'y', 'z'))
    measured = bool(old) and '!!! CHUA DO' not in text
    d = [float(v) for v in D[:5]] + [0.0] * max(0, 5 - len(D))
    with open(path, 'w') as f:
        src = f'{n} anh, RMS {rms:.2f} px' if rms is not None else 'nhap tu ost.yaml (camera_calibration)'
        f.write(f'# Camera {car} - tools/cam_calib.py {time.strftime("%Y-%m-%d %H:%M")}, {src}.\n')
        f.write('# De len camera: trong cargo.yaml cho rieng xe nay (cargo_measure.car_cfg).\n')
        f.write(f'width: {wh[0]}\nheight: {wh[1]}\n')
        f.write(f'fx: {K[0, 0]:.2f}\nfy: {K[1, 1]:.2f}\ncx: {K[0, 2]:.2f}\ncy: {K[1, 2]:.2f}\n')
        f.write('dist: [' + ', '.join(f'{v:.5f}' for v in d) + ']\n')
        f.write('# Vi tri tam ong kinh so voi TAM TRUC BANH (m): x truoc, y trai, z cao tu san.\n')
        if not measured:
            f.write('# !!! CHUA DO - so mac dinh. Do bang thuoc roi sua 3 dong duoi.\n')
        f.write(f'x: {x}\ny: {y}\nz: {z}\n')
    return path, measured


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('car')
    ap.add_argument('--board', default='9x6', help='so goc TRONG cot x hang (mac dinh 9x6)')
    ap.add_argument('--square', type=float, default=0.025, help='canh o vuong (m), do bang thuoc')
    ap.add_argument('--n', type=int, default=15)
    ap.add_argument('--dir', help='tinh tu anh da luu, khong can ROS')
    ap.add_argument('--ost', help='nhap ost.yaml / calibrationdata.tar.gz cua camera_calibration')
    a = ap.parse_args()
    if a.ost:
        K, D, wh = read_ost(a.ost)
        hfov = 2 * np.degrees(np.arctan(wh[0] / 2 / K[0, 0]))
        print(f'{a.ost}: {wh[0]}x{wh[1]}, fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} cx {K[0, 2]:.1f} '
              f'cy {K[1, 2]:.1f}, dist [{", ".join(f"{v:.4f}" for v in D[:5])}], goc ngang {hfov:.1f} do')
        path, measured = write_yaml(a.car, K, D, wh, None, 0)
        print(f'Da ghi {path}' + ('' if measured else ' - !!! sua x, y, z (vi tri camera) trong file do'))
        return
    size = tuple(int(v) for v in a.board.lower().split('x'))
    out = os.path.expanduser(f'~/agv_logs/calib/{a.car}')
    views, wh = [], None

    if a.dir:
        for p in sorted(glob.glob(os.path.join(os.path.expanduser(a.dir), '*.png'))):
            img = cv2.imread(p)
            c = detect(img, size)
            if c is not None:
                views.append(c)
                wh = (img.shape[1], img.shape[0])
        print(f'{len(views)} anh co ban co trong {a.dir}')
    else:
        import rclpy
        from sensor_msgs.msg import Image
        os.makedirs(out, exist_ok=True)
        rclpy.init()
        node = rclpy.create_node('cam_calib')
        latest = {}
        node.create_subscription(Image, f'/{a.car}/camera/image_raw', lambda m: latest.__setitem__('m', m), 2)
        print(f'Cho anh /{a.car}/camera/image_raw ... (ban co {size[0]}x{size[1]} goc trong, '
              f'o {a.square * 100:.1f} cm). Ctrl-C de dung.')
        last_c, still, last_seen, t0 = None, 0, None, time.time()
        try:
            while len(views) < a.n:
                rclpy.spin_once(node, timeout_sec=0.1)
                m = latest.pop('m', None)
                if m is None:
                    if time.time() - t0 > 10 and wh is None:
                        print('  chua co anh - esp32_cam_node.py da chay chua?')
                        t0 = time.time()
                    continue
                img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).copy()
                wh = (m.width, m.height)
                c = detect(img, size)
                if c is None:
                    still = 0
                    continue
                # ban co dung yen 2 anh lien (khong nhoe) va khac ro cac anh da lay
                moved = 1e9 if last_seen is None else float(np.mean(np.linalg.norm(c - last_seen, axis=2)))
                last_seen = c
                still = still + 1 if moved < 2.0 else 0
                if still < 1:
                    continue
                novel = min((float(np.mean(np.linalg.norm(c - v, axis=2))) for v in views), default=1e9)
                if novel < 40:
                    continue
                views.append(c)
                cv2.imwrite(os.path.join(out, f'{len(views):02d}.png'), img)
                xs, ys = c[:, 0, 0], c[:, 0, 1]
                print(f'  lay anh {len(views)}/{a.n}: ban co o x {xs.min():.0f}-{xs.max():.0f}, '
                      f'y {ys.min():.0f}-{ys.max():.0f} px')
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.try_shutdown()

    if len(views) < 8:
        sys.exit(f'Chi co {len(views)} anh - can it nhat 8 (nen 15)')
    rms, K, D, errs = calibrate(views, size, a.square, wh)
    print(f'\nRMS {rms:.3f} px ({len(views)} anh); anh te nhat {max(errs):.2f} px')
    print(f'fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} cx {K[0, 2]:.1f} cy {K[1, 2]:.1f} '
          f'dist [{", ".join(f"{v:.4f}" for v in D[:5])}]')
    hfov = 2 * np.degrees(np.arctan(wh[0] / 2 / K[0, 0]))
    print(f'Goc ngang {hfov:.1f} do')
    if rms > 1.0:
        print('CANH BAO: RMS > 1 px - ban co cong / nhoe / do sai --square? Chup lai.')
    path, measured = write_yaml(a.car, K, D, wh, rms, len(views))
    print(f'Da ghi {path}')
    if not measured:
        print('!!! Sua x, y, z (vi tri camera so voi tam truc banh, m) trong file do bang thuoc.')


if __name__ == '__main__':
    main()

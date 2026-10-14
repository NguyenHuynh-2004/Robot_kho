#!/usr/bin/env python3
"""Lay anh ESP32-CAM cua tung xe qua Wi-Fi -> /agvN/camera/image_raw (bgr8).

    python3 tools/esp32_cam_node.py                  # ca 2 xe theo cargo.yaml cameras:
    python3 tools/esp32_cam_node.py agv1             # mot xe
    python3 tools/esp32_cam_node.py agv1 --url agv1=http://192.168.1.210:81/stream

Moi camera MOT luong rieng chi giu anh MOI NHAT (giong esp32_cam_node.py cua do an
truoc), timer phat ra ROS theo `rate`. Hai che do (cargo.yaml cameras.<xe>):
  url: ...     : doc LUONG MJPEG o dung dia chi do (firmware stream co san tren
                 ESP32-CAM, vd http://192.168.1.200:81/stream). Dang dung.
  mode capture : GET http://<host>/capture moi 1/rate s (can firmware/esp32_cam) - nhe Wi-Fi.
  mode stream  : luong http://<host>:81/stream.
Doc luong thi ESP32-CAM gui lien tuc (~1-4 Mbit/s moi xe tuy co anh) - chung Wi-Fi voi
micro-ROS cua LiDAR; thay LiDAR/odom giat thi giam do phan giai/chat luong o camera.
Mat ket noi (xe tat nguon, ESP32-CAM sut ap) thi tu ket noi lai moi 2 s, bao mot lan.
rotate: 0/90/180/270 - camera lap nguoc thi dat 180 (hoac bat vflip+hmirror trong firmware).
port / stream_port: mac dinh 80 / 81 (doi khi thu voi ESP32-CAM gia tren laptop).
"""
import os
import sys
import threading
import time
import urllib.request

# FFmpeg trong OpenCV in moi lan noi lai luong that bai ("Connection refused") - tat,
# node tu bao mat ket noi mot lan.
os.environ.setdefault('OPENCV_FFMPEG_LOGLEVEL', '-8')
os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')
import cv2  # noqa: E402
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cargo_measure as cm   # noqa: E402

ROT = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}


class Camera:
    def __init__(self, node, car, host=None, mode='capture', rate=3.0, rotate=0, port=80, stream_port=81,
                 url=None):
        self.node, self.car, self.host, self.full_url = node, car, host, url
        if url:
            mode = 'capture' if url.rstrip('/').endswith('/capture') else 'stream'
        self.port, self.stream_port = int(port), int(stream_port)
        self.mode, self.rate, self.rotate = mode, float(rate), int(rotate)
        self.pub = node.create_publisher(Image, f'/{car}/camera/image_raw', 2)
        self.lock = threading.Lock()
        self.frame, self.frame_t, self.sent_t = None, 0.0, 0.0
        self.online = None
        self.count, self.bytes = 0, 0
        self.running = True
        target = self.capture_loop if mode == 'capture' else self.stream_loop
        threading.Thread(target=target, daemon=True).start()

    def url(self):
        if self.full_url:
            return self.full_url
        if self.mode == 'capture':
            return f'http://{self.host}:{self.port}/capture'
        return f'http://{self.host}:{self.stream_port}/stream'

    def set_online(self, ok, why=''):
        if ok != self.online:
            self.online = ok
            log = self.node.get_logger()
            if ok:
                log.info(f'[{self.car}] camera {self.url()} DA KET NOI')
            else:
                log.warn(f'[{self.car}] camera {self.url()} KHONG KET NOI ({why}) - thu lai moi 2 s')

    def store(self, img, nbytes):
        if self.rotate in ROT:
            img = cv2.rotate(img, ROT[self.rotate])
        with self.lock:
            self.frame, self.frame_t = img, time.time()
            self.count += 1
            self.bytes += nbytes

    def capture_loop(self):
        period = 1.0 / max(self.rate, 0.1)
        while self.running:
            t0 = time.time()
            try:
                with urllib.request.urlopen(self.url(), timeout=2.0) as r:
                    data = r.read()
                img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    raise ValueError('anh JPEG hong')
                self.store(img, len(data))
                self.set_online(True)
            except Exception as e:            # noqa: BLE001 - mang/HTTP: thu lai
                self.set_online(False, str(e)[:80])
                time.sleep(2.0)
                continue
            time.sleep(max(0.0, period - (time.time() - t0)))

    def stream_loop(self):
        while self.running:
            params = []
            for name, val in (('CAP_PROP_OPEN_TIMEOUT_MSEC', 3000), ('CAP_PROP_READ_TIMEOUT_MSEC', 3000)):
                if hasattr(cv2, name):
                    params += [getattr(cv2, name), val]
            cap = cv2.VideoCapture(self.url(), cv2.CAP_FFMPEG, params)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                self.set_online(False, 'khong mo duoc luong')
                time.sleep(2.0)
                continue
            while self.running:
                ok, img = cap.read()
                if not ok:
                    self.set_online(False, 'luong bi ngat')
                    break
                self.store(img, 0)
                self.set_online(True)
            cap.release()
            time.sleep(2.0)

    def publish(self):
        with self.lock:
            if self.frame is None or self.frame_t <= self.sent_t:
                return
            img, self.sent_t = self.frame, self.frame_t
        m = Image()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = f'{self.car}/camera_link'
        m.height, m.width = img.shape[:2]
        m.encoding, m.step = 'bgr8', img.shape[1] * 3
        m.data = img.tobytes()
        self.pub.publish(m)

    def stats(self):
        with self.lock:
            n, b = self.count, self.bytes
            self.count = self.bytes = 0
            shape = None if self.frame is None else self.frame.shape
        return n, b, shape


class Esp32CamNode(Node):
    def __init__(self, cams):
        super().__init__('esp32_cam_node')
        self.cams = [Camera(self, **c) for c in cams]
        rate = max(c.rate for c in self.cams)
        self.create_timer(1.0 / rate, lambda: [c.publish() for c in self.cams])
        self.create_timer(10.0, self.report)
        for c in self.cams:
            self.get_logger().info(f'[{c.car}] {c.mode} {c.url()} -> /{c.car}/camera/image_raw '
                                   f'({c.rate:.1f} Hz, xoay {c.rotate})')

    def report(self):
        parts = []
        for c in self.cams:
            n, b, shape = c.stats()
            if c.online:
                size = f'{shape[1]}x{shape[0]}' if shape else '?'
                kb = f', {b / 10 / 1024:.0f} KB/s' if b else ''
                parts.append(f'{c.car} {n / 10:.1f} anh/s {size}{kb}')
            else:
                parts.append(f'{c.car} MAT KET NOI')
        self.get_logger().info('Camera: ' + ' | '.join(parts))

    def destroy_node(self):
        for c in self.cams:
            c.running = False
        super().destroy_node()


def main():
    rclpy.init(args=sys.argv)
    argv = remove_ros_args(sys.argv)[1:]
    conf = cm.load_cfg().get('cameras', {})
    over = {}
    for flag in ('--host', '--url', '--mode', '--rate', '--rotate', '--port', '--stream_port'):
        while flag in argv:
            i = argv.index(flag)
            val = argv[i + 1]
            argv = argv[:i] + argv[i + 2:]
            if '=' in val:
                car, v = val.split('=', 1)
                over.setdefault(car, {})[flag[2:]] = v
            else:
                over.setdefault('*', {})[flag[2:]] = val
    cars = argv or sorted(conf)
    cams = []
    for car in cars:
        c = dict(conf.get(car, {}))
        c.update(over.get('*', {}))
        c.update(over.get(car, {}))
        if 'host' not in c and 'url' not in c:
            sys.exit(f'{car}: chua co url/host trong cargo.yaml cameras: (hoac --url {car}=http://...)')
        cams.append({'car': car, 'host': c.get('host'), 'url': c.get('url'), 'mode': c.get('mode', 'capture'),
                     'rate': float(c.get('rate', 3.0)), 'rotate': int(c.get('rotate', 0)),
                     'port': int(c.get('port', 80)), 'stream_port': int(c.get('stream_port', 81))})
    node = Esp32CamNode(cams)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        # luong stream co the dang ket trong FFmpeg (C++): thoat thang, khong cho no
        os._exit(0)


if __name__ == '__main__':
    main()

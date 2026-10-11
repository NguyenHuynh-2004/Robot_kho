#!/usr/bin/env python3
"""Nhiem vu DO HANG cho doi xe: toi o (5,c) quay Bac -> do rong x cao hang phia
truoc (cargo_measure.py: LiDAR + camera) -> phan loai (cargo.yaml classes) ->
chay ve o (1,k) tuong ung. Xe KHONG cho hang, chi chay ve o.

    python3 cargo_task.py agv1 agv2              # chay cung traffic_manager.py
    > 1 3          xe 1 do hang o (5,3)
    > 2 4          xe 2 do hang o (5,4) (hai xe lam song song duoc)
Ngoai terminal: ros2 topic pub --once /fleet/cargo std_msgs/msg/String "{data: '1 3'}"

Di chuyen giao cho traffic_manager qua /fleet/goal (van giu cho, ne xe kia).
Ket qua: in ra, /fleet/cargo_result (JSON), anh chu thich + results.csv trong
~/agv_logs/cargo/, chu kich thuoc tren o dich trong RViz (/fleet/markers).
"""
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import ColorRGBA, String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

import cargo_measure as cm
from grid_board import Board, config_path

OUT = os.path.expanduser('~/agv_logs/cargo')
ARRIVE_D = 0.05            # m: coi la da toi o
ARRIVE_YAW = math.radians(5)
SETTLE = 1.0               # s dung yen truoc khi do
TIMEOUT = 180.0            # s cho moi chang


class Task:
    def __init__(self, car, col):
        self.car, self.col = car, col
        self.state = 'goto5'
        self.t = 0.0             # luc vao trang thai hien tai
        self.target = None       # (r, c) tu 0
        self.slot = None
        self.result = None
        self.tries = 0


class CargoTask(Node):
    def __init__(self, cars):
        super().__init__('cargo_task')
        self.cfg = cm.load_cfg()
        self.board = Board.load(config_path(True))
        self.cars = cars
        self.status, self.scans, self.img, self.lidar_x = {}, {}, {}, {}
        self.static_tf = {c: {} for c in cars}
        self.tasks = {}
        self.goal_pub = self.create_publisher(String, '/fleet/goal', 10)
        self.res_pub = self.create_publisher(String, '/fleet/cargo_result', 10)
        self.mk_pub = self.create_publisher(MarkerArray, '/fleet/markers', 10)
        self.create_subscription(String, '/fleet/status', self.cb_status, 50)
        self.create_subscription(String, '/fleet/cargo', lambda m: self.command(m.data), 10)
        self.create_subscription(Log, '/rosout', self.cb_rosout, 100)
        static = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        for c in cars:
            self.scans[c] = []
            self.create_subscription(LaserScan, f'/{c}/scan', lambda m, c=c: self.cb_scan(c, m),
                                     qos_profile_sensor_data)
            self.create_subscription(Image, f'/{c}/camera/image_raw',
                                     lambda m, c=c: self.img.__setitem__(c, (time.time(), m)), 2)
            self.create_subscription(TFMessage, f'/{c}/tf_static', lambda m, c=c: self.cb_static(c, m), static)
        self.labels = {}
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.publish_labels)
        os.makedirs(OUT, exist_ok=True)
        self.get_logger().info(f'Do hang: {", ".join(cars)}. Lenh "<xe> <cot>" vd "1 3" = xe 1 do hang o (5,3)')

    # -------------------------------------------------------------- dau vao
    def cb_status(self, m):
        d = json.loads(m.data)
        self.status[d['name']] = d

    def cb_scan(self, c, m):
        self.scans[c].append((time.time(), (m.angle_min, m.angle_increment, list(m.ranges))))
        del self.scans[c][:-30]

    def cb_static(self, c, m):
        for t in m.transforms:
            q = t.transform.rotation
            self.static_tf[c][t.child_frame_id] = (
                t.header.frame_id, t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        x, y, th, f = 0.0, 0.0, 0.0, 'lidar_link'
        for _ in range(10):              # lidar_link -> ... -> base_footprint (2D)
            if f == 'base_footprint':
                self.lidar_x[c] = x
                return
            if f not in self.static_tf[c]:
                return
            p, tx, ty, yaw = self.static_tf[c][f]
            x, y = tx + x * math.cos(yaw) - y * math.sin(yaw), ty + x * math.sin(yaw) + y * math.cos(yaw)
            th += yaw
            f = p

    def cb_rosout(self, m):
        # traffic_manager tu choi dich (o da la dich xe kia, ngoai sa ban...)
        if m.name != 'traffic_manager' or m.level < 40:
            return
        for tk in self.tasks.values():
            if m.msg.startswith(f'[{tk.car}]') and tk.state in ('goto5', 'goto1'):
                tk.reject = m.msg

    def command(self, text):
        tok = text.split()
        if len(tok) != 2:
            self.get_logger().error(f'Lenh "{text}": can "<xe> <cot>", vd "1 3"')
            return
        car = self.cars[int(tok[0]) - 1] if tok[0].isdigit() and 1 <= int(tok[0]) <= len(self.cars) else tok[0]
        if car not in self.cars:
            self.get_logger().error(f'Khong co xe "{tok[0]}"')
            return
        col = int(tok[1])
        if not 1 <= col <= self.board.cols:
            self.get_logger().error(f'Cot {col} ngoai sa ban')
            return
        if car in self.tasks and self.tasks[car].state not in ('done', 'failed'):
            self.get_logger().error(f'[{car}] dang lam nhiem vu khac')
            return
        tk = Task(car, col)
        tk.reject = None
        self.tasks[car] = tk
        self.go(tk, (self.board.rows - 1, col - 1), 'b')

    # ------------------------------------------------------------ dieu khien
    def go(self, tk, node, heading=''):
        tk.target, tk.t, tk.reject = node, time.time(), None
        r, c = node
        self.goal_pub.publish(String(data=f'{self.cars.index(tk.car) + 1} {r + 1} {c + 1} {heading}'.strip()))
        self.get_logger().info(f'[{tk.car}] -> ({r + 1},{c + 1})' + (' quay Bac' if heading == 'b' else ''))

    def arrived(self, tk, need_north):
        s = self.status.get(tk.car)
        if not s or s.get('x') is None or time.time() - tk.t < 1.5:
            return False
        x, y = self.board.xy(*tk.target)
        if math.hypot(s['x'] - x, s['y'] - y) > ARRIVE_D or s['state'] not in ('done', 'idle'):
            return False
        if need_north:
            north = self.board.yaw + math.pi / 2
            return abs(math.atan2(math.sin(s['yaw'] - north), math.cos(s['yaw'] - north))) < ARRIVE_YAW
        return True

    def fail(self, tk, why):
        tk.state = 'failed'
        self.get_logger().error(f'[{tk.car}] DO HANG (5,{tk.col}) THAT BAI: {why}')
        self.res_pub.publish(String(data=json.dumps({'car': tk.car, 'col': tk.col, 'ok': False, 'error': why})))

    def tick(self):
        now = time.time()
        for tk in list(self.tasks.values()):
            if tk.state in ('done', 'failed'):
                continue
            s = self.status.get(tk.car, {})
            if tk.state in ('goto5', 'goto1'):
                if tk.reject:
                    self.fail(tk, f'traffic_manager tu choi: {tk.reject}')
                elif s.get('state') in ('failed', 'estop'):
                    self.fail(tk, f'xe bao {s.get("state")}')
                elif now - tk.t > TIMEOUT:
                    self.fail(tk, f'qua {TIMEOUT:.0f} s chua toi {tk.target}')
                elif self.arrived(tk, tk.state == 'goto5'):
                    if tk.state == 'goto5':
                        tk.state, tk.t = 'settle', now
                    else:
                        tk.state = 'done'
                        self.get_logger().info(f'[{tk.car}] XONG: hang (5,{tk.col}) -> da ve o (1,{tk.slot})')
            elif tk.state == 'settle' and now - tk.t > SETTLE:
                tk.state, tk.t = 'collect', now
            elif tk.state == 'collect':
                sc = [x for t, x in self.scans[tk.car] if t > tk.t]
                im = self.img.get(tk.car)
                if len(sc) >= self.cfg['lidar']['scans'] and im and im[0] > tk.t:
                    self.measure(tk, sc, im[1])
                elif now - tk.t > 10:
                    self.fail(tk, 'khong du anh quet / anh camera trong 10 s')
            elif tk.state == 'waitslot':
                self.try_slot(tk)

    def measure(self, tk, scans, m):
        frame = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        lx = self.lidar_x.get(tk.car, self.cfg['lidar']['x'])
        try:
            res, vis = cm.measure(frame, scans[-self.cfg['lidar']['scans']:], self.cfg, lx)
        except ValueError as e:
            tk.tries += 1
            if tk.tries < 3:
                self.get_logger().warn(f'[{tk.car}] do loi ({e}) - do lai')
                tk.state, tk.t = 'settle', time.time()
            else:
                self.fail(tk, f'khong do duoc: {e}')
            return
        cls = cm.classify(self.cfg, res['width'], res['height'])
        stamp = time.strftime('%Y%m%d-%H%M%S')
        path = f'{OUT}/{stamp}_{tk.car}_c{tk.col}.png'
        cv2.imwrite(path, vis)
        tk.result = dict(res, car=tk.car, col=tk.col, image=path, cls=cls['name'] if cls else None,
                         slot=cls['col'] if cls else None, ok=cls is not None)
        msg = (f'[{tk.car}] Hang o (5,{tk.col}): rong {res["width"] * 100:.1f} cm, cao {res["height"] * 100:.1f} cm '
               f'(Z {res["z_cam"] * 100:.1f} cm, mat lech {res["face_yaw_deg"]:+.1f} do) -> '
               + (f'loai "{cls["name"]}" -> o (1,{cls["col"]})' if cls else 'KHONG khop loai nao'))
        self.get_logger().info(msg)
        for w in res['warnings']:
            self.get_logger().warn(f'[{tk.car}] canh bao: {w}')
        new = not os.path.exists(f'{OUT}/results.csv')
        with open(f'{OUT}/results.csv', 'a') as f:
            if new:
                f.write('thoi_gian,xe,cot,rong_cm,cao_cm,z_cm,lech_do,loai,o_dich,canh_bao,anh\n')
            f.write(f'{stamp},{tk.car},{tk.col},{res["width"] * 100:.2f},{res["height"] * 100:.2f},'
                    f'{res["z_cam"] * 100:.2f},{res["face_yaw_deg"]:.1f},{tk.result["cls"]},'
                    f'{tk.result["slot"]},"{"; ".join(res["warnings"])}",{path}\n')
        out = {k: v for k, v in tk.result.items() if k != 'edges_px'}
        self.res_pub.publish(String(data=json.dumps(out)))
        if not cls:
            self.fail(tk, 'kich thuoc khong khop loai nao trong cargo.yaml')
            return
        tk.slot = cls['col']
        self.labels[tk.slot] = f'{cls["name"]}: {res["width"] * 100:.1f}x{res["height"] * 100:.1f} cm'
        tk.state, tk.t = 'waitslot', time.time()
        self.try_slot(tk)

    def try_slot(self, tk):
        """O (1,k) dang la dich cua xe khac / co xe khac do -> cho."""
        node = (0, tk.slot - 1)
        x, y = self.board.xy(*node)
        for o in self.tasks.values():
            if o is not tk and o.state == 'goto1' and o.target == node:
                return self.wait_note(tk, f'o (1,{tk.slot}) la dich cua {o.car}')
        for name, s in self.status.items():
            if name != tk.car and s.get('x') is not None and math.hypot(s['x'] - x, s['y'] - y) < 0.3:
                o = self.tasks.get(name)
                if (o is None or o.state in ('done', 'failed')) and s.get('state') in ('done', 'idle') \
                        and getattr(tk, 'asked', None) != name:
                    # xe da giao xong dang do o o dich: lui len hang 2 nhuong o
                    tk.asked = name
                    up = self.free_node_row2(tk.slot - 1, name)
                    if up is not None:
                        self.goal_pub.publish(String(
                            data=f'{self.cars.index(name) + 1} {up[0] + 1} {up[1] + 1}'))
                        self.get_logger().info(f'[{name}] nhuong o (1,{tk.slot}) cho {tk.car} '
                                               f'-> lui len ({up[0] + 1},{up[1] + 1})')
                return self.wait_note(tk, f'{name} dang o (1,{tk.slot})')
        tk.state = 'goto1'
        self.go(tk, node)

    def free_node_row2(self, c, who):
        """O hang 2 gan cot c nhat khong co xe nao dung / sap toi."""
        busy = set()
        for name, s in self.status.items():
            if name != who and s.get('x') is not None:
                busy.add(self.board.closest(s['x'], s['y']))
        busy |= {o.target for o in self.tasks.values() if o.state in ('goto5', 'goto1')}
        for cc in sorted(range(self.board.cols), key=lambda k: abs(k - c)):
            if (1, cc) not in busy:
                return (1, cc)
        return None

    def wait_note(self, tk, why):
        if getattr(tk, 'noted', None) != why:
            tk.noted = why
            self.get_logger().info(f'[{tk.car}] cho: {why}')
        if time.time() - tk.t > TIMEOUT:
            self.fail(tk, f'cho qua lau: {why}')

    def publish_labels(self):
        ma = MarkerArray()
        for k, text in self.labels.items():
            x, y = self.board.xy(0, k - 1)
            t = Marker()
            t.header.frame_id, t.ns, t.id, t.type = 'map', 'cargo_result', k, Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y - 0.18, 0.15
            t.scale.z = 0.06
            t.color = ColorRGBA(r=1.0, g=0.85, b=0.2, a=1.0)
            t.text = text
            ma.markers.append(t)
        if ma.markers:
            self.mk_pub.publish(ma)


def stdin_loop(node):
    if sys.stdin.isatty():
        print('Nhap "<xe> <cot>" (vd "1 3" = xe 1 do hang o (5,3)), "q" thoat.')
    for line in sys.stdin:
        line = line.strip()
        if line in ('q', 'quit', 'exit'):
            break
        if line:
            node.command(line)
    else:
        return                 # het dau vao (chay nen): van nhan lenh qua /fleet/cargo
    rclpy.try_shutdown()


def main():
    rclpy.init(args=sys.argv)
    cars = remove_ros_args(sys.argv)[1:] or ['agv1', 'agv2']
    node = CargoTask(cars)
    threading.Thread(target=stdin_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

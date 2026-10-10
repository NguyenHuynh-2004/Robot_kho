#!/usr/bin/env python3
"""Theo doi doi xe: gom LOI va TRANG THAI vao file de doc thang (nguoi hoac tro ly).

    python3 tools/fleet_monitor.py --out ~/agv_logs/latest [agv1 agv2]

MOT node ROS (mot tien trinh - khong lam day gioi han do tim DDS LOCALHOST):
  - /rosout: moi node ROS 2 (Nav2, AMCL, fleet_agent, traffic_manager ...).
      WARN tro len -> errors.log (gop dong lap lai); INFO cua fleet_agent /
      traffic_manager -> events.log (dong thoi gian chay xe).
  - moi xe:
      nhip /agvN/scan, joint_states  -> ESP32 con song khong, Wi-Fi tot khong
      /agvN/diag      (firmware moi: ESTOP + ly do, RSSI, ly do reset, RPM, PWM)
      /agvN/amcl_pose -> do tin cay dinh vi (do lech chuan x/y/goc)
      /agvN/cmd_vel + joint_states -> co lenh chay ma banh dung yen = nghi ESTOP
  - /fleet/status: vi tri (o, huong, lech khoi tam o), trang thai agent.
  - do thi ROS: node nao THIEU so voi mot he dang chay du.
Ghi status.json moi giay (ghi file tam roi doi ten: doc luc nao cung nguyen ven)
va su kien doi trang thai (xe mat/co lai, ESTOP, dinh vi kem) vao events.log.
Khong phat gi len ROS, khong dieu khien xe.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, WS)
from grid_board import Board, config_path          # noqa: E402
from grid_navigator import heading_name            # noqa: E402

LEVEL = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
EXPECT_HZ = {'scan': 6.0, 'js': 40.0}   # duoi muc nay = Wi-Fi yeu / ESP32 cham
LOST_S = 2.0            # khong co /scan lau hon -> xe mat ket noi
CMD_MOVE_V = 0.03       # m/s - lenh coi la "dang bao chay"
CMD_MOVE_W = 0.30       # rad/s
WHEEL_STILL = 0.05      # rad trong ESTOP_S giay - banh coi nhu dung yen
ESTOP_S = 2.0
AMCL_BAD_XY = 0.10      # m - do lech chuan vi tri AMCL lon hon -> dinh vi kem (luc chay ~5 cm la thuong)
AMCL_BAD_YAW = 8.0      # do
REPEAT_S = 30.0         # cung mot dong loi trong 30 s chi ghi mot lan, kem so lan lap
EVENT_NODES = ('fleet_agent', 'traffic_manager')
# Node phai co khi he 2 xe chay du (tools/fleet_real.sh + traffic_manager)
PER_ROBOT_NODES = ['esp32_agv_base_{i}', 'amcl', 'wheel_odometry', 'robot_state_publisher',
                   'controller_server', 'planner_server', 'collision_monitor', 'fleet_agent']
SHARED_NODES = ['/map_server', '/traffic_manager']


def now_str():
    t = time.time()
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t)) + f'.{int(t % 1 * 10)}'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Car:
    def __init__(self, name):
        self.name = name
        self.idx = int(''.join(ch for ch in name if ch.isdigit()) or 0)
        self.count = {'scan': 0, 'js': 0}
        self.hz = {'scan': 0.0, 'js': 0.0}
        self.last = {'scan': 0.0, 'js': 0.0, 'diag': 0.0, 'amcl': 0.0,
                     'fleet': 0.0, 'cmd': 0.0}
        self.scan_valid = None          # ti le tia hop le cua scan cuoi
        self.wheels = None              # (t, [goc trai, goc phai])
        self.wheel_hist = []            # [(t, pos)] trong ESTOP_S giay gan nhat
        self.cmd = (0.0, 0.0)
        self.cmd_move_since = None      # bat dau co lenh chay lien tuc tu luc nao
        self.diag = None
        self.amcl = None
        self.fleet = None
        self.connected = False
        self.connected_since = 0.0
        self.low_ticks = 0              # so giay lien tiep nhip thap
        self.flags = set()              # trang thai bat thuong dang co (de bao khi doi)


class FleetMonitor(Node):
    def __init__(self, names, out, grid_file=None, emulate=False):
        super().__init__('fleet_monitor')
        self.expect = [t for t in PER_ROBOT_NODES if not (emulate and t.startswith('esp32'))]
        self.out = out
        os.makedirs(out, exist_ok=True)
        self.err_f = open(os.path.join(out, 'errors.log'), 'a', buffering=1)
        self.evt_f = open(os.path.join(out, 'events.log'), 'a', buffering=1)
        self.board = Board.load(grid_file or config_path(True))
        self.cars = {n: Car(n) for n in names}
        self.t_start = time.time()
        self.t_tick = time.time()
        self.repeat = {}                # (node, msg) -> [lan dau ghi, so lan bi bo]
        self.recent_errors = []         # 20 loi moi nhat cho status.json
        self.err_count = {'WARN': 0, 'ERROR': 0, 'FATAL': 0}
        self.missing_nodes = []
        self.nodes_seen = []

        sensor = qos_profile_sensor_data
        # raw=True: khong giai ma, chi giai ma vai tin moi giay khi can noi dung.
        # KHONG nghe imu/data (50 Hz): moi callback rclpy ton ~1 ms, nghe ca imu lan
        # joint_states cua 2 xe = ~20% mot nhan. Nhip IMU co trong /agvN/diag (firmware).
        for n, car in self.cars.items():
            self.create_subscription(LaserScan, f'/{n}/scan',
                                     lambda m, c=car: self.cb_scan(c, m), sensor, raw=True)
            self.create_subscription(JointState, f'/{n}/joint_states',
                                     lambda m, c=car: self.cb_js(c, m), sensor, raw=True)
            self.create_subscription(String, f'/{n}/diag',
                                     lambda m, c=car: self.cb_diag(c, m), sensor)
            self.create_subscription(Twist, f'/{n}/cmd_vel',
                                     lambda m, c=car: self.cb_cmd(c, m), sensor)
            amcl_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                  durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(PoseWithCovarianceStamped, f'/{n}/amcl_pose',
                                     lambda m, c=car: self.cb_amcl(c, m), amcl_qos)
        self.create_subscription(String, '/fleet/status', self.cb_fleet, 50)
        rosout_qos = QoSProfile(depth=1000, history=QoSHistoryPolicy.KEEP_LAST,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Log, '/rosout', self.cb_rosout, rosout_qos)
        self.create_timer(1.0, self.tick)
        self.create_timer(5.0, self.check_graph)
        self.event(None, f'bat dau theo doi {", ".join(names)} (domain '
                         f'{os.environ.get("ROS_DOMAIN_ID", "0")}), ghi vao {out}')

    # ------------------------------------------------------------------ ghi file
    def event(self, car, text):
        line = f'{now_str()} {"[" + car.name + "] " if car else ""}{text}'
        self.evt_f.write(line + '\n')
        print(line, flush=True)

    def error(self, level, node, text):
        key = (node, text[:120])
        now = time.time()
        rec = self.repeat.get(key)
        if rec and now - rec[0] < REPEAT_S:
            rec[1] += 1
            return
        if rec and rec[1]:
            self.err_f.write(f'{now_str()} {level:5} [{node}] (lap lai {rec[1]} lan trong '
                             f'{REPEAT_S:.0f} s truoc)\n')
        self.repeat[key] = [now, 0]
        self.err_f.write(f'{now_str()} {level:5} [{node}] {text}\n')
        self.recent_errors.append({'t': now_str(), 'level': level, 'node': node,
                                   'msg': text[:300]})
        del self.recent_errors[:-20]

    # ------------------------------------------------------------------ callback
    def cb_count(self, car, key):
        car.count[key] += 1
        car.last[key] = time.time()

    def cb_scan(self, car, raw):
        self.cb_count(car, 'scan')
        if car.count['scan'] != 1:              # giai ma 1 scan / giay
            return
        msg = deserialize_message(raw, LaserScan)
        r = msg.ranges
        if r:
            ok = sum(1 for v in r if msg.range_min <= v <= msg.range_max)
            car.scan_valid = ok / len(r)

    def cb_js(self, car, raw):
        self.cb_count(car, 'js')
        if car.count['js'] % 5 != 1:            # 10 Hz du de biet banh co quay
            return
        msg = deserialize_message(raw, JointState)
        if len(msg.position) >= 2:
            t = time.time()
            car.wheels = (t, list(msg.position[:2]))
            car.wheel_hist.append((t, msg.position[0], msg.position[1]))
            while car.wheel_hist and t - car.wheel_hist[0][0] > ESTOP_S:
                car.wheel_hist.pop(0)

    def cb_diag(self, car, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            d = {'raw': msg.data}
        car.diag = d
        car.last['diag'] = time.time()

    def cb_cmd(self, car, msg):
        t = time.time()
        car.cmd = (msg.linear.x, msg.angular.z)
        car.last['cmd'] = t
        moving = abs(msg.linear.x) > CMD_MOVE_V or abs(msg.angular.z) > CMD_MOVE_W
        if moving and car.cmd_move_since is None:
            car.cmd_move_since = t
        elif not moving:
            car.cmd_move_since = None

    def cb_amcl(self, car, msg):
        p, c = msg.pose.pose, msg.pose.covariance
        car.amcl = {'x': p.position.x, 'y': p.position.y, 'yaw': yaw_of(p.orientation),
                    'sd_x': math.sqrt(max(c[0], 0)), 'sd_y': math.sqrt(max(c[7], 0)),
                    'sd_yaw_deg': math.degrees(math.sqrt(max(c[35], 0)))}
        car.last['amcl'] = time.time()

    def cb_fleet(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        car = self.cars.get(d.get('name'))
        if car is None:
            return
        if car.fleet is None or d.get('state') != car.fleet.get('state'):
            self.event(car, f'trang thai agent: {d.get("state")}')
        car.fleet = d
        car.last['fleet'] = time.time()

    def cb_rosout(self, msg):
        node = msg.name
        if node == 'fleet_monitor':
            return
        level = LEVEL.get(msg.level, str(msg.level))
        if msg.level >= 30:
            if level in self.err_count:
                self.err_count[level] += 1
            self.error(level, node, msg.msg)
        elif msg.level == 20 and any(k in node for k in EVENT_NODES):
            self.evt_f.write(f'{now_str()} [{node}] {msg.msg}\n')

    def check_graph(self):
        seen = {(ns.rstrip('/') + '/' + n) for n, ns in self.get_node_names_and_namespaces()}
        self.nodes_seen = sorted(seen)
        missing = [n for n in SHARED_NODES if n not in seen]
        for car in self.cars.values():
            for tmpl in self.expect:
                full = f'/{car.name}/' + tmpl.format(i=car.idx)
                if full not in seen:
                    missing.append(full)
        if missing != self.missing_nodes:
            gone = sorted(set(missing) - set(self.missing_nodes))
            back = sorted(set(self.missing_nodes) - set(missing))
            if gone:
                self.event(None, f'THIEU node: {", ".join(gone)}')
            if back:
                self.event(None, f'node co lai: {", ".join(back)}')
            self.missing_nodes = missing

    # ------------------------------------------------------------------ moi giay
    def flag(self, car, key, on, text_on, text_off=None):
        if on and key not in car.flags:
            car.flags.add(key)
            self.event(car, text_on)
            self.error('WARN', f'monitor/{car.name}', text_on)
        elif not on and key in car.flags:
            car.flags.discard(key)
            if text_off:
                self.event(car, text_off)

    def car_status(self, car, now, dt):
        for k in car.hz:
            car.hz[k] = round(car.count[k] / dt, 1) if dt > 0 else 0.0
            car.count[k] = 0
        age = {k: (round(now - v, 1) if v else None) for k, v in car.last.items()}
        connected = car.last['scan'] > 0 and now - car.last['scan'] < LOST_S
        if connected != car.connected:
            car.connected = connected
            car.connected_since = now
            self.event(car, 'ESP32 KET NOI (co /scan)' if connected else
                       f'ESP32 MAT KET NOI - khong co /{car.name}/scan > {LOST_S:.0f} s')
        problems = []
        if connected and now - car.connected_since > 3:     # giay dau dem thieu
            for k, lo in EXPECT_HZ.items():
                if car.hz[k] < lo:
                    problems.append(f'{k} {car.hz[k]} Hz (< {lo})')
        car.low_ticks = car.low_ticks + 1 if problems else 0
        self.flag(car, 'rate', car.low_ticks >= 2,
                  'nhip du lieu thap: ' + ', '.join(problems) + ' - Wi-Fi yeu / ESP32 cham',
                  'nhip du lieu binh thuong')

        # Lenh chay lien tuc ESTOP_S giay ma banh khong quay -> ESTOP / ket banh / het pin
        still = None
        if len(car.wheel_hist) >= 2:
            _, l0, r0 = car.wheel_hist[0]
            _, l1, r1 = car.wheel_hist[-1]
            still = abs(l1 - l0) < WHEEL_STILL and abs(r1 - r0) < WHEEL_STILL
        cmd_fresh = car.last['cmd'] and now - car.last['cmd'] < 0.5
        suspect = bool(connected and still and cmd_fresh and car.cmd_move_since
                       and now - car.cmd_move_since > ESTOP_S)
        diag = car.diag if car.last['diag'] and now - car.last['diag'] < 3 else None
        estop = diag.get('estop') if diag else None
        self.flag(car, 'estop', bool(estop), f'!!! ESTOP tren ESP32: {estop}', 'het ESTOP')
        self.flag(car, 'stuck', suspect and not estop,
                  f'co lenh chay {ESTOP_S:.0f} s ma banh dung yen -> nghi ESTOP / ket banh / '
                  'het pin (firmware cu khong bao ESTOP: xem Serial)', 'banh quay lai')

        pose = None
        f = car.fleet if car.last['fleet'] and now - car.last['fleet'] < 2 else None
        src = (f, 'tf') if f and f.get('x') is not None else (car.amcl, 'amcl_pose')
        if src[0] is not None:
            x, y, yaw = src[0]['x'], src[0]['y'], src[0]['yaw']
            r, c = self.board.closest(x, y)
            nx, ny = self.board.xy(r, c)
            hname, hoff = heading_name(yaw, self.board.yaw)
            pose = {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(math.degrees(yaw), 1),
                    'row': r + 1, 'col': c + 1, 'off_node_cm': round(math.hypot(x - nx, y - ny) * 100, 1),
                    'heading': hname, 'heading_off_deg': round(hoff, 1), 'source': src[1]}
        amcl_bad = False
        if car.amcl:
            amcl_bad = car.amcl['sd_x'] > AMCL_BAD_XY or car.amcl['sd_y'] > AMCL_BAD_XY or \
                car.amcl['sd_yaw_deg'] > AMCL_BAD_YAW
        self.flag(car, 'amcl', amcl_bad,
                  f'dinh vi KEM: AMCL lech chuan x {car.amcl["sd_x"] * 100:.1f} cm, '
                  f'y {car.amcl["sd_y"] * 100:.1f} cm, goc {car.amcl["sd_yaw_deg"]:.1f} do'
                  if car.amcl else '', 'dinh vi on lai')
        return {
            'connected': connected,
            'rate_hz': dict(car.hz),
            'scan_valid_ratio': round(car.scan_valid, 2) if car.scan_valid is not None else None,
            'age_s': age,
            'diag': diag,
            'estop': estop,
            'suspect_stuck': suspect,
            'cmd_vel': {'v': round(car.cmd[0], 3), 'w': round(car.cmd[1], 3)} if cmd_fresh else None,
            'pose': pose,
            'amcl_sd': {k: round(v, 3) for k, v in car.amcl.items() if k.startswith('sd')}
            if car.amcl else None,
            'agent': {k: f.get(k) for k in ('state', 'route_id', 'blocked_nodes')} if f else None,
            'flags': sorted(car.flags),
        }

    def tick(self):
        now = time.time()
        dt, self.t_tick = now - self.t_tick, now
        status = {
            'time': now_str(), 'unix': round(now, 1),
            'uptime_s': round(now - self.t_start),
            'domain': int(os.environ.get('ROS_DOMAIN_ID', '0')),
            'cars': {n: self.car_status(c, now, dt) for n, c in self.cars.items()},
            'missing_nodes': self.missing_nodes,
            'node_count': len(self.nodes_seen),
            'error_count': self.err_count,
            'recent_errors': self.recent_errors[-10:],
        }
        tmp = os.path.join(self.out, '.status.json.tmp')
        with open(tmp, 'w') as fh:
            json.dump(status, fh, ensure_ascii=False, indent=1)
        os.replace(tmp, os.path.join(self.out, 'status.json'))


def main():
    rclpy.init(args=sys.argv)
    from rclpy.utilities import remove_ros_args
    ap = argparse.ArgumentParser()
    ap.add_argument('names', nargs='*', default=['agv1', 'agv2'])
    ap.add_argument('--out', default=os.path.expanduser('~/agv_logs/latest'))
    ap.add_argument('--grid', default=None)
    ap.add_argument('--emulate', action='store_true', help='ESP32 gia: khong co node esp32_agv_base_N')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])
    node = FleetMonitor(args.names or ['agv1', 'agv2'], os.path.expanduser(args.out), args.grid,
                        args.emulate)
    ex = SingleThreadedExecutor()      # MultiThreaded + topic 50 Hz = an CPU (xem techContext)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.event(None, 'dung theo doi')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

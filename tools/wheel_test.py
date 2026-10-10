#!/usr/bin/env python3
"""Thu chieu dong co + encoder mot xe: tien, lui, quay trai, quay phai (moi lenh 2 s).

    python3 tools/wheel_test.py agv1            # KE XE HO BANH truoc khi chay!

Gui thang /<xe>/cmd_vel (cham, 0.1 m/s / 1 rad/s) va doc /<xe>/joint_states. In
bang "can / do" cho tung banh. Dung khi vua nap firmware moi, doi mach, cam lai
day dong co hay encoder. Dung KHI KHONG chay Nav2/dieu phoi cho xe do.

Doc ket qua:
  - ca 4 dong "DUNG"                         -> day dong co + encoder khop firmware
  - TIEN ca hai banh bao LUI, xe ESTOP       -> day dong co trai/phai noi CHEO so voi
                                                firmware (doi nhom chan dong co cua xe)
  - mot banh "khong quay"                    -> dong co/encoder banh do (day, nguon 12 V)
  - firmware co /diag thi ESTOP hien o dong cuoi.
"""
import json
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

R, HALF = 0.03281, 0.100


def main():
    if len(sys.argv) < 2 or not sys.argv[1].startswith('agv'):
        sys.exit('Dung: python3 tools/wheel_test.py agv1   (ke xe ho banh)')
    car = sys.argv[1]
    rclpy.init()
    n = Node('wheel_test')
    pub = n.create_publisher(Twist, f'/{car}/cmd_vel', 10)
    js, diag = [], []
    n.create_subscription(JointState, f'/{car}/joint_states',
                          lambda m: js.append((time.time(), m.position[0], m.position[1])),
                          qos_profile_sensor_data)
    n.create_subscription(String, f'/{car}/diag', lambda m: diag.append(m.data),
                          qos_profile_sensor_data)

    def phase(v, w, d):
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < d:
            if time.time() >= nxt:
                m = Twist()
                m.linear.x, m.angular.z = float(v), float(w)
                pub.publish(m)
                nxt += 0.05
            rclpy.spin_once(n, timeout_sec=0.01)

    phase(0, 0, 1.0)
    if not js:
        sys.exit(f'Khong nhan duoc /{car}/joint_states - xe da bat nguon / ket noi chua?')
    print(f'{car}: {len(js)} goi joint_states/1 s. Bat dau (ke xe ho banh)...')
    rows = []
    for name, v, w in (('TIEN', 0.10, 0.0), ('LUI', -0.10, 0.0),
                       ('QUAY TRAI', 0.0, 1.0), ('QUAY PHAI', 0.0, -1.0)):
        p0 = js[-1]
        phase(v, w, 2.0)
        phase(0, 0, 2.0)
        p1 = js[-1]
        rows.append((name, (v - w * HALF) / R, (v + w * HALF) / R, p1[1] - p0[1], p1[2] - p0[2]))

    def verdict(need, got):
        if abs(got) < 0.3:
            return 'khong quay'
        return 'DUNG' if (got > 0) == (need > 0) else 'NGUOC'

    print(f'\n{"lenh":10s} | {"banh trai can/do (rad)":28s} | banh phai can/do (rad)')
    for name, el, er, dl, dr in rows:
        print(f'{name:10s} | {el:+5.1f} / {dl:+6.2f}  {verdict(el, dl):10s} | '
              f'{er:+5.1f} / {dr:+6.2f}  {verdict(er, dr)}')
    if diag:
        try:
            e = json.loads(diag[-1]).get('estop')
            print(f'\nESP32: {"ESTOP: " + e if e else "khong ESTOP"}')
        except ValueError:
            pass
    phase(0, 0, 0.5)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

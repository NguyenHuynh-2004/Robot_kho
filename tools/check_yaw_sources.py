#!/usr/bin/env python3
"""So ba nguon goc quay (yaw) trong luc xe chay quanh phong:
  quat  quaternion cua BNO055. Firmware goi bno.begin() khong tham so, tuc
        che do NDOF - CO dung tu ke de neo huong.
  gyro  tich phan van toc goc z (cung du lieu /imu/data, khong dung tu ke)
  banh  (goc banh phai - goc banh trai) * r / khoang cach hai banh

    cd ~/robot_ws_3 && python3 tools/check_yaw_sources.py --duration 120

Lai xe di khap phong, co NHIEU DOAN CHAY THANG dai 1-2 m. Thuan doc, khong
publish gi - chay song song voi bat cu che do nao (SLAM, AMCL, Nav2).

DOC KET QUA
  Cot "thang q-g": cong don (quaternion xoay) - (gyro xoay) CHI trong luc xe
  chay thang. Chay thang thi huong that khong doi, gyro doc ~0, nen quaternion
  cung KHONG duoc xoay. Neu no van xoay vai do roi xoay nguoc lai tuy CHO xe
  dang dung, thi tu ke dang bi sat thep / dong co / day dien keo lech: huong
  quaternion sai theo VI TRI trong phong. Loi kieu do khong phai sai ti le gyro
  (chay thang thi khong co gi de nhan ti le) va khong phai banh truot.

  bien do "thang q-g" < 1.5 do  tu ke khong phai thu pham
                      > 3 do    tu ke keo lech -> dung yaw_source:=imu_gyro
"""
import argparse
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState

STILL_WHEEL = 0.05     # rad/s banh - duoi muc nay coi nhu dung yen (hoc bias)
MOVE_WHEEL = 0.5       # rad/s banh trung binh - tren muc nay coi la dang chay
STRAIGHT_GZ = 0.05     # rad/s (~3 do/s) - quay cham hon the nay coi la thang


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Y(Node):
    def __init__(self, radius, sep):
        super().__init__('check_yaw_sources')
        self.radius, self.sep = radius, sep
        be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(Imu, '/imu/data', self.cb_imu, be)
        self.create_subscription(JointState, '/joint_states', self.cb_js, be)
        self.create_subscription(Odometry, '/odom', self.cb_odom, be)

        self.x = self.y = 0.0
        self.vl = self.vr = 0.0
        self.have_js = False
        self.enc0 = None
        self.enc = 0.0

        self.bias = 0.0
        self.bias_ready = False  # da hoc bias it nhat mot lan
        self.reset_req = False   # main yeu cau xoa cong don (lam trong callback)
        self.still_win = []
        self.prev_t = None
        self.prev_gz = None
        self.prev_q = None
        self.quat = 0.0          # quaternion xoay cong don ke tu luc bat dau
        self.gyro = 0.0          # gyro xoay cong don
        self.gaps = 0            # so lo hong > 0.5 s bi bo qua
        self.n_imu = 0

        self.s_qg = 0.0          # cong don (quat - gyro) chi khi chay thang
        self.s_min = self.s_max = 0.0
        self.s_dist = 0.0        # quang duong chay thang (m)
        self.s_where = [(0.0, 0.0, 0.0)]   # (s_qg, x, y) de biet lech o dau

    def cb_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y

    def cb_js(self, m):
        try:
            il = m.name.index('left_wheel_joint')
            ir = m.name.index('right_wheel_joint')
        except ValueError:
            return
        if len(m.position) > max(il, ir):
            e = (m.position[ir] - m.position[il]) * self.radius / self.sep
            if self.enc0 is None:
                self.enc0 = e
            self.enc = e - self.enc0
        if len(m.velocity) > max(il, ir):
            self.vl, self.vr = m.velocity[il], m.velocity[ir]
            self.have_js = True

    def cb_imu(self, m):
        q = m.orientation
        if abs(q.w) + abs(q.x) + abs(q.y) + abs(q.z) < 1e-6:
            return                       # IMU chua san sang
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        gz = m.angular_velocity.z
        y = yaw_of(q)
        self.n_imu += 1

        still = (self.have_js and abs(self.vl) < STILL_WHEEL
                 and abs(self.vr) < STILL_WHEEL)
        if still:
            self.still_win.append(gz)
            if len(self.still_win) >= 50:
                tgt = sum(self.still_win[-100:]) / len(self.still_win[-100:])
                if self.bias_ready:
                    self.bias += 0.2 * (tgt - self.bias)
                else:
                    self.bias = tgt          # lan dau: lay luon
                    self.bias_ready = True
        else:
            self.still_win = []

        if self.reset_req:
            self.reset_req = False
            self.quat = self.gyro = 0.0
            self.s_qg = self.s_min = self.s_max = self.s_dist = 0.0
            self.s_where = [(0.0, self.x, self.y)]
            self.enc0 = None
            self.gaps = 0

        if self.prev_t is None:
            self.prev_t, self.prev_gz, self.prev_q = t, gz, y
            return
        dt = t - self.prev_t
        if dt <= 0.0:
            return
        dq = wrap(y - self.prev_q)
        if dt < 0.5:
            # Hinh thang: mat vai goi giua chung thi van tinh du goc
            dg = (0.5 * (gz + self.prev_gz) - self.bias) * dt
        else:
            dg = dq                      # lo hong lon: khong biet, bo qua
            self.gaps += 1
        self.quat += dq
        self.gyro += dg

        moving = 0.5 * (abs(self.vl) + abs(self.vr)) > MOVE_WHEEL
        # Ca hai dau khoang deu phai thang - mau dau tien sau khi thoi quay
        # con mang nua goc quay cua mau truoc.
        if (moving and abs(gz - self.bias) < STRAIGHT_GZ
                and abs(self.prev_gz - self.bias) < STRAIGHT_GZ and dt < 0.5):
            self.s_qg += dq - dg
            self.s_min = min(self.s_min, self.s_qg)
            self.s_max = max(self.s_max, self.s_qg)
            self.s_dist += 0.5 * (abs(self.vl) + abs(self.vr)) * self.radius * dt
            if abs(self.s_qg - self.s_where[-1][0]) > math.radians(0.5):
                self.s_where.append((self.s_qg, self.x, self.y))

        self.prev_t, self.prev_gz, self.prev_q = t, gz, y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--every', type=float, default=2.0)
    ap.add_argument('--wheel-radius', type=float, default=0.03281)
    ap.add_argument('--wheel-separation', type=float, default=0.200)
    a = ap.parse_args()

    rclpy.init()
    n = Y(a.wheel_radius, a.wheel_separation)
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    end = time.time() + 20
    while time.time() < end and (n.n_imu < 10 or not n.have_js):
        time.sleep(0.2)
    if n.n_imu < 10:
        print('Khong nhan duoc /imu/data.'); rclpy.shutdown(); return
    if not n.have_js:
        print('Khong nhan duoc /joint_states.'); rclpy.shutdown(); return

    # Gyro BNO055 lech ~0.01 rad/s luc dung yen (do tren xe that). Chua tru
    # bias ma da cong don thi vai giay dau da sai vai do.
    if not n.bias_ready:
        print('De xe DUNG YEN 2 giay de hoc bias gyro...')
    end = time.time() + 30
    while time.time() < end and not n.bias_ready:
        time.sleep(0.2)
    if not n.bias_ready:
        print('Xe khong dung yen lan nao trong 30 s - khong hoc duoc bias gyro.')
        rclpy.shutdown(); return
    n.reset_req = True
    time.sleep(0.2)

    D = math.degrees
    print(f'Bias gyro {n.bias:+.5f} rad/s. Tat ca tinh bang DO, lay luc nay '
          'lam 0. Lai xe di, nhieu doan chay thang.\n')
    print(f"{'t(s)':>5} {'x':>6} {'y':>6} {'quat':>8} {'gyro':>8} {'banh':>8}"
          f" {'q-g':>6} {'thang q-g':>10} {'m thang':>8}")
    qg_min = qg_max = 0.0
    t0 = time.time()
    while time.time() - t0 < a.duration:
        time.sleep(a.every)
        qg = D(n.quat - n.gyro)
        qg_min, qg_max = min(qg_min, qg), max(qg_max, qg)
        print(f"{time.time()-t0:5.0f} {n.x:+6.2f} {n.y:+6.2f} {D(n.quat):+8.1f}"
              f" {D(n.gyro):+8.1f} {D(n.enc):+8.1f} {qg:+6.1f}"
              f" {D(n.s_qg):+10.1f} {n.s_dist:8.2f}")

    amp = D(n.s_max - n.s_min)
    print('\n=== tong ket ===')
    print(f"  bias gyro hoc duoc       : {n.bias:+.5f} rad/s")
    print(f"  lo hong IMU > 0.5 s      : {n.gaps}")
    print(f"  tong xoay  quat {D(n.quat):+.1f}  gyro {D(n.gyro):+.1f}  "
          f"banh {D(n.enc):+.1f} do")
    print(f"  q-g ca buoi: tu {qg_min:+.1f} den {qg_max:+.1f} do")
    print(f"  CHAY THANG {n.s_dist:.2f} m: quaternion tu xoay so voi gyro "
          f"tu {D(n.s_min):+.1f} den {D(n.s_max):+.1f} do -> bien do {amp:.1f} do")
    if n.s_dist < 2.0:
        print('\n  Chay thang chua du 2 m - chua ket luan duoc. Lai them doan thang.')
    elif amp > 3.0:
        print('\n  TU KE KEO LECH: chay thang ma quaternion van xoay. Huong cua no')
        print('  sai theo vi tri trong phong. Dung yaw_source:=imu_gyro.')
        print('  Cac cho no lech (x, y theo odom):')
        for s, x, y in n.s_where[1::max(1, len(n.s_where) // 12)]:
            print(f"    ({x:+.2f}, {y:+.2f})  thang q-g {D(s):+.1f} do")
    elif amp < 1.5:
        print('\n  Tu ke KHONG phai thu pham: chay thang quaternion dung yen.')
    else:
        print('\n  Lung chung (1.5-3 do). Lai them doan thang dai roi do lai.')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

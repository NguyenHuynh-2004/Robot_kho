#!/usr/bin/env python3
"""Tinh odometry tren LAPTOP tu du lieu tho cua ESP32.

ESP32 chi phat:
    /joint_states   goc banh TICH LUY (rad) + toc do goc (rad/s)
    /imu/data       quaternion tuyet doi + gyro + gia toc
Node nay ghep lai thanh /odom va TF odom -> base_footprint.

VI SAO TACH RA NHU VAY
  - Sua mot y tuong ve odometry khong con phai nap lai chip.
  - wheel_radius roi khoi firmware: ESP32 gui GOC banh, node nay nhan voi ban
    kinh. Hieu chinh ban kinh lan = sua tham so ROS, khong phai nap chip.
  - Tinh bang float64, va dung duoc quaternion tuyet doi cua BNO055 thay vi
    tich phan toc do goc.

VI SAO ESP32 GUI GIA TRI TICH LUY, KHONG GUI SO GIA
  Da do tren duong truyen nay: /odom co duoi tre toi 32 ms, /scan mat 7 khoang
  > 200 ms trong 12 giay. Voi so gia thi moi goi roi la mat han quang duong do,
  vinh vien. Voi gia tri tich luy thi goi roi khong ton gi - goi sau mang ca
  gia tri. Node nay vi the chi lay HIEU giua hai lan nhan, khong cong don.

THAM SO
  wheel_radius      m, ban kinh LAN (nho hon ban kinh hinh hoc vi lop bi nen)
  wheel_separation  m, chi dung khi yaw_source = wheels
  yaw_source        imu_quaternion | imu_gyro | wheels
      imu_quaternion  goc tuyet doi tu BNO055, KHONG TROI. Nen dung.
      imu_gyro        tich phan gyro z - giong y firmware cu, de so sanh
      wheels          suy tu chenh lech hai banh, du phong khi IMU hong
"""
import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

# Mot buoc nhay goc banh lon hon the nay giua hai message khong phai chuyen
# dong that - la ESP32 vua khoi dong lai va total_counts ve 0. 5 rad o banh
# 32 mm la 16 cm trong mot chu ky 20 ms, tuc 8 m/s.
JUMP_RAD = 5.0


def yaw_of(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('wheel_radius', 0.03225)
        self.declare_parameter('wheel_separation', 0.200)
        self.declare_parameter('yaw_source', 'imu_quaternion')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')
        self.declare_parameter('publish_tf', True)

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.R = float(g('wheel_radius'))
        self.sep = float(g('wheel_separation'))
        self.yaw_source = str(g('yaw_source'))
        self.odom_frame = str(g('odom_frame'))
        self.base_frame = str(g('base_frame'))
        self.left_joint = str(g('left_joint'))
        self.right_joint = str(g('right_joint'))
        self.publish_tf = bool(g('publish_tf'))

        if self.yaw_source not in ('imu_quaternion', 'imu_gyro', 'wheels'):
            self.get_logger().error(
                f"yaw_source '{self.yaw_source}' khong hop le, dung 'wheels'")
            self.yaw_source = 'wheels'

        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self._prev = None          # (t, pos_l, pos_r)
        self._imu_yaw = None       # yaw tuyet doi moi nhat tu quaternion
        self._imu_yaw0 = None      # yaw luc bat dau -> de odom xuat phat tu 0
        self._gyro_z = 0.0
        self._lock = threading.Lock()
        self._warned_missing = False

        # ESP32 phat /joint_states RELIABLE va /imu/data BEST_EFFORT.
        # QoS phai khop, lech la khong nhan duoc message nao ma cung khong bao
        # loi o cho de thay.
        self.create_subscription(
            JointState, 'joint_states', self.cb_js,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=50))
        self.create_subscription(
            Imu, 'imu/data', self.cb_imu,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=50))

        self.pub = self.create_publisher(Odometry, 'odom', 20)
        self.tfb = TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(
            f"wheel_radius={self.R:.5f} m, wheel_separation={self.sep:.3f} m, "
            f"yaw_source={self.yaw_source}")

    # ------------------------------------------------------------------
    def cb_imu(self, m: Imu):
        with self._lock:
            self._gyro_z = m.angular_velocity.z
            y = yaw_of(m.orientation)
            # Quaternion toan 0 nghia la IMU chua san sang
            q = m.orientation
            if abs(q.w) + abs(q.x) + abs(q.y) + abs(q.z) > 1e-6:
                self._imu_yaw = y
                if self._imu_yaw0 is None:
                    self._imu_yaw0 = y

    # ------------------------------------------------------------------
    def cb_js(self, m: JointState):
        try:
            il = m.name.index(self.left_joint)
            ir = m.name.index(self.right_joint)
        except ValueError:
            if not self._warned_missing:
                self._warned_missing = True
                self.get_logger().error(
                    f"/joint_states khong co '{self.left_joint}' / "
                    f"'{self.right_joint}'. Dang co: {list(m.name)}")
            return
        if len(m.position) <= max(il, ir):
            return

        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        pl, pr = m.position[il], m.position[ir]

        with self._lock:
            if self._prev is None:
                self._prev = (t, pl, pr)
                return
            t0, pl0, pr0 = self._prev
            dt = t - t0
            if dt <= 0.0 or dt > 1.0:
                # Dau thoi gian giat lui hoac nghi qua lau -> lay lai moc,
                # dung tinh mot buoc khong lo.
                self._prev = (t, pl, pr)
                return

            dphi_l = pl - pl0
            dphi_r = pr - pr0
            if abs(dphi_l) > JUMP_RAD or abs(dphi_r) > JUMP_RAD:
                self.get_logger().warn(
                    'Goc banh nhay dot ngot - co le ESP32 vua khoi dong lai. '
                    'Lay lai moc, giu nguyen vi tri dang co.')
                self._prev = (t, pl, pr)
                return

            d_l = dphi_l * self.R
            d_r = dphi_r * self.R
            d_center = 0.5 * (d_l + d_r)

            if self.yaw_source == 'imu_quaternion' and self._imu_yaw is not None:
                new_th = wrap(self._imu_yaw - self._imu_yaw0)
                d_theta = wrap(new_th - self.th)
            elif self.yaw_source == 'imu_gyro':
                d_theta = self._gyro_z * dt
            else:
                d_theta = (d_r - d_l) / self.sep

            # Runge-Kutta bac 2: lay goc o GIUA chu ky, chinh xac hon Euler
            # thuan khi xe vua chay vua quay.
            th_mid = self.th + 0.5 * d_theta
            self.x += d_center * math.cos(th_mid)
            self.y += d_center * math.sin(th_mid)
            self.th = wrap(self.th + d_theta)
            self.v = d_center / dt
            self.w = d_theta / dt
            self._prev = (t, pl, pr)

            x, y, th, v, w = self.x, self.y, self.th, self.v, self.w

        self.publish(m.header.stamp, x, y, th, v, w)

    # ------------------------------------------------------------------
    def publish(self, stamp, x, y, th, v, w):
        qz = math.sin(th * 0.5)
        qw = math.cos(th * 0.5)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w
        # Encoder kha tin o x/y, trung binh o yaw; z/roll/pitch vo nghia voi xe
        # chay tren mat phang nen dat rat lon de AMCL/EKF bo qua.
        o.pose.covariance[0] = 0.002
        o.pose.covariance[7] = 0.002
        o.pose.covariance[14] = 1e6
        o.pose.covariance[21] = 1e6
        o.pose.covariance[28] = 1e6
        o.pose.covariance[35] = 0.005
        o.twist.covariance[0] = 0.002
        o.twist.covariance[35] = 0.005
        self.pub.publish(o)

        if self.tfb is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tfb.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch gui SIGINT roi SIGTERM; cai thu hai thanh
        # ExternalShutdownException. Bat ca hai de tat cho sach, khong de lai
        # traceback lam nguoi doc tuong co loi.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Tim xem xe chay nguoc la do TAY CAM dao dau hay do DONG CO lap nguoc.

    cd ~/robot_ws_3 && python3 tools/check_direction.py

Hai nguyen nhan cho cung mot trieu chung "tien thanh lui, trai thanh phai":
  - tay cam dao dau  -> chi lai tay bi nguoc, Nav2 van chay dung
  - ca hai dong co lap nguoc -> MOI THU nguoc, va odometry noi doi (bao tien
    trong khi xe lui), SLAM se sai hoan toan

Phan biet bang cach xem /cmd_vel: no la LENH gui xuong, nam giua tay cam va
dong co.

CACH LAM:
  1. Ke banh xe len khoi mat dat, hoac de cho xe co duong chay thang.
  2. Chay file nay.
  3. Khi thay dong "BAT DAU DO", GIU LB va DAY CAN LEN het co, giu nguyen
     cho den khi no in ket qua.
  4. Doc ket luan.

Thuan doc, khong publish gi."""
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

DUR = 8.0


class D(Node):
    def __init__(self):
        super().__init__('check_direction')
        self.joy_y = []
        self.joy_x = []
        self.cmd_v = []
        self.cmd_w = []
        self.odom_v = []
        self.odom_w = []
        rel = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=20)
        be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(Joy, '/joy', self.cb_joy, be)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, rel)
        self.create_subscription(Odometry, '/odom', self.cb_odom, rel)
        self.recording = False

    def cb_joy(self, m):
        if self.recording and len(m.axes) > 1:
            self.joy_x.append(m.axes[0])
            self.joy_y.append(m.axes[1])

    def cb_cmd(self, m):
        if self.recording:
            self.cmd_v.append(m.linear.x)
            self.cmd_w.append(m.angular.z)

    def cb_odom(self, m):
        if self.recording:
            self.odom_v.append(m.twist.twist.linear.x)
            self.odom_w.append(m.twist.twist.angular.z)


def peak(xs):
    """Gia tri xa 0 nhat - can day het co nen lay dinh, khong lay trung binh."""
    return max(xs, key=abs) if xs else 0.0


def main():
    rclpy.init()
    n = D()

    end = time.time() + 6
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

    print("\n>>> BAT DAU DO - GIU LB VA DAY CAN LEN HET CO ngay bay gio <<<\n")
    n.recording = True
    end = time.time() + DUR
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.recording = False

    jy, jx = peak(n.joy_y), peak(n.joy_x)
    cv, cw = peak(n.cmd_v), peak(n.cmd_w)
    ov = peak(n.odom_v)

    print("=== do duoc ===")
    print(f"  /joy   axes[1] (can doc)  dinh {jy:+.2f}   "
          f"({len(n.joy_y)} mau)")
    print(f"  /joy   axes[0] (can ngang) dinh {jx:+.2f}")
    print(f"  /cmd_vel linear.x          dinh {cv:+.3f} m/s  "
          f"({len(n.cmd_v)} mau)")
    print(f"  /cmd_vel angular.z         dinh {cw:+.3f} rad/s")
    print(f"  /odom    twist.linear.x    dinh {ov:+.3f} m/s  "
          f"({len(n.odom_v)} mau)")

    print("\n=== ket luan ===")
    if not n.cmd_v:
        print("  Khong nhan duoc /cmd_vel nao.")
        print("  -> Co giu LB khong? teleop_twist_joy co dang chay khong?")
    elif abs(cv) < 0.01:
        print("  /cmd_vel gan nhu bang 0 -> lenh khong den duoc.")
        print("  -> Kiem tra joy_udp_bridge.py va windows_gamepad_bridge.py.")
    elif cv < 0:
        print("  DAY CAN LEN NHUNG /cmd_vel AM -> loi o chuoi TAY CAM.")
        print("  Xe va dong co khong co toi. Sua o phia ROS:")
        print("    config/joy_teleop.yaml -> doi dau scale_linear.x va")
        print("    scale_linear_turbo.x (teleop_twist_joy khong co co dao dau,")
        print("    dung he so am). Tuong tu cho scale_angular neu re cung nguoc.")
        print("  KHONG dung den INVERT_MOTOR_x trong firmware.")
    else:
        print("  /cmd_vel DUONG = lenh dung la TIEN.")
        print(f"  odom bao {ov:+.3f} m/s.")
        print("  Gio tra loi: LUC DO XE CHAY VE PHIA NAO?")
        print()
        print("   - Xe chay LUI  -> ca hai dong co lap nguoc. Odometry dang")
        print("     NOI DOI (bao tien khi xe lui) nen SLAM chac chan sai.")
        print("     Sua trong firmware: dao CA BON co cung luc")
        print("       INVERT_MOTOR_L false->true   INVERT_MOTOR_R true->false")
        print("       INVERT_ENC_L   false->true   INVERT_ENC_R   true->false")
        print("     Phai dao ca encoder, neu chi dao dong co thi bao ve")
        print("     'banh quay nguoc chieu PWM' se kich ESTOP.")
        print("   - Xe chay TIEN -> lenh va xe deu dung, khong co loi gi o day.")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

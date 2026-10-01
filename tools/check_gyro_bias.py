#!/usr/bin/env python3
"""Do do lech khong (bias) cua gyro Z - nguyen nhan kinh dien cua kieu loi
"chay dau thi khop, cang lau cang lech".

    cd ~/robot_ws_3 && python3 tools/check_gyro_bias.py

CACH LAM: de xe DUNG YEN HOAN TOAN tren san, khong cham vao, trong 60 giay.

VI SAO:
  Firmware dat USE_GYRO_FOR_YAW = 1, tuc goc quay cua odometry duoc TICH PHAN
  tu gyro_z:  odom_th += gyro_z_rps * dt  (50 Hz, trong motor_task).
  Khong co buoc tru bias nao. Neu gyro doc lech 0.005 rad/s luc dung yen thi
  moi phut odom tu quay them 17 do du xe khong he nhuc nhich. Ban do se khop o
  dau roi lech dan - dung nhu trieu chung.

  Cong cu cung so sanh voi QUATERNION cua BNO055. BNO055 chay che do NDOF tu
  hop gia toc + tu ke nen quaternion KHONG troi, trong khi tich phan gyro thi
  co. Hai so lech nhau bao nhieu chinh la luong troi.

Thuan doc, khong publish gi."""
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu

DUR = 60.0


class G(Node):
    def __init__(self):
        super().__init__('check_gyro_bias')
        self.t = []
        self.wz = []
        self.yaw = []
        self.create_subscription(
            Imu, '/imu/data', self.cb,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=20))

    def cb(self, m):
        q = m.orientation
        self.t.append(time.time())
        self.wz.append(m.angular_velocity.z)
        self.yaw.append(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def main():
    rclpy.init()
    n = G()
    end = time.time() + 6
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.t.clear(); n.wz.clear(); n.yaw.clear()

    print(f"\n>>> DE XE DUNG YEN HOAN TOAN trong {DUR:.0f} giay, dung cham vao <<<\n")
    t0 = time.time()
    end = t0 + DUR
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
        left = end - time.time()
        if int(left) % 10 == 0 and left > 1:
            print(f"  con {left:.0f} s   ", end='\r', flush=True)
    dt = time.time() - t0

    if len(n.wz) < 50:
        print(f"Chi nhan duoc {len(n.wz)} mau IMU. Agent va ESP32 co chay khong?")
        n.destroy_node(); rclpy.shutdown(); return

    bias = statistics.mean(n.wz)
    noise = statistics.pstdev(n.wz)
    # Tich phan hinh thang - chinh la phep ma firmware dang lam
    drift = 0.0
    for (ta, wa), (tb, wb) in zip(zip(n.t, n.wz), zip(n.t[1:], n.wz[1:])):
        drift += 0.5 * (wa + wb) * (tb - ta)

    dyaw_q = math.degrees((n.yaw[-1] - n.yaw[0] + math.pi) % (2 * math.pi) - math.pi)

    print(f"\n=== {len(n.wz)} mau trong {dt:.1f} s, xe dung yen ===")
    print(f"  bias gyro Z          : {bias:+.5f} rad/s  "
          f"({math.degrees(bias):+.3f} do/s)")
    print(f"  nhieu (do lech chuan): {noise:.5f} rad/s")
    print(f"  TICH PHAN gyro       : {math.degrees(drift):+.2f} do "
          f"trong {dt:.0f} s")
    print(f"  quaternion BNO055 doi: {dyaw_q:+.2f} do")

    per_min = math.degrees(bias) * 60
    print(f"\n=== ket luan ===")
    print(f"  Troi do bias: {per_min:+.1f} do/phut, "
          f"{per_min*5:+.0f} do sau 5 phut chay.")
    if abs(per_min) < 1.0:
        print("  Duoi 1 do/phut - khong dang ke, bias khong phai nguyen nhan.")
    elif abs(per_min) < 5.0:
        print("  1-5 do/phut - dang ke voi phien ve ban do dai. Nen tru bias.")
    else:
        print("  TREN 5 do/phut - DAY CHINH LA NGUYEN NHAN ban do lech dan.")
    print(f"\n  Gia tri can tru trong firmware:")
    print(f"      #define GYRO_Z_BIAS_RPS  {bias:+.5f}f")
    print(f"  (tot nhat la do lai bias moi lan khoi dong, xem goi y ben duoi)")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

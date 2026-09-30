#!/usr/bin/env python3
"""Thu subscribe tung topic cua ESP32 bang ca RELIABLE lan BEST_EFFORT.

    cd ~/robot_ws_3 && python3 tools/check_qos.py

VI SAO CAN: QoS lech thi ROS 2 khong chuyen MOT message nao, va khong bao loi
o cho de thay - chi co mot dong WARN chim trong log. Da dinh dung bay nay:
firmware phat ca 4 topic o best-effort, nhung tf2_ros::TransformListener luon
subscribe /tf o RELIABLE, nen tf2 khong nhan duoc gi, frame "odom" khong ton
tai, slam_toolbox loai sach scan va khong bao gio sinh ra ban do.

KET QUA MONG DOI sau khi da sua firmware:
    /tf        RELIABLE     nhan duoc      <- tf2 can
    /odom      RELIABLE     nhan duoc      <- nav2_util::OdomSmoother can
    /scan      BEST_EFFORT  nhan duoc      <- SensorDataQoS, dung la best-effort
    /imu/data  BEST_EFFORT  nhan duoc      <- nt

Luu y: /tf con co robot_state_publisher phat (reliable) nen dong /tf RELIABLE
co the "nhan duoc" du ESP32 van sai. Muon chac thi xem tools/tftree tren cay
TF co frame "odom" khong.

Thuan doc, khong publish gi - an toan khi xe dang de tren san."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

CASES = [('/tf', TFMessage), ('/odom', Odometry),
         ('/imu/data', Imu), ('/scan', LaserScan)]


def try_one(topic, msg_type, rel):
    name = 'RELIABLE' if rel == QoSReliabilityPolicy.RELIABLE else 'BEST_EFFORT'
    n = Node('qos_test')
    got = []
    n.create_subscription(
        msg_type, topic, lambda m: got.append(1),
        QoSProfile(reliability=rel, history=QoSHistoryPolicy.KEEP_LAST, depth=10))
    end = time.time() + 6
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    print(f"  {topic:12s} subscribe {name:12s} -> "
          f"{'nhan ' + str(len(got)) + ' msg' if got else '*** KHONG NHAN GI ***'}")
    return bool(got)


def main():
    rclpy.init()
    print("=== ESP32 phat topic o che do nao? ===")
    for topic, t in CASES:
        try_one(topic, t, QoSReliabilityPolicy.RELIABLE)
        try_one(topic, t, QoSReliabilityPolicy.BEST_EFFORT)
        print()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

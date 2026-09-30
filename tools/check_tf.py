#!/usr/bin/env python3
"""Xem cay TF that su dang co gi, va thu tra cuu dung phep bien doi ma
slam_toolbox can.

    cd ~/robot_ws_3 && python3 tools/check_tf.py

Phai thay frame "odom" trong cay, va dong "odom -> lidar_link  OK".
Neu khong co "odom": ESP32 khong den duoc tf2. Xem tools/check_qos.py -
day tung la loi lech QoS tren /tf.

Thuan doc, khong publish gi."""
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros


def main():
    rclpy.init()
    n = Node('tftree')
    buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
    tf2_ros.TransformListener(buf, n)

    end = time.time() + 12
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)

    print("=== cay TF ===")
    print(buf.all_frames_as_string())

    now = n.get_clock().now()
    print(f"gio may (ROS): {now.nanoseconds * 1e-9:.3f}")

    for a, b in (('odom', 'base_footprint'), ('base_footprint', 'base_link'),
                 ('base_link', 'lidar_link'), ('odom', 'lidar_link'),
                 ('map', 'odom')):
        # time=0 nghia la "ban moi nhat co san"
        try:
            t = buf.lookup_transform(a, b, Time())
            ts = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            age = now.nanoseconds * 1e-9 - ts
            print(f"  {a:16s} -> {b:16s} OK   stamp={ts:.3f} (tuoi {age:+.3f} s)")
        except Exception as e:
            print(f"  {a:16s} -> {b:16s} HONG: {type(e).__name__}: {e}")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

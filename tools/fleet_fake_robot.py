#!/usr/bin/env python3
"""Xe AO dong hoc cho mo phong nhieu xe (khong can Gazebo, khong can xe that).

- cmd_vel -> van toc theo quan tinh bac nhat (tau 0.12 s), tre lenh, vung chet
  dong co, mat lenh 0.5 s thi dung - giong firmware;
- phat odom, TF odom->base_footprint, TF tinh map->odom (dinh vi hoan hao,
  thay AMCL) va base_footprint->lidar_link;
- LiDAR ao: thay XE KHAC (hinh tron duong kinh 18 cm) - de thu chuyen hai
  xe thay nhau la vat can - va VAT CAN dat tu ngoai qua topic /sim/obstacles
  (std_msgs/String "x,y;x,y", moi vat la hinh tron duong kinh 20 cm; chuoi
  rong = bo het).

    python3 tools/fleet_fake_robot.py X Y YAW_DO --other /agv2/odom \\
        --ros-args -r __ns:=/agv1 -r /tf:=/agv1/tf -r /tf_static:=/agv1/tf_static
Bien moi truong: FAKE_DELAY (s, tre lenh -> banh; xe that ~0.3).
"""
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

TAU, DT = 0.12, 0.02
DEAD_V, DEAD_W = 0.04, 0.25
DELAY = float(os.environ.get('FAKE_DELAY', '0.3'))
BODY_R = 0.09
OBST_R = 0.10
N_RAYS = 360


class FakeRobot(Node):
    def __init__(self, x, y, yaw, others):
        super().__init__('fake_robot')
        self.x, self.y, self.th = x, y, yaw
        self.v = self.w = 0.0
        self.cmd, self.cmd_t = (0.0, 0.0), -1.0
        self.hist = []
        self.other_pose = {}
        self.obstacles = []
        self.create_subscription(Twist, 'cmd_vel', self.cb_cmd, 10)
        self.create_subscription(String, '/sim/obstacles', self.cb_obst, 10)
        for topic in others:
            self.create_subscription(Odometry, topic,
                                     lambda m, t=topic: self.cb_other(t, m), 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.tfb = TransformBroadcaster(self)
        st = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        a = TransformStamped()
        a.header.stamp, a.header.frame_id, a.child_frame_id = now, 'map', 'odom'
        a.transform.rotation.w = 1.0
        b = TransformStamped()
        b.header.stamp, b.header.frame_id, b.child_frame_id = now, 'base_footprint', 'lidar_link'
        b.transform.translation.z = 0.075
        b.transform.rotation.w = 1.0
        st.sendTransform([a, b])
        self.create_timer(DT, self.step)
        self.create_timer(0.11, self.scan)

    def cb_cmd(self, m):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.hist.append((t + DELAY, (m.linear.x, m.angular.z)))

    def cb_obst(self, m):
        obs = []
        for item in m.data.split(';'):
            if item.strip():
                x, y = (float(v) for v in item.split(','))
                obs.append((x, y, OBST_R))
        self.obstacles = obs

    def cb_other(self, topic, m):
        p = m.pose.pose.position          # map->odom la dong nhat nen odom = map
        self.other_pose[topic] = (p.x, p.y)

    def step(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        while self.hist and self.hist[0][0] <= t:
            self.cmd_t, self.cmd = self.hist.pop(0)
        vc, wc = self.cmd if t - self.cmd_t < 0.5 else (0.0, 0.0)
        vl, vr = vc - wc * 0.1, vc + wc * 0.1
        if max(abs(vl), abs(vr)) < DEAD_V and abs(wc) < DEAD_W:
            vc = wc = 0.0
        a = DT / TAU
        self.v += (vc - self.v) * a
        self.w += (wc - self.w) * a
        self.th += self.w * DT
        self.x += self.v * math.cos(self.th) * DT
        self.y += self.v * math.sin(self.th) * DT
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = now.to_msg(), 'odom', 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        self.tfb.sendTransform(tf)
        o = Odometry()
        o.header, o.child_frame_id = tf.header, 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation = tf.transform.rotation
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(o)

    def scan(self):
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'lidar_link'
        s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / N_RAYS
        s.angle_max = s.angle_min + (N_RAYS - 1) * s.angle_increment
        s.range_min, s.range_max = 0.12, 8.0
        rng = [float('inf')] * N_RAYS
        circles = [(ox, oy, BODY_R) for ox, oy in self.other_pose.values()] + self.obstacles
        for ox, oy, rad in circles:
            dx, dy = ox - self.x, oy - self.y
            d = math.hypot(dx, dy)
            if d <= rad or d > s.range_max:
                continue
            centre = math.atan2(dy, dx) - self.th
            half = math.asin(min(1.0, rad / d))
            for k in range(N_RAYS):
                a = s.angle_min + k * s.angle_increment
                diff = math.atan2(math.sin(a - centre), math.cos(a - centre))
                if abs(diff) <= half:
                    # giao tia voi hinh tron
                    proj = d * math.cos(diff)
                    perp = d * math.sin(diff)
                    r = proj - math.sqrt(max(0.0, rad ** 2 - perp ** 2))
                    rng[k] = min(rng[k], max(r, s.range_min))
        s.ranges = rng
        self.scan_pub.publish(s)


def main():
    rclpy.init(args=sys.argv)
    argv = remove_ros_args(sys.argv)[1:]
    others = []
    if '--other' in argv:
        i = argv.index('--other')
        others = argv[i + 1].split(',')
        argv = argv[:i] + argv[i + 2:]
    x, y, yaw = float(argv[0]), float(argv[1]), math.radians(float(argv[2]))
    node = FakeRobot(x, y, yaw, others)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

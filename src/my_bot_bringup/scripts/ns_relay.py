#!/usr/bin/env python3
"""Chuyen tiep topic giua namespace cua MOT xe va goc - de chay 1 xe nhu cu.

Firmware dat moi topic vao /agv<XE>/ (de hai xe dung chung mot agent). Chay
MOT xe thi Nav2, AMCL, grid_navigator, RViz van dung /scan, /cmd_vel... o goc,
nen node nay noi hai ben:

    /agvN/scan          -> /scan            (best-effort, giong ESP32)
    /agvN/imu/data      -> /imu/data        (best-effort)
    /agvN/joint_states  -> /joint_states    (best-effort)
    /cmd_vel            -> /agvN/cmd_vel

Chuyen NGUYEN goi da tuan tu hoa (raw), khong giai ma - tre ~0.1 ms.
bringup_real.launch.py xe:=N tu chay node nay; chay tay:
    ros2 run my_bot_bringup ns_relay.py agv2
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState, LaserScan

BEST_EFFORT = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
RELIABLE = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=10)


class NsRelay(Node):
    def __init__(self, ns):
        super().__init__('ns_relay')
        self.pubs = []
        for name, typ in (('scan', LaserScan), ('imu/data', Imu), ('joint_states', JointState)):
            self.relay(typ, f'/{ns}/{name}', f'/{name}', BEST_EFFORT, BEST_EFFORT)
        # cmd_vel: phat RELIABLE nhu Nav2 van gui thang cho ESP32 truoc day -
        # ESP32 nghe best-effort van nhan duoc (con nguoc lai thi khong).
        self.relay(Twist, '/cmd_vel', f'/{ns}/cmd_vel', RELIABLE, RELIABLE)
        self.get_logger().info(f'Chuyen tiep /{ns}/{{scan, imu/data, joint_states}} -> goc, '
                               f'/cmd_vel -> /{ns}/cmd_vel')

    def relay(self, typ, src, dst, qos_in, qos_out):
        pub = self.create_publisher(typ, dst, qos_out)
        self.create_subscription(typ, src, pub.publish, qos_in, raw=True)
        self.pubs.append(pub)


def main():
    rclpy.init(args=sys.argv)
    argv = remove_ros_args(sys.argv)[1:]
    if not argv or not argv[0].startswith('agv'):
        print('Dung: ns_relay.py agv<so xe>   (vd agv2)')
        return
    node = NsRelay(argv[0])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

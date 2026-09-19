#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class LidarController(Node):
    def __init__(self):
        super().__init__('lidar_controller')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )
        
        self.obstacle_detected = False
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Hệ thống LiDAR Controller đã khởi động. Sẵn sàng tránh vật cản!")

    def scan_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
        
        if num_ranges > 0:
            # Lidar quét từ -180 đến 180 độ. Tia ở giữa (index 180) chính là phía trước mũi xe.
            # Lấy 30 tia bên trái và 30 tia bên phải tính từ tia trung tâm.
            mid = num_ranges // 2
            front_angles = ranges[mid - 30 : mid + 30]
            
            valid_ranges = [r for r in front_angles if 0.1 < r < 8.0]
            
            if valid_ranges:
                min_distance = min(valid_ranges)
                if min_distance < 1.0:
                    if not self.obstacle_detected:
                        self.get_logger().warn(f"Cảnh báo: Phát hiện vật cản cách {min_distance:.2f}m ở phía trước!")
                        self.obstacle_detected = True
                else:
                    self.obstacle_detected = False

    def control_loop(self):
        cmd = Twist()
        
        if self.obstacle_detected:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 
        else:
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
            
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LidarController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""Nhan UDP tu tools/windows_gamepad_bridge.py -> phat topic /joy.

Chay trong WSL:
    python3 tools/joy_udp_bridge.py

Ket qua la mot sensor_msgs/Joy binh thuong, nen teleop_twist_joy dung y nguyen
config/joy_teleop.yaml - khong phai viet lai logic ti le toc do hay nut chet.

Chuoi day du:
    Windows: windows_gamepad_bridge.py  (doc XInput, gui UDP)
        -> WSL: joy_udp_bridge.py       (phat /joy)
        -> teleop_twist_joy             (/joy -> /cmd_vel)
        -> ESP32                        (/cmd_vel -> banh xe)

Tham so:
    port        (int,   9002)  cong UDP, phai khop DEST_PORT ben Windows
    deadzone    (float, 0.10)  bo qua do rung cua can quanh vi tri giua
    timeout_sec (float, 0.30)  mat goi lau hon nay -> phat /joy toan 0
"""

import json
import socket
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

N_AXES = 6
N_BUTTONS = 14


class JoyUdpBridge(Node):
    def __init__(self):
        super().__init__('joy_udp_bridge')

        self.declare_parameter('port', 9002)
        self.declare_parameter('deadzone', 0.10)
        self.declare_parameter('timeout_sec', 0.30)

        port = self.get_parameter('port').value
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.pub = self.create_publisher(Joy, 'joy', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)

        self.last_rx_time = None
        self.packets = 0
        self.zero_sent = False

        # Doc socket that nhanh; phat /joy moi lan co goi moi.
        self.create_timer(0.005, self.poll)
        self.create_timer(2.0, self.report)

        self.get_logger().info(
            f"Dang cho goi UDP o cong {port}. Chay windows_gamepad_bridge.py tren Windows.")

    # ------------------------------------------------------------------
    def apply_deadzone(self, v):
        if abs(v) < self.deadzone:
            return 0.0
        # Gian lai phan con lai ve day du -1..1 de khong bi nhay bac
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - self.deadzone) / (1.0 - self.deadzone)

    def publish_joy(self, axes, buttons):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joy'
        msg.axes = [float(a) for a in axes]
        msg.buttons = [int(b) for b in buttons]
        self.pub.publish(msg)

    def poll(self):
        got = False
        # Doc het goi dang xep hang, chi giu goi MOI NHAT: tay cam la trang
        # thai hien tai, khong phai hang doi su kien.
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                latest = data
                got = True
            except BlockingIOError:
                break
            except OSError as e:
                self.get_logger().warn(f"Loi doc socket: {e}")
                break

        now = self.get_clock().now()

        if got and latest is not None:
            try:
                p = json.loads(latest.decode('ascii'))
                axes = [self.apply_deadzone(float(v)) for v in p['axes']][:N_AXES]
                buttons = [int(v) for v in p['buttons']][:N_BUTTONS]
            except (ValueError, KeyError, TypeError) as e:
                self.get_logger().warn(f"Goi UDP khong doc duoc: {e}")
                return
            axes += [0.0] * (N_AXES - len(axes))
            buttons += [0] * (N_BUTTONS - len(buttons))
            self.publish_joy(axes, buttons)
            self.last_rx_time = now
            self.packets += 1
            self.zero_sent = False
            return

        # Mat lien lac -> phat mot /joy toan 0 de teleop ha /cmd_vel ve 0.
        # Khong co doan nay thi teleop giu gia tri cuoi va xe chay tiep khi
        # script ben Windows bi tat hay may treo.
        if self.last_rx_time is not None and not self.zero_sent:
            dt = (now - self.last_rx_time).nanoseconds * 1e-9
            if dt > self.timeout_sec:
                self.publish_joy([0.0] * N_AXES, [0] * N_BUTTONS)
                self.zero_sent = True
                self.get_logger().warn(
                    f"Mat goi UDP {dt:.2f}s -> da phat /joy toan 0 de dung xe.")

    def report(self):
        if self.packets == 0:
            self.get_logger().warn("Chua nhan goi nao. Script ben Windows da chay chua?")
        else:
            self.get_logger().info(f"Da nhan {self.packets} goi.")


def main():
    rclpy.init()
    node = JoyUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

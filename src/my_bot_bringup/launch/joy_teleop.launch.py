"""Lai xe bang tay cam -> /cmd_vel.

Dung thay cho teleop_twist_keyboard khi ve ban do: can analog cho toc do
muot hon nen ban do it nhoe hon han so voi dieu khien bang ban phim.

Chay:
    ros2 launch my_bot_bringup joy_teleop.launch.py

Tham so:
    device_id:=1     doi neu may co nhieu tay cam / thiet bi input
    cmd_vel_topic:=...  doi neu can bom vao topic khac

WSL: tay cam USB khong tu hien trong WSL, phai gan qua usbipd-win truoc.
Kiem tra da thay tay cam chua:
    ls /dev/input/event*
    ros2 run joy joy_enumerate_devices

Chi so truc/nut khac nhau giua cac tay cam - xem ghi chu trong
config/joy_teleop.yaml de do lai bang 'ros2 topic echo /joy'.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('my_bot_bringup')
    params_file = os.path.join(bringup_pkg, 'config', 'joy_teleop.yaml')

    device_id = LaunchConfiguration('device_id')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0',
                              description='Thiet bi tay cam thu may (0 = dau tien)'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='Topic nhan lenh toc do'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true khi lai xe trong Gazebo'),

        # Doc tay cam -> /joy. Qua SDL2 nen dung /dev/input/event*,
        # khong can /dev/input/js0 (kernel WSL khong co joydev).
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[params_file,
                        {'device_id': device_id, 'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # /joy -> /cmd_vel
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', cmd_vel_topic)],
            output='screen',
        ),
    ])

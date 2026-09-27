"""Khoi dong xe THAT (thay cho gazebo.launch.py).

Tren xe that, /scan /odom /tf /imu/data deu do ESP32 phat qua micro-ROS,
nen o day chi con 3 viec:
    1. micro_ros_agent  - cau noi UDP giua ESP32 va ROS 2
    2. robot_state_publisher - TF tinh giua cac link (base_link -> lidar_link...)
    3. joint_state_publisher - goc banh xe gia, chi de RViz khong bao loi do

Firmware: firmware/esp32_agv_base/esp32_agv_base.ino

Cai micro_ros_agent truoc khi chay lan dau:
    sudo apt install ros-jazzy-micro-ros-agent
hoac build tu nguon neu apt khong co goi nay:
    https://github.com/micro-ROS/micro-ROS-Agent

Chay:
    ros2 launch my_bot_bringup bringup_real.launch.py
Roi cap nguon ESP32. Log agent phai hien "client_key" va danh sach topic.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory('my_bot_description')
    xacro_file = os.path.join(description_pkg, 'description', 'robot.urdf.xacro')

    start_agent = LaunchConfiguration('start_agent')
    agent_port = LaunchConfiguration('agent_port')

    # sim:=false -> bo plugin DiffDrive va sensor gpu_lidar cua Gazebo,
    # vi tren xe that ESP32 lo phan do.
    robot_description = {
        'robot_description': Command(['xacro ', xacro_file, ' sim:=false']),
        'use_sim_time': False,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_agent', default_value='true',
            description='false neu ban tu chay micro_ros_agent o terminal rieng'),
        DeclareLaunchArgument(
            'agent_port', default_value='8888',
            description='Phai khop AGENT_PORT trong esp32_agv_base.ino'),

        # 1. Cau noi micro-ROS. udp4 vi firmware dung Wi-Fi.
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['udp4', '--port', agent_port],
            output='screen',
            condition=IfCondition(start_agent),
        ),

        # 2. TF tinh giua cac link, doc tu URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        # 3. Banh xe la khop continuous nen robot_state_publisher doi /joint_states.
        #    Xe that khong phat topic nay -> node nay bom goc 0 vao cho du TF.
        #    Thuan tuy hien thi: banh trong RViz se khong quay.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': False}],
        ),
    ])

"""Khoi dong xe THAT (thay cho gazebo.launch.py).

Tren xe that, /scan /odom /tf /imu/data deu do ESP32 phat qua micro-ROS,
nen o day chi con 3 viec:
    1. micro_ros_agent  - cau noi UDP giua ESP32 va ROS 2
    2. robot_state_publisher - TF tinh giua cac link (base_link -> lidar_link...)
    3. joint_state_publisher - goc banh xe gia, chi de RViz khong bao loi do

Firmware: firmware/esp32_agv_base/esp32_agv_base.ino

QUAN TRONG - micro_ros_agent nam o workspace RIENG:
Khong co goi apt ros-jazzy-micro-ros-agent (apt chi co micro-ros-msgs va
micro-ros-diagnostic-*). Tren may nay agent da duoc build san o ~/microros_ws,
nen PHAI source workspace do truoc, neu khong launch se bao
"package 'micro_ros_agent' not found":

    source ~/microros_ws/install/setup.bash

Nen them thang vao ~/.bashrc cho khoi quen.

Neu may khac chua co agent, dung Docker cho nhanh:
    docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888
roi launch voi start_agent:=false

Chay:
    ros2 launch my_bot_bringup bringup_real.launch.py
Roi cap nguon ESP32. Log agent phai hien "client_key" va danh sach topic.
"""

import os
from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory('my_bot_description')
    xacro_file = os.path.join(description_pkg, 'description', 'robot.urdf.xacro')

    start_agent = LaunchConfiguration('start_agent')
    agent_port = LaunchConfiguration('agent_port')

    # micro_ros_agent KHONG co trong apt (khong ton tai goi
    # ros-jazzy-micro-ros-agent). Neu chua cai thi launch se do ngay voi
    # "package 'micro_ros_agent' not found" ma khong noi phai lam gi. Kiem
    # truoc de bo qua node do va in huong dan, phan con lai van chay duoc.
    try:
        get_package_share_directory('micro_ros_agent')
        agent_installed = True
    except PackageNotFoundError:
        agent_installed = False

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
        *([Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['udp4', '--port', agent_port],
            output='screen',
            condition=IfCondition(start_agent),
        )] if agent_installed else [
            LogInfo(msg='=========================================================='),
            LogInfo(msg='CHUA CAI micro_ros_agent -> bo qua, ESP32 se KHONG ket noi.'),
            LogInfo(msg='Khong co goi apt ros-jazzy-micro-ros-agent. Chon 1 trong 2:'),
            LogInfo(msg=''),
            LogInfo(msg='  (a) Docker, nhanh nhat:'),
            LogInfo(msg='      docker run -it --rm --net=host '
                        'microros/micro-ros-agent:jazzy udp4 --port 8888'),
            LogInfo(msg=''),
            LogInfo(msg='  (b) Build tu nguon, xem README trong tools/'),
            LogInfo(msg=''),
            LogInfo(msg='Chay agent o terminal rieng roi launch lai voi:'),
            LogInfo(msg='      start_agent:=false'),
            LogInfo(msg='=========================================================='),
        ]),

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

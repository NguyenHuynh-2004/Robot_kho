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
from launch_ros.parameter_descriptions import ParameterValue


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
    # ParameterValue(..., value_type=str) la BAT BUOC, khong phai cho dep.
    # Command() tra ve mot substitution, va launch_ros se thu yaml.safe_load
    # chuoi ket qua de doan kieu. Chuoi do la ca file URDF, nen chi can trong
    # no co mot dong ket thuc bang dau hai cham la YAML coi do la khoa cua mot
    # mapping va bao:
    #     Unable to parse the value of parameter robot_description as yaml
    # URDF cu khong co dong nao nhu vay nen tinh co chay duoc - do la an may,
    # khong phai dung. Ep kieu str thi launch_ros khong parse nua.
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' sim:=false']), value_type=str),
        'use_sim_time': False,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_agent', default_value='true',
            description='false neu ban tu chay micro_ros_agent o terminal rieng'),
        DeclareLaunchArgument(
            'agent_port', default_value='8888',
            description='Phai khop AGENT_PORT trong esp32_agv_base.ino'),
        # DA HIEU CHINH, va lan nay bang phep do LAP LAI DUOC.
        #
        # Ba lan chay thang 2.00 m, sau khi da sua ENC_PULSES_PER_REV 1200->2150:
        #       1.967 m   1.953 m   1.978 m     -> trung binh 1.966 m
        # Tan mat 1.3%, nen ti so dang tin. He so 2.000/1.966 = 1.0173.
        #       0.03225 * 1.0173 = 0.03281
        #
        # (Truoc khi sua hang so encoder, ba phep tuong tu cho 1.955 roi 3.494 m
        #  - gap 1.79 lan - nen khong dung duoc. Chenh lech do chinh la
        #  2150/1200 = 1.79.)
        #
        # LUU Y: day KHONG con la ban kinh vat ly. 0.03281 lon hon ban kinh hinh
        # hoc 0.03225, trong khi lop bi nen thi phai cho so NHO hon. No la he so
        # hieu chinh gop ca phan du cua ENC_PULSES_PER_REV (do duoc 2137-2166
        # tuy banh, dang lam tron 2150). Doi dong co hay lop thi do lai.
        DeclareLaunchArgument(
            'wheel_radius', default_value='0.03281',
            description='Ban kinh LAN cua banh sau (m). Hieu chinh bang '
                        'tools/check_odom_scale.py - KHONG phai nap lai chip'),
        DeclareLaunchArgument(
            'yaw_source', default_value='imu_quaternion',
            description='imu_quaternion (khong troi, nen dung) | imu_gyro '
                        '(tich phan, giong firmware cu) | wheels (du phong)'),

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

        # 3. Odometry tinh TREN LAPTOP tu /joint_states + /imu/data.
        #    Truoc day ESP32 tu tinh va phat /odom + /tf, con /joint_states thi
        #    joint_state_publisher bom goc 0 gia vao cho du TF (banh trong RViz
        #    khong bao gio quay). Gio ESP32 phat /joint_states THAT nen
        #    joint_state_publisher khong con can nua, va doi mot y tuong ve
        #    odometry khong phai nap lai chip.
        Node(
            package='my_bot_bringup',
            executable='wheel_odometry.py',
            name='wheel_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                # Ban kinh LAN, nho hon ban kinh hinh hoc 0.03225 vi lop bi nen.
                # Hieu chinh bang tools/check_odom_scale.py roi sua SO NAY -
                # khong phai nap lai firmware nua.
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'wheel_separation': 0.200,
                'yaw_source': LaunchConfiguration('yaw_source'),
            }],
        ),
    ])

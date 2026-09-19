import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # 1. Đường dẫn đến các gói hệ thống
    description_pkg = get_package_share_directory('my_bot_description')
    bringup_pkg = get_package_share_directory('my_bot_bringup')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # 2. Xử lý file Xacro thành dữ liệu URDF thuần túy
    xacro_file = os.path.join(description_pkg, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}

    # 3. Đường dẫn đến file mô phỏng World (.sdf) và cấu hình cầu nối
    world_file = os.path.join(bringup_pkg, 'worlds', 'my_world.sdf')
    bridge_config_file = os.path.join(bringup_pkg, 'config', 'ros_gz_bridge.yaml')

    # --- CÁC NODE ROS 2 ---

    # Node 1: Robot State Publisher (Xuất bản dữ liệu TF - Tọa độ của các khớp)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Node 2: Khởi động Gazebo Harmonic cùng với file World.
    # Mặc định hiện GUI; chạy 'headless:=true' để chỉ bật server (dùng khi test tự động
    # hoặc chạy trên máy không có màn hình).
    headless = LaunchConfiguration('headless')
    gz_flags = PythonExpression([
        "'-s -r' if '", headless, "'.lower() in ('true', '1') else '-r'"
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [gz_flags, ' ', world_file]}.items()
    )

    # Node 3: Spawn Robot (Đẩy mô hình từ ROS 2 vào thế giới ảo Gazebo)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.1'],
        output='screen'
    )

    # Node 4: Cầu nối dữ liệu (Bridge) giữa ROS 2 và Gazebo Harmonic.
    # Danh sách topic nằm trong config/ros_gz_bridge.yaml để tránh khai báo hai nơi lệch nhau.
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # Node 5: Joint State Publisher (Khai báo góc xoay mặc định cho bánh xe để RViz2 không báo lỗi đỏ)
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[robot_description]
    )

    # Đóng gói và trả về toàn bộ Launch Description
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='true = chi chay Gazebo server (khong GUI), dung khi test tu dong'
        ),
        node_robot_state_publisher,
        node_joint_state_publisher,
        gazebo,
        spawn_entity,
        bridge_node
    ])
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Truoc day use_sim_time bi dong cung = 'true' nen khong ve duoc ban do
    # tren xe that: slam_toolbox se doi /clock mai mai va khong xu ly scan nao.
    #   Mo phong: ros2 launch my_bot_bringup slam.launch.py
    #   Xe that : ros2 launch my_bot_bringup slam.launch.py use_sim_time:=false
    use_sim_time = LaunchConfiguration('use_sim_time')

    start_async_slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='true khi chay Gazebo, false khi ve ban do tren xe that'),
        start_async_slam_cmd,
    ])

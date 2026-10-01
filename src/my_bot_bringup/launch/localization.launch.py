"""Chi chay DINH VI tren ban do da luu: map_server + AMCL. Khong co bo dieu khien.

    Xe that : ros2 launch my_bot_bringup localization.launch.py use_sim_time:=false
    Gazebo  : ros2 launch my_bot_bringup localization.launch.py

VI SAO TACH RIENG khoi navigation.launch.py:
  Kiem dinh vi truoc, dieu huong sau. navigation.launch.py bat ca bo dieu
  khien, velocity_smoother va collision_monitor - nhung node do co the phat
  /cmd_vel va tranh voi tay cam. O day chi co map_server va AMCL, nen lai tay
  thoai mai trong luc xem AMCL co bam dung khong.

PHAI TAT slam_toolbox truoc: ca hai deu phat TF map->odom, chay chung la hai
nguon su that tranh nhau.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    my_bot_dir = get_package_share_directory('my_bot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Cung file tham so voi navigation.launch.py, de AMCL o day va o do giong het
    nav2_params_path = os.path.join(my_bot_dir, 'config', 'navigation.yaml')
    sim_map = os.path.join(my_bot_dir, 'config', 'my_room_map.yaml')
    real_map = os.path.join(my_bot_dir, 'config', 'real_room_map.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
            'map': LaunchConfiguration('map'),
            'autostart': 'true',
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='true khi chay Gazebo, false khi chay xe that'),
        DeclareLaunchArgument(
            'map',
            default_value=PythonExpression([
                "'", sim_map, "' if '", use_sim_time, "' == 'true' else '",
                real_map, "'"]),
            description='Ban do. Mac dinh: Gazebo -> my_room_map, '
                        'xe that -> real_room_map'),
        localization,
    ])

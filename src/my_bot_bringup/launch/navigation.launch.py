import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    my_bot_dir = get_package_share_directory('my_bot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_path = os.path.join(my_bot_dir, 'config', 'navigation.yaml')
    map_yaml_path = os.path.join(my_bot_dir, 'config', 'my_room_map.yaml')

    # Mo phong dung /clock cua Gazebo, xe that dung dong ho he thong.
    #   Mo phong: ros2 launch my_bot_bringup navigation.launch.py
    #   Xe that : ros2 launch my_bot_bringup navigation.launch.py use_sim_time:=false
    #
    # navigation.yaml CO Y KHONG chua use_sim_time: nav2 cap tham so nay cho moi
    # node bang SetParameter tu launch argument (giong nav2_params.yaml goc cua
    # Jazzy). De lai trong yaml se thanh hai nguon su that, va vi nav2 Jazzy
    # khong ghi de use_sim_time qua param_rewrites nen rat de bi ket o gia tri cu.
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
            'map': map_yaml_path,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='true khi chay Gazebo, false khi chay xe that'),
        nav2_launch,
    ])

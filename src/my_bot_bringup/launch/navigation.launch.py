import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    my_bot_dir = get_package_share_directory('my_bot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_path = os.path.join(my_bot_dir, 'config', 'navigation.yaml')
    map_yaml_path = os.path.join(my_bot_dir, 'config', 'my_room_map.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path,
            'map': map_yaml_path,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([nav2_launch])
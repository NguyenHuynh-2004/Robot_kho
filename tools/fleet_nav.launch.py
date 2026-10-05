"""Nav2 (navigation_launch.py) cho MOT xe trong namespace rieng.

navigation_launch.py cua Jazzy chi them namespace vao TEN THAM SO (root_key),
khong day cac node vao namespace - viec do bringup_launch.py lam bang
PushROSNamespace. Goi thang navigation_launch.py namespace:=agv1 thi node van
la /controller_server, khong khop tham so agv1.controller_server -> nap bo dieu
khien mac dinh, chet voi "No critics defined for FollowPath".

    ros2 launch tools/fleet_nav.launch.py namespace:=agv1 params_file:=...
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushROSNamespace


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    params = LaunchConfiguration('params_file')
    nav = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('params_file'),
        GroupAction([
            PushROSNamespace(ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav),
                launch_arguments={'namespace': ns, 'params_file': params,
                                  'use_sim_time': 'false', 'autostart': 'true'}.items()),
        ]),
    ])

"""Nav2 (navigation_launch.py) cho MOT xe trong namespace rieng.

navigation_launch.py cua Jazzy chi them namespace vao TEN THAM SO (root_key),
khong day cac node vao namespace - viec do bringup_launch.py lam bang
PushROSNamespace. Goi thang navigation_launch.py namespace:=agv1 thi node van
la /controller_server, khong khop tham so agv1.controller_server -> nap bo dieu
khien mac dinh, chet voi "No critics defined for FollowPath".

    ros2 launch tools/fleet_nav.launch.py namespace:=agv1 params_file:=...

Nav2 cua moi xe chay GOP trong MOT tien trinh (container /<ns>/nav2_container,
giong bringup_launch.py cua xe don). Truoc day moi node mot tien trinh: hai xe
~26 tien trinh Nav2, cong agent/AMCL/odom... qua ~40 thanh vien DDS - voi
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST cac tien trinh bat SAU khong con duoc
ai thay (do duoc: fleet_agent agv2 chay ma /fleet/status khong co no).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    params = LaunchConfiguration('params_file')
    nav = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('params_file'),
        GroupAction([
            PushROSNamespace(ns),
            Node(name='nav2_container', package='rclcpp_components',
                 executable='component_container_isolated',
                 # File tham so (da gan namespace) cho CA tien trinh: costmap la node
                 # CON tao trong controller/planner, chi nhan tham so qua file cua
                 # tien trinh - thieu dong nay costmap chay mac dinh (khong /map,
                 # nghe nham topic scan -> "out of bounds", "timed out").
                 parameters=[ParameterFile(RewrittenYaml(source_file=params, root_key=ns,
                                                         param_rewrites={}, convert_types=True),
                                           allow_substs=True),
                             {'autostart': True, 'use_sim_time': False}],
                 remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                 # fleet_agent xoa local costmap moi 1 s (vat can KET) -> bo dong INFO
                 # "Received request to clear except a region" moi giay
                 arguments=['--ros-args', '--log-level',
                            [ns, '.local_costmap.local_costmap:=warn']],
                 output='screen'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav),
                launch_arguments={'namespace': ns, 'params_file': params,
                                  'use_sim_time': 'false', 'autostart': 'true',
                                  'use_composition': 'True',
                                  'container_name': 'nav2_container'}.items()),
        ]),
    ])

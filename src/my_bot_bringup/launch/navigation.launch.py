import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    my_bot_dir = get_package_share_directory('my_bot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_path = os.path.join(my_bot_dir, 'config', 'navigation.yaml')

    # HAI ban do, KHONG dung chung:
    #   my_room_map   - phong Gazebo 10x10 m (goc -5.1, -5.2), cho Che do A
    #   real_room_map - phong THAT, ve bang slam_toolbox tren xe, cho Che do B
    # Ghi ban do that de len my_room_map thi Gazebo se nap sai ban do.
    # Mac dinh tu chon theo use_sim_time; muon ban khac thi truyen map:=<duong dan>.
    sim_map = os.path.join(my_bot_dir, 'config', 'my_room_map.yaml')
    real_map = os.path.join(my_bot_dir, 'config', 'real_room_map.yaml')
    map_yaml_path = LaunchConfiguration('map')

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
        DeclareLaunchArgument(
            'map',
            default_value=PythonExpression([
                "'", sim_map, "' if '", use_sim_time, "' == 'true' else '",
                real_map, "'"]),
            description='Ban do cho AMCL. Mac dinh: Gazebo -> my_room_map, '
                        'xe that -> real_room_map'),
        nav2_launch,
    ])

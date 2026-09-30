import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    bringup_dir = get_package_share_directory('my_bot_bringup')

    # Ban sao cua mapper_params_online_async.yaml, chinh 4 tham so cho hop voi
    # YDLIDAR X2 va cach firmware dong goi /scan. Xem chu thich trong file.
    # Quan trong nhat: occupancy_threshold 0.1 -> 0.3, vi 0.1 khien mot tia rac
    # tren 9 lan di qua sach cung du to den mot o.
    default_params = os.path.join(bringup_dir, 'config', 'slam_async.yaml')

    # Truoc day use_sim_time bi dong cung = 'true' nen khong ve duoc ban do
    # tren xe that: slam_toolbox se doi /clock mai mai va khong xu ly scan nao.
    #   Mo phong: ros2 launch my_bot_bringup slam.launch.py
    #   Xe that : ros2 launch my_bot_bringup slam.launch.py use_sim_time:=false
    use_sim_time = LaunchConfiguration('use_sim_time')

    start_async_slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='true khi chay Gazebo, false khi ve ban do tren xe that'),
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_params,
            description='Doi sang ban goc cua slam_toolbox neu muon so sanh'),
        start_async_slam_cmd,
    ])

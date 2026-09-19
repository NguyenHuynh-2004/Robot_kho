import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Thư mục của thư viện SLAM Toolbox
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Chạy file online_async_launch.py của slam_toolbox
    start_async_slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        start_async_slam_cmd
    ])

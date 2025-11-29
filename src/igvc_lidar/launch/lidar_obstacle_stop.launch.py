from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():
    # 1) Include the Unitree LiDAR driver launch
    unitree_share = get_package_share_directory('unitree_lidar_ros2')
    unitree_launch = os.path.join(unitree_share, 'launch', 'launch.py')

    unitree_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(unitree_launch)
    )

    # 2) Your LiDAR→Arduino stop node
    #    (this is the same as: ros2 run igvc_lidar lidar_serial_stop_node ...)
    lidar_stop_node = Node(
        package='igvc_lidar',
        executable='lidar_serial_stop_node',
        name='lidar_serial_stop_node',
        parameters=[{
            'serial_port': '/dev/ttyACM0',  # change if your Mega is on ACM1 etc.
            'start_distance': 1.6,
            'stop_distance': 1.3,
        }],
        output='screen',
    )

    return LaunchDescription([
        unitree_ld,
        lidar_stop_node,
    ])

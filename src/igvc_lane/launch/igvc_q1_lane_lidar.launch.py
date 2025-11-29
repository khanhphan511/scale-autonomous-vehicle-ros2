from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # 1) OAK-D camera (depthai_ros_driver camera.launch.py)
    depthai_share = get_package_share_directory('depthai_ros_driver')
    depthai_launch = os.path.join(depthai_share, 'launch', 'camera.launch.py')

    oak_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depthai_launch)
    )

    # 2) Unitree LiDAR driver (unitree_lidar_ros2/launch/launch.py)
    unitree_share = get_package_share_directory('unitree_lidar_ros2')
    unitree_launch = os.path.join(unitree_share, 'launch', 'launch.py')

    unitree_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(unitree_launch)
    )

    # 3) Lane detector node
    lane_detector = Node(
        package='igvc_lane',
        executable='lane_detector',
        name='lane_detector_node',
        parameters=[{
            'image_topic': '/oak/rgb/image_rect',
            # your tuned values:
            'lower_v': 100.0,
            'min_hist_value': 100.0,
            'roi_bottom_fraction': 0.5,
        }],
        output='screen',
    )

    # 4) LiDAR stop logic (publishes /lidar/stop)
    lidar_stop_logic = Node(
        package='igvc_lidar',
        executable='lidar_stop_logic_node',
        name='lidar_stop_logic_node',
        parameters=[{
            'start_distance': 1.6,
            'stop_distance': 1.4,
        }],
        output='screen',
    )

    # 5) Combined lane + LiDAR → Arduino serial
    lane_serial_steer = Node(
        package='igvc_lane',
        executable='lane_serial_steer',
        name='lane_serial_steer_node',
        parameters=[{
            'serial_port': '/dev/ttyACM0',  # change if Mega shows up as ACM1
            'deadband': 0.20,
            'nudge_period': 0.40,
        }],
        output='screen',
    )

    return LaunchDescription([
        oak_camera,
        unitree_lidar,
        lane_detector,
        lidar_stop_logic,
        lane_serial_steer,
    ])

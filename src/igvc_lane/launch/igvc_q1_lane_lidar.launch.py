#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- 1) OAK-D camera launch (same as: ros2 launch depthai_ros_driver camera.launch.py)
    depthai_launch = os.path.join(
        get_package_share_directory('depthai_ros_driver'),
        'launch',
        'camera.launch.py'
    )

    oak_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depthai_launch)
    )

    # --- 2) Unitree LiDAR driver node
    # This is equivalent to what the Unitree launch file does for our purposes.
    unilidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',   # this is the node that publishes /unilidar/cloud
        name='unitree_lidar_ros2_node',
        output='screen'
    )

    # --- 3) Lane detector (camera → /lane/center_offset)
    lane_detector = Node(
        package='igvc_lane',
        executable='lane_detector',
        name='lane_detector_node',
        output='screen',
        parameters=[
            {'image_topic': '/oak/rgb/image_rect'},
            # your usual tuning defaults; you can still override with ros2 param set
            {'roi_bottom_fraction': 0.3},
            {'min_hist_value': 100.0},
            {'lower_v': 100.0},
        ],
    )

    # --- 4) LiDAR stop logic ( /unilidar/cloud → /lidar/stop )
    lidar_stop_logic = Node(
        package='igvc_lidar',
        executable='lidar_stop_logic_node',
        name='lidar_stop_logic_node',
        output='screen',
        parameters=[
            {'start_distance': 1.6},
            {'stop_distance': 1.2},
        ],
    )

    # --- 5) Lane + LiDAR → Arduino serial controller
    lane_serial_steer = Node(
        package='igvc_lane',
        executable='lane_serial_steer',
        name='lane_serial_steer_node',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyACM0'},
            {'deadband': 0.20},
            {'nudge_period': 0.40},
        ],
    )

    return LaunchDescription([
        oak_camera,
        unilidar_node,
        lane_detector,
        lidar_stop_logic,
        lane_serial_steer,
    ])

Scale Autonomous Vehicle (ROS 2 IGVC Workspace)
================================================

This workspace contains ROS 2 packages for a scale autonomous vehicle:

- igvc_lidar: LiDAR-based obstacle detection and stop logic.
- igvc_lane: Lane detection and steering / speed control using an OAK-D depth camera.
- unilidar_sdk: Unitree LiDAR driver (cloned from vendor; not versioned here if ignored).

The project uses the DepthAI SDK for the OAK-D depth camera
(see: https://github.com/luxonis/depthai). Only my ROS 2 nodes are included here.

------------------------------------------------------------
How to build
------------------------------------------------------------

1. Open a terminal.
2. Run these commands:

  - cd ~/ros2_igvc_ws
  - colcon build
  - source install/setup.bash

------------------------------------------------------------
Launch examples
------------------------------------------------------------

1) LiDAR-only obstacle stop

   ros2 launch igvc_lidar lidar_obstacle_stop.launch.py

2) Lane-keeping + obstacle stop (LiDAR + depth camera)

   ros2 launch igvc_lane igvc_q1_lane_lidar.launch.py

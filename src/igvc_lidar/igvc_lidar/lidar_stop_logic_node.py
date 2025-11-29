#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool


class LidarStopLogicNode(Node):
    """
    Read /unilidar/cloud, compute min distance in a front sector,
    and publish a Bool on /lidar/stop:

        True  -> should stop now
        False -> path clear

    Uses hysteresis:
      - stop if min_dist < stop_distance
      - only clear again if min_dist > start_distance
    """

    def __init__(self):
        super().__init__('lidar_stop_logic_node')

        # ---- Parameters ----
        self.declare_parameter('front_angle_deg', 60.0)   # +/-30 deg
        self.declare_parameter('min_height', -0.2)        # floor filter
        self.declare_parameter('max_height', 1.5)
        self.declare_parameter('detection_range', 10.0)

        self.declare_parameter('start_distance', 1.6)     # clear if farther than this
        self.declare_parameter('stop_distance', 1.3)      # stop if closer than this

        self.front_angle = math.radians(
            float(self.get_parameter('front_angle_deg').value)
        )
        self.half_angle = self.front_angle / 2.0

        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.detection_range = float(self.get_parameter('detection_range').value)

        self.start_distance = float(self.get_parameter('start_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)

        # Hysteresis state
        self.stop_active = False

        # Publisher: /lidar/stop  (no leading slash in code, but it will appear as /lidar/stop)
        self.pub_stop = self.create_publisher(Bool, 'lidar/stop', 10)

        # Subscribe to Unitree LiDAR cloud
        self.sub_cloud = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.cloud_callback,
            10
        )

        self.get_logger().info(
            f"LidarStopLogicNode running. start_distance={self.start_distance:.2f} m, "
            f"stop_distance={self.stop_distance:.2f} m"
        )

    def cloud_callback(self, msg: PointCloud2):
        # Find min distance in front sector
        min_dist = float('inf')

        for x, y, z, intensity, ring, t in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z', 'intensity', 'ring', 'time'),
            skip_nans=True
        ):
            # Height filter
            if not (self.min_height <= z <= self.max_height):
                continue

            r = math.sqrt(x * x + y * y)
            if r == 0.0 or r > self.detection_range:
                continue

            angle = math.atan2(y, x)  # x forward, y left/right
            if abs(angle) > self.half_angle:
                continue

            if r < min_dist:
                min_dist = r

        if min_dist == float('inf'):
            min_dist = float('nan')

        # Hysteresis logic
        old_stop = self.stop_active
        new_stop = old_stop

        if math.isnan(min_dist):
            # No points in front -> treat as clear
            new_stop = False
        else:
            if not old_stop:
                # Currently clear: start stopping if we get closer than stop_distance
                if min_dist < self.stop_distance:
                    new_stop = True
            else:
                # Currently stopped: only clear if we are farther than start_distance
                if min_dist > self.start_distance:
                    new_stop = False

        self.stop_active = new_stop

        # Publish Bool
        msg_out = Bool()
        msg_out.data = self.stop_active
        self.pub_stop.publish(msg_out)

        self.get_logger().info(
            f"min_dist_front={min_dist:.2f} m, stop={self.stop_active}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarStopLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

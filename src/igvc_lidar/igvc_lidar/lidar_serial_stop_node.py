#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import serial


class LidarSerialStopNode(Node):
    """
    Simple garage-test node:

    - Subscribes: /unilidar/cloud (Unitree LiDAR point cloud)
    - Computes min distance in a front sector
    - Sends 'F' over USB serial to Arduino when path is clear
    - Sends 'S' when obstacle is closer than stop_distance

    Arduino sketch interprets:
      'F' -> forward (center steering)
      'S' -> stop
    """

    def __init__(self):
        super().__init__('lidar_serial_stop_node')

        # ---- Parameters ----
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.declare_parameter('front_angle_deg', 60.0)  # +/-30 deg in front
        self.declare_parameter('min_height', -0.2)       # ignore floor
        self.declare_parameter('max_height', 1.5)        # ignore ceiling
        self.declare_parameter('detection_range', 10.0)  # max range [m]

        # Hysteresis: start if clear > start_distance, stop if < stop_distance
        # 3 ft ~ 0.91 m, so stop at ~1.0 m with margin
        self.declare_parameter('start_distance', 1.6)
        self.declare_parameter('stop_distance', 1.3)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baudrate').value)

        self.front_angle = math.radians(self.get_parameter('front_angle_deg').value)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.detection_range = float(self.get_parameter('detection_range').value)
        self.start_distance = float(self.get_parameter('start_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)

        self.half_angle = self.front_angle / 2.0

        # Serial to Arduino
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.5)
            time.sleep(2.0)  # wait for Arduino reset
            self.get_logger().info(f"Opened serial port {port} at {baud} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise

        self.driving = False
        self.last_command = None

        # Subscribe to LiDAR cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.cloud_callback,
            10
        )

    def send_cmd(self, ch: str):
        """Send one character + newline to Arduino."""
        try:
            msg = (ch + '\n').encode('ascii')
            self.ser.write(msg)
            self.ser.flush()
            self.get_logger().info(f"Sent command: {ch}")
        except Exception as e:
            self.get_logger().error(f"Error sending command '{ch}': {e}")

    def cloud_callback(self, msg: PointCloud2):
        # Find min distance in front sector
        min_dist = float('inf')

        for x, y, z, intensity, ring, t in point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z', 'intensity', 'ring', 'time'),
                skip_nans=True):

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

        self.get_logger().debug(f"min_dist_front = {min_dist}")

        # Simple state machine with hysteresis
        if not self.driving:
            # currently stopped
            if not math.isnan(min_dist) and min_dist > self.start_distance:
                self.send_cmd('F')
                self.driving = True
                self.last_command = 'F'
        else:
            # currently driving
            if math.isnan(min_dist) or min_dist < self.stop_distance:
                self.send_cmd('S')
                self.driving = False
                self.last_command = 'S'

    def destroy_node(self):
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarSerialStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



#!/usr/bin/env python3
import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


class LaneSerialSteerNode(Node):
    """
    Combined lane-follow + LiDAR stop serial controller.

    - Subscribes:
        /lane/center_offset  (Float32, normalized -1..+1)
        /lidar/stop          (Bool)

    - Talks to Arduino over USB serial (/dev/ttyACM0):
        'F' : forward (drive)
        'L' : nudge steering LEFT
        'R' : nudge steering RIGHT
        'S' : STOP (hard brake on Arduino side)

    Behaviour:
      * When LiDAR says stop=True -> send 'S' once and mark driving=False.
      * When LiDAR says stop=False AND we are not driving yet -> send 'F' once.
      * While driving, use lane offset to send occasional L/R nudges.

    Sign convention (fixed):
      offset_norm > 0  -> lane center to RIGHT -> car is LEFT -> steer RIGHT ('R')
      offset_norm < 0  -> lane center to LEFT  -> car is RIGHT -> steer LEFT ('L')
    """

    def __init__(self):
        super().__init__('lane_serial_steer_node')

        # ---- Parameters ----
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('deadband', 0.20)      # don't correct if |offset| < 0.20
        self.declare_parameter('nudge_period', 0.40)  # seconds between L/R commands

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baudrate').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.nudge_period = float(self.get_parameter('nudge_period').value)

        # ---- Serial ----
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.5)
            time.sleep(2.0)  # allow Arduino to reset
            self.get_logger().info(f"Opened serial port {port} at {baud} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise

        # ---- State ----
        self.offset_norm = 0.0
        self.have_offset = False

        self.lidar_stop = False          # latest /lidar/stop flag
        self.driving = False             # do we think car is currently driving?

        self.last_nudge_time = 0.0

        # ---- Subscriptions ----
        self.sub_lane = self.create_subscription(
            Float32,
            '/lane/center_offset',
            self.lane_callback,
            10
        )

        self.sub_lidar = self.create_subscription(
            Bool,
            '/lidar/stop',
            self.lidar_callback,
            10
        )

        # ---- Timer ----
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    # ---------- helpers ----------

    def send_cmd(self, ch: str):
        try:
            msg = (ch + '\n').encode('ascii')
            self.ser.write(msg)
            self.ser.flush()
            self.get_logger().info(f"Sent command: {ch}")
        except Exception as e:
            self.get_logger().error(f"Error sending command '{ch}': {e}")

    # ---------- callbacks ----------

    def lane_callback(self, msg: Float32):
        self.offset_norm = float(msg.data)
        self.have_offset = True

    def lidar_callback(self, msg: Bool):
        self.lidar_stop = bool(msg.data)

    def timer_callback(self):
        # --- 1) LiDAR stop has highest priority ---
        if self.lidar_stop:
            if self.driving:
                self.get_logger().info("LiDAR stop=True -> sending 'S'")
                self.send_cmd('S')
                self.driving = False
            # When stopped, don't send steering nudges
            return

        # --- 2) Path clear: if we are not yet driving, send 'F' once ---
        if not self.driving:
            self.get_logger().info("LiDAR stop=False and not driving -> sending 'F'")
            self.send_cmd('F')
            self.driving = True
            # don't steer on this same tick; wait for next cycle
            return

        # --- 3) Lane-based steering (only if we have offset) ---
        if not self.have_offset:
            return

        off = self.offset_norm
        db = self.deadband

        # If we are close enough to center, do nothing
        if abs(off) < db:
            return

        now = time.time()
        if now - self.last_nudge_time < self.nudge_period:
            return  # wait before next nudge

        # Sign convention:
        #  offset_norm > 0 -> lane center to RIGHT -> car LEFT -> steer RIGHT ('R')
        #  offset_norm < 0 -> lane center to LEFT  -> car RIGHT -> steer LEFT ('L')
        if off > db:
            cmd = 'R'
        elif off < -db:
            cmd = 'L'
        else:
            return

        self.send_cmd(cmd)
        self.last_nudge_time = now

    def destroy_node(self):
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneSerialSteerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

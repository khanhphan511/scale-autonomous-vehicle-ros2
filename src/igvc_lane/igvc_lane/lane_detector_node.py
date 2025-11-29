#!/usr/bin/env python3
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class LaneDetectorNode(Node):
    """
    Simple lane detector for IGVC-style straight lane.

    Subscribes:
      - image_topic (sensor_msgs/Image), default: /oak/rgb/image_rect

    Publishes:
      - /lane/center_offset : std_msgs/Float32
          normalized offset: 0 = centered
          >0 : lane center is to the RIGHT of image center
          <0 : lane center is to the LEFT of image center
      - /lane/debug_image   : sensor_msgs/Image
      - /lane/debug_mask    : sensor_msgs/Image (binary mask overlay)
    """

    def __init__(self):
        super().__init__("lane_detector_node")
        self.bridge = CvBridge()

        # ---- Parameters (can be tuned at run-time) ----
        self.declare_parameter("image_topic", "/oak/rgb/image_rect")
        # fraction of image height used at the bottom (0.35 = bottom 35%)
        self.declare_parameter("roi_bottom_fraction", 0.35)
        # minimum histogram peak to trust a lane
        self.declare_parameter("min_hist_value", 3000.0)
        # HSV thresholds for "white" tape
        self.declare_parameter("lower_v", 210.0)   # brightness lower bound
        self.declare_parameter("upper_v", 255.0)   # brightness upper bound
        self.declare_parameter("max_s", 80.0)      # maximum saturation

        self.image_topic = self.get_parameter("image_topic").value
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")

        self.sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )

        self.offset_pub = self.create_publisher(Float32, "/lane/center_offset", 10)
        self.debug_pub = self.create_publisher(Image, "/lane/debug_image", 10)
        self.mask_pub = self.create_publisher(Image, "/lane/debug_mask", 10)

        self.last_print_time = time.time()

    # --------------------------------------------------
    # main callback
    # --------------------------------------------------
    def image_callback(self, msg: Image):
        # ROS -> OpenCV
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        h, w, _ = cv_img.shape
        mid_x = w // 2

        # Read tunable parameters each frame so you can change them with ros2 param set
        roi_bottom_fraction = float(self.get_parameter("roi_bottom_fraction").value)
        min_hist_value = float(self.get_parameter("min_hist_value").value)
        lower_v = int(self.get_parameter("lower_v").value)
        upper_v = int(self.get_parameter("upper_v").value)
        max_s = int(self.get_parameter("max_s").value)

        # ---- ROI: bottom part of the image ----
        roi_y = int(h * (1.0 - roi_bottom_fraction))
        roi_y = max(0, min(roi_y, h - 1))   # clamp
        roi = cv_img[roi_y:h, :]

        # ---- HSV threshold for white tape ----
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, lower_v], dtype=np.uint8)
        upper_white = np.array([180, max_s, upper_v], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # small blur to kill speckle
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # ---- 1D histogram along X ----
        hist = np.sum(mask, axis=0).astype(np.float32)

        left_hist = hist[:mid_x]
        right_hist = hist[mid_x:]

        left_peak_val = float(np.max(left_hist)) if left_hist.size > 0 else 0.0
        right_peak_val = float(np.max(right_hist)) if right_hist.size > 0 else 0.0

        left_peak_idx = int(np.argmax(left_hist)) if left_hist.size > 0 else 0
        right_peak_idx = (
            int(np.argmax(right_hist)) + mid_x if right_hist.size > 0 else w - 1
        )

        lanes_found = (
            left_peak_val > min_hist_value and right_peak_val > min_hist_value
        )

        if lanes_found:
            lane_center_x = 0.5 * (left_peak_idx + right_peak_idx)
            offset_pixels = lane_center_x - mid_x
            offset_norm = offset_pixels / float(mid_x)
        else:
            lane_center_x = float(mid_x)
            offset_norm = 0.0

        # ---- Publish offset ----
        offset_msg = Float32()
        offset_msg.data = float(offset_norm)
        self.offset_pub.publish(offset_msg)

        # ---- Periodic console print ----
        now = time.time()
        if now - self.last_print_time > 1.0:
            self.get_logger().info(
                f"offset_norm={offset_norm:+.3f}, "
                f"left_peak={left_peak_val:.0f}, right_peak={right_peak_val:.0f}, "
                f"min_hist={min_hist_value:.0f}, lower_v={lower_v}"
            )
            self.last_print_time = now

        # ---- Build debug image ----
        debug = roi.copy()
        # ROI rectangle
        cv2.rectangle(
            debug, (0, 0), (w - 1, debug.shape[0] - 1), (255, 255, 255), 1
        )

        # image vertical center (blue)
        cv2.line(
            debug, (mid_x, 0), (mid_x, debug.shape[0] - 1), (255, 0, 0), 2
        )
        # detected lane peaks (magenta)
        cv2.line(
            debug,
            (left_peak_idx, 0),
            (left_peak_idx, debug.shape[0] - 1),
            (255, 0, 255),
            2,
        )
        cv2.line(
            debug,
            (right_peak_idx, 0),
            (right_peak_idx, debug.shape[0] - 1),
            (255, 0, 255),
            2,
        )
        # lane center (green)
        cv2.line(
            debug,
            (int(lane_center_x), 0),
            (int(lane_center_x), debug.shape[0] - 1),
            (0, 255, 0),
            2,
        )

        text = f"offset_norm={offset_norm:+.3f}"
        cv2.putText(
            debug,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        full_debug = cv_img.copy()
        full_debug[roi_y:h, :] = debug
        debug_msg = self.bridge.cv2_to_imgmsg(full_debug, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

        # ---- Publish mask debug too ----
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        full_mask = np.zeros_like(cv_img)
        full_mask[roi_y:h, :] = mask_color
        mask_msg = self.bridge.cv2_to_imgmsg(full_mask, encoding="bgr8")
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

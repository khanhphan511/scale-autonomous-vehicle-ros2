#!/usr/bin/env python3

import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


# ---------- Helper functions (non-ROS) ----------

def region_selection(image: np.ndarray) -> np.ndarray:
    """
    Polygon region of interest similar to the reference project.
    Works for either 1‑channel or 3‑channel images.
    """
    mask = np.zeros_like(image)

    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    rows, cols = image.shape[:2]
    bottom_left  = (int(cols * 0.10), int(rows * 0.95))
    top_left     = (int(cols * 0.40), int(rows * 0.60))
    top_right    = (int(cols * 0.60), int(rows * 0.60))
    bottom_right = (int(cols * 0.90), int(rows * 0.95))

    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]],
                        dtype=np.int32)
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def average_slope_intercept(lines):
    """
    From Hough line segments, compute weighted‑average left and right lane lines
    in (slope, intercept) form.

    left: negative slope
    right: positive slope  (image coordinates: x right, y down)
    """
    left_lines, left_weights = [], []
    right_lines, right_weights = [], []

    if lines is None:
        return None, None

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 == x2:
                # skip perfectly vertical (undefined slope)
                continue

            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            length = np.hypot(x2 - x1, y2 - y1)

            if slope < 0:
                left_lines.append((slope, intercept))
                left_weights.append(length)
            else:
                right_lines.append((slope, intercept))
                right_weights.append(length)

    left_lane = (np.dot(left_weights, left_lines) / np.sum(left_weights)
                 if len(left_weights) > 0 else None)
    right_lane = (np.dot(right_weights, right_lines) / np.sum(right_weights)
                  if len(right_weights) > 0 else None)

    return left_lane, right_lane


def pixel_points(y1, y2, line):
    """
    Convert (slope, intercept) into pixel endpoints at y1 and y2.
    Coordinates are in the same frame as the input points (ROI frame).
    """
    if line is None:
        return None

    slope, intercept = line
    if abs(slope) < 1e-6:
        return None

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    y1 = int(y1)
    y2 = int(y2)
    return (x1, y1), (x2, y2)


# ---------- ROS node ----------

class LaneDetectorNode(Node):
    """
    Lane detector for IGVC‑style yellow tape, using HSV + Canny + Hough.

    Subscribes:
      - image_topic (sensor_msgs/Image), default: /oak/rgb/image_rect

    Publishes:
      - /lane/center_offset : std_msgs/Float32
            normalised offset: 0 = centred
            >0 : vehicle centre is to the RIGHT of lane centre
            <0 : vehicle centre is to the LEFT of lane centre
      - /lane/debug_image   : sensor_msgs/Image
      - /lane/debug_mask    : sensor_msgs/Image (binary mask overlay)

    The sign of the published offset follows the original reference
    implementation: the node publishes a positive offset when the lane
    centre lies to the right of the image centre (i.e. the vehicle is
    offset to the left and should steer right) and a negative offset
    when the lane centre lies to the left of the image centre (i.e. the
    vehicle is offset to the right and should steer left).
    """

    def __init__(self):
        super().__init__("lane_detector_node")
        self.bridge = CvBridge()

        # ---- Parameters (tunable at run‑time) ----
        self.declare_parameter("image_topic", "/oak/rgb/image_rect")
        # bottom fraction of image used for ROI crop
        self.declare_parameter("roi_bottom_fraction", 0.40)

        # HSV thresholds for YELLOW tape
        self.declare_parameter("lower_h", 18.0)
        self.declare_parameter("upper_h", 40.0)
        self.declare_parameter("lower_s", 80.0)
        self.declare_parameter("upper_s", 255.0)
        self.declare_parameter("lower_v", 120.0)
        self.declare_parameter("upper_v", 255.0)

        # Canny edge thresholds
        self.declare_parameter("canny_low", 50.0)
        self.declare_parameter("canny_high", 150.0)

        # Hough transform parameters
        self.declare_parameter("hough_threshold", 20)      # # of votes
        self.declare_parameter("hough_min_line_length", 20.0)
        self.declare_parameter("hough_max_line_gap", 300.0)

        # Lane‑width logic
        self.declare_parameter("min_hist_value", 0.0)  # unused here, but kept for compatibility
        self.declare_parameter("default_lane_half_width_px", 120.0)
        self.declare_parameter("lane_width_scale_single", 1.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")

        self.sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )

        self.offset_pub = self.create_publisher(Float32, "/lane/center_offset", 10)
        self.debug_pub = self.create_publisher(Image, "/lane/debug_image", 10)
        self.mask_pub = self.create_publisher(Image, "/lane/debug_mask", 10)

        # learned lane half‑width (pixels), updated whenever BOTH lanes visible
        self.lane_half_width_px = None

        self.last_print_time = time.time()

    # --------------------------------------------------
    # main callback
    # --------------------------------------------------
    def image_callback(self, msg: Image):
        # ROS -> OpenCV (BGR)
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        h, w, _ = cv_img.shape
        mid_x = w // 2

        # ---- Read parameters ----
        roi_bottom_fraction = float(self.get_parameter("roi_bottom_fraction").value)

        lower_h = int(self.get_parameter("lower_h").value)
        upper_h = int(self.get_parameter("upper_h").value)
        lower_s = int(self.get_parameter("lower_s").value)
        upper_s = int(self.get_parameter("upper_s").value)
        lower_v = int(self.get_parameter("lower_v").value)
        upper_v = int(self.get_parameter("upper_v").value)

        canny_low = float(self.get_parameter("canny_low").value)
        canny_high = float(self.get_parameter("canny_high").value)

        hough_threshold = int(self.get_parameter("hough_threshold").value)
        hough_min_line_length = float(
            self.get_parameter("hough_min_line_length").value
        )
        hough_max_line_gap = float(
            self.get_parameter("hough_max_line_gap").value
        )

        default_half = float(self.get_parameter("default_lane_half_width_px").value)
        lane_width_scale_single = float(
            self.get_parameter("lane_width_scale_single").value
        )

        # Clamp hue to valid range
        lower_h = max(0, min(lower_h, 179))
        upper_h = max(0, min(upper_h, 179))

        # ---- ROI: bottom crop ----
        roi_y = int(h * (1.0 - roi_bottom_fraction))
        roi_y = max(0, min(roi_y, h - 1))
        roi_bgr = cv_img[roi_y:h, :]
        roi_h, roi_w = roi_bgr.shape[:2]

        # ---- Color threshold for yellow lane (HSV) ----
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([lower_h, lower_s, lower_v], dtype=np.uint8)
        upper_yellow = np.array([upper_h, upper_s, upper_v], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Slight blur to clean speckle
        mask_blur = cv2.GaussianBlur(mask, (5, 5), 0)

        # ---- Edge detection ----
        edges = cv2.Canny(mask_blur, canny_low, canny_high)

        # ---- Region‑of‑interest polygon on the ROI ----
        edges_roi = region_selection(edges)

        # ---- Hough transform ----
        rho = 1.0
        theta = np.pi / 180.0
        lines = cv2.HoughLinesP(
            edges_roi,
            rho=rho,
            theta=theta,
            threshold=hough_threshold,
            minLineLength=hough_min_line_length,
            maxLineGap=hough_max_line_gap,
        )

        left_lane, right_lane = average_slope_intercept(lines)

        # Convert to pixel segments in ROI coordinates
        y1_roi = roi_h - 1
        y2_roi = int(roi_h * 0.6)

        left_line = pixel_points(y1_roi, y2_roi, left_lane) if left_lane is not None else None
        right_line = pixel_points(y1_roi, y2_roi, right_lane) if right_lane is not None else None

        # ---- Decide which lanes we really trust ----
        left_valid = left_line is not None
        right_valid = right_line is not None

        lane_center_x = float(mid_x)
        mode = "NONE"

        # Measure lane width at the bottom of ROI, if both present
        two_lanes_confident = False
        lane_width_px = None

        if left_valid and right_valid:
            x_left_bottom = left_line[0][0]
            x_right_bottom = right_line[0][0]
            lane_width_px = float(x_right_bottom - x_left_bottom)

            # require some separation and consistency with learned width
            min_sep_px = 0.05 * w  # at least 5% of image width

            if lane_width_px > min_sep_px:
                if self.lane_half_width_px is None:
                    two_lanes_confident = True
                else:
                    expected_width = 2.0 * self.lane_half_width_px
                    low = 0.5 * expected_width
                    high = 1.5 * expected_width
                    if low <= lane_width_px <= high:
                        two_lanes_confident = True

        # ---- Case 1: BOTH lanes visible and consistent ----
        if two_lanes_confident and left_valid and right_valid:
            mode = "BOTH"
            x_left_bottom = left_line[0][0]
            x_right_bottom = right_line[0][0]
            lane_width_px = float(x_right_bottom - x_left_bottom)
            half_width_px = lane_width_px * 0.5

            # Learn/smooth lane half‑width
            if self.lane_half_width_px is None:
                self.lane_half_width_px = half_width_px
            else:
                self.lane_half_width_px = (
                    0.9 * self.lane_half_width_px + 0.1 * half_width_px
                )

            lane_center_x = 0.5 * (x_left_bottom + x_right_bottom)

        # ---- Case 2: only LEFT lane visible ----
        elif left_valid and not right_valid:
            mode = "LEFT_ONLY"
            x_left_bottom = left_line[0][0]
            half = (
                self.lane_half_width_px
                if self.lane_half_width_px is not None
                else default_half
            )
            half *= lane_width_scale_single
            lane_center_x = x_left_bottom + half

        # ---- Case 3: only RIGHT lane visible ----
        elif right_valid and not left_valid:
            mode = "RIGHT_ONLY"
            x_right_bottom = right_line[0][0]
            half = (
                self.lane_half_width_px
                if self.lane_half_width_px is not None
                else default_half
            )
            half *= lane_width_scale_single
            lane_center_x = x_right_bottom - half

        # ---- Case 4: no lane visible ----
        else:
            mode = "NONE"
            lane_center_x = float(mid_x)

        # Clamp centre in full‑image coordinates
        lane_center_x = max(0.0, min(float(w - 1), lane_center_x + 0.0))

        # Compute normalised offset.  A positive value indicates the lane
        # centre lies to the right of the image centre (the vehicle is
        # offset to the left and should steer right), while a negative
        # value indicates the lane centre lies to the left (the vehicle is
        # offset to the right and should steer left).
        offset_pixels = lane_center_x - mid_x
        offset_norm = offset_pixels / float(mid_x)

        # ---- Publish offset ----
        offset_msg = Float32()
        offset_msg.data = float(offset_norm)
        self.offset_pub.publish(offset_msg)

        # ---- Logging ----
        now = time.time()
        if now - self.last_print_time > 1.0:
            self.get_logger().info(
                "mode={}, offset_norm={:+.3f}, "
                "lane_half_width_px={}, lane_width_px={}, "
                "H=[{}, {}], S=[{}, {}], V=[{}, {}]".format(
                    mode,
                    offset_norm,
                    (None if self.lane_half_width_px is None else f"{self.lane_half_width_px:.1f}"),
                    (None if lane_width_px is None else f"{lane_width_px:.1f}"),
                    lower_h,
                    upper_h,
                    lower_s,
                    upper_s,
                    lower_v,
                    upper_v,
                )
            )
            self.last_print_time = now

        # ---- Build debug image (full frame) ----
        debug = cv_img.copy()

        # Draw ROI polygon (shifted by roi_y)
        rows, cols = roi_h, roi_w
        bottom_left  = (int(cols * 0.10), roi_y + int(rows * 0.95))
        top_left     = (int(cols * 0.40), roi_y + int(rows * 0.60))
        top_right    = (int(cols * 0.60), roi_y + int(rows * 0.60))
        bottom_right = (int(cols * 0.90), roi_y + int(rows * 0.95))
        roi_pts = np.array([[bottom_left, top_left, top_right, bottom_right]],
                           dtype=np.int32)
        cv2.polylines(debug, roi_pts, isClosed=True, color=(255, 255, 255), thickness=1)

        # image vertical centre (blue)
        cv2.line(debug, (mid_x, roi_y), (mid_x, h - 1), (255, 0, 0), 2)

        # Draw lane lines (magenta)
        if left_valid:
            (x1, y1), (x2, y2) = left_line
            cv2.line(
                debug,
                (x1, y1 + roi_y),
                (x2, y2 + roi_y),
                (255, 0, 255),
                3,
            )
        if right_valid:
            (x1, y1), (x2, y2) = right_line
            cv2.line(
                debug,
                (x1, y1 + roi_y),
                (x2, y2 + roi_y),
                (255, 0, 255),
                3,
            )

        # lane centre (green) – draw inside ROI
        x_center_int = int(lane_center_x)
        cv2.line(
            debug,
            (x_center_int, roi_y),
            (x_center_int, h - 1),
            (0, 255, 0),
            2,
        )

        text = f"{mode}, offset_norm={offset_norm:+.3f}"
        cv2.putText(
            debug,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

        # ---- Publish mask debug (only yellow mask in ROI) ----
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

#!/usr/bin/env python3
"""
vision_perception/line_mapper.py
=================================
ROS 2 Humble lane-detection node for the F1Tenth / Go-Kart using ZED 2i.

SUBSCRIBES
----------
  /zed/zed_node/rgb/image_rect_color         sensor_msgs/Image
  /zed/zed_node/point_cloud/cloud_registered  sensor_msgs/PointCloud2

PUBLISHES
---------
  /perception/lane_guidance    geometry_msgs/Vector3
    .x  lateral_error_m   -- signed metres; + means lane centre is LEFT
    .y  heading_error_rad -- signed radians; + means lane curves left
    .z  detection_quality -- 0.0 (no lanes), 0.5 (one lane), 1.0 (both)

  /perception/debug/lane_mask  sensor_msgs/Image  (mono8, visualise only)
"""

import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Vector3
import message_filters
from cv_bridge import CvBridge


# ============================================================================
#  PURE PROCESSING FUNCTIONS  (no ROS imports, unit-testable)
# ============================================================================

def lane_mask_blue(bgr_roi: np.ndarray) -> np.ndarray:
    """
    HSV colour mask tuned for blue tape.
    Tweak lower/upper if your tape colour or lighting changes.
    """
    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
    lower = np.array([99, 63, 133])
    upper = np.array([155, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)

    # Morphological clean-up (remove noise, fill gaps)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=1)
    return mask


def mask_to_edges(mask: np.ndarray) -> np.ndarray:
    """Morphological gradient -> thin edge ring around the mask blob."""
    k = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, k)


def robust_xyz(pc_xyz: np.ndarray, x: int, y: int, r: int = 2):
    """
    Return the nearest valid (finite) XYZ in a small neighbourhood
    around image pixel (x, y).
    """
    h, w = pc_xyz.shape[:2]
    best, best_d2 = None, 1e9

    for dy in range(-r, r + 1):
        yy = y + dy
        if not (0 <= yy < h):
            continue
        for dx in range(-r, r + 1):
            xx = x + dx
            if not (0 <= xx < w):
                continue
            p = pc_xyz[yy, xx, :3]
            if np.isfinite(p).all():
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best, best_d2 = p.copy(), d2
    return best


def split_two_lanes_by_y(xy: np.ndarray, iters: int = 10):
    """
    1-D two-means clustering in the BEV Y axis to separate left / right lanes.
    Y is lateral in the ROS convention (+ = left).
    """
    if xy.shape[0] < 2:
        return xy, xy

    y = xy[:, 1]
    c1, c2 = np.percentile(y, [25, 75])

    for _ in range(iters):
        lbl = np.abs(y - c1) < np.abs(y - c2)
        if lbl.all() or (~lbl).all():
            break
        c1, c2 = y[lbl].mean(), y[~lbl].mean()

    a, b = xy[lbl], xy[~lbl]
    if a.shape[0] == 0 or b.shape[0] == 0:
        return a, b

    # Left lane = larger Y (further left in world frame)
    return (a, b) if a[:, 1].mean() > b[:, 1].mean() else (b, a)


def fit_line_xy(xy: np.ndarray):
    """Fit a 2-D line to BEV XY points via OpenCV's robust line fit."""
    v = cv2.fitLine(xy.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01)
    return tuple(float(c) for c in v.flatten())


def compute_lane_errors(left_xy, right_xy, left_fit, right_fit,
                        min_pts: int = 30, look_ahead: float = 1.0,
                        half_width: float = 0.25):
    """
    Derive lateral and heading errors from detected lane lines.
    Coordinate frame: X forward, Y left, Z up (ROS standard).
    """
    have_left = (left_xy is not None and left_xy.shape[0] >= min_pts
                 and left_fit is not None)
    have_right = (right_xy is not None and right_xy.shape[0] >= min_pts
                  and right_fit is not None)
    quality = (float(have_left) + float(have_right)) / 2.0

    if not have_left and not have_right:
        return 0.0, 0.0, 0.0

    def y_at(fit, x_eval):
        vx, vy, x0, y0 = fit
        return y0 if abs(vx) < 1e-6 else y0 + (vy / vx) * (x_eval - x0)

    if have_left and have_right:
        c0 = (y_at(left_fit, 0.0) + y_at(right_fit, 0.0)) / 2.0
        c1 = (y_at(left_fit, look_ahead) + y_at(right_fit, look_ahead)) / 2.0
    elif have_left:
        c0 = y_at(left_fit, 0.0) - half_width
        c1 = y_at(left_fit, look_ahead) - half_width
    else:
        c0 = y_at(right_fit, 0.0) + half_width
        c1 = y_at(right_fit, look_ahead) + half_width

    lat_err = c0
    hdg_err = math.atan2(c1 - c0, look_ahead)

    return lat_err, hdg_err, quality


# ============================================================================
#  ROS 2 NODE
# ============================================================================

class LineMapperNode(Node):
    """
    Fuses ZED colour image with registered point cloud to detect blue-tape
    lane boundaries and publish driving guidance.
    """

    # -- Tunables (change here or via ROS params later) -----------------------
    MIN_PTS = 30          # minimum edge pixels for a valid lane fit
    MAX_SAMPLES = 400     # cap for 3-D lifting (speed vs. accuracy)
    STRIDE = 3            # subsample stride on edge pixels
    MAX_RANGE = 3.0       # metres -- ignore points beyond this
    Z_TOL = 0.15          # metres -- floor filter tolerance
    EDGE_MARGIN = 5       # pixels -- discard image border artefacts

    def __init__(self):
        super().__init__('line_mapper')
        self.bridge = CvBridge()

        # -- Publishers -------------------------------------------------------
        self._guidance_pub = self.create_publisher(
            Vector3, '/perception/lane_guidance', 10)
        self._debug_pub = self.create_publisher(
            Image, '/perception/debug/lane_mask', 10)

        # -- Synchronized subscribers -----------------------------------------
        image_sub = message_filters.Subscriber(
            self, Image, '/zed/zed_node/rgb/image_rect_color')
        cloud_sub = message_filters.Subscriber(
            self, PointCloud2, '/zed/zed_node/point_cloud/cloud_registered')

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, cloud_sub], queue_size=5, slop=0.05)
        self._sync.registerCallback(self._synced_callback)

        self.get_logger().info(
            'LineMapper ready. Waiting for ZED image + point cloud ...')

    # -- Helpers --------------------------------------------------------------

    def _cloud_to_xyz(self, cloud_msg: PointCloud2) -> np.ndarray:
        """Reshape the registered point cloud into (H, W, 3) float32."""
        raw = np.frombuffer(cloud_msg.data, dtype=np.uint8).view(np.float32)
        n_fields = cloud_msg.point_step // 4
        pts = raw.reshape(cloud_msg.height * cloud_msg.width, n_fields)
        xyz = pts[:, :3].reshape(cloud_msg.height, cloud_msg.width, 3)
        return np.ascontiguousarray(xyz, dtype=np.float32)

    def _publish_zero(self):
        """Publish a zero-quality guidance message when detection fails."""
        self._guidance_pub.publish(Vector3(x=0.0, y=0.0, z=0.0))

    # -- Main synchronized callback -------------------------------------------

    def _synced_callback(self, image_msg: Image, cloud_msg: PointCloud2):
        # 1. Decode
        bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        pc_xyz = self._cloud_to_xyz(cloud_msg)
        h, w = bgr.shape[:2]

        # 2. Mask & edges on the lower half (road visible here)
        roi = bgr[h // 2:, :]
        mask = lane_mask_blue(roi)
        edges = mask_to_edges(mask)

        # 3. Collect edge pixel indices
        ys_roi, xs_roi = np.where(edges > 0)
        if len(xs_roi) == 0:
            self._publish_zero()
            return

        # Subsample
        sel = np.arange(0, len(xs_roi), self.STRIDE)
        xs_roi, ys_roi = xs_roi[sel], ys_roi[sel]

        if len(xs_roi) > self.MAX_SAMPLES:
            idx = np.linspace(0, len(xs_roi) - 1, self.MAX_SAMPLES, dtype=int)
            xs_roi, ys_roi = xs_roi[idx], ys_roi[idx]

        # Map back to full-image coordinates
        ys_full = ys_roi + h // 2
        xs_full = xs_roi

        # Discard border pixels
        keep = (
            (xs_full >= self.EDGE_MARGIN) & (xs_full < w - self.EDGE_MARGIN) &
            (ys_full >= self.EDGE_MARGIN) & (ys_full < h - self.EDGE_MARGIN)
        )
        xs_full, ys_full = xs_full[keep], ys_full[keep]

        # 4. Lift pixels into 3-D
        pts_list = []
        for x, y in zip(xs_full, ys_full):
            p = robust_xyz(pc_xyz, int(x), int(y), r=2)
            if p is not None:
                pts_list.append(p)

        if len(pts_list) < self.MIN_PTS:
            self._publish_zero()
            return

        pts = np.array(pts_list, dtype=np.float32)

        # Distance filter
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r > 0.3) & (r < self.MAX_RANGE)]

        # Floor filter
        pts = pts[np.abs(pts[:, 2]) < self.Z_TOL]

        xy = pts[:, :2]
        if xy.shape[0] < self.MIN_PTS:
            self._publish_zero()
            return

        # 5. Split and fit
        left_xy, right_xy = split_two_lanes_by_y(xy)

        left_fit = fit_line_xy(left_xy) if left_xy.shape[0] >= self.MIN_PTS else None
        right_fit = fit_line_xy(right_xy) if right_xy.shape[0] >= self.MIN_PTS else None

        # 6. Compute errors
        lat_err, hdg_err, quality = compute_lane_errors(
            left_xy, right_xy, left_fit, right_fit, min_pts=self.MIN_PTS)

        # 7. Publish guidance
        guidance = Vector3()
        guidance.x = float(lat_err)
        guidance.y = float(hdg_err)
        guidance.z = float(quality)
        self._guidance_pub.publish(guidance)

        # 8. Publish debug mask
        debug_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        debug_msg.header = image_msg.header
        self._debug_pub.publish(debug_msg)

        self.get_logger().debug(
            f'lat={lat_err:.3f}m  hdg={math.degrees(hdg_err):.1f}deg  '
            f'q={quality:.1f}  pts={xy.shape[0]}')


# ============================================================================
#  ENTRY POINT
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = LineMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

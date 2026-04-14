#!/usr/bin/env python3
"""
vision_perception/slam_visualizer.py — Diagnostic BEV visualization (v3).

Shows:
  - Raw map points (small dots, faded)
  - Fitted polyline curves (thick solid lines) for left/right lanes
  - Centerline derived from curves (dashed green)
  - Car position + heading
  - Odom trail
  - Guidance arrow
  - HUD with stats

PUBLISHES
  /perception/debug/bev_map   Image (bgr8)
  /perception/debug/slam_diag Image (bgr8)
"""

import json
import math
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from cv_bridge import CvBridge


def quat_to_yaw(ori):
    siny = 2.0 * (ori.w * ori.z + ori.x * ori.y)
    cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
    return math.atan2(siny, cosy)


# ============================================================================
#  BEV RENDERER
# ============================================================================

class BevRenderer:

    COL_BG = (12, 12, 16)
    COL_GRID = (30, 30, 35)
    COL_LEFT_RAW = (180, 80, 20)     # dim blue
    COL_RIGHT_RAW = (20, 20, 160)    # dim red
    COL_LEFT_CURVE = (255, 160, 50)  # bright blue
    COL_RIGHT_CURVE = (50, 50, 255)  # bright red
    COL_CENTER = (60, 220, 60)       # green
    COL_CAR = (255, 255, 255)
    COL_TRAIL = (50, 180, 200)
    COL_GUIDANCE = (0, 255, 255)
    COL_TEXT = (180, 180, 180)
    COL_WARN = (30, 80, 255)

    def __init__(self, img_size=500, meters_per_pixel=0.008):
        self.img_size = img_size
        self.mpp = meters_per_pixel
        self.center = img_size // 2

    def world_to_bev(self, wx, wy, car_xy, cos_yaw, sin_yaw):
        dx = wx - car_xy[0]
        dy = wy - car_xy[1]
        rx = cos_yaw * dx - sin_yaw * dy
        ry = sin_yaw * dx + cos_yaw * dy
        px = int(self.center - ry / self.mpp)
        py = int(self.center - rx / self.mpp)
        return px, py

    def _in_bounds(self, px, py):
        return 0 <= px < self.img_size and 0 <= py < self.img_size

    def render(self, car_xy, car_yaw,
               left_raw_pts, right_raw_pts,
               left_curve_pts, right_curve_pts,
               odom_trail, guidance_lat, guidance_hdg, quality,
               diagnostics=None, raw_guidance=None):

        img = np.full((self.img_size, self.img_size, 3), self.COL_BG, dtype=np.uint8)
        cos_yaw = math.cos(-car_yaw)
        sin_yaw = math.sin(-car_yaw)

        # Grid
        grid_px = int(0.5 / self.mpp)
        for i in range(-6, 7):
            pos = self.center + i * grid_px
            if 0 <= pos < self.img_size:
                cv2.line(img, (pos, 0), (pos, self.img_size), self.COL_GRID, 1)
                cv2.line(img, (0, pos), (self.img_size, pos), self.COL_GRID, 1)

        # Range rings
        for r_m in [0.5, 1.0, 1.5, 2.0]:
            r_px = int(r_m / self.mpp)
            cv2.circle(img, (self.center, self.center), r_px, (35, 35, 45), 1)
            cv2.putText(img, f'{r_m:.1f}m',
                        (self.center + r_px + 2, self.center - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.25, (60, 60, 70), 1)

        # Odom trail
        for i, (ox, oy) in enumerate(odom_trail):
            alpha = i / max(len(odom_trail), 1)
            px, py = self.world_to_bev(ox, oy, car_xy, cos_yaw, sin_yaw)
            if self._in_bounds(px, py):
                c = tuple(int(v * alpha) for v in self.COL_TRAIL)
                cv2.circle(img, (px, py), 1, c, -1)

        # Raw points (small, dim)
        for pt in left_raw_pts:
            px, py = self.world_to_bev(pt[0], pt[1], car_xy, cos_yaw, sin_yaw)
            if self._in_bounds(px, py):
                cv2.circle(img, (px, py), 1, self.COL_LEFT_RAW, -1)
        for pt in right_raw_pts:
            px, py = self.world_to_bev(pt[0], pt[1], car_xy, cos_yaw, sin_yaw)
            if self._in_bounds(px, py):
                cv2.circle(img, (px, py), 1, self.COL_RIGHT_RAW, -1)

        # Fitted curves (thick connected lines)
        self._draw_curve(img, left_curve_pts, car_xy, cos_yaw, sin_yaw,
                         self.COL_LEFT_CURVE, thickness=3)
        self._draw_curve(img, right_curve_pts, car_xy, cos_yaw, sin_yaw,
                         self.COL_RIGHT_CURVE, thickness=3)

        # Centerline from curves (dashed green)
        if left_curve_pts and right_curve_pts:
            center_pts = self._compute_centerline(
                left_curve_pts, right_curve_pts)
            self._draw_curve(img, center_pts, car_xy, cos_yaw, sin_yaw,
                             self.COL_CENTER, thickness=2, dashed=True)

        # Guidance arrow
        if abs(guidance_lat) > 0.001 or abs(guidance_hdg) > 0.001:
            arrow_len = 0.5 / self.mpp
            end_x = int(self.center - guidance_lat / self.mpp)
            end_y = int(self.center - arrow_len)
            cv2.arrowedLine(img, (self.center, self.center),
                            (end_x, end_y), self.COL_GUIDANCE, 2, tipLength=0.3)

        # Car triangle
        car_pts = np.array([
            [self.center, self.center - 14],
            [self.center - 7, self.center + 7],
            [self.center + 7, self.center + 7],
        ], dtype=np.int32)
        cv2.fillPoly(img, [car_pts], self.COL_CAR)

        # ---- HUD ----
        y = 15
        cv2.putText(img, "SLAM BEV v3", (5, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        y += 18

        # Quality bar
        q_w = 80
        q_fill = int(q_w * quality)
        q_col = (0, 200, 0) if quality >= 0.5 else self.COL_WARN
        cv2.rectangle(img, (5, y - 8), (5 + q_w, y + 4), (50, 50, 50), -1)
        cv2.rectangle(img, (5, y - 8), (5 + q_fill, y + 4), q_col, -1)
        cv2.putText(img, f'Q={quality:.1f}', (q_w + 10, y + 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, self.COL_TEXT, 1)
        y += 16

        cv2.putText(img, f'Lat: {guidance_lat:+.3f}m', (5, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, self.COL_TEXT, 1)
        y += 14
        cv2.putText(img, f'Hdg: {math.degrees(guidance_hdg):+.1f}deg', (5, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, self.COL_TEXT, 1)
        y += 14

        if diagnostics:
            cv2.putText(img,
                f'Raw: {diagnostics.get("raw_left", 0)}L/{diagnostics.get("raw_right", 0)}R',
                (5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.30, self.COL_TEXT, 1)
            y += 13
            cv2.putText(img,
                f'Curve: {diagnostics.get("curve_left", 0)}L/{diagnostics.get("curve_right", 0)}R',
                (5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (100, 255, 100), 1)
            y += 14

        if raw_guidance is not None:
            cv2.putText(img, f'Raw lat: {raw_guidance.x:+.3f}m', (5, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (100, 100, 100), 1)
            y += 12
            cv2.putText(img, f'Raw hdg: {math.degrees(raw_guidance.y):+.1f}deg', (5, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (100, 100, 100), 1)

        # Legend
        bh = self.img_size - 10
        cv2.putText(img, "raw=dim  curve=bright  center=green  arrow=steer",
                    (5, bh), cv2.FONT_HERSHEY_SIMPLEX, 0.26, (90, 90, 90), 1)

        return img

    def _draw_curve(self, img, pts_list, car_xy, cos_yaw, sin_yaw,
                    color, thickness=2, dashed=False):
        if not pts_list or len(pts_list) < 2:
            return
        bev_pts = []
        for pt in pts_list:
            px, py = self.world_to_bev(pt[0], pt[1], car_xy, cos_yaw, sin_yaw)
            if self._in_bounds(px, py):
                bev_pts.append((px, py))

        if len(bev_pts) < 2:
            return

        if dashed:
            for i in range(0, len(bev_pts) - 1, 2):
                j = min(i + 1, len(bev_pts) - 1)
                cv2.line(img, bev_pts[i], bev_pts[j], color, thickness)
        else:
            for i in range(len(bev_pts) - 1):
                cv2.line(img, bev_pts[i], bev_pts[i + 1], color, thickness)

    def _compute_centerline(self, left_pts, right_pts):
        """Simple: pair left/right by index (both are uniformly sampled)."""
        n = min(len(left_pts), len(right_pts))
        if n == 0:
            return []
        center = []
        for i in range(n):
            cx = (left_pts[i][0] + right_pts[i][0]) / 2.0
            cy = (left_pts[i][1] + right_pts[i][1]) / 2.0
            center.append((cx, cy))
        return center


# ============================================================================
#  DIAGNOSTIC STRIP
# ============================================================================

class DiagStrip:
    def __init__(self, width=500, height=150, history=200):
        self.w = width
        self.h = height
        self.history = history
        self._lat_hist = deque(maxlen=history)
        self._hdg_hist = deque(maxlen=history)
        self._q_hist = deque(maxlen=history)

    def push(self, lat, hdg, q):
        self._lat_hist.append(lat)
        self._hdg_hist.append(hdg)
        self._q_hist.append(q)

    def render(self):
        img = np.full((self.h, self.w, 3), (12, 12, 16), dtype=np.uint8)
        mid_y = self.h // 2
        cv2.line(img, (0, mid_y), (self.w, mid_y), (40, 40, 40), 1)

        n = len(self._lat_hist)
        if n < 2:
            return img

        lat = list(self._lat_hist)
        hdg = list(self._hdg_hist)
        q = list(self._q_hist)

        scale_lat = (self.h // 2 - 10) / 0.3
        scale_hdg = (self.h // 2 - 10) / 0.5

        for i in range(1, n):
            x0 = int((i - 1) / self.history * self.w)
            x1 = int(i / self.history * self.w)

            y0l = np.clip(int(mid_y - lat[i-1] * scale_lat), 0, self.h - 1)
            y1l = np.clip(int(mid_y - lat[i] * scale_lat), 0, self.h - 1)
            cv2.line(img, (x0, y0l), (x1, y1l), (200, 200, 0), 1)

            y0h = np.clip(int(mid_y - hdg[i-1] * scale_hdg), 0, self.h - 1)
            y1h = np.clip(int(mid_y - hdg[i] * scale_hdg), 0, self.h - 1)
            cv2.line(img, (x0, y0h), (x1, y1h), (200, 0, 200), 1)

            q_h = int(q[i] * 8)
            cv2.line(img, (x1, self.h - 1), (x1, self.h - 1 - q_h),
                     (0, 150, 0) if q[i] >= 0.5 else (0, 80, 200), 1)

        cv2.putText(img, "Lat (cyan)  Hdg (magenta)  Q (bar)",
                    (5, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (120, 120, 120), 1)

        return img


# ============================================================================
#  ROS 2 NODE
# ============================================================================

class SlamVisualizerNode(Node):

    def __init__(self):
        super().__init__('slam_visualizer')
        self.bridge = CvBridge()

        self._bev = BevRenderer(img_size=500, meters_per_pixel=0.008)
        self._strip = DiagStrip(width=500, height=150, history=200)

        self._car_xy = np.array([0.0, 0.0])
        self._car_yaw = 0.0
        self._odom_trail = deque(maxlen=500)
        self._left_raw = []
        self._right_raw = []
        self._left_curve = []
        self._right_curve = []
        self._guidance = Vector3()
        self._raw_guidance = Vector3()
        self._diagnostics = {}

        self._bev_pub = self.create_publisher(
            Image, '/perception/debug/bev_map', 5)
        self._diag_pub = self.create_publisher(
            Image, '/perception/debug/slam_diag', 5)

        self.create_subscription(
            Odometry, '/zed/zed_node/odom', self._odom_cb, 10)
        self.create_subscription(
            PointCloud2, '/slam/lane_map', self._map_cb, 5)
        self.create_subscription(
            PointCloud2, '/slam/lane_curves', self._curves_cb, 5)
        self.create_subscription(
            Vector3, '/perception/lane_guidance', self._guidance_cb, 10)
        self.create_subscription(
            Vector3, '/perception/lane_guidance_raw', self._raw_guidance_cb, 10)
        self.create_subscription(
            String, '/slam/diagnostics', self._diag_cb, 5)

        self.create_timer(0.25, self._render_tick)  # 4 Hz
        self.get_logger().info('SlamVisualizer v3 ready (curves + centerline).')

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        self._car_xy = np.array([pos.x, pos.y])
        self._car_yaw = quat_to_yaw(msg.pose.pose.orientation)
        self._odom_trail.append((pos.x, pos.y))

    def _decode_labeled_pc2(self, msg):
        n_fields = msg.point_step // 4
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, n_fields)
        if raw.shape[0] == 0:
            return [], []
        pts = raw[:, :2]  # XY only
        labels = raw[:, 3] if n_fields >= 4 else np.zeros(raw.shape[0])
        left = pts[labels < 0.5].tolist()
        right = pts[labels >= 0.5].tolist()
        return left, right

    def _map_cb(self, msg):
        self._left_raw, self._right_raw = self._decode_labeled_pc2(msg)

    def _curves_cb(self, msg):
        self._left_curve, self._right_curve = self._decode_labeled_pc2(msg)

    def _guidance_cb(self, msg):
        self._guidance = msg
        self._strip.push(msg.x, msg.y, msg.z)

    def _raw_guidance_cb(self, msg):
        self._raw_guidance = msg

    def _diag_cb(self, msg):
        try:
            self._diagnostics = json.loads(msg.data)
        except Exception:
            pass

    def _render_tick(self):
        bev_img = self._bev.render(
            self._car_xy, self._car_yaw,
            self._left_raw, self._right_raw,
            self._left_curve, self._right_curve,
            list(self._odom_trail),
            self._guidance.x, self._guidance.y, self._guidance.z,
            self._diagnostics, self._raw_guidance)

        bev_msg = self.bridge.cv2_to_imgmsg(bev_img, encoding='bgr8')
        bev_msg.header.stamp = self.get_clock().now().to_msg()
        self._bev_pub.publish(bev_msg)

        strip_img = self._strip.render()
        strip_msg = self.bridge.cv2_to_imgmsg(strip_img, encoding='bgr8')
        strip_msg.header.stamp = self.get_clock().now().to_msg()
        self._diag_pub.publish(strip_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

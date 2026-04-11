#!/usr/bin/env python3
"""
vision_perception/lane_slam.py — Lane SLAM node (v3).

Major improvements:
  - Odom pose interpolation: aligns point cloud timestamps with odom poses
    to account for IMU/vision timing offset.
  - Persistent consolidated map: raw points are fitted into polylines sampled
    at 0.1m intervals. Consolidated points persist until the car passes them.
  - Curve fitting: polyline fit per lane, published for visualization.
  - Guidance computed from consolidated curves + live data fusion.

SUBSCRIBES
  /perception/lane_points_raw   PointCloud2 (camera frame, with label channel)
  /perception/lane_guidance_raw Vector3
  /zed/zed_node/odom            Odometry

PUBLISHES
  /perception/lane_guidance     Vector3 (fused — controller reads this)
  /slam/lane_map                PointCloud2 (world frame raw+consolidated)
  /slam/lane_curves             PointCloud2 (world frame, sampled polylines only)
  /slam/diagnostics             String (JSON)
"""

import json
import math
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Header


# ============================================================================
#  GEOMETRY
# ============================================================================

def quat_to_rotation_matrix(qx, qy, qz, qw):
    r00 = 1 - 2*(qy*qy + qz*qz)
    r01 = 2*(qx*qy - qz*qw)
    r02 = 2*(qx*qz + qy*qw)
    r10 = 2*(qx*qy + qz*qw)
    r11 = 1 - 2*(qx*qx + qz*qz)
    r12 = 2*(qy*qz - qx*qw)
    r20 = 2*(qx*qz - qy*qw)
    r21 = 2*(qy*qz + qx*qw)
    r22 = 1 - 2*(qx*qx + qy*qy)
    return np.array([[r00, r01, r02],
                     [r10, r11, r12],
                     [r20, r21, r22]], dtype=np.float64)


def make_transform_raw(px, py, pz, qx, qy, qz, qw):
    R = quat_to_rotation_matrix(qx, qy, qz, qw)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[0, 3] = px
    T[1, 3] = py
    T[2, 3] = pz
    return T


def make_transform(position, orientation):
    return make_transform_raw(
        position.x, position.y, position.z,
        orientation.x, orientation.y, orientation.z, orientation.w)


def transform_points(pts_3d, T):
    N = pts_3d.shape[0]
    ones = np.ones((N, 1), dtype=np.float64)
    pts_h = np.hstack([pts_3d.astype(np.float64), ones])
    transformed = (T @ pts_h.T).T
    return transformed[:, :3].astype(np.float32)


def yaw_from_quat(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def lerp(a, b, t):
    return a + (b - a) * t


def slerp_yaw(y0, y1, t):
    diff = math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))
    return y0 + diff * t


# ============================================================================
#  ODOM POSE BUFFER (for timestamp interpolation)
# ============================================================================

class OdomBuffer:
    """
    Stores recent odom poses with timestamps for interpolation.
    When a point cloud arrives, look up the odom pose at the cloud's
    capture time — not the latest pose — to account for sensor timing offsets.
    """

    def __init__(self, max_size=200):
        # Each entry: (timestamp_sec, px, py, pz, qx, qy, qz, qw)
        self._buf = deque(maxlen=max_size)

    def add(self, t_sec, pos, ori):
        self._buf.append((
            t_sec, pos.x, pos.y, pos.z,
            ori.x, ori.y, ori.z, ori.w))

    @property
    def latest(self):
        if not self._buf:
            return None
        return self._buf[-1]

    def lookup(self, t_sec):
        """
        Interpolate odom pose at time t_sec.
        Returns (T_world_cam_4x4, car_xy, car_yaw) or None.
        """
        if len(self._buf) < 2:
            if self._buf:
                e = self._buf[-1]
                T = make_transform_raw(e[1], e[2], e[3], e[4], e[5], e[6], e[7])
                yaw = yaw_from_quat(e[4], e[5], e[6], e[7])
                return T, np.array([e[1], e[2]]), yaw
            return None

        # Find bracketing entries
        buf = self._buf
        if t_sec <= buf[0][0]:
            e = buf[0]
        elif t_sec >= buf[-1][0]:
            e = buf[-1]
        else:
            # Binary-ish search (buffer is sorted by time)
            lo, hi = None, None
            for i in range(len(buf) - 1):
                if buf[i][0] <= t_sec <= buf[i + 1][0]:
                    lo, hi = buf[i], buf[i + 1]
                    break
            if lo is None:
                e = buf[-1]
            else:
                dt = hi[0] - lo[0]
                alpha = (t_sec - lo[0]) / dt if dt > 1e-9 else 0.0
                alpha = max(0.0, min(1.0, alpha))
                px = lerp(lo[1], hi[1], alpha)
                py = lerp(lo[2], hi[2], alpha)
                pz = lerp(lo[3], hi[3], alpha)
                # Simple lerp on quaternion (good enough for small dt)
                qx = lerp(lo[4], hi[4], alpha)
                qy = lerp(lo[5], hi[5], alpha)
                qz = lerp(lo[6], hi[6], alpha)
                qw = lerp(lo[7], hi[7], alpha)
                norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
                if norm > 1e-9:
                    qx /= norm; qy /= norm; qz /= norm; qw /= norm
                T = make_transform_raw(px, py, pz, qx, qy, qz, qw)
                yaw = yaw_from_quat(qx, qy, qz, qw)
                return T, np.array([px, py]), yaw

        T = make_transform_raw(e[1], e[2], e[3], e[4], e[5], e[6], e[7])
        yaw = yaw_from_quat(e[4], e[5], e[6], e[7])
        return T, np.array([e[1], e[2]]), yaw


# ============================================================================
#  POLYLINE FITTING + RESAMPLING
# ============================================================================

def fit_and_resample_lane(pts_xy, spacing=0.1, smooth_window=5):
    """
    Given Nx2 world-frame lane points, sort by cumulative arc length,
    smooth, and resample at uniform spacing.
    Returns Mx2 resampled points, or None if too few.
    """
    if pts_xy.shape[0] < 3:
        return None

    # Sort by greedy nearest-neighbor traversal (handles curves)
    ordered = _sort_by_traversal(pts_xy)
    if ordered.shape[0] < 3:
        return None

    # Smooth with moving average
    if smooth_window > 1 and ordered.shape[0] >= smooth_window:
        kernel = np.ones(smooth_window) / smooth_window
        sx = np.convolve(ordered[:, 0], kernel, mode='valid')
        sy = np.convolve(ordered[:, 1], kernel, mode='valid')
        ordered = np.column_stack([sx, sy])

    if ordered.shape[0] < 2:
        return None

    # Compute cumulative arc length
    diffs = np.diff(ordered, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cum_len = np.concatenate([[0.0], np.cumsum(seg_lengths)])
    total_len = cum_len[-1]

    if total_len < spacing:
        return ordered

    # Resample at uniform spacing
    n_samples = int(total_len / spacing) + 1
    sample_dists = np.linspace(0, total_len, n_samples)

    resampled = np.zeros((n_samples, 2), dtype=np.float32)
    for i, d in enumerate(sample_dists):
        idx = np.searchsorted(cum_len, d, side='right') - 1
        idx = max(0, min(idx, len(ordered) - 2))
        seg_d = d - cum_len[idx]
        seg_l = seg_lengths[idx] if seg_lengths[idx] > 1e-9 else 1e-9
        t = min(seg_d / seg_l, 1.0)
        resampled[i] = ordered[idx] * (1 - t) + ordered[idx + 1] * t

    return resampled


def _sort_by_traversal(pts):
    """Greedy nearest-neighbor traversal starting from the point with smallest X."""
    N = pts.shape[0]
    if N <= 2:
        return pts

    # Start from the point closest to the car (smallest X in world... or just pick min-X)
    start = np.argmin(pts[:, 0])
    visited = np.zeros(N, dtype=bool)
    order = [start]
    visited[start] = True

    for _ in range(N - 1):
        last = pts[order[-1]]
        dists = np.linalg.norm(pts - last, axis=1)
        dists[visited] = 1e9
        nxt = np.argmin(dists)
        if dists[nxt] > 1e8:
            break
        order.append(nxt)
        visited[nxt] = True

    return pts[order]


# ============================================================================
#  PERSISTENT LANE MAP
# ============================================================================

class PersistentLaneMap:
    """
    Two-tier lane map:
      - Raw: recent observations (deque, used for fitting). Expire after raw_max_age.
      - Consolidated: polyline samples at 0.1m spacing. Persist until the car
        has driven past them by >behind_prune_m.
    """

    def __init__(self, raw_max_age=8.0, raw_max_pts=8000,
                 consolidate_interval=2.0, sample_spacing=0.1,
                 behind_prune_m=1.5):
        self.raw_max_age = raw_max_age
        self.sample_spacing = sample_spacing
        self.behind_prune_m = behind_prune_m
        self.consolidate_interval = consolidate_interval

        # Raw: (xyz_3d, timestamp)
        self._left_raw = deque(maxlen=raw_max_pts)
        self._right_raw = deque(maxlen=raw_max_pts)

        # Consolidated: Nx2 world XY polyline samples (persistent)
        self._left_curve = None   # Mx2
        self._right_curve = None  # Mx2

        self._last_consolidate = 0.0

    def add_raw(self, left_world_3d, right_world_3d, timestamp):
        if left_world_3d is not None:
            for p in left_world_3d[::2]:
                self._left_raw.append((p[:2].copy(), timestamp))
        if right_world_3d is not None:
            for p in right_world_3d[::2]:
                self._right_raw.append((p[:2].copy(), timestamp))

    def consolidate(self, now, car_xy, car_yaw):
        """
        Fit polylines to raw points + existing consolidated, resample,
        and prune points behind the car.
        """
        if now - self._last_consolidate < self.consolidate_interval:
            return
        self._last_consolidate = now

        # Prune old raw
        cutoff = now - self.raw_max_age
        while self._left_raw and self._left_raw[0][1] < cutoff:
            self._left_raw.popleft()
        while self._right_raw and self._right_raw[0][1] < cutoff:
            self._right_raw.popleft()

        # Merge raw + existing consolidated for fitting
        self._left_curve = self._refit_lane(
            self._left_raw, self._left_curve, car_xy, car_yaw)
        self._right_curve = self._refit_lane(
            self._right_raw, self._right_curve, car_xy, car_yaw)

    def _refit_lane(self, raw_deque, old_curve, car_xy, car_yaw):
        # Collect raw points
        raw_pts = [p for p, t in raw_deque]
        all_pts = []
        if raw_pts:
            all_pts.append(np.array(raw_pts, dtype=np.float32))
        if old_curve is not None and old_curve.shape[0] > 0:
            all_pts.append(old_curve)

        if not all_pts:
            return None

        merged = np.vstack(all_pts)

        # Deduplicate (voxel grid 2cm)
        if merged.shape[0] > 0:
            keys = (merged / 0.02).astype(np.int32)
            _, idx = np.unique(keys, axis=0, return_index=True)
            merged = merged[idx]

        # Prune points behind the car
        if merged.shape[0] > 0:
            dx = merged[:, 0] - car_xy[0]
            dy = merged[:, 1] - car_xy[1]
            cos_y = math.cos(car_yaw)
            sin_y = math.sin(car_yaw)
            forward = dx * cos_y + dy * sin_y  # dot with heading
            keep = forward > -self.behind_prune_m
            merged = merged[keep]

        if merged.shape[0] < 3:
            return merged if merged.shape[0] > 0 else None

        # Fit and resample
        resampled = fit_and_resample_lane(merged, spacing=self.sample_spacing)
        return resampled

    def get_raw_points(self, now):
        cutoff = now - self.raw_max_age
        left = [p for p, t in self._left_raw if t > cutoff]
        right = [p for p, t in self._right_raw if t > cutoff]
        l = np.array(left, dtype=np.float32) if left else None
        r = np.array(right, dtype=np.float32) if right else None
        return l, r

    def get_curves(self):
        return self._left_curve, self._right_curve

    def get_all_points(self, now):
        """Return combined raw + consolidated for each lane (world XY)."""
        cutoff = now - self.raw_max_age
        left_raw = [p for p, t in self._left_raw if t > cutoff]
        right_raw = [p for p, t in self._right_raw if t > cutoff]

        def combine(raw_list, curve):
            parts = []
            if raw_list:
                parts.append(np.array(raw_list, dtype=np.float32))
            if curve is not None:
                parts.append(curve)
            if parts:
                merged = np.vstack(parts)
                # Quick dedup
                keys = (merged / 0.02).astype(np.int32)
                _, idx = np.unique(keys, axis=0, return_index=True)
                return merged[idx]
            return None

        return combine(left_raw, self._left_curve), combine(right_raw, self._right_curve)

    @property
    def stats(self):
        lc = self._left_curve.shape[0] if self._left_curve is not None else 0
        rc = self._right_curve.shape[0] if self._right_curve is not None else 0
        return {
            'raw_left': len(self._left_raw),
            'raw_right': len(self._right_raw),
            'curve_left': lc,
            'curve_right': rc,
        }


# ============================================================================
#  ERROR COMPUTATION
# ============================================================================

def compute_errors_binned(left_xy, right_xy, min_pts=3,
                          x_min=0.08, x_max=2.0, n_bins=12,
                          half_width=0.20):
    bin_edges = np.linspace(x_min, x_max, n_bins + 1)
    have_left = left_xy is not None and left_xy.shape[0] >= min_pts
    have_right = right_xy is not None and right_xy.shape[0] >= min_pts
    quality = (float(have_left) + float(have_right)) / 2.0

    if not have_left and not have_right:
        return 0.0, 0.0, 0.0

    center_ys, center_xs = [], []
    for i in range(n_bins):
        lo, hi = bin_edges[i], bin_edges[i + 1]
        x_mid = (lo + hi) / 2.0
        left_y = right_y = None
        if have_left:
            m = (left_xy[:, 0] >= lo) & (left_xy[:, 0] < hi)
            if m.sum() >= 2:
                left_y = np.median(left_xy[m, 1])
        if have_right:
            m = (right_xy[:, 0] >= lo) & (right_xy[:, 0] < hi)
            if m.sum() >= 2:
                right_y = np.median(right_xy[m, 1])
        if left_y is not None and right_y is not None:
            center_ys.append((left_y + right_y) / 2.0)
            center_xs.append(x_mid)
        elif left_y is not None:
            center_ys.append(left_y - half_width)
            center_xs.append(x_mid)
        elif right_y is not None:
            center_ys.append(right_y + half_width)
            center_xs.append(x_mid)

    if not center_ys:
        return 0.0, 0.0, quality

    cys = np.array(center_ys)
    cxs = np.array(center_xs)
    weights = 1.0 / (cxs ** 2)
    weights /= weights.sum()
    lat_err = float(np.dot(weights, cys))

    if len(cys) >= 3:
        mid = len(cys) // 2
        hdg_err = math.atan2(cys[mid] - cys[0], cxs[mid] - cxs[0])
    elif len(cys) >= 2:
        hdg_err = math.atan2(cys[-1] - cys[0], cxs[-1] - cxs[0])
    else:
        hdg_err = 0.0

    return lat_err, hdg_err, quality


# ============================================================================
#  PointCloud2 HELPER
# ============================================================================

def make_pc2(pts_3d_or_2d, header, label_channel=None):
    pts = pts_3d_or_2d.astype(np.float32)
    if pts.ndim == 2 and pts.shape[1] == 2:
        pts = np.hstack([pts, np.zeros((pts.shape[0], 1), dtype=np.float32)])

    N = pts.shape[0]
    if label_channel is not None:
        data = np.hstack([pts, label_channel.astype(np.float32).reshape(-1, 1)])
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='label', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        step = 16
    else:
        data = pts
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        step = 12

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = N
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = step
    msg.row_step = step * N
    msg.data = data.tobytes()
    msg.is_dense = True
    return msg


# ============================================================================
#  ROS 2 NODE
# ============================================================================

class LaneSlamNode(Node):

    MIN_PTS = 6
    MAP_RECALL_RANGE = 2.5
    MIN_RANGE = 0.08

    ALPHA_LAT_LIVE = 0.80
    ALPHA_LAT_MAP = 0.50
    ALPHA_HDG_LIVE = 0.75
    ALPHA_HDG_MAP = 0.45

    def __init__(self):
        super().__init__('lane_slam')
        self._smooth_lat = 0.0
        self._smooth_hdg = 0.0

        # Odom interpolation buffer
        self._odom_buf = OdomBuffer(max_size=300)
        self._have_odom = False
        self._car_world_xy = np.array([0.0, 0.0])
        self._car_yaw = 0.0
        self._T_cam_world = np.eye(4, dtype=np.float64)

        # Persistent lane map
        self._lane_map = PersistentLaneMap(
            raw_max_age=10.0,       # keep raw for 10s
            raw_max_pts=10000,
            consolidate_interval=1.5,
            sample_spacing=0.1,
            behind_prune_m=1.0,     # prune only 1m behind car
        )

        self._latest_raw = Vector3(x=0.0, y=0.0, z=0.0)

        # Publishers
        self._guidance_pub = self.create_publisher(
            Vector3, '/perception/lane_guidance', 10)
        self._map_pub = self.create_publisher(
            PointCloud2, '/slam/lane_map', 5)
        self._curves_pub = self.create_publisher(
            PointCloud2, '/slam/lane_curves', 5)
        self._diag_pub = self.create_publisher(
            String, '/slam/diagnostics', 5)

        # Subscribers
        self.create_subscription(
            Odometry, '/zed/zed_node/odom', self._odom_cb, 10)
        self.create_subscription(
            PointCloud2, '/perception/lane_points_raw', self._points_cb, 10)
        self.create_subscription(
            Vector3, '/perception/lane_guidance_raw', self._raw_guidance_cb, 10)

        # Periodic: consolidate + publish (2 Hz)
        self.create_timer(0.5, self._periodic_cb)

        self.get_logger().info('LaneSlam v3 (interpolation + persistent map) ready.')

    # ---- Odom ----

    def _odom_cb(self, msg):
        self._have_odom = True
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self._odom_buf.add(t_sec, pos, ori)

        # Update latest pose (for visualization / quick access)
        self._car_world_xy = np.array([pos.x, pos.y])
        siny = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self._car_yaw = math.atan2(siny, cosy)

    # ---- Raw guidance fallback ----

    def _raw_guidance_cb(self, msg):
        self._latest_raw = msg

    # ---- Points from perception ----

    def _points_cb(self, msg):
        if not self._have_odom:
            self._guidance_pub.publish(self._latest_raw)
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        # ---- INTERPOLATED POSE at point cloud capture time ----
        pc_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose_result = self._odom_buf.lookup(pc_time)
        if pose_result is None:
            self._guidance_pub.publish(self._latest_raw)
            return

        T_world_cam, car_xy_at_capture, car_yaw_at_capture = pose_result
        try:
            T_cam_world = np.linalg.inv(T_world_cam)
        except np.linalg.LinAlgError:
            T_cam_world = np.eye(4, dtype=np.float64)

        # Also compute current T_cam_world for recall
        latest = self._odom_buf.latest
        if latest:
            T_wc_now = make_transform_raw(latest[1], latest[2], latest[3],
                                          latest[4], latest[5], latest[6], latest[7])
            try:
                self._T_cam_world = np.linalg.inv(T_wc_now)
            except np.linalg.LinAlgError:
                pass

        # Decode PointCloud2
        n_fields = msg.point_step // 4
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, n_fields)
        if raw.shape[0] == 0:
            self._guidance_pub.publish(self._latest_raw)
            return

        pts_cam_3d = raw[:, :3]
        labels = raw[:, 3] if n_fields >= 4 else np.zeros(raw.shape[0])

        left_cam = pts_cam_3d[labels < 0.5]
        right_cam = pts_cam_3d[labels >= 0.5]

        # ---- Store to world map using INTERPOLATED pose ----
        left_world = transform_points(left_cam, T_world_cam) if left_cam.shape[0] > 0 else None
        right_world = transform_points(right_cam, T_world_cam) if right_cam.shape[0] > 0 else None
        self._lane_map.add_raw(left_world, right_world, now)

        # ---- Recall consolidated + raw from map ----
        left_all_w, right_all_w = self._lane_map.get_all_points(now)

        # Transform to current camera frame for guidance computation
        left_cam_merged = self._world_to_cam(left_all_w)
        right_cam_merged = self._world_to_cam(right_all_w)

        left_cam_merged = self._range_filter(left_cam_merged)
        right_cam_merged = self._range_filter(right_cam_merged)

        # Also keep live points
        merged_left = self._merge(left_cam, left_cam_merged)
        merged_right = self._merge(right_cam, right_cam_merged)

        have_left = merged_left is not None and merged_left.shape[0] >= self.MIN_PTS
        have_right = merged_right is not None and merged_right.shape[0] >= self.MIN_PTS

        used_map = (
            (left_cam.shape[0] < self.MIN_PTS and left_cam_merged is not None) or
            (right_cam.shape[0] < self.MIN_PTS and right_cam_merged is not None)
        )

        if not have_left and not have_right:
            self._guidance_pub.publish(self._latest_raw)
            return

        # ---- Compute guidance ----
        left_xy = merged_left[:, :2] if have_left else None
        right_xy = merged_right[:, :2] if have_right else None

        lat_err, hdg_err, quality = compute_errors_binned(
            left_xy, right_xy, min_pts=3,
            x_min=self.MIN_RANGE, x_max=self.MAP_RECALL_RANGE, n_bins=12)

        if quality > 0:
            a_lat = self.ALPHA_LAT_MAP if used_map else self.ALPHA_LAT_LIVE
            a_hdg = self.ALPHA_HDG_MAP if used_map else self.ALPHA_HDG_LIVE
            self._smooth_lat = a_lat * lat_err + (1 - a_lat) * self._smooth_lat
            self._smooth_hdg = a_hdg * hdg_err + (1 - a_hdg) * self._smooth_hdg

        guidance = Vector3()
        guidance.x = float(self._smooth_lat)
        guidance.y = float(self._smooth_hdg)
        guidance.z = float(quality)
        self._guidance_pub.publish(guidance)

        src = "MAP" if used_map else "LIVE"
        stats = self._lane_map.stats
        self.get_logger().info(
            f'[{src}] lat={self._smooth_lat:.3f}m  '
            f'hdg={math.degrees(self._smooth_hdg):.1f}deg  '
            f'q={quality:.1f}  '
            f'raw={stats["raw_left"]}L/{stats["raw_right"]}R  '
            f'curve={stats["curve_left"]}L/{stats["curve_right"]}R')

    # ---- Helpers ----

    def _world_to_cam(self, world_pts):
        if world_pts is None or world_pts.shape[0] == 0:
            return None
        if world_pts.shape[1] == 2:
            world_pts = np.hstack([world_pts,
                                   np.zeros((world_pts.shape[0], 1), dtype=np.float32)])
        return transform_points(world_pts, self._T_cam_world)

    def _range_filter(self, cam_pts):
        if cam_pts is None or cam_pts.shape[0] == 0:
            return None
        r = np.linalg.norm(cam_pts[:, :2], axis=1)
        mask = (cam_pts[:, 0] > self.MIN_RANGE) & (r < self.MAP_RECALL_RANGE)
        filtered = cam_pts[mask]
        return filtered if filtered.shape[0] > 0 else None

    def _merge(self, live, mapped):
        if live is not None and live.shape[0] >= self.MIN_PTS:
            if mapped is not None and mapped.shape[0] > 0:
                n_map = min(mapped.shape[0], max(live.shape[0] // 2, 20))
                idx = np.linspace(0, mapped.shape[0] - 1, n_map, dtype=int)
                return np.vstack([live, mapped[idx]])
            return live
        elif mapped is not None and mapped.shape[0] >= self.MIN_PTS:
            return mapped
        elif live is not None and live.shape[0] > 0:
            if mapped is not None:
                return np.vstack([live, mapped])
            return live
        return mapped

    # ---- Periodic: consolidate + publish ----

    def _periodic_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # Consolidate (fit curves, prune behind car)
        self._lane_map.consolidate(now, self._car_world_xy, self._car_yaw)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'odom'

        # Publish all map points
        left_all, right_all = self._lane_map.get_all_points(now)
        pts_list, labels_list = [], []
        if left_all is not None:
            pts_list.append(left_all)
            labels_list.append(np.zeros(left_all.shape[0], dtype=np.float32))
        if right_all is not None:
            pts_list.append(right_all)
            labels_list.append(np.ones(right_all.shape[0], dtype=np.float32))
        if pts_list:
            all_pts = np.vstack(pts_list)
            all_labels = np.concatenate(labels_list)
            self._map_pub.publish(make_pc2(all_pts, header, label_channel=all_labels))

        # Publish fitted curves
        left_curve, right_curve = self._lane_map.get_curves()
        curve_pts, curve_labels = [], []
        if left_curve is not None and left_curve.shape[0] > 0:
            curve_pts.append(left_curve)
            curve_labels.append(np.zeros(left_curve.shape[0], dtype=np.float32))
        if right_curve is not None and right_curve.shape[0] > 0:
            curve_pts.append(right_curve)
            curve_labels.append(np.ones(right_curve.shape[0], dtype=np.float32))
        if curve_pts:
            cp = np.vstack(curve_pts)
            cl = np.concatenate(curve_labels)
            self._curves_pub.publish(make_pc2(cp, header, label_channel=cl))

        # Diagnostics
        stats = self._lane_map.stats
        diag = {
            'raw_left': stats['raw_left'],
            'raw_right': stats['raw_right'],
            'curve_left': stats['curve_left'],
            'curve_right': stats['curve_right'],
            'car_x': float(self._car_world_xy[0]),
            'car_y': float(self._car_world_xy[1]),
            'car_yaw_deg': float(math.degrees(self._car_yaw)),
            'smooth_lat': float(self._smooth_lat),
            'smooth_hdg_deg': float(math.degrees(self._smooth_hdg)),
        }
        self._diag_pub.publish(String(data=json.dumps(diag)))


def main(args=None):
    rclpy.init(args=args)
    node = LaneSlamNode()
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

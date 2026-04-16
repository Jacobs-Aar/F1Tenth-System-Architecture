#!/usr/bin/env python3
"""
vision_perception/line_mapper.py — Pure perception node (v3).

Detects blue-tape lane edges, lifts to 3D, splits left/right using
propagation-based labeling that follows curves correctly.

PUBLISHES
  /perception/lane_guidance_raw  Vector3 (live-only guidance)
  /perception/lane_points_raw    PointCloud2 (camera frame, with label channel)
  /perception/debug/lane_mask    Image (mono8)
  /perception/debug/raw_crop     Image (bgr8)
  /perception/debug/edges        Image (mono8)
  /perception/debug/bev_local    Image (bgr8)
"""

import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import Vector3
import message_filters
from cv_bridge import CvBridge


# ============================================================================
#  IMAGE PROCESSING
# ============================================================================

def lane_mask_blue(bgr_roi):
    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (99, 63, 133), (155, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=1)
    return mask


def mask_to_edges(mask):
    return cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, np.ones((5, 5), np.uint8))


def vectorized_xyz_lookup(pc_xyz, xs_pc, ys_pc):
    pts = pc_xyz[ys_pc, xs_pc, :3]
    valid = np.isfinite(pts).all(axis=1)
    return pts[valid]


# ============================================================================
#  BIN-CHAIN LANE SPLITTER (O(N), curve-robust, clamped centroids)
# ============================================================================

def split_lanes_propagated(pts_xy, seed_x_max=0.4, n_bins=20, iters=8,
                           max_shift=0.05, min_gap=0.10):
    """
    Fast curve-robust lane splitter using bin-chaining with clamped centroids.

    1. Bin all points by X into n_bins slices.
    2. In the nearest populated bin, two-means on Y to get left/right centroids.
    3. Chain forward: each bin assigns points using the previous bin's centroids,
       then updates centroids — BUT the update is CLAMPED so each centroid can
       move at most max_shift per bin. This makes it physically impossible for
       a centroid to jump across to the other lane (~0.4m away).
    4. Minimum gap enforced: if centroids get closer than min_gap, freeze them.

    Returns (left_xy, right_xy, labels_array) indexed to original ordering.
    """
    N = pts_xy.shape[0]
    if N < 4:
        if N < 2:
            return pts_xy, pts_xy, np.zeros(N, dtype=np.int32)
        median_y = np.median(pts_xy[:, 1])
        labels = (pts_xy[:, 1] < median_y).astype(np.int32)
        return pts_xy[labels == 0], pts_xy[labels == 1], labels

    x_vals = pts_xy[:, 0]
    y_vals = pts_xy[:, 1]
    x_lo = max(x_vals.min(), 0.05)
    x_hi = x_vals.max() + 0.01

    bin_edges = np.linspace(x_lo, x_hi, n_bins + 1)
    labels = np.full(N, -1, dtype=np.int32)
    bin_idx = np.clip(np.digitize(x_vals, bin_edges) - 1, 0, n_bins - 1)

    # ---- Seed from the nearest bin with enough points ----
    c_left = None
    c_right = None
    seed_bin = -1

    for b in range(n_bins):
        mask = bin_idx == b
        if mask.sum() < 4:
            continue

        y_bin = y_vals[mask]
        c_left, c_right = np.percentile(y_bin, [75, 25])

        for _ in range(iters):
            lbl = np.abs(y_bin - c_left) < np.abs(y_bin - c_right)
            if lbl.all() or (~lbl).all():
                break
            c_left = y_bin[lbl].mean()
            c_right = y_bin[~lbl].mean()

        if c_left < c_right:
            c_left, c_right = c_right, c_left

        # Assign seed bin
        bin_labels = (np.abs(y_vals[mask] - c_left) >= np.abs(y_vals[mask] - c_right)).astype(np.int32)
        labels[mask] = bin_labels
        seed_bin = b
        break

    if c_left is None:
        median_y = np.median(y_vals)
        labels = (y_vals < median_y).astype(np.int32)
        return pts_xy[labels == 0], pts_xy[labels == 1], labels

    # ---- Chain through all bins (forward and backward from seed) ----
    def chain_bins(bin_range, c_l, c_r):
        cl, cr = c_l, c_r
        for b in bin_range:
            mask = bin_idx == b
            count = mask.sum()
            if count == 0:
                continue
            if (labels[mask] >= 0).all():
                # Already assigned (seed bin) — just update centroids (clamped)
                cl, cr = _update_centroids_clamped(
                    y_vals, mask, labels, cl, cr, max_shift, min_gap)
                continue

            # Assign using current centroids
            bin_y = y_vals[mask]
            bin_labels = (np.abs(bin_y - cl) >= np.abs(bin_y - cr)).astype(np.int32)
            labels[mask] = bin_labels

            # Update centroids with clamping
            cl, cr = _update_centroids_clamped(
                y_vals, mask, labels, cl, cr, max_shift, min_gap)

    # Forward from seed
    chain_bins(range(seed_bin, n_bins), c_left, c_right)
    # Backward from seed (for any points closer than the seed bin)
    chain_bins(range(seed_bin - 1, -1, -1), c_left, c_right)

    # Any remaining unassigned
    unassigned = labels < 0
    if unassigned.any():
        labels[unassigned] = (
            np.abs(y_vals[unassigned] - c_left) >=
            np.abs(y_vals[unassigned] - c_right)
        ).astype(np.int32)

    left = pts_xy[labels == 0]
    right = pts_xy[labels == 1]

    # Verify: left should have higher Y near car
    near = x_vals < seed_x_max * 2
    if near.any() and left.shape[0] > 0 and right.shape[0] > 0:
        left_near_y = y_vals[(labels == 0) & near]
        right_near_y = y_vals[(labels == 1) & near]
        if left_near_y.shape[0] > 0 and right_near_y.shape[0] > 0:
            if left_near_y.mean() < right_near_y.mean():
                left, right = right, left
                labels = 1 - labels

    return left, right, labels


def _update_centroids_clamped(y_vals, mask, labels, c_left, c_right,
                              max_shift, min_gap):
    """
    Compute new centroids from a bin's assigned points, but clamp the
    movement so neither centroid can shift more than max_shift.
    Also enforce minimum gap between centroids.
    """
    left_y = y_vals[mask & (labels == 0)]
    right_y = y_vals[mask & (labels == 1)]

    new_cl = c_left
    new_cr = c_right

    if left_y.shape[0] >= 2:
        raw = left_y.mean()
        delta = np.clip(raw - c_left, -max_shift, max_shift)
        new_cl = c_left + delta

    if right_y.shape[0] >= 2:
        raw = right_y.mean()
        delta = np.clip(raw - c_right, -max_shift, max_shift)
        new_cr = c_right + delta

    # Enforce minimum gap (left must stay above right)
    if new_cl - new_cr < min_gap:
        # Freeze — don't update
        return c_left, c_right

    return new_cl, new_cr


# ============================================================================
#  ERROR COMPUTATION
# ============================================================================

def compute_errors_binned(left_xy, right_xy, min_pts=3,
                          x_min=0.08, x_max=1.5, n_bins=10,
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
#  HELPER: PointCloud2
# ============================================================================

def points_to_pc2(pts_3d, header, label_channel=None):
    N = pts_3d.shape[0]
    if label_channel is not None:
        data = np.hstack([pts_3d.astype(np.float32),
                          label_channel.astype(np.float32).reshape(-1, 1)])
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='label', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        step = 16
    else:
        data = pts_3d.astype(np.float32)
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

class LineMapperNode(Node):

    MIN_PTS = 6
    MAX_SAMPLES = 3000
    STRIDE = 2
    MAX_RANGE = 1.5
    MIN_RANGE = 0.08
    EDGE_MARGIN = 5
    ROI_FRACTION = 0.50

    ALPHA_LAT = 0.80
    ALPHA_HDG = 0.75

    def __init__(self):
        super().__init__('line_mapper')
        self.bridge = CvBridge()
        self._smooth_lat = 0.0
        self._smooth_hdg = 0.0

        self._guidance_pub = self.create_publisher(
            Vector3, '/perception/lane_guidance_raw', 10)
        self._points_pub = self.create_publisher(
            PointCloud2, '/perception/lane_points_raw', 10)
        self._debug_pub = self.create_publisher(
            Image, '/perception/debug/lane_mask', 10)
            
        # --- NEW PUBLISHERS FOR WEB VISUALIZER ---
        self._raw_crop_pub = self.create_publisher(
            Image, '/perception/debug/raw_crop', 10)
        self._edges_pub = self.create_publisher(
            Image, '/perception/debug/edges', 10)
        self._bev_local_pub = self.create_publisher(
            Image, '/perception/debug/bev_local', 10)

        image_sub = message_filters.Subscriber(
            self, Image, '/zed/zed_node/rgb/color/rect/image')
        cloud_sub = message_filters.Subscriber(
            self, PointCloud2, '/zed/zed_node/point_cloud/cloud_registered')

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, cloud_sub], queue_size=10, slop=0.1)
        self._sync.registerCallback(self._synced_callback)

        self.get_logger().info('LineMapper v3 (propagation splitter) ready.')

    def _cloud_to_xyz(self, cloud_msg):
        raw = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        n_fields = cloud_msg.point_step // 4
        pts = raw.view(np.float32).reshape(
            cloud_msg.height, cloud_msg.width, n_fields)
        return np.ascontiguousarray(pts[:, :, :3], dtype=np.float32)

    def _synced_callback(self, image_msg, cloud_msg):
        t0 = time.time()
        try:
            bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        pc_xyz = self._cloud_to_xyz(cloud_msg)
        pc_h, pc_w = pc_xyz.shape[:2]
        img_h, img_w = bgr.shape[:2]

        y0 = int(img_h * self.ROI_FRACTION)
        roi = bgr[y0:, :]

        # --- 1. PUBLISH RAW CROP ---
        crop_msg = self.bridge.cv2_to_imgmsg(roi, encoding='bgr8')
        crop_msg.header = image_msg.header
        self._raw_crop_pub.publish(crop_msg)

        mask = lane_mask_blue(roi)
        edges = mask_to_edges(mask)

        # --- 2. PUBLISH FILTERED EDGES ---
        edges_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
        edges_msg.header = image_msg.header
        self._edges_pub.publish(edges_msg)

        ys_roi, xs_roi = np.where(edges > 0)

        debug_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        debug_msg.header = image_msg.header
        self._debug_pub.publish(debug_msg)

        if len(xs_roi) == 0:
            self._guidance_pub.publish(Vector3(x=0.0, y=0.0, z=0.0))
            return

        if len(xs_roi) > self.MAX_SAMPLES:
            idx = np.random.choice(len(xs_roi), self.MAX_SAMPLES, replace=False)
            xs_roi, ys_roi = xs_roi[idx], ys_roi[idx]
        elif self.STRIDE > 1:
            sel = np.arange(0, len(xs_roi), self.STRIDE)
            xs_roi, ys_roi = xs_roi[sel], ys_roi[sel]

        ys_img = ys_roi + y0
        xs_img = xs_roi
        keep = (
            (xs_img >= self.EDGE_MARGIN) & (xs_img < img_w - self.EDGE_MARGIN) &
            (ys_img >= self.EDGE_MARGIN) & (ys_img < img_h - self.EDGE_MARGIN)
        )
        xs_img, ys_img = xs_img[keep], ys_img[keep]

        if len(xs_img) == 0:
            self._guidance_pub.publish(Vector3(x=0.0, y=0.0, z=0.0))
            return

        scale_x = pc_w / img_w
        scale_y = pc_h / img_h
        xs_pc = np.clip((xs_img * scale_x).astype(np.int32), 0, pc_w - 1)
        ys_pc = np.clip((ys_img * scale_y).astype(np.int32), 0, pc_h - 1)
        pts = vectorized_xyz_lookup(pc_xyz, xs_pc, ys_pc)

        if pts.shape[0] < self.MIN_PTS:
            self._guidance_pub.publish(Vector3(x=0.0, y=0.0, z=0.0))
            return

        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r > self.MIN_RANGE) & (r < self.MAX_RANGE)]

        if pts.shape[0] < self.MIN_PTS:
            self._guidance_pub.publish(Vector3(x=0.0, y=0.0, z=0.0))
            return

        xy = pts[:, :2]
        left_xy, right_xy, labels_orig = split_lanes_propagated(xy, seed_x_max=0.4)

        # --- 3. PUBLISH LOCAL BEV ---
        # Create a blank dark canvas
        bev_img = np.full((400, 400, 3), (12, 12, 16), dtype=np.uint8)
        mpp = 0.005  # meters per pixel scale
        center_x = 200
        bottom_y = 380

        # Draw left lane (blue-ish)
        for pt in left_xy:
            x_m, y_m = pt[0], pt[1] # ZED frame: x is forward, y is left
            px = int(center_x - (y_m / mpp))
            py = int(bottom_y - (x_m / mpp))
            if 0 <= px < 400 and 0 <= py < 400:
                cv2.circle(bev_img, (px, py), 2, (255, 160, 50), -1) 

        # Draw right lane (red-ish)
        for pt in right_xy:
            x_m, y_m = pt[0], pt[1]
            px = int(center_x - (y_m / mpp))
            py = int(bottom_y - (x_m / mpp))
            if 0 <= px < 400 and 0 <= py < 400:
                cv2.circle(bev_img, (px, py), 2, (50, 50, 255), -1) 

        bev_msg = self.bridge.cv2_to_imgmsg(bev_img, encoding='bgr8')
        bev_msg.header = image_msg.header
        self._bev_local_pub.publish(bev_msg)

        have_left = left_xy.shape[0] >= self.MIN_PTS
        have_right = right_xy.shape[0] >= self.MIN_PTS

        if have_left or have_right:
            pc2_msg = points_to_pc2(
                pts, image_msg.header,
                label_channel=labels_orig.astype(np.float32))
            self._points_pub.publish(pc2_msg)

        lat_err, hdg_err, quality = compute_errors_binned(
            left_xy if have_left else None,
            right_xy if have_right else None,
            min_pts=3, x_min=self.MIN_RANGE, x_max=self.MAX_RANGE, n_bins=10)

        if quality > 0:
            self._smooth_lat = self.ALPHA_LAT * lat_err + (1 - self.ALPHA_LAT) * self._smooth_lat
            self._smooth_hdg = self.ALPHA_HDG * hdg_err + (1 - self.ALPHA_HDG) * self._smooth_hdg

        guidance = Vector3()
        guidance.x = float(self._smooth_lat)
        guidance.y = float(self._smooth_hdg)
        guidance.z = float(quality)
        self._guidance_pub.publish(guidance)

        dt = time.time() - t0
        hz = 1.0 / dt if dt > 0 else 0
        self.get_logger().info(
            f'[LIVE] lat={self._smooth_lat:.3f}m  '
            f'hdg={math.degrees(self._smooth_hdg):.1f}deg  '
            f'q={quality:.1f}  pts={pts.shape[0]}  {hz:.0f}Hz')


def main(args=None):
    rclpy.init(args=args)
    node = LineMapperNode()
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

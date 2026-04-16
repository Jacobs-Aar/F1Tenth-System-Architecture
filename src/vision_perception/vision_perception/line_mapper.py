#!/usr/bin/env python3
"""
vision_perception/line_mapper.py — Pure perception node (v5.0).

MAJOR OVERHAUL FROM v4.3
=========================
1. FULL-RANGE PARALLEL-FRONTIER BFS
   v4.3 ran BFS only on 0.50-1.25 m, then transferred labels to all other
   points via nearest-neighbour lookup. On curves, a far-field point's
   nearest labelled neighbour can sit on the WRONG tape because the tape
   has bent away from its near-field location. Result: ~30% of far-field
   points get swapped labels, which corrupts the map.

   v5.0 grows two chains simultaneously from near-field seeds over the
   ENTIRE observation range. At each round, every unlabelled point within
   `max_jump` of either chain is a candidate; if both chains claim the same
   point, it goes to whichever chain's source is closer. With
   `max_jump = 0.08 m` (well under the 0.40 m lane separation), noisy
   points cannot bridge to the wrong tape, and the chains follow curves
   naturally because new assignments are always anchored to an already-
   labelled member of the same chain.

2. FIT-BEFORE-STORE
   line_mapper now NN-orders each labelled side, moving-average smooths,
   and arc-length resamples at 0.05 m. What we publish is already-fitted,
   already-spaced samples — not raw observations. This removes most
   within-frame variance before the points ever reach SLAM.

3. TIER-LABELLED PUBLISHING
   Each sample is tagged NEAR (sample_x < 1.25 m) or FAR (1.25-2.50 m).
   Encoded in the label channel:
     0.00 → left NEAR     0.25 → left FAR
     0.75 → right FAR     1.00 → right NEAR
   lane_slam routes samples into the correct tier.

4. FAR-EXTRAPOLATION GUARD
   Samples are truncated at `max_observed_x + 0.05 m` on each side, so the
   fit never fabricates samples beyond where we actually see the tape.
   FAR samples are only emitted if at least MIN_FAR_OBS observations exist
   in the far band for that side — otherwise we'd just be extrapolating.

PUBLISHES
  /perception/lane_guidance_raw   Vector3
  /perception/lane_points_raw     PointCloud2 (camera frame, fitted + resampled)
  /perception/debug/lane_mask     Image (mono8)
  /perception/debug/raw_crop      Image (bgr8)
  /perception/debug/edges         Image (mono8)
  /perception/debug/bev_local     Image (bgr8)
"""

import math
import time
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import Vector3
import message_filters
from cv_bridge import CvBridge


# ============================================================================
#  IMAGE PROCESSING  (unchanged from v4.3)
# ============================================================================

def lane_mask_blue(bgr_roi):
    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
    h, _ = hsv.shape[:2]

    near_lo = np.array([ 99,  63, 133], dtype=np.uint8)
    near_hi = np.array([155, 255, 255], dtype=np.uint8)
    mask_near = cv2.inRange(hsv, near_lo, near_hi)

    far_lo = np.array([ 90,  45,  85], dtype=np.uint8)
    far_hi = np.array([158, 255, 255], dtype=np.uint8)
    mask_far = cv2.inRange(hsv, far_lo, far_hi)

    split_near = int(h * 0.40)
    split_far  = int(h * 0.60)

    combined = np.zeros_like(mask_near)
    combined[:split_near]          = mask_far[:split_near]
    combined[split_near:split_far] = cv2.bitwise_or(
        mask_near[split_near:split_far], mask_far[split_near:split_far])
    combined[split_far:]           = mask_near[split_far:]

    MIN_BLOB_PX = 40
    n_lbl, lbl_map, stats, _ = cv2.connectedComponentsWithStats(
        combined, connectivity=8)
    cleaned = np.zeros_like(combined)
    for lbl in range(1, n_lbl):
        if stats[lbl, cv2.CC_STAT_AREA] >= MIN_BLOB_PX:
            cleaned[lbl_map == lbl] = 255

    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN,  np.ones((3, 3), np.uint8))
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cleaned = cv2.dilate(cleaned, np.ones((3, 3), np.uint8), iterations=1)
    return cleaned


def mask_to_edges(mask):
    return cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, np.ones((5, 5), np.uint8))


def vectorized_xyz_lookup(pc_xyz, xs_pc, ys_pc):
    pts = pc_xyz[ys_pc, xs_pc, :3]
    valid = np.isfinite(pts).all(axis=1)
    return pts[valid]


# ============================================================================
#  VOXEL DOWNSAMPLE  (unchanged)
# ============================================================================

def voxel_mean_downsample(pts_xy, voxel_size=0.05):
    N = pts_xy.shape[0]
    if N == 0:
        return pts_xy.astype(np.float32)
    pts = pts_xy.astype(np.float32)
    keys = np.floor(pts / voxel_size).astype(np.int32)
    offset = keys.min(axis=0)
    k = keys - offset
    stride = int(k[:, 1].max()) + 1
    flat = k[:, 0] * stride + k[:, 1]
    order = np.argsort(flat, kind='stable')
    flat_s = flat[order]; pts_s = pts[order]
    breaks = np.concatenate([[0], np.flatnonzero(np.diff(flat_s)) + 1, [N]])
    n_cells = len(breaks) - 1
    out = np.empty((n_cells, 2), dtype=np.float32)
    for i in range(n_cells):
        out[i] = pts_s[breaks[i]:breaks[i+1]].mean(axis=0)
    return out


# ============================================================================
#  CURVE-AWARE LANE SPLITTER  (v5.0 — parallel-frontier BFS, full range)
# ============================================================================

def split_lanes_chain(pts_xy, seed_x_max=0.50, max_jump=0.08,
                     gap_jump=0.14, voxel_size=0.05, min_lane_sep=0.15,
                     verify_x_max=0.70):
    """
    Curve-robust lane splitter.

    Algorithm:
      1. Voxel-downsample to 0.05 m (reduces comparison count)
      2. Find two seeds in the near field via clamped two-means on Y
      3. Grow two chains with PARALLEL-FRONTIER BFS over the FULL range,
         max_jump = 0.08 m (< 1/4 lane separation)
      4. Close small gaps with a second pass at gap_jump = 0.14 m
      5. Transfer downsampled labels back to raw pts_xy via nearest
         labelled neighbour (within max_jump * 2)
      6. Orientation check: left should have Y > right in the near field;
         swap labels if not.

    Returns:
      labels : (N,) int32 array with values in {0 (left), 1 (right), -1}
    """
    N = pts_xy.shape[0]
    if N < 4:
        if N < 2:
            return np.zeros(N, dtype=np.int32)
        labels = np.where(pts_xy[:, 1] >= 0, 0, 1).astype(np.int32)
        return labels

    ds_pts = voxel_mean_downsample(pts_xy, voxel_size=voxel_size)
    ds_N = ds_pts.shape[0]
    if ds_N < 4:
        return np.where(pts_xy[:, 1] >= 0, 0, 1).astype(np.int32)

    # 1. Seeds
    seed_L, seed_R = _find_seeds(ds_pts, seed_x_max, min_lane_sep)
    if seed_L < 0 and seed_R < 0:
        # Can't even find one seed — default to Y-median split
        med = float(np.median(ds_pts[:, 1]))
        ds_labels = np.where(ds_pts[:, 1] >= med, 0, 1).astype(np.int32)
        return _transfer_labels(pts_xy, ds_pts, ds_labels, max_jump * 2.5)

    # 2. Parallel-frontier BFS
    ds_labels = np.full(ds_N, -1, dtype=np.int32)
    if seed_L >= 0: ds_labels[seed_L] = 0
    if seed_R >= 0: ds_labels[seed_R] = 1

    # Pre-compute neighbour lists at both max_jump and gap_jump
    nbrs_main = _build_neighbour_lists(ds_pts, max_jump)
    _parallel_bfs(ds_pts, ds_labels, nbrs_main)

    if (ds_labels == -1).any():
        nbrs_gap = _build_neighbour_lists(ds_pts, gap_jump)
        _parallel_bfs(ds_pts, ds_labels, nbrs_gap)

    # 3. Handle single-seed case (only one tape visible)
    if seed_L < 0 or seed_R < 0:
        assigned_lbl = 0 if seed_L >= 0 else 1
        other_lbl = 1 - assigned_lbl
        # Any still-unlabelled points → assign them to the present label too
        ds_labels[ds_labels == -1] = assigned_lbl
        # But we want NO points on the other side
        labels = _transfer_labels(pts_xy, ds_pts, ds_labels, max_jump * 2.5)
        # Force any unlabelled raw point to the present side
        labels[labels == -1] = assigned_lbl
        return labels

    # 4. Transfer to full set
    labels = _transfer_labels(pts_xy, ds_pts, ds_labels, max_jump * 2.5)

    # 5. Orientation check in near field
    near_mask = pts_xy[:, 0] < verify_x_max
    lm = (labels == 0) & near_mask
    rm = (labels == 1) & near_mask
    if lm.any() and rm.any():
        if pts_xy[lm, 1].mean() < pts_xy[rm, 1].mean():
            swap = labels.copy()
            swap[labels == 0] = 1
            swap[labels == 1] = 0
            labels = swap

    return labels


def _find_seeds(ds_pts, seed_x_max, min_lane_sep):
    """Find two seed indices via clamped two-means on Y in the near band.
       Returns (seed_L, seed_R); either can be -1 if only one tape visible."""
    near_mask = ds_pts[:, 0] < seed_x_max
    if near_mask.sum() < 2:
        # Fallback: use nearest-X 20% of points
        x_thresh = np.percentile(ds_pts[:, 0], 20)
        near_mask = ds_pts[:, 0] <= x_thresh
        if near_mask.sum() < 2:
            if near_mask.sum() == 1:
                idx = int(np.where(near_mask)[0][0])
                # Single point — place it on the side matching its Y sign
                return (idx, -1) if ds_pts[idx, 1] >= 0 else (-1, idx)
            return -1, -1

    near_idx = np.where(near_mask)[0]
    near_pts = ds_pts[near_idx]
    y_vals = near_pts[:, 1]

    spread = float(y_vals.max() - y_vals.min())
    if spread < 0.08:
        # Single-tape near field
        idx = int(near_idx[int(np.argmax(y_vals))])
        return (idx, -1) if y_vals.mean() >= 0 else (-1, idx)

    c_left = float(np.percentile(y_vals, 75))
    c_right = float(np.percentile(y_vals, 25))
    for _ in range(8):
        m = np.abs(y_vals - c_left) < np.abs(y_vals - c_right)
        if m.all() or (~m).all():
            break
        nl = float(y_vals[m].mean())
        nr = float(y_vals[~m].mean())
        if abs(nl - c_left) < 1e-4 and abs(nr - c_right) < 1e-4:
            break
        c_left, c_right = nl, nr
    if c_left < c_right:
        c_left, c_right = c_right, c_left

    if (c_left - c_right) < min_lane_sep:
        # Not a true two-tape separation — treat as single tape
        idx = int(near_idx[int(np.argmax(y_vals))])
        return (idx, -1) if y_vals.mean() >= 0 else (-1, idx)

    lm = np.abs(y_vals - c_left) < np.abs(y_vals - c_right)
    rm = ~lm
    # Seed = min-X point in each cluster
    seed_L = int(near_idx[lm][np.argmin(near_pts[lm, 0])]) if lm.any() else -1
    seed_R = int(near_idx[rm][np.argmin(near_pts[rm, 0])]) if rm.any() else -1
    return seed_L, seed_R


def _build_neighbour_lists(ds_pts, radius):
    """For each point, list indices of other points within `radius`. O(N^2)
       but N is ~100 after downsampling, so this is fast."""
    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(ds_pts)
        return tree.query_ball_tree(tree, radius)
    except ImportError:
        N = ds_pts.shape[0]
        d = np.linalg.norm(
            ds_pts[:, None, :] - ds_pts[None, :, :], axis=2)
        return [list(np.where(d[i] <= radius)[0]) for i in range(N)]


def _parallel_bfs(ds_pts, ds_labels, neighbours):
    """Grow chain 0 and chain 1 simultaneously. When both claim the same
       point in a round, assign it to whichever's source is closer."""
    qL = deque(int(i) for i in np.where(ds_labels == 0)[0])
    qR = deque(int(i) for i in np.where(ds_labels == 1)[0])
    while qL or qR:
        claim_L = {}
        claim_R = {}
        while qL:
            src = qL.popleft()
            for j in neighbours[src]:
                if ds_labels[j] == -1 and j not in claim_L:
                    claim_L[j] = src
        while qR:
            src = qR.popleft()
            for j in neighbours[src]:
                if ds_labels[j] == -1 and j not in claim_R:
                    claim_R[j] = src
        if not claim_L and not claim_R:
            break
        newL = deque(); newR = deque()
        for j in set(claim_L) | set(claim_R):
            if ds_labels[j] != -1:
                continue
            in_L = j in claim_L
            in_R = j in claim_R
            if in_L and not in_R:
                ds_labels[j] = 0; newL.append(j)
            elif in_R and not in_L:
                ds_labels[j] = 1; newR.append(j)
            else:
                dL = float(np.linalg.norm(ds_pts[j] - ds_pts[claim_L[j]]))
                dR = float(np.linalg.norm(ds_pts[j] - ds_pts[claim_R[j]]))
                if dL <= dR:
                    ds_labels[j] = 0; newL.append(j)
                else:
                    ds_labels[j] = 1; newR.append(j)
        qL = newL; qR = newR


def _transfer_labels(full_pts, ds_pts, ds_labels, max_dist):
    N = full_pts.shape[0]
    labels = np.full(N, -1, dtype=np.int32)
    try:
        from scipy.spatial import cKDTree
        labelled_mask = ds_labels >= 0
        if not labelled_mask.any():
            return labels
        tree = cKDTree(ds_pts[labelled_mask])
        lbls_l = ds_labels[labelled_mask]
        dists, idx = tree.query(full_pts, k=1)
        valid = dists <= max_dist
        labels[valid] = lbls_l[idx[valid]]
    except ImportError:
        labelled = np.where(ds_labels >= 0)[0]
        if labelled.size == 0:
            return labels
        for i in range(N):
            d = np.linalg.norm(ds_pts[labelled] - full_pts[i], axis=1)
            j = int(np.argmin(d))
            if d[j] <= max_dist:
                labels[i] = ds_labels[labelled[j]]
    return labels


# ============================================================================
#  POLYLINE FIT + ARC-LENGTH RESAMPLE
# ============================================================================

def _order_by_nn(pts):
    """Greedy nearest-neighbour ordering starting from min-X point."""
    N = pts.shape[0]
    if N <= 2:
        return pts
    start = int(np.argmin(pts[:, 0]))
    visited = np.zeros(N, dtype=bool)
    order = [start]
    visited[start] = True
    for _ in range(N - 1):
        last = pts[order[-1]]
        d = np.linalg.norm(pts - last, axis=1)
        d[visited] = 1e9
        nxt = int(np.argmin(d))
        if d[nxt] > 0.40:
            break
        order.append(nxt); visited[nxt] = True
    return pts[order]


def fit_and_resample(pts_xy, spacing=0.05, smooth_window=5,
                     min_pts=6, max_gap=0.30):
    """Fit a polyline to (unordered) pts, return Mx2 samples at `spacing`
       along arc length. None if too few points or fit is broken."""
    if pts_xy is None or pts_xy.shape[0] < min_pts:
        return None
    ordered = _order_by_nn(pts_xy.astype(np.float32))
    if ordered.shape[0] < min_pts:
        return None

    if smooth_window > 1 and ordered.shape[0] >= smooth_window:
        k = np.ones(smooth_window, dtype=np.float32) / smooth_window
        sx = np.convolve(ordered[:, 0], k, mode='valid')
        sy = np.convolve(ordered[:, 1], k, mode='valid')
        ordered = np.stack([sx, sy], axis=1)
    if ordered.shape[0] < 2:
        return None

    diffs = np.diff(ordered, axis=0)
    seg_len = np.linalg.norm(diffs, axis=1)
    # Truncate before any huge gap (disconnected component)
    if (seg_len > max_gap).any():
        cut = int(np.argmax(seg_len > max_gap)) + 1
        ordered = ordered[:cut]
        if ordered.shape[0] < 2:
            return None
        diffs = np.diff(ordered, axis=0)
        seg_len = np.linalg.norm(diffs, axis=1)

    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = float(cum[-1])
    if total < spacing:
        return ordered

    n = int(total / spacing) + 1
    ds = np.linspace(0.0, total, n)
    out = np.empty((n, 2), dtype=np.float32)
    for i, d in enumerate(ds):
        idx = int(np.searchsorted(cum, d, side='right') - 1)
        idx = max(0, min(idx, len(ordered) - 2))
        seg = seg_len[idx] if seg_len[idx] > 1e-9 else 1e-9
        t = min((d - cum[idx]) / seg, 1.0)
        out[i] = ordered[idx] * (1 - t) + ordered[idx+1] * t
    return out


# ============================================================================
#  ERROR COMPUTATION  (unchanged from v4.3)
# ============================================================================

def compute_errors_binned(left_xy, right_xy, min_pts=3,
                          x_min=0.08, x_max=2.0, n_bins=12,
                          half_width=0.25):
    bin_edges = np.linspace(x_min, x_max, n_bins + 1)
    hl = left_xy  is not None and left_xy.shape[0]  >= min_pts
    hr = right_xy is not None and right_xy.shape[0] >= min_pts
    q = (float(hl) + float(hr)) / 2.0
    if not hl and not hr:
        return 0.0, 0.0, 0.0

    cys = []; cxs = []
    for i in range(n_bins):
        lo, hi = bin_edges[i], bin_edges[i+1]; xm = 0.5*(lo+hi)
        ly = ry = None
        if hl:
            m = (left_xy[:, 0] >= lo) & (left_xy[:, 0] < hi)
            if m.sum() >= 2: ly = float(np.median(left_xy[m, 1]))
        if hr:
            m = (right_xy[:, 0] >= lo) & (right_xy[:, 0] < hi)
            if m.sum() >= 2: ry = float(np.median(right_xy[m, 1]))
        if ly is not None and ry is not None:
            cys.append(0.5*(ly+ry)); cxs.append(xm)
        elif ly is not None or ry is not None:
            tape = ly if ly is not None else ry
            off = -half_width if tape >= 0 else half_width
            cys.append(tape+off); cxs.append(xm)

    if not cys: return 0.0, 0.0, q
    cys = np.asarray(cys); cxs = np.asarray(cxs)
    w = 1.0/(cxs**2); w /= w.sum()
    lat = float(np.dot(w, cys))
    if len(cys) >= 3:
        mid = len(cys)//2
        hdg = math.atan2(cys[mid]-cys[0], cxs[mid]-cxs[0])
    elif len(cys) >= 2:
        hdg = math.atan2(cys[-1]-cys[0], cxs[-1]-cxs[0])
    else: hdg = 0.0
    return lat, hdg, q


# ============================================================================
#  BEV debug  (unchanged)
# ============================================================================

def render_bev(left_xy, right_xy, max_range=2.5,
               canvas_w=400, canvas_h=400):
    img = np.full((canvas_h, canvas_w, 3), (18, 18, 22), dtype=np.uint8)
    ppx = (canvas_h * 0.90) / max_range
    ox = canvas_w // 2; oy = int(canvas_h * 0.97)
    for d in np.arange(0.5, max_range + 0.5, 0.5):
        py = oy - int(d * ppx)
        if 0 <= py < canvas_h:
            cv2.line(img, (0, py), (canvas_w, py), (40, 40, 48), 1)
            cv2.putText(img, f'{d:.1f}m', (4, py-2),
                        cv2.FONT_HERSHEY_PLAIN, 0.7, (70, 70, 80), 1)
    cv2.circle(img, (ox, oy), 5, (200, 200, 200), -1)
    for pts, col in [(left_xy, (255, 200, 50)), (right_xy, (50, 80, 255))]:
        for pt in pts:
            px = ox - int(pt[1] * ppx); py = oy - int(pt[0] * ppx)
            if 0 <= px < canvas_w and 0 <= py < canvas_h:
                cv2.circle(img, (px, py), 2, col, -1)
    cv2.putText(img, 'L', (8,  16), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 200, 50), 1)
    cv2.putText(img, 'R', (24, 16), cv2.FONT_HERSHEY_PLAIN, 1.0, (50,  80, 255), 1)
    return img


# ============================================================================
#  PointCloud2 helper — 4-valued label channel (side × tier)
# ============================================================================

LABEL_LEFT_NEAR  = 0.00
LABEL_LEFT_FAR   = 0.25
LABEL_RIGHT_FAR  = 0.75
LABEL_RIGHT_NEAR = 1.00


def samples_to_pc2_tiered(left_samples, right_samples,
                          left_tier_far, right_tier_far,
                          header):
    """left_samples, right_samples: Mx2 arrays (x,y in camera frame).
       *_tier_far: bool arrays, True where tier is FAR.
       Returns PointCloud2 with 4-valued label channel."""
    parts = []; lbls = []
    if left_samples is not None and left_samples.shape[0] > 0:
        parts.append(left_samples)
        lbls.append(np.where(left_tier_far, LABEL_LEFT_FAR, LABEL_LEFT_NEAR).astype(np.float32))
    if right_samples is not None and right_samples.shape[0] > 0:
        parts.append(right_samples)
        lbls.append(np.where(right_tier_far, LABEL_RIGHT_FAR, LABEL_RIGHT_NEAR).astype(np.float32))
    if not parts:
        return None
    xy = np.vstack(parts)
    lbl = np.concatenate(lbls)
    xyz = np.hstack([xy, np.zeros((xy.shape[0], 1), dtype=np.float32)])
    data = np.hstack([xyz, lbl.reshape(-1, 1)]).astype(np.float32)

    msg = PointCloud2()
    msg.header = header; msg.height = 1; msg.width = data.shape[0]
    msg.fields = [
        PointField(name='x',     offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',     offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',     offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='label', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False; msg.point_step = 16
    msg.row_step = 16 * data.shape[0]; msg.data = data.tobytes(); msg.is_dense = True
    return msg


# ============================================================================
#  NODE
# ============================================================================

class LineMapperNode(Node):

    MIN_PTS     = 6
    MAX_SAMPLES = 3000
    STRIDE      = 2
    EDGE_MARGIN = 5
    ROI_FRACTION = 0.40
    MAX_RANGE    = 2.5
    MIN_RANGE    = 0.08

    # Labeling
    CHAIN_SEED_X_MAX = 0.50
    CHAIN_MAX_JUMP   = 0.08
    CHAIN_GAP_JUMP   = 0.14
    CHAIN_MIN_LANE_SEP = 0.15

    # False-split guard for mis-seeded single-tape frames
    COLLAPSE_GAP     = 0.15
    NEAR_FIELD_X     = 0.60

    # Fitting + resampling
    SAMPLE_SPACING   = 0.05
    FIT_SMOOTH_WINDOW = 9
    FIT_MIN_PTS      = 6
    MIN_FAR_OBS      = 2           # need >=2 observations beyond 1.25 m
                                   # to emit any FAR samples

    # Tier boundary
    NEAR_X_MAX       = 1.25

    # Guidance smoothing
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
        self._debug_mask_pub = self.create_publisher(
            Image, '/perception/debug/lane_mask', 10)
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

        self.get_logger().info(
            f'LineMapper v5.0 ready. '
            f'chain max_jump={self.CHAIN_MAX_JUMP}m gap={self.CHAIN_GAP_JUMP}m '
            f'spacing={self.SAMPLE_SPACING}m tier_cut={self.NEAR_X_MAX}m')

    def _cloud_to_xyz(self, cloud_msg):
        raw = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        nf = cloud_msg.point_step // 4
        pts = raw.view(np.float32).reshape(
            cloud_msg.height, cloud_msg.width, nf)
        return np.ascontiguousarray(pts[:, :, :3], dtype=np.float32)

    def _synced_callback(self, image_msg, cloud_msg):
        t0 = time.time()

        # 1. Decode
        try:
            bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}'); return

        pc_xyz = self._cloud_to_xyz(cloud_msg)
        pc_h, pc_w = pc_xyz.shape[:2]
        img_h, img_w = bgr.shape[:2]

        # 2. ROI
        y0 = int(img_h * self.ROI_FRACTION)
        roi = bgr[y0:, :]
        m = self.bridge.cv2_to_imgmsg(roi, encoding='bgr8')
        m.header = image_msg.header; self._raw_crop_pub.publish(m)

        # 3. Mask + edges
        mask = lane_mask_blue(roi)
        edges = mask_to_edges(mask)
        for img_out, pub, enc in [
            (mask,  self._debug_mask_pub, 'mono8'),
            (edges, self._edges_pub,      'mono8'),
        ]:
            m = self.bridge.cv2_to_imgmsg(img_out, encoding=enc)
            m.header = image_msg.header; pub.publish(m)

        # 4. Edge pixel coordinates
        ys_roi, xs_roi = np.where(edges > 0)
        if len(xs_roi) == 0:
            self._guidance_pub.publish(Vector3(x=0.0, y=0.0, z=0.0)); return
        if len(xs_roi) > self.MAX_SAMPLES:
            idx = np.random.choice(len(xs_roi), self.MAX_SAMPLES, replace=False)
            xs_roi, ys_roi = xs_roi[idx], ys_roi[idx]
        elif self.STRIDE > 1:
            sel = np.arange(0, len(xs_roi), self.STRIDE)
            xs_roi, ys_roi = xs_roi[sel], ys_roi[sel]

        # 5. Pixel -> 3D
        ys_img = ys_roi + y0
        keep = (
            (xs_roi >= self.EDGE_MARGIN) & (xs_roi < img_w - self.EDGE_MARGIN) &
            (ys_img >= self.EDGE_MARGIN) & (ys_img < img_h - self.EDGE_MARGIN))
        xs_img, ys_img = xs_roi[keep], ys_img[keep]
        if len(xs_img) == 0:
            self._guidance_pub.publish(Vector3(x=0.0, y=0.0, z=0.0)); return

        sx, sy = pc_w / img_w, pc_h / img_h
        xs_pc = np.clip((xs_img * sx).astype(np.int32), 0, pc_w - 1)
        ys_pc = np.clip((ys_img * sy).astype(np.int32), 0, pc_h - 1)
        pts = vectorized_xyz_lookup(pc_xyz, xs_pc, ys_pc)
        if pts.shape[0] < self.MIN_PTS:
            self._guidance_pub.publish(Vector3(x=0.0, y=0.0, z=0.0)); return

        # 6. Range + ground-plane filter
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r > self.MIN_RANGE) & (r < self.MAX_RANGE)]
        pts = pts[(pts[:, 2] > -0.30) & (pts[:, 2] < 0.15)]
        if pts.shape[0] < self.MIN_PTS:
            self._guidance_pub.publish(Vector3(x=0.0, y=0.0, z=0.0)); return

        xy = pts[:, :2]

        # 7. Curve-aware labelling over the FULL range
        labels = split_lanes_chain(
            xy,
            seed_x_max=self.CHAIN_SEED_X_MAX,
            max_jump=self.CHAIN_MAX_JUMP,
            gap_jump=self.CHAIN_GAP_JUMP,
            voxel_size=0.05,
            min_lane_sep=self.CHAIN_MIN_LANE_SEP)

        left_xy  = xy[labels == 0]
        right_xy = xy[labels == 1]

        # 8. False-split guard (as v4.3)
        have_left  = left_xy.shape[0]  >= self.MIN_PTS
        have_right = right_xy.shape[0] >= self.MIN_PTS
        if have_left and have_right:
            l_near = left_xy [left_xy [:, 0] < self.NEAR_FIELD_X]
            r_near = right_xy[right_xy[:, 0] < self.NEAR_FIELD_X]
            if l_near.shape[0] >= 3 and r_near.shape[0] >= 3:
                l_y = float(l_near[:, 1].mean())
                r_y = float(r_near[:, 1].mean())
                gap = abs(l_y - r_y)
                same_side = (l_y * r_y) > 0
                if gap < self.COLLAPSE_GAP or same_side:
                    all_xy = np.vstack([left_xy, right_xy])
                    med_y = float(np.median(all_xy[:, 1]))
                    if med_y >= 0:
                        left_xy = all_xy; right_xy = np.empty((0, 2), np.float32)
                    else:
                        right_xy = all_xy; left_xy = np.empty((0, 2), np.float32)
                    have_left  = left_xy.shape[0]  >= self.MIN_PTS
                    have_right = right_xy.shape[0] >= self.MIN_PTS
                    self.get_logger().warn(
                        f'[SINGLE-TAPE] collapsed gap={gap:.3f} '
                        f'same_side={same_side} → '
                        f'{"LEFT" if med_y>=0 else "RIGHT"}')

        # 9. Fit + resample each side, tier-tag by sample X
        left_samples,  left_tier_far  = self._fit_and_tier(left_xy,  have_left)
        right_samples, right_tier_far = self._fit_and_tier(right_xy, have_right)

        # 10. Publish tier-labelled samples for SLAM
        pc = samples_to_pc2_tiered(
            left_samples,  right_samples,
            left_tier_far, right_tier_far,
            image_msg.header)
        if pc is not None:
            self._points_pub.publish(pc)

        # 11. Local BEV (for debug visualizer)
        bev = render_bev(
            left_samples  if left_samples  is not None else np.empty((0, 2), np.float32),
            right_samples if right_samples is not None else np.empty((0, 2), np.float32),
            max_range=self.MAX_RANGE)
        bm = self.bridge.cv2_to_imgmsg(bev, encoding='bgr8')
        bm.header = image_msg.header
        self._bev_local_pub.publish(bm)

        # 12. Guidance errors (from samples — already smoothed)
        lat, hdg, q = compute_errors_binned(
            left_samples  if left_samples  is not None else None,
            right_samples if right_samples is not None else None,
            min_pts=3, x_min=self.MIN_RANGE, x_max=self.MAX_RANGE, n_bins=12)
        if q > 0:
            self._smooth_lat = self.ALPHA_LAT*lat + (1-self.ALPHA_LAT)*self._smooth_lat
            self._smooth_hdg = self.ALPHA_HDG*hdg + (1-self.ALPHA_HDG)*self._smooth_hdg
        g = Vector3()
        g.x = float(self._smooth_lat); g.y = float(self._smooth_hdg); g.z = float(q)
        self._guidance_pub.publish(g)

        dt = time.time() - t0
        mode = ('LR' if (have_left and have_right) else
                'L'  if have_left else
                'R'  if have_right else '-')
        nls = left_samples.shape[0]  if left_samples  is not None else 0
        nrs = right_samples.shape[0] if right_samples is not None else 0
        nlf = int(left_tier_far.sum())  if left_tier_far  is not None and left_tier_far.size else 0
        nrf = int(right_tier_far.sum()) if right_tier_far is not None and right_tier_far.size else 0
        self.get_logger().info(
            f'[{mode}] lat={self._smooth_lat:+.3f}m '
            f'hdg={math.degrees(self._smooth_hdg):+.1f}° q={q:.1f} '
            f'raw={pts.shape[0]} Lsamp={nls}({nlf}far) Rsamp={nrs}({nrf}far) '
            f'{1.0/max(dt,1e-6):.0f}Hz')

    def _fit_and_tier(self, side_xy, have_side):
        """Fit polyline, resample at 0.05 m, tier-tag, truncate to observed.
           Returns (samples Nx2 or None, tier_far bool N or empty)."""
        if not have_side or side_xy is None or side_xy.shape[0] < self.FIT_MIN_PTS:
            return None, np.empty(0, dtype=bool)

        max_obs_x = float(side_xy[:, 0].max())
        n_far_obs = int(np.sum(side_xy[:, 0] >= self.NEAR_X_MAX))

        samples = fit_and_resample(
            side_xy,
            spacing=self.SAMPLE_SPACING,
            smooth_window=self.FIT_SMOOTH_WINDOW,
            min_pts=self.FIT_MIN_PTS,
            max_gap=0.30)
        if samples is None or samples.shape[0] == 0:
            return None, np.empty(0, dtype=bool)

        # Truncate: no samples beyond what we actually observed (+ tiny buffer)
        in_range = samples[:, 0] <= (max_obs_x + 0.05)
        samples = samples[in_range]
        if samples.shape[0] == 0:
            return None, np.empty(0, dtype=bool)

        # Drop FAR samples if we don't have enough FAR observations
        tier_far = samples[:, 0] >= self.NEAR_X_MAX
        if n_far_obs < self.MIN_FAR_OBS and tier_far.any():
            samples = samples[~tier_far]
            tier_far = tier_far[~tier_far]   # empty
        return samples, tier_far


def main(args=None):
    rclpy.init(args=args)
    node = LineMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()


# ============================================================================
#  PARAMETER LOOKUP TABLE
# ============================================================================
#
# Parameter              File         Default    Description
# ─────────────────────────────────────────────────────────────────────────────
# ROI_FRACTION           line_mapper  0.40       Top fraction of image to skip.
# MIN_RANGE              line_mapper  0.08 m     Closest 3D distance kept.
# MAX_RANGE              line_mapper  2.5 m      Furthest 3D distance kept.
# CHAIN_SEED_X_MAX       line_mapper  0.50 m     Near band for seeding BFS.
# CHAIN_MAX_JUMP         line_mapper  0.08 m     BFS hop radius. < lane_sep/4.
# CHAIN_GAP_JUMP         line_mapper  0.14 m     Gap-bridge pass. < lane_sep/3.
# CHAIN_MIN_LANE_SEP     line_mapper  0.15 m     Min Y gap for two-tape detect.
# COLLAPSE_GAP           line_mapper  0.15 m     Same-side fuse threshold.
# NEAR_FIELD_X           line_mapper  0.60 m     False-split check zone.
# SAMPLE_SPACING         line_mapper  0.05 m     Arc-length resample step.
# FIT_SMOOTH_WINDOW      line_mapper  5          Moving-average window.
# FIT_MIN_PTS            line_mapper  6          Min labelled pts to fit.
# MIN_FAR_OBS            line_mapper  2          Min far observations to emit
#                                                any FAR samples.
# NEAR_X_MAX             line_mapper  1.25 m     NEAR/FAR tier boundary.
# ALPHA_LAT              line_mapper  0.80       Lateral smoothing.
# ALPHA_HDG              line_mapper  0.75       Heading smoothing.
# ─────────────────────────────────────────────────────────────────────────────

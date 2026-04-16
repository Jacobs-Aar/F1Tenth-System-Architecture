#!/usr/bin/env python3
"""
vision_perception/lane_slam.py — Lane SLAM node (v6.3).

CHANGES FROM v6.2
=================
RACING-LINE PREDICTIVE TURNING
  Compute signed curvature κ(s) along the car-frame centerline and evaluate
  at two preview distances:
      κ_near at X = RACING_NEAR_X  (current curvature)
      κ_far  at X = RACING_FAR_X   (upcoming curvature)

  Then offset the pursuit target laterally:
      offset_y = NEAR_GAIN * κ_near  -  FAR_GAIN * κ_far
      offset_y clipped to ±RACING_MAX_OFFSET

  Result across turn phases:
    straight           → offset ≈ 0         (centerline)
    approach           → offset negative    (outside setup — κ_far > 0)
    apex               → offset positive    (inside apex  — κ_near dominates)
    exit               → offset positive    (hold inside until κ_near drops)

  Applied equally to map-based and live-fallback pursuit targets. Fires as
  soon as any centerline is available, so it works during lap-1 mapping —
  which is when the car most needs the extra visibility of the outside line
  to SEE into the turn while approaching.

  Racing line is applied to pursuit_target only. Controller/PD path is
  unchanged.

CHANGES FROM v6.1 (retained)
  - Live-observation pursuit fallback when map has no usable pairing
  - 1 Hz majority-vote map cleanup

LABEL CHANNEL (from line_mapper v5.0):
  0.00 = left NEAR   0.25 = left FAR   0.75 = right FAR   1.00 = right NEAR
"""

import json
import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, PoseStamped
from std_msgs.msg import String, Header


# ============================================================================
#  GEOMETRY
# ============================================================================

def _quat_to_R(qx, qy, qz, qw):
    return np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),       1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)


def pose_to_T(px, py, pz, qx, qy, qz, qw):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _quat_to_R(qx, qy, qz, qw)
    T[:3, 3] = [px, py, pz]
    return T


def transform_pts(pts, T):
    if pts is None or pts.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float32)
    h = np.hstack([pts.astype(np.float64), np.ones((len(pts), 1))])
    return (T @ h.T).T[:, :3].astype(np.float32)


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))


def _lerp(a, b, t): return a + (b - a) * t


# ============================================================================
#  ODOM BUFFER
# ============================================================================

class OdomBuffer:
    def __init__(self, max_size=400):
        self._buf = deque(maxlen=max_size)

    def add(self, t_sec, px, py, pz, qx, qy, qz, qw):
        if self._buf and t_sec < self._buf[-1][0]:
            return
        self._buf.append((t_sec, px, py, pz, qx, qy, qz, qw))

    def is_ready(self):
        return len(self._buf) >= 2

    @property
    def latest(self):
        return self._buf[-1] if self._buf else None

    def lookup(self, t_sec):
        if not self._buf: return None
        buf = self._buf
        if t_sec <= buf[0][0]:  return self._entry_to_pose(buf[0])
        if t_sec >= buf[-1][0]: return self._entry_to_pose(buf[-1])
        for i in range(len(buf) - 1):
            if buf[i][0] <= t_sec <= buf[i+1][0]:
                lo = buf[i]; hi = buf[i+1]
                dt = hi[0] - lo[0]
                a = max(0.0, min(1.0, (t_sec - lo[0])/dt if dt > 1e-9 else 0.0))
                px = _lerp(lo[1], hi[1], a); py = _lerp(lo[2], hi[2], a); pz = _lerp(lo[3], hi[3], a)
                qx = _lerp(lo[4], hi[4], a); qy = _lerp(lo[5], hi[5], a)
                qz = _lerp(lo[6], hi[6], a); qw = _lerp(lo[7], hi[7], a)
                n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
                if n > 1e-9: qx/=n; qy/=n; qz/=n; qw/=n
                T = pose_to_T(px, py, pz, qx, qy, qz, qw)
                return T, np.array([px, py]), yaw_from_quat(qx, qy, qz, qw)
        return self._entry_to_pose(buf[-1])

    @staticmethod
    def _entry_to_pose(e):
        T = pose_to_T(e[1], e[2], e[3], e[4], e[5], e[6], e[7])
        return T, np.array([e[1], e[2]]), yaw_from_quat(e[4], e[5], e[6], e[7])


# ============================================================================
#  POLYLINE SAMPLE MAP
# ============================================================================

TIER_NEAR = 0
TIER_FAR  = 1


class _SideMap:
    CELL = 0.10
    NEAR_OVERWRITE_R = 0.05
    NEAR_MERGE_R     = 0.03
    FAR_MERGE_R      = 0.05
    MAX_NEAR_COUNT   = 20
    MAX_SAMPLES      = 6000

    def __init__(self):
        self._xs = []; self._ys = []
        self._tier = []; self._count = []
        self._t_last = []
        self._alive = []
        self._grid = {}
        self._n_alive = 0

    def _key(self, x, y):
        return (int(math.floor(x/self.CELL)), int(math.floor(y/self.CELL)))

    def _neighbor_indices(self, x, y, radius):
        r = int(math.ceil(radius/self.CELL))
        kx, ky = self._key(x, y)
        out = []
        for dx in range(-r, r+1):
            for dy in range(-r, r+1):
                cell = self._grid.get((kx+dx, ky+dy))
                if cell: out.extend(cell)
        rr = radius*radius
        live = []
        for i in out:
            if not self._alive[i]: continue
            ddx = self._xs[i]-x; ddy = self._ys[i]-y
            if ddx*ddx + ddy*ddy <= rr:
                live.append(i)
        return live

    def _remove(self, i):
        if not self._alive[i]: return
        self._alive[i] = False
        self._n_alive -= 1

    def insert(self, x, y, tier, t_now):
        if self._n_alive >= self.MAX_SAMPLES:
            return False
        if tier == TIER_NEAR:
            for i in self._neighbor_indices(x, y, self.NEAR_OVERWRITE_R):
                if self._tier[i] == TIER_FAR:
                    self._remove(i)
            best = -1; best_d2 = 1e9
            for i in self._neighbor_indices(x, y, self.NEAR_MERGE_R):
                if self._tier[i] != TIER_NEAR: continue
                ddx = self._xs[i]-x; ddy = self._ys[i]-y
                d2 = ddx*ddx + ddy*ddy
                if d2 < best_d2:
                    best_d2 = d2; best = i
            if best >= 0:
                c = self._count[best]
                new_c = min(c+1, self.MAX_NEAR_COUNT)
                self._xs[best] = (self._xs[best]*c + x)/(c+1)
                self._ys[best] = (self._ys[best]*c + y)/(c+1)
                self._count[best] = new_c
                self._t_last[best] = t_now
                return True
            self._direct_add(x, y, TIER_NEAR, 1, t_now)
            return True
        # FAR insertion
        for i in self._neighbor_indices(x, y, self.NEAR_OVERWRITE_R):
            if self._tier[i] == TIER_NEAR:
                return False
        best = -1; best_d2 = 1e9
        for i in self._neighbor_indices(x, y, self.FAR_MERGE_R):
            if self._tier[i] != TIER_FAR: continue
            ddx = self._xs[i]-x; ddy = self._ys[i]-y
            d2 = ddx*ddx + ddy*ddy
            if d2 < best_d2:
                best_d2 = d2; best = i
        if best >= 0:
            c = self._count[best]
            self._xs[best] = (self._xs[best]*c + x)/(c+1)
            self._ys[best] = (self._ys[best]*c + y)/(c+1)
            self._count[best] = c+1
            self._t_last[best] = t_now
            return True
        self._direct_add(x, y, TIER_FAR, 1, t_now)
        return True

    def _direct_add(self, x, y, tier, count, t_last):
        idx = len(self._xs)
        self._xs.append(float(x)); self._ys.append(float(y))
        self._tier.append(int(tier)); self._count.append(int(count))
        self._t_last.append(float(t_last)); self._alive.append(True)
        self._grid.setdefault(self._key(x, y), []).append(idx)
        self._n_alive += 1

    def _snapshot(self):
        out = []
        for i in range(len(self._xs)):
            if self._alive[i]:
                out.append((self._xs[i], self._ys[i],
                            self._tier[i], self._count[i], self._t_last[i]))
        return out

    def _clear(self):
        self._xs.clear(); self._ys.clear()
        self._tier.clear(); self._count.clear()
        self._t_last.clear(); self._alive.clear()
        self._grid.clear()
        self._n_alive = 0

    def decay_far(self, t_now, max_age_s):
        n_evicted = 0
        cutoff = t_now - max_age_s
        for i in range(len(self._xs)):
            if not self._alive[i]: continue
            if self._tier[i] == TIER_FAR and self._t_last[i] < cutoff:
                self._remove(i)
                n_evicted += 1
        return n_evicted

    def all_points(self):
        if self._n_alive == 0:
            return np.empty((0, 2), dtype=np.float32), np.empty(0, dtype=np.int32)
        pts = []; tiers = []
        for i in range(len(self._xs)):
            if self._alive[i]:
                pts.append((self._xs[i], self._ys[i]))
                tiers.append(self._tier[i])
        return np.asarray(pts, dtype=np.float32), np.asarray(tiers, dtype=np.int32)

    def near_pts(self, car_xy, radius):
        r2 = radius*radius
        cx = float(car_xy[0]); cy = float(car_xy[1])
        out = []
        for i in range(len(self._xs)):
            if not self._alive[i]: continue
            ddx = self._xs[i]-cx; ddy = self._ys[i]-cy
            if ddx*ddx + ddy*ddy < r2:
                out.append((self._xs[i], self._ys[i]))
        return np.asarray(out, dtype=np.float32) if out else None

    def size(self):
        return self._n_alive


class PolylineSampleMap:
    def __init__(self):
        self._left = _SideMap()
        self._right = _SideMap()
        self._locked = False

    def lock(self): self._locked = True
    def is_locked(self): return self._locked

    def insert(self, x, y, side, tier, t_now):
        if self._locked: return
        (self._left if side == 'left' else self._right).insert(x, y, tier, t_now)

    def decay_far(self, t_now, max_age_s):
        if self._locked:
            return 0, 0
        return (self._left.decay_far(t_now, max_age_s),
                self._right.decay_far(t_now, max_age_s))

    def size(self):
        return self._left.size(), self._right.size()

    def is_populated(self, thresh=30):
        l, r = self.size()
        return l >= thresh and r >= thresh

    def get_all(self):
        return self._left.all_points(), self._right.all_points()

    def near_pts(self, car_xy, radius):
        return self._left.near_pts(car_xy, radius), self._right.near_pts(car_xy, radius)

    def cleanup_by_majority(self, radius=0.10, min_neighbors=4, strict_margin=2,
                             min_total=30):
        left_info = self._left._snapshot()
        right_info = self._right._snapshot()
        n_l = len(left_info); n_r = len(right_info)
        if n_l + n_r < min_total:
            return 0, 0
        all_info = left_info + right_info
        all_xy = np.array([(s[0], s[1]) for s in all_info], dtype=np.float32)
        all_side = np.concatenate([np.zeros(n_l, dtype=np.int32),
                                    np.ones(n_r, dtype=np.int32)])
        new_side = all_side.copy()
        try:
            from scipy.spatial import cKDTree
            tree = cKDTree(all_xy)
            for i in range(len(all_info)):
                idxs = tree.query_ball_point(all_xy[i], radius)
                if len(idxs) <= 1: continue
                nbrs = [j for j in idxs if j != i]
                if len(nbrs) < min_neighbors: continue
                nbr_sides = all_side[nbrs]
                own = all_side[i]
                other_count = int((nbr_sides == (1 - own)).sum())
                own_count = int((nbr_sides == own).sum())
                if other_count >= own_count + strict_margin:
                    new_side[i] = 1 - own
        except ImportError:
            for i in range(len(all_info)):
                d = np.linalg.norm(all_xy - all_xy[i], axis=1)
                mask = (d > 1e-6) & (d < radius)
                if mask.sum() < min_neighbors: continue
                nbrs = all_side[mask]
                own = all_side[i]
                other_count = int((nbrs == (1 - own)).sum())
                own_count = int((nbrs == own).sum())
                if other_count >= own_count + strict_margin:
                    new_side[i] = 1 - own
        if np.array_equal(new_side, all_side):
            return 0, 0
        flips_to_left  = int(((all_side == 1) & (new_side == 0)).sum())
        flips_to_right = int(((all_side == 0) & (new_side == 1)).sum())
        self._left._clear()
        self._right._clear()
        for i, (x, y, tier, count, t_last) in enumerate(all_info):
            target = self._left if new_side[i] == 0 else self._right
            target._direct_add(x, y, tier, count, t_last)
        return flips_to_left, flips_to_right


# ============================================================================
#  CENTERLINE + PURSUIT HELPERS
# ============================================================================

def _order_by_nn(pts):
    N = pts.shape[0]
    if N <= 2: return pts
    start = int(np.argmin(pts[:, 0]))
    visited = np.zeros(N, dtype=bool)
    order = [start]; visited[start] = True
    for _ in range(N-1):
        last = pts[order[-1]]
        d = np.linalg.norm(pts - last, axis=1)
        d[visited] = 1e9
        nxt = int(np.argmin(d))
        if d[nxt] > 0.5: break
        order.append(nxt); visited[nxt] = True
    return pts[order]


def _resample_arc(pts_xy, spacing=0.10, smooth_w=3, max_gap=0.80):
    if pts_xy is None or pts_xy.shape[0] < 2: return pts_xy
    ordered = _order_by_nn(pts_xy.astype(np.float32))
    if ordered.shape[0] < 2: return ordered
    if smooth_w > 1 and ordered.shape[0] >= smooth_w:
        k = np.ones(smooth_w, dtype=np.float32)/smooth_w
        sx = np.convolve(ordered[:, 0], k, mode='valid')
        sy = np.convolve(ordered[:, 1], k, mode='valid')
        ordered = np.stack([sx, sy], axis=1)
    if ordered.shape[0] < 2: return ordered
    diffs = np.diff(ordered, axis=0)
    seg = np.linalg.norm(diffs, axis=1)
    if (seg > max_gap).any():
        cut = int(np.argmax(seg > max_gap)) + 1
        ordered = ordered[:cut]
        if ordered.shape[0] < 2: return ordered
        diffs = np.diff(ordered, axis=0); seg = np.linalg.norm(diffs, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1])
    if total < spacing: return ordered
    n = int(total/spacing)+1
    ds = np.linspace(0, total, n)
    out = np.empty((n, 2), dtype=np.float32)
    for i, d in enumerate(ds):
        idx = int(np.searchsorted(cum, d, side='right')-1)
        idx = max(0, min(idx, len(ordered)-2))
        s = seg[idx] if seg[idx] > 1e-9 else 1e-9
        t = min((d - cum[idx])/s, 1.0)
        out[i] = ordered[idx]*(1-t) + ordered[idx+1]*t
    return out


def _pick_target_on_polyline(polyline, lookahead):
    """Given Nx2 car-frame points (X=forward), pick the point at `lookahead`
       metres of arc length from the forward-most start."""
    fwd = polyline[:, 0] > -0.05
    if fwd.sum() < 2: return None
    pts = polyline[fwd]
    order = np.argsort(pts[:, 0])
    pts = pts[order]
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1])
    if total < 0.10: return None
    s_tgt = min(lookahead, total)
    i = int(np.searchsorted(cum, s_tgt, side='right') - 1)
    i = max(0, min(i, len(pts) - 2))
    t = 0.0 if seg[i] < 1e-6 else (s_tgt - cum[i]) / seg[i]
    t = max(0.0, min(1.0, t))
    p = pts[i] * (1 - t) + pts[i+1] * t
    return float(p[0]), float(p[1])


def extract_centerline_world(left_all_xy, right_all_xy, car_xy, car_yaw,
                              max_pair_dist=0.70, min_pair_dist=0.15,
                              max_ahead=2.5, max_behind=0.3):
    """World-frame centerline from L/R map samples."""
    if left_all_xy is None or right_all_xy is None: return None
    if left_all_xy.shape[0] == 0 or right_all_xy.shape[0] == 0: return None
    c = math.cos(car_yaw); s = math.sin(car_yaw)
    def _rel(pts):
        dx = pts[:, 0]-car_xy[0]; dy = pts[:, 1]-car_xy[1]
        return dx*c + dy*s, -dx*s + dy*c
    lrx, lry = _rel(left_all_xy); rrx, rry = _rel(right_all_xy)
    lmask = (lrx > -max_behind) & (lrx < max_ahead)
    rmask = (rrx > -max_behind) & (rrx < max_ahead)
    if lmask.sum() < 2 or rmask.sum() < 2: return None
    L = np.stack([lrx[lmask], lry[lmask]], axis=1)
    R = np.stack([rrx[rmask], rry[rmask]], axis=1)
    centers = []
    max_pair2 = max_pair_dist**2
    for i in range(L.shape[0]):
        d2 = ((R - L[i])**2).sum(axis=1)
        j = int(np.argmin(d2))
        if d2[j] > max_pair2: continue
        gap = math.sqrt(d2[j])
        if gap < min_pair_dist: continue
        centers.append((L[i]+R[j])*0.5)
    if len(centers) < 2: return None
    C = np.asarray(centers, dtype=np.float32)
    order = np.argsort(C[:, 0])
    C = C[order]
    wx = car_xy[0] + C[:, 0]*c - C[:, 1]*s
    wy = car_xy[1] + C[:, 0]*s + C[:, 1]*c
    world = np.stack([wx, wy], axis=1).astype(np.float32)
    return _resample_arc(world, spacing=0.10, smooth_w=3, max_gap=0.80)


def world_to_car_frame(polyline_world, car_xy, car_yaw):
    """Transform Nx2 world-frame polyline to car frame (X=forward, Y=left)."""
    if polyline_world is None or polyline_world.shape[0] == 0:
        return None
    c = math.cos(car_yaw); s = math.sin(car_yaw)
    dx = polyline_world[:, 0] - car_xy[0]
    dy = polyline_world[:, 1] - car_xy[1]
    rx = dx*c + dy*s
    ry = -dx*s + dy*c
    return np.stack([rx, ry], axis=1).astype(np.float32)


def build_live_centerline_car(left_cam_xy, right_cam_xy, half_track_width=0.25,
                                max_pair_dist=0.70, min_pair_dist=0.15):
    """Centerline in car frame (= camera frame) built from live observations.
       Dual-tape → pair-based midpoints. Single-tape → lateral offset."""
    have_l = left_cam_xy  is not None and left_cam_xy.shape[0]  >= 2
    have_r = right_cam_xy is not None and right_cam_xy.shape[0] >= 2
    if not have_l and not have_r: return None
    if have_l and have_r:
        max_p2 = max_pair_dist**2
        centers = []
        for i in range(left_cam_xy.shape[0]):
            d2 = ((right_cam_xy - left_cam_xy[i])**2).sum(axis=1)
            j = int(np.argmin(d2))
            if d2[j] > max_p2: continue
            gap = math.sqrt(d2[j])
            if gap < min_pair_dist: continue
            centers.append((left_cam_xy[i] + right_cam_xy[j]) * 0.5)
        if len(centers) < 2: return None
        C = np.asarray(centers, dtype=np.float32)
        order = np.argsort(C[:, 0]); C = C[order]
    elif have_l:
        order = np.argsort(left_cam_xy[:, 0])
        C = left_cam_xy[order].copy()
        C[:, 1] -= half_track_width
    else:
        order = np.argsort(right_cam_xy[:, 0])
        C = right_cam_xy[order].copy()
        C[:, 1] += half_track_width
    return _resample_arc(C, spacing=0.10, smooth_w=3, max_gap=0.80)


# ============================================================================
#  RACING LINE  (curvature-based target offset)
# ============================================================================

def compute_centerline_curvatures(polyline_car):
    """Signed curvature at each interior point via 3-point osculating circle.
       κ > 0 = left turn. Boundary values copied from nearest interior."""
    n = polyline_car.shape[0]
    if n < 3:
        return np.zeros(n, dtype=np.float32)
    kappa = np.zeros(n, dtype=np.float32)
    for i in range(1, n - 1):
        p0 = polyline_car[i-1]; p1 = polyline_car[i]; p2 = polyline_car[i+1]
        v1 = p1 - p0; v2 = p2 - p1
        cross = float(v1[0]*v2[1] - v1[1]*v2[0])
        a = float(np.linalg.norm(v1))
        b = float(np.linalg.norm(v2))
        c = float(np.linalg.norm(p2 - p0))
        denom = a * b * c
        if denom > 1e-6:
            kappa[i] = 2.0 * cross / denom
    kappa[0] = kappa[1]
    kappa[-1] = kappa[-2]
    # Smooth with [0.25, 0.5, 0.25] to tame per-segment noise
    if n >= 3:
        k_pad = np.concatenate([[kappa[0]], kappa, [kappa[-1]]])
        kappa = (0.25*k_pad[:-2] + 0.50*k_pad[1:-1] + 0.25*k_pad[2:]).astype(np.float32)
    return kappa


def sample_kappa_at_x(polyline_car, kappa_arr, target_x):
    """Linear-interpolate κ at target_x along car-frame polyline."""
    n = polyline_car.shape[0]
    if n < 2: return 0.0
    xs = polyline_car[:, 0]
    if target_x <= xs[0]:  return float(kappa_arr[0])
    if target_x >= xs[-1]: return float(kappa_arr[-1])
    idx = int(np.searchsorted(xs, target_x))
    idx = max(1, min(idx, n - 1))
    x_lo = float(xs[idx-1]); x_hi = float(xs[idx])
    k_lo = float(kappa_arr[idx-1]); k_hi = float(kappa_arr[idx])
    if x_hi - x_lo < 1e-6: return k_lo
    t = (target_x - x_lo) / (x_hi - x_lo)
    return k_lo + t * (k_hi - k_lo)


def compute_racing_offset(polyline_car, near_x, far_x,
                           near_gain, far_gain, max_offset):
    """Racing-line lateral offset for pursuit target (car frame Y).
       Returns (offset_y, κ_near, κ_far)."""
    if polyline_car is None or polyline_car.shape[0] < 3:
        return 0.0, 0.0, 0.0
    kappa = compute_centerline_curvatures(polyline_car)
    kn = sample_kappa_at_x(polyline_car, kappa, near_x)
    kf = sample_kappa_at_x(polyline_car, kappa, far_x)
    offset = near_gain * kn - far_gain * kf
    offset = max(-max_offset, min(max_offset, offset))
    return float(offset), float(kn), float(kf)


# ============================================================================
#  ERROR COMPUTATION (legacy PD path)
# ============================================================================

def compute_errors_binned(left_xy, right_xy, min_pts=3,
                          x_min=0.08, x_max=2.5, n_bins=12,
                          half_width=0.25, lat_near_x=0.90, max_hdg_x=None):
    bin_edges = np.linspace(x_min, x_max, n_bins+1)
    hl = left_xy is not None and left_xy.shape[0] >= min_pts
    hr = right_xy is not None and right_xy.shape[0] >= min_pts
    q = (float(hl)+float(hr))/2.0
    if not hl and not hr: return 0.0, 0.0, 0.0
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
    if lat_near_x is not None:
        m = cxs <= lat_near_x
        if m.sum() >= 1:
            w = 1.0/(cxs[m]**2); w /= w.sum()
            lat = float(np.dot(w, cys[m]))
        else:
            w = 1.0/(cxs**2); w /= w.sum(); lat = float(np.dot(w, cys))
    else:
        w = 1.0/(cxs**2); w /= w.sum(); lat = float(np.dot(w, cys))
    if max_hdg_x is not None:
        m = cxs <= max_hdg_x
        hcys = cys[m] if m.sum() >= 2 else cys
        hcxs = cxs[m] if m.sum() >= 2 else cxs
    else: hcys, hcxs = cys, cxs
    if len(hcys) >= 3:
        mid = len(hcys)//2
        hdg = math.atan2(hcys[mid]-hcys[0], hcxs[mid]-hcxs[0])
    elif len(hcys) >= 2:
        hdg = math.atan2(hcys[-1]-hcys[0], hcxs[-1]-hcxs[0])
    else: hdg = 0.0
    return lat, hdg, q


# ============================================================================
#  PointCloud2 helpers
# ============================================================================

def make_labeled_pc2(left_2d, right_2d, header, tiers_left=None, tiers_right=None):
    parts = []; lbls = []
    if left_2d is not None and left_2d.shape[0] > 0:
        parts.append(left_2d)
        if tiers_left is not None and tiers_left.size:
            lbls.append(np.where(tiers_left == TIER_NEAR, 0.0, 0.25).astype(np.float32))
        else:
            lbls.append(np.zeros(left_2d.shape[0], dtype=np.float32))
    if right_2d is not None and right_2d.shape[0] > 0:
        parts.append(right_2d)
        if tiers_right is not None and tiers_right.size:
            lbls.append(np.where(tiers_right == TIER_NEAR, 1.0, 0.75).astype(np.float32))
        else:
            lbls.append(np.ones(right_2d.shape[0], dtype=np.float32))
    if not parts: return None
    p2 = np.vstack(parts); lbl = np.concatenate(lbls)
    p3 = np.hstack([p2, np.zeros((p2.shape[0], 1), dtype=np.float32)])
    data = np.hstack([p3, lbl.reshape(-1, 1)]).astype(np.float32)
    msg = PointCloud2(); msg.header = header; msg.height = 1; msg.width = data.shape[0]
    msg.fields = [
        PointField(name='x',     offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',     offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',     offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='label', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False; msg.point_step = 16
    msg.row_step = 16*data.shape[0]; msg.data = data.tobytes(); msg.is_dense = True
    return msg


def make_simple_pc2(pts_xy, header):
    if pts_xy is None or pts_xy.shape[0] == 0: return None
    p3 = np.hstack([pts_xy.astype(np.float32),
                    np.zeros((pts_xy.shape[0], 1), dtype=np.float32)])
    msg = PointCloud2(); msg.header = header; msg.height = 1; msg.width = p3.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False; msg.point_step = 12
    msg.row_step = 12*p3.shape[0]; msg.data = p3.tobytes(); msg.is_dense = True
    return msg


# ============================================================================
#  NODE
# ============================================================================

class LaneSlamNode(Node):

    MIN_PTS             = 4
    MIN_RANGE           = 0.08
    MAX_RANGE           = 2.5
    RECALL_RADIUS       = 4.0

    BLIND_RANGE         = 0.40
    LIVE_RELIABLE_RANGE = 1.40
    BLEND_RATIO         = 0.5
    HALF_TRACK_WIDTH    = 0.25
    LAT_NEAR_X          = 0.90

    ALPHA_LAT_LIVE = 0.80; ALPHA_LAT_MAP = 0.50
    ALPHA_HDG_LIVE = 0.75; ALPHA_HDG_MAP = 0.45
    ALPHA_PURSUIT  = 0.60

    LAP_MIN_DIST       = 2.0
    LAP_CLOSE_RADIUS   = 0.40

    LOOKAHEAD              = 0.60
    PURSUIT_MIN_CENTER_PTS = 3

    FAR_MAX_AGE_S = 10.0

    # Cleanup
    CLEANUP_PERIOD_S   = 1.0
    CLEANUP_RADIUS     = 0.10
    CLEANUP_MIN_NBRS   = 4
    CLEANUP_MARGIN     = 2

    # Racing line (NEW in v6.3)
    RACING_LINE_ENABLED = True
    RACING_NEAR_X       = 0.50   # m — κ sample for "current turn" detection
    RACING_FAR_X        = 1.50   # m — κ sample for "upcoming turn" detection
    RACING_NEAR_GAIN    = 0.20   # m² — apex gain (toward inside of current turn)
    RACING_FAR_GAIN     = 0.10   # m² — setup gain (toward outside of upcoming)
    RACING_MAX_OFFSET   = 0.15   # m — clip to stay within lane (< HALF_TRACK_WIDTH)

    def __init__(self):
        super().__init__('lane_slam')

        self._odom = OdomBuffer(max_size=400)
        self._map = PolylineSampleMap()
        self._have_anchor = False
        self._start_xy = None
        self._cum_dist = 0.0
        self._last_pose_for_dist = None

        self._smooth_lat = 0.0; self._smooth_hdg = 0.0
        self._pursuit_target = None
        self._pursuit_src = 'none'
        self._last_racing_offset = 0.0
        self._last_kappa_near = 0.0
        self._last_kappa_far = 0.0
        self._using_pose_topic = False

        self._latest_raw = Vector3(x=0.0, y=0.0, z=0.0)

        self._guidance_pub = self.create_publisher(Vector3, '/perception/lane_guidance', 10)
        self._pursuit_pub  = self.create_publisher(Vector3, '/perception/pursuit_target', 10)
        self._map_pub      = self.create_publisher(PointCloud2, '/slam/lane_map', 5)
        self._center_pub   = self.create_publisher(PointCloud2, '/slam/centerline', 5)
        self._diag_pub     = self.create_publisher(String, '/slam/diagnostics', 5)

        self.create_subscription(PoseStamped, '/zed/zed_node/pose', self._pose_cb, 20)
        self.create_subscription(Odometry, '/zed/zed_node/odom', self._odom_cb, 20)
        self.create_subscription(PointCloud2, '/perception/lane_points_raw', self._points_cb, 10)
        self.create_subscription(Vector3, '/perception/lane_guidance_raw', self._raw_guidance_cb, 10)

        self.create_timer(0.5, self._periodic_cb)
        self.create_timer(self.CLEANUP_PERIOD_S, self._cleanup_cb)

        self.get_logger().info(
            f'LaneSlam v6.3 ready | racing-line ENABLED '
            f'(near_x={self.RACING_NEAR_X}m Kn_gain={self.RACING_NEAR_GAIN} '
            f'far_x={self.RACING_FAR_X}m Kf_gain={self.RACING_FAR_GAIN} '
            f'max_offset={self.RACING_MAX_OFFSET}m)')

    # ── pose ───────────────────────────────────────────────────────────
    def _add_pose(self, stamp, p, o):
        t_sec = stamp.sec + stamp.nanosec*1e-9
        self._odom.add(t_sec, p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        cur = np.array([p.x, p.y])
        if self._start_xy is None:
            self._start_xy = cur.copy(); self._last_pose_for_dist = cur.copy()
        else:
            d = float(np.linalg.norm(cur - self._last_pose_for_dist))
            if d > 0.02:
                self._cum_dist += d
                self._last_pose_for_dist = cur.copy()
            if (not self._map.is_locked()
                    and self._cum_dist > self.LAP_MIN_DIST
                    and self._map.is_populated()):
                if float(np.linalg.norm(cur - self._start_xy)) < self.LAP_CLOSE_RADIUS:
                    self._map.lock()
                    self.get_logger().warn(
                        f'[LAP] MAP LOCKED after {self._cum_dist:.2f} m')

    def _pose_cb(self, msg):
        self._using_pose_topic = True
        self._add_pose(msg.header.stamp, msg.pose.position, msg.pose.orientation)

    def _odom_cb(self, msg):
        if not self._using_pose_topic:
            self._add_pose(msg.header.stamp,
                           msg.pose.pose.position, msg.pose.pose.orientation)

    def _raw_guidance_cb(self, msg):
        self._latest_raw = msg
        if not self._odom.is_ready():
            self._guidance_pub.publish(msg)

    # ── main ──────────────────────────────────────────────────────────
    def _points_cb(self, msg):
        if not self._odom.is_ready():
            self._guidance_pub.publish(self._latest_raw); return

        pc_t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        pose_at = self._odom.lookup(pc_t)
        if pose_at is None:
            self._guidance_pub.publish(self._latest_raw); return
        T_cap, _, _ = pose_at

        pose_now = self._odom.latest
        T_now = pose_to_T(pose_now[1], pose_now[2], pose_now[3],
                          pose_now[4], pose_now[5], pose_now[6], pose_now[7])
        try: T_map_to_cam = np.linalg.inv(T_now)
        except np.linalg.LinAlgError: T_map_to_cam = np.eye(4)
        car_xy_now = np.array([pose_now[1], pose_now[2]])
        car_yaw_now = yaw_from_quat(pose_now[4], pose_now[5], pose_now[6], pose_now[7])

        nf = msg.point_step // 4
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, nf)
        if raw.shape[0] == 0:
            self._guidance_pub.publish(self._latest_raw); return
        xyz = raw[:, :3]
        labels = raw[:, 3] if nf >= 4 else np.zeros(raw.shape[0])

        left_near_m  = labels < 0.125
        left_far_m   = (labels >= 0.125) & (labels < 0.50)
        right_far_m  = (labels >= 0.50)  & (labels < 0.875)
        right_near_m = labels >= 0.875

        t_now = self.get_clock().now().nanoseconds * 1e-9

        def _ingest(mask, side, tier):
            if not mask.any(): return False
            pts_cam = xyz[mask]
            world = transform_pts(pts_cam, T_cap)[:, :2]
            for wx, wy in world:
                self._map.insert(wx, wy, side, tier, t_now)
            return True

        dual = left_near_m.any() and right_near_m.any()
        single = (left_near_m.any() or right_near_m.any()) and not dual

        if not self._map.is_locked():
            if dual:
                self._have_anchor = True
                _ingest(left_near_m,  'left',  TIER_NEAR)
                _ingest(left_far_m,   'left',  TIER_FAR)
                _ingest(right_near_m, 'right', TIER_NEAR)
                _ingest(right_far_m,  'right', TIER_FAR)
            elif single and self._have_anchor:
                if left_near_m.any() or left_far_m.any():
                    side = self._infer_side(xyz[left_near_m | left_far_m], T_cap)
                    if side is not None:
                        _ingest(left_near_m, side, TIER_NEAR)
                        _ingest(left_far_m,  side, TIER_FAR)
                if right_near_m.any() or right_far_m.any():
                    side = self._infer_side(xyz[right_near_m | right_far_m], T_cap)
                    if side is not None:
                        _ingest(right_near_m, side, TIER_NEAR)
                        _ingest(right_far_m,  side, TIER_FAR)

        left_cam_all  = xyz[left_near_m  | left_far_m]
        right_cam_all = xyz[right_near_m | right_far_m]

        self._publish_legacy_guidance(left_cam_all, right_cam_all,
                                       left_cam_all.shape[0] >= self.MIN_PTS,
                                       right_cam_all.shape[0] >= self.MIN_PTS,
                                       T_map_to_cam, car_xy_now)

        self._publish_pursuit_target(
            car_xy_now, car_yaw_now,
            left_cam_all[:, :2]  if left_cam_all.shape[0]  >= 2 else None,
            right_cam_all[:, :2] if right_cam_all.shape[0] >= 2 else None)

    def _infer_side(self, pts_cam_3d, T_world_from_cam):
        if pts_cam_3d.shape[0] == 0: return None
        centroid_cam = pts_cam_3d.mean(axis=0).reshape(1, 3)
        centroid_world = transform_pts(centroid_cam, T_world_from_cam)[0, :2]
        (lp, _), (rp, _) = self._map.get_all()
        dl = float(np.min(np.linalg.norm(lp - centroid_world, axis=1))) if lp.size else 1e9
        dr = float(np.min(np.linalg.norm(rp - centroid_world, axis=1))) if rp.size else 1e9
        if dl > 1.0 and dr > 1.0: return None
        return 'left' if dl <= dr else 'right'

    # ── legacy PD guidance (unchanged) ───────────────────────────────
    def _publish_legacy_guidance(self, left_cam, right_cam, live_l_ok, live_r_ok,
                                  T_map_to_cam, car_xy_now):
        l_world, r_world = self._map.near_pts(car_xy_now, self.RECALL_RADIUS)
        l_mc = self._world_xy_to_cam(l_world, T_map_to_cam)
        r_mc = self._world_xy_to_cam(r_world, T_map_to_cam)
        l_mc = self._range_filter(l_mc); r_mc = self._range_filter(r_mc)
        map_ready = self._map.is_populated()

        def _zone(p, x_lo, x_hi):
            if p is None or p.shape[0] == 0:
                return np.empty((0, 3), np.float32)
            m = (p[:, 0] >= max(x_lo, self.MIN_RANGE)) & (p[:, 0] < x_hi)
            return p[m]

        def _assemble(live, map_c):
            parts = []
            z1 = _zone(map_c, self.MIN_RANGE, self.BLIND_RANGE)
            if z1.shape[0] > 0: parts.append(z1)
            z2l = _zone(live,  self.BLIND_RANGE, self.LIVE_RELIABLE_RANGE)
            z2m = _zone(map_c, self.BLIND_RANGE, self.LIVE_RELIABLE_RANGE)
            if z2l.shape[0] > 0:
                parts.append(z2l)
                if z2m.shape[0] > 0:
                    nb = min(z2m.shape[0], max(int(z2l.shape[0]*self.BLEND_RATIO), 10))
                    idx = np.linspace(0, z2m.shape[0]-1, nb, dtype=int)
                    parts.append(z2m[idx])
            elif z2m.shape[0] >= self.MIN_PTS: parts.append(z2m)
            z3m = _zone(map_c, self.LIVE_RELIABLE_RANGE, self.MAX_RANGE)
            z3l = _zone(live,  self.LIVE_RELIABLE_RANGE, self.MAX_RANGE)
            if map_ready and z3m.shape[0] > 0: parts.append(z3m)
            elif z3l.shape[0] > 0: parts.append(z3l)
            if not parts: return None
            r = np.vstack(parts)
            return r if r.shape[0] >= self.MIN_PTS else None

        def _assemble_map_only(map_c):
            parts = []
            for lo, hi in [(self.MIN_RANGE, self.BLIND_RANGE),
                           (self.BLIND_RANGE, self.LIVE_RELIABLE_RANGE),
                           (self.LIVE_RELIABLE_RANGE, self.MAX_RANGE)]:
                z = _zone(map_c, lo, hi)
                if z.shape[0] > 0: parts.append(z)
            if not parts: return None
            r = np.vstack(parts)
            return r if r.shape[0] >= self.MIN_PTS else None

        merged_l = _assemble(left_cam,  l_mc) if live_l_ok else _assemble_map_only(l_mc)
        merged_r = _assemble(right_cam, r_mc) if live_r_ok else _assemble_map_only(r_mc)
        have_l = merged_l is not None; have_r = merged_r is not None
        if not have_l and not have_r:
            self._guidance_pub.publish(self._latest_raw); return

        lat, hdg, q = compute_errors_binned(
            merged_l[:, :2] if have_l else None,
            merged_r[:, :2] if have_r else None,
            min_pts=3, x_min=self.MIN_RANGE, x_max=self.MAX_RANGE, n_bins=12,
            half_width=self.HALF_TRACK_WIDTH,
            lat_near_x=self.LAT_NEAR_X,
            max_hdg_x=self.LIVE_RELIABLE_RANGE if not map_ready else None)

        if q > 0:
            al = self.ALPHA_LAT_MAP if map_ready else self.ALPHA_LAT_LIVE
            ah = self.ALPHA_HDG_MAP if map_ready else self.ALPHA_HDG_LIVE
            self._smooth_lat = al*lat + (1-al)*self._smooth_lat
            self._smooth_hdg = ah*hdg + (1-ah)*self._smooth_hdg

        g = Vector3()
        g.x = float(self._smooth_lat); g.y = float(self._smooth_hdg); g.z = float(q)
        self._guidance_pub.publish(g)

        state = 'LOCK' if self._map.is_locked() else ('MAP+' if map_ready else 'LAP1')
        anchor = 'A' if self._have_anchor else '-'
        nl, nr = self._map.size()
        self.get_logger().info(
            f'[{state}|{anchor}|P={self._pursuit_src}|R={self._last_racing_offset:+.2f}m '
            f'κn={self._last_kappa_near:+.2f} κf={self._last_kappa_far:+.2f}] '
            f'lat={self._smooth_lat:+.3f}m hdg={math.degrees(self._smooth_hdg):+.1f}° '
            f'q={q:.1f} {nl}L/{nr}R dist={self._cum_dist:.2f}m')

    def _world_xy_to_cam(self, world_xy, T_map_to_cam):
        if world_xy is None or world_xy.shape[0] == 0: return None
        w3 = np.hstack([world_xy, np.zeros((world_xy.shape[0], 1), dtype=np.float32)])
        return transform_pts(w3, T_map_to_cam)

    def _range_filter(self, cam):
        if cam is None or cam.shape[0] == 0: return None
        r = np.linalg.norm(cam[:, :2], axis=1)
        mask = (cam[:, 0] > self.MIN_RANGE) & (r < self.MAX_RANGE)
        out = cam[mask]
        return out if out.shape[0] > 0 else None

    # ── pursuit target (map → live fallback + racing line) ──────────
    def _publish_pursuit_target(self, car_xy_now, car_yaw_now,
                                 live_left_xy, live_right_xy):
        centerline_car = None
        source = 'none'
        quality = 0.0

        # Strategy 1: map
        (lp, _), (rp, _) = self._map.get_all()
        if lp.shape[0] >= 3 and rp.shape[0] >= 3:
            center_world = extract_centerline_world(
                lp, rp, car_xy_now, car_yaw_now,
                max_pair_dist=0.70, min_pair_dist=0.15,
                max_ahead=2.5, max_behind=0.3)
            if center_world is not None and center_world.shape[0] >= self.PURSUIT_MIN_CENTER_PTS:
                centerline_car = world_to_car_frame(center_world, car_xy_now, car_yaw_now)
                source = 'map'
                quality = 1.0
                # Publish world-frame centerline for viz
                h = Header(); h.stamp = self.get_clock().now().to_msg(); h.frame_id = 'map'
                pc = make_simple_pc2(center_world, h)
                if pc is not None:
                    self._center_pub.publish(pc)

        # Strategy 2: live observations
        if centerline_car is None:
            centerline_car = build_live_centerline_car(
                live_left_xy, live_right_xy,
                half_track_width=self.HALF_TRACK_WIDTH,
                max_pair_dist=0.70, min_pair_dist=0.15)
            if centerline_car is not None:
                source = 'live'
                quality = 0.6

        if centerline_car is None:
            self._pursuit_target = None
            self._pursuit_src = 'none'
            self._last_racing_offset = 0.0
            self._last_kappa_near = 0.0
            self._last_kappa_far = 0.0
            self._pursuit_pub.publish(Vector3(x=0.0, y=0.0, z=0.0))
            return

        # Pick target at lookahead along centerline
        target = _pick_target_on_polyline(centerline_car, self.LOOKAHEAD)
        if target is None:
            self._pursuit_target = None
            self._pursuit_src = 'none'
            self._last_racing_offset = 0.0
            self._last_kappa_near = 0.0
            self._last_kappa_far = 0.0
            self._pursuit_pub.publish(Vector3(x=0.0, y=0.0, z=0.0))
            return

        # Racing-line offset
        if self.RACING_LINE_ENABLED:
            r_off, kn, kf = compute_racing_offset(
                centerline_car,
                near_x=self.RACING_NEAR_X,
                far_x=self.RACING_FAR_X,
                near_gain=self.RACING_NEAR_GAIN,
                far_gain=self.RACING_FAR_GAIN,
                max_offset=self.RACING_MAX_OFFSET)
        else:
            r_off, kn, kf = 0.0, 0.0, 0.0

        tx, ty = target
        ty_adj = ty + r_off

        # Smooth
        if self._pursuit_target is None:
            self._pursuit_target = (tx, ty_adj)
        else:
            a = self.ALPHA_PURSUIT
            self._pursuit_target = (a*tx + (1-a)*self._pursuit_target[0],
                                    a*ty_adj + (1-a)*self._pursuit_target[1])
        self._pursuit_src = source
        self._last_racing_offset = r_off
        self._last_kappa_near = kn
        self._last_kappa_far = kf

        out = Vector3()
        out.x = float(self._pursuit_target[0])
        out.y = float(self._pursuit_target[1])
        out.z = float(quality)
        self._pursuit_pub.publish(out)

    # ── periodic: FAR decay + map viz ─────────────────────────────────
    def _periodic_cb(self):
        t_now = self.get_clock().now().nanoseconds * 1e-9
        ev_l, ev_r = self._map.decay_far(t_now, self.FAR_MAX_AGE_S)
        if ev_l or ev_r:
            self.get_logger().info(f'[DECAY] evicted FAR: {ev_l}L {ev_r}R')

        h = Header(); h.stamp = self.get_clock().now().to_msg(); h.frame_id = 'map'
        (lp, lt), (rp, rt) = self._map.get_all()
        pc = make_labeled_pc2(
            lp if lp.size else None,
            rp if rp.size else None,
            h,
            tiers_left=lt if lt.size else None,
            tiers_right=rt if rt.size else None)
        if pc is not None: self._map_pub.publish(pc)
        nl, nr = self._map.size()
        diag = {
            'cells_left': nl, 'cells_right': nr,
            'anchor': self._have_anchor, 'locked': self._map.is_locked(),
            'cum_dist': float(self._cum_dist),
            'smooth_lat': float(self._smooth_lat),
            'smooth_hdg_deg': float(math.degrees(self._smooth_hdg)),
            'pursuit_tx': float(self._pursuit_target[0]) if self._pursuit_target else None,
            'pursuit_ty': float(self._pursuit_target[1]) if self._pursuit_target else None,
            'pursuit_src': self._pursuit_src,
            'racing_offset': float(self._last_racing_offset),
            'kappa_near': float(self._last_kappa_near),
            'kappa_far':  float(self._last_kappa_far),
            'pose_src': 'pose' if self._using_pose_topic else 'odom',
        }
        self._diag_pub.publish(String(data=json.dumps(diag)))

    # ── periodic: majority-vote cleanup ───────────────────────────────
    def _cleanup_cb(self):
        fl, fr = self._map.cleanup_by_majority(
            radius=self.CLEANUP_RADIUS,
            min_neighbors=self.CLEANUP_MIN_NBRS,
            strict_margin=self.CLEANUP_MARGIN,
            min_total=30)
        if fl or fr:
            self.get_logger().info(
                f'[CLEANUP] flipped {fl}→L {fr}→R '
                f'(radius={self.CLEANUP_RADIUS}m)')


def main(args=None):
    rclpy.init(args=args)
    node = LaneSlamNode()
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

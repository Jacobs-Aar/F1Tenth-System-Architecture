#!/usr/bin/env python3
"""
vision_perception/slam_visualizer.py — SLAM diagnostic BEV (v4.1).

FIX FROM v4
  v4 subscribed to /perception/lane_points_raw and built its own local
  VoxelLaneMap — storing ALL points at ALL ranges. This meant the bev_map
  visualization showed the full noisy far-field spread even after lane_slam
  was updated to only store close-range (<1.2m) observations.

  v4.1 subscribes to /slam/lane_map instead. This is the PointCloud2 that
  lane_slam publishes from its own range-gated VoxelLaneMap — only clean
  close-range observations. The visualization now accurately shows exactly
  what the car has in memory, nothing more.

  No local map accumulation needed: lane_slam publishes its full map every
  0.5s, so we just render whatever the latest publish contains.

PUBLISHES
  /perception/debug/bev_map    Image bgr8   (scrolling world-frame BEV)
  /perception/debug/slam_diag  Image bgr8   (time strip: lat/hdg/quality)
"""

import json
import math
from collections import deque
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge


# ── Geometry helpers ──────────────────────────────────────────────────────────

def _quat_to_R(qx, qy, qz, qw):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)

def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz))


# ============================================================================
#  BEV RENDERER
# ============================================================================

class BevRenderer:
    """
    Scrolling bird's-eye view centred on the car, world-frame coordinates.
    Car forward direction always points up on screen.
    """

    COL_BG        = (18,  18,  22)
    COL_GRID      = (35,  35,  42)
    COL_LEFT      = (255, 200, 50)    # amber — matches bev_local
    COL_RIGHT     = (50,  80, 255)    # blue  — matches bev_local
    COL_TRAIL     = (60, 160, 180)
    COL_CAR       = (220, 220, 220)
    COL_GUIDANCE  = (0,   255, 255)
    COL_TEXT      = (160, 160, 160)

    def __init__(self, size: int = 500, view_m: float = 10.0):
        self.size  = size
        self.ppx   = size / view_m
        self.cx    = size // 2
        self.cy    = size // 2

    def _w2px(self, wx, wy, car_x, car_y, cos_y, sin_y):
        dx = wx - car_x; dy = wy - car_y
        rx =  cos_y*dx + sin_y*dy
        ry = -sin_y*dx + cos_y*dy
        return int(self.cx + rx*self.ppx), int(self.cy - ry*self.ppx)

    def _in(self, px, py):
        return 0 <= px < self.size and 0 <= py < self.size

    def render(self, car_xy, car_yaw,
               left_world,   # Nx2 float32 or None
               right_world,  # Nx2 float32 or None
               odom_trail,
               guidance_lat, guidance_hdg, quality,
               diagnostics=None):

        img   = np.full((self.size, self.size, 3), self.COL_BG, np.uint8)
        cos_y = math.cos(-car_yaw)
        sin_y = math.sin(-car_yaw)
        cx, cy = float(car_xy[0]), float(car_xy[1])

        # ── Grid (0.5 m cells) ───────────────────────────────────────────────
        view_m = self.size / self.ppx
        half   = int(view_m / 2 / 0.5) + 2
        for i in range(-half, half+1):
            d = i * 0.5
            for (ax0, ay0), (ax1, ay1) in [
                ((cx-view_m, cy+d), (cx+view_m, cy+d)),
                ((cx+d, cy-view_m), (cx+d, cy+view_m)),
            ]:
                p0 = self._w2px(ax0, ay0, cx, cy, cos_y, sin_y)
                p1 = self._w2px(ax1, ay1, cx, cy, cos_y, sin_y)
                cv2.line(img, p0, p1, self.COL_GRID, 1)

        # ── Range rings ──────────────────────────────────────────────────────
        for r in [1.0, 2.0, 3.0, 5.0]:
            rpx = int(r * self.ppx)
            if rpx < self.size:
                cv2.circle(img, (self.cx, self.cy), rpx, (40,40,52), 1)
                cv2.putText(img, f'{r:.0f}m',
                            (self.cx+rpx+2, self.cy-3),
                            cv2.FONT_HERSHEY_PLAIN, 0.7, (60,60,72), 1)

        # ── Odom trail ────────────────────────────────────────────────────────
        for i, (ox, oy) in enumerate(odom_trail):
            alpha = i / max(len(odom_trail), 1)
            px, py = self._w2px(ox, oy, cx, cy, cos_y, sin_y)
            if self._in(px, py):
                c = tuple(int(v*alpha) for v in self.COL_TRAIL)
                cv2.circle(img, (px, py), 1, c, -1)

        # ── Lane points ───────────────────────────────────────────────────────
        for pts, col in [(left_world, self.COL_LEFT), (right_world, self.COL_RIGHT)]:
            if pts is None: continue
            for pt in pts:
                px, py = self._w2px(pt[0], pt[1], cx, cy, cos_y, sin_y)
                if self._in(px, py):
                    cv2.circle(img, (px, py), 2, col, -1)

        # ── Guidance arrow ────────────────────────────────────────────────────
        if abs(guidance_lat) > 0.005 or abs(guidance_hdg) > 0.005:
            arm   = 1.5
            tf    = arm * math.cos(guidance_hdg)
            tl    = arm * math.sin(guidance_hdg) + guidance_lat
            tip_x = cx + tf*math.cos(car_yaw) - tl*math.sin(car_yaw)
            tip_y = cy + tf*math.sin(car_yaw) + tl*math.cos(car_yaw)
            tx, ty = self._w2px(tip_x, tip_y, cx, cy, cos_y, sin_y)
            cv2.arrowedLine(img, (self.cx, self.cy), (tx, ty),
                            self.COL_GUIDANCE, 2, tipLength=0.25)

        # ── Car triangle ──────────────────────────────────────────────────────
        tri = np.array([
            [self.cx,     self.cy-14],
            [self.cx-7,   self.cy+7],
            [self.cx+7,   self.cy+7],
        ], np.int32)
        cv2.fillPoly(img, [tri], self.COL_CAR)

        # ── HUD ──────────────────────────────────────────────────────────────
        y = 14
        cv2.putText(img, 'SLAM BEV v4.1', (5, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200,200,200), 1); y += 16

        qf  = int(80*quality)
        qcl = (0,200,0) if quality >= 0.5 else (30,80,255)
        cv2.rectangle(img, (5,y-8), (85,y+4), (50,50,50), -1)
        cv2.rectangle(img, (5,y-8), (5+qf,y+4), qcl, -1)
        cv2.putText(img, f'Q={quality:.1f}', (90,y+3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, self.COL_TEXT, 1); y += 16

        cv2.putText(img, f'Lat:{guidance_lat:+.3f}m',    (5,y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, self.COL_TEXT, 1); y += 14
        cv2.putText(img, f'Hdg:{math.degrees(guidance_hdg):+.1f}°', (5,y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, self.COL_TEXT, 1); y += 14

        if diagnostics:
            nl = diagnostics.get('map_cells_left',  0)
            nr = diagnostics.get('map_cells_right', 0)
            mr = diagnostics.get('map_ready', False)
            src = diagnostics.get('pose_source', '?')
            store_r = diagnostics.get('map_store_range', '?')
            cv2.putText(img, f'Cells:{nl}L/{nr}R',
                        (5,y), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (100,255,100), 1); y += 12
            lap_col = (0,220,0) if mr else (200,200,0)
            cv2.putText(img, f'{"MAP READY" if mr else "LAP 1 - building map"}',
                        (5,y), cv2.FONT_HERSHEY_SIMPLEX, 0.28, lap_col, 1); y += 12
            src_col = (0,220,0) if src == 'map' else (30,80,255)
            cv2.putText(img, f'Pose:{src}  Store:<{store_r}m',
                        (5,y), cv2.FONT_HERSHEY_SIMPLEX, 0.26, src_col, 1)

        bh = self.size - 10
        cv2.putText(img, 'amber=L  blue=R  (stored map only)',
                    (5,bh), cv2.FONT_HERSHEY_SIMPLEX, 0.26, (70,70,70), 1)

        return img


# ============================================================================
#  DIAGNOSTIC TIME STRIP
# ============================================================================

class DiagStrip:
    def __init__(self, w=500, h=120, hist=200):
        self.w=w; self.h=h; self.hist=hist
        self._lat=deque(maxlen=hist); self._hdg=deque(maxlen=hist)
        self._q=deque(maxlen=hist)

    def push(self, lat, hdg, q):
        self._lat.append(lat); self._hdg.append(hdg); self._q.append(q)

    def render(self):
        img = np.full((self.h, self.w, 3), (18,18,22), np.uint8)
        mid = self.h//2
        cv2.line(img, (0,mid), (self.w,mid), (40,40,40), 1)
        n = len(self._lat)
        if n < 2: return img
        lat=list(self._lat); hdg=list(self._hdg); q=list(self._q)
        sl=(mid-10)/0.3; sh=(mid-10)/0.5
        for i in range(1,n):
            x0=int((i-1)/self.hist*self.w); x1=int(i/self.hist*self.w)
            y0l=np.clip(int(mid-lat[i-1]*sl),0,self.h-1)
            y1l=np.clip(int(mid-lat[i]*sl),  0,self.h-1)
            cv2.line(img,(x0,y0l),(x1,y1l),(200,200,0),1)
            y0h=np.clip(int(mid-hdg[i-1]*sh),0,self.h-1)
            y1h=np.clip(int(mid-hdg[i]*sh),  0,self.h-1)
            cv2.line(img,(x0,y0h),(x1,y1h),(200,0,200),1)
            qh=int(q[i]*8)
            cv2.line(img,(x1,self.h-1),(x1,self.h-1-qh),
                     (0,150,0) if q[i]>=0.5 else (0,80,200),1)
        cv2.putText(img,'yellow=lat  magenta=hdg  bar=quality',
                    (5,12),cv2.FONT_HERSHEY_SIMPLEX,0.28,(110,110,110),1)
        return img


# ============================================================================
#  ROS 2 NODE
# ============================================================================

class SlamVisualizerNode(Node):

    def __init__(self):
        super().__init__('slam_visualizer')
        self.bridge = CvBridge()

        self._bev   = BevRenderer(size=500, view_m=10.0)
        self._strip = DiagStrip(w=500, h=120, hist=200)

        # Pose
        self._car_xy        = np.zeros(2)
        self._car_yaw       = 0.0
        self._have_pose     = False
        self._using_map_src = False
        self._odom_trail    = deque(maxlen=1000)

        # Map points — received directly from lane_slam's /slam/lane_map.
        # This is the range-gated map (only X < MAP_STORE_RANGE observations),
        # so what you see here is exactly what the car has stored in memory.
        self._left_world:  Optional[np.ndarray] = None
        self._right_world: Optional[np.ndarray] = None

        # Guidance + diagnostics
        self._guidance    = Vector3()
        self._diagnostics = {}

        # ── Publishers ─────────────────────────────────────────────────────
        self._bev_pub  = self.create_publisher(Image, '/perception/debug/bev_map',   5)
        self._diag_pub = self.create_publisher(Image, '/perception/debug/slam_diag', 5)

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(PoseStamped, '/zed/zed_node/pose', self._pose_cb,  20)
        self.create_subscription(Odometry,    '/zed/zed_node/odom', self._odom_cb,  20)

        # KEY CHANGE from v4: subscribe to /slam/lane_map (lane_slam's
        # range-gated output) instead of /perception/lane_points_raw.
        # This ensures the visualization shows exactly what lane_slam stores —
        # no independent local accumulation that could diverge from reality.
        self.create_subscription(PointCloud2, '/slam/lane_map',
                                 self._map_cb, 5)

        self.create_subscription(Vector3, '/perception/lane_guidance',
                                 self._guidance_cb, 10)
        self.create_subscription(String, '/slam/diagnostics',
                                 self._slam_diag_cb, 5)

        self.create_timer(0.25, self._render_tick)
        self.get_logger().info(
            'SlamVisualizer v4.1 ready. '
            'Rendering from /slam/lane_map (range-gated, matches car memory).')

    # ── Pose ─────────────────────────────────────────────────────────────────

    def _update_pose(self, px, py, pz, qx, qy, qz, qw):
        self._car_xy  = np.array([px, py])
        self._car_yaw = math.atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz))
        self._have_pose = True
        self._odom_trail.append((px, py))

    def _pose_cb(self, msg: PoseStamped):
        self._using_map_src = True
        p=msg.pose.position; o=msg.pose.orientation
        self._update_pose(p.x,p.y,p.z,o.x,o.y,o.z,o.w)

    def _odom_cb(self, msg: Odometry):
        if not self._using_map_src:
            p=msg.pose.pose.position; o=msg.pose.pose.orientation
            self._update_pose(p.x,p.y,p.z,o.x,o.y,o.z,o.w)

    # ── Map from lane_slam ────────────────────────────────────────────────────

    def _map_cb(self, msg: PointCloud2):
        """
        Receive lane_slam's published map. This already contains only the
        range-gated points (X < MAP_STORE_RANGE at time of observation).
        We just unpack and store — no local filtering or accumulation.
        """
        nf  = msg.point_step // 4
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, nf)
        if raw.shape[0] == 0:
            self._left_world = None; self._right_world = None; return

        pts    = raw[:, :2]  # XY world frame (Z=0 for flat track)
        labels = raw[:, 3] if nf >= 4 else np.zeros(raw.shape[0])

        left  = pts[labels  < 0.5]
        right = pts[labels >= 0.5]
        self._left_world  = left  if left.shape[0]  > 0 else None
        self._right_world = right if right.shape[0] > 0 else None

    def _guidance_cb(self, msg: Vector3):
        self._guidance = msg
        self._strip.push(msg.x, msg.y, msg.z)

    def _slam_diag_cb(self, msg: String):
        try: self._diagnostics = json.loads(msg.data)
        except Exception: pass

    # ── Render ────────────────────────────────────────────────────────────────

    def _render_tick(self):
        bev_img = self._bev.render(
            self._car_xy, self._car_yaw,
            self._left_world, self._right_world,
            list(self._odom_trail),
            self._guidance.x, self._guidance.y, self._guidance.z,
            self._diagnostics)

        bm = self.bridge.cv2_to_imgmsg(bev_img, encoding='bgr8')
        bm.header.stamp = self.get_clock().now().to_msg()
        self._bev_pub.publish(bm)

        sm = self.bridge.cv2_to_imgmsg(self._strip.render(), encoding='bgr8')
        sm.header.stamp = bm.header.stamp
        self._diag_pub.publish(sm)


# ============================================================================
#  ENTRY POINT
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = SlamVisualizerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()

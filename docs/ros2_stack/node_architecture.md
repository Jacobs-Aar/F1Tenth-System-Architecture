# ROS 2 Node Architecture

## Node Graph

The stack consists of 5 nodes launched by a single launch file:

```
ros2 launch f1tenth_bringup f1tenth_stack.launch.py
```

### Node 1: zed_wrapper/zed_node (Third-Party)

**Source:** Installed via the Stereolabs ZED SDK base Docker image.

**Publishes:**
- `/zed/zed_node/rgb/image_rect_color` — `sensor_msgs/Image` (BGR8, 1080p @ 15-30fps)
- `/zed/zed_node/point_cloud/cloud_registered` — `sensor_msgs/PointCloud2` (XYZRGB, registered to the colour image)

**Configuration in launch file:**
- `camera_model: zed2i`
- `depth_mode: NEURAL` — uses the ZED's neural depth engine for best quality
- `point_cloud_freq: 15.0` — Hz, balances quality with Jetson compute budget
- `coordinate_system: RIGHT_HANDED_Z_UP_X_FWD` — matches ROS convention

### Node 2: vision_perception/line_mapper (Custom Python)

**Source:** `src/vision_perception/vision_perception/line_mapper.py`

**Subscribes to:**
- `/zed/zed_node/rgb/image_rect_color`
- `/zed/zed_node/point_cloud/cloud_registered`

Uses `message_filters.ApproximateTimeSynchronizer` to fuse the image and point cloud.

**Publishes:**
- `/perception/lane_guidance` — `geometry_msgs/Vector3`
  - `.x` = lateral error (metres, + means steer left)
  - `.y` = heading error (radians, + means steer left)
  - `.z` = detection quality (0.0 = no lanes, 0.5 = one lane, 1.0 = both)
- `/perception/debug/lane_mask` — `sensor_msgs/Image` (mono8, for RViz debugging)

**Processing Pipeline:**
1. Crop lower half of image (road area)
2. HSV colour mask for blue tape
3. Morphological edge extraction
4. Subsample edge pixels, lift to 3D via point cloud
5. Distance + floor filter
6. K-means split into left/right lanes
7. Robust line fitting per lane
8. Compute lateral + heading error from fitted lines

### Node 3: control_planning/kart_controller (Custom C++)

**Source:** `src/control_planning/src/kart_controller.cpp`

**Subscribes to:**
- `/perception/lane_guidance` — lane errors from vision
- `/emergency_stop` — `std_msgs/Bool` for safety kill

**Publishes:**
- `/drive` — `ackermann_msgs/AckermannDriveStamped` (steering angle + speed)

**Control Algorithm:** PD controller on lateral error + P controller on heading error.
All gains are tunable via `vesc_params.yaml`.

### Node 4: vesc_ackermann/ackermann_to_vesc_node (Third-Party)

**Source:** Installed via `ros-humble-vesc-ackermann` apt package.

**Subscribes to:**
- `/drive` — AckermannDriveStamped

**Publishes:**
- `/commands/motor/speed` — `std_msgs/Float64` (ERPM)
- `/commands/servo/position` — `std_msgs/Float64` (0.0 - 1.0)

**Parameters:** `speed_to_erpm_gain`, `steering_angle_to_servo_gain/offset` from `vesc_params.yaml`.

### Node 5: vesc_driver/vesc_driver_node (Third-Party)

**Source:** Installed via `ros-humble-vesc-driver` apt package.

**Subscribes to:**
- `/commands/motor/speed`
- `/commands/servo/position`

**Publishes (feedback):**
- `/sensors/core` — VESC telemetry (voltage, current, ERPM, etc.)

**Parameters:** `port: /dev/ttyACM0` and safety limits from `vesc_params.yaml`.

## Topic Summary

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/zed/.../image_rect_color` | Image | zed_node | line_mapper |
| `/zed/.../cloud_registered` | PointCloud2 | zed_node | line_mapper |
| `/perception/lane_guidance` | Vector3 | line_mapper | kart_controller |
| `/perception/debug/lane_mask` | Image | line_mapper | (RViz) |
| `/emergency_stop` | Bool | (external) | kart_controller |
| `/drive` | AckermannDriveStamped | kart_controller | ackermann_to_vesc |
| `/commands/motor/speed` | Float64 | ackermann_to_vesc | vesc_driver |
| `/commands/servo/position` | Float64 | ackermann_to_vesc | vesc_driver |

## Debugging Commands

```bash
# List all active nodes
ros2 node list

# List all active topics
ros2 topic list

# See what a topic is publishing (live)
ros2 topic echo /perception/lane_guidance

# Check publish rate
ros2 topic hz /perception/lane_guidance

# View the node graph visually (requires GUI forwarding)
rqt_graph
```

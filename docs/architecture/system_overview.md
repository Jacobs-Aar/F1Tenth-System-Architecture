# System Architecture Overview

## Design Philosophy

The entire autonomous stack runs inside a single Docker container on an NVIDIA Jetson.
This makes the system portable — the exact same container can be used on the F1Tenth car
today and migrated to the full-scale go-kart later by simply copying it to a new Jetson.

All custom logic lives in two ROS 2 packages (`vision_perception` and `control_planning`).
Everything else — the ZED camera driver, VESC motor driver, and Ackermann translator —
are installed from standard apt packages and do not need modification.

## Data Flow

```
ZED 2i Camera (USB 3.0)
    |
    v
[zed_wrapper]  (third-party ROS 2 node)
    |--- /zed/zed_node/rgb/image_rect_color     (sensor_msgs/Image)
    |--- /zed/zed_node/point_cloud/cloud_registered (sensor_msgs/PointCloud2)
    |
    v
[vision_perception / line_mapper]  (custom Python node)
    |--- /perception/lane_guidance  (geometry_msgs/Vector3)
    |         .x = lateral_error_m
    |         .y = heading_error_rad
    |         .z = detection_quality (0.0 - 1.0)
    |
    |--- /perception/debug/lane_mask  (sensor_msgs/Image, mono8)
    |
    v
[control_planning / kart_controller]  (custom C++ node)
    |--- /drive  (ackermann_msgs/AckermannDriveStamped)
    |
    v
[vesc_ackermann / ackermann_to_vesc_node]  (apt-installed)
    |--- /commands/motor/speed     (std_msgs/Float64)
    |--- /commands/servo/position  (std_msgs/Float64)
    |
    v
[vesc_driver / vesc_driver_node]  (apt-installed)
    |--- Serial USB to VESC hardware (/dev/ttyACM0)
    |
    v
Physical Motor + Steering Servo
```

## Emergency Stop

The `kart_controller` node subscribes to `/emergency_stop` (`std_msgs/Bool`).
When `true` is received, the node immediately publishes zero speed and zero steering
until `false` is received. Any external system (remote control, watchdog timer, etc.)
can publish to this topic to halt the car.

## Parameter Tuning

All tunable parameters are centralised in `src/f1tenth_bringup/config/vesc_params.yaml`.
This single YAML file is loaded by the launch file and distributed to every node that
needs it. You can override individual parameters at launch time:

```bash
ros2 launch f1tenth_bringup f1tenth_stack.launch.py speed_nominal:=0.5
```

Or at runtime:

```bash
ros2 param set /control_planning_node kp_lateral 1.2
```

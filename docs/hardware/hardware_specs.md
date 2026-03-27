# Hardware Specifications

## F1Tenth Platform

### Compute

| Component | Spec |
|-----------|------|
| Board | NVIDIA Jetson Orin Nano / Xavier NX (depends on your lab's car) |
| OS | JetPack 6 (Ubuntu 22.04 + L4T 36.x) |
| GPU | NVIDIA Ampere / Volta (CUDA-capable) |
| RAM | 8 GB shared CPU/GPU |

### Sensors

| Sensor | Model | Interface | Key Specs |
|--------|-------|-----------|-----------|
| Camera | Stereolabs ZED 2i | USB 3.0 | 120° FOV, 1080p @ 30fps, stereo depth + point cloud |
| Motor Controller | VESC (Vedder ESC) | USB Serial (/dev/ttyACM0) | Reports ERPM, accepts duty/speed/current commands |

### Actuators

| Actuator | Control Method |
|----------|---------------|
| Drive motor | VESC ERPM commands via `ackermann_to_vesc` |
| Steering servo | VESC servo position (0.0 - 1.0) via `ackermann_to_vesc` |

### Wiring Summary

```
Jetson USB 3.0 port  --->  ZED 2i camera
Jetson USB 2.0 port  --->  VESC (serial /dev/ttyACM0)
VESC power output    --->  Brushless drive motor
VESC servo output    --->  Steering servo
Battery (LiPo)       --->  VESC power input
```

## Calibration Values You Must Measure

These live in `src/f1tenth_bringup/config/vesc_params.yaml` and **must** be calibrated
on the physical car. The defaults are reasonable starting points but will not be
accurate for your specific hardware.

### speed_to_erpm_gain (~4614)

1. Command the car at exactly 1.0 m/s via `/drive`
2. Read the ERPM value from `/sensors/core`
3. `speed_to_erpm_gain = measured_erpm / 1.0`

### steering_angle_to_servo_offset (~0.5304)

1. Command 0.0 radians steering
2. Adjust `steering_angle_to_servo_offset` until the wheels point perfectly straight
3. Typical range: 0.45 - 0.55

### steering_angle_to_servo_gain (~-1.2135)

1. Command the maximum physical steering angle (measure it with a protractor)
2. Adjust `steering_angle_to_servo_gain` so the commanded angle matches reality
3. Negative value means the servo direction is inverted (common)

## Go-Kart Platform (Future)

This section will be filled in when the go-kart migration begins. The architecture
is designed so that the same `vision_perception` and `control_planning` nodes work
on both platforms — only the bringup package and VESC parameters change.

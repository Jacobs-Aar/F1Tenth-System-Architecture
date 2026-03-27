# F1Tenth Autonomous Racing Stack

A containerised ROS 2 Humble autonomous driving stack for the F1Tenth (1:10 scale) platform, with a migration path to a full-scale autonomous go-kart.

The car uses a **ZED 2i stereo camera** to detect blue-tape lane boundaries, a **PD controller** to compute steering commands, and a **VESC motor controller** to drive the wheels — all orchestrated through ROS 2 nodes running inside a single Docker container on an NVIDIA Jetson.

---

## Repository Structure

```
F1Tenth-System-Architecture/
├── docker/
│   ├── f1tenth/
│   │   ├── Dockerfile              # Jetson image: ZED SDK + ROS 2 + VESC drivers
│   │   └── docker-compose.yml      # Container config (GPU, USB, volumes)
│   └── go_kart/                    # (WIP) future go-kart Docker config
├── docs/
│   ├── architecture/
│   │   └── system_overview.md      # Data flow diagram and design decisions
│   ├── docker/
│   │   └── setup_guide.md          # Docker explained for beginners
│   ├── hardware/
│   │   └── hardware_specs.md       # Sensors, wiring, calibration guide
│   └── ros2_stack/
│       └── node_architecture.md    # All 5 nodes, topics, and message types
├── scripts/
│   └── f1tenth/
│       ├── setup_host.sh           # One-time Jetson/VM prerequisite installer
│       ├── launch_stack.sh         # One-command full-stack launcher
│       └── teleop_keyboard.py      # Keyboard manual driving for testing
├── src/                            # ROS 2 workspace (mounted into container)
│   ├── vision_perception/          # Custom Python: blue-tape lane detection
│   │   ├── vision_perception/
│   │   │   ├── __init__.py
│   │   │   └── line_mapper.py      # THE perception node
│   │   ├── resource/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   ├── control_planning/           # Custom C++: PD lane-following controller
│   │   ├── src/
│   │   │   └── kart_controller.cpp # THE control node
│   │   ├── include/
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   └── f1tenth_bringup/            # Launch files + config (glue package)
│       ├── launch/
│       │   └── f1tenth_stack.launch.py  # Starts ALL 5 nodes at once
│       ├── config/
│       │   └── vesc_params.yaml    # ALL tunable parameters in one place
│       ├── f1tenth_bringup/
│       │   └── __init__.py
│       ├── resource/
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── .gitignore
└── README.md                       # You are here
```

---

## How It Works (The Pipeline)

```
ZED 2i Camera
    │
    ▼
[zed_wrapper]  publishes colour image + 3D point cloud
    │
    ▼
[line_mapper]  detects blue tape → computes lateral + heading error
    │
    ▼
[kart_controller]  PD control → outputs steering angle + speed
    │
    ▼
[ackermann_to_vesc]  converts Ackermann command → VESC ERPM + servo
    │
    ▼
[vesc_driver]  sends serial commands to the physical VESC hardware
    │
    ▼
Car moves autonomously
```

All 5 nodes launch with a single command. You never have to start them individually.

---

## Prerequisites

You need a Linux machine (Ubuntu 22.04 recommended). This can be:
- The **NVIDIA Jetson** on the actual F1Tenth car (production)
- An **Ubuntu VM** (VirtualBox, etc.) for development without hardware

### Install Everything Automatically

```bash
chmod +x scripts/f1tenth/setup_host.sh
./scripts/f1tenth/setup_host.sh
```

This installs Docker Engine, Docker Compose, and the NVIDIA Container Toolkit.

### Or Install Manually

1. **Git**: `sudo apt install git`
2. **Docker Engine**: https://docs.docker.com/engine/install/ubuntu/
3. **Docker Compose**: `sudo apt install docker-compose-plugin`
4. **NVIDIA Container Toolkit** (Jetson/GPU only): https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

After installing, log out and back in so Docker group permissions take effect.

---

## Quick Start (3 Commands)

### 1. Clone the Repository

```bash
git clone https://github.com/Jacobs-Aar/F1Tenth-System-Architecture.git
cd F1Tenth-System-Architecture
```

### 2. Build the Docker Image

```bash
cd docker/f1tenth
docker compose build
```

This takes about 10 minutes the first time (downloads the ZED SDK + ROS 2 base image). Subsequent builds are cached and nearly instant.

### 3. Launch Everything

**Option A — One-command launcher (recommended):**

```bash
cd ../..   # back to repo root
chmod +x scripts/f1tenth/launch_stack.sh
./scripts/f1tenth/launch_stack.sh
```

This starts the container, builds the workspace, and launches all nodes.

**Option B — Step by step (if you want to understand each step):**

```bash
# Start container in background
cd docker/f1tenth
docker compose up -d

# Open a shell inside the container
docker exec -it f1tenth_stack bash

# (You are now INSIDE the container)
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch the full autonomous stack
ros2 launch f1tenth_bringup f1tenth_stack.launch.py
```

Press **Ctrl+C** to stop. The container keeps running in the background.

---

## Development Workflow

### Editing Code

Your `src/` folder is **live-mounted** into the container. This means:

- Edit files on your host machine with any editor (VS Code, vim, etc.)
- Changes are **instantly visible** inside the container
- **Python changes** take effect immediately (thanks to `--symlink-install`)
- **C++ changes** require a rebuild inside the container:

```bash
docker exec -it f1tenth_stack bash
cd /ros2_ws
colcon build --symlink-install --packages-select control_planning
source install/setup.bash
```

### Testing Without Hardware (VM Development)

If you are developing on a VM without a Jetson or ZED camera:

1. **Edit the Dockerfile** — change the `FROM` line to:
   ```
   FROM ros:humble-ros-base
   ```
2. The ZED node will fail to start, but `vision_perception` and `control_planning` will still build.
3. You can **feed fake data** using a recorded ROS bag:
   ```bash
   ros2 bag play my_recording.db3
   ```

### Manual Driving (Teleop Test)

To test the VESC and steering without the vision stack:

```bash
# Inside the container — start only the VESC nodes
ros2 run vesc_driver vesc_driver_node --ros-args -p port:=/dev/ttyACM0
# In a second terminal inside the container
python3 /ros2_ws/src/../scripts/f1tenth/teleop_keyboard.py
```

Controls: W/S = speed, A/D = steer, SPACE = stop, Q = quit.

---

## Configuration and Tuning

All tunable parameters live in one file:

```
src/f1tenth_bringup/config/vesc_params.yaml
```

### Key Parameters

| Parameter | Default | What It Does |
|-----------|---------|--------------|
| `speed_nominal` | 1.0 m/s | Cruise speed while lane tracking |
| `kp_lateral` | 0.8 | How aggressively the car corrects lateral drift |
| `kd_lateral` | 0.05 | Damping on lateral correction (reduces oscillation) |
| `kp_heading` | 0.5 | How aggressively the car follows heading changes |
| `max_steer_angle` | 0.34 rad | Hard clamp on steering (~20 degrees) |
| `min_quality` | 0.4 | Minimum lane detection confidence to act on |
| `speed_to_erpm_gain` | 4614.0 | **MUST CALIBRATE** on physical car |
| `steering_angle_to_servo_offset` | 0.5304 | **MUST CALIBRATE** — makes wheels straight at 0 rad |
| `steering_angle_to_servo_gain` | -1.2135 | **MUST CALIBRATE** — maps rad to servo units |

### Override at Launch Time

```bash
ros2 launch f1tenth_bringup f1tenth_stack.launch.py speed_nominal:=0.5
```

### Override at Runtime (No Restart)

```bash
ros2 param set /control_planning_node kp_lateral 1.2
```

### Calibration Procedure

See `docs/hardware/hardware_specs.md` for detailed step-by-step calibration of ERPM gain, servo offset, and servo gain.

---

## Debugging

### Useful Commands (Run Inside Container)

```bash
# Are all nodes running?
ros2 node list

# What topics exist?
ros2 topic list

# Watch lane guidance output live
ros2 topic echo /perception/lane_guidance

# Check perception frame rate
ros2 topic hz /perception/lane_guidance

# View the debug lane mask image (requires GUI forwarding or RViz)
ros2 topic echo /perception/debug/lane_mask --no-arr

# Send an emergency stop
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: true"

# Release emergency stop
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "data: false"
```

---

## Docker Cheat Sheet

If you are new to Docker, here is everything you need:

| What you want to do | Command |
|---------------------|---------|
| Build the image | `cd docker/f1tenth && docker compose build` |
| Start the container | `docker compose up -d` |
| Open a shell inside | `docker exec -it f1tenth_stack bash` |
| Stop the container | `docker compose down` |
| See running containers | `docker ps` |
| See container logs | `docker logs f1tenth_stack` |
| Rebuild from scratch | `docker compose build --no-cache` |
| Remove all Docker data | `docker system prune -a` (careful!) |

**Key concept:** Your `src/` code is mounted into the container as a volume. You never need to rebuild the Docker *image* just because you changed Python or C++ code. You only rebuild the image if you change the *Dockerfile* (e.g., adding a new apt package).

---

## ROS 2 Cheat Sheet

If you are new to ROS 2:

| Concept | What it means |
|---------|---------------|
| **Node** | A running program that does one thing (e.g., read camera, compute steering) |
| **Topic** | A named channel that nodes publish/subscribe to (e.g., `/drive`) |
| **Message** | The data format on a topic (e.g., `AckermannDriveStamped`) |
| **Package** | A folder of code that builds together (e.g., `vision_perception`) |
| **Launch file** | A Python script that starts multiple nodes at once |
| **colcon build** | Compiles all packages in the workspace |
| **source install/setup.bash** | Tells your terminal where to find the just-built packages |

---

## Troubleshooting

### "Container exited immediately"
- Check logs: `docker logs f1tenth_stack`
- Usually means the NVIDIA runtime is not installed. Run `scripts/f1tenth/setup_host.sh`.

### "Cannot find package vision_perception"
- You forgot to build and source inside the container:
  ```bash
  cd /ros2_ws && colcon build --symlink-install && source install/setup.bash
  ```

### "VESC not found on /dev/ttyACM0"
- Check the port on your host: `ls /dev/ttyACM*`
- Update `docker-compose.yml` devices section with the correct port.
- Make sure you are in the `dialout` group: `groups` should show `dialout`.

### "ZED camera not detected"
- Make sure the ZED is plugged into a **USB 3.0** port (blue port).
- Inside the container, run: `lsusb | grep -i stereo`

### Build error: "package 'zed_wrapper' not found"
- This is expected if you are using the `ros:humble-ros-base` image (VM mode).
- The ZED wrapper only exists in the Stereolabs base image.
- For VM testing, comment out the `zed_node` in the launch file and use a ROS bag.

---

## What To Do Next

1. **Get the basics working** — run `setup_host.sh`, build the image, enter the container, run `colcon build`.
2. **Test teleop** — use `teleop_keyboard.py` to verify the VESC responds to commands.
3. **Calibrate the VESC** — measure `speed_to_erpm_gain`, `servo_offset`, and `servo_gain` on the physical car.
4. **Test perception** — run just the ZED + `line_mapper` and watch `/perception/lane_guidance` with `ros2 topic echo`.
5. **Close the loop** — launch the full stack and let the car drive autonomously at low speed (`speed_nominal:=0.3`).
6. **Tune** — adjust PD gains in `vesc_params.yaml` until the car tracks smoothly.

---

## License

MIT

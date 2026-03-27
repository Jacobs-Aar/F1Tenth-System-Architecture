# Docker Setup Guide

## Why Docker?

Docker isolates the entire ROS 2 stack (OS libraries, CUDA, ZED SDK, VESC drivers)
into a single container image. This means:

- **No "it works on my machine" problems** — the container is the same everywhere.
- **Easy migration** — copy the image to a new Jetson and it runs identically.
- **No conflicts** — the host OS stays clean; ROS 2 deps live only in the container.

## Base Image Selection

The Dockerfile uses a Stereolabs base image that bundles the ZED SDK with CUDA
and ROS 2 Humble, purpose-built for Jetson:

```
stereolabs/zed:4.2-ros2-humble-l4t36.4.0
```

**Breaking this tag down:**
- `4.2` — ZED SDK version
- `ros2-humble` — ROS 2 Humble pre-installed
- `l4t36.4.0` — NVIDIA L4T (Linux for Tegra) version matching JetPack 6

**For VM development** (no Jetson hardware), replace the FROM line with:
```
FROM ros:humble-ros-base
```
The ZED node won't run, but everything else will build and you can test with
recorded ROS bag files.

## What the Dockerfile Installs

On top of the base image, the Dockerfile adds:

| Category | Packages |
|----------|----------|
| ROS 2 add-ons | ackermann-msgs, cv-bridge, image-transport, message-filters, sensor-msgs-py |
| VESC stack | vesc, vesc-msgs, vesc-driver, vesc-ackermann |
| Build tools | colcon, pip, cmake, git |
| Python | numpy, opencv-python-headless |

## docker-compose.yml Explained

| Setting | Purpose |
|---------|---------|
| `context: ../../` | Build context is the repo root so Docker can see both `docker/` and `src/` |
| `volumes: ../../src:/ros2_ws/src` | Live-mounts your source code — edit on host, instantly visible in container |
| `devices` | Passes through USB (ZED camera) and serial (VESC) to the container |
| `network_mode: host` | Container shares the Jetson's network — ROS 2 DDS discovery works automatically |
| `privileged: true` | Full hardware access, required for ZED USB on Jetson |
| GPU reservations | Grants the container access to the NVIDIA GPU for CUDA / ZED SDK |

## Common Commands

```bash
# Build the image (first time takes ~10 minutes)
cd docker/f1tenth
docker compose build

# Start the container in the background
docker compose up -d

# Open a shell inside the running container
docker exec -it f1tenth_stack bash

# Stop and remove the container
docker compose down

# Rebuild after changing the Dockerfile
docker compose build --no-cache
```

## Troubleshooting

**"Cannot connect to VESC on /dev/ttyACM0"**
- Run `ls /dev/ttyACM*` on the host to find the correct port.
- Update the `devices` section in `docker-compose.yml` accordingly.

**"nvidia-container-runtime not found"**
- Install the NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

**"Permission denied" on USB devices**
- The `privileged: true` flag should handle this. If not, add your user to the `dialout` group:
  `sudo usermod -a -G dialout $USER` then log out and back in.

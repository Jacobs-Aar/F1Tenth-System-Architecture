# F1Tenth Final Pipeline Test — Cold Boot to Autonomous Driving

This procedure assumes the Docker image is already built and blue tape is laid out in a circular track on the floor.

---

## Step 0 — Clean Shutdown (ensure nothing left over)

Run on the Jetson host:

```bash
cd ~/AandRF1Tenth/F1Tenth-System-Architecture/docker/f1tenth
docker compose down
docker ps -a --filter name=f1tenth_stack
```

Confirm output says nothing is running. If a container is listed, force remove:

```bash
docker rm -f f1tenth_stack
```

---

## Step 1 — Start Container Fresh

```bash
cd ~/AandRF1Tenth/F1Tenth-System-Architecture/docker/f1tenth
docker compose up -d
```

Wait a few seconds, then verify:

```bash
docker ps --filter name=f1tenth_stack --format "{{.Names}} {{.Status}}"
```

Expected: `f1tenth_stack Up X seconds`

---

## Step 2 — Build the ROS 2 Workspace

```bash
docker exec -it f1tenth_stack bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /opt/f1tenth_deps/install/setup.bash && \
   cd /ros2_ws && \
   colcon build --symlink-install 2>&1 | tail -5"
```

Expected: `Summary: 3 packages finished`

---

## Step 3 — Verify All Packages

```bash
docker exec -it f1tenth_stack bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /opt/f1tenth_deps/install/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 pkg list | grep -E 'vision_perception|control_planning|f1tenth_bringup|vesc_driver|vesc_ackermann'"
```

Expected: all 5 packages listed.

---

## Step 4 — Verify Hardware

Run on host:

```bash
echo "=== ZED Camera ===" && lsusb | grep -i "stereo\|2b03"
echo "=== VESC ===" && ls /dev/ttyACM* 2>/dev/null || echo "NO VESC"
```

Expected: ZED shows `STEREOLABS`, VESC shows `/dev/ttyACM0`.

---

## Step 5 — Launch Full Autonomous Stack (speed=0)

This is the main launch. Open **Terminal 1 (T1)** — SSH into the Jetson:

```bash
docker exec -it f1tenth_stack bash
```

Inside the container in T1, run:

```bash
source /opt/ros/humble/setup.bash
source /opt/f1tenth_deps/install/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch f1tenth_bringup f1tenth_stack.launch.py speed_nominal:=0.0
```

This starts all 5 nodes:
- ZED camera (takes ~10-15 seconds to initialize)
- Vision perception (line_mapper)
- Control planning (kart_controller)
- Ackermann to VESC translator
- VESC driver

Wait until you see output from line_mapper showing `lat=... hdg=... q=...` — this means the full pipeline is working. The car will NOT move because speed is 0.

If the ZED takes time for "Recomputing alignment..." that's normal — the rest of the pipeline starts working once data flows.

---

## Step 6 — Verify All Nodes Running

Open **Terminal 2 (T2)** — SSH into Jetson:

```bash
docker exec -it f1tenth_stack bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /opt/f1tenth_deps/install/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 node list"
```

Expected (names may vary slightly):
```
/ackermann_to_vesc
/control_planning_node
/vision_perception_node
/vesc_driver
/zed/zed_node
/zed/zed_state_publisher
```

---

## Step 7 — Monitor Control Outputs

Keep **T2** as the monitoring terminal:

```bash
docker exec -it f1tenth_stack bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /opt/f1tenth_deps/install/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 topic echo /drive"
```

You should see `AckermannDriveStamped` messages with:
- `speed: 0.0` (because we set speed_nominal to 0)
- `steering_angle:` some non-zero value that changes as the car "sees" the blue tape

This confirms the full pipeline: camera → perception → control → drive commands.

If `steering_angle` is always 0.0, point the camera at blue tape on the ground to verify perception is detecting it.

---

## Step 8 — Verify Perception is Detecting Lanes

Open **Terminal 3 (T3)**:

```bash
docker exec -it f1tenth_stack bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /opt/f1tenth_deps/install/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 topic echo /perception/lane_guidance"
```

Expected (with blue tape visible):
```
x: 0.15    # lateral error (non-zero)
y: -0.03   # heading error (non-zero)
z: 0.5     # or 1.0 — quality > 0 means lanes detected
```

If `z` is always 0.0, the camera cannot see the blue tape. Check camera placement and lighting.

Once you confirm `z > 0` consistently, the perception pipeline is working. Press Ctrl+C in T3.

---

## Step 9 — Enable Speed 0.5 (Car Will Move!)

> **WARNING: The car will drive autonomously. Be ready to grab it or run the emergency stop.**

Place the car inside the blue tape circular track, pointed along the lane direction.

Open **T3** as the command terminal:

```bash
docker exec -it f1tenth_stack bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /opt/f1tenth_deps/install/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 param set /control_planning_node speed_nominal 0.5"
```

The car should begin moving and following the blue tape lane.

---

## Step 10 — Emergency Stop (if needed)

### Graceful stop:

```bash
docker exec -it f1tenth_stack bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /opt/f1tenth_deps/install/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'"
```

### BRUTE FORCE stop (if car is unresponsive):

```bash
docker kill f1tenth_stack
```

This instantly kills the container — no more commands reach the VESC and the car stops.

### Resume after graceful stop:

```bash
docker exec -it f1tenth_stack bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /opt/f1tenth_deps/install/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: false}'"
```

---

## Step 11 — Live Tuning (if car oscillates or behaves poorly)

Reduce aggressiveness:

```bash
docker exec -it f1tenth_stack bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /opt/f1tenth_deps/install/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 param set /control_planning_node kp_lateral 0.5 && \
   ros2 param set /control_planning_node kd_lateral 0.1"
```

Increase speed gradually:

```bash
docker exec -it f1tenth_stack bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /opt/f1tenth_deps/install/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 param set /control_planning_node speed_nominal 0.8"
```

Set speed back to zero (soft stop without e-stop):

```bash
docker exec -it f1tenth_stack bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /opt/f1tenth_deps/install/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 param set /control_planning_node speed_nominal 0.0"
```

---

## Step 12 — Clean Shutdown

Press Ctrl+C in T1 to stop the launch file, then:

```bash
cd ~/AandRF1Tenth/F1Tenth-System-Architecture/docker/f1tenth
docker compose down
```

---

## Quick Reference Card

| Action | Command (from host) |
|---|---|
| Start container | `docker compose up -d` |
| Build workspace | `docker exec -it f1tenth_stack bash -c "source /opt/ros/humble/setup.bash && source /opt/f1tenth_deps/install/setup.bash && cd /ros2_ws && colcon build --symlink-install"` |
| Launch stack (speed=0) | Inside container: `ros2 launch f1tenth_bringup f1tenth_stack.launch.py speed_nominal:=0.0` |
| Set speed | `docker exec -it f1tenth_stack bash -c "source /opt/ros/humble/setup.bash && source /opt/f1tenth_deps/install/setup.bash && source /ros2_ws/install/setup.bash && ros2 param set /control_planning_node speed_nominal 0.5"` |
| Emergency stop | `docker exec -it f1tenth_stack bash -c "source /opt/ros/humble/setup.bash && source /opt/f1tenth_deps/install/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'"` |
| Brute force stop | `docker kill f1tenth_stack` |
| Resume | `docker exec -it f1tenth_stack bash -c "source /opt/ros/humble/setup.bash && source /opt/f1tenth_deps/install/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: false}'"` |
| Shutdown | `docker compose down` |
| Teleop (manual) | `docker exec -it f1tenth_stack bash -c "source /opt/ros/humble/setup.bash && source /opt/f1tenth_deps/install/setup.bash && source /ros2_ws/install/setup.bash && python3 /ros2_ws/scripts/f1tenth/teleop_keyboard.py"` |

#!/usr/bin/env bash
# =============================================================================
# scripts/f1tenth/launch_stack.sh
# =============================================================================
# ONE-COMMAND launcher for the full F1Tenth autonomous stack.
#
# This script:
#   1. Starts the Docker container (if not already running)
#   2. Builds the ROS 2 workspace inside the container
#   3. Launches all 5 nodes via the bringup launch file
#
# Usage (from the repo root):
#   chmod +x scripts/f1tenth/launch_stack.sh
#   ./scripts/f1tenth/launch_stack.sh
#
# To override speed:
#   ./scripts/f1tenth/launch_stack.sh speed_nominal:=0.5
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DOCKER_DIR="$REPO_ROOT/docker/f1tenth"
CONTAINER_NAME="f1tenth_stack"

# Pass any extra args through to the launch file (e.g., speed_nominal:=0.5)
LAUNCH_ARGS="${*:-}"

echo "============================================"
echo "  F1Tenth Autonomous Stack Launcher"
echo "============================================"

# -- Step 1: Ensure container is running --------------------------------------
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[1/3] Starting Docker container..."
    cd "$DOCKER_DIR"
    docker compose up -d
    # Give it a moment to initialise
    sleep 2
else
    echo "[1/3] Container already running."
fi

# -- Step 2: Build workspace inside container ---------------------------------
echo "[2/3] Building ROS 2 workspace (colcon build)..."
docker exec -it "$CONTAINER_NAME" bash -c \
    "source /opt/ros/humble/setup.bash && \
     cd /ros2_ws && \
     colcon build --symlink-install 2>&1 | tail -5"

# -- Step 3: Launch the full stack --------------------------------------------
echo "[3/3] Launching autonomous stack..."
echo "       (Press Ctrl+C to stop)"
echo ""

docker exec -it "$CONTAINER_NAME" bash -c \
    "source /opt/ros/humble/setup.bash && \
     source /ros2_ws/install/setup.bash && \
     ros2 launch f1tenth_bringup f1tenth_stack.launch.py $LAUNCH_ARGS"

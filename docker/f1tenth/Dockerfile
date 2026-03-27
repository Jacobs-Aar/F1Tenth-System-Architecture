# =============================================================================
# docker/f1tenth/Dockerfile
# =============================================================================
# Target:  NVIDIA Jetson with JetPack 6 (Ubuntu 22.04, L4T 36.x)
#
# BEFORE BUILDING: Verify the correct Stereolabs base image tag for your
#     JetPack version at: https://hub.docker.com/r/stereolabs/zed/tags
#     Search for tags containing "ros2-humble" and "l4t36"
#
# For VM development (no Jetson / no ZED hardware):
#   Replace the FROM line with:  FROM ros:humble-ros-base
#   You won't be able to run the ZED node, but all other nodes will build
#   and you can feed fake data via a ROS 2 bag file for testing.
# =============================================================================

# -- Base: ZED SDK 4.2 + CUDA + ROS 2 Humble on JetPack 6 -------------------
FROM stereolabs/zed:4.2-ros2-humble-l4t36.4.0

ENV DEBIAN_FRONTEND=noninteractive

# -- System packages ----------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS 2 core add-ons
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-message-filters \
    ros-humble-sensor-msgs-py \
    # VESC F1Tenth driver stack
    ros-humble-vesc \
    ros-humble-vesc-msgs \
    ros-humble-vesc-driver \
    ros-humble-vesc-ackermann \
    # Build tools
    python3-colcon-common-extensions \
    python3-pip \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# -- Python dependencies ------------------------------------------------------
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python-headless

# -- Workspace setup -----------------------------------------------------------
# The src/ directory is NOT copied here -- it is mounted at runtime via
# docker-compose volumes so edits on the host are immediately visible
# inside the container without rebuilding the image.
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# -- Shell convenience ---------------------------------------------------------
# Auto-source ROS 2 and the workspace on every new bash session
RUN echo "source /opt/ros/humble/setup.bash"                       >> ~/.bashrc && \
    echo "[ -f /ros2_ws/install/setup.bash ] && \
          source /ros2_ws/install/setup.bash"                       >> ~/.bashrc && \
    echo "echo 'ROS 2 Humble ready -- workspace: /ros2_ws'"        >> ~/.bashrc

CMD ["/bin/bash"]

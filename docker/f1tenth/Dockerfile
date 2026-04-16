# =============================================================================
# docker/f1tenth/Dockerfile
# =============================================================================
# Target:  NVIDIA Jetson — JetPack 6.2, L4T R36.4.3, arm64
# Base:    stereolabs/zed:5.2-devel-l4t-r36.4
#
# Workspace architecture (ROS 2 overlay pattern):
#   Layer 1: /opt/ros/humble          <- ROS 2 base (apt-installed)
#   Layer 2: /opt/f1tenth_deps        <- VESC + ZED wrapper (built from source)
#   Layer 3: /ros2_ws                 <- YOUR code (mounted from host at runtime)
#
#   Each layer overlays the previous one. The deps in /opt/f1tenth_deps are
#   baked into the Docker image and never affected by the src/ volume mount.
# =============================================================================

FROM stereolabs/zed:5.2-devel-l4t-r36.4

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8

# -- 1. Locale + basic tools --------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common \
        curl \
        locales \
        gnupg2 \
        lsb-release \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# -- 2. Add ROS 2 Humble apt repository ---------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list

# -- 3. Install ROS 2 Humble + all dependencies (verified arm64) --------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-message-filters \
    ros-humble-sensor-msgs-py \
    ros-humble-serial-driver \
    ros-humble-io-context \
    ros-humble-asio-cmake-module \
    ros-humble-udp-msgs \
    libasio-dev \
    ros-humble-xacro \
    ros-humble-diagnostic-updater \
    ros-humble-nmea-msgs \
    ros-humble-geographic-msgs \
    ros-humble-robot-localization \
    ros-humble-backward-ros \
    ros-humble-launch-testing-ament-cmake \
    nlohmann-json3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    build-essential \
    cmake \
    git \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# -- 4. Initialise rosdep -----------------------------------------------------
RUN rosdep init 2>/dev/null || true \
    && rosdep update --rosdistro humble

# -- 5. Build VESC + ZED wrapper into a SEPARATE permanent workspace -----------
#    /opt/f1tenth_deps is baked into the image. The volume mount only touches
#    /ros2_ws/src, so these dependencies are never overwritten or broken.
RUN mkdir -p /opt/f1tenth_deps/src \
    && cd /opt/f1tenth_deps/src \
    && git clone https://github.com/f1tenth/vesc.git --branch ros2 \
    && git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git \
    && cd /opt/f1tenth_deps \
    && apt-get update \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && \
       rosdep install --from-paths src --ignore-src -r -y || true && \
       colcon build \
           --packages-skip zed_debug \
           --cmake-args=-DCMAKE_BUILD_TYPE=Release \
           --parallel-workers \$(nproc)" \
    && rm -rf /opt/f1tenth_deps/build \
    && rm -rf /opt/f1tenth_deps/log \
    && rm -rf /var/lib/apt/lists/*

# -- 6. Python Dependencies ---------------------------------------------------
RUN pip3 install --no-cache-dir 'numpy<2' opencv-python-headless scipy

# -- 7. Prepare the user workspace --------------------------------------------
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# -- 8. Shell convenience -----------------------------------------------------
#    Source order matters: humble -> f1tenth_deps -> ros2_ws (overlay chain)
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /opt/f1tenth_deps/install/setup.bash' >> ~/.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'echo "=== ROS 2 Humble ready | workspace: /ros2_ws ==="' >> ~/.bashrc

CMD ["/bin/bash"]

FROM ros:jazzy-ros-base

# ── Variáveis de ambiente ────────────────────────────────────────────────────
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=27

# ── Chave/lista do repositório ROS2 Jazzy ────────────────────────────────────
RUN apt-get update --allow-unauthenticated || true && \
    apt-get install -y curl ca-certificates && \
    export ROS_APT_SOURCE_VERSION=$(curl -s \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    apt-get install -y /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb && \
    rm -rf /var/lib/apt/lists/*

# ── Ferramentas básicas ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    git \
    wget \
    nano \
    vim \
    usbutils \
    net-tools \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-transforms3d \
    python3-opencv \
    build-essential \
    cmake \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# ── rosdep ───────────────────────────────────────────────────────────────────
RUN rosdep init || true && \
    rosdep update

# ── Pacotes ROS ──────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-twist-mux \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf-transformations \
    ros-jazzy-cv-bridge \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-std-srvs \
    && rm -rf /var/lib/apt/lists/*

# ── ODrive ───────────────────────────────────────────────────────────────────
RUN pip3 install --no-deps odrive --break-system-packages

################################################################################
# Compila o SDK primeiro
################################################################################

COPY etc/YDLidar-SDK /tmp/YDLidar-SDK

WORKDIR /tmp/YDLidar-SDK/build

RUN cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/ydlidar-sdk && \
    make -j$(nproc) && \
    make install

RUN find /opt/ydlidar-sdk -name "*Config.cmake" && \
    find /opt/ydlidar-sdk -name "CYdLidar.h" && \
    find /opt/ydlidar-sdk -name "libydlidar*"

################################################################################
# Workspace
################################################################################

WORKDIR /robis_ws

COPY src ./src

RUN find src -mindepth 1 -maxdepth 1 -type d \
    ! -name robis_bringup \
    ! -name robis_description \
    ! -name robis_navigation \
    ! -name robis_odrive \
    ! -name robis_ydlidar \
    ! -name robis_vision \
    -exec rm -rf {} +

RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install \
        --from-paths src \
        --ignore-src \
        -r -y || true

ENV CMAKE_PREFIX_PATH=/opt/ydlidar-sdk
ENV LD_LIBRARY_PATH=/opt/ydlidar-sdk/lib
ENV LIBRARY_PATH=/opt/ydlidar-sdk/lib
ENV CPLUS_INCLUDE_PATH=/opt/ydlidar-sdk/include

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build \
        --symlink-install \
        --cmake-args \
            -DCMAKE_PREFIX_PATH=/opt/ydlidar-sdk

################################################################################
# Entrypoint
################################################################################

COPY <<'EOF' /entrypoint.sh
#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /robis_ws/install/setup.bash ]; then
    source /robis_ws/install/setup.bash
fi

exec "$@"
EOF

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
# CPU control/sensors. GPU inference will use its own L4T-compatible image.
FROM docker.io/library/ros:jazzy-ros-base AS runtime
ENV DEBIAN_FRONTEND=noninteractive \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=27 \
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/opt/odrive-python
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-opencv python3-yaml python3-serial python3-transforms3d \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    libusb-1.0-0 usbutils v4l-utils \
    ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher ros-jazzy-xacro \
    ros-jazzy-tf-transformations ros-jazzy-cv-bridge \
    ros-jazzy-message-filters ros-jazzy-vision-msgs \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*
# Keep NumPy below 2 for cv_bridge/OpenCV; isolate the pip API from apt files.
ARG ODRIVE_VERSION=0.6.11.post0
RUN python3 -m pip install --no-cache-dir --target /opt/odrive-python \
        "numpy<2" "setuptools<80" "transforms3d==0.4.1" "pyserial==3.5" "odrive==${ODRIVE_VERSION}" \
    && python3 -c 'import odrive; from odrive.enums import AXIS_STATE_IDLE'
WORKDIR /robis_ws
COPY docker/entrypoint.sh /usr/local/bin/robis-entrypoint
RUN chmod +x /usr/local/bin/robis-entrypoint
ENTRYPOINT ["/usr/local/bin/robis-entrypoint"]
CMD ["bash"]

FROM runtime AS development
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config git ca-certificates python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
ARG YDLIDAR_SDK_COMMIT=7c0a2328fd85b65960c37287a67b0edbee8caa4b
RUN git init /tmp/YDLidar-SDK \
    && git -C /tmp/YDLidar-SDK fetch --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git "${YDLIDAR_SDK_COMMIT}" \
    && git -C /tmp/YDLidar-SDK checkout --detach FETCH_HEAD \
    && test "$(git -C /tmp/YDLidar-SDK rev-parse HEAD)" = "${YDLIDAR_SDK_COMMIT}" \
    && cmake -S /tmp/YDLidar-SDK -B /tmp/ydlidar-build \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON \
        -DBUILD_EXAMPLES=OFF -DBUILD_CSHARP=OFF -DBUILD_TEST=OFF \
        -DCMAKE_INSTALL_PREFIX=/opt/ydlidar-sdk \
    && cmake --build /tmp/ydlidar-build --parallel 1 \
    && cmake --install /tmp/ydlidar-build \
    && printf '%s\n' /opt/ydlidar-sdk/lib > /etc/ld.so.conf.d/ydlidar-sdk.conf \
    && ldconfig \
    && test -f /opt/ydlidar-sdk/lib/cmake/ydlidar_sdk/ydlidar_sdkConfig.cmake \
    && rm -rf /tmp/YDLidar-SDK /tmp/ydlidar-build
COPY config /robis_ws/config
COPY scripts/build_workspace.sh /robis_ws/scripts/build_workspace.sh
RUN chmod +x /robis_ws/scripts/build_workspace.sh

FROM development AS robot-sources
# These are the only workspace packages shipped to the robot.
COPY src/robis_interfaces /robis_ws/src/robis_interfaces
COPY src/robis_description /robis_ws/src/robis_description
COPY src/robis_vision /robis_ws/src/robis_vision
COPY src/robis_odrive /robis_ws/src/robis_odrive
COPY src/robis_ydlidar /robis_ws/src/robis_ydlidar
COPY src/robis_bringup /robis_ws/src/robis_bringup

FROM robot-sources AS builder
RUN /robis_ws/scripts/build_workspace.sh basic \
    && . /opt/ros/jazzy/setup.sh && . /robis_ws/install/setup.sh \
    && ros2 launch robis_bringup basic_real.launch.py --show-args \
    && test -x /robis_ws/install/robis_ydlidar/lib/robis_ydlidar/robis_ydlidar_node

FROM runtime AS robot
COPY --from=builder /robis_ws/install /robis_ws/install
COPY --from=development /opt/ydlidar-sdk /opt/ydlidar-sdk
RUN printf '%s\n' /opt/ydlidar-sdk/lib > /etc/ld.so.conf.d/ydlidar-sdk.conf \
    && ldconfig \
    && . /opt/ros/jazzy/setup.sh && . /robis_ws/install/setup.sh \
    && ldd /robis_ws/install/robis_ydlidar/lib/robis_ydlidar/robis_ydlidar_node > /tmp/robis-libraries \
    && if grep -q 'not found' /tmp/robis-libraries; then cat /tmp/robis-libraries; exit 1; fi \
    && grep -q libydlidar_sdk /tmp/robis-libraries \
    && rm /tmp/robis-libraries
STOPSIGNAL SIGINT
CMD ["ros2", "launch", "robis_bringup", "basic_real.launch.py"]

# Complete physical profile. The basic image above remains small and independent.
FROM runtime AS navigation-runtime
COPY --from=development /opt/ydlidar-sdk /opt/ydlidar-sdk
RUN printf '%s\n' /opt/ydlidar-sdk/lib > /etc/ld.so.conf.d/ydlidar-sdk.conf \
    && ldconfig && apt-get update && apt-get install -y --no-install-recommends \
    python3-venv python3-matplotlib python3-scipy python3-pil \
    python3-psutil python3-requests \
    ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server ros-jazzy-nav2-common \
    ros-jazzy-nav2-controller ros-jazzy-nav2-smoother ros-jazzy-nav2-planner \
    ros-jazzy-nav2-navfn-planner ros-jazzy-nav2-behaviors ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-waypoint-follower ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-lifecycle-manager ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-nav2-regulated-pure-pursuit-controller \
    && rm -rf /var/lib/apt/lists/*
COPY config /robis_ws/config

FROM navigation-runtime AS navigation-builder
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
COPY --from=robot-sources /robis_ws/src /robis_ws/src
COPY src/robis_navigation /robis_ws/src/robis_navigation
COPY src/robis_semantic_navigation /robis_ws/src/robis_semantic_navigation
COPY scripts/build_workspace.sh /robis_ws/scripts/build_workspace.sh
RUN /robis_ws/scripts/build_workspace.sh navigation

FROM navigation-runtime AS navigation-perception
ENV VIRTUAL_ENV=/opt/detector PATH=/opt/detector/bin:$PATH \
    PYTHONPATH=/opt/detector/lib/python3.12/site-packages:/opt/odrive-python \
    OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 YOLO_AUTOINSTALL=false \
    YOLO_CONFIG_DIR=/robis_ws/data/detector-config
RUN python3 -m venv --system-site-packages /opt/detector \
    && python3 -m pip install --no-cache-dir torch==2.9.1 torchvision==0.24.1 \
        --index-url https://download.pytorch.org/whl/cpu \
    && python3 -m pip install --no-cache-dir --no-deps \
        ultralytics==8.4.21 ultralytics-thop==2.0.18 polars==1.26.0
# Keep apt OpenCV (GStreamer/V4L2), rather than replacing it with a pip wheel.
RUN mkdir -p /opt/robis-models && cd /opt/robis-models \
    && python3 -c 'import cv2, numpy as np; from ultralytics import YOLO; m=YOLO("yolo26n-seg.pt"); assert m.task == "segment"; m.predict(np.zeros((480,640,3), dtype=np.uint8), imgsz=320, device="cpu", verbose=False); assert "GStreamer:                   YES" in cv2.getBuildInformation()'

FROM navigation-perception AS robot-navigation
COPY --from=navigation-builder /robis_ws/install /robis_ws/install
RUN . /opt/ros/jazzy/setup.sh && . /robis_ws/install/setup.sh \
    && ros2 launch robis_bringup semantic_real.launch.py --show-args \
    && python3 -c 'import odrive; from robis_semantic_navigation.server import SemanticNavigationServer' \
    && test -x /robis_ws/install/robis_ydlidar/lib/robis_ydlidar/robis_ydlidar_node \
    && test ! -d /robis_ws/install/robis_gazebo \
    && test ! -d /opt/ros/jazzy/share/ros_gz_sim \
    && test ! -d /robis_ws/src
CMD ["ros2", "launch", "robis_bringup", "semantic_real.launch.py"]

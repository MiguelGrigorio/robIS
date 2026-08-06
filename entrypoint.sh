#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

cd /root/ros2_ws
if [ -d src ] && [ "$(ls -A src 2>/dev/null)" ]; then
    echo "[entrypoint] Buildando workspace (colcon build)..."
    # A imagem de simulação não compila os drivers de hardware.
    colcon build --symlink-install \
        --packages-select \
        robis_bringup robis_description robis_gazebo robis_navigation \
        robis_vision
    source install/setup.bash
    ulimit -c 0
else
    echo "[entrypoint] AVISO: src/ vazio ou não montado — pulando o colcon build."
    echo "[entrypoint] Confira o volume no docker-compose.yml (ex: ../robis/src/:/root/ros2_ws/src/)."
fi

exec "$@"

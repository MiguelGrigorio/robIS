#!/usr/bin/env bash
set -euo pipefail
profile="${ROBIS_ROBOT_PROFILE:-basic}"
case "$profile" in basic|navigation) ;; *) echo 'Perfil deve ser basic ou navigation.' >&2; exit 2 ;; esac
image="${ROBIS_ROBOT_IMAGE:-robis:jazzy-$profile}"
if docker container inspect robis >/dev/null 2>&1; then
    echo 'O container robis já existe. Consulte docker logs robis ou use docker start robis.' >&2
    echo 'Para usar uma imagem nova, pare e remova o container antigo primeiro.' >&2
    exit 1
fi
arguments=()
command=()
if [[ "$profile" == navigation ]]; then
    task_data="${ROBIS_ROBOT_DATA_DIR:-$HOME/robis/data}"
    if [[ ! -f "$task_data/calibration/robot_camera.yaml" ]]; then
        echo "Forneça a calibração USB em $task_data/calibration/robot_camera.yaml antes de iniciar." >&2
        exit 2
    fi
    task_data=$(cd -- "$task_data" && pwd)
    arguments+=(-v "$task_data:/robis_ws/data")
    command=(ros2 launch robis_bringup semantic_real.launch.py
        "mode:=${ROBIS_REAL_MODE:-mapping}"
        "map:=/robis_ws/data/navigation/map.yaml"
        "camera_info_url:=/robis_ws/data/calibration/robot_camera.yaml"
        "map_checksum:=${ROBIS_MAP_CHECKSUM:-\"\"}")
fi
docker run -d --name robis --init --runtime runc --network host --privileged \
    --stop-signal SIGINT \
    -e ROS_DOMAIN_ID="${ROBIS_ROBOT_DOMAIN_ID:-27}" \
    -v /dev:/dev -v /run/udev:/run/udev:ro "${arguments[@]}" "$image" "${command[@]}"
echo 'Iniciado. Logs: docker logs -f robis. Parar: docker stop robis.'

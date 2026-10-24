#!/usr/bin/env bash
# Run the public package tests in an isolated container, without GPU or devices.
set -euo pipefail
cd /robis_ws
bash scripts/build_workspace.sh simulation
# ROS setup scripts can reference unset variables.
set +u
source install/setup.bash
set -u
python3 -m pytest -q -o cache_dir=/tmp/robis-pytest \
    src/robis_description/test src/robis_gazebo/test \
    src/robis_vision/test src/robis_semantic_navigation/test \
    src/robis_odrive/test/test_driver_ros.py

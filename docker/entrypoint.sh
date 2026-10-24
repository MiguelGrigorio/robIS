#!/usr/bin/env bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd /robis_ws
# Only Compose development services enable this. Production never builds at boot.
if [[ "${ROBIS_BUILD_ON_START:-false}" == true ]]; then
    /robis_ws/scripts/build_workspace.sh "${ROBIS_BUILD_PROFILE:-basic}" --symlink
fi
if [[ -f /robis_ws/install/setup.bash ]]; then
    source /robis_ws/install/setup.bash
fi
exec "$@"

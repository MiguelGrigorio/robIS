#!/usr/bin/env bash
set -e
profile="${1:-basic}"
case "$profile" in basic|navigation|simulation) ;; *) echo "Perfil inválido: $profile" >&2; exit 2 ;; esac
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$script_dir/.." && pwd)"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "$workspace_dir"
mapfile -t packages < "config/packages.$profile"
package_paths=()
for package in "${packages[@]}"; do
    package_paths+=("src/$package")
done
options=()
if [[ "${2:-}" == --symlink ]]; then
    options+=(--symlink-install)
fi
if [[ "$profile" != simulation ]]; then
    options+=(--cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
        -Dydlidar_sdk_DIR=/opt/ydlidar-sdk/lib/cmake/ydlidar_sdk)
fi
# Sequential package execution and one CMake worker fit the Nano's 4 GB budget.
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
# Colcon's Make generator otherwise appends -j<CPU count>, overriding CMake's
# environment limit. MAKEFLAGS makes it respect the same worker budget.
export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL}}"
# Discover only this profile: otherwise Colcon requires excluded local drivers
# when preparing the environment of packages such as robis_bringup.
colcon build --executor sequential --base-paths "${package_paths[@]}" --packages-select "${packages[@]}" "${options[@]}"

#!/usr/bin/env bash
set -euo pipefail
owned="$1"
mode="${2:-full}"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
set -u
cd -- "$owned"
if [[ "$mode" == cpu ]]; then
  packages=(--packages-up-to xq_autonomy ego_planner)
else
  packages=(--packages-up-to xq_sim_bringup)
fi
export CMAKE_BUILD_PARALLEL_LEVEL="${IMPACT_BUILD_JOBS:-2}"
colcon --log-base "$owned/log" build --base-paths "$owned/source/src" \
  --build-base "$owned/build" --install-base "$owned/install" --executor sequential \
  "${packages[@]}" --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF

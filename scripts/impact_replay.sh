#!/usr/bin/env bash
set -euo pipefail
run="$1"; kind="$2"; install="$3"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
source "$install/setup.bash"
set -u
# Replay has its own ROS domain and Gazebo partition. No MAVROS/SITL is launched.
export ROS_DOMAIN_ID=168 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
export GZ_PARTITION="impact_replay_${USER}_$$"
if [[ "$kind" == rosbag ]]; then
  [[ -f "$run/rosbag/metadata.yaml" ]] || { echo "rosbag metadata missing" >&2; exit 2; }
  # The original /clock is recorded; do not add a second generated clock publisher.
  exec ros2 bag play "$run/rosbag"
else
  : "${ARDUPILOT_GAZEBO_ROOT:?Set the plugin resource path first}"
  [[ -d "$run/gz_record" ]] || { echo "Gazebo recording missing" >&2; exit 2; }
  export GZ_SIM_RESOURCE_PATH="$install/xq_gz_assets/share/xq_gz_assets/models:$ARDUPILOT_GAZEBO_ROOT/models:$ARDUPILOT_GAZEBO_ROOT/worlds"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_ROOT/build"
  exec gz sim --playback "$run/gz_record"
fi

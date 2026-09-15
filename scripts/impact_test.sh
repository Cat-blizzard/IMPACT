#!/usr/bin/env bash
set -euo pipefail
root="$1"
owned="$2"
set +u
source /opt/ros/humble/setup.bash
source "$owned/install/setup.bash"
set -u
export PYTHONPATH="$root/src/xq_autonomy:$root/scripts:${PYTHONPATH:-}"
cd -- "$root"
python3 -m pytest -q --rootdir "$root" "$root/src/xq_autonomy/test" "$root/tests"
export ROS_DOMAIN_ID=169 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
python3 "$root/scripts/impact_ros_contract.py"

#!/usr/bin/env bash
set -euo pipefail
run="$1"
profile="$2"
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
: "${IMPACT_INSTALL:?}" "${ARDUPILOT_ROOT:?}" "${ARDUPILOT_GAZEBO_ROOT:?}"
set +u
source /opt/ros/humble/setup.bash
source "$IMPACT_INSTALL/setup.bash"
set -u
export GZ_SIM_RESOURCE_PATH="$IMPACT_INSTALL/xq_gz_assets/share/xq_gz_assets/models:$ARDUPILOT_GAZEBO_ROOT/models:$ARDUPILOT_GAZEBO_ROOT/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_ROOT/build"
export SDF_PATH="$GZ_SIM_RESOURCE_PATH"
mkdir -p -- "$run/ros_logs" "$run/sitl_runtime"
pids=()
phase="init"
failure_trap() {
  rc=$?
  printf '{"phase":"%s","line":%s,"command":"%s","exit_code":%s}\n' "$phase" "$1" "$2" "$rc" >"$run/first-failure.json"
  return "$rc"
}
trap 'failure_trap "$LINENO" "$BASH_COMMAND"' ERR
cleanup() {
  status=$?
  date -Is >"$run/cleanup-started-at.txt"
  printf 'exit_code=%s phase=%s\n' "$status" "$phase" >>"$run/cleanup-started-at.txt"
  for pid in "${pids[@]}"; do printf 'pid=%s alive=%s\n' "$pid" "$(kill -0 "$pid" 2>/dev/null && echo true || echo false)" >>"$run/cleanup-started-at.txt"; done
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  for _ in {1..10}; do
    alive=false
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive=true; done
    [[ "$alive" == false ]] && break
    sleep 1
  done
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; done
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() {
  label="$1"; shift
  setsid "$@" >"$run/$label.log" 2>&1 < /dev/null &
  pid=$!
  pids+=("$pid")
  echo "$pid" >"$run/$label.pid"
}
wait_log() {
  deadline=$((SECONDS+$3))
  until grep -Fq -- "$2" "$run/$1.log"; do
    ((SECONDS < deadline)) || { echo "Timeout: $1 $2" >&2; exit 2; }
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null || { echo "Child exited: $pid" >&2; exit 3; }; done
    sleep 0.5
  done
}
sha256sum "$ARDUPILOT_ROOT/build/sitl/bin/arducopter" \
 "$ARDUPILOT_GAZEBO_ROOT/build/libArduPilotPlugin.so" >"$run/external-binaries.sha256"
git -C "$ARDUPILOT_ROOT" rev-parse HEAD >"$run/ardupilot-revision.txt"
git -C "$ARDUPILOT_GAZEBO_ROOT" rev-parse HEAD >"$run/ardupilot-gazebo-revision.txt"
dpkg-query -W 'ros-humble-*' 'libgz-*' 'gz-*' >"$run/system-packages.txt" 2>&1 || true
if [[ "$profile" == server_gpu ]]; then
  phase="start_gpu_monitor"
  start gpu_monitor nvidia-smi --query-gpu=timestamp,name,uuid,utilization.gpu,memory.used --format=csv -l 1
fi
phase="start_sitl"
pushd "$run/sitl_runtime" >/dev/null
start sitl "$ARDUPILOT_ROOT/build/sitl/bin/arducopter" -S --model JSON --speedup 1 --slave 0 --wipe \
 --defaults "$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm,$ARDUPILOT_ROOT/Tools/autotest/default_params/gazebo-iris.parm,$IMPACT_INSTALL/xq_autonomy/share/xq_autonomy/config/xq_p4_extnav.parm" \
 --sim-address=127.0.0.1 -I0
popd >/dev/null
phase="wait_sitl"
wait_log sitl "SERIAL0 on TCP port 5760" 30
phase="start_mavros"
start mavros ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 namespace:=uav1/mavros
phase="start_gazebo"
gz_args=(-r -s -v 4 --record-path "$run/gz_record" --record-period 0.05)
[[ "$profile" == local_cpu ]] && gz_args+=(--headless-rendering)
start gazebo gz sim "${gz_args[@]}" "$run/world.sdf"
wait_log sitl "JSON received" 90
wait_log mavros "Got HEARTBEAT" 60
phase="start_rosbag"
start rosbag ros2 bag record -o "$run/rosbag" \
 /clock /tf /tf_static /livox/lidar /livox/imu /localization/odom /localization/geometry \
 /xq/p5/cloud_map /integrity/directional /integrity/information_map \
 /impact/planner_goal /impact/planner_candidate /impact/certified_bspline \
 /impact/authorization /impact/position_cmd /impact/mission_stage /impact/status \
 /impact/arbiter_status /uav1/mavros/state /uav1/mavros/local_position/odom \
 /uav1/mavros/setpoint_position/local /xq/eval/p5/ground_truth /xq/p4/extnav/status
start stack ros2 launch xq_sim_bringup impact_sitl.launch.py run_dir:="$run"
phase="wait_localization"
python3 "$root/scripts/wait_for_odometry.py" --topic /localization/odom --timeout 120 \
  --output "$run/first-odom.json" --diagnostics "$run/first-odom-diagnostics.json"
phase="wait_integrity"
timeout 30 ros2 topic echo --no-daemon --once /integrity/directional >"$run/first-integrity.txt"
phase="audit_runtime_graph"
ros2 topic info /xq/eval/p5/ground_truth -v >"$run/truth-graph.txt"
ros2 topic info /uav1/mavros/setpoint_position/local -v >"$run/setpoint-graph.txt"
ros2 topic info /xq/p4/extnav/status -v >"$run/extnav-status-graph.txt"
ros2 topic info /uav1/mavros/odometry/out -v >"$run/extnav-output-graph.txt"
audit_args=("$run" "$profile")
[[ "${IMPACT_SMOKE_ONLY:-0}" == 1 ]] && audit_args+=(--smoke)
python3 "$root/scripts/impact_runtime_audit.py" "${audit_args[@]}"
if [[ "${IMPACT_SMOKE_ONLY:-0}" == 1 ]]; then
  phase="smoke_observation"
  started=$(date +%s)
  sleep 30
  cat >"$run/smoke.json" <<EOF
{"mode":"NO_ARM_NO_TAKEOFF","observation_window_s":30,"started_epoch":$started,"mission_started":false,"arm_requested":false,"takeoff_requested":false}
EOF
  exit 0
fi
session="$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["session_id"])' "$run/run.json")"
task_timeout="$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["configuration"]["task_timeout_sim_s"])' "$run/run.json")"
start mission ros2 run xq_autonomy impact_mission --ros-args \
 -p use_sim_time:=true -p session_id:="$session" -p result_file:="$run/mission.json" \
 -p mission_timeout_s:=700.0 -p task_timeout_sim_s:="$task_timeout" \
 -r /uav1/mavros/setpoint_position/local:=/impact/mission_hold
deadline=$((SECONDS+720))
while [[ ! -f "$run/mission.json" ]]; do
  ((SECONDS < deadline)) || { echo "Mission result missing" >&2; exit 4; }
  for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null || { echo "Child exited: $pid" >&2; exit 5; }; done
  sleep 1
done
sleep 2
[[ -s "$run/evaluation.json" ]] || { echo "Evaluation missing" >&2; exit 6; }
sha256sum -c "$run/external-binaries.sha256" >"$run/external-assets-audit.txt"

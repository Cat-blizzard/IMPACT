#!/usr/bin/env bash
set -Eeuo pipefail
run="$1"
profile="$2"
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
: "${IMPACT_INSTALL:?}" "${ARDUPILOT_ROOT:?}" "${ARDUPILOT_GAZEBO_ROOT:?}"
set +u
source /opt/ros/humble/setup.bash
source "$IMPACT_INSTALL/setup.bash"
set -u
python3 "$root/scripts/verify_runtime_build.py" --install "$IMPACT_INSTALL" \
  --manifest "${IMPACT_BUILD_MANIFEST:-$IMPACT_INSTALL/../build-manifest.json}" \
  --output "$run/runtime-build-verification.json"
export GZ_SIM_RESOURCE_PATH="$IMPACT_INSTALL/xq_gz_assets/share/xq_gz_assets/models:$ARDUPILOT_GAZEBO_ROOT/models:$ARDUPILOT_GAZEBO_ROOT/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_ROOT/build"
export SDF_PATH="$GZ_SIM_RESOURCE_PATH"
mkdir -p -- "$run/ros_logs" "$run/sitl_runtime"
pids=()
labels=()
phase="init"
failure_trap() {
  rc=$?
  printf '{"phase":"%s","line":%s,"command":"%s","exit_code":%s}\n' "$phase" "$1" "$2" "$rc" >"$run/first-failure.json"
  return "$rc"
}
trap 'failure_trap "$LINENO" "$BASH_COMMAND"' ERR
cleanup() {
  status=$?
  set +e
  # Child launchers conventionally return their signal status during normal
  # cleanup.  They are cleanup outcomes, not a new first failure.
  trap - ERR
  date -Is >"$run/cleanup-started-at.txt"
  printf 'exit_code=%s phase=%s\n' "$status" "$phase" >>"$run/cleanup-started-at.txt"
  for pid in "${pids[@]}"; do printf 'pid=%s alive=%s\n' "$pid" "$(kill -0 "$pid" 2>/dev/null && echo true || echo false)" >>"$run/cleanup-started-at.txt"; done
  trap - EXIT INT TERM
  : >"$run/cleanup-processes.jsonl"
  cleanup_initial=()
  cleanup_signals=()
  group_running() {
    ps -eo pgid=,stat= | awk -v group="$1" '$1 == group && $2 !~ /^Z/ {found=1} END {exit !found}'
  }
  record_group() {
    local group="$1" label="$2" stage="$3"
    ps -eo pid=,ppid=,pgid=,sid=,stat=,etimes=,cmd= | awk \
      -v group="$group" -v label="$label" -v stage="$stage" \
      'BEGIN {OFS="\t"} $3 == group {print stage,label,$0}' \
      >>"$run/cleanup-process-groups.tsv"
  }
  printf 'stage\tlabel\tpid\tppid\tpgid\tsid\tstat\tetimes\tcmd\n' \
    >"$run/cleanup-process-groups.tsv"
  for index in "${!pids[@]}"; do
    pid="${pids[index]}"
    initial_alive=false
    kill -0 "$pid" 2>/dev/null && initial_alive=true
    signal=INT
    if [[ -f "$run/gazebo.pid" && "$pid" == "$(cat "$run/gazebo.pid")" ]]; then
      signal=TERM
    elif [[ "${labels[index]}" == sitl ]]; then
      signal=TERM
    fi
    cleanup_initial+=("$initial_alive")
    cleanup_signals+=("$signal")
  done
  # SITL only checks its exit flag from the main loop. Keep Gazebo feeding the
  # JSON backend while SITL handles TERM, then stop launchers and Gazebo once.
  for index in "${!pids[@]}"; do
    [[ "${labels[index]}" == sitl && "${cleanup_initial[index]}" == true ]] || continue
    kill -TERM "${pids[index]}" 2>/dev/null || true
    for _ in {1..5}; do
      group_running "${pids[index]}" || break
      sleep 1
    done
  done
  for index in "${!pids[@]}"; do
    [[ "${labels[index]}" != sitl && "${cleanup_initial[index]}" == true ]] || continue
    kill -"${cleanup_signals[index]}" "${pids[index]}" 2>/dev/null || true
  done
  for _ in {1..10}; do
    alive=false
    for pid in "${pids[@]}"; do group_running "$pid" && alive=true; done
    [[ "$alive" == false ]] && break
    sleep 1
  done
  for index in "${!pids[@]}"; do
    pid="${pids[index]}"
    if group_running "$pid"; then
      record_group "$pid" "${labels[index]}" before_term
      kill -TERM -- "-$pid" 2>/dev/null || true
    fi
  done
  sleep 2
  for index in "${!pids[@]}"; do
    pid="${pids[index]}"
    if group_running "$pid"; then
      record_group "$pid" "${labels[index]}" before_kill
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null
    wait_status=$?
    # A launch parent can be reaped before every child in its process group has
    # observed SIGKILL.  Give the kernel a bounded interval to remove those
    # members; only a group still running after this interval is a residual.
    for _ in {1..10}; do
      group_running "$pid" || break
      sleep 0.5
    done
    residual=false
    if group_running "$pid"; then
      residual=true
      record_group "$pid" "${labels[index]}" residual
    fi
    python3 - "${labels[index]}" "$pid" "${cleanup_initial[index]}" \
      "${cleanup_signals[index]}" "$wait_status" "$residual" >>"$run/cleanup-processes.jsonl" <<'PY'
import json,sys
label,pid,initial,signal,status,residual=sys.argv[1:]
print(json.dumps({'label':label,'pid':int(pid),'initial_alive':initial=='true',
 'signal':signal,'wait_status':int(status),'residual':residual=='true'},separators=(',',':')))
PY
  done
  find "$run/sitl_runtime/logs" -maxdepth 1 -type f -name '*.BIN' -print0 2>/dev/null \
    | sort -z | xargs -0 -r sha256sum >"$run/dataflash.sha256"
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
  labels+=("$label")
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
graph_probe() {
  local topic="$1" output="$2" expected="${3:-}" attempt=0 rc=0
  local deadline=$((SECONDS+20)) attempt_output records
  records="${output%.txt}-attempts.jsonl"
  : >"$records"
  while true; do
    attempt=$((attempt+1))
    attempt_output="${output%.txt}-attempt-${attempt}.txt"
    set +e
    timeout 10 ros2 topic info --no-daemon --spin-time 3 "$topic" -v \
      >"$attempt_output" 2>&1
    rc=$?
    set -e
    cp -- "$attempt_output" "$output"
    python3 - "$records" "$attempt" "$topic" "$rc" "${ROS_DOMAIN_ID:-0}" <<'PY'
import datetime,json,sys
path,attempt,topic,status,domain=sys.argv[1:]
with open(path,"a",encoding="utf-8") as stream:
    stream.write(json.dumps({"attempt":int(attempt),"collected_at":datetime.datetime.now().astimezone().isoformat(),
        "command":["ros2","topic","info","--no-daemon","--spin-time","3",topic,"-v"],
        "exit_code":int(status),"ros_domain_id":domain},separators=(",",":"))+"\n")
PY
    if [[ "$rc" == 0 && ( -z "$expected" || "$expected" == "$(grep -Fom1 -- "$expected" "$output" || true)" ) ]]; then
      break
    fi
    ((SECONDS < deadline)) || break
    sleep 0.5
  done
  [[ "$rc" == 0 ]] || printf 'probe_exit_nonzero=true\n' >>"$output"
  printf '%s\n' "$(date -Is)" >"${output%.txt}-collected-at.txt"
}
sha256sum "$ARDUPILOT_ROOT/build/sitl/bin/arducopter" \
 "$ARDUPILOT_GAZEBO_ROOT/build/libArduPilotPlugin.so" >"$run/external-binaries.sha256"
git -C "$ARDUPILOT_ROOT" rev-parse HEAD >"$run/ardupilot-revision.txt"
git -C "$ARDUPILOT_GAZEBO_ROOT" rev-parse HEAD >"$run/ardupilot-gazebo-revision.txt"
dpkg-query -W 'ros-humble-*' 'libgz-*' 'gz-*' >"$run/system-packages.txt" 2>&1 || true
phase="verify_renderer"
timeout 10 glxinfo -B >"$run/renderer.txt" 2>&1 || {
  echo "Unable to record the configured renderer." >&2
  exit 2
}
if [[ "$profile" == local_cpu ]] && ! grep -Eiq 'llvmpipe|software rasterizer' "$run/renderer.txt"; then
  echo "local_cpu did not resolve a software renderer." >&2
  exit 2
fi
if [[ "$profile" == server_gpu ]]; then
  if ! grep -Eiq 'renderer.*nvidia|device.*nvidia' "$run/renderer.txt" \
      || grep -Eiq 'llvmpipe|softpipe|software rasterizer' "$run/renderer.txt"; then
    echo "server_gpu did not resolve the required NVIDIA hardware renderer." >&2
    exit 2
  fi
fi
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
 /xq/p5/cloud_map /xq/p5/navigation_map /xq/p5/exploration/status \
 /impact/information_cloud /grid_map/occupancy_inflate \
 /integrity/directional /integrity/debug /integrity/information_map \
 /impact/planner_goal /impact/planner_candidate /impact/certified_bspline \
 /impact/authorization /impact/position_cmd /impact/mission_stage /impact/status \
 /impact/arbiter_status /uav1/mavros/state /uav1/mavros/extended_state \
 /uav1/mavros/local_position/odom \
 /uav1/mavros/odometry/out /uav1/mavros/statustext/recv /uav1/mavros/sys_status \
 /uav1/mavros/estimator_status \
 /uav1/mavros/imu/data \
 /uav1/mavros/setpoint_position/local /uav1/mavros/setpoint_raw/local \
 /xq/eval/p5/ground_truth /xq/p4/extnav/status
start stack ros2 launch xq_sim_bringup impact_sitl.launch.py run_dir:="$run"
phase="wait_localization"
python3 "$root/scripts/wait_for_odometry.py" --topic /localization/odom --timeout 120 \
  --output "$run/first-odom.json" --diagnostics "$run/first-odom-diagnostics.json"
phase="wait_integrity"
python3 "$root/scripts/wait_for_integrity.py" --topic /integrity/directional --timeout 30 \
  --output "$run/first-integrity.json" --diagnostics "$run/first-integrity-diagnostics.json"
phase="audit_runtime_graph"
# Every graph probe below explicitly bypasses the ROS daemon.  Controlling the
# daemon here is both unnecessary and unsafe: `ros2 daemon stop` can itself
# block indefinitely, preventing the bounded probes from ever running.
printf '%s\n' 'bypassed: graph probes use --no-daemon; no daemon control issued' \
  >"$run/ros-daemon-control.txt"
graph_probe /xq/eval/p5/ground_truth "$run/truth-graph.txt" "Node name: impact_evaluator"
graph_probe /uav1/mavros/setpoint_raw/local "$run/setpoint-graph.txt" "Node name: impact_arbiter"
graph_probe /xq/p4/extnav/status "$run/extnav-status-graph.txt" "Node name: xq_p4_external_nav"
graph_probe /uav1/mavros/odometry/out "$run/extnav-output-graph.txt" "Node name: odometry"
graph_probe /uav1/mavros/state "$run/mavros-state-graph.txt" "Node name: sys"
timeout 15 ros2 node list --no-daemon --spin-time 2 >"$run/ros-nodes-prearm.txt" 2>&1
printf '%s\n' "$(date -Is)" >"$run/ros-nodes-prearm-collected-at.txt"
timeout 15 ros2 service list --no-daemon --spin-time 2 -t >"$run/ros-services-prearm.txt" 2>&1
printf '%s\n' "$(date -Is)" >"$run/ros-services-prearm-collected-at.txt"
audit_args=("$run" "$profile")
[[ "${IMPACT_SMOKE_ONLY:-0}" == 1 ]] && audit_args+=(--smoke)
python3 "$root/scripts/impact_runtime_audit.py" "${audit_args[@]}"
if [[ "${IMPACT_SMOKE_ONLY:-0}" == 1 ]]; then
  phase="smoke_node_audit"
  timeout 10 ros2 node list --no-daemon --spin-time 2 >"$run/runtime-nodes.txt"
  printf '%s\n' "$(date -Is)" >"$run/runtime-nodes-collected-at.txt"
  if grep -Eq '(^|/)impact_mission($|/)' "$run/runtime-nodes.txt"; then
    echo "Smoke mode unexpectedly started impact_mission" >&2
    exit 7
  fi
  phase="smoke_observation"
  python3 "$root/scripts/startup_smoke_monitor.py" --seconds 30 --output "$run/smoke-observation.json"
  phase="smoke_bag_validation"
  rosbag_pid="$(cat "$run/rosbag.pid")"
  kill -INT -- "-$rosbag_pid" 2>/dev/null || true
  set +e
  wait "$rosbag_pid" 2>/dev/null
  rosbag_status=$?
  set -e
  printf 'rosbag_pid=%s wait_status=%s\n' "$rosbag_pid" "$rosbag_status" >"$run/smoke-rosbag-stop.txt"
  timeout 20 ros2 bag info "$run/rosbag" >"$run/smoke-bag-info.txt"
  python3 - "$run" <<'PY'
import json, pathlib, sys
r=pathlib.Path(sys.argv[1]); obs=json.loads((r/'smoke-observation.json').read_text())
info=(r/'smoke-bag-info.txt').read_text()
gaz=(r/'gazebo.log').read_text(errors='replace')
launch=(r/'launcher.log').read_text(errors='replace')
nodes=(r/'runtime-nodes.txt').read_text().splitlines()
result={'mode':'NO_ARM_NO_TAKEOFF','observation':obs,'bag_readable':bool(info.strip()),
        'mission_node_absent':not any(x.rstrip('/').endswith('/impact_mission') or x == 'impact_mission' for x in nodes),
        'gazebo_exit_evidence':{'signaled': 'Segmentation fault' in gaz or 'core dumped' in gaz,
                                'launcher_signaled': 'Segmentation fault' in launch or 'core dumped' in launch,
                                'log_tail':(gaz+'\n'+launch)[-1000:]}}
result['passed']=result['bag_readable'] and result['mission_node_absent'] and obs.get('passed') is True and not result['gazebo_exit_evidence']['signaled'] and not result['gazebo_exit_evidence']['launcher_signaled']
(r/'smoke.json').write_text(json.dumps(result,indent=2)+'\n')
raise SystemExit(0 if result['passed'] else 1)
PY
  exit 0
fi
session="$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["session_id"])' "$run/run.json")"
task_timeout="$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["configuration"]["task_timeout_sim_s"])' "$run/run.json")"
mission_timeout_s=700
termination_timeout_s=90
start mission ros2 run xq_autonomy impact_mission --ros-args \
 -p use_sim_time:=true -p session_id:="$session" -p result_file:="$run/mission.json" \
 -p mission_timeout_s:="${mission_timeout_s}.0" -p task_timeout_sim_s:="$task_timeout" \
 -p failsafe_termination_timeout_s:="${termination_timeout_s}.0" \
 -r /uav1/mavros/setpoint_position/local:=/impact/mission_hold
deadline=$((SECONDS+mission_timeout_s+termination_timeout_s+20))
while [[ ! -f "$run/mission.json" ]]; do
  ((SECONDS < deadline)) || { echo "Mission result missing" >&2; exit 4; }
  for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null || { echo "Child exited: $pid" >&2; exit 5; }; done
  sleep 1
done
sleep 2
[[ -s "$run/evaluation.json" ]] || { echo "Evaluation missing" >&2; exit 6; }
sha256sum -c "$run/external-binaries.sha256" >"$run/external-assets-audit.txt"

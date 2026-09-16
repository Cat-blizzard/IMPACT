#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/.." && pwd -P)"
install_root="${IMPACT_INSTALL:-${workspace_root}/xq_install}"
ardupilot_root="${ARDUPILOT_ROOT:-/home/accelerate/ardupilot}"
plugin_root="${ARDUPILOT_GAZEBO_ROOT:-/home/accelerate/ardupilot_gazebo}"
build_manifest="${IMPACT_BUILD_MANIFEST:-${install_root}/.xq_build_manifest.json}"
requested_run_dir=""
profile="server_gpu"
minimum_eval_duration_s=70
smoke_only=false
observation_seconds=45

usage() {
  echo "Usage: $0 [--profile local_cpu|server_gpu] [--minimum-eval-duration SECONDS] [--run-dir PATH] [--smoke-only] [--observation-seconds SECONDS]" >&2
}

while (($#)); do
  case "$1" in
    --profile) profile="$2"; shift 2 ;;
    --minimum-eval-duration) minimum_eval_duration_s="$2"; shift 2 ;;
    --run-dir) requested_run_dir="$2"; shift 2 ;;
    --smoke-only) smoke_only=true; shift ;;
    --observation-seconds) observation_seconds="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done
[[ "$profile" == local_cpu || "$profile" == server_gpu ]] || { echo "Invalid profile" >&2; exit 2; }
[[ "${minimum_eval_duration_s}" =~ ^[1-9][0-9]*$ ]] || {
  echo "Invalid minimum evaluation duration." >&2; exit 2;
}
((minimum_eval_duration_s >= 60)) || {
  echo "P4 evaluation must cover at least 60 seconds." >&2; exit 2;
}
[[ "${observation_seconds}" =~ ^[1-9][0-9]*$ ]] || {
  echo "Invalid smoke observation duration." >&2; exit 2;
}
((observation_seconds >= 30)) || {
  echo "P4 smoke observation must cover at least 30 seconds." >&2; exit 2;
}

for required in \
  "${install_root}/setup.bash" \
  "${workspace_root}/scripts/audit_external_assets.sh" \
  "${install_root}/xq_autonomy/share/xq_autonomy/config/xq_p4_extnav.parm" \
  "${ardupilot_root}/build/sitl/bin/arducopter" \
  "${plugin_root}/build/libArduPilotPlugin.so"; do
  [[ -e "${required}" ]] || { echo "Missing dependency: ${required}" >&2; exit 2; }
done

if ss -H -ltnp | grep -Eq '(:|\])5760[[:space:]]'; then
  echo "TCP 5760 is already in use; refusing to interfere with another SITL." >&2
  exit 3
fi
if ss -H -lunp | grep -Eq '(:|\])9002[[:space:]]'; then
  echo "UDP 9002 is already in use; refusing to interfere with another Gazebo vehicle." >&2
  exit 3
fi

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -n "${requested_run_dir}" ]]; then
  run_dir="$(realpath -m -- "${requested_run_dir}")"
else
  run_dir="${workspace_root}/experiments/results/external_nav/p4_${timestamp}_$$"
fi
case "${run_dir}" in
  "${workspace_root}"/experiments/results/external_nav/*) ;;
  *) echo "Run directory must stay below experiments/results/external_nav." >&2; exit 2 ;;
esac
[[ ! -e "${run_dir}" ]] || { echo "Refusing to reuse run directory: ${run_dir}" >&2; exit 2; }
mkdir -p -- "${run_dir}/ros_logs" "${run_dir}/sitl_runtime"
phase="setup"
failure_trap() {
  local rc="$1" line="$2" command="$3"
  if [[ ! -e "${run_dir}/first-failure.json" ]]; then
    python3 - "${run_dir}/first-failure.json" "${phase}" "${line}" "${command}" "${rc}" <<'PY'
import datetime,json,pathlib,sys
path,phase,line,command,code=sys.argv[1:]
pathlib.Path(path).write_text(json.dumps({"phase":phase,"line":int(line),
    "command":command,"exit_code":int(code),
    "recorded_at_utc":datetime.datetime.now(datetime.timezone.utc).isoformat()},
    ensure_ascii=False,indent=2)+"\n")
PY
  fi
  return "${rc}"
}
trap 'failure_trap "$?" "$LINENO" "$BASH_COMMAND"' ERR

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH SDF_PATH
unset RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
set +u
source /opt/ros/humble/setup.bash
source "${install_root}/setup.bash"
set -u

# Refuse stale/wrong installations before starting SITL, Gazebo or any task.
phase="verify_runtime_build"
python3 "${script_dir}/verify_runtime_build.py" --install "${install_root}" \
  --manifest "${build_manifest}" --output "${run_dir}/runtime-build-verification.json"

export ROS_DOMAIN_ID=$((102 + (10#$(date +%S) + $$) % 80))
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1
export GZ_PARTITION="xq_p4_${USER:-wsl}_${timestamp}_$$"
export ROS_LOG_DIR="${run_dir}/ros_logs"
if [[ "$profile" == local_cpu ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe GALLIUM_DRIVER=llvmpipe
  export EGL_PLATFORM=surfaceless QT_QPA_PLATFORM=offscreen
else
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
fi
phase="verify_renderer"
timeout 10 glxinfo -B >"${run_dir}/renderer.txt" 2>&1 || {
  echo "Unable to record the configured renderer." >&2; exit 2;
}
if [[ "$profile" == local_cpu ]] && ! grep -Eiq 'llvmpipe|software rasterizer' "${run_dir}/renderer.txt"; then
  echo "local_cpu did not resolve a software renderer." >&2; exit 2
fi
export GZ_SIM_RESOURCE_PATH="${install_root}/xq_gz_assets/share/xq_gz_assets/models:${plugin_root}/models:${plugin_root}/worlds"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${plugin_root}/build"

world="${install_root}/xq_gz_assets/share/xq_gz_assets/worlds/xq_p4_external_nav.sdf"
p4_params="${install_root}/xq_autonomy/share/xq_autonomy/config/xq_p4_extnav.parm"
mission_result="${run_dir}/mission-result.json"
evaluation_result="${run_dir}/localization-evaluation.json"
[[ -f "${world}" ]] || { echo "Installed P4 world is missing." >&2; exit 2; }

cat >"${run_dir}/run.env" <<EOF
run_started_utc=${timestamp}
mode=$([[ "${smoke_only}" == true ]] && echo NO_ARM_SMOKE || echo P4_FLIGHT)
minimum_eval_duration_s=${minimum_eval_duration_s}
observation_seconds=${observation_seconds}
profile=${profile}
ros_domain_id=${ROS_DOMAIN_ID}
gz_partition=${GZ_PARTITION}
world=${world}
install_root=${install_root}
ardupilot_root=${ardupilot_root}
plugin_root=${plugin_root}
build_manifest=${build_manifest}
mavros_namespace=/uav1/mavros
external_nav_topic=/uav1/mavros/odometry/out
gps_disabled=GPS_TYPE:0,SIM_GPS_DISABLE:1
sitl_runtime_dir=${run_dir}/sitl_runtime
EOF

sha256sum \
  "${world}" \
  "${p4_params}" \
  "${install_root}/xq_gz_assets/share/xq_gz_assets/models/xq_iris_mid360_ardupilot/model.sdf" \
  "${install_root}/xq_fast_lio/share/xq_fast_lio/config/xq_p4.yaml" \
  "${install_root}/xq_gz_bridge/share/xq_gz_bridge/config/p4_external_nav.yaml" \
  "${ardupilot_root}/build/sitl/bin/arducopter" \
  "${ardupilot_root}/Tools/autotest/default_params/copter.parm" \
  "${ardupilot_root}/Tools/autotest/default_params/gazebo-iris.parm" \
  "${plugin_root}/build/libArduPilotPlugin.so" \
  "${plugin_root}/models/iris_with_ardupilot/model.sdf" \
  "${plugin_root}/models/iris_with_standoffs/model.sdf" \
  >"${run_dir}/runtime-dependencies.sha256"
if [[ -f "${build_manifest}" ]]; then
  cp -- "${build_manifest}" "${run_dir}/xq-build-manifest.json"
else
  echo "Build manifest not found: ${build_manifest}" >&2
  exit 2
fi

before_audit="${run_dir}/external-assets.before.sha256"
after_audit="${run_dir}/external-assets.after.sha256"
bash "${script_dir}/audit_external_assets.sh" snapshot "${before_audit}" >/dev/null

declare -a pids=()
declare -a labels=()
cleanup_done=false
stop_done=false
stop_status=0

start_group() {
  local label="$1" log_file="$2"
  shift 2
  setsid "$@" >"${log_file}" 2>&1 < /dev/null &
  local pid=$!
  local pgid
  sleep 0.2
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
  [[ "${pgid}" == "${pid}" ]] || {
    echo "Failed to isolate process group for ${label}." >&2; return 4;
  }
  pids+=("${pid}")
  labels+=("${label}")
  echo "${pid}" >"${run_dir}/${label}.pid"
}

stop_groups() {
  [[ "${stop_done}" == false ]] || return "${stop_status}"
  stop_done=true
  local index pid round wait_status residual forced_kill
  local -a initial_alive=() signals=() forced_kills=()
  group_running() {
    ps -eo pgid=,stat= | awk -v group="$1" '$1 == group && $2 !~ /^Z/ {found=1} END {exit !found}'
  }
  record_group() {
    local group="$1" label="$2" stage="$3"
    ps -eo pid=,ppid=,pgid=,sid=,stat=,etimes=,cmd= | awk \
      -v group="$group" -v label="$label" -v stage="$stage" \
      'BEGIN {OFS="\t"} $3 == group {print stage,label,$0}' \
      >>"${run_dir}/cleanup-process-groups.tsv"
  }
  date -Is >"${run_dir}/cleanup-started-at.txt"
  printf 'phase=%s\n' "${phase}" >>"${run_dir}/cleanup-started-at.txt"
  : >"${run_dir}/cleanup-processes.jsonl"
  printf 'stage\tlabel\tpid\tppid\tpgid\tsid\tstat\tetimes\tcmd\n' \
    >"${run_dir}/cleanup-process-groups.tsv"
  for index in "${!pids[@]}"; do
    pid="${pids[index]}"
    if group_running "${pid}"; then initial_alive+=(true); else initial_alive+=(false); fi
    if [[ "${labels[index]}" == sitl || "${labels[index]}" == gazebo ]]; then
      signals+=(TERM)
    else
      signals+=(INT)
    fi
    forced_kills+=(false)
  done
  # SITL checks its termination flag in the main loop. Keep Gazebo serving the
  # JSON backend while SITL exits, then stop the remaining groups once.
  for index in "${!pids[@]}"; do
    [[ "${labels[index]}" == sitl && "${initial_alive[index]}" == true ]] || continue
    kill -TERM -- "-${pids[index]}" 2>/dev/null || true
    for _ in {1..10}; do group_running "${pids[index]}" || break; sleep 0.5; done
  done
  for index in "${!pids[@]}"; do
    [[ "${labels[index]}" != sitl && "${initial_alive[index]}" == true ]] || continue
    kill -"${signals[index]}" -- "-${pids[index]}" 2>/dev/null || true
  done
  for round in {1..20}; do
    local alive=false
    for pid in "${pids[@]}"; do group_running "${pid}" && alive=true; done
    [[ "${alive}" == false ]] && break
    sleep 0.5
  done
  for index in "${!pids[@]}"; do
    pid="${pids[index]}"
    if group_running "${pid}"; then
      record_group "${pid}" "${labels[index]}" before_term
      kill -TERM -- "-${pid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for index in "${!pids[@]}"; do
    pid="${pids[index]}"
    if group_running "${pid}"; then
      forced_kills[index]=true
      record_group "${pid}" "${labels[index]}" before_kill
      kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null
    wait_status=$?
    for _ in {1..10}; do group_running "${pid}" || break; sleep 0.5; done
    residual=false
    if group_running "${pid}"; then
      residual=true
      record_group "${pid}" "${labels[index]}" residual
    fi
    forced_kill="${forced_kills[index]}"
    python3 - "${labels[index]}" "${pid}" "${initial_alive[index]}" \
      "${signals[index]}" "${wait_status}" "${forced_kill}" "${residual}" \
      >>"${run_dir}/cleanup-processes.jsonl" <<'PY'
import json,sys
label,pid,initial,signal,status,forced,residual=sys.argv[1:]
print(json.dumps({'label':label,'pid':int(pid),'initial_alive':initial=='true',
 'signal':signal,'wait_status':int(status),'forced_kill':forced=='true',
 'residual':residual=='true'},separators=(',',':')))
PY
    if [[ "${residual}" == true || "${forced_kill}" == true ]]; then stop_status=12; fi
    case "${wait_status}" in 0|130|143) ;; *) stop_status=12 ;; esac
  done
  return "${stop_status}"
}

finish_audit() {
  [[ "${cleanup_done}" == false ]] || return 0
  bash "${script_dir}/audit_external_assets.sh" snapshot "${after_audit}" >/dev/null
  bash "${script_dir}/audit_external_assets.sh" compare \
    "${before_audit}" "${after_audit}" >"${run_dir}/isolation-audit.txt" 2>&1
  cleanup_done=true
}

inventory_dataflash() {
  find "${run_dir}/sitl_runtime/logs" -maxdepth 1 -type f -name '*.BIN' -print0 \
    | sort -z \
    | xargs -0 -r sha256sum >"${run_dir}/dataflash.sha256"
  find "${run_dir}/sitl_runtime/logs" -maxdepth 1 -type f -name '*.BIN' -printf '%f %s bytes\n' \
    | sort >"${run_dir}/dataflash.inventory.txt"
}

cleanup() {
  local status=$?
  if ((status != 0)) && [[ ! -e "${run_dir}/first-failure.json" ]]; then
    set +e
    failure_trap "${status}" "0" "explicit exit or signal in phase ${phase}"
  fi
  trap - EXIT INT TERM ERR
  set +e
  stop_groups
  local process_status=$?
  finish_audit
  local audit_status=$?
  inventory_dataflash
  ((process_status == 0)) || status="${process_status}"
  ((audit_status == 0)) || status="${audit_status}"
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_log() {
  local file="$1" pattern="$2" timeout_s="$3" label="$4"
  local deadline=$((SECONDS + timeout_s))
  while ((SECONDS < deadline)); do
    [[ -f "${file}" ]] && grep -Fq -- "${pattern}" "${file}" && return 0
    sleep 0.5
  done
  echo "Timeout waiting for ${label}: ${pattern}" >&2
  return 1
}

graph_probe() {
  local topic="$1" output="$2"
  timeout 15 ros2 topic info --no-daemon "${topic}" -v >"${output}" 2>&1
  date -Is >"${output%.txt}-collected-at.txt"
}

assert_core_alive() {
  local index
  for index in 0 1 2 3 4; do
    if ! kill -0 "${pids[index]}" 2>/dev/null; then
      echo "Core process exited early: ${labels[index]}" >&2
      return 1
    fi
  done
}

pushd "${run_dir}/sitl_runtime" >/dev/null
phase="start_sitl"
start_group sitl "${run_dir}/sitl.log" \
  "${ardupilot_root}/build/sitl/bin/arducopter" \
  -S --model JSON --speedup 1 --slave 0 --wipe \
  --defaults "${ardupilot_root}/Tools/autotest/default_params/copter.parm,${ardupilot_root}/Tools/autotest/default_params/gazebo-iris.parm,${p4_params}" \
  --sim-address=127.0.0.1 -I0
popd >/dev/null
wait_log "${run_dir}/sitl.log" "SERIAL0 on TCP port 5760" 30 "SITL MAVLink listener"

phase="start_mavros"
start_group mavros "${run_dir}/mavros.log" \
  ros2 launch mavros apm.launch \
  fcu_url:=tcp://127.0.0.1:5760 namespace:=uav1/mavros
wait_log "${run_dir}/sitl.log" "Loaded defaults" 45 "ArduPilot defaults"

gz_args=(-r -s --headless-rendering -v 3)
phase="start_gazebo"
start_group gazebo "${run_dir}/gazebo.log" gz sim "${gz_args[@]}" "${world}"
wait_log "${run_dir}/sitl.log" "JSON received" 90 "SITL-Gazebo JSON link"
wait_log "${run_dir}/mavros.log" "Got HEARTBEAT" 60 "MAVROS heartbeat"

# Keep the primary SQLite file directly readable for post-flight correlation.
# Compression can be performed on a verified copy after recording; a killed
# compression/finalization process must not make the only evidence unreadable.
phase="start_rosbag"
start_group rosbag "${run_dir}/rosbag.log" \
  ros2 bag record --compression-mode none \
  -o "${run_dir}/rosbag" \
  /clock /livox/lidar /livox/imu /localization/odom \
  /uav1/mavros/odometry/out /uav1/mavros/local_position/odom \
  /uav1/mavros/state /uav1/mavros/extended_state /uav1/mavros/statustext/recv \
  /uav1/mavros/sys_status /uav1/mavros/estimator_status /uav1/mavros/imu/data \
  /uav1/mavros/setpoint_position/local \
  /xq/p4/extnav/status /xq/eval/p4/ground_truth

phase="start_p4_stack"
start_group p4_stack "${run_dir}/p4-stack.log" \
  ros2 launch xq_sim_bringup xq_p4_external_nav.launch.py \
  evaluation_result_file:="${evaluation_result}" \
  minimum_duration_s:="${minimum_eval_duration_s}.0"

# Wait for the real algorithm output, not only process startup.  Use the
# diagnostic probe shared with P5 so QoS discovery failures are captured
# separately from a genuine FAST-LIO no-odom failure.
phase="wait_localization"
python3 "${script_dir}/wait_for_odometry.py" \
  --topic /localization/odom --timeout 100 \
  --output "${run_dir}/first-localization-odom.txt" \
  --diagnostics "${run_dir}/first-localization-odom.diagnostics.json" || {
    echo "FAST-LIO did not publish /localization/odom." >&2
    exit 5
  }
assert_core_alive

phase="audit_prearm_graph"
timeout 15 ros2 topic list --no-daemon -t >"${run_dir}/ros-topics.txt" 2>&1
timeout 15 ros2 node list --no-daemon >"${run_dir}/ros-nodes.txt" 2>&1
timeout 15 ros2 service list --no-daemon -t >"${run_dir}/ros-services.txt" 2>&1
timeout 15 gz topic -l >"${run_dir}/gz-topics.txt" 2>&1
graph_probe /xq/p4/extnav/status "${run_dir}/extnav-status-graph.txt"
graph_probe /uav1/mavros/odometry/out "${run_dir}/external-nav-topic-graph.txt"
graph_probe /uav1/mavros/odometry/in "${run_dir}/mavros-odometry-in-graph.txt"
graph_probe /xq/eval/p4/ground_truth "${run_dir}/ground-truth-topic-graph.txt"

python3 - "${script_dir}" "${run_dir}" <<'PY'
import json, pathlib, sys
sys.path.insert(0, sys.argv[1])
from impact_runtime_audit import _endpoints, check_extnav
run = pathlib.Path(sys.argv[2])
status = (run / "extnav-status-graph.txt").read_text()
output = (run / "external-nav-topic-graph.txt").read_text()
identity = (run / "mavros-odometry-in-graph.txt").read_text()
truth = (run / "ground-truth-topic-graph.txt").read_text()
extnav = check_extnav(status, output, identity)
recorder_participants = {x["participant_gid"] for x in extnav["output_subscribers"]
                         if x["node"].startswith("rosbag2_recorder")}
truth_subscribers = _endpoints(truth, "SUBSCRIPTION")
resolved = []
for endpoint in truth_subscribers:
    node = endpoint["node"]
    if node == "_NODE_NAME_UNKNOWN_" and endpoint["participant_gid"] in recorder_participants:
        node = "rosbag2_recorder@gid"
    resolved.append({**endpoint, "resolved_node": node})
truth_isolated = (any(item["resolved_node"] == "xq_p4_evaluator" for item in resolved) and all(
    item["resolved_node"] == "xq_p4_evaluator" or
    item["resolved_node"].startswith("rosbag2_recorder") for item in resolved))
report = {"extnav": extnav, "truth_endpoints": resolved,
          "truth_isolation": truth_isolated}
report["passed"] = bool(truth_isolated and extnav["status_single_publisher"] and
    extnav["output_single_adapter_publisher"] and extnav["mavros_output_subscription"])
(run / "p4-runtime-audit.json").write_text(json.dumps(report, indent=2) + "\n")
if not report["passed"]:
    raise SystemExit("P4 graph audit failed")
PY

grep -q '/uav1/mavros/odometry/out' "${run_dir}/ros-topics.txt" || {
  echo "MAVROS odometry input topic is absent." >&2; exit 6;
}
grep -q '/xq/p4/lidar/points' "${run_dir}/gz-topics.txt" || {
  echo "P4 LiDAR Gazebo topic is absent." >&2; exit 6;
}
grep -q '/xq/p4/imu' "${run_dir}/gz-topics.txt" || {
  echo "P4 IMU Gazebo topic is absent." >&2; exit 6;
}

if [[ "${smoke_only}" == true ]]; then
  phase="no_arm_observation"
  set +e
  timeout "$((observation_seconds + 20))" \
    python3 "${script_dir}/startup_smoke_monitor.py" \
      --seconds "${observation_seconds}" \
      --output "${run_dir}/no-arm-observation.json" \
      >"${run_dir}/no-arm-observation.log" 2>&1
  observation_status=$?
  set -e

  phase="cleanup"
  set +e
  stop_groups
  process_status=$?
  finish_audit
  audit_status=$?
  inventory_dataflash
  timeout 30 ros2 bag info "${run_dir}/rosbag" >"${run_dir}/rosbag-info.txt" 2>&1
  bag_info_status=$?
  python3 "${script_dir}/analyze_external_nav_health.py" "${run_dir}" \
    --output "${run_dir}/external-nav-health-diagnostic.json" \
    >"${run_dir}/external-nav-health-diagnostic.log" 2>&1
  diagnostic_status=$?
  set -e

  phase="validate_no_arm_smoke"
  python3 - "${run_dir}" "${observation_status}" "${process_status}" \
    "${audit_status}" "${bag_info_status}" "${diagnostic_status}" <<'PY'
import json
import pathlib
import sys

run = pathlib.Path(sys.argv[1])
observation_status, process_status, audit_status, bag_info_status, diagnostic_status = (
    int(value) for value in sys.argv[2:]
)
observation_path = run / "no-arm-observation.json"
diagnostic_path = run / "external-nav-health-diagnostic.json"
observation = json.loads(observation_path.read_text()) if observation_path.is_file() else {}
diagnostic = json.loads(diagnostic_path.read_text()) if diagnostic_path.is_file() else {}
cleanup = [
    json.loads(line)
    for line in (run / "cleanup-processes.jsonl").read_text().splitlines()
    if line.strip()
] if (run / "cleanup-processes.jsonl").is_file() else []
required_labels = {"sitl", "mavros", "gazebo", "rosbag", "p4_stack"}
dataflash = diagnostic.get("dataflash", {}).get("files", [])
ingress = diagnostic.get("comparison", {}).get("fcu_vision_ingress", {})
logs = "\n".join(
    (run / name).read_text(errors="replace")
    for name in ("p4-stack.log", "mavros.log", "gazebo.log", "sitl.log")
    if (run / name).is_file()
)
crash_markers = [
    marker for marker in (
        "Traceback (most recent call last)",
        "terminate called after throwing",
        "Segmentation fault",
        "core dumped",
        "process has died",
    ) if marker in logs
]
visp_continuous = bool(dataflash) and all(
    item.get("visp", {}).get("count", 0) > 1
    and item["visp"].get("receive_gaps_at_least_300ms") == 0
    and item["visp"].get("max_receive_gap_s") is not None
    and item["visp"]["max_receive_gap_s"] < 0.3
    for item in dataflash
)
ratio = ingress.get("dataflash_to_ros_output_ratio")
checks = {
    "observation_command_passed": observation_status == 0,
    "observation_criteria_passed": observation.get("passed") is True,
    "never_armed": observation.get("fcu_armed_count") == 0,
    "no_arm_or_takeoff_evidence": not observation.get("arm_or_takeoff_texts", ["missing"]),
    "mission_node_absent": "/xq_p4_mission" not in (run / "ros-nodes.txt").read_text(),
    "mission_result_absent": not (run / "mission-result.json").exists(),
    "cleanup_command_passed": process_status == 0,
    "cleanup_audit_passed": audit_status == 0,
    "cleanup_labels_complete": required_labels <= {item.get("label") for item in cleanup},
    "cleanup_exit_codes_accepted": bool(cleanup) and all(
        item.get("wait_status") in (0, 130, 143) for item in cleanup
    ),
    "cleanup_no_forced_kill": bool(cleanup) and all(
        not item.get("forced_kill") for item in cleanup
    ),
    "cleanup_no_residual": bool(cleanup) and all(not item.get("residual") for item in cleanup),
    "cleanup_no_crash_markers": not crash_markers,
    "rosbag_info_readable": bag_info_status == 0 and bool(
        (run / "rosbag-info.txt").read_text().strip()
    ),
    "diagnostic_readable": diagnostic_status == 0 and bool(diagnostic),
    "dataflash_visp_continuous": visp_continuous,
    "dataflash_receives_at_least_90pct_of_ros_output": isinstance(ratio, (int, float))
    and ratio >= 0.9,
    "no_critical_fcu_fault": diagnostic.get("dataflash", {}).get(
        "first_critical_message"
    ) is None,
}
report = {
    "schema_version": 1,
    "gate": "P4_CPU_NO_ARM_EXTERNAL_NAV_SMOKE",
    "status": "PASS" if all(checks.values()) else "FAIL",
    "checks": checks,
    "observation": observation,
    "fcu_vision_ingress": ingress,
    "dataflash": dataflash,
    "cleanup_processes": cleanup,
    "crash_markers": crash_markers,
}
(run / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
if report["status"] != "PASS":
    raise SystemExit("P4 no-arm smoke validation failed")
print(json.dumps({"status": report["status"], "checks": checks,
                  "fcu_vision_ingress": ingress}, indent=2))
PY
  echo "PASS: P4 no-arm ExternalNav smoke completed."
  echo "Results: ${run_dir}"
  exit 0
fi

phase="start_p4_mission"
start_group mission "${run_dir}/mission.log" \
  ros2 run xq_autonomy xq_p4_mission --ros-args \
  -p result_file:="${mission_result}" \
  -p mission_timeout_s:=240.0 \
  -p takeoff_altitude_m:=2.0 \
  -p square_side_m:=2.0 \
  -p hover_duration_s:=5.0

# The task has a pre-arm health phase, leaving time to prove that exactly the
# bound P4 node owns the command topic before any ARM request can be accepted.
sleep 1
timeout 15 ros2 node list --no-daemon >"${run_dir}/ros-nodes-with-mission.txt" 2>&1
[[ "$(grep -c '^/xq_p4_mission$' "${run_dir}/ros-nodes-with-mission.txt")" == 1 ]] || {
  echo "Expected exactly one xq_p4_mission node." >&2; exit 7;
}
graph_probe /uav1/mavros/setpoint_position/local "${run_dir}/p4-setpoint-graph.txt"
python3 - "${script_dir}" "${run_dir}" <<'PY'
import json, pathlib, sys
sys.path.insert(0, sys.argv[1])
from impact_runtime_audit import _endpoints
run = pathlib.Path(sys.argv[2])
report_path = run / "p4-runtime-audit.json"
report = json.loads(report_path.read_text())
publishers = _endpoints((run / "p4-setpoint-graph.txt").read_text(), "PUBLISHER")
report["setpoint_publishers"] = publishers
report["single_p4_mission"] = len(publishers) == 1 and publishers[0]["node"] == "xq_p4_mission"
report["passed"] = bool(report["passed"] and report["single_p4_mission"])
report_path.write_text(json.dumps(report, indent=2) + "\n")
if not report["passed"]:
    raise SystemExit("P4 mission ownership audit failed")
PY

mission_deadline=$((SECONDS + 260))
phase="wait_p4_mission"
while [[ ! -s "${mission_result}" ]] && ((SECONDS < mission_deadline)); do
  assert_core_alive
  sleep 1
done
[[ -s "${mission_result}" ]] || { echo "Mission result was not produced." >&2; exit 7; }
mission_status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${mission_result}")"
[[ "${mission_status}" == "PASS" ]] || { cat "${mission_result}" >&2; exit 8; }

evaluation_deadline=$((SECONDS + minimum_eval_duration_s + 30))
evaluation_status="IN_PROGRESS"
phase="wait_localization_evaluation"
while ((SECONDS < evaluation_deadline)); do
  assert_core_alive
  if [[ -s "${evaluation_result}" ]]; then
    evaluation_status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "${evaluation_result}")"
    [[ "${evaluation_status}" != "IN_PROGRESS" ]] && break
  fi
  sleep 1
done
[[ "${evaluation_status}" == "PASS" ]] || {
  [[ -s "${evaluation_result}" ]] && cat "${evaluation_result}" >&2
  echo "P4 in-flight FAST-LIO evaluation did not pass." >&2
  exit 9
}

timeout 10 ros2 topic echo --no-daemon --once /uav1/mavros/state \
  >"${run_dir}/final-state.txt" 2>&1
grep -q 'connected: true' "${run_dir}/final-state.txt" || {
  echo "MAVROS was not connected at P4 completion." >&2; exit 10;
}

# Ground truth must have exactly the project bridge publisher and evaluator
# subscriber.  The algorithm and flight-control nodes must not appear.
if grep -Eq 'Node name: /(xq_fast_lio|xq_p4_external_nav|xq_p4_mission)' \
  "${run_dir}/ground-truth-topic-graph.txt"; then
  echo "Ground truth leaked into an algorithm or flight-control node." >&2
  exit 11
fi

set +e
phase="cleanup"
stop_groups
process_status=$?
set -e
((process_status == 0)) || {
  echo "P4 process cleanup did not pass; see cleanup-processes.jsonl." >&2
  exit "${process_status}"
}
finish_audit
inventory_dataflash
timeout 30 ros2 bag info "${run_dir}/rosbag" >"${run_dir}/rosbag-info.txt"
phase="validate_artifacts"
python3 "${script_dir}/analyze_external_nav_health.py" "${run_dir}" \
  --output "${run_dir}/external-nav-health-diagnostic.json" \
  >"${run_dir}/external-nav-health-diagnostic.log"

python3 - "${run_dir}" <<'PY'
import json, pathlib, sys
run = pathlib.Path(sys.argv[1])
diagnostic = json.loads((run / "external-nav-health-diagnostic.json").read_text())
cleanup = [json.loads(line) for line in (run / "cleanup-processes.jsonl").read_text().splitlines()
           if line.strip()]
required_labels = {"sitl", "mavros", "gazebo", "rosbag", "p4_stack", "mission"}
counts = diagnostic["rosbag"]["topic_counts"]
required_topics = ("/localization/odom", "/uav1/mavros/odometry/out",
                   "/uav1/mavros/state", "/xq/p4/extnav/status",
                   "/xq/eval/p4/ground_truth")
dataflash = diagnostic["dataflash"]["files"]
logs = "\n".join((run / name).read_text(errors="replace") for name in
                 ("p4-stack.log", "mavros.log", "gazebo.log", "sitl.log", "mission.log"))
log_errors = [marker for marker in ("Traceback (most recent call last)",
    "terminate called after throwing", "Segmentation fault", "core dumped", "process has died")
    if marker in logs]
checks = {
    "cleanup_labels_complete": required_labels <= {item["label"] for item in cleanup},
    "cleanup_exit_codes_accepted": all(item["wait_status"] in (0, 130, 143) for item in cleanup),
    "cleanup_no_forced_kill": all(not item["forced_kill"] for item in cleanup),
    "cleanup_no_residual": all(not item["residual"] for item in cleanup),
    "cleanup_no_log_errors": not log_errors,
    "rosbag_info_readable": bool((run / "rosbag-info.txt").read_text().strip()),
    "rosbag_key_topics_have_data": all(int(counts.get(topic, 0)) > 0 for topic in required_topics),
    "dataflash_readable": bool(dataflash) and all(item["size_bytes"] > 0 for item in dataflash),
    "dataflash_has_vision_data": bool(dataflash) and all(item["visp"]["count"] > 0 for item in dataflash),
    "no_critical_fcu_fault": diagnostic["dataflash"]["first_critical_message"] is None,
}
report = {"schema_version": 1, "checks": checks, "passed": all(checks.values()),
          "cleanup_processes": cleanup, "cleanup_log_errors": log_errors,
          "rosbag_topic_counts": {topic: counts.get(topic, 0) for topic in required_topics},
          "dataflash": dataflash,
          "first_dataflash_fault": diagnostic["dataflash"]["first_critical_message"],
          "first_ros_fcu_fault": diagnostic["rosbag"]["status_text"]["first_fault"]}
(run / "artifact-validation.json").write_text(json.dumps(report, indent=2) + "\n")
if not report["passed"]:
    raise SystemExit("P4 artifact/cleanup validation failed")
PY

python3 - "${run_dir}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

run = Path(sys.argv[1])
mission = json.loads((run / "mission-result.json").read_text(encoding="utf-8"))
evaluation = json.loads((run / "localization-evaluation.json").read_text(encoding="utf-8"))
metadata = run / "rosbag" / "metadata.yaml"
isolation = (run / "isolation-audit.txt").read_text(encoding="utf-8")
runtime_audit = json.loads((run / "p4-runtime-audit.json").read_text(encoding="utf-8"))
artifacts = json.loads((run / "artifact-validation.json").read_text(encoding="utf-8"))
gazebo_log = (run / "gazebo.log").read_text(encoding="utf-8", errors="replace")
launcher_log = (run / "launcher.log").read_text(encoding="utf-8", errors="replace") if (run / "launcher.log").exists() else ""
summary = {
    "schema_version": 2,
    "gate": "P4_GPS_OFF_FAST_LIO_EXTERNAL_NAV_CLOSED_LOOP",
    "status": "PENDING",
    "mission_status": mission["status"],
    "mission_elapsed_s": mission["elapsed_s"],
    "mission_checks": mission["checks"],
    "task_result": mission.get("task_result"),
    "termination": mission.get("termination"),
    "health_gate": mission.get("health_gate"),
    "fcu_status_texts": mission.get("fcu_status_texts", []),
    "verified_parameters": mission["verified_parameters"],
    "external_nav": mission["external_nav"],
    "localization_status": evaluation["status"],
    "localization_metrics": evaluation["metrics"],
    "rosbag_present": metadata.is_file(),
    "dataflash_present": any((run / "sitl_runtime" / "logs").glob("*.BIN")),
    "dataflash_inventory": (run / "dataflash.inventory.txt").read_text(encoding="utf-8").splitlines()
    if (run / "dataflash.inventory.txt").is_file() else [],
    "ground_truth_isolated": True,
    "external_assets_unchanged": "PASS:" in isolation,
    "runtime_audit": runtime_audit,
    "artifact_validation": artifacts,
    "gazebo_clean_exit": not any(x in gazebo_log + launcher_log for x in ("Segmentation fault", "core dumped")),
    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
}
completion_passed = (summary["gazebo_clean_exit"] and runtime_audit["passed"] and
                     artifacts["passed"] and
                     mission.get("termination", {}).get("confirmed") is True)
summary["status"] = "PASS" if completion_passed else "FAIL"
(run / "summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
)
if not completion_passed:
    raise SystemExit("P4 completion evidence did not pass")
print(json.dumps(summary, ensure_ascii=False, indent=2))
PY

echo "PASS: P4 GPS-off FAST-LIO ExternalNav closed loop completed."
echo "Results: ${run_dir}"

"""IMPACT Gate P4: GPS-off ExternalNav square-flight state machine."""

from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, StatusText, SysStatus
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode, StreamRate
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class ConsecutiveHealthGate:
    """Latch health only after new, healthy samples; any bad sample resets it."""

    def __init__(self, required_samples: int) -> None:
        if required_samples < 1:
            raise ValueError("required_samples must be positive")
        self.required_samples = required_samples
        self.consecutive_samples = 0
        self.last_sample_id: int | None = None
        self.last_failure_reasons: list[str] = []

    def observe(
        self, healthy: bool, reasons: list[str], sample_id: int | None = None
    ) -> bool:
        if not healthy:
            self.consecutive_samples = 0
            self.last_failure_reasons = list(reasons)
        elif sample_id is None or sample_id != self.last_sample_id:
            self.consecutive_samples += 1
        self.last_sample_id = sample_id
        return self.consecutive_samples >= self.required_samples


_FCU_FAULT_TOKENS = (
    "visodom: not healthy",
    "visodom: roll/pitch diff",
    "ekf variance",
    "ekf failsafe",
    "potential thrust loss",
)

PREARM_CHECK = 1 << 28
VISION_POSITION = 1 << 7


def _sensor_enabled_and_healthy(status: SysStatus, mask: int) -> bool:
    return bool(status.sensors_enabled & mask) and bool(status.sensors_health & mask)


def _fcu_fault_reason(text: str) -> str | None:
    lowered = text.strip().lower()
    for token in _FCU_FAULT_TOKENS:
        if token in lowered:
            return token
    return None


class P4MissionNode(Node):
    """Verify EKF source, then fly takeoff-hover-square-return-land."""

    # ArduPilot's MAV_CMD_NAV_TAKEOFF handler owns the Guided TakeOff
    # sub-mode until the climb is complete.  Sending SET_POSITION_TARGET while
    # it is climbing switches Guided back to position control and cancels the
    # takeoff altitude target.  Position setpoints therefore start only after
    # ASCEND has confirmed a stable arrival at the requested altitude.
    POSITION_CONTROL_PHASES = frozenset(("HOVER", "TRACK_SQUARE"))
    TERMINATION_PHASES = frozenset(("LAND", "DESCEND", "FAILSAFE_WAIT"))

    REQUIRED_PARAMS = {
        "AHRS_EKF_TYPE": 3,
        "EK3_ENABLE": 1,
        "EK3_SRC1_POSXY": 6,
        "EK3_SRC1_VELXY": 6,
        "EK3_SRC1_POSZ": 6,
        "EK3_SRC1_VELZ": 6,
        "EK3_SRC1_YAW": 6,
        "VISO_TYPE": 2,
        "GPS_TYPE": 0,
        "GPS_TYPE2": 0,
        "SIM_GPS_DISABLE": 1,
        "SIM_GPS2_DISABLE": 1,
    }

    def __init__(self) -> None:
        super().__init__("xq_p4_mission")
        self.declare_parameter("mavros_prefix", "/uav1/mavros")
        self.declare_parameter("takeoff_altitude_m", 2.0)
        self.declare_parameter("square_side_m", 2.0)
        self.declare_parameter("hover_duration_s", 5.0)
        self.declare_parameter("arrival_tolerance_m", 0.45)
        self.declare_parameter("mission_timeout_s", 240.0)
        self.declare_parameter("command_timeout_s", 8.0)
        self.declare_parameter("result_file", "")
        self.declare_parameter("extnav_status_topic", "/xq/p4/extnav/status")
        self.declare_parameter("raw_odom_topic", "/localization/odom")
        self.declare_parameter("health_gate_consecutive_samples", 8)
        self.declare_parameter("health_status_max_age_s", 0.7)
        self.declare_parameter("health_odom_max_age_s", 0.7)
        self.declare_parameter("health_fault_window_s", 2.0)
        self.declare_parameter("failsafe_termination_timeout_s", 90.0)
        self.declare_parameter("prearm_timeout_s", 60.0)
        self.declare_parameter("prearm_check_interval_s", 5.0)
        self.declare_parameter("sys_status_max_age_s", 2.5)
        self.declare_parameter("fcu_state_max_age_s", 2.5)
        self.declare_parameter("arm_confirmation_timeout_s", 8.0)

        prefix = str(self.get_parameter("mavros_prefix").value).rstrip("/")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(State, f"{prefix}/state", self._state_cb, qos)
        self.create_subscription(SysStatus, f"{prefix}/sys_status", self._sys_status_cb, qos)
        self.create_subscription(Odometry, f"{prefix}/local_position/odom", self._odom_cb, qos)
        self.create_subscription(
            String,
            str(self.get_parameter("extnav_status_topic").value),
            self._extnav_cb,
            reliable,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("raw_odom_topic").value),
            self._raw_odom_cb,
            reliable,
        )
        self.create_subscription(
            StatusText,
            f"{prefix}/statustext/recv",
            self._status_text_cb,
            # MAVROS publishes FCU status text with best-effort QoS in this
            # setup; reliable would silently create an incompatible endpoint.
            qos,
        )
        self.setpoint_publisher = self.create_publisher(
            PoseStamped, f"{prefix}/setpoint_position/local", 20
        )
        self.origin_publisher = self.create_publisher(
            GeoPointStamped, f"{prefix}/global_position/set_gp_origin", 10
        )
        self.arm_client = self.create_client(CommandBool, f"{prefix}/cmd/arming")
        self.mode_client = self.create_client(SetMode, f"{prefix}/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, f"{prefix}/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, f"{prefix}/cmd/land")
        self.stream_client = self.create_client(StreamRate, f"{prefix}/set_stream_rate")
        self.prearm_client = self.create_client(CommandLong, f"{prefix}/cmd/command")
        # MAVROS2 exposes pulled FCU parameters through its standard ROS 2
        # parameter service instead of the deprecated MAVROS ParamGet service.
        self.param_client = self.create_client(
            GetParameters, f"{prefix}/param/get_parameters"
        )

        self.fcu_state = State()
        self.have_state = False
        self.armed_seen = False
        self.fcu_state_last_wall = 0.0
        self.fcu_sys_status = SysStatus()
        self.have_sys_status = False
        self.sys_status_last_wall = 0.0
        self.have_odom = False
        self.have_raw_odom = False
        self.current_xyz = (0.0, 0.0, 0.0)
        self.current_orientation = None
        self.current_odom_stamp = 0.0
        self.current_odom_frame_id = ""
        self.current_odom_child_frame_id = ""
        self.current_odom_last_wall = 0.0
        self.current_odom_stamp_violations = 0
        self.current_odom_max_gap_s = 0.0
        self.raw_xyz = (0.0, 0.0, 0.0)
        self.raw_orientation = None
        self.raw_odom_stamp = 0.0
        self.raw_odom_frame_id = ""
        self.raw_odom_child_frame_id = ""
        self.raw_odom_last_wall = 0.0
        self.raw_odom_stamp_violations = 0
        self.raw_odom_max_gap_s = 0.0
        self.origin_xyz = (0.0, 0.0, 0.0)
        self.target: tuple[float, float, float] | None = None
        self.extnav_status: dict[str, object] = {}
        self.extnav_last_wall = 0.0
        self.fcu_status_texts: list[dict[str, object]] = []
        self.last_fcu_fault: dict[str, object] | None = None
        self.last_fcu_fault_key = ""
        self.health_gate = ConsecutiveHealthGate(
            int(self.get_parameter("health_gate_consecutive_samples").value)
        )
        self.health_stable = False
        self.health_samples: list[dict[str, object]] = []
        self.last_health_reason_key = ""
        self.task_failure_reason: str | None = None
        self.termination_reason: str | None = None
        self.verified_params: dict[str, int] = {}
        self.param_names = list(self.REQUIRED_PARAMS)
        self.param_index = 0
        self.pending_param = None
        self.last_param_request = 0.0
        self.pending_command = None
        self.phase = "WAIT_FCU"
        self.phase_started = time.monotonic()
        self.started = self.phase_started
        self.termination_started: float | None = None
        self.last_request = 0.0
        self.last_prearm_request = 0.0
        self.arm_request_count = 0
        self.arm_request_wall: float | None = None
        self.last_origin_publish = 0.0
        self.arrival_started: float | None = None
        self.waypoints: list[tuple[float, float, float]] = []
        self.waypoint_index = 0
        self.completed_waypoints: list[dict[str, float]] = []
        self.events: list[dict[str, object]] = []
        self.finalized = False
        self.create_timer(0.1, self._tick)
        self._event("START", "P4 GPS-off ExternalNav mission created")

    def _state_cb(self, message: State) -> None:
        self.fcu_state = message
        self.have_state = True
        self.armed_seen = self.armed_seen or bool(message.armed)
        self.fcu_state_last_wall = time.monotonic()

    def _sys_status_cb(self, message: SysStatus) -> None:
        self.fcu_sys_status = message
        self.have_sys_status = True
        self.sys_status_last_wall = time.monotonic()

    def _odom_cb(self, message: Odometry) -> None:
        p = message.pose.pose.position
        xyz = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(value) for value in xyz):
            return
        stamp = self._message_stamp(message)
        if self.have_odom and stamp <= self.current_odom_stamp:
            self.current_odom_stamp_violations += 1
        elif self.have_odom:
            self.current_odom_max_gap_s = max(
                self.current_odom_max_gap_s, stamp - self.current_odom_stamp
            )
        self.current_xyz = xyz
        self.current_orientation = message.pose.pose.orientation
        self.current_odom_stamp = stamp
        self.current_odom_frame_id = str(message.header.frame_id)
        self.current_odom_child_frame_id = str(message.child_frame_id)
        self.current_odom_last_wall = time.monotonic()
        if not self.have_odom:
            self.origin_xyz = xyz
            self.have_odom = True
            self._event("FCU_ODOM_LOCK", f"origin={xyz}")

    @staticmethod
    def _message_stamp(message: Odometry) -> float:
        return float(message.header.stamp.sec) + 1e-9 * float(message.header.stamp.nanosec)

    def _raw_odom_cb(self, message: Odometry) -> None:
        p = message.pose.pose.position
        xyz = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(value) for value in xyz):
            return
        stamp = self._message_stamp(message)
        if self.have_raw_odom and stamp <= self.raw_odom_stamp:
            self.raw_odom_stamp_violations += 1
        elif self.have_raw_odom:
            self.raw_odom_max_gap_s = max(self.raw_odom_max_gap_s, stamp - self.raw_odom_stamp)
        self.raw_xyz = xyz
        self.raw_orientation = message.pose.pose.orientation
        self.raw_odom_stamp = stamp
        self.raw_odom_frame_id = str(message.header.frame_id)
        self.raw_odom_child_frame_id = str(message.child_frame_id)
        self.raw_odom_last_wall = time.monotonic()
        self.have_raw_odom = True

    def _extnav_cb(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(status, dict):
            self.extnav_status = status
            self.extnav_last_wall = time.monotonic()

    def _status_text_cb(self, message: StatusText) -> None:
        text = str(message.text)
        record = {
            "elapsed_s": round(time.monotonic() - self.started, 3),
            "severity": int(message.severity),
            "text": text,
        }
        self.fcu_status_texts.append(record)
        if len(self.fcu_status_texts) > 200:
            self.fcu_status_texts = self.fcu_status_texts[-200:]
        reason = _fcu_fault_reason(text)
        if reason is None:
            return
        self.last_fcu_fault = {**record, "reason": reason}
        key = f"{reason}:{text}"
        if key != self.last_fcu_fault_key:
            self.last_fcu_fault_key = key
            self._event("FCU_HEALTH_FAULT", f"{reason}: {text}")

    def _event(self, kind: str, detail: str) -> None:
        record = {
            "elapsed_s": round(time.monotonic() - self.started, 3),
            "kind": kind,
            "phase": self.phase,
            "detail": detail,
        }
        self.events.append(record)
        self.get_logger().info(f"P4 {kind}: {detail}")

    def _transition(self, phase: str, detail: str) -> None:
        now = time.monotonic()
        if phase in self.TERMINATION_PHASES and getattr(self, "termination_started", None) is None:
            self.termination_started = now
        self.phase = phase
        self.phase_started = now
        self.pending_command = None
        self.last_request = 0.0
        self.arrival_started = None
        self._event("TRANSITION", f"{phase}: {detail}")

    def _publish_origin(self) -> None:
        now = time.monotonic()
        if now - self.last_origin_publish < 1.0:
            return
        message = GeoPointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.position.latitude = -35.363262
        message.position.longitude = 149.165237
        message.position.altitude = 584.0
        self.origin_publisher.publish(message)
        self.last_origin_publish = now

    def _publish_setpoint(self) -> None:
        if self.target is None:
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = self.target
        if self.current_orientation is not None:
            message.pose.orientation = self.current_orientation
        else:
            message.pose.orientation.w = 1.0
        self.setpoint_publisher.publish(message)

    def _send_command(self, kind: str) -> None:
        now = time.monotonic()
        if self.pending_command is not None or now - self.last_request < 2.0:
            return
        if kind == "stream":
            if not self.stream_client.service_is_ready():
                return
            request = StreamRate.Request()
            request.stream_id = StreamRate.Request.STREAM_ALL
            request.message_rate = 20
            request.on_off = True
            future = self.stream_client.call_async(request)
        elif kind == "mode":
            if not self.mode_client.service_is_ready():
                return
            request = SetMode.Request()
            request.custom_mode = "GUIDED"
            future = self.mode_client.call_async(request)
        elif kind == "arm":
            if self.arm_request_count:
                return
            if not self.arm_client.service_is_ready():
                return
            request = CommandBool.Request()
            request.value = True
            future = self.arm_client.call_async(request)
            self.arm_request_count += 1
            self.arm_request_wall = now
        elif kind == "prearm":
            interval = float(self.get_parameter("prearm_check_interval_s").value)
            if now - self.last_prearm_request < interval or not self.prearm_client.service_is_ready():
                return
            future = self.prearm_client.call_async(CommandLong.Request(command=401))
            self.last_prearm_request = now
        elif kind == "takeoff":
            if not self.takeoff_client.service_is_ready():
                return
            request = CommandTOL.Request()
            request.altitude = float(self.get_parameter("takeoff_altitude_m").value)
            future = self.takeoff_client.call_async(request)
        elif kind == "land":
            if not self.land_client.service_is_ready():
                return
            request = CommandTOL.Request()
            future = self.land_client.call_async(request)
        else:
            raise ValueError(kind)
        self.pending_command = (kind, future)
        self.last_request = time.monotonic()
        self._event("REQUEST", kind)

    def _poll_command(self) -> None:
        if self.pending_command is None:
            return
        kind, future = self.pending_command
        if not future.done():
            timeout_s = float(self.get_parameter("command_timeout_s").value)
            if time.monotonic() - self.last_request > timeout_s:
                self.pending_command = None
                retry = kind != "arm"
                self._event("RESPONSE", f"{kind} timeout after {timeout_s:.1f}s; retry={retry}")
                if kind == "arm":
                    self._failure_to_land("ARM response timeout; automatic retry disabled")
            return
        self.pending_command = None
        try:
            response = future.result()
            accepted = True if kind == "stream" else bool(
                response.mode_sent if kind == "mode" else response.success
            )
        except Exception as exc:
            self._event("RESPONSE", f"{kind} exception={exc!r}")
            if kind == "arm":
                self._failure_to_land("ARM response exception; automatic retry disabled")
            return
        self._event("RESPONSE", f"{kind} accepted={accepted}")
        if not accepted:
            if kind == "arm":
                self._failure_to_land("ARM rejected; automatic retry disabled")
            return
        if kind == "stream" and self.phase == "SET_STREAM":
            self._transition("VERIFY_NAV", "MAVLink streams requested")
        elif kind == "takeoff" and self.phase == "TAKEOFF":
            self._transition("ASCEND", "takeoff accepted")
        elif kind == "land" and self.phase == "LAND":
            self.target = None
            self._transition("DESCEND", "landing accepted")

    def _poll_param(self) -> None:
        if self.pending_param is not None:
            name, future = self.pending_param
            if not future.done():
                return
            self.pending_param = None
            try:
                response = future.result()
            except Exception as exc:
                self._finish("FAIL", f"parameter {name} query failed: {exc!r}")
                return
            if len(response.values) != 1:
                self._event("PARAM_WAIT", f"{name} not pulled from FCU yet")
                return
            parameter = response.values[0]
            if parameter.type == ParameterType.PARAMETER_INTEGER:
                value = int(parameter.integer_value)
            elif parameter.type == ParameterType.PARAMETER_DOUBLE:
                value = int(round(parameter.double_value))
            elif parameter.type == ParameterType.PARAMETER_NOT_SET:
                self._event("PARAM_WAIT", f"{name} not pulled from FCU yet")
                return
            else:
                self._finish(
                    "FAIL", f"parameter {name} has unexpected ROS type {parameter.type}"
                )
                return
            expected = self.REQUIRED_PARAMS[name]
            if value != expected:
                self._finish("FAIL", f"parameter {name}={value}, expected {expected}")
                return
            self.verified_params[name] = value
            self.param_index += 1
            self._event("PARAM", f"{name}={value}")
        now = time.monotonic()
        if (
            self.param_index < len(self.param_names)
            and self.param_client.service_is_ready()
            and now - self.last_param_request >= 1.0
        ):
            name = self.param_names[self.param_index]
            request = GetParameters.Request()
            request.names = [name]
            self.pending_param = (name, self.param_client.call_async(request))
            self.last_param_request = now

    def _health_snapshot(self, require_prearm: bool = False) -> dict[str, object]:
        now = time.monotonic()
        reasons: list[str] = []
        status_age = now - self.extnav_last_wall if self.extnav_last_wall else math.inf
        if status_age > float(self.get_parameter("health_status_max_age_s").value):
            reasons.append("extnav_status_stale")
        if self.extnav_status.get("healthy") is not True:
            status_reasons = self.extnav_status.get("health_reasons", [])
            if isinstance(status_reasons, list) and status_reasons:
                reasons.extend(f"extnav:{item}" for item in status_reasons)
            else:
                reasons.append("extnav_unhealthy")
        if int(self.extnav_status.get("mavros_subscribers", 0)) <= 0:
            reasons.append("mavros_odom_subscriber_missing")
        if not self.have_raw_odom:
            reasons.append("raw_odom_missing")
        elif now - self.raw_odom_last_wall > float(self.get_parameter("health_odom_max_age_s").value):
            reasons.append("raw_odom_stale")
        if self.raw_odom_stamp_violations:
            reasons.append("raw_odom_stamp_nonmonotonic")
        if not self.have_odom:
            reasons.append("fcu_odom_missing")
        elif now - self.current_odom_last_wall > float(self.get_parameter("health_odom_max_age_s").value):
            reasons.append("fcu_odom_stale")
        if self.current_odom_stamp_violations:
            reasons.append("fcu_odom_stamp_nonmonotonic")
        if not self.have_state or not self.fcu_state.connected:
            reasons.append("fcu_disconnected")
        sys_status_age = now - self.sys_status_last_wall if self.have_sys_status else math.inf
        prearm_healthy = self.have_sys_status and _sensor_enabled_and_healthy(
            self.fcu_sys_status, PREARM_CHECK
        )
        vision_healthy = self.have_sys_status and _sensor_enabled_and_healthy(
            self.fcu_sys_status, VISION_POSITION
        )
        if require_prearm:
            if sys_status_age > float(self.get_parameter("sys_status_max_age_s").value):
                reasons.append("fcu_sys_status_stale")
            if not prearm_healthy:
                reasons.append("fcu_prearm_unhealthy")
            if not vision_healthy:
                reasons.append("fcu_vision_unhealthy")
        if self.last_fcu_fault is not None:
            fault_age = now - self.started - float(self.last_fcu_fault["elapsed_s"])
            if fault_age <= float(self.get_parameter("health_fault_window_s").value):
                reasons.append(f"fcu:{self.last_fcu_fault['reason']}")
        return {
            "healthy": not reasons,
            "reasons": reasons,
            "extnav_status_age_s": status_age if math.isfinite(status_age) else None,
            "extnav_status_sequence": self.extnav_status.get("status_sequence"),
            "extnav_status": dict(self.extnav_status),
            "raw_odom": {
                "received": self.have_raw_odom,
                "age_s": now - self.raw_odom_last_wall if self.raw_odom_last_wall else None,
                "stamp_s": self.raw_odom_stamp,
                "frame_id": self.raw_odom_frame_id,
                "child_frame_id": self.raw_odom_child_frame_id,
                "xyz_m": list(self.raw_xyz),
                "stamp_nonmonotonic": self.raw_odom_stamp_violations,
                "max_gap_s": self.raw_odom_max_gap_s,
            },
            "fcu_odom": {
                "received": self.have_odom,
                "age_s": now - self.current_odom_last_wall if self.current_odom_last_wall else None,
                "stamp_s": self.current_odom_stamp,
                "frame_id": self.current_odom_frame_id,
                "child_frame_id": self.current_odom_child_frame_id,
                "xyz_m": list(self.current_xyz),
                "stamp_nonmonotonic": self.current_odom_stamp_violations,
                "max_gap_s": self.current_odom_max_gap_s,
            },
            "fcu_state": {
                "connected": bool(self.fcu_state.connected),
                "armed": bool(self.fcu_state.armed),
                "guided": bool(self.fcu_state.guided),
                "mode": str(self.fcu_state.mode),
                "system_status": int(self.fcu_state.system_status),
            },
            "fcu_sys_status": {
                "received": self.have_sys_status,
                "age_s": sys_status_age if math.isfinite(sys_status_age) else None,
                "sensors_present": int(self.fcu_sys_status.sensors_present),
                "sensors_enabled": int(self.fcu_sys_status.sensors_enabled),
                "sensors_health": int(self.fcu_sys_status.sensors_health),
                "prearm_healthy": bool(prearm_healthy),
                "vision_healthy": bool(vision_healthy),
            },
            "last_fcu_fault": self.last_fcu_fault,
        }

    def _observe_health(
        self, require_params: bool, require_prearm: bool = False
    ) -> tuple[bool, dict[str, object]]:
        snapshot = self._health_snapshot(require_prearm=require_prearm)
        reasons = list(snapshot["reasons"])
        if require_params and len(self.verified_params) != len(self.REQUIRED_PARAMS):
            reasons.append("fcu_parameters_incomplete")
        healthy = not reasons
        sample_id = snapshot.get("extnav_status_sequence")
        sample_id = int(sample_id) if isinstance(sample_id, int) else None
        stable = self.health_gate.observe(healthy, reasons, sample_id)
        snapshot["healthy_for_gate"] = healthy
        snapshot["gate_stable"] = stable
        snapshot["gate_consecutive_samples"] = self.health_gate.consecutive_samples
        snapshot["gate_required_samples"] = self.health_gate.required_samples
        self.health_stable = stable
        self.health_samples.append(
            {
                "elapsed_s": round(time.monotonic() - self.started, 3),
                "phase": self.phase,
                "sample": snapshot,
            }
        )
        if len(self.health_samples) > 4000:
            self.health_samples = self.health_samples[-4000:]
        reason_key = ",".join(reasons)
        if not healthy and reason_key != self.last_health_reason_key:
            self.last_health_reason_key = reason_key
            self._event("HEALTH_UNHEALTHY", reason_key)
        elif healthy and self.last_health_reason_key:
            self.last_health_reason_key = ""
            self._event("HEALTH_SAMPLE_RECOVERED", "current sample is healthy; gate is rebuilding")
        return stable, snapshot

    def _nav_ready(self) -> bool:
        stable, _ = self._observe_health(require_params=True)
        if stable:
            self._event(
                "HEALTH_GATE_PASS",
                f"{self.health_gate.consecutive_samples} consecutive ExternalNav/EKF samples",
            )
        return stable

    def _flight_health_loss(self, snapshot: dict[str, object]) -> None:
        if self.finalized or self.phase in ("WAIT_FCU", "VERIFY_NAV", "LAND", "DESCEND", "FAILSAFE_WAIT"):
            return
        reasons = ",".join(str(item) for item in snapshot.get("reasons", [])) or "unknown"
        self._failure_to_land(f"flight health lost: {reasons}")

    def _failure_to_land(self, reason: str) -> None:
        """Route an in-flight failure through landing and termination confirmation."""
        if self.finalized or self.phase in ("LAND", "DESCEND", "FAILSAFE_WAIT"):
            return
        self.task_failure_reason = reason
        if not self._fcu_state_is_fresh():
            self._transition("FAILSAFE_WAIT", reason)
            self._event("TASK_FAILURE", reason)
            self._event("TERMINATION_UNCONFIRMED", "FCU state is unavailable or stale; no safe state assumption")
        elif self.fcu_state.armed and str(self.fcu_state.mode).upper() == "LAND":
            self._transition("FAILSAFE_WAIT", reason)
            self._event("TASK_FAILURE", reason)
            self._event("FAILSAFE_PRESERVED", "FCU already in LAND; no mode or takeoff request")
        elif self.fcu_state.armed:
            self._transition("LAND", reason)
            self._event("TASK_FAILURE", reason)
        else:
            self._finish("FAIL", reason)

    def _fcu_state_is_fresh(self) -> bool:
        if not self.have_state or not self.fcu_state_last_wall:
            return False
        try:
            max_age = float(self.get_parameter("fcu_state_max_age_s").value)
        except (AttributeError, TypeError, ValueError):
            max_age = 2.5
        return time.monotonic() - self.fcu_state_last_wall <= max_age

    def _termination_evidence(self) -> dict[str, object]:
        """Describe termination without treating drifting altitude as ground truth."""
        state_fresh = self._fcu_state_is_fresh()
        disarmed = state_fresh and not bool(self.fcu_state.armed)
        mode_is_land = str(self.fcu_state.mode).upper() == "LAND"
        flight_started = bool(getattr(self, "armed_seen", False))
        confirmed = bool(disarmed and (not flight_started or mode_is_land))
        odom_fresh = bool(
            self.have_odom
            and time.monotonic() - self.current_odom_last_wall
            <= float(self.get_parameter("health_odom_max_age_s").value)
        )
        altitude_error = abs(self.current_xyz[2] - self.origin_xyz[2]) if self.have_odom else None
        return {
            "confirmed": confirmed,
            "state_fresh": state_fresh,
            "disarmed": bool(disarmed),
            "mode_is_land": mode_is_land,
            "armed_seen": flight_started,
            "evidence": (
                "fresh_fcu_disarm_in_land" if confirmed and flight_started
                else "fresh_fcu_remained_disarmed" if confirmed
                else None
            ),
            "odom_fresh": odom_fresh,
            "altitude_error_from_origin_m": altitude_error,
            "altitude_near_origin": bool(odom_fresh and altitude_error is not None and altitude_error <= 0.35),
        }

    def _distance_to_target(self) -> float:
        if self.target is None:
            return math.inf
        return math.sqrt(sum((self.current_xyz[i] - self.target[i]) ** 2 for i in range(3)))

    def _arrived_for(self, dwell_s: float) -> bool:
        tolerance = float(self.get_parameter("arrival_tolerance_m").value)
        if self._distance_to_target() > tolerance:
            self.arrival_started = None
            return False
        if self.arrival_started is None:
            self.arrival_started = time.monotonic()
        return time.monotonic() - self.arrival_started >= dwell_s

    def _finish(self, status: str, reason: str) -> None:
        if self.finalized:
            return
        termination = self._termination_evidence()
        state_fresh = bool(termination["state_fresh"])
        termination_confirmed = bool(termination["confirmed"])
        if status == "PASS" and self.task_failure_reason:
            status, reason = "FAIL", self.task_failure_reason
        elif status == "PASS" and not termination_confirmed:
            status, reason = "FAIL", "landing/disarm not confirmed by fresh FCU state"
        self.finalized = True
        self.phase = "DONE" if status == "PASS" else "FAILED"
        self._event(status, reason)
        if termination_confirmed:
            default_reason = (
                "FCU disarmed in LAND after flight"
                if termination["armed_seen"] else "FCU remained disarmed"
            )
            self.termination_reason = self.termination_reason or default_reason
        else:
            self.termination_reason = self.termination_reason or "FCU remains armed or state unavailable"
        result = {
            "schema_version": 2,
            "gate": "P4_GPS_OFF_EXTERNAL_NAV_CLOSED_LOOP",
            "status": status,
            "reason": reason,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "elapsed_s": round(time.monotonic() - self.started, 3),
            "command_timeout_s": float(self.get_parameter("command_timeout_s").value),
            "arm_request_count": self.arm_request_count,
            "verified_parameters": self.verified_params,
            "external_nav": self.extnav_status,
            "fcu_sys_status": self._health_snapshot(require_prearm=False)["fcu_sys_status"],
            "health_gate": {
                "required_consecutive_samples": self.health_gate.required_samples,
                "last_consecutive_samples": self.health_gate.consecutive_samples,
                "stable_before_failure": self.health_stable,
                "last_failure_reasons": self.health_gate.last_failure_reasons,
                "samples_recorded": len(self.health_samples),
            },
            "fcu_status_texts": self.fcu_status_texts,
            "completed_waypoints": self.completed_waypoints,
            "checks": {
                "gps_disabled": self.verified_params.get("GPS_TYPE") == 0
                and self.verified_params.get("SIM_GPS_DISABLE") == 1,
                "ekf_sources_external_nav": all(
                    self.verified_params.get(name) == 6
                    for name in ("EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_POSZ", "EK3_SRC1_VELZ", "EK3_SRC1_YAW")
                ),
                "takeoff_and_hover": any(event["phase"] == "HOVER" for event in self.events),
                "square_and_return": len(self.completed_waypoints) == 4,
                "landed_and_disarmed": termination_confirmed,
                "prearm_gate_passed": any(event["kind"] == "PREARM_GATE_PASS" for event in self.events),
                "single_arm_request": self.arm_request_count == 1,
            },
            "task_result": {
                "success": status == "PASS",
                "failure_reason": None if status == "PASS" else (self.task_failure_reason or reason),
            },
            "termination": {
                **termination,
                "confirmed": termination_confirmed,
                "reason": self.termination_reason,
                "armed": bool(self.fcu_state.armed),
                "mode": self.fcu_state.mode,
                "state_age_s": (time.monotonic() - self.fcu_state_last_wall
                                 if self.fcu_state_last_wall else None),
            },
            "health_samples": self.health_samples,
            "final": {
                "connected": bool(self.fcu_state.connected),
                "armed": bool(self.fcu_state.armed),
                "mode": self.fcu_state.mode,
                "xyz_m": list(self.current_xyz),
            },
            "events": self.events,
        }
        path = Path(str(self.get_parameter("result_file").value))
        if str(path) in ("", "."):
            self.get_logger().error("result_file parameter is empty")
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)

    def _tick(self) -> None:
        if self.finalized:
            return
        self._publish_origin()
        if self.phase in self.POSITION_CONTROL_PHASES:
            self._publish_setpoint()
        self._poll_command()
        now = time.monotonic()
        if self.phase in self.TERMINATION_PHASES:
            termination_started = getattr(self, "termination_started", None)
            if termination_started is None:
                termination_started = self.phase_started
            if now - termination_started > float(self.get_parameter("failsafe_termination_timeout_s").value):
                confirmed = bool(self._termination_evidence()["confirmed"])
                self.termination_reason = ("FCU disarmed at termination deadline" if confirmed
                                           else "FCU termination not confirmed before timeout")
                self._event("TERMINATION_CONFIRMED" if confirmed else "TERMINATION_UNCONFIRMED",
                            self.termination_reason)
                self._finish("FAIL", self.task_failure_reason or "termination deadline exceeded")
                return
        elif now - self.started > float(self.get_parameter("mission_timeout_s").value):
            self._failure_to_land(f"mission timeout in {self.phase}")
            return
        if self.have_state and not self.fcu_state.connected and self.phase != "WAIT_FCU":
            if self.phase == "FAILSAFE_WAIT":
                pass
            else:
                self._flight_health_loss(self._health_snapshot())
                if self.phase == "FAILSAFE_WAIT":
                    return
                self._finish("FAIL", f"FCU disconnected in {self.phase}")
                return

        if self.phase in ("SET_GUIDED", "ARM", "TAKEOFF", "ASCEND", "HOVER", "TRACK_SQUARE"):
            _, snapshot = self._observe_health(require_params=False, require_prearm=True)
            if not bool(snapshot["healthy_for_gate"]):
                self._flight_health_loss(snapshot)
                if self.phase == "FAILSAFE_WAIT":
                    return

        if self.phase == "FAILSAFE_WAIT":
            if self._fcu_state_is_fresh() and not self.fcu_state.armed:
                self.termination_reason = "FCU disarmed after health loss"
                self._event("TERMINATION_CONFIRMED", self.termination_reason)
                self._finish("FAIL", f"{self.task_failure_reason or 'flight health lost'}; termination confirmed")
            elif (self._fcu_state_is_fresh() and self.fcu_state.connected
                  and self.fcu_state.armed and str(self.fcu_state.mode).upper() != "LAND"):
                self._transition("LAND", "fresh FCU state restored; complete failure termination")
            return

        if self.phase == "WAIT_FCU":
            if self.have_state and self.fcu_state.connected:
                self._transition("SET_STREAM", "FCU heartbeat connected")
        elif self.phase == "SET_STREAM":
            self._send_command("stream")
        elif self.phase == "VERIFY_NAV":
            self._poll_param()
            if self.finalized:
                return
            if self._nav_ready():
                self._transition("WAIT_PREARM", "ExternalNav and FCU parameters verified")
            elif now - self.phase_started > 90.0:
                self._finish("FAIL", "ExternalNav did not become healthy or parameters were not verified")
        elif self.phase == "WAIT_PREARM":
            stable, _ = self._observe_health(require_params=True, require_prearm=True)
            if stable:
                self.target = self.current_xyz
                self._event("PREARM_GATE_PASS", "FCU PREARM and VISION health confirmed")
                self._transition("SET_GUIDED", "FCU is armable with healthy ExternalNav")
            elif now - self.phase_started > float(self.get_parameter("prearm_timeout_s").value):
                self._finish("FAIL", "FCU prearm health did not become ready before timeout")
            else:
                self._send_command("prearm")
        elif self.phase == "SET_GUIDED":
            if self.fcu_state.mode == "GUIDED":
                self._transition("ARM", "GUIDED confirmed")
            else:
                self._send_command("mode")
        elif self.phase == "ARM":
            if self.fcu_state.armed:
                self.origin_xyz = self.current_xyz
                altitude = float(self.get_parameter("takeoff_altitude_m").value)
                self.target = (self.origin_xyz[0], self.origin_xyz[1], self.origin_xyz[2] + altitude)
                self._transition("TAKEOFF", "armed with ExternalNav")
            elif self.arm_request_wall is not None and now - self.arm_request_wall > float(
                self.get_parameter("arm_confirmation_timeout_s").value
            ):
                self._finish("FAIL", "ARM was not confirmed; automatic retry disabled")
            else:
                self._send_command("arm")
        elif self.phase == "TAKEOFF":
            self._send_command("takeoff")
        elif self.phase == "ASCEND":
            if self._arrived_for(2.0):
                self._transition("HOVER", "takeoff altitude reached")
            elif now - self.phase_started > 45.0:
                self._failure_to_land("takeoff altitude not reached")
        elif self.phase == "HOVER":
            if now - self.phase_started >= float(self.get_parameter("hover_duration_s").value):
                side = float(self.get_parameter("square_side_m").value)
                x, y, z = self.target
                self.waypoints = [(x + side, y, z), (x + side, y + side, z), (x, y + side, z), (x, y, z)]
                self.waypoint_index = 0
                self.target = self.waypoints[0]
                self._transition("TRACK_SQUARE", "hover complete; first rectangle corner commanded")
        elif self.phase == "TRACK_SQUARE":
            if self._arrived_for(1.5):
                self.completed_waypoints.append(
                    {"index": self.waypoint_index + 1, "x": self.current_xyz[0], "y": self.current_xyz[1], "z": self.current_xyz[2]}
                )
                self.waypoint_index += 1
                if self.waypoint_index >= len(self.waypoints):
                    self._transition("LAND", "rectangle and return completed")
                else:
                    self.target = self.waypoints[self.waypoint_index]
                    self.phase_started = now
                    self.arrival_started = None
                    self._event("WAYPOINT", f"commanded corner {self.waypoint_index + 1}")
            elif now - self.phase_started > 45.0:
                self._failure_to_land(f"timeout reaching rectangle corner {self.waypoint_index + 1}")
        elif self.phase == "LAND":
            self._send_command("land")
        elif self.phase == "DESCEND":
            if self._termination_evidence()["confirmed"]:
                status = "FAIL" if self.task_failure_reason else "PASS"
                reason = self.task_failure_reason or "GPS-off LIO ExternalNav takeoff-hover-rectangle-return-land completed"
                self._finish(status, reason)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P4MissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Suppress only Humble's invalid-context shutdown race.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Final EGO-spline certification and short-step recovery; no truth subscriptions."""
from __future__ import annotations
import hashlib
import json
import math
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import (DirectionalIntegrity, InformationMap, LocalizationGeometry, PlannerGoal,
                                  PlannerCandidate, TrajectoryAuthorization)
from .alert_limit import sample_bspline, compute_alert_limit
from .minimum_excitation import (generate_discrete_candidates, build_information_profile,
                                 CandidateForecast, evaluate_candidate)
from .p10_active_perception_node import _cloud_xyz
from .sitl_integrity import certify_final, RecoveryCycle
from .localization_health import geometry_health

# GPU SITL can take several seconds to brake to the low-speed arrival gate
# after an accepted lateral/vertical recovery spline.  Keep the request alive
# while it settles so the required step-done -> observation sequence is not
# replaced by a premature timeout.
RECOVERY_SETTLE_TIMEOUT_S = 8.0
RECOVERY_INFORMATION_VISIBILITY_RADIUS_M = 2.8
RECOVERY_SPEED_MPS = 0.3
RECOVERY_LATENCY_P99_S = 0.15
RECOVERY_BODY_RADIUS_M = 0.35
RECOVERY_BASE_RESERVE_M = 0.10
RECOVERY_TRACKING_RESERVE_M = 0.10
RECOVERY_MARGIN_RESERVE_M = 0.10
# Require a measurable covariance reduction and independent geometric
# information gain on the same pre-recovery direction.
RECOVERY_MIN_INFORMATION_GAIN_M2 = 1.0e-6
RECOVERY_TOTAL_BUDGET_S = 45.0
LOCALIZATION_RESTORATION_DWELL_S = 1.0
LOCALIZATION_ANCHOR_WEAK_FRACTION = 0.05
# An accepted spline is still an execution failure when the estimator has not
# made measurable progress toward the requested mission goal for a sustained
# interval.  This is deliberately longer than one planner cycle and is only
# evaluated while a mission trajectory is actively authorized.
EXECUTION_PROGRESS_TIMEOUT_S = 20.0
# FAST-LIO's bounded estimator lag on the GPU SITL path can make a healthy
# forward segment appear as only a few centimetres of goal-distance gain over
# one watchdog window.  Keep the 20 s fail-closed deadline, but use a smaller
# measured gain so the guard does not land a vehicle that is still advancing;
# a genuinely stationary execution still trips the same deadline.
EXECUTION_MIN_PROGRESS_M = 0.10
EXECUTION_WEAK_AXIS_ALIGNMENT = 0.75
# Declare mission completion while the vehicle is already inside the measured
# goal gate and braking. Waiting for near-zero speed lets the vehicle coast
# past the gate because the FCU position controller has non-zero stopping lag.
EXECUTION_GOAL_SPEED_MPS = 0.35
# The mission-stage topic is a liveness heartbeat.  A half-second watchdog
# races normal GPU SITL scheduling during a replan and can revoke a valid
# trajectory before the recovery state machine observes its new data.
STAGE_HEARTBEAT_TIMEOUT_S = 2.0


def minimum_recovery_information_radius_m() -> float:
    """Lower bound before directional PL for a safe informative surface."""
    latency_reserve = (
        RECOVERY_SPEED_MPS * RECOVERY_LATENCY_P99_S
        + 0.5 * RECOVERY_LATENCY_P99_S * RECOVERY_LATENCY_P99_S
    )
    return (
        RECOVERY_BODY_RADIUS_M
        + RECOVERY_BASE_RESERVE_M
        + RECOVERY_TRACKING_RESERVE_M
        + latency_reserve
        + RECOVERY_MARGIN_RESERVE_M
    )


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def ros_stamp(value):
    from builtin_interfaces.msg import Time
    ns = max(0, int(value * 1e9))
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def xyz(point):
    return np.array((point.x, point.y, point.z), float)


def main_node(cls, args=None):
    rclpy.init(args=args)
    node = cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class SITLSupervisor(Node):
    def __init__(self):
        super().__init__("impact_supervisor")
        for key, value in {"session_id": "", "strategy": "recovery", "calibration_file": "",
                           "goal": [12., 0., 2.], "speed_limit": 0.65,
                           "goal_tolerance_m": 0.45,
                           "mission_speed_scale": 1.0,
                           "recovery_release_speed_scale": 0.5,
                           "execution_progress_timeout_s": EXECUTION_PROGRESS_TIMEOUT_S,
                           "minimum_translation_weak_fraction": 0.02,
                           "localization_recovery_timeout_s": 8.0,
                           "localization_sensor_range_m": 40.0,
                           "recovery_observation_window_s": 3.0,
                           "event_file": "", "margin_reserve": 0.10,
                           "recovery_information_visibility_radius_m":
                               RECOVERY_INFORMATION_VISIBILITY_RADIUS_M}.items():
            self.declare_parameter(key, value)
        self.declare_parameter("recovery_release_waypoint", [math.nan] * 3)
        self.session = self.get_parameter("session_id").value
        if not self.session:
            raise ValueError("session_id is required")
        payload = Path(self.get_parameter("calibration_file").value).read_bytes()
        calibration = json.loads(payload)
        if not calibration.get("train_only") or calibration.get("test_data_used", True):
            raise ValueError("calibration must be frozen and train-only")
        self.k = max(float(v["k95"]) for v in calibration["directional"].values())
        if not math.isfinite(self.k) or self.k <= 0:
            raise ValueError("invalid calibration")
        self.calibration_sha = hashlib.sha256(payload).hexdigest()
        self.strategy = self.get_parameter("strategy").value
        self.goal = np.array(self.get_parameter("goal").value, float)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        if not math.isfinite(self.goal_tolerance) or self.goal_tolerance <= 0.:
            raise ValueError("goal_tolerance_m must be finite and positive")
        self.limit = float(self.get_parameter("speed_limit").value)
        self.mission_speed_scale = float(self.get_parameter("mission_speed_scale").value)
        if not 0.1 <= self.mission_speed_scale <= 1.0:
            raise ValueError("mission_speed_scale must be between 0.1 and 1.0")
        self.recovery_release_speed_scale = float(
            self.get_parameter("recovery_release_speed_scale").value
        )
        if not 0.1 <= self.recovery_release_speed_scale <= 1.0:
            raise ValueError("recovery_release_speed_scale must be between 0.1 and 1.0")
        self.execution_progress_timeout = float(
            self.get_parameter("execution_progress_timeout_s").value
        )
        if not math.isfinite(self.execution_progress_timeout) or self.execution_progress_timeout <= 0.:
            raise ValueError("execution_progress_timeout_s must be finite and positive")
        self.minimum_translation_weak_fraction = float(
            self.get_parameter("minimum_translation_weak_fraction").value)
        self.localization_recovery_timeout = float(
            self.get_parameter("localization_recovery_timeout_s").value)
        if not math.isfinite(self.localization_recovery_timeout) or self.localization_recovery_timeout <= 0:
            raise ValueError("localization recovery timeout must be finite and positive")
        geometry_health(np.eye(3), 30,
                        minimum_weak_fraction=self.minimum_translation_weak_fraction)
        self.recovery_information_visibility_radius = float(
            self.get_parameter("recovery_information_visibility_radius_m").value
        )
        if (
            not math.isfinite(self.recovery_information_visibility_radius)
            or self.recovery_information_visibility_radius
            <= minimum_recovery_information_radius_m()
        ):
            raise ValueError(
                "recovery information visibility radius cannot admit a surface "
                "outside the fixed safety and margin reserves"
            )
        self.cycle = RecoveryCycle()
        self.odom = self.cloud = self.integrity = self.information = self.geometry = None
        self.localization_loss_since = None
        self.localization_restored_since = None
        self.last_healthy_position = None
        self.localization_sensor_range = float(self.get_parameter("localization_sensor_range_m").value)
        release_waypoint = np.asarray(
            self.get_parameter("recovery_release_waypoint").value, dtype=float
        )
        unset_release_waypoint = (
            release_waypoint.size == 3 and np.isnan(release_waypoint).all()
        )
        if (release_waypoint.size not in (3,) or
                not (unset_release_waypoint or np.isfinite(release_waypoint).all())):
            raise ValueError("recovery release waypoint must be empty or a finite 3-vector")
        self.recovery_release_waypoint = None if unset_release_waypoint else release_waypoint
        self.recovery_release_active = False
        self.recovery_release_complete = False
        self.recovery_observation_window = float(self.get_parameter("recovery_observation_window_s").value)
        if not math.isfinite(self.recovery_observation_window) or self.recovery_observation_window < 0.5:
            raise ValueError("recovery observation window must be at least 0.5 s")
        if not math.isfinite(self.localization_sensor_range) or self.localization_sensor_range <= 0.5:
            raise ValueError("localization sensor range must exceed 0.5 m")
        self.localization_health = {}
        self.received = {}
        self.active = self.pending = None
        self.enabled = False
        self.completed = False
        self.reset = False
        self.last_sim = -math.inf
        self.last_request_wall = -math.inf
        self.last_plan_sim = -math.inf
        self.last_recovery_sim = -math.inf
        self.target = self.goal.copy()
        self.recovery_origin = None
        self.recovery_before = None
        self.recovery_before_information = None
        self.recovery_step_before_information = None
        self.recovery_after_information = None
        self.recovery_observation_request_id = None
        self.recovery_step_request_id = None
        self.recovery_observed = False
        self.recovery_required = False
        self.recovery_started_sim = None
        self.recovery_spent_sim = 0.0
        self.recovery_information_improved = None
        self.observing_until = None
        self.ready_wall = 0.
        self.candidate_highwater = 0
        self.last_metrics = {}
        self.execution_fault = None
        self.progress_anchor_sim = None
        self.progress_anchor_distance = None
        self.progress_anchor_position = None
        self.stage = "WAIT"
        self.stage_wall = 0.
        self.goal_pub = self.create_publisher(PlannerGoal, "/impact/planner_goal", 10)
        self.auth_pub = self.create_publisher(TrajectoryAuthorization, "/impact/authorization", 20)
        self.spline_pub = self.create_publisher(Bspline, "/impact/certified_bspline", 20)
        self.status_pub = self.create_publisher(String, "/impact/status", 10)
        for kind, topic, callback in [
            (Odometry, "/localization/odom", lambda m: self.input("odom", m)),
            (PointCloud2, "/cloud_registered", lambda m: self.input("cloud", m)),
            (DirectionalIntegrity, "/integrity/directional", lambda m: self.input("integrity", m)),
            (LocalizationGeometry, "/localization/geometry", lambda m: self.input("geometry", m)),
            (InformationMap, "/integrity/information_map", lambda m: self.input("information", m))]:
            self.create_subscription(kind, topic, callback, qos_profile_sensor_data)
        self.create_subscription(PlannerCandidate, "/impact/planner_candidate", self.candidate, 20)
        self.create_subscription(String, "/impact/mission_stage", self.mission, 10)
        self.events = None
        event_path = self.get_parameter("event_file").value
        if event_path:
            self.events = open(event_path, "a", encoding="utf-8", buffering=1)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.tick, clock=self.wall_clock)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def input(self, name, message):
        previous = getattr(self, name)
        if previous and stamp_s(message.header.stamp) < stamp_s(previous.header.stamp):
            return
        setattr(self, name, message)
        self.received[name] = time.monotonic()

    def mission(self, message):
        try:
            data = json.loads(message.data)
            if data["session_id"] != self.session:
                return
            self.stage, self.stage_wall = data["phase"], time.monotonic()
            self.enabled = self.stage == "ACTIVE" and not self.reset
        except (ValueError, KeyError):
            return

    def event(self, kind, **fields):
        data = dict(event=kind, sim_time=self.now_s(), wall_time=time.monotonic(),
                    session_id=self.session, request_id=self.cycle.request, **fields)
        if self.events:
            self.events.write(json.dumps(data, allow_nan=False) + "\n")
        return data

    def fresh(self):
        now = self.now_s()
        for name in ("odom", "cloud", "integrity", "geometry"):
            value = getattr(self, name)
            if value is None or value.header.frame_id != "xq_lio_map":
                return False
            if not 0 <= now - stamp_s(value.header.stamp) <= 0.5:
                return False
            if time.monotonic() - self.received[name] > 0.5:
                return False
        return True

    def request(self, target, intent="mission", scale=1.0):
        if self.execution_fault:
            return
        if intent != "mission" and self.recovery_required:
            self._capture_recovery_step_baseline()
        self.target = np.asarray(target, float)
        req = PlannerGoal()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = "xq_lio_map"
        req.session_id = self.session
        req.request_id = self.cycle.issue(intent)
        req.goal = Point(x=float(target[0]), y=float(target[1]), z=float(target[2]))
        req.speed_scale = float(scale)
        self.goal_pub.publish(req)
        self.pending = None
        self.last_request_wall = time.monotonic()
        self.last_plan_sim = self.now_s()
        self.event("PLAN_REQUEST", intent=intent, goal=self.target.tolist(), speed_scale=scale)

    def _capture_recovery_step_baseline(self):
        """Capture a fresh fixed-direction baseline immediately before an action."""
        direction = (self.recovery_before_information.get("direction")
                     if self.recovery_before_information else None)
        self.recovery_step_before_information = self._fixed_information_snapshot(direction)

    def authorization(self, candidate, accepted, reason):
        msg = TrajectoryAuthorization()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "xq_lio_map"
        msg.session_id = self.session
        msg.request_id = candidate.request_id
        msg.trajectory_id = candidate.trajectory.traj_id
        msg.valid_until = ros_stamp(self.now_s() + (0.3 if accepted else 0.))
        msg.authorized, msg.reason = accepted, reason
        self.auth_pub.publish(msg)

    def revoke(self, reason):
        if self.active:
            self.authorization(self.active, False, reason)
            self.event("REVOKE", reason=reason, trajectory_id=self.active.trajectory.traj_id)
        self.active = None

    def candidate(self, message):
        if (not self.enabled or self.reset or self.execution_fault
                or message.session_id != self.session):
            return
        if self.completed or self.cycle.phase not in ("PLANNING", "EXECUTING"):
            return
        if message.request_id != self.cycle.request or message.header.frame_id != "xq_lio_map":
            return
        if message.trajectory.traj_id <= self.candidate_highwater:
            return
        if not 0 <= self.now_s() - stamp_s(message.header.stamp) <= 0.5:
            return
        self.candidate_highwater = message.trajectory.traj_id
        self.pending = message

    def certify(self, candidate, tracking=False):
        started = time.perf_counter()
        b = candidate.trajectory
        points = np.array([xyz(p) for p in b.pos_pts])
        elapsed = self.now_s() - stamp_s(b.start_time)
        if elapsed < 0:
            raise ValueError("future trajectory")
        duration = b.knots[len(points)] - b.knots[b.order]
        terminal_hold = (self.cycle.intent != "mission" and self.cycle.phase == "EXECUTING"
                         and duration <= elapsed <= duration + RECOVERY_SETTLE_TIMEOUT_S)
        obstacles = _cloud_xyz(self.cloud)
        # Conservative voxel representatives: reserve an extra voxel diagonal below.
        if len(obstacles):
            _, indices = np.unique(np.floor(obstacles / 0.1).astype(np.int64), axis=0, return_index=True)
            obstacles = obstacles[indices]
        error = 0.
        if tracking:
            parameter = b.knots[b.order] + elapsed
            if terminal_hold:
                first = sample_bspline(points, np.array(b.knots), b.order, 0.05,
                                       b.knots[len(points)] - 1e-6)[-1]
            else:
                first = sample_bspline(points, np.array(b.knots), b.order, 0.05, parameter)[0]
            error = float(np.linalg.norm(first - xyz(self.odom.pose.pose.position)))
        age = max(self.now_s() - stamp_s(m.header.stamp) for m in (self.odom, self.cloud, self.integrity))
        result = certify_final(points, np.array(b.knots), b.order, obstacles,
            np.array(self.integrity.integrity_covariance).reshape(3, 3), strategy=self.strategy,
            k_alpha=self.k, elapsed=elapsed, input_age=age, tracking_error=error,
            speed_limit=self.limit, reserve=float(self.get_parameter("margin_reserve").value),
            body_radius=0.35 + math.sqrt(3) * 0.1, terminal_hold=terminal_hold)
        health = self._measured_geometry_health()
        if not health["healthy"] and self.cycle.intent == "mission":
            result = replace(result, accepted=False, reason="LOCALIZATION_UNOBSERVABLE")
        elif self.localization_loss_since is not None and self.cycle.intent == "mission":
            result = replace(result, accepted=False, reason="LOCALIZATION_RESTORATION_PENDING")
        self.last_metrics = dict(AL=result.alert, PL=result.protection, margin=result.margin,
                                 speed_bound=result.speed, tracking_error=error,
                                 critical_direction=list(result.direction),
                                 source_stamp=stamp_s(self.integrity.header.stamp))
        self.last_metrics["localization_geometry"] = health
        self.last_metrics["terminal_hold"] = terminal_hold
        self.event("CERTIFICATION_TIMING", elapsed_wall_s=time.perf_counter()-started,
                   input_age_sim_s=age, remaining_recheck=tracking,
                   trajectory_id=b.traj_id)
        return result

    def recovery_intents(self, rejected):
        """Rank finite P10 intents with map information; authorize none here."""
        if self.information is None:
            self.event("RECOVERY_UNAVAILABLE", reason="information_missing")
            return []
        if not self.information.valid:
            self.event("RECOVERY_UNAVAILABLE", reason="information_invalid")
            return []
        if self.information.ground_truth_used:
            self.event("RECOVERY_UNAVAILABLE", reason="ground_truth_information")
            return []
        information_age = self.now_s() - stamp_s(self.information.header.stamp)
        if not 0 <= information_age <= 1.0:
            self.event("RECOVERY_UNAVAILABLE", reason="information_stale",
                       information_age_s=information_age)
            return []
        position = xyz(self.odom.pose.pose.position)
        direction = self.goal - position
        direction[2] = 0
        norm = np.linalg.norm(direction)
        if norm < 0.1:
            return []
        # A one metre probe can stop inside the same longitudinally weak
        # interval. The bounded 2.5 m probe reaches the first forward support
        # while remaining a finite, certification-checked recovery action.
        baseline = position + np.linspace(0, min(norm, 2.5), 9)[:, None] * direction / norm
        candidates = generate_discrete_candidates(baseline, baseline_duration=3., vertical_offset=0.50,
            previous_high_quality_pose=self.last_healthy_position)
        info = self.information
        output = []
        for candidate in candidates:
            if candidate.name == "baseline":
                continue
            try:
                # Execute the informative portion of the bounded candidate,
                # rather than stopping at its midpoint. The midpoint removes
                # most of the forward probe and can leave the vehicle inside
                # the same weak interval across recovery retries.
                middle = candidate.positions[max(1, int(round(0.75 * (len(candidate.positions) - 1))))]
                if candidate.name == "backtrack": middle = candidate.positions[1]
                if candidate.name == "short_hover": middle = position
                if candidate.name in ("up_offset", "down_offset", "left_lateral", "right_lateral"):
                    middle = np.asarray(middle, dtype=float)
                if not 0.65 <= middle[2] <= 2.9:
                    continue
                step_positions = position + np.linspace(0., 1., 9)[:, None] * (middle - position)
                candidate = replace(candidate, positions=step_positions)
                alert = compute_alert_limit(step_positions, _cloud_xyz(self.cloud),
                    speed_mps=RECOVERY_SPEED_MPS,
                    latency_p99_s=RECOVERY_LATENCY_P99_S,
                    maximum_acceleration_mps2=1., body_radius_m=RECOVERY_BODY_RADIUS_M,
                    base_reserve_m=RECOVERY_BASE_RESERVE_M,
                    tracking_reserve_m=RECOVERY_TRACKING_RESERVE_M)
                profile = build_information_profile(candidate.positions,
                    np.array([xyz(p) for p in info.positions]), np.array([xyz(p) for p in info.normals]),
                    np.array(info.static_confidence), np.array(info.geometry_quality), np.array(info.last_seen_s),
                    now=self.now_s(),
                    visibility_radius=self.recovery_information_visibility_radius,
                    age_time_constant=10., information_scale=2500.)
                forecast = evaluate_candidate(CandidateForecast(candidate, alert.alert_limits,
                    alert.obstacle_directions, profile), np.array(self.integrity.integrity_covariance).reshape(3,3),
                    k_alpha=self.k, margin_reserve=RECOVERY_MARGIN_RESERVE_M,
                    baseline_duration=3.,
                    lambda_energy=0.25, lambda_distance=1., minimum_prediction_variance=1e-5)
                # Future improvement only ranks actions whose rough geometry is feasible.
                if not forecast.feasible:
                    continue
                weak_prediction = None
                if self.recovery_before_information:
                    weak = np.array(self.recovery_before_information["direction"])
                    # Forecast the executed short step, including known distant
                    # anchors at the real sensor range. Prediction never authorizes.
                    continuation = middle + direction / norm * min(norm, 0.5)
                    step_profile = build_information_profile(np.array([position, middle, continuation]),
                        np.array([xyz(p) for p in info.positions]),
                        np.array([xyz(p) for p in info.normals]),
                        np.array(info.static_confidence), np.array(info.geometry_quality),
                        np.array(info.last_seen_s), now=self.now_s(),
                        visibility_radius=self.localization_sensor_range,
                        age_time_constant=10., information_scale=2500., range_edge_taper_m=0.5,
                        sensor_vertical_fov_rad=(-0.12217304764, 0.90757121104),
                        sensor_height_offset_m=0.12)
                    observed = np.einsum("i,nij,j->n", weak, step_profile, weak)
                    weak_prediction = dict(direction=weak.tolist(), before=float(observed[0]),
                                           after=float(observed[1]),
                                           continuation=float(observed[2]))
                    variance_before = float(weak @ np.asarray(
                        self.integrity.integrity_covariance, dtype=float).reshape(3, 3) @ weak)
                    variance_after = 1.0 / (1.0 / max(variance_before, 1.0e-12) + float(observed[1]))
                    weak_prediction["variance_before_m2"] = variance_before
                    weak_prediction["variance_after_m2"] = variance_after
                    if variance_before - variance_after < RECOVERY_MIN_INFORMATION_GAIN_M2:
                        continue
                # Recovery splines use the same physical speed envelope as the
                # mission arm. A raw scale of 1.0 would restore the nominal
                # 0.65 m/s planner speed and be rejected by the 0.30 m/s
                # recoverable certification limit before any observation.
                scale = self.mission_speed_scale
                if candidate.name == "slow_trajectory":
                    scale *= 0.5
                output.append((forecast.cost, candidate.name, middle, scale, forecast.minimum_margin,
                               weak_prediction))
            except (ValueError, np.linalg.LinAlgError):
                continue
        # Prefer support that survives a short forward continuation. Recovering
        # at an entry anchor alone can otherwise create a backtrack/retry loop.
        # These forecasts rank candidates; they never replace live certification.
        output.sort(key=lambda item: (
            0 if item[1] == "up_offset" else 1,
            -min(item[5]["after"], item[5]["continuation"])
            if item[5] else 0.0,
            item[0], item[1]))
        self.event(
            "RECOVERY_FORECAST",
            information_visibility_radius_m=self.recovery_information_visibility_radius,
            candidates=[dict(name=x[1], cost=x[0], predicted_margin=x[4],
                             fixed_direction_information=x[5], target=x[2].tolist()) for x in output],
        )
        return [(x[1], x[2], x[3]) for x in output]

    def _reset_progress_watch(self):
        self.progress_anchor_sim = None
        self.progress_anchor_distance = None
        self.progress_anchor_position = None

    def _weak_axis_alignment(self, direction):
        if self.integrity is None:
            return None
        try:
            weak = np.asarray(self.integrity.weak_direction_map, dtype=float).reshape(3)
            norm = float(np.linalg.norm(weak))
            direction = np.asarray(direction, dtype=float).reshape(3)
            direction_norm = float(np.linalg.norm(direction))
            if (norm <= 0.0 or direction_norm <= 0.0
                    or not np.isfinite(np.r_[weak, direction]).all()):
                return None
            return abs(float(np.dot(weak / norm, direction / direction_norm)))
        except (TypeError, ValueError):
            return None

    def _fixed_information_snapshot(self, direction=None):
        """Return covariance information on one fixed direction across recovery."""
        if self.integrity is None or self.geometry is None:
            return None
        try:
            covariance = np.asarray(self.integrity.integrity_covariance, dtype=float).reshape(3, 3)
            information = np.asarray(self.integrity.information_matrix, dtype=float).reshape(3, 3)
            raw_information = np.asarray(self.geometry.information_matrix, dtype=float).reshape(3, 3)
            if direction is None:
                direction = np.asarray(self.integrity.weak_direction_map, dtype=float).reshape(3)
            direction = np.asarray(direction, dtype=float).reshape(3)
            norm = float(np.linalg.norm(direction))
            if norm <= 0.0 or not np.isfinite(np.r_[covariance.ravel(), information.ravel(),
                                                  raw_information.ravel(), direction]).all():
                return None
            direction = direction / norm
            variance = float(direction @ covariance @ direction)
            observed_information = float(direction @ information @ direction)
            observed_raw_information = float(direction @ raw_information @ direction)
            if (variance < 0.0 or observed_information < 0.0 or observed_raw_information < 0.0
                    or not math.isfinite(variance) or not math.isfinite(observed_information)):
                return None
            return dict(direction=direction.tolist(), variance_m2=variance,
                        geometric_information=observed_information,
                        raw_geometric_information=observed_raw_information,
                        raw_stamp_s=stamp_s(self.geometry.header.stamp),
                        protection_m=self.k * math.sqrt(variance),
                        stamp_s=stamp_s(self.integrity.header.stamp))
        except (TypeError, ValueError, np.linalg.LinAlgError):
            return None

    def _retain_recovered_height(self, position):
        if self.recovery_release_waypoint is not None or position[2] < self.goal[2] + 0.2:
            return
        direction = self.goal - position
        direction[2] = 0.0
        distance = float(np.linalg.norm(direction))
        if distance <= self.goal_tolerance:
            return
        # A successful elevated observation must not immediately be undone by
        # descending to the nominal goal. This target is only a plan request;
        # the full optimized spline still needs the same live certification.
        self.recovery_release_waypoint = position + direction / distance * min(distance, 2.0)
        self.recovery_release_complete = False
        self.event("RECOVERY_HEIGHT_CONTINUATION_REQUESTED",
                   goal=self.recovery_release_waypoint.tolist())

    def _check_execution_progress(self, now, position):
        """Fail closed when an authorized mission execution stops advancing."""
        if self.execution_fault or not self.active or self.cycle.intent != "mission":
            self._reset_progress_watch()
            return
        if self.cycle.phase != "EXECUTING":
            self._reset_progress_watch()
            return
        direction = self.goal - position
        distance = float(np.linalg.norm(direction))
        if distance <= self.goal_tolerance:
            self._reset_progress_watch()
            return
        if self.progress_anchor_sim is None:
            self.progress_anchor_sim = now
            self.progress_anchor_distance = distance
            self.progress_anchor_position = position.copy()
            return
        improvement = float(self.progress_anchor_distance - distance)
        if improvement >= EXECUTION_MIN_PROGRESS_M:
            self.progress_anchor_sim = now
            self.progress_anchor_distance = distance
            self.progress_anchor_position = position.copy()
            return
        elapsed = now - self.progress_anchor_sim
        if elapsed < getattr(self, "execution_progress_timeout", EXECUTION_PROGRESS_TIMEOUT_S):
            return
        alignment = self._weak_axis_alignment(direction)
        # The stop is based on measured lack of progress.  Alignment is
        # retained as evidence when the weak direction explains the failure;
        # it is not a prerequisite, so a planner/actuator stall still fails
        # closed if the directional message is temporarily unavailable.
        self.execution_fault = dict(
            reason="EXECUTION_STALLED",
            elapsed_s=float(elapsed),
            distance_to_goal_m=distance,
            progress_m=improvement,
            position=position.tolist(),
            weak_axis_alignment=alignment,
            weak_axis_aligned=bool(alignment is not None
                                   and alignment >= EXECUTION_WEAK_AXIS_ALIGNMENT),
        )
        self.revoke("EXECUTION_STALLED")
        self.cycle.phase = "FAIL_CLOSED"
        self.event("EXECUTION_FAIL_CLOSED", **self.execution_fault)

    def _require_information_recovery(self):
        if self.strategy != "recovery" or self.recovery_required:
            return
        self.recovery_required = True
        if self.recovery_started_sim is None:
            self.recovery_started_sim = self.now_s()
        self.recovery_before = dict(self.last_metrics)
        self.recovery_before_information = self._fixed_information_snapshot(
            self.localization_health.get("weak_direction")
            if self.localization_health.get("healthy") is False else None)
        self.recovery_step_before_information = None
        self.recovery_after_information = None
        self.recovery_observation_request_id = None
        self.recovery_step_request_id = None
        self.recovery_information_improved = None
        self.recovery_observed = False

    def _measured_geometry_health(self):
        if self.geometry is None:
            return dict(healthy=False, reasons=["geometry_missing"])
        return geometry_health(self.geometry.information_matrix, self.geometry.effective_points,
                               minimum_weak_fraction=self.minimum_translation_weak_fraction)

    def _check_localization_health(self, now):
        recovery_elapsed = self.recovery_spent_sim + (
            now - self.recovery_started_sim if self.recovery_started_sim is not None else 0.0)
        if self.recovery_required and recovery_elapsed >= RECOVERY_TOTAL_BUDGET_S:
            self.execution_fault = dict(reason="RECOVERY_BUDGET_EXCEEDED",
                                        elapsed_s=float(recovery_elapsed))
            self.pending = None
            self.cycle.phase = "FAIL_CLOSED"
            self.revoke("RECOVERY_BUDGET_EXCEEDED")
            self.event("EXECUTION_FAIL_CLOSED", **self.execution_fault)
            return
        self.localization_health = self._measured_geometry_health()
        if self.localization_health["healthy"]:
            if self.localization_loss_since is None:
                if self.localization_health["weak_fraction"] >= LOCALIZATION_ANCHOR_WEAK_FRACTION:
                    self.last_healthy_position = xyz(self.odom.pose.pose.position).copy()
                return
            if self.localization_restored_since is None:
                self.localization_restored_since = stamp_s(self.geometry.header.stamp)
            if stamp_s(self.geometry.header.stamp) - self.localization_restored_since >= LOCALIZATION_RESTORATION_DWELL_S:
                self.event("LOCALIZATION_GEOMETRY_RESTORED", geometry=self.localization_health,
                           sustained_observation_s=LOCALIZATION_RESTORATION_DWELL_S)
                self.localization_loss_since = None
                self.localization_restored_since = None
                return
        else:
            self.localization_restored_since = None
            if self.localization_loss_since is None:
                self.localization_loss_since = now
                self.event("LOCALIZATION_GEOMETRY_DEGRADED", geometry=self.localization_health)
            if self.active and self.cycle.intent == "mission":
                self._require_information_recovery()
                self.revoke("LOCALIZATION_UNOBSERVABLE")
        elapsed = now - self.localization_loss_since
        if self.strategy != "recovery" or elapsed >= self.localization_recovery_timeout:
            self.execution_fault = dict(reason="LOCALIZATION_UNOBSERVABLE", elapsed_s=elapsed,
                                        geometry=self.localization_health)
            self.pending = None
            self.revoke("LOCALIZATION_UNOBSERVABLE")
            self.cycle.phase = "FAIL_CLOSED"
            self.event("EXECUTION_FAIL_CLOSED", **self.execution_fault)

    def tick(self):
        now = self.now_s()
        if now < self.last_sim - 1e-6:
            self.revoke("CLOCK_RESET")
            self.reset, self.enabled = True, False
            self.pending = None
            self.odom = self.cloud = self.integrity = self.information = self.geometry = None
            self.localization_loss_since = None
            self.localization_restored_since = None
            self.last_healthy_position = None
            self.event("CLOCK_RESET_RESTART_REQUIRED")
        self.last_sim = now
        if time.monotonic() - self.stage_wall > STAGE_HEARTBEAT_TIMEOUT_S:
            self.enabled = False
        if not self.enabled or not self.fresh():
            self.revoke("DISABLED_OR_STALE")
            self.status()
            return
        if self.execution_fault:
            self.revoke(self.execution_fault["reason"])
            self.status()
            return
        self._check_localization_health(now)
        if self.execution_fault:
            self.status()
            return
        if self.active:
            try:
                result = self.certify(self.active, tracking=True)
                if not result.accepted:
                    if self.cycle.intent == "mission":
                        self._require_information_recovery()
                    self.revoke(result.reason)
                else:
                    self.authorization(self.active, True, "REVALIDATED")
            except (ValueError, np.linalg.LinAlgError):
                if self.cycle.intent == "mission":
                    # A numerically invalid recheck is still a loss of
                    # certification.  Route it through the same information
                    # recovery state machine as an explicit margin rejection;
                    # otherwise the next mission request can be re-authorized
                    # without a recovery step or new observation.
                    self._require_information_recovery()
                self.revoke("EXPIRED_OR_INVALID")
        if self.pending:
            candidate, self.pending = self.pending, None
            try:
                result = self.certify(candidate, tracking=True)
                accepted, reason = result.accepted, result.reason
            except (ValueError, np.linalg.LinAlgError) as error:
                accepted, reason = False, str(error)
            recovery_comparison = None
            if accepted and self.cycle.intent == "mission" and self.recovery_required:
                before, after = self.recovery_before, self.last_metrics
                information_before = (self.recovery_step_before_information
                                      or self.recovery_before_information)
                if before:
                    da, dp = after["AL"]-before["AL"], after["PL"]-before["PL"]
                    recovery_comparison = dict(
                        before=before, after=dict(after), delta_AL=da, delta_PL=dp,
                        delta_margin=da-dp,
                        information_before=information_before,
                        information_initial=self.recovery_before_information,
                        information_after=self.recovery_after_information,
                        observation_request_id=self.recovery_observation_request_id,
                        information_improved=self.recovery_information_improved is True,
                        comparison="Fixed pre-recovery direction, covariance and measured information",
                    )
                if (self.recovery_information_improved is not True
                        or not self.recovery_observed
                        or self.recovery_step_request_id is None
                        or self.recovery_step_request_id != self.recovery_observation_request_id):
                    accepted, reason = False, "RECOVERY_INFORMATION_NOT_IMPROVED"
                elif not recovery_comparison:
                    accepted, reason = False, "RECOVERY_EVIDENCE_MISSING"
            self.event("CERTIFY", accepted=accepted, reason=reason,
                       trajectory_id=candidate.trajectory.traj_id, **self.last_metrics)
            if not accepted and self.cycle.intent == "mission":
                self._require_information_recovery()
            self.authorization(candidate, accepted, reason)
            if accepted:
                self.active = candidate
                self.spline_pub.publish(candidate.trajectory)
                self.cycle.result(candidate.request_id, True)
                if recovery_comparison:
                    self.event("RECOVERY_CONFIRMED", **recovery_comparison)
                    self.recovery_release_active = bool(
                        self.recovery_release_waypoint is not None
                        and not self.recovery_release_complete
                        and np.linalg.norm(self.target - self.recovery_release_waypoint) < 1.0e-6
                    )
                    self.cycle.remaining.clear()
                    self.recovery_required = False
                    if self.recovery_started_sim is not None:
                        self.recovery_spent_sim += max(0.0, now - self.recovery_started_sim)
                    self.recovery_started_sim = None
                    self.recovery_observed = False
                    self.recovery_information_improved = None
            elif not self.active:
                if recovery_comparison:
                    self.event("RECOVERY_NOT_BENEFICIAL", reason=reason, **recovery_comparison)
                self.cycle.result(candidate.request_id, False)
                if self.strategy == "recovery" and self.cycle.intent == "mission" and now-self.last_recovery_sim > 2:
                    resume_remaining = self.recovery_observed and bool(self.cycle.remaining)
                    if self.recovery_observed:
                        self.event("RECOVERY_NOT_CONFIRMED", before=self.recovery_before,
                                   after=dict(self.last_metrics),
                                   remaining_intents=[item[0] for item in self.cycle.remaining])
                        self.recovery_observed = False
                    if not resume_remaining:
                        self.cycle.remaining = self.recovery_intents(candidate)
                        self.recovery_origin = xyz(self.odom.pose.pose.position)
                        self.recovery_after_information = None
                        self.recovery_observation_request_id = None
                        self.recovery_step_request_id = None
                        self.recovery_information_improved = None
                        self.last_recovery_sim = now
        position = xyz(self.odom.pose.pose.position)
        self._check_execution_progress(now, position)
        if self.execution_fault:
            self.status()
            return
        speed = float(np.linalg.norm(xyz(self.odom.twist.twist.linear)))
        if (self.active and self.recovery_release_active
                and self.recovery_release_waypoint is not None
                and np.linalg.norm(position - self.recovery_release_waypoint) < 0.35
                and speed < 0.20):
            self.recovery_release_active = False
            self.recovery_release_complete = True
            self.event("RECOVERY_RELEASE_WAYPOINT_REACHED", position=position.tolist())
            self.revoke("RECOVERY_RELEASE_WAYPOINT_REACHED")
            self.recovery_release_waypoint = None
            self.request(self.goal, scale=self.mission_speed_scale)
        elif (self.active and not self.recovery_required
                and self.localization_health.get("healthy") is True
                and self.cycle.intent == "mission"
                and np.linalg.norm(position - self.goal) < self.goal_tolerance
                and speed < EXECUTION_GOAL_SPEED_MPS):
            self.completed = True
            self.revoke("GOAL_REACHED")
        elif self.cycle.intent != "mission" and self.cycle.phase == "EXECUTING" and np.linalg.norm(position-self.target) < 0.1 and speed < 0.15:
            self.revoke("RECOVERY_STEP_DONE")
            self.cycle.arrived(now)
            self.recovery_step_request_id = self.cycle.request
            self.observing_until = now + self.recovery_observation_window
            self.event("RECOVERY_STEP_DONE")
        elif self.cycle.phase == "OBSERVING" and now >= self.observing_until and self.cycle.observed(stamp_s(self.integrity.header.stamp)):
            after_information = self._fixed_information_snapshot(
                self.recovery_before_information["direction"]
                if self.recovery_before_information else None
            )
            information_before = (self.recovery_step_before_information
                                  or self.recovery_before_information)
            information_improved = bool(
                information_before and after_information
                and after_information["stamp_s"] > information_before["stamp_s"]
                and after_information["stamp_s"] > self.cycle.observation_after
                and after_information["raw_stamp_s"] > max(
                    information_before["raw_stamp_s"], self.cycle.observation_after)
                and after_information["variance_m2"]
                <= information_before["variance_m2"] - RECOVERY_MIN_INFORMATION_GAIN_M2
                and after_information["geometric_information"]
                > information_before["geometric_information"]
                + 1.0e-9 * max(1.0, information_before["geometric_information"])
                and after_information["raw_geometric_information"]
                > information_before["raw_geometric_information"]
                + 1.0e-9 * max(1.0, information_before["raw_geometric_information"])
            )
            self.event("NEW_OBSERVATION", before=self.recovery_before,
                       observed_PL=float(self.integrity.weak_direction_protection_level),
                       information_before=self.recovery_before_information,
                       information_step_before=information_before,
                       information_after=after_information,
                       information_improved=information_improved)
            self.recovery_observed = True
            self.recovery_information_improved = information_improved
            self.recovery_after_information = after_information
            self.recovery_observation_request_id = self.cycle.request
            if information_improved:
                self._retain_recovered_height(position)
            target = self.goal
            scale = self.mission_speed_scale
            if (self.recovery_release_waypoint is not None
                    and not self.recovery_release_complete):
                target = self.recovery_release_waypoint
                scale = self.recovery_release_speed_scale
            self.request(target, scale=scale)
        elif not self.completed and not self.active and self.cycle.phase != "OBSERVING" and now-self.last_plan_sim > 1:
            settling_recovery = self.cycle.intent != "mission" and self.cycle.phase == "EXECUTING"
            if settling_recovery and now-self.last_plan_sim <= RECOVERY_SETTLE_TIMEOUT_S:
                # EGO splines can end before the vehicle has braked below the
                # arrival-speed threshold. Keep the request correlated while
                # the arbiter holds instead of skipping to the next intent.
                pass
            elif self.cycle.phase == "PLANNING" and time.monotonic()-self.last_request_wall < 3:
                pass
            else:
                if settling_recovery:
                    self.event("RECOVERY_STEP_TIMEOUT", intent=self.cycle.intent,
                               target=self.target.tolist(), position=position.tolist(), speed=speed)
                    self.cycle.phase = "WAITING"
                if self.cycle.remaining:
                    name, target, scale = self.cycle.remaining.pop(0)
                    if name == "short_hover":
                        self._capture_recovery_step_baseline()
                        self.cycle.intent = name
                        self.cycle.arrived(now)
                        self.observing_until = now + self.recovery_observation_window
                    else:
                        self.request(target, name, scale)
                else:
                    self.request(self.goal, scale=self.mission_speed_scale)
        self.status()

    def status(self):
        self.status_pub.publish(String(data=json.dumps(dict(
            session_id=self.session, sim_time=self.now_s(), enabled=self.enabled,
            completed=self.completed, reset=self.reset, intent=self.cycle.intent,
            phase=self.cycle.phase, request_id=self.cycle.request,
            recovery_spent_sim_s=self.recovery_spent_sim,
            authorized=bool(self.active), fail_closed=bool(self.execution_fault),
            execution_fault=self.execution_fault, calibration_sha256=self.calibration_sha,
            ground_truth_subscribed=False, **self.last_metrics), allow_nan=False)))


def main(args=None):
    main_node(SITLSupervisor, args)

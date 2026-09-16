"""Final EGO-spline certification and short-step recovery; no truth subscriptions."""
from __future__ import annotations
import hashlib
import json
import math
import time
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
from xq_sim_interfaces.msg import (DirectionalIntegrity, InformationMap, PlannerGoal,
                                  PlannerCandidate, TrajectoryAuthorization)
from .alert_limit import sample_bspline, compute_alert_limit
from .minimum_excitation import (generate_discrete_candidates, build_information_profile,
                                 CandidateForecast, evaluate_candidate)
from .p10_active_perception_node import _cloud_xyz
from .sitl_integrity import certify_final, RecoveryCycle


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
                           "event_file": "", "margin_reserve": 0.10}.items():
            self.declare_parameter(key, value)
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
        self.limit = float(self.get_parameter("speed_limit").value)
        self.cycle = RecoveryCycle()
        self.odom = self.cloud = self.integrity = self.information = None
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
        self.recovery_observed = False
        self.observing_until = None
        self.ready_wall = 0.
        self.candidate_highwater = 0
        self.last_metrics = {}
        self.stage = "WAIT"
        self.stage_wall = 0.
        self.goal_pub = self.create_publisher(PlannerGoal, "/impact/planner_goal", 10)
        self.auth_pub = self.create_publisher(TrajectoryAuthorization, "/impact/authorization", 20)
        self.spline_pub = self.create_publisher(Bspline, "/impact/certified_bspline", 20)
        self.status_pub = self.create_publisher(String, "/impact/status", 10)
        for kind, topic, callback in [
            (Odometry, "/localization/odom", lambda m: self.input("odom", m)),
            (PointCloud2, "/xq/p5/cloud_map", lambda m: self.input("cloud", m)),
            (DirectionalIntegrity, "/integrity/directional", lambda m: self.input("integrity", m)),
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
        for name in ("odom", "cloud", "integrity"):
            value = getattr(self, name)
            if value is None or value.header.frame_id != "xq_lio_map":
                return False
            if not 0 <= now - stamp_s(value.header.stamp) <= 0.5:
                return False
            if time.monotonic() - self.received[name] > 0.5:
                return False
        return True

    def request(self, target, intent="mission", scale=1.0):
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
        if not self.enabled or self.reset or message.session_id != self.session:
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
        obstacles = _cloud_xyz(self.cloud)
        # Conservative voxel representatives: reserve an extra voxel diagonal below.
        if len(obstacles):
            _, indices = np.unique(np.floor(obstacles / 0.1).astype(np.int64), axis=0, return_index=True)
            obstacles = obstacles[indices]
        error = 0.
        if tracking:
            first = sample_bspline(points, np.array(b.knots), b.order, 0.05,
                                    b.knots[b.order] + elapsed)[0]
            error = float(np.linalg.norm(first - xyz(self.odom.pose.pose.position)))
        age = max(self.now_s() - stamp_s(m.header.stamp) for m in (self.odom, self.cloud, self.integrity))
        result = certify_final(points, np.array(b.knots), b.order, obstacles,
            np.array(self.integrity.integrity_covariance).reshape(3, 3), strategy=self.strategy,
            k_alpha=self.k, elapsed=elapsed, input_age=age, tracking_error=error,
            speed_limit=self.limit, reserve=float(self.get_parameter("margin_reserve").value),
            body_radius=0.35 + math.sqrt(3) * 0.1)
        self.last_metrics = dict(AL=result.alert, PL=result.protection, margin=result.margin,
                                 speed_bound=result.speed, tracking_error=error,
                                 critical_direction=list(result.direction),
                                 source_stamp=stamp_s(self.integrity.header.stamp))
        self.event("CERTIFICATION_TIMING", elapsed_wall_s=time.perf_counter()-started,
                   input_age_sim_s=age, remaining_recheck=tracking,
                   trajectory_id=b.traj_id)
        return result

    def recovery_intents(self, rejected):
        """Rank finite P10 intents with map information; authorize none here."""
        if self.information is None or not self.information.valid or self.information.ground_truth_used:
            return []
        if not 0 <= self.now_s() - stamp_s(self.information.header.stamp) <= 0.5:
            return []
        position = xyz(self.odom.pose.pose.position)
        direction = self.goal - position
        direction[2] = 0
        norm = np.linalg.norm(direction)
        if norm < 0.1:
            return []
        baseline = position + np.linspace(0, min(norm, 1.), 9)[:, None] * direction / norm
        candidates = generate_discrete_candidates(baseline, baseline_duration=3.)
        info = self.information
        output = []
        for candidate in candidates:
            if candidate.name == "baseline":
                continue
            try:
                alert = compute_alert_limit(candidate.positions, _cloud_xyz(self.cloud),
                    speed_mps=0.3, latency_p99_s=0.15, maximum_acceleration_mps2=1.,
                    body_radius_m=0.35, base_reserve_m=0.10, tracking_reserve_m=0.10)
                profile = build_information_profile(candidate.positions,
                    np.array([xyz(p) for p in info.positions]), np.array([xyz(p) for p in info.normals]),
                    np.array(info.static_confidence), np.array(info.geometry_quality), np.array(info.last_seen_s),
                    now=self.now_s(), visibility_radius=0.55, age_time_constant=10., information_scale=2500.)
                forecast = evaluate_candidate(CandidateForecast(candidate, alert.alert_limits,
                    alert.obstacle_directions, profile), np.array(self.integrity.integrity_covariance).reshape(3,3),
                    k_alpha=self.k, margin_reserve=0.10, baseline_duration=3.,
                    lambda_energy=0.25, lambda_distance=1., minimum_prediction_variance=1e-5)
                # Future improvement only ranks actions whose rough geometry is feasible.
                if not forecast.feasible:
                    continue
                middle = candidate.positions[len(candidate.positions)//2]
                if candidate.name == "backtrack": middle = candidate.positions[1]
                if candidate.name == "short_hover": middle = position
                if not 0.65 <= middle[2] <= 2.9:
                    continue
                scale = 0.5 if candidate.name == "slow_trajectory" else 1.0
                output.append((forecast.cost, candidate.name, middle, scale, forecast.minimum_margin))
            except (ValueError, np.linalg.LinAlgError):
                continue
        output.sort(key=lambda item: (item[0], item[1]))
        self.event("RECOVERY_FORECAST", candidates=[dict(name=x[1], cost=x[0], predicted_margin=x[4]) for x in output])
        return [(x[1], x[2], x[3]) for x in output]

    def tick(self):
        now = self.now_s()
        if now < self.last_sim - 1e-6:
            self.revoke("CLOCK_RESET")
            self.reset, self.enabled = True, False
            self.pending = None
            self.odom = self.cloud = self.integrity = self.information = None
            self.event("CLOCK_RESET_RESTART_REQUIRED")
        self.last_sim = now
        if time.monotonic() - self.stage_wall > 0.5:
            self.enabled = False
        if not self.enabled or not self.fresh():
            self.revoke("DISABLED_OR_STALE")
            self.status()
            return
        if self.active:
            try:
                result = self.certify(self.active, tracking=True)
                if not result.accepted:
                    self.revoke(result.reason)
                else:
                    self.authorization(self.active, True, "REVALIDATED")
            except (ValueError, np.linalg.LinAlgError):
                self.revoke("EXPIRED_OR_INVALID")
        if self.pending:
            candidate, self.pending = self.pending, None
            try:
                result = self.certify(candidate, tracking=True)
                accepted, reason = result.accepted, result.reason
            except (ValueError, np.linalg.LinAlgError) as error:
                accepted, reason = False, str(error)
            self.event("CERTIFY", accepted=accepted, reason=reason,
                       trajectory_id=candidate.trajectory.traj_id, **self.last_metrics)
            self.authorization(candidate, accepted, reason)
            if accepted:
                self.active = candidate
                self.spline_pub.publish(candidate.trajectory)
                self.cycle.result(candidate.request_id, True)
                if self.cycle.intent == "mission" and self.recovery_observed and self.recovery_before:
                    before, after = self.recovery_before, self.last_metrics
                    da, dp = after["AL"]-before["AL"], after["PL"]-before["PL"]
                    self.event("RECOVERY_CONFIRMED", before=before, after=after,
                               delta_AL=da, delta_PL=dp, delta_margin=da-dp,
                               comparison="Each final spline uses its own critical point/direction")
                    self.recovery_observed = False
            elif not self.active:
                self.cycle.result(candidate.request_id, False)
                if self.strategy == "recovery" and self.cycle.intent == "mission" and now-self.last_recovery_sim > 2:
                    self.cycle.remaining = self.recovery_intents(candidate)
                    self.recovery_origin = xyz(self.odom.pose.pose.position)
                    self.recovery_before = dict(self.last_metrics)
                    self.recovery_observed = False
                    self.last_recovery_sim = now
        position = xyz(self.odom.pose.pose.position)
        speed = float(np.linalg.norm(xyz(self.odom.twist.twist.linear)))
        if np.linalg.norm(position - self.goal) < 0.45 and speed < 0.15:
            self.completed = True
            self.revoke("GOAL_REACHED")
        elif self.cycle.intent != "mission" and self.cycle.phase == "EXECUTING" and np.linalg.norm(position-self.target) < 0.2 and speed < 0.15:
            self.revoke("RECOVERY_STEP_DONE")
            self.cycle.arrived(now)
            self.observing_until = now + 0.5
            self.event("RECOVERY_STEP_DONE")
        elif self.cycle.phase == "OBSERVING" and now >= self.observing_until and self.cycle.observed(stamp_s(self.integrity.header.stamp)):
            self.event("NEW_OBSERVATION", before=self.recovery_before,
                       observed_PL=float(self.integrity.weak_direction_protection_level))
            self.cycle.remaining.clear()
            self.recovery_observed = True
            self.request(self.goal)
        elif not self.completed and not self.active and self.cycle.phase != "OBSERVING" and now-self.last_plan_sim > 1:
            if self.cycle.phase == "PLANNING" and time.monotonic()-self.last_request_wall < 3:
                pass
            elif self.cycle.remaining:
                name, target, scale = self.cycle.remaining.pop(0)
                if name == "short_hover":
                    self.cycle.intent = name
                    self.cycle.arrived(now)
                    self.observing_until = now + 1.
                else:
                    self.request(target, name, scale)
            else:
                self.request(self.goal)
        self.status()

    def status(self):
        self.status_pub.publish(String(data=json.dumps(dict(
            session_id=self.session, sim_time=self.now_s(), enabled=self.enabled,
            completed=self.completed, reset=self.reset, intent=self.cycle.intent,
            phase=self.cycle.phase, request_id=self.cycle.request,
            authorized=bool(self.active), calibration_sha256=self.calibration_sha,
            ground_truth_subscribed=False, **self.last_metrics), allow_nan=False)))


def main(args=None):
    main_node(SITLSupervisor, args)

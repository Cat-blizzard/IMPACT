"""Independent 3-D truth evaluator and time-aligned recording for P16."""
import json
import math
import time
from collections import deque
from pathlib import Path
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from .sitl_supervisor_node import xyz, stamp_s, main_node


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x*x+y*y+z*z+w*w
    if not math.isfinite(n) or abs(n-1.) > 0.02:
        raise ValueError("invalid orientation")
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def box_clearance(position, boxes, radius=0.35):
    """Signed distance to oriented world boxes, minus the vehicle envelope."""
    best = math.inf
    for box in boxes:
        c, s = math.cos(box.get("yaw", 0.)), math.sin(box.get("yaw", 0.))
        p = np.asarray(position)-np.asarray(box["center"])
        local = np.array((c*p[0]+s*p[1], -s*p[0]+c*p[1], p[2]))
        d = np.abs(local)-np.asarray(box["size"])/2
        distance = np.linalg.norm(np.maximum(d, 0)) + min(float(np.max(d)), 0.)
        best = min(best, float(distance)-radius)
    return best


class SITLEvaluator(Node):
    def __init__(self):
        super().__init__("impact_evaluator")
        for key in ("result_dir", "scenario_file"):
            self.declare_parameter(key, "")
        self.root = Path(self.get_parameter("result_dir").value)
        self.boxes = json.loads(Path(self.get_parameter("scenario_file").value).read_text())["boxes"]
        self.truth = deque(maxlen=300)
        self.transform = None
        self.status_data = {}
        self.phase = "WAIT"
        self.squared_errors = []
        self.coverage = []
        self.authorized_samples = 0
        self.integrity_violations = 0
        self.samples = self.collision_samples = self.collision_events = self.hmi = 0
        self.was_collision = False
        self.minimum_clearance = math.inf
        self.previous = None
        self.path_length = self.stopped_time = 0.
        self.started_wall = time.monotonic()
        self.start_sim = None
        self.last_active_sim = None
        self.previous_authorized = False
        self.braking_since = None
        self.stopping_durations = []
        self.dropped_pairs = 0
        self.telemetry = open(self.root / "telemetry.jsonl", "a", buffering=1)
        self.create_subscription(Odometry, "/xq/eval/p5/ground_truth", self.truth_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/localization/odom", self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(String, "/impact/status", self.status_cb, 10)
        self.create_subscription(String, "/impact/mission_stage", self.stage_cb, 10)
        self.create_timer(1., self.write)

    def truth_cb(self, msg):
        self.truth.append(msg)

    def stage_cb(self, msg):
        try: self.phase = json.loads(msg.data)["phase"]
        except (ValueError, KeyError): pass

    def status_cb(self, msg):
        try: self.status_data = json.loads(msg.data)
        except ValueError: pass

    def odom_cb(self, msg):
        if not self.truth:
            return
        stamp = stamp_s(msg.header.stamp)
        truth = min(self.truth, key=lambda m: abs(stamp_s(m.header.stamp)-stamp))
        if abs(stamp_s(truth.header.stamp)-stamp) > 0.05:
            self.dropped_pairs += 1
            return
        gt, estimate = xyz(truth.pose.pose.position), xyz(msg.pose.pose.position)
        if self.transform is None:
            try:
                r = rotation(truth.pose.pose.orientation) @ rotation(msg.pose.pose.orientation).T
                self.transform = r, gt-r@estimate
            except ValueError:
                return
        if self.phase != "ACTIVE":
            self.previous = None
            return
        r, t = self.transform
        error = float(np.linalg.norm(r@estimate+t-gt))
        if not math.isfinite(error):
            return
        clearance = box_clearance(gt, self.boxes)
        collision = clearance <= 0
        self.collision_samples += int(collision)
        self.collision_events += int(collision and not self.was_collision)
        self.was_collision = collision
        self.minimum_clearance = min(self.minimum_clearance, clearance)
        self.samples += 1
        self.squared_errors.append(error*error)
        if self.start_sim is None: self.start_sim = stamp
        self.last_active_sim = stamp
        speed = 0.
        if self.previous:
            old_stamp, old_gt = self.previous
            dt = stamp-old_stamp
            length = float(np.linalg.norm(gt-old_gt))
            self.path_length += length
            if dt > 0:
                speed = length/dt
                if speed < 0.05: self.stopped_time += dt
        self.previous = stamp, gt
        state = self.status_data
        state_fresh = abs(stamp-state.get("sim_time", -1e6)) <= 0.2
        if state_fresh:
            authorized = bool(state.get("authorized"))
            if self.previous_authorized and not authorized:
                self.braking_since = stamp
            if self.braking_since is not None and speed < 0.05:
                self.stopping_durations.append(stamp-self.braking_since)
                self.braking_since = None
            if authorized:
                self.braking_since = None
            self.previous_authorized = authorized
        projected_error = None
        direction = np.asarray(state.get("critical_direction", []))
        if state_fresh and direction.shape == (3,) and abs(np.linalg.norm(direction)-1) < 1e-5:
            projected_error = abs(float(np.dot(direction, estimate-r.T@(gt-t))))
            self.coverage.append(projected_error <= state["PL"])
            self.authorized_samples += int(bool(state.get("authorized")))
            self.integrity_violations += int(bool(state.get("authorized")) and projected_error > state["AL"])
        # PL in status is directional. Do not compare it to the 3-D ATE norm.
        row = dict(sim_time=stamp, truth=gt.tolist(), estimated_world=(r@estimate+t).tolist(),
            error_3d_m=error, clearance_m=clearance, speed_mps=speed,
            phase=state.get("phase"), intent=state.get("intent"),
            AL=state.get("AL") if state_fresh else None,
            PL=state.get("PL") if state_fresh else None,
            margin=state.get("margin") if state_fresh else None,
            authorized=state.get("authorized") if state_fresh else None,
            projected_error_m=projected_error,
            truth_evaluation_only=True)
        if collision and state_fresh and state.get("authorized"):
            self.hmi += 1
        self.telemetry.write(json.dumps(row, allow_nan=False)+"\n")

    def write(self):
        elapsed = self.last_active_sim-self.start_sim if self.last_active_sim is not None else 0.
        data = dict(schema_version=1, validation="SIMULATED", samples=self.samples,
            collision_samples=self.collision_samples, collision_events=self.collision_events,
            authorized_collision_samples=self.hmi, matched_pose_tolerance_s=0.05,
            dropped_pose_pairs=self.dropped_pairs, path_length_m=self.path_length,
            stopped_time_sim_s=self.stopped_time,
            revoke_to_stop_sim_s=self.stopping_durations,
            collision_definition="Signed distance to independent world OBBs minus 0.35 m vehicle envelope <= 0; geometric envelope contact, not a Gazebo contact-sensor label",
            minimum_truth_clearance_m=self.minimum_clearance if self.samples else None,
            ate_rms_m=math.sqrt(float(np.mean(self.squared_errors))) if self.samples else None,
            elapsed_sim_s=elapsed, elapsed_wall_s=time.monotonic()-self.started_wall,
            pl_coverage=float(np.mean(self.coverage)) if self.coverage else None,
            pl_coverage_samples=len(self.coverage),
            pl_coverage_note="Current error projected onto the reported critical trajectory direction; not whole-mission coverage",
            integrity_violation_samples=self.integrity_violations,
            availability=self.authorized_samples/len(self.coverage) if self.coverage else None,
            independent_geometry=True, fixed_initial_pose_alignment=True)
        # This status describes independent geometric evaluation, not mission success.
        data["checks"] = {
            "sufficient_samples": self.samples >= 50,
            "collision_free": self.collision_events == 0 and self.collision_samples == 0,
            "finite_metrics": self.samples > 0 and all(math.isfinite(data[key]) for key in
                ("minimum_truth_clearance_m", "ate_rms_m", "path_length_m")),
        }
        data["status"] = ("PASS" if all(data["checks"].values()) else
                          "IN_PROGRESS" if self.samples < 50 else "FAIL")
        path = self.root / "evaluation.json"
        temp = path.with_suffix(".tmp")
        temp.write_text(json.dumps(data, indent=2)+"\n")
        temp.replace(path)


def main(args=None):
    main_node(SITLEvaluator, args)

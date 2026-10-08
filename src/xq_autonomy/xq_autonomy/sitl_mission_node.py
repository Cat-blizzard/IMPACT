"""Reuse P4 GPS-off handshake; run P16 mission and confirm termination separately."""
import json
import time
from rclpy.clock import Clock, ClockType
from pathlib import Path
from std_msgs.msg import String
from xq_sim_interfaces.msg import LocalizationGeometry
from rclpy.qos import qos_profile_sensor_data
from .localization_health import geometry_health
from .p4_mission_node import P4MissionNode
from .sitl_supervisor_node import main_node


class SITLMission(P4MissionNode):
    POSITION_CONTROL_PHASES = frozenset()

    def __init__(self):
        super().__init__()
        # The inherited timer used ROS time. Wall watchdogs must still run while
        # Gazebo is paused or /clock disappears; task durations below remain ROS time.
        for timer in list(self.timers):
            self.destroy_timer(timer)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self._tick, clock=self.wall_clock)
        self.declare_parameter("session_id", "")
        self.declare_parameter("task_timeout_sim_s", 180.)
        # GPU SITL can briefly delay the supervisor/arbiter status callback
        # while a large point-cloud callback is serialized.  Keep this as a
        # bounded wall watchdog rather than treating one missed callback as a
        # flight failure; estimator/FCU health gates remain independent.
        self.declare_parameter("control_status_timeout_s", 1.0)
        self.declare_parameter("geometry_max_age_s", 0.7)
        # A development policy floor: each translational direction must carry
        # at least 2% of instantaneous information before takeoff. Do not use
        # accumulated frames to make a rank-deficient scene appear healthy.
        self.declare_parameter("takeoff_minimum_weak_fraction", 0.02)
        self.declare_parameter("takeoff_minimum_geometry_points", 30)
        self.geometry = None
        self.geometry_wall = 0.
        self.create_subscription(LocalizationGeometry, "/localization/geometry",
                                 self.geometry_cb, qos_profile_sensor_data)
        self.session = self.get_parameter("session_id").value
        self.stage_pub = self.create_publisher(String, "/impact/mission_stage", 10)
        self.create_subscription(String, "/impact/status", self.status_cb, 10)
        self.create_subscription(String, "/impact/arbiter_status", self.arbiter_cb, 10)
        self.task_status = {}
        self.task_wall = self.arbiter_wall = 0.
        self.arbiter_status = {}
        self.task_started = None
        self.task_ended = None
        self.task_success = False
        self.task_reason = "NOT_STARTED"
        self.last_sim = -1.

    def geometry_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        previous = self.geometry
        if previous is not None:
            previous_stamp = previous.header.stamp.sec + previous.header.stamp.nanosec * 1e-9
            if stamp <= previous_stamp:
                return
        self.geometry, self.geometry_wall = msg, time.monotonic()

    def _health_snapshot(self, require_prearm=False):
        snapshot = super()._health_snapshot(require_prearm=require_prearm)
        reasons = list(snapshot["reasons"])
        measured = dict(healthy=False, reasons=["geometry_missing"])
        geometry = self.geometry
        if geometry is not None:
            measured = geometry_health(
                geometry.information_matrix, geometry.effective_points,
                minimum_weak_fraction=float(self.get_parameter("takeoff_minimum_weak_fraction").value),
                minimum_points=int(self.get_parameter("takeoff_minimum_geometry_points").value))
            geometry_stamp = geometry.header.stamp.sec + geometry.header.stamp.nanosec * 1e-9
            wall_age = time.monotonic() - self.geometry_wall
            sim_age = self.get_clock().now().nanoseconds / 1e9 - geometry_stamp
            maximum_age = float(self.get_parameter("geometry_max_age_s").value)
            measured.update(stamp_s=geometry_stamp, wall_age_s=wall_age, sim_age_s=sim_age)
            if wall_age > maximum_age or sim_age > maximum_age or sim_age < -0.05:
                measured["reasons"].append("geometry_stale")
                measured["healthy"] = False
            if geometry.header.frame_id != self.raw_odom_frame_id:
                measured["reasons"].append("geometry_frame_mismatch")
                measured["healthy"] = False
        # ACTIVE degradation belongs to trajectory revocation/recovery. Missing
        # or invalid sensor evidence still terminates flight independently.
        rank_required = self.phase in {
            "WAIT_FCU", "SET_STREAM", "VERIFY_NAV", "WAIT_PREARM", "SET_GUIDED",
            "ARM", "TAKEOFF", "ASCEND", "HOVER"}
        reasons.extend(reason for reason in measured["reasons"]
                       if rank_required or reason != "geometry_translation_unobservable")
        snapshot.update(healthy=not reasons, reasons=reasons,
                        localization_geometry={**measured, "takeoff_rank_required": rank_required})
        return snapshot

    def status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data["session_id"] == self.session:
                self.task_status, self.task_wall = data, time.monotonic()
        except (ValueError, KeyError):
            pass

    def arbiter_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data["session_id"] == self.session:
                self.arbiter_status, self.arbiter_wall = data, time.monotonic()
        except (ValueError, KeyError):
            pass

    def _failure_to_land(self, reason):
        if self.finalized or self.phase in ("LAND", "DESCEND", "FAILSAFE_WAIT"):
            return
        self.task_success = False
        self.task_reason = reason
        if self.task_started is not None and self.task_ended is None:
            self.task_ended = self.get_clock().now().nanoseconds / 1e9
        super()._failure_to_land(reason)

    def _finish(self, status, reason):
        if self.finalized:
            return
        if self.task_failure_reason:
            status = "FAIL"
            self.task_success = False
            self.task_reason = self.task_failure_reason
        elif status != "PASS" and not self.task_success and self.task_reason == "NOT_STARTED":
            self.task_reason = reason
        termination = self._termination_evidence()
        final_health = self._health_snapshot(require_prearm=True)
        preflight_refusal = dict(
            confirmed=bool(status != "PASS" and self.task_started is None
                           and not termination["armed_seen"]
                           and termination["confirmed"]
                           and final_health["reasons"]),
            phase=self.phase, reasons=list(final_health["reasons"]))
        state_fresh = bool(termination["state_fresh"])
        termination_confirmed = bool(termination["confirmed"])
        if not self.termination_reason:
            if termination_confirmed:
                self.termination_reason = (
                    "FCU disarmed in LAND after flight"
                    if termination["armed_seen"] else "FCU remained disarmed"
                )
            else:
                self.termination_reason = "FCU termination not confirmed"
        self.finalized = True
        self.phase = "DONE" if status == "PASS" else "FAILED"
        self._event(status, reason)
        result = dict(schema_version=1, validation="SIMULATED", session_id=self.session,
            status=status, reason=reason, task_success=self.task_success,
            task_reason=self.task_reason, termination_confirmed=termination_confirmed,
            termination={**termination, "confirmed": termination_confirmed,
                "state_fresh": state_fresh, "reason": self.termination_reason,
                "armed": bool(self.fcu_state.armed), "mode": self.fcu_state.mode,
                "state_age_s": (time.monotonic()-self.fcu_state_last_wall
                                if self.fcu_state_last_wall else None)},
            final_health=final_health, preflight_refusal=preflight_refusal,
            task_controller_failure=self.task_status.get("execution_fault"),
            verified_parameters=self.verified_params, events=self.events,
            elapsed_wall_s=time.monotonic()-self.started,
            elapsed_sim_s=None if self.task_started is None else (self.task_ended if self.task_ended is not None else self.get_clock().now().nanoseconds/1e9)-self.task_started)
        path = Path(self.get_parameter("result_file").value)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
        tmp.replace(path)

    def _tick(self):
        if not hasattr(self, "stage_pub"):
            return
        now = self.get_clock().now().nanoseconds / 1e9
        wall = time.monotonic()
        self.stage_pub.publish(String(data=json.dumps(dict(session_id=self.session, phase=self.phase, sim_time=now))))
        if self.finalized:
            return
        if now < self.last_sim - 1e-6:
            self.task_success = False
            self.task_reason = self.task_failure_reason = "CLOCK_RESET"
            self._failure_to_land("CLOCK_RESET")
        self.last_sim = now
        if self.phase in ("LAND", "DESCEND"):
            # These phases run locally, so retain the parent's shared budget
            # across FAILSAFE_WAIT -> LAND -> DESCEND instead of restarting it.
            termination_started = self.termination_started
            if termination_started is None:
                termination_started = self.phase_started
            if wall-termination_started > float(self.get_parameter("failsafe_termination_timeout_s").value):
                confirmed = bool(self._termination_evidence()["confirmed"])
                self.termination_reason = ("FCU disarmed at termination deadline" if confirmed
                                           else "FCU termination not confirmed before timeout")
                self._event("TERMINATION_CONFIRMED" if confirmed else "TERMINATION_UNCONFIRMED",
                            self.termination_reason)
                self._finish(
                    "PASS" if confirmed and self.task_success and not self.task_failure_reason else "FAIL",
                    self.task_reason if confirmed else (self.task_failure_reason or "termination deadline exceeded"),
                )
                return
        if self.phase in ("HOVER", "ACTIVE"):
            _, snapshot = self._observe_health(require_params=False, require_prearm=True)
            reasons = list(snapshot["reasons"])
            if not self._fcu_state_is_fresh():
                reasons.append("fcu_state_stale")
            if not self.fcu_state.armed:
                reasons.append("fcu_disarmed_during_task")
            if str(self.fcu_state.mode).upper() != "GUIDED":
                reasons.append("fcu_not_guided")
            # The supervisor owns trajectory certification and execution
            # progress.  Once it fail-closes, terminate the task immediately;
            # waiting for the generic task timeout would leave an active FCU
            # mode running after the controller has withdrawn authorization.
            if self.task_status.get("fail_closed") is True:
                fault = self.task_status.get("execution_fault") or {}
                reason = str(fault.get("reason") or "EXECUTION_FAIL_CLOSED")
                reasons.append(f"supervisor:{reason}")
            if reasons:
                self._flight_health_loss({**snapshot, "reasons": reasons})
                return
        if self.phase == "HOVER":
            # P4 has already confirmed takeoff; the arbiter maintains position.
            if wall-self.phase_started >= 1.0:
                self.task_started = now
                self._transition("ACTIVE", "enable certified EGO task")
        elif self.phase == "ACTIVE":
            status_timeout = float(self.get_parameter("control_status_timeout_s").value)
            fresh = wall-self.task_wall < status_timeout and wall-self.arbiter_wall < status_timeout
            reason = None
            if fresh and self.task_status.get("completed"):
                self.task_success, reason = True, "GOAL_REACHED"
            elif now-self.task_started > float(self.get_parameter("task_timeout_sim_s").value):
                reason = "TASK_TIMEOUT"
            elif wall-self.phase_started > 3 and (not fresh or self.arbiter_status.get("mode") in ("TF_FAILURE", "STALE_ODOM_HOLD")):
                reason = "CONTROL_HEALTH_FAILURE"
            elif wall-self.started > float(self.get_parameter("mission_timeout_s").value):
                reason = "WALL_WATCHDOG"
            if reason:
                self.task_ended = now
                if self.task_success:
                    self.task_reason = reason
                    self._transition("LAND", reason)
                else:
                    self._failure_to_land(reason)
        elif self.phase == "LAND":
            self._poll_command()
            if self.phase != "LAND":
                return
            self._send_command("land")
        elif self.phase == "DESCEND":
            if self._termination_evidence()["confirmed"]:
                self._finish("PASS" if self.task_success else "FAIL", self.task_reason)
        else:
            super()._tick()


def main(args=None):
    main_node(SITLMission, args)

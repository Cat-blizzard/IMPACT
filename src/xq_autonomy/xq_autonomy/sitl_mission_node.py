"""Reuse P4 GPS-off handshake; run P16 mission and confirm termination separately."""
import json
import time
from rclpy.clock import Clock, ClockType
from pathlib import Path
from std_msgs.msg import String
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
            if reasons:
                self._flight_health_loss({**snapshot, "reasons": reasons})
                return
        if self.phase == "HOVER":
            # P4 has already confirmed takeoff; the arbiter maintains position.
            if wall-self.phase_started >= 1.0:
                self.task_started = now
                self._transition("ACTIVE", "enable certified EGO task")
        elif self.phase == "ACTIVE":
            fresh = wall-self.task_wall < 1.0 and wall-self.arbiter_wall < 1.0
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
            if wall-self.phase_started > 30:
                self._finish("FAIL", "LAND_COMMAND_TIMEOUT")
        elif self.phase == "DESCEND":
            if self._termination_evidence()["confirmed"]:
                self._finish("PASS" if self.task_success else "FAIL", self.task_reason)
            elif wall-self.phase_started > 90:
                self._finish("FAIL", "LANDING_NOT_CONFIRMED")
        else:
            super()._tick()


def main(args=None):
    main_node(SITLMission, args)

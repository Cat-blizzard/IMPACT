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

    def _finish(self, status, reason):
        if self.finalized:
            return
        self.finalized = True
        self.phase = "DONE" if status == "PASS" else "FAILED"
        result = dict(schema_version=1, validation="SIMULATED", session_id=self.session,
            status=status, reason=reason, task_success=self.task_success,
            task_reason=self.task_reason, termination_confirmed=not self.fcu_state.armed and self.have_state,
            verified_parameters=self.verified_params, events=self.events,
            elapsed_wall_s=time.monotonic()-self.started,
            elapsed_sim_s=None if self.task_started is None else (self.task_ended or self.get_clock().now().nanoseconds/1e9)-self.task_started)
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
            self.task_reason = "CLOCK_RESET"
            self._transition("LAND", "clock reset: run must restart")
        self.last_sim = now
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
                self.task_reason = reason
                self._transition("LAND", reason)
        elif self.phase == "LAND":
            self._poll_command()
            self._send_command("land")
            if wall-self.phase_started > 30:
                self._finish("FAIL", "LAND_COMMAND_TIMEOUT")
        elif self.phase == "DESCEND":
            if self.have_state and not self.fcu_state.armed and abs(self.current_xyz[2]-self.origin_xyz[2]) <= 0.35:
                self._finish("PASS" if self.task_success else "FAIL", self.task_reason)
            elif wall-self.phase_started > 90:
                self._finish("FAIL", "LANDING_NOT_CONFIRMED")
        else:
            super()._tick()


def main(args=None):
    main_node(SITLMission, args)

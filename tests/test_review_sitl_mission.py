"""Regression checks against the actual P16 mission and inherited health methods."""
import json
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from mavros_msgs.msg import State, SysStatus

from xq_autonomy.p4_mission_node import ConsecutiveHealthGate, PREARM_CHECK, VISION_POSITION
from xq_autonomy.sitl_mission_node import SITLMission
import xq_autonomy.sitl_mission_node as mission_module


@pytest.fixture
def mission(tmp_path, monkeypatch):
    clock = SimpleNamespace(wall=1000.0, sim=10.0)
    monkeypatch.setattr(mission_module.time, "monotonic", lambda: clock.wall)
    node = object.__new__(SITLMission)
    params = dict(result_file=str(tmp_path / "mission.json"), task_timeout_sim_s=180.0,
        mission_timeout_s=700.0, health_status_max_age_s=0.7, health_odom_max_age_s=0.7,
        sys_status_max_age_s=2.5, fcu_state_max_age_s=2.5, health_fault_window_s=2.0,
        failsafe_termination_timeout_s=90.0)
    node.get_parameter = lambda key: SimpleNamespace(value=params[key])
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock.sim * 1e9)))
    node.stage_pub = SimpleNamespace(publish=lambda _: None)
    values = dict(session="review-session", finalized=False, phase="ACTIVE", started=900.0,
        phase_started=990.0, task_started=0.0, task_ended=None, task_success=False,
        task_reason="NOT_STARTED", task_failure_reason=None, termination_reason=None,
        termination_started=None, last_sim=10.0, task_wall=1000.0, arbiter_wall=1000.0,
        task_status=dict(completed=False), arbiter_status=dict(mode="TRACK"),
        have_state=True, armed_seen=True, fcu_state_last_wall=1000.0, have_sys_status=True,
        sys_status_last_wall=1000.0, have_odom=True, current_odom_last_wall=1000.0,
        current_odom_stamp=10.0, current_odom_frame_id="map", current_odom_child_frame_id="base_link",
        current_odom_stamp_violations=0, current_odom_max_gap_s=0.0, current_xyz=(0., 0., 2.),
        current_orientation=None, have_raw_odom=True, raw_odom_last_wall=1000.0,
        raw_odom_stamp=10.0, raw_odom_frame_id="xq_lio_map", raw_odom_child_frame_id="xq_base_link",
        raw_odom_stamp_violations=0, raw_odom_max_gap_s=0.0, raw_xyz=(0., 0., 2.),
        extnav_last_wall=1000.0, extnav_status=dict(healthy=True, mavros_subscribers=1, status_sequence=1),
        last_fcu_fault=None, health_samples=[], last_health_reason_key="", verified_params={},
        origin_xyz=(0., 0., 0.), events=[], pending_command=None, last_request=0.0,
        arrival_started=None, target=None)
    for key, value in values.items():
        setattr(node, key, value)
    node.fcu_state = State(connected=True, armed=True, guided=True, mode="GUIDED")
    node.fcu_sys_status = SysStatus(sensors_enabled=PREARM_CHECK | VISION_POSITION,
                                   sensors_health=PREARM_CHECK | VISION_POSITION)
    node.health_gate = ConsecutiveHealthGate(1)
    node._event = lambda kind, detail: node.events.append(dict(kind=kind, detail=detail, phase=node.phase))
    commands = []
    node._send_command = commands.append
    node._poll_command = lambda: None
    return node, clock, params, commands


@pytest.mark.parametrize("phase", ["HOVER", "ACTIVE"])
@pytest.mark.parametrize("fault, expected_phase, token", [
    ("disconnected", "LAND", "fcu_disconnected"),
    ("external_nav", "LAND", "extnav:source_stale"),
    ("ekf", "LAND", "fcu:ekf variance"),
    ("stale_state", "FAILSAFE_WAIT", "fcu_state_stale"),
    ("fcu_land", "FAILSAFE_WAIT", "fcu_not_guided"),
])
def test_flight_health_precedes_task_completion(mission, phase, fault, expected_phase, token):
    node, _, _, commands = mission
    node.phase = phase
    node.task_status["completed"] = True
    if fault == "disconnected":
        node.fcu_state.connected = False
    elif fault == "external_nav":
        node.extnav_status.update(healthy=False, health_reasons=["source_stale"])
    elif fault == "ekf":
        node.last_fcu_fault = dict(elapsed_s=100.0, reason="ekf variance")
    elif fault == "stale_state":
        node.fcu_state_last_wall = 990.0
    else:
        node.fcu_state.mode = "LAND"
    node._tick()
    assert node.phase == expected_phase
    assert not node.task_success
    assert token in node.task_reason
    assert node.task_failure_reason == node.task_reason
    assert commands == []


def test_healthy_task_can_reach_goal_and_confirm_landing(mission):
    node, _, params, _ = mission
    node.task_status["completed"] = True
    node._tick()
    assert node.phase == "LAND" and node.task_success
    node.phase = "DESCEND"
    node.current_xyz = node.origin_xyz
    node.fcu_state.armed = False
    node.fcu_state.mode = "LAND"
    node._tick()
    result = json.loads(mission_module.Path(params["result_file"]).read_text())
    assert result["status"] == "PASS" and result["task_success"]
    assert result["termination_confirmed"] and result["termination"]["state_fresh"]


def test_health_failure_reason_survives_successful_landing(mission):
    node, _, params, _ = mission
    node.extnav_status.update(healthy=False, health_reasons=["source_stale"])
    node._tick()
    reason = node.task_reason
    node.phase = "DESCEND"
    node.current_xyz = node.origin_xyz
    node.fcu_state.armed = False
    node.fcu_state.mode = "LAND"
    node._tick()
    result = json.loads(mission_module.Path(params["result_file"]).read_text())
    assert result["status"] == "FAIL" and not result["task_success"]
    assert result["task_reason"] == reason and "source_stale" in reason
    assert result["termination_confirmed"]


def test_descend_requires_fresh_disarm_in_land(mission):
    node, _, _, _ = mission
    node.phase = "DESCEND"
    node.task_success = True
    node.task_reason = "GOAL_REACHED"
    node.fcu_state.armed = False
    node.fcu_state.mode = "GUIDED"
    node._tick()
    assert not node.finalized


def test_lio_altitude_bias_is_diagnostic_after_fresh_land_disarm(mission):
    node, _, params, _ = mission
    node.phase = "DESCEND"
    node.task_success = True
    node.task_reason = "GOAL_REACHED"
    node.current_xyz = (node.origin_xyz[0], node.origin_xyz[1], node.origin_xyz[2] + 0.8)
    node.fcu_state.armed = False
    node.fcu_state.mode = "LAND"
    node._tick()
    result = json.loads(mission_module.Path(params["result_file"]).read_text())
    assert result["status"] == "PASS"
    assert result["termination"]["evidence"] == "fresh_fcu_disarm_in_land"
    assert not result["termination"]["altitude_near_origin"]


def test_stale_disarm_cannot_confirm_termination_on_timeout(mission):
    node, _, params, _ = mission
    node.phase = "DESCEND"
    node.phase_started = 900.0
    node.current_xyz = node.origin_xyz
    node.fcu_state.armed = False
    node.fcu_state.mode = "LAND"
    node.fcu_state_last_wall = 990.0
    node._tick()
    result = json.loads(mission_module.Path(params["result_file"]).read_text())
    assert result["status"] == "FAIL"
    assert not result["termination_confirmed"]
    assert not result["termination"]["state_fresh"]


def test_task_timeout_lands_without_erasing_task_failure(mission):
    node, clock, _, _ = mission
    clock.sim = 200.0
    node._tick()
    assert node.phase == "LAND"
    assert not node.task_success
    assert node.task_reason == node.task_failure_reason == "TASK_TIMEOUT"


def test_land_ack_does_not_send_duplicate_command(mission):
    node, _, _, commands = mission
    node.phase = "LAND"
    node._poll_command = lambda: node._transition("DESCEND", "landing accepted")
    node._tick()
    assert node.phase == "DESCEND" and not commands


def test_clock_reset_during_descent_cannot_restore_task_success(mission):
    node, clock, params, _ = mission
    node.phase = "DESCEND"
    node.task_success = True
    node.task_reason = "GOAL_REACHED"
    node.current_xyz = node.origin_xyz
    node.fcu_state.armed = False
    node.fcu_state.mode = "LAND"
    clock.sim = 1.0
    node._tick()
    result = json.loads(mission_module.Path(params["result_file"]).read_text())
    assert result["status"] == "FAIL" and not result["task_success"]
    assert result["task_reason"] == "CLOCK_RESET"


def test_preflight_failure_records_actual_reason(mission):
    node, _, params, _ = mission
    node._finish("FAIL", "ExternalNav parameters were not verified")
    result = json.loads(mission_module.Path(params["result_file"]).read_text())
    assert result["task_reason"] == "ExternalNav parameters were not verified"
    assert not result["task_success"]


@pytest.mark.parametrize("disarmed", [False, True])
def test_shared_termination_budget_survives_failsafe_land_and_descend(mission, disarmed):
    node, clock, params, commands = mission
    node.fcu_state_last_wall = 900.0
    node._failure_to_land("CONTROL_HEALTH_FAILURE")
    assert node.phase == "FAILSAFE_WAIT" and node.termination_started == 1000.0

    clock.wall = 1080.0
    node.fcu_state_last_wall = clock.wall
    node._publish_origin = lambda: None
    node._tick()  # Use the inherited FAILSAFE_WAIT recovery, not a manual phase change.
    assert node.phase == "LAND" and node.termination_started == 1000.0

    clock.wall = 1081.0
    node._poll_command = lambda: node._transition("DESCEND", "landing accepted")
    node._tick()
    assert node.phase == "DESCEND" and node.termination_started == 1000.0
    node.fcu_state.mode = "LAND"

    clock.wall = 1091.0
    node.fcu_state_last_wall = clock.wall
    node.fcu_state.armed = not disarmed
    node._tick()
    result = json.loads(mission_module.Path(params["result_file"]).read_text())
    assert node.finalized and result["status"] == "FAIL"
    assert not result["task_success"]
    assert result["task_reason"] == "CONTROL_HEALTH_FAILURE"
    assert result["termination_confirmed"] is disarmed
    assert result["termination"]["state_fresh"]
    assert result["termination"]["reason"] == (
        "FCU disarmed at termination deadline" if disarmed
        else "FCU termination not confirmed before timeout")
    assert commands == []

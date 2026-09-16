"""Regression coverage for task failure, termination budgets and smoke windows."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from startup_smoke_monitor import analyze_samples, PREARM_CHECK, VISION_POSITION
from xq_autonomy import p4_mission_node as mission


@pytest.fixture
def p4(tmp_path, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(mission, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    node = object.__new__(mission.P4MissionNode)
    node.finalized = False
    node.phase = "TRACK_SQUARE"
    node.started = 90.0
    node.phase_started = 99.0
    node.termination_started = None
    node.task_failure_reason = None
    node.termination_reason = None
    node.have_state = True
    node.fcu_state_last_wall = clock[0]
    node.fcu_state = SimpleNamespace(connected=True, armed=True, mode="GUIDED")
    node.current_xyz = (0.0, 0.0, 0.0)
    node.origin_xyz = (0.0, 0.0, 0.0)
    node.events = []
    node.commands = []
    node.arm_request_count = 1
    node.verified_params = dict(node.REQUIRED_PARAMS)
    node.extnav_status = {}
    node.health_gate = SimpleNamespace(required_samples=8, consecutive_samples=8,
                                      last_failure_reasons=[])
    node.health_stable = True
    node.health_samples = []
    node.fcu_status_texts = []
    node.completed_waypoints = []
    node.result_path = tmp_path / "mission.json"
    parameters = {"mission_timeout_s": 240.0, "failsafe_termination_timeout_s": 90.0,
                  "fcu_state_max_age_s": 2.5, "command_timeout_s": 8.0,
                  "result_file": str(node.result_path)}
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node._event = lambda kind, detail: node.events.append(
        {"kind": kind, "detail": detail, "phase": node.phase})
    node._health_snapshot = lambda **_: {"fcu_sys_status": {}}
    node._publish_origin = lambda: None
    node._publish_setpoint = lambda: None
    node._poll_command = lambda: None
    node._send_command = lambda kind: node.commands.append(kind)
    return node, clock


def test_health_failure_remains_failure_after_confirmed_landing(p4):
    node, _ = p4
    node._flight_health_loss({"reasons": ["raw_odom_stamp_nonmonotonic"]})
    node._tick()
    assert node.commands == ["land"]
    node._transition("DESCEND", "landing accepted")
    node.fcu_state.armed = False
    node.fcu_state.mode = "LAND"
    node._tick()
    result = json.loads(node.result_path.read_text())
    assert result["status"] == "FAIL"
    assert not result["task_result"]["success"]
    assert "raw_odom_stamp_nonmonotonic" in result["task_result"]["failure_reason"]
    assert result["termination"]["confirmed"]
    assert not result["checks"]["square_and_return"]


def test_finish_cannot_override_recorded_task_failure(p4):
    node, _ = p4
    node.task_failure_reason = "failed before landing"
    node.fcu_state.armed = False
    node._finish("PASS", "landed")
    result = json.loads(node.result_path.read_text())
    assert result["status"] == "FAIL"
    assert result["task_result"]["failure_reason"] == "failed before landing"
    assert result["termination"]["confirmed"]


def test_mission_timeout_still_sends_land_and_observes_disarm(p4):
    node, clock = p4
    node.started = clock[0] - 241.0
    node._tick()
    assert node.phase == "LAND"
    node._tick()
    assert node.commands == ["land"]
    clock[0] += 20.0
    node.fcu_state_last_wall = clock[0]
    node._transition("DESCEND", "landing accepted")
    assert node.termination_started == 100.0
    node.fcu_state.armed = False
    node._tick()
    result = json.loads(node.result_path.read_text())
    assert result["status"] == "FAIL"
    assert result["termination"]["confirmed"]


@pytest.mark.parametrize("phase", ["LAND", "DESCEND", "FAILSAFE_WAIT"])
def test_termination_has_one_bounded_budget_after_task_timeout(p4, phase):
    node, clock = p4
    node.started = clock[0] - 241.0
    node._failure_to_land("mission timeout")
    clock[0] += 45.0
    node._transition(phase, "termination continues")
    clock[0] += 46.0
    node.fcu_state_last_wall = clock[0]
    node._tick()
    result = json.loads(node.result_path.read_text())
    assert result["status"] == "FAIL"
    assert not result["termination"]["confirmed"]
    assert "timeout" in result["termination"]["reason"]


@pytest.mark.parametrize("mode, expected_phase, expected_commands", [
    ("GUIDED", "LAND", ["land"]), ("LAND", "FAILSAFE_WAIT", []),
])
def test_fresh_armed_state_resumes_only_missing_landing(p4, mode, expected_phase, expected_commands):
    node, clock = p4
    node.fcu_state_last_wall = clock[0] - 10.0
    node._flight_health_loss({"reasons": ["raw_odom_stale"]})
    assert node.phase == "FAILSAFE_WAIT"
    node.fcu_state_last_wall = clock[0]
    node.fcu_state.mode = mode
    node._tick()
    node._tick()
    assert node.phase == expected_phase
    assert node.commands == expected_commands
    assert node.termination_started == 100.0


def test_stale_disarm_cannot_complete_descent(p4):
    node, clock = p4
    node._transition("DESCEND", "landing accepted")
    node.fcu_state.armed = False
    node.fcu_state_last_wall = clock[0] - 10.0
    node._tick()
    assert not node.finalized
    assert not node.result_path.exists()


def smoke_samples():
    samples = {key: [] for key in ("odom", "extnav", "extnav_output", "fcu",
                                   "sys_status", "estimator", "statustext", "fcu_imu")}
    for index in range(1, 451):
        elapsed = index / 10.0
        header = SimpleNamespace(stamp=SimpleNamespace(sec=index // 10,
                                                       nanosec=(index % 10) * 100000000))
        for key in ("odom", "extnav_output"):
            samples[key].append((elapsed, elapsed, SimpleNamespace(header=header)))
        samples["extnav"].append((elapsed, elapsed, SimpleNamespace(data='{"healthy":true}')))
        samples["fcu"].append((elapsed, elapsed, SimpleNamespace(connected=True, armed=False, mode="STABILIZE")))
        status = SimpleNamespace(sensors_present=PREARM_CHECK | VISION_POSITION,
                                 sensors_enabled=PREARM_CHECK | VISION_POSITION,
                                 sensors_health=PREARM_CHECK | VISION_POSITION)
        samples["sys_status"].append((elapsed, elapsed, status))
    return samples


def test_complete_healthy_smoke_window_passes():
    report = analyze_samples(smoke_samples(), 0.0, 45.0, 45.0)
    assert report["passed"]


@pytest.mark.parametrize("topic", ["odom", "extnav", "extnav_output", "fcu", "sys_status"])
@pytest.mark.parametrize("boundary", ["start", "end"])
def test_smoke_rejects_missing_window_boundary(topic, boundary):
    samples = smoke_samples()
    samples[topic] = samples[topic][:2] if boundary == "end" else samples[topic][-2:]
    report = analyze_samples(samples, 0.0, 45.0, 45.0)
    assert not report["continuity"][topic]
    assert not report["passed"]


def test_smoke_rejects_early_observation_end():
    samples = {key: values[:100] for key, values in smoke_samples().items()}
    report = analyze_samples(samples, 0.0, 10.0, 45.0)
    assert not report["criteria"]["observation_window_completed"]
    assert not report["passed"]


@pytest.mark.parametrize("payload", ["{}", "[]", "not json"])
def test_smoke_rejects_unusable_health_message(payload):
    samples = smoke_samples()
    samples["extnav"][0][2].data = payload
    report = analyze_samples(samples, 0.0, 45.0, 45.0)
    assert not report["criteria"]["external_nav_continuously_healthy"]
    assert not report["passed"]


def test_successful_descent_without_task_failure_still_passes(p4):
    node, _ = p4
    node.completed_waypoints = [{"index": index} for index in range(1, 5)]
    node._transition("DESCEND", "landing accepted")
    node.fcu_state.armed = False
    node._tick()
    result = json.loads(node.result_path.read_text())
    assert result["status"] == "PASS"
    assert result["task_result"]["success"]
    assert result["termination"]["confirmed"]


def test_disarm_at_deadline_preserves_confirmed_termination(p4):
    node, clock = p4
    node._failure_to_land("mission timeout")
    clock[0] += 91.0
    node.fcu_state_last_wall = clock[0]
    node.fcu_state.armed = False
    node._tick()
    result = json.loads(node.result_path.read_text())
    assert result["status"] == "FAIL"
    assert result["termination"]["confirmed"]
    assert result["termination"]["reason"] == "FCU disarmed at termination deadline"
    assert result["events"][-2]["kind"] == "TERMINATION_CONFIRMED"

"""P5 landing must retain its shared termination budget after task failure."""
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from mavros_msgs.msg import State
from xq_autonomy.p5_mission_node import P5MissionNode
import xq_autonomy.p5_mission_node as mission_module
import xq_autonomy.p3_evaluator_node as p3_module


def test_p5_mission_timeout_enters_land_and_keeps_commanding(monkeypatch):
    monkeypatch.setattr(mission_module.time, "monotonic", lambda: 1000.)
    node = object.__new__(P5MissionNode)
    node.finalized = False
    node.phase = "EXPLORE"
    node.phase_started = 950.
    node.started = 100.
    node.termination_started = None
    node.have_state = True
    node.fcu_state = State(connected=True, armed=True, mode="GUIDED")
    node.task_failure_reason = None
    node.target = None
    node.get_parameter = lambda key: SimpleNamespace(value={
        "mission_timeout_s": 700., "failsafe_termination_timeout_s": 90.,
    }[key])
    node._publish_origin = lambda: None
    node._publish_setpoint = lambda: None
    node._publish_enable = lambda _: None
    node._poll_command = lambda: None
    node._event = lambda *_: None
    node._fcu_state_is_fresh = lambda: True
    commands = []
    node._send_command = commands.append
    node._failure_to_land("mission timeout in EXPLORE")
    assert node.phase == "LAND" and node.termination_started == 1000.
    node._tick()
    assert node.phase == "LAND" and commands == ["land"]
    assert not node.finalized


def test_p3_shutdown_does_not_hide_unrelated_evaluator_error(monkeypatch):
    node = SimpleNamespace(_write=lambda: None, destroy_node=lambda: None)
    monkeypatch.setattr(p3_module, "P3EvaluatorNode", lambda: node)
    monkeypatch.setattr(p3_module.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(p3_module.rclpy, "ok", lambda: False)
    def broken_spin(_):
        raise ValueError("evaluator failed")
    monkeypatch.setattr(p3_module.rclpy, "spin", broken_spin)
    with pytest.raises(ValueError, match="evaluator failed"):
        p3_module.main()

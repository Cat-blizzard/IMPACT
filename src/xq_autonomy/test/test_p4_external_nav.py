import math

import pytest

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType

from xq_autonomy.p4_external_nav_node import (
    P4ExternalNavNode,
    _reported_body_velocity,
    _world_to_body,
)
from xq_autonomy.p4_mission_node import (
    ConsecutiveHealthGate,
    P4MissionNode,
    _fcu_fault_reason,
)


def test_world_velocity_is_rotated_into_body_frame() -> None:
    half = math.sqrt(0.5)
    velocity = _world_to_body((1.0, 0.0, 0.0), (half, 0.0, 0.0, half))
    assert velocity == pytest.approx((0.0, -1.0, 0.0), abs=1e-12)


def test_external_nav_covariance_never_claims_false_certainty() -> None:
    source = [0.0] * 36
    source[0] = float("nan")
    result = P4ExternalNavNode._covariance_with_floor(source, 0.0025, 0.0012)
    assert [result[index] for index in (0, 7, 14)] == pytest.approx([0.0025] * 3)
    assert [result[index] for index in (21, 28, 35)] == pytest.approx([0.0012] * 3)
    assert all(math.isfinite(value) for value in result)


def test_esekf_velocity_is_preferred_only_when_covariance_marks_it_available() -> None:
    message = Odometry()
    message.twist.twist.linear.x = 1.25
    assert _reported_body_velocity(message) is None
    message.twist.covariance[0] = 0.01
    assert _reported_body_velocity(message) == pytest.approx((1.25, 0.0, 0.0))


def test_p4_contract_disables_gps_and_selects_external_nav() -> None:
    params = P4MissionNode.REQUIRED_PARAMS
    assert params["GPS_TYPE"] == 0
    assert params["SIM_GPS_DISABLE"] == 1
    assert params["VISO_TYPE"] == 2
    for name in (
        "EK3_SRC1_POSXY",
        "EK3_SRC1_VELXY",
        "EK3_SRC1_POSZ",
        "EK3_SRC1_VELZ",
        "EK3_SRC1_YAW",
    ):
        assert params[name] == 6


def test_position_setpoints_cannot_cancel_guided_takeoff() -> None:
    phases = P4MissionNode.POSITION_CONTROL_PHASES
    assert "TAKEOFF" not in phases
    assert "ASCEND" not in phases
    assert phases == {"HOVER", "TRACK_SQUARE"}


def test_ros_not_set_is_distinct_from_a_real_fcu_parameter_value() -> None:
    assert ParameterType.PARAMETER_NOT_SET == 0
    assert ParameterType.PARAMETER_INTEGER != ParameterType.PARAMETER_NOT_SET


def test_health_gate_requires_new_consecutive_samples_and_resets_on_fault() -> None:
    gate = ConsecutiveHealthGate(3)
    assert not gate.observe(True, [], sample_id=1)
    assert not gate.observe(True, [], sample_id=1)
    assert not gate.observe(True, [], sample_id=2)
    assert gate.observe(True, [], sample_id=3)
    assert not gate.observe(False, ["fcu:ekf variance"], sample_id=4)
    assert gate.consecutive_samples == 0
    assert not gate.observe(True, [], sample_id=4)
    assert gate.observe(True, [], sample_id=5) is False
    assert gate.observe(True, [], sample_id=6) is False
    assert gate.observe(True, [], sample_id=7) is True


@pytest.mark.parametrize(
    ("text", "reason"),
    (
        ("PreArm: VisOdom: not healthy", "visodom: not healthy"),
        ("PreArm: VisOdom: roll/pitch diff 13.5 deg (>10)", "visodom: roll/pitch diff"),
        ("EKF variance", "ekf variance"),
        ("EKF Failsafe: changed to LAND Mode", "ekf failsafe"),
        ("Potential Thrust Loss (3)", "potential thrust loss"),
    ),
)
def test_fcu_fault_text_is_classified(text: str, reason: str) -> None:
    assert _fcu_fault_reason(text) == reason


def test_benign_fcu_text_is_not_classified_as_fault() -> None:
    assert _fcu_fault_reason("EKF3 lane switch 0") is None

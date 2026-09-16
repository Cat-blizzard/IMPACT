"""Regression cases for anisotropic obstacle certification and arbiter resets."""
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest

from xq_autonomy.alert_limit import sample_bspline
from xq_autonomy.integrity_margin import compute_directional_protection_levels
from xq_autonomy.sitl_integrity import certify_final, derivative_bounds


def short_spline(duration=1.):
    points = np.column_stack((np.linspace(0., .03, 4), np.zeros(4), np.full(4, 2.)))
    return points, np.array([0.] * 4 + [duration] * 4)


@pytest.mark.parametrize("strategy", ["hard_gate", "recovery"])
def test_nearer_low_uncertainty_obstacle_cannot_hide_unsafe_direction(strategy):
    points, knots = short_spline()
    covariance = np.diag([1., 1e-6, 1e-6])
    dangerous = np.array([[3., 0., 2.]])
    distractor = np.array([[0., 2., 2.]])
    before = certify_final(points, knots, 3, dangerous, covariance,
                           strategy=strategy, k_alpha=3.)
    after = certify_final(points, knots, 3, np.vstack((dangerous, distractor)),
                          covariance, strategy=strategy, k_alpha=3.)
    assert not before.accepted and before.reason == "MARGIN"
    assert not after.accepted and after.reason == "MARGIN"
    assert after.margin <= before.margin + 1e-12
    assert after.direction == pytest.approx((1., 0., 0.))
    assert after.alert - after.protection == pytest.approx(after.margin)


@pytest.mark.parametrize("safe_cloud", [False, True])
def test_blocked_certification_matches_exhaustive_all_pairs_and_obstacle_order(safe_cloud):
    points, knots = short_spline(duration=2.)
    rng = np.random.default_rng(481)
    obstacles = rng.uniform((-4., -4., -1.), (4., 4., 5.), size=(1031, 3))
    if safe_cloud:
        offsets = obstacles - np.array([0., 0., 2.])
        obstacles = (offsets / np.linalg.norm(offsets, axis=1)[:, None]
                     * rng.uniform(4., 6., size=(len(offsets), 1)) + np.array([0., 0., 2.]))
    rotation = np.array([[.8, -.6, 0.], [.6, .8, 0.], [0., 0., 1.]])
    covariance = rotation @ np.diag([.8, .02, .003]) @ rotation.T
    speed, _ = derivative_bounds(points, knots, 3)
    interval = min(.05, .025 / max(speed, .01))
    samples = sample_bspline(points, knots, 3, interval)
    # Independent full Cartesian-product oracle, including pairs across both
    # production block boundaries. No nearest-neighbor filtering is allowed.
    delta = obstacles[None, :, :] - samples[:, None, :]
    distance = np.linalg.norm(delta, axis=2)
    directions = (delta / distance[:, :, None]).reshape(-1, 3)
    protection = compute_directional_protection_levels(directions, covariance, 3.)
    fixed_reserve = (.35 + .10 + speed * interval + .10 + speed ** 2 / (2 * .7)
                     + speed * .15 + .5 * .15 ** 2)
    limits = distance.ravel() - fixed_reserve
    margins = limits - protection
    critical = int(np.argmin(margins))
    for cloud in (obstacles, obstacles[::-1]):
        result = certify_final(points, knots, 3, cloud, covariance,
                               strategy="hard_gate", k_alpha=3.)
        assert result.margin == pytest.approx(margins[critical], abs=1e-12)
        assert result.alert == pytest.approx(limits[critical], abs=1e-12)
        assert result.protection == pytest.approx(protection[critical], abs=1e-12)
        assert result.direction == pytest.approx(directions[critical], abs=1e-12)
        assert result.accepted == bool(np.min(limits) >= 0 and margins[critical] >= .10)


def test_obstacle_at_trajectory_sample_is_a_clearance_rejection():
    points, knots = short_spline()
    result = certify_final(points, knots, 3, np.array([[0., 0., 2.]]), np.eye(3) * .01,
                           strategy="hard_gate", k_alpha=3.)
    assert not result.accepted and result.reason == "CLEARANCE"
    assert np.isfinite((result.alert, result.protection, result.margin)).all()


@pytest.mark.parametrize("covariance, obstacles", [
    (np.full((3, 3), np.nan), np.array([[0., 2., 2.]])),
    (np.diag([-1., .01, .01]), np.array([[0., 2., 2.]])),
    (np.eye(2), np.array([[0., 2., 2.]])),
    (np.eye(3), np.empty((0, 3))),
    (np.eye(3), np.array([[np.nan, 2., 2.]])),
    (np.eye(3), np.array([[np.inf, 2., 2.]])),
])
def test_invalid_certification_inputs_still_fail_closed(covariance, obstacles):
    points, knots = short_spline()
    with pytest.raises(ValueError):
        certify_final(points, knots, 3, obstacles, covariance,
                      strategy="hard_gate", k_alpha=3.)


@pytest.fixture
def arbiter(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.time import Time
    from xq_autonomy.sitl_arbiter_node import SITLArbiter
    rclpy.init(args=["--ros-args", "-p", "session_id:=reset-review"])
    node = SITLArbiter()
    now = [10.]
    monkeypatch.setattr(node, "get_clock", lambda: SimpleNamespace(now=lambda: Time(seconds=now[0])))
    publications, statuses = [], []
    node.pub = SimpleNamespace(publish=publications.append)
    node.status_pub = SimpleNamespace(publish=statuses.append)
    node.transform = lambda value: value
    node.transform_yaw = 0.
    node.phase = "ACTIVE"
    node.stage_wall = time.monotonic()
    node.guard.clock(now[0])
    node.brake_state = (np.array([10., 0., 2.]), np.array([1., 0., 0.]), 10.)
    try:
        yield node, now, publications, statuses
    finally:
        node.destroy_node()
        rclpy.shutdown()


def odometry_at(x, stamp):
    from nav_msgs.msg import Odometry
    from xq_autonomy.sitl_supervisor_node import ros_stamp
    message = Odometry()
    message.header.frame_id = "xq_lio_map"
    message.header.stamp = ros_stamp(stamp)
    message.pose.pose.position.x = float(x)
    message.pose.pose.position.z = 2.
    message.pose.pose.orientation.w = 1.
    return message


@pytest.mark.parametrize("first_detector", ["authorization", "tick"])
def test_clock_reset_discards_old_brake_regardless_of_callback_order(arbiter, first_detector):
    from geometry_msgs.msg import PoseStamped
    from quadrotor_msgs.msg import PositionCommand
    from xq_sim_interfaces.msg import TrajectoryAuthorization
    from xq_autonomy.sitl_supervisor_node import ros_stamp
    node, now, publications, statuses = arbiter
    node.odometry(odometry_at(10., 10.))
    node.position_command(PositionCommand())
    node.mission_hold(PoseStamped())
    now[0] = 1.
    if first_detector == "authorization":
        authorization = TrajectoryAuthorization()
        authorization.header.frame_id = "xq_lio_map"
        authorization.header.stamp = ros_stamp(1.)
        authorization.session_id = node.guard.session
        authorization.request_id = authorization.trajectory_id = 1
        authorization.valid_until = ros_stamp(1.)
        authorization.authorized = False
        node.authorization(authorization)
        assert node.guard.reset_latched
        # Even a new odometry callback between authorization and tick must not
        # cause the previous world's x=10 braking endpoint to be reused.
        node.odometry(odometry_at(0., 1.))
    node.tick()
    assert node.guard.reset_latched and node.guard.current is None
    assert node.command is None and node.odom is None and node.hold is None
    assert node.brake_state is None
    assert not publications
    # Continued data may establish a fresh brake/hold in the reset world's
    # coordinates, but cannot revive the old target or authorization.
    node.odometry(odometry_at(0., 1.))
    node.tick()
    assert len(publications) == 1
    position = publications[-1].pose.position
    assert (position.x, position.y, position.z) == pytest.approx((0., 0., 2.))
    status = json.loads(statuses[-1].data)
    assert status["reset"] and not status["authorized"]
    # A second discontinuity before the required process restart must also
    # discard the hold established from the first reset's coordinate origin.
    publications.clear()
    now[0] = .5
    if first_detector == "authorization":
        authorization.header.stamp = ros_stamp(.5)
        authorization.valid_until = ros_stamp(.5)
        node.authorization(authorization)
        node.odometry(odometry_at(-5., .5))
    node.tick()
    assert not publications and node.brake_state is None and node.odom is None
    node.odometry(odometry_at(-5., .5))
    node.tick()
    assert publications[-1].pose.position.x == pytest.approx(-5.)


def tracking_inputs(node, now, *, expires=10.3):
    from quadrotor_msgs.msg import PositionCommand
    from xq_sim_interfaces.msg import TrajectoryAuthorization
    from xq_autonomy.sitl_supervisor_node import ros_stamp
    node.odometry(odometry_at(0., now[0]))
    command = PositionCommand()
    command.header.frame_id = "xq_lio_map"
    command.header.stamp = ros_stamp(now[0])
    command.trajectory_id = 7
    command.position.x, command.position.z, command.yaw = .2, 2., .1
    node.position_command(command)
    auth = TrajectoryAuthorization()
    auth.header.frame_id = "xq_lio_map"
    auth.header.stamp = ros_stamp(now[0])
    auth.session_id = node.guard.session
    auth.request_id, auth.trajectory_id = 4, 7
    auth.valid_until = ros_stamp(expires)
    auth.authorized = True
    node.authorization(auth)
    return command, auth


def test_execution_telemetry_links_exact_final_setpoint_and_received_authorization(arbiter):
    node, now, publications, statuses = arbiter
    command, auth = tracking_inputs(node, now)
    node.tick()
    status = json.loads(statuses[-1].data)
    assert status["schema_version"] == 2
    assert status["mode"] == "TRACK" and status["authorized"]
    execution = status["execution"]
    final = publications[-1]
    assert execution["emitted"] and execution["frame_id"] == final.header.frame_id == "map"
    assert execution["setpoint_stamp_ns"] == final.header.stamp.sec * 10**9 + final.header.stamp.nanosec
    assert execution["position"] == pytest.approx([final.pose.position.x, final.pose.position.y, final.pose.position.z])
    assert execution["command_position"] == pytest.approx([.2, 0., 2.])
    assert execution["command_yaw"] == pytest.approx(command.yaw)
    assert execution["trajectory_id"] == command.trajectory_id
    assert execution["request_id"] == auth.request_id
    assert execution["command_stamp"] == 10.
    assert execution["authorization_issued"] == 10.
    assert execution["authorization_expires"] == pytest.approx(10.3)
    assert status["authorization"]["accepted"] is True
    assert status["authorization"]["received_sim_time"] == 10.


def test_expired_authorization_reports_false_and_brakes_old_command(arbiter):
    node, now, publications, statuses = arbiter
    tracking_inputs(node, now)
    node.tick()
    now[0] = 10.31
    node.odometry(odometry_at(0., now[0]))
    node.tick()
    status = json.loads(statuses[-1].data)
    assert node.guard.current is not None  # retained history is not active authority
    assert status["mode"] == "BRAKE" and status["authorized"] is False
    assert status["execution"]["emitted"]
    assert status["execution"]["trajectory_id"] is None
    assert publications[-1].pose.position.x == pytest.approx(0.)


def test_revocation_receipt_is_recorded_and_cannot_execute_old_target(arbiter):
    from xq_autonomy.sitl_supervisor_node import ros_stamp
    node, now, publications, statuses = arbiter
    _, auth = tracking_inputs(node, now)
    node.tick()
    now[0] = 10.1
    node.odometry(odometry_at(0., now[0]))
    auth.header.stamp = auth.valid_until = ros_stamp(now[0])
    auth.authorized = False
    node.authorization(auth)
    node.tick()
    status = json.loads(statuses[-1].data)
    assert status["mode"] == "BRAKE" and not status["authorized"]
    assert status["authorization"]["accepted"] is False
    assert status["authorization"]["received_sim_time"] == pytest.approx(10.1)
    assert status["execution"]["request_id"] is None
    assert publications[-1].pose.position.x == pytest.approx(0.)


@pytest.mark.parametrize("stale_input", ["odom", "stage", "auth"])
def test_stale_inputs_do_not_report_active_authority(arbiter, stale_input):
    node, now, _, statuses = arbiter
    tracking_inputs(node, now)
    setattr(node, stale_input + "_wall", time.monotonic() - 1.)
    node.tick()
    status = json.loads(statuses[-1].data)
    assert not status["authorized"] and status["mode"] != "TRACK"


def test_failed_transform_cannot_claim_setpoint_was_emitted(arbiter):
    node, now, publications, statuses = arbiter
    tracking_inputs(node, now)
    def invalid_transform(_):
        raise ValueError("invalid transform")
    node.transform = invalid_transform
    node.tick()
    status = json.loads(statuses[-1].data)
    assert not publications and status["mode"] == "TF_FAILURE"
    assert status["execution"] == {"emitted": False}


def test_inactive_phase_has_no_claim_of_authorized_execution(arbiter):
    node, now, publications, statuses = arbiter
    tracking_inputs(node, now)
    node.phase = "DESCEND"
    node.tick()
    status = json.loads(statuses[-1].data)
    assert not publications and not status["authorized"]
    assert status["execution"] == {"emitted": False}


def test_same_sim_tick_revocation_has_unambiguous_local_order(arbiter):
    node, now, publications, statuses = arbiter
    _, auth = tracking_inputs(node, now)
    node.tick()
    before = json.loads(statuses[-1].data)
    auth.authorized = False
    node.authorization(auth)
    node.tick()
    after = json.loads(statuses[-1].data)
    assert before["sim_time"] == after["sim_time"]
    assert before["mode"] == "TRACK" and after["mode"] == "BRAKE"
    assert before["decision_sequence"] < after["authorization"]["receipt_sequence"] < after["decision_sequence"]
    assert before["authorization"]["receipt_sequence"] < before["decision_sequence"]
    assert all(0 <= age < .5 for age in before["input_age_wall"].values())


def test_real_arbiter_callbacks_produce_auditable_track_revoke_and_expiry(arbiter):
    from traj_utils.msg import Bspline
    from xq_autonomy.sitl_supervisor_node import ros_stamp
    from audit_stage_a_authorization import (AUTH, STATUS, COMMAND, OUTPUT, SPLINE,
                                             audit_records, normalize)
    node, now, publications, statuses = arbiter
    records = []
    def record(topic, message):
        records.append(dict(topic=topic, message=normalize(topic, message), recorded_ns=len(records)))
    def tick():
        previous = len(publications)
        node.tick()
        record(STATUS, statuses[-1])
        for message in publications[previous:]:
            record(OUTPUT, message)
    command, auth = tracking_inputs(node, now)
    spline = Bspline()
    spline.traj_id = command.trajectory_id
    record(AUTH, auth)
    record(COMMAND, command)
    record(SPLINE, spline)
    tick()
    now[0] = 10.1
    node.odometry(odometry_at(0., now[0]))
    auth.authorized = False
    auth.header.stamp = auth.valid_until = ros_stamp(now[0])
    node.authorization(auth)
    record(AUTH, auth)
    tick()
    now[0] = 10.2
    node.odometry(odometry_at(0., now[0]))
    command.header.stamp = ros_stamp(now[0])
    command.trajectory_id = 8
    node.position_command(command)
    auth.header.stamp = ros_stamp(now[0])
    auth.request_id, auth.trajectory_id = 5, 8
    auth.valid_until = ros_stamp(10.4)
    auth.authorized = True
    node.authorization(auth)
    spline.traj_id = 8
    record(AUTH, auth)
    record(COMMAND, command)
    record(SPLINE, spline)
    tick()
    now[0] = 10.41
    node.odometry(odometry_at(0., now[0]))
    tick()
    report = audit_records(records, node.guard.session)
    assert report["status"] == "PASS", report
    assert report["checks"]["revocation_output_observed"]
    assert report["checks"]["expiration_output_observed"]
    assert not report["physical_stop_verified"]

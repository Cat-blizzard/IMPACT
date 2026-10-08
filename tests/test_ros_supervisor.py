"""ROS-node state transitions with simulated sensor messages, without Gazebo."""
import json
import time
from types import SimpleNamespace
import numpy as np
import pytest

pytest.importorskip("rclpy")
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from xq_sim_interfaces.msg import DirectionalIntegrity, LocalizationGeometry, PlannerCandidate
from xq_autonomy.p10_information_map_node import _xyz_cloud
from xq_autonomy.sitl_supervisor_node import (
    RECOVERY_INFORMATION_VISIBILITY_RADIUS_M, RECOVERY_SETTLE_TIMEOUT_S,
    SITLSupervisor, minimum_recovery_information_radius_m, ros_stamp,
)
from xq_autonomy.sitl_evaluator_node import box_clearance
import impact


@pytest.fixture
def node(tmp_path):
    rclpy.init(args=["--ros-args", "-p", "session_id:=unit-session", "-p",
        "calibration_file:="+str(impact.ROOT/impact.config()["calibration"]),
        "-p", "event_file:="+str(tmp_path/"events.jsonl")])
    instance=SITLSupervisor()
    instance.now_s=lambda:10.
    instance.enabled=True
    instance.stage_wall=time.monotonic()
    instance.last_plan_sim=10.
    instance.odom=Odometry()
    instance.odom.header=Header(frame_id="xq_lio_map",stamp=ros_stamp(10.))
    instance.odom.pose.pose.position.z=2.
    instance.odom.pose.pose.orientation.w=1.
    instance.cloud=_xyz_cloud(instance.odom.header,np.array([[0.,3.,2.],[0.,-3.,2.],[0.,0.,0.]]))
    instance.integrity=DirectionalIntegrity()
    instance.integrity.header=instance.odom.header
    instance.integrity.integrity_covariance=(np.eye(3)*1e-12).flatten().tolist()
    instance.geometry = LocalizationGeometry()
    instance.geometry.header = instance.odom.header
    instance.geometry.information_matrix = (np.eye(3) * 30.).flatten().tolist()
    instance.geometry.effective_points = 300
    instance.received={name:time.monotonic() for name in ("odom","cloud","integrity","geometry")}
    instance.test_pubs={name:[] for name in ("auth_pub","goal_pub","spline_pub","status_pub")}
    for name,values in instance.test_pubs.items():
        setattr(instance,name,SimpleNamespace(publish=values.append))
    yield instance
    instance.events.close()
    instance.destroy_node()
    rclpy.shutdown()


def candidate(node,request=None,identifier=1):
    msg=PlannerCandidate()
    msg.session_id=node.session
    msg.request_id=node.cycle.request if request is None else request
    msg.header=node.odom.header
    msg.trajectory.order=3
    msg.trajectory.traj_id=identifier
    msg.trajectory.start_time=ros_stamp(10.)
    msg.trajectory.knots=[0.]*4+[1.]*4
    msg.trajectory.pos_pts=[Point(x=x,y=0.,z=2.) for x in (0.,.1,.2,.3)]
    return msg


def test_final_optimized_trajectory_rejected_and_no_intents_hold(node):
    node.cycle.issue("mission")
    msg=candidate(node)
    msg.trajectory.pos_pts[-1].x=100.
    node.candidate(msg); node.tick()
    assert node.active is None
    assert not node.test_pubs["spline_pub"]
    assert not node.test_pubs["auth_pub"][-1].authorized
    assert not node.cycle.remaining
    assert not node.completed


def test_recovery_information_radius_can_include_a_safe_surface(node):
    assert node.recovery_information_visibility_radius == pytest.approx(
        RECOVERY_INFORMATION_VISIBILITY_RADIUS_M
    )
    assert (
        node.recovery_information_visibility_radius
        > minimum_recovery_information_radius_m()
    )


def test_active_trajectory_recertifies_current_covariance_and_revokes(node):
    node.cycle.issue("mission")
    node.candidate(candidate(node)); node.tick()
    assert node.active is not None
    node.integrity.integrity_covariance=(np.eye(3)*1e6).flatten().tolist()
    node.tick()
    assert node.active is None
    assert not node.test_pubs["auth_pub"][-1].authorized


def test_unobservable_axis_revokes_even_when_obstacle_margin_is_safe(node):
    node.cycle.issue("mission")
    node.candidate(candidate(node)); node.tick()
    assert node.active is not None
    node.geometry.information_matrix = np.diag([0.8, 60., 39.2]).flatten().tolist()
    node.tick()
    assert node.active is None
    assert node.recovery_required
    assert node.recovery_before_information["direction"] == pytest.approx([1., 0., 0.])
    assert not node.test_pubs["auth_pub"][-1].authorized
    node.fresh = lambda: True
    node.now_s = lambda: 10. + node.localization_recovery_timeout
    node.tick()
    assert node.execution_fault["reason"] == "LOCALIZATION_UNOBSERVABLE"
    assert node.cycle.phase == "FAIL_CLOSED"


def test_baseline_rank_loss_terminates_without_recovery(node):
    node.strategy = "baseline"
    node.geometry.information_matrix = np.diag([0.8, 60., 39.2]).flatten().tolist()
    node.tick()
    assert node.execution_fault["reason"] == "LOCALIZATION_UNOBSERVABLE"
    assert not node.active and not node.completed


def test_geometry_flicker_cannot_reset_loss_deadline(node):
    node.geometry.information_matrix = np.diag([.8, 60., 39.2]).flatten().tolist()
    node._check_localization_health(10.)
    node.geometry.information_matrix = (np.eye(3) * 30.).flatten().tolist()
    node.geometry.header.stamp = ros_stamp(10.5)
    node._check_localization_health(10.5)
    assert node.localization_loss_since == 10.
    node.geometry.information_matrix = np.diag([.8, 60., 39.2]).flatten().tolist()
    node._check_localization_health(10.6)
    node._check_localization_health(18.1)
    assert node.execution_fault["reason"] == "LOCALIZATION_UNOBSERVABLE"


def test_recovery_total_budget_fails_closed_even_if_geometry_is_healthy(node):
    node.strategy = "recovery"
    node.recovery_required = True
    node.recovery_started_sim = 10.0
    node.now_s = lambda: 55.0
    node._check_localization_health(55.0)
    assert node.execution_fault["reason"] == "RECOVERY_BUDGET_EXCEEDED"
    assert node.cycle.phase == "FAIL_CLOSED"


def test_repeated_recoveries_share_total_budget(node):
    node.recovery_spent_sim = 40.0
    node.now_s = lambda: 10.0
    node._require_information_recovery()
    node._check_localization_health(15.0)
    assert node.execution_fault["reason"] == "RECOVERY_BUDGET_EXCEEDED"
    assert node.execution_fault["elapsed_s"] == pytest.approx(45.0)


def test_geometry_restore_requires_new_sustained_observations(node):
    node.geometry.information_matrix = np.diag([.8, 60., 39.2]).flatten().tolist()
    node._check_localization_health(10.)
    node.geometry.information_matrix = (np.eye(3) * 30.).flatten().tolist()
    node.geometry.header.stamp = ros_stamp(10.5)
    node._check_localization_health(10.5)
    node._check_localization_health(12.)
    assert node.localization_loss_since == 10.
    node.geometry.header.stamp = ros_stamp(11.6)
    node._check_localization_health(12.)
    assert node.localization_loss_since is None


def test_anchor_requires_stronger_than_gate_geometry(node):
    node.geometry.information_matrix = (np.eye(3) * 30.).flatten().tolist()
    node._check_localization_health(10.)
    assert node.last_healthy_position.tolist() == [0., 0., 2.]
    node.odom.pose.pose.position.x = 1.
    node.geometry.information_matrix = np.diag([4., 60., 36.]).flatten().tolist()
    node._check_localization_health(10.1)
    assert node.localization_health["healthy"] is True
    assert node.last_healthy_position.tolist() == [0., 0., 2.]


def test_recovery_must_reach_anchor_and_observe_a_full_memory_window(node):
    node.cycle.issue("backtrack")
    node.cycle.result(node.cycle.request, True)
    node.target = np.array([.15, 0., 2.])
    node.tick()
    assert node.cycle.phase == "EXECUTING"
    node.target = np.array([.05, 0., 2.])
    node.tick()
    assert node.cycle.phase == "OBSERVING"
    assert node.observing_until == pytest.approx(10. + node.recovery_observation_window)


def test_recovery_endpoint_hold_is_recertified_and_bounded(node):
    node.cycle.issue("up_offset")
    msg = candidate(node)
    node.cycle.result(node.cycle.request, True)
    node.now_s = lambda: 11.2
    for name in ("odom", "cloud", "integrity", "geometry"):
        getattr(node, name).header.stamp = ros_stamp(11.2)
    result = node.certify(msg, tracking=True)
    assert result.accepted
    assert node.last_metrics["terminal_hold"] is True
    node.cloud = _xyz_cloud(node.odom.header, np.array([[.3, 0., 2.]]))
    assert not node.certify(msg, tracking=True).accepted
    node.now_s = lambda: 19.1
    with pytest.raises(ValueError):
        node.certify(msg, tracking=True)


def test_mission_expiry_does_not_receive_recovery_endpoint_hold(node):
    node.cycle.issue("mission")
    msg = candidate(node)
    node.now_s = lambda: 11.2
    with pytest.raises(ValueError):
        node.certify(msg, tracking=True)


def test_unapproved_goal_position_is_not_completion(node):
    node.odom.pose.pose.position.x = node.goal[0]
    node.tick()
    assert not node.completed


def test_elevated_recovery_continuation_only_requests_a_bounded_target(node):
    node._retain_recovered_height(np.array([4.0, 0.1, 2.5]))
    assert node.recovery_release_waypoint == pytest.approx([6.0, 0.075, 2.5], abs=0.001)
    assert not node.active
    assert not node.test_pubs["spline_pub"]
    assert not node.test_pubs["auth_pub"]


def test_nominal_height_recovery_does_not_create_an_elevated_continuation(node):
    node._retain_recovered_height(np.array([4.0, 0.1, 2.0]))
    assert node.recovery_release_waypoint is None


def test_late_candidate_cannot_replace_current_recovery_request(node):
    old=node.cycle.issue("left")
    node.cycle.issue("right")
    node.candidate(candidate(node,request=old))
    assert node.pending is None


def test_active_margin_loss_cannot_resume_without_information_recovery(node):
    node.strategy = "recovery"
    node.cycle.issue("mission")
    node.candidate(candidate(node)); node.tick()
    assert node.active is not None
    node.integrity.integrity_covariance = (np.eye(3) * 1e6).flatten().tolist()
    node.tick()
    assert node.active is None
    assert node.recovery_required
    node.integrity.integrity_covariance = (np.eye(3) * 1e-12).flatten().tolist()
    node.cycle.issue("mission")
    published = len(node.test_pubs["spline_pub"])
    node.candidate(candidate(node, identifier=2)); node.tick()
    assert node.active is None
    assert len(node.test_pubs["spline_pub"]) == published
    assert not node.test_pubs["auth_pub"][-1].authorized


def test_mission_rejection_latches_recovery_during_cooldown(node):
    node.strategy = "recovery"
    node.last_recovery_sim = 10.
    node.cycle.issue("mission")
    node.integrity.integrity_covariance = (np.eye(3) * 1e6).flatten().tolist()
    node.candidate(candidate(node)); node.tick()
    assert node.recovery_required
    assert not node.cycle.remaining


def test_observation_resumes_planning_but_needs_online_certification(node):
    node.cycle.issue("left")
    node.recovery_step_request_id = node.cycle.request
    node.cycle.arrived(9.)
    node.observing_until=9.5
    node.recovery_before=dict(AL=10.,PL=0.,margin=10.)
    node.recovery_required = True
    node.integrity.weak_direction_map = [1., 0., 0.]
    node.integrity.information_matrix = (np.eye(3) * 30.).flatten().tolist()
    node.recovery_before_information = dict(direction=[1., 0., 0.], variance_m2=1e-4,
        geometric_information=20., raw_geometric_information=10., raw_stamp_s=9.,
        protection_m=0.1, stamp_s=9.)
    node.tick()
    assert node.cycle.intent == "mission" and node.cycle.phase == "PLANNING"
    assert node.test_pubs["goal_pub"] and not node.completed
    node.candidate(candidate(node)); node.tick()
    assert node.active is not None
    rows=[json.loads(line) for line in impact.Path(node.events.name).read_text().splitlines()]
    event=next(r for r in rows if r["event"] == "RECOVERY_CONFIRMED")
    assert event["delta_margin"] == pytest.approx(event["delta_AL"]-event["delta_PL"])
    assert not node.cycle.remaining
    assert event["information_improved"] is True
    assert event["information_after"]["geometric_information"] > 20.
    assert event["delta_margin"] < 0.0


def test_information_gain_allows_release_without_historical_margin_gain(node):
    node.cycle.issue("left")
    node.recovery_step_request_id = node.cycle.request
    node.cycle.arrived(9.)
    node.observing_until = 9.5
    node.recovery_before = dict(AL=10., PL=0., margin=10.)
    node.recovery_required = True
    node.integrity.weak_direction_map = [1., 0., 0.]
    node.integrity.information_matrix = (np.eye(3) * 30.).flatten().tolist()
    node.recovery_before_information = dict(direction=[1., 0., 0.], variance_m2=1e-4,
        geometric_information=20., raw_geometric_information=10., raw_stamp_s=9.,
        protection_m=0.1, stamp_s=9.)
    node.tick()
    node.candidate(candidate(node)); node.tick()
    rows = [json.loads(line) for line in impact.Path(node.events.name).read_text().splitlines()]
    event = next(row for row in rows if row["event"] == "RECOVERY_CONFIRMED")
    assert event["information_improved"] is True
    assert event["delta_margin"] < 0.0
    assert node.active is not None
    assert node.test_pubs["spline_pub"]
    assert node.test_pubs["auth_pub"][-1].authorized


@pytest.mark.parametrize("evidence", ["missing", "clearance_only", "covariance_only", "stale", "no_step",
                                     "memory_only", "raw_stale"])
def test_mission_cannot_resume_without_new_geometric_information(node, evidence):
    node.recovery_before = dict(AL=0.1, PL=0.2, margin=-0.1)
    node.recovery_required = True
    node.recovery_before_information = dict(direction=[1., 0., 0.], variance_m2=1e-4,
        geometric_information=20., raw_geometric_information=10., raw_stamp_s=9.,
        protection_m=0.1, stamp_s=9.)
    node.integrity.weak_direction_map = [1., 0., 0.]
    node.integrity.information_matrix = (np.eye(3) * 30.).flatten().tolist()
    node.cycle.issue("left")
    node.recovery_step_request_id = node.cycle.request if evidence != "no_step" else None
    node.cycle.arrived(9.)
    node.observing_until = 9.5
    if evidence == "missing":
        node.recovery_before_information = None
    elif evidence == "clearance_only":
        node.integrity.integrity_covariance = (np.eye(3) * 2e-4).flatten().tolist()
    elif evidence == "covariance_only":
        node.integrity.information_matrix = (np.eye(3) * 10.).flatten().tolist()
    elif evidence == "memory_only":
        node.geometry.information_matrix = (np.eye(3) * 5.).flatten().tolist()
    elif evidence == "raw_stale":
        node.recovery_before_information["raw_stamp_s"] = 10.
    else:
        node.recovery_before_information["stamp_s"] = 10.
    node.tick()
    node.candidate(candidate(node)); node.tick()
    assert node.active is None
    assert node.recovery_required
    assert not node.test_pubs["spline_pub"]
    assert not node.test_pubs["auth_pub"][-1].authorized
    # Retry without another observation must retain the recovery latch.
    node.cycle.issue("mission")
    node.candidate(candidate(node, identifier=2)); node.tick()
    assert node.active is None
    assert not node.test_pubs["spline_pub"]


def test_unconfirmed_observation_continues_remaining_recovery_intents(node):
    remaining = [("backtrack", np.array([-.5, 0., 2.]), 1.0)]
    node.cycle.remaining = list(remaining)
    node.recovery_before = dict(AL=1., PL=2., margin=-1.)
    node.recovery_required = True
    node.recovery_observed = True
    node.cycle.issue("mission")
    node.integrity.integrity_covariance = (np.eye(3) * 1e6).flatten().tolist()
    node.candidate(candidate(node))
    node.tick()
    assert node.active is None
    assert node.cycle.remaining == remaining
    assert not node.recovery_observed
    rows = [json.loads(line) for line in impact.Path(node.events.name).read_text().splitlines()]
    event = next(row for row in rows if row["event"] == "RECOVERY_NOT_CONFIRMED")
    assert event["remaining_intents"] == ["backtrack"]


def test_expired_recovery_spline_waits_for_vehicle_to_settle(node):
    node.now_s = lambda: 10.0
    node.fresh = lambda: True
    request = node.cycle.issue("up_offset")
    assert node.cycle.result(request, True)
    node.target = np.array([0.3, 0.0, 2.0])
    node.last_plan_sim = 10.0 - 0.5 * RECOVERY_SETTLE_TIMEOUT_S
    node.active = None
    node.cycle.remaining = [("down_offset", np.array([0.0, 0.0, 1.7]), 1.0)]

    node.tick()
    assert node.cycle.phase == "EXECUTING"
    assert not node.test_pubs["goal_pub"]
    assert len(node.cycle.remaining) == 1

    node.odom.pose.pose.position.x = 0.3
    node.tick()
    assert node.cycle.phase == "OBSERVING"
    assert len(node.cycle.remaining) == 1
    rows = [json.loads(line) for line in impact.Path(node.events.name).read_text().splitlines()]
    assert any(row["event"] == "RECOVERY_STEP_DONE" for row in rows)


def test_unreached_recovery_step_times_out_before_next_intent(node):
    node.now_s = lambda: 10.0
    node.fresh = lambda: True
    request = node.cycle.issue("up_offset")
    assert node.cycle.result(request, True)
    node.target = np.array([0.5, 0.0, 2.0])
    node.last_plan_sim = 10.0 - RECOVERY_SETTLE_TIMEOUT_S - 0.1
    node.active = None
    node.cycle.remaining = [("down_offset", np.array([0.0, 0.0, 1.7]), 1.0)]

    node.tick()
    assert node.cycle.intent == "down_offset"
    assert node.cycle.phase == "PLANNING"
    assert len(node.test_pubs["goal_pub"]) == 1
    rows = [json.loads(line) for line in impact.Path(node.events.name).read_text().splitlines()]
    timeout = next(row for row in rows if row["event"] == "RECOVERY_STEP_TIMEOUT")
    assert timeout["intent"] == "up_offset"


def test_clock_reset_invalidates_pending_and_active(node):
    node.cycle.issue("mission")
    node.candidate(candidate(node)); node.tick()
    assert node.active is not None
    node.now_s=lambda:1.
    node.tick()
    assert node.reset and node.active is None and node.pending is None and node.odom is None


def test_independent_clearance_uses_oriented_geometry():
    box=dict(center=[0.,0.,0.],size=[4.,1.,2.],yaw=np.pi/2)
    assert box_clearance([0.,2.,0.],[box],radius=.2) == pytest.approx(-.2)
    assert box_clearance([2.,0.,0.],[box],radius=.2) == pytest.approx(1.3)


def test_mission_timeout_remains_failure_and_uses_wall_timer(tmp_path, monkeypatch):
    from xq_autonomy.sitl_mission_node import SITLMission
    from rclpy.clock import ClockType
    from mavros_msgs.msg import State
    rclpy.init(args=["--ros-args","-p","session_id:=mission-test","-p","result_file:="+str(tmp_path/"mission.json")])
    mission=SITLMission()
    try:
        assert all(t.clock.clock_type == ClockType.STEADY_TIME for t in mission.timers)
        # Isolate the timeout path with a healthy, armed FCU. The review mission
        # tests separately exercise the real inherited health checks.
        mission._state_cb(State(connected=True, armed=True, guided=True, mode="GUIDED"))
        monkeypatch.setattr(mission, "_observe_health", lambda **_: (True, {"reasons": []}))
        mission.phase="ACTIVE"
        mission.phase_started=time.monotonic()
        mission.task_started=mission.get_clock().now().nanoseconds/1e9-181.
        mission._tick()
        assert mission.phase == "LAND" and mission.task_reason == "TASK_TIMEOUT"
        assert not mission.task_success
        mission._finish("FAIL","LANDING_NOT_CONFIRMED")
        result=json.loads((tmp_path/"mission.json").read_text())
        assert not result["task_success"] and not result["termination_confirmed"]
    finally:
        mission.destroy_node(); rclpy.shutdown()

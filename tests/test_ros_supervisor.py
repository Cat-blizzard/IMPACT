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
from xq_sim_interfaces.msg import DirectionalIntegrity, PlannerCandidate
from xq_autonomy.p10_information_map_node import _xyz_cloud
from xq_autonomy.sitl_supervisor_node import (
    RECOVERY_SETTLE_TIMEOUT_S, SITLSupervisor, ros_stamp,
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
    instance.received={name:time.monotonic() for name in ("odom","cloud","integrity")}
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


def test_active_trajectory_recertifies_current_covariance_and_revokes(node):
    node.cycle.issue("mission")
    node.candidate(candidate(node)); node.tick()
    assert node.active is not None
    node.integrity.integrity_covariance=(np.eye(3)*1e6).flatten().tolist()
    node.tick()
    assert node.active is None
    assert not node.test_pubs["auth_pub"][-1].authorized


def test_late_candidate_cannot_replace_current_recovery_request(node):
    old=node.cycle.issue("left")
    node.cycle.issue("right")
    node.candidate(candidate(node,request=old))
    assert node.pending is None


def test_observation_resumes_planning_but_needs_online_certification(node):
    node.cycle.issue("left")
    node.cycle.arrived(9.)
    node.observing_until=9.5
    node.recovery_before=dict(AL=1.,PL=2.,margin=-1.)
    node.tick()
    assert node.cycle.intent == "mission" and node.cycle.phase == "PLANNING"
    assert node.test_pubs["goal_pub"] and not node.completed
    node.candidate(candidate(node)); node.tick()
    assert node.active is not None
    rows=[json.loads(line) for line in impact.Path(node.events.name).read_text().splitlines()]
    event=next(r for r in rows if r["event"] == "RECOVERY_CONFIRMED")
    assert event["delta_margin"] == pytest.approx(event["delta_AL"]-event["delta_PL"])
    assert not node.cycle.remaining


def test_unconfirmed_observation_continues_remaining_recovery_intents(node):
    remaining = [("backtrack", np.array([-.5, 0., 2.]), 1.0)]
    node.cycle.remaining = list(remaining)
    node.recovery_before = dict(AL=1., PL=2., margin=-1.)
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

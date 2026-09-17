"""Actual DDS/message/TF/arbiter smoke test; no Gazebo or vehicle is launched."""
import argparse
import hashlib
import inspect
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from xq_sim_interfaces.msg import TrajectoryAuthorization
from xq_autonomy.sitl_arbiter_node import SITLArbiter
from xq_autonomy.sitl_supervisor_node import ros_stamp


def file_hash(path):
    digest=hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda:handle.read(1048576),b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output")
    parser.add_argument("--build-manifest")
    args = parser.parse_args()
    rclpy.init(args=["--ros-args","-p","session_id:=contract-test"])
    arbiter=SITLArbiter()
    driver=Node("impact_contract_driver")
    executor=SingleThreadedExecutor()
    executor.add_node(arbiter); executor.add_node(driver)
    odom=driver.create_publisher(Odometry,"/localization/odom",20)
    command=driver.create_publisher(PositionCommand,"/impact/position_cmd",20)
    auth=driver.create_publisher(TrajectoryAuthorization,"/impact/authorization",20)
    stage=driver.create_publisher(String,"/impact/mission_stage",20)
    results=[]
    statuses=[]
    driver.create_subscription(PositionTarget,"/uav1/mavros/setpoint_raw/local",
                               lambda m:results.append(m),20)
    driver.create_subscription(String,"/impact/arbiter_status",
                               lambda m:statuses.append(json.loads(m.data)),20)
    broadcaster=StaticTransformBroadcaster(driver)
    transform=TransformStamped()
    transform.header.frame_id="map"; transform.child_frame_id="xq_lio_map"
    transform.transform.rotation.w=1.
    transform.header.stamp=driver.get_clock().now().to_msg()
    broadcaster.sendTransform(transform)
    def spin(duration, authorized=False, traj=7, frame="xq_lio_map", session="contract-test"):
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:
            now=driver.get_clock().now().nanoseconds/1e9
            m=Odometry(); m.header.frame_id="xq_lio_map"; m.header.stamp=ros_stamp(now)
            m.pose.pose.position.z=2.; m.pose.pose.orientation.w=1.
            odom.publish(m)
            stage.publish(String(data=json.dumps(dict(session_id="contract-test",phase="ACTIVE"))))
            a=TrajectoryAuthorization(); a.session_id=session; a.request_id=1; a.trajectory_id=7
            a.header.frame_id="xq_lio_map"
            a.header.stamp=ros_stamp(now); a.valid_until=ros_stamp(now+.25)
            a.authorized=authorized; auth.publish(a)
            c=PositionCommand(); c.trajectory_id=traj; c.header.frame_id=frame; c.header.stamp=ros_stamp(now)
            c.position.x=.2; c.position.z=2.; c.velocity.x=.3; command.publish(c)
            # Drain input/output callbacks each cycle; do not manufacture a growing
            # executor backlog that still contains the previous test phase.
            for _ in range(16): executor.spin_once(timeout_sec=0.001)
            time.sleep(.005)
    def spin_expired_lease(duration=.65):
        now=driver.get_clock().now().nanoseconds/1e9
        expiry=now+.25
        a=TrajectoryAuthorization(); a.session_id="contract-test"; a.request_id=2; a.trajectory_id=8
        a.header.frame_id="xq_lio_map"; a.header.stamp=ros_stamp(now)
        a.valid_until=ros_stamp(expiry); a.authorized=True; auth.publish(a)
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:
            now=driver.get_clock().now().nanoseconds/1e9
            m=Odometry(); m.header.frame_id="xq_lio_map"; m.header.stamp=ros_stamp(now)
            m.pose.pose.position.z=2.; m.pose.pose.orientation.w=1.; odom.publish(m)
            stage.publish(String(data=json.dumps(dict(session_id="contract-test",phase="ACTIVE"))))
            c=PositionCommand(); c.trajectory_id=8; c.header.frame_id="xq_lio_map"
            c.header.stamp=ros_stamp(now); c.position.x=.2; c.position.z=2.
            c.velocity.x=.3; command.publish(c)
            for _ in range(16): executor.spin_once(timeout_sec=0.001)
            time.sleep(.005)
        return expiry
    checks={}
    observations={}
    failure=None
    def require(name, condition):
        checks[name]=bool(condition)
        assert condition, name.replace("_", " ")
    try:
        spin(1.0)
        require("uncertified_command_blocked", results and max(m.position.x for m in results)<.01)
        results.clear(); spin(.6,True)
        require("certified_feedforward_preserved", any(
            m.position.x>.19 and abs(m.velocity.x-.3)<1e-6
            and m.type_mask == PositionTarget.IGNORE_YAW_RATE for m in results))
        results.clear(); spin(.6,True,traj=8)
        require("wrong_trajectory_blocked", all(m.position.x<.01 for m in results[3:]))
        results.clear(); spin(.6,True,frame="world")
        require("wrong_frame_blocked", all(m.position.x<.01 for m in results[3:]))
        results.clear(); spin(.6,False)
        require("revoked_trajectory_blocked", all(m.position.x<.01 for m in results[3:]))
        results.clear(); spin(.6,True,session="old-session")
        require("old_session_blocked", all(m.position.x<.01 for m in results[3:]))
        results.clear(); statuses.clear()
        expiry=spin_expired_lease()
        before=[item for item in statuses if item.get("sim_time", expiry)>=expiry-.25]
        after=[item for item in statuses if item.get("sim_time", 0.)>=expiry]
        require("lease_tracked_before_expiry", any(item.get("mode")=="TRACK" for item in before))
        require("expired_lease_never_tracks", bool(after)
            and not any(item.get("mode")=="TRACK" for item in after))
        require("expired_lease_brakes", any(item.get("mode")=="BRAKE"
            and item.get("authorized") is False
            and item.get("execution",{}).get("emitted") is True for item in after))
        observations.update(expiry_sim_time=expiry, status_samples=len(statuses),
                            post_expiry_samples=len(after), output_samples=len(results))
        print("PASS: real ROS messages, TF, matching IDs, frame rejection, revocation, expiry and old-session isolation")
    except Exception as error:
        failure=f"{type(error).__name__}: {error}"
        raise
    finally:
        if args.output:
            output=Path(args.output).resolve()
            output.parent.mkdir(parents=True,exist_ok=True)
            module=Path(inspect.getfile(SITLArbiter)).resolve()
            manifest=Path(args.build_manifest).resolve() if args.build_manifest else None
            build={}
            if manifest and manifest.is_file():
                data=json.loads(manifest.read_text())
                install=Path(data.get("install_root", "")).resolve()
                try:
                    relative=module.relative_to(install).as_posix()
                except ValueError:
                    relative=None
                expected=data.get("installed_files", {}).get(relative) if relative else None
                checks["loaded_arbiter_matches_build_manifest"]=(
                    expected is not None and expected == file_hash(module))
                if not checks["loaded_arbiter_matches_build_manifest"] and failure is None:
                    failure="RuntimeError: loaded arbiter does not match the build manifest"
                build=dict(path=str(manifest),sha256=file_hash(manifest),
                           source_sha256=data.get("source_sha256"),git=data.get("git"),
                           install_root=str(install),module_relative_path=relative,
                           expected_module_sha256=expected)
            output.write_text(json.dumps(dict(schema_version=1,
                gate="STAGE_A_AUTHORIZATION_RUNTIME_CONTRACT",
                status="PASS" if failure is None and checks and all(checks.values()) else "FAIL",
                checks=checks, observations=observations, failure=failure,
                build=build,
                loaded_arbiter_module=dict(path=str(module),sha256=file_hash(module)),
                physical_stop_verified=False,
                scope="Isolated real DDS/TF/arbiter behavior; no Gazebo, FCU, ARM or vehicle motion."),
                indent=2)+"\n")
        executor.shutdown(); driver.destroy_node(); arbiter.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()

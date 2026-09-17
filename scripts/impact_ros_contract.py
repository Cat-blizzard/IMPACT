"""Actual DDS/message/TF/arbiter smoke test; no Gazebo or vehicle is launched."""
import json
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


def main():
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
    driver.create_subscription(PositionTarget,"/uav1/mavros/setpoint_raw/local",
                               lambda m:results.append(m),20)
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
    try:
        spin(1.0)
        assert results and max(m.position.x for m in results)<.01, "uncertified command leaked"
        results.clear(); spin(.6,True)
        assert any(m.position.x>.19 and abs(m.velocity.x-.3)<1e-6
                   and m.type_mask == PositionTarget.IGNORE_YAW_RATE for m in results), (
                       "certified position/velocity command did not traverse real DDS/TF")
        results.clear(); spin(.6,True,traj=8)
        assert all(m.position.x<.01 for m in results[3:]), "wrong trajectory ID leaked"
        results.clear(); spin(.6,True,frame="world")
        assert all(m.position.x<.01 for m in results[3:]), "wrong frame leaked"
        results.clear(); spin(.6,False)
        assert all(m.position.x<.01 for m in results[3:]), "revoked trajectory leaked"
        results.clear(); spin(.6,True,session="old-session")
        assert all(m.position.x<.01 for m in results[3:]), "old session leaked"
        print("PASS: real ROS messages, TF, matching IDs, frame rejection, revocation and old-session isolation")
    finally:
        executor.shutdown(); driver.destroy_node(); arbiter.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()

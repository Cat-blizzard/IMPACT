"""Only P16 publisher of MAVROS position targets; continuously brakes on revocation."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from .sitl_integrity import Authorization, ExecutionGuard, brake_samples
from .sitl_supervisor_node import stamp_s, xyz, main_node
from xq_sim_interfaces.msg import TrajectoryAuthorization


class SITLArbiter(Node):
    def __init__(self):
        super().__init__("impact_arbiter")
        self.declare_parameter("session_id", "")
        self.guard = ExecutionGuard(self.get_parameter("session_id").value)
        self.odom = self.command = self.hold = None
        self.odom_wall = self.command_wall = self.stage_wall = self.auth_wall = 0.
        self.phase = "WAIT_FCU"
        self.brake_state = None
        self.reset_cleanup_generation = 0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PoseStamped, "/uav1/mavros/setpoint_position/local", 20)
        self.status_pub = self.create_publisher(String, "/impact/arbiter_status", 10)
        self.create_subscription(Odometry, "/localization/odom", self.odometry, qos_profile_sensor_data)
        self.create_subscription(PositionCommand, "/impact/position_cmd", self.position_command, 20)
        self.create_subscription(PoseStamped, "/impact/mission_hold", self.mission_hold, 10)
        self.create_subscription(TrajectoryAuthorization, "/impact/authorization", self.authorization, 20)
        self.create_subscription(String, "/impact/mission_stage", self.stage, 10)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.tick, clock=self.wall_clock)

    def odometry(self, msg):
        if msg.header.frame_id != "xq_lio_map" or not np.isfinite(np.r_[xyz(msg.pose.pose.position), xyz(msg.twist.twist.linear)]).all():
            return
        self.odom, self.odom_wall = msg, time.monotonic()

    def position_command(self, msg):
        self.command, self.command_wall = msg, time.monotonic()

    def mission_hold(self, msg):
        self.hold = msg

    def stage(self, msg):
        try:
            data = json.loads(msg.data)
            if data["session_id"] == self.guard.session:
                self.phase, self.stage_wall = data["phase"], time.monotonic()
        except (ValueError, KeyError):
            pass

    def authorization(self, msg):
        if msg.header.frame_id != "xq_lio_map":
            return
        auth = Authorization(msg.session_id, msg.request_id, msg.trajectory_id,
            stamp_s(msg.header.stamp), stamp_s(msg.valid_until), msg.authorized)
        if self.guard.update(auth, self.get_clock().now().nanoseconds / 1e9):
            self.auth_wall = time.monotonic()

    def transform(self, point):
        # Actual TF application; never relabel coordinates as a transform.
        transform = self.buffer.lookup_transform("map", "xq_lio_map", rclpy.time.Time())
        q = transform.transform.rotation
        u, w = np.array((q.x, q.y, q.z)), float(q.w)
        norm = float(np.dot(u, u) + w*w)
        if abs(norm-1) > 1e-5:
            raise ValueError("invalid frame quaternion")
        # This controller uses upright ENU frames. A tilted map requires a full
        # attitude/control adapter and must not silently pass as a yaw offset.
        if abs(q.x) > 1e-5 or abs(q.y) > 1e-5:
            raise ValueError("map transform is not upright ENU")
        self.transform_yaw = 2 * math.atan2(q.z, q.w)
        return point + 2*np.cross(u, np.cross(u, point)+w*point) + xyz(transform.transform.translation)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self.guard.clock(now)
        if self.guard.reset_latched and self.reset_cleanup_generation != self.guard.reset_generation:
            # update() in the authorization callback can detect the reset first.
            # Clean each clock discontinuity regardless of its callback; only odometry
            # received after this cleanup may establish a new braking endpoint.
            self.command = self.odom = self.hold = self.brake_state = None
            self.odom_wall = self.command_wall = self.auth_wall = 0.
            self.reset_cleanup_generation = self.guard.reset_generation
        wall = time.monotonic()
        if not self.odom:
            return
        measured = xyz(self.odom.pose.pose.position)
        fresh = (wall-self.odom_wall < 0.5 and 0 <= now-stamp_s(self.odom.header.stamp) <= 0.5)
        phase_fresh = wall-self.stage_wall < 0.5
        mode, target, yaw = "INACTIVE", None, 0.
        # Takeoff and landing belong to FCU submodes: no competing position targets.
        if self.phase in ("ACTIVE", "HOVER"):
            cmd = self.command
            if (self.phase == "ACTIVE" and phase_fresh and not self.guard.reset_latched
                and fresh and cmd and wall-self.auth_wall < 0.5
                and self.guard.allows(cmd.trajectory_id, stamp_s(cmd.header.stamp), now,
                    cmd.header.frame_id, np.r_[xyz(cmd.position), cmd.yaw], wall-self.command_wall)):
                target, yaw, mode = xyz(cmd.position), cmd.yaw, "TRACK"
                if np.linalg.norm(target-measured) > 0.35:
                    target, mode = None, "BRAKE"
                else:
                    self.brake_state = None
            if target is None:
                mode = "BRAKE" if fresh else "STALE_ODOM_HOLD"
                if self.brake_state is None and fresh:
                    self.brake_state = (measured, xyz(self.odom.twist.twist.linear), now)
                if self.brake_state:
                    p, v, start = self.brake_state
                    target = brake_samples(p, v, max(0., now-start))
        if not phase_fresh or self.guard.reset_latched:
            # The mission process / FCU failsafe owns termination after loss of authority.
            self.guard.current = None
        if target is not None:
            try:
                target = self.transform(target)
                yaw += self.transform_yaw
                message = PoseStamped()
                message.header.frame_id = "map"
                # MAVROS uses wall time, while authorization used original simulation time.
                message.header.stamp = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME).now().to_msg()
                message.pose.position.x, message.pose.position.y, message.pose.position.z = target.tolist()
                message.pose.orientation.z = math.sin(yaw/2)
                message.pose.orientation.w = math.cos(yaw/2)
                self.pub.publish(message)
            except (TransformException, ValueError):
                mode = "TF_FAILURE"
        self.status_pub.publish(String(data=json.dumps(dict(session_id=self.guard.session,
            sim_time=now, mode=mode, fresh_odom=fresh, reset=self.guard.reset_latched,
            authorized=bool(self.guard.current), ground_truth_subscribed=False))))


def main(args=None):
    main_node(SITLArbiter, args)

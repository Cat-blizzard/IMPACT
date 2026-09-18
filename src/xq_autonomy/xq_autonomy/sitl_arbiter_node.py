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
from mavros_msgs.msg import PositionTarget
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
        self.last_authorization_receipt = None
        self.event_sequence = 0
        self.brake_state = None
        self.reset_cleanup_generation = 0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PositionTarget, "/uav1/mavros/setpoint_raw/local", 20)
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
        now = self.get_clock().now().nanoseconds / 1e9
        if self.guard.update(auth, now):
            self.event_sequence += 1
            self.auth_wall = time.monotonic()
            self.last_authorization_receipt = dict(session_id=auth.session,
                request_id=auth.request, trajectory_id=auth.trajectory,
                issued=auth.issued, expires=auth.expires, accepted=auth.accepted,
                received_sim_time=now, receipt_sequence=self.event_sequence)

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

    def rotate(self, vector):
        transform = self.buffer.lookup_transform("map", "xq_lio_map", rclpy.time.Time())
        q = transform.transform.rotation
        u, w = np.array((q.x, q.y, q.z)), float(q.w)
        norm = float(np.dot(u, u) + w*w)
        if abs(norm-1) > 1e-5 or abs(q.x) > 1e-5 or abs(q.y) > 1e-5:
            raise ValueError("map transform is not upright ENU")
        return vector + 2*np.cross(u, np.cross(u, vector)+w*vector)

    def tick(self):
        self.event_sequence += 1
        decision_sequence = self.event_sequence
        now = self.get_clock().now().nanoseconds / 1e9
        self.guard.clock(now)
        if self.guard.reset_latched and self.reset_cleanup_generation != self.guard.reset_generation:
            # update() in the authorization callback can detect the reset first.
            # Clean each clock discontinuity regardless of its callback; only odometry
            # received after this cleanup may establish a new braking endpoint.
            self.command = self.odom = self.hold = self.brake_state = None
            self.odom_wall = self.command_wall = self.auth_wall = 0.
            self.reset_cleanup_generation = self.guard.reset_generation
            self.last_authorization_receipt = None
        wall = time.monotonic()
        if not self.odom:
            return
        measured = xyz(self.odom.pose.pose.position)
        fresh = (wall-self.odom_wall < 0.5 and 0 <= now-stamp_s(self.odom.header.stamp) <= 0.5)
        phase_fresh = wall-self.stage_wall < 0.5
        mode, target, velocity, acceleration, yaw, yaw_rate = (
            "INACTIVE", None, np.zeros(3), np.zeros(3), 0., 0.)
        lease = self.guard.current
        authorized = bool(lease and lease.accepted
            and lease.issued <= now < lease.expires
            and 0 <= wall-self.auth_wall < 0.5 and not self.guard.reset_latched
            and self.phase == "ACTIVE" and phase_fresh and fresh)
        execution = dict(emitted=False)
        rejection_reason = None
        # Takeoff and landing belong to FCU submodes: no competing position targets.
        if self.phase in ("ACTIVE", "HOVER"):
            cmd = self.command
            if (self.phase == "ACTIVE" and phase_fresh and not self.guard.reset_latched
                and fresh and cmd and wall-self.auth_wall < 0.5
                and (rejection_reason := self.guard.allow_reason(
                    cmd.trajectory_id, stamp_s(cmd.header.stamp), now,
                    cmd.header.frame_id,
                    np.r_[xyz(cmd.position), xyz(cmd.velocity), xyz(cmd.acceleration),
                          cmd.yaw, cmd.yaw_dot], wall-self.command_wall)) is None):
                target, velocity, acceleration = (
                    xyz(cmd.position), xyz(cmd.velocity), xyz(cmd.acceleration))
                yaw, yaw_rate, mode = cmd.yaw, cmd.yaw_dot, "TRACK"
                if np.linalg.norm(target-measured) > 0.35:
                    target, mode = None, "BRAKE"
                    rejection_reason = "TRACKING_DISTANCE"
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
                velocity = self.rotate(velocity)
                acceleration = self.rotate(acceleration)
                yaw += self.transform_yaw
                message = PositionTarget()
                message.header.frame_id = "map"
                # MAVROS uses wall time, while authorization used original simulation time.
                message.header.stamp = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME).now().to_msg()
                message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                message.type_mask = PositionTarget.IGNORE_YAW_RATE
                if mode != "TRACK":
                    message.type_mask |= (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY
                        | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX
                        | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ)
                message.position.x, message.position.y, message.position.z = target.tolist()
                message.velocity.x, message.velocity.y, message.velocity.z = velocity.tolist()
                (message.acceleration_or_force.x, message.acceleration_or_force.y,
                 message.acceleration_or_force.z) = acceleration.tolist()
                message.yaw = float(yaw)
                message.yaw_rate = float(yaw_rate)
                self.pub.publish(message)
                # Bind this decision to the exact final MAVROS publication.
                # Receipt timing and command provenance are recorded separately
                # from the ROS bag writer's cross-topic arrival order.
                tracking = mode == "TRACK"
                execution = dict(emitted=True,
                    setpoint_stamp_ns=int(message.header.stamp.sec)*1000000000
                        + int(message.header.stamp.nanosec),
                    frame_id=message.header.frame_id, coordinate_frame=int(message.coordinate_frame),
                    type_mask=int(message.type_mask), position=target.tolist(),
                    velocity=velocity.tolist(), acceleration=acceleration.tolist(),
                    yaw=float(yaw), yaw_rate=float(yaw_rate),
                    trajectory_id=int(cmd.trajectory_id) if tracking else None,
                    request_id=lease.request if tracking else None,
                    command_stamp=stamp_s(cmd.header.stamp) if tracking else None,
                    command_position=xyz(cmd.position).tolist() if tracking else None,
                    command_velocity=xyz(cmd.velocity).tolist() if tracking else None,
                    command_acceleration=xyz(cmd.acceleration).tolist() if tracking else None,
                    command_yaw=float(cmd.yaw) if tracking else None,
                    command_yaw_rate=float(cmd.yaw_dot) if tracking else None,
                    authorization_issued=lease.issued if tracking else None,
                    authorization_expires=lease.expires if tracking else None)
            except (TransformException, ValueError):
                mode = "TF_FAILURE"
        self.status_pub.publish(String(data=json.dumps(dict(schema_version=3,
            session_id=self.guard.session, sim_time=now, mode=mode,
            decision_sequence=decision_sequence,
            input_age_wall=dict(authorization=wall-self.auth_wall, command=wall-self.command_wall,
                stage=wall-self.stage_wall, odom=wall-self.odom_wall),
            phase=self.phase, phase_fresh=phase_fresh, fresh_odom=fresh,
            reset=self.guard.reset_latched, authorized=authorized,
            authorization=self.last_authorization_receipt, execution=execution,
            rejection_reason=rejection_reason,
            measured_position=measured.tolist(),
            measured_velocity=xyz(self.odom.twist.twist.linear).tolist(),
            odom_stamp=stamp_s(self.odom.header.stamp), ground_truth_subscribed=False))))


def main(args=None):
    main_node(SITLArbiter, args)

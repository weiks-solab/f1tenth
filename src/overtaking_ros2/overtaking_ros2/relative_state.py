"""Project measured vehicle states for on-car observation."""

import math
from uuid import uuid4

from f1tenth_overtaking_core.geometry import FrenetTracker, TrackingOptions, load_geometry
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from overtaking_msgs.msg import RelativeState, SteeringState, VehicleState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def read_odometry(message, frame):
    if message.header.frame_id != 'map' or message.child_frame_id != frame:
        raise ValueError('frame_mismatch')
    pose = message.pose.pose
    x, y = pose.position.x, pose.position.y
    speed = message.twist.twist.linear.x
    q = pose.orientation
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in (x, y, speed, norm)) or norm < 1e-9:
        raise ValueError('nonfinite_or_invalid_pose')
    qx, qy, qz, qw = (v / norm for v in (q.x, q.y, q.z, q.w))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return x, y, yaw, speed


class LiveRelativeState(Node):
    def __init__(self):
        super().__init__('relative_state')
        for name in ('geometry_dir', 'map_yaml', 'geometry_id', 'ego_odom', 'opponent_odom',
                     'ego_body_frame', 'opponent_body_frame', 'steering_topic'):
            self.declare_parameter(name, '')
        self.declare_parameter('max_age_s', 0.15)
        self.declare_parameter('max_skew_s', 0.03)
        self.declare_parameter('max_speed_mps', 0.0)
        self.age = self.get_parameter('max_age_s').value
        self.skew = self.get_parameter('max_skew_s').value
        speed = self.get_parameter('max_speed_mps').value
        if not all(math.isfinite(v) and v > 0 for v in (self.age, self.skew, speed)):
            raise ValueError('input timing and speed limits must be positive and finite')
        inputs = [self.get_parameter(name).value for name in ('ego_odom', 'opponent_odom')]
        self.frames = [self.get_parameter(name).value for name in ('ego_body_frame', 'opponent_body_frame')]
        if not all(inputs + self.frames) or len(set(inputs)) != 2 or len(set(self.frames)) != 2:
            raise ValueError('two distinct odometry topics and body frames are required')

        self.geometry = load_geometry(
            self.get_parameter('geometry_dir').value, self.get_parameter('map_yaml').value,
            inspect=True)
        if self.get_parameter('geometry_id').value != self.geometry.geometry_id:
            raise ValueError('unexpected geometry ID')
        options = TrackingOptions(max_pose_age_s=self.age, max_speed_mps=speed)
        self.trackers = [FrenetTracker(self.geometry, options, source_reference=frame)
                         for frame in self.frames]
        self.latest = [None, None]
        self.consumed = [-1, -1]
        self.seen_opponent = False
        self.progress_offset = None
        self.run_id = uuid4().hex
        self.sequence = 0
        self.publisher = self.create_publisher(RelativeState, 'relative_state', 1)
        for index, topic in enumerate(inputs):
            self.create_subscription(
                Odometry, topic, lambda msg, i=index: self.receive(i, msg), qos_profile_sensor_data)
        self.steering = None
        steering_topic = self.get_parameter('steering_topic').value
        if steering_topic:
            self.create_subscription(SteeringState, steering_topic, self.receive_steering, qos_profile_sensor_data)
        self.create_timer(0.05, self.observe)

    def receive_steering(self, msg):
        if self.steering is None or stamp_ns(msg.header.stamp) > stamp_ns(self.steering.header.stamp):
            if 0 < stamp_ns(msg.header.stamp) <= self.get_clock().now().nanoseconds:
                self.steering = msg

    def add_rollout(self, msg, now):
        odom = self.latest[0]
        steering = self.steering
        if steering is None:
            return False
        stamp = stamp_ns(steering.header.stamp)
        twist = odom.twist.twist
        if (steering.header.frame_id != self.frames[0] or not 0 <= now - stamp <= self.age * 1e9
                or abs(stamp - stamp_ns(odom.header.stamp)) > self.skew * 1e9
                or steering.phase_ticks >= 5 or not all(math.isfinite(x) for x in (
                    steering.measured_angle_rad, steering.held_command_rad, *steering.delay_buffer_rad,
                    twist.angular.z, twist.linear.x, twist.linear.y)) or twist.linear.x < 0):
            return False
        msg.steering = steering
        msg.yaw_rate_radps = twist.angular.z
        msg.slip_angle_rad = math.atan2(twist.linear.y, twist.linear.x)
        msg.ego.steering_angle_rad = steering.measured_angle_rad
        msg.ego.steering_valid = True
        msg.ego.steering_source = VehicleState.STEERING_MEASURED
        msg.ego.steering_stamp = steering.header.stamp
        return True

    def receive(self, index, message):
        stamp = stamp_ns(message.header.stamp)
        if stamp <= 0 or stamp > self.get_clock().now().nanoseconds:
            return
        old = self.latest[index]
        if old is None or stamp > stamp_ns(old.header.stamp):
            self.latest[index] = message

    def project(self, index, message, now):
        try:
            x, y, yaw, speed = read_odometry(message, self.frames[index])
        except ValueError as exc:
            return VehicleState(), None, str(exc)
        projected = self.trackers[index].update(
            (x, y), yaw, stamp_ns(message.header.stamp) / 1e9, now=now / 1e9,
            geometry_id=self.geometry.geometry_id, signed_speed=speed)
        if not projected['valid']:
            return VehicleState(), None, projected['reason']
        stamp = message.header.stamp
        return VehicleState(
            pose=Pose2D(x=x, y=y, theta=yaw), speed_mps=speed, s_m=projected['s_wrapped'],
            d_m=projected['d'], e_psi_rad=projected['e_psi'], jacobian=projected['jacobian'],
            pose_stamp=stamp, speed_stamp=stamp, valid=True, speed_source=VehicleState.SPEED_ODOM,
        ), projected['s_unwrapped'], ''

    def observe(self):
        now = self.get_clock().now().nanoseconds
        msg = RelativeState()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.geometry_id = self.geometry.geometry_id
        msg.opponent_status = RelativeState.LOST if self.seen_opponent else RelativeState.UNKNOWN
        stamps = [stamp_ns(item.header.stamp) if item is not None else -1 for item in self.latest]
        unavailable = [name for name, stamp in zip(('ego', 'opponent'), stamps)
                       if stamp <= 0 or not 0 <= now - stamp <= self.age * 1e9]
        if unavailable:
            msg.reason = 'missing_or_stale_odometry:' + ','.join(unavailable)
        elif any(stamp <= last for stamp, last in zip(stamps, self.consumed)):
            # The second half of the next pair can arrive on the next DDS callback.
            return
        elif abs(stamps[0] - stamps[1]) > self.skew * 1e9:
            msg.reason = 'odometry_pair_skew'
        else:
            self.consumed = stamps
            msg.header.stamp = self.latest[stamps.index(max(stamps))].header.stamp
            msg.ego, ego_s, ego_reason = self.project(0, self.latest[0], now)
            msg.opponent, other_s, other_reason = self.project(1, self.latest[1], now)
            if msg.ego.valid and msg.opponent.valid:
                gap = other_s - ego_s
                length = self.geometry.curve.L
                if self.progress_offset is None:
                    self.progress_offset = -round(gap / length) * length
                gap += self.progress_offset
                msg.delta_s_unwrapped_m = gap
                msg.delta_s_local_m = (gap + length / 2) % length - length / 2
                msg.relative_speed_mps = (
                    msg.opponent.speed_mps * math.cos(msg.opponent.e_psi_rad) / msg.opponent.jacobian
                    - msg.ego.speed_mps * math.cos(msg.ego.e_psi_rad) / msg.ego.jacobian)
                self.seen_opponent = True
                msg.opponent_status = RelativeState.TRACKED
                msg.rollout_valid = self.add_rollout(msg, now)
                if not msg.rollout_valid:
                    msg.reason = 'missing_or_invalid_rollout_feedback'
            else:
                msg.reason = f'ego={ego_reason or "valid"}; opponent={other_reason or "valid"}'
        self.sequence += 1
        msg.state_id = f'{self.run_id}:{self.sequence}'
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = LiveRelativeState()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

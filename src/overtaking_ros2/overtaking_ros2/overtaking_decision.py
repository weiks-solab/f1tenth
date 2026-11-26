"""Select a five-path plan from time-aligned live measurements."""
from concurrent.futures import ThreadPoolExecutor
from copy import copy
import math
from pathlib import Path
from time import monotonic

import numpy as np
import rclpy
import yaml
from f1tenth_overtaking_core.collision.discs import OccupancyMap, ThreeDiscBody
from f1tenth_overtaking_core.overtaking.rollout import SingleTrackRollout
from f1tenth_overtaking_core.overtaking.two_car_decision import OvertakingSelector
from f1tenth_overtaking_core.tracking import OpenCandidatePP
from geometry_msgs.msg import Point
from overtaking_msgs.msg import CandidatePaths, MapAssets, PlanningPolicy, RelativeState, TargetPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from .planning import load_assets, map_qos, planning_state, stamp_ns


class Pursuit:
    def __init__(self, config, model):
        self.config = config
        self.wheelbase = model['lf'] + model['lr']
        self.max_steering = min(model['s_max'], -model['s_min'])

    def lookahead(self, speed):
        return self.config.pp_lookahead_m

    def pursuit(self, path):
        return OpenCandidatePP(path['xy'], self.config.speed_mps, self.lookahead(0),
                               self.wheelbase, self.max_steering)


PERIOD = 0.1
MAX_SOURCE_AGE = 0.15
MAX_PLAN_AGE = 0.25


def vehicles(state):
    stamp = stamp_ns(state.header.stamp)
    ego, lead = [planning_state(v, state, stamp) for v in (state.ego, state.opponent)]
    lead['delta_s_unwrapped_m'] = state.delta_s_unwrapped_m
    return ego, lead


def state_issue(state, geometry, now):
    if geometry is None:
        return 'waiting_for_map'
    if state is None:
        return 'waiting_for_relative_state'
    stamp = stamp_ns(state.header.stamp)
    if (not state.state_id or state.geometry_id != geometry.geometry_id
            or state.header.frame_id != 'map' or not 0 <= now - stamp < MAX_SOURCE_AGE * 1e9):
        return 'stale_or_mismatched_state'
    if state.opponent_status != RelativeState.TRACKED:
        return state.reason or 'opponent_unavailable'
    for v in (state.ego, state.opponent):
        if (not v.valid or not np.isfinite([v.pose.x, v.pose.y, v.pose.theta, v.speed_mps,
                v.s_m, v.d_m, v.e_psi_rad, v.jacobian]).all() or v.jacobian < .2
                or not 0 <= v.s_m < geometry.curve.L or v.speed_mps < 0
                or not 0 <= now - stamp_ns(v.pose_stamp) < MAX_SOURCE_AGE * 1e9
                or not 0 <= now - stamp_ns(v.speed_stamp) < MAX_SOURCE_AGE * 1e9
                or max(abs(stamp - stamp_ns(v.pose_stamp)), abs(stamp - stamp_ns(v.speed_stamp))) > 30_000_000):
            return 'invalid_vehicle_measurement'
    if not math.isfinite(state.delta_s_unwrapped_m):
        return 'invalid_gap'
    return ''


def feedback_ok(state, model, body_frame):
    s = state.steering
    return (state.rollout_valid and state.ego.steering_valid and s.header.frame_id == body_frame
            and s.phase_ticks < 5 and abs(stamp_ns(s.header.stamp) - stamp_ns(state.ego.pose_stamp)) <= 30_000_000
            and math.isfinite(state.yaw_rate_radps) and abs(state.slip_angle_rad) < math.pi / 2
            and state.ego.speed_mps <= model['v_max'] * math.cos(state.slip_angle_rad)
            and all(model['s_min'] <= v <= model['s_max'] for v in (
                s.measured_angle_rad, s.held_command_rad, *s.delay_buffer_rad)))


def select(selector, state, candidates, controller, body, occupancy, model):
    steer = state.steering
    v = state.ego
    # Use the velocity magnitude expected by the single-track model.
    velocity = v.speed_mps / math.cos(state.slip_angle_rad)
    predictor = SingleTrackRollout(controller, body, occupancy, params=model,
        timestep=model['timestep'], steer_ticks=5, pair_clearance=.05, wall_clearance=.05,
        physics_tick=steer.phase_ticks,
        state=[v.pose.x, v.pose.y, steer.measured_angle_rad, velocity, v.pose.theta,
               state.yaw_rate_radps, state.slip_angle_rad], steer_buffer=steer.delay_buffer_rad)
    selector.rollout = predictor
    selector.held_steering = steer.held_command_rad
    paths = [dict(candidate_id=p.candidate_id, target_d_m=p.target_d_m,
                  sampled_valid=p.sampled_valid, footprint_sampled_valid=p.footprint_sampled_valid,
                  reason=p.rejection_reason, xy=np.array([(q.x, q.y) for q in p.points]))
             for p in candidates.paths]
    result = selector(dict(paths=paths), *vehicles(state))
    target = TargetPath(header=state.header, state_id=state.state_id, plan_id=candidates.plan_id,
                        geometry_id=state.geometry_id, reason=result['selection_reason'],
                        valid_until=(Time.from_msg(state.header.stamp) + Duration(seconds=MAX_PLAN_AGE)).to_msg())
    path = result['selected_path']
    if path is not None and result['target_speed_mps'] > 0:
        target.mode = getattr(TargetPath, result['selection_reason'].upper())
        target.target_speed_mps = result['target_speed_mps']
        target.candidate_id = path['candidate_id']
        target.points = [Point(x=float(x), y=float(y)) for x, y in path['xy']]
    return selector, target, monotonic()


class OvertakingDecision(Node):
    def __init__(self):
        super().__init__('overtaking_decision')
        for key in ('maps_root', 'manifest_sha256', 'model_file', 'ego_body_frame'):
            self.declare_parameter(key, '')
        self.frame = self.get_parameter('ego_body_frame').value
        self.model = yaml.safe_load(Path(self.get_parameter('model_file').value).read_text())
        if self.model['timestep'] != .01:
            raise ValueError('feedback phase and delay samples require a 10 ms model timestep')
        self.geometry = self.latest = self.pending = self.job = None
        self.last_used = -1
        self.previous_plan_id = ''
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.target_pub = self.create_publisher(TargetPath, 'target_path', 1)
        self.policy_pub = self.create_publisher(PlanningPolicy, 'planning_policy', 1)
        self.create_subscription(MapAssets, 'map_assets', self.receive_map, map_qos())
        self.create_subscription(RelativeState, 'relative_state', self.receive_state, 1)
        self.create_subscription(CandidatePaths, 'candidate_paths', self.receive_paths, 1)
        self.create_timer(PERIOD, self.tick)

    def receive_map(self, msg):
        self.geometry = self.pending = self.latest = None
        self.last_used = -1
        self.previous_plan_id = ''
        try:
            geo, _, config, _ = load_assets(self, msg)
        except (OSError, ValueError, KeyError) as exc:
            self.get_logger().error(f'Cannot load map: {exc}')
            return
        self.controller = Pursuit(config, self.model)
        self.body = ThreeDiscBody(config.footprint_length_m, config.footprint_width_m)
        self.occupancy = OccupancyMap.from_raster(geo.raster)
        self.selector = OvertakingSelector(geo, self.controller, config.speed_mps, rollout=None,
            body=self.body, timestep=self.model['timestep'], pair_clearance=.05, wall_clearance=.05)
        self.geometry, self.domain_id = geo, msg.domain_id

    def receive_state(self, msg):
        if self.latest is not None and stamp_ns(msg.header.stamp) <= stamp_ns(self.latest.header.stamp):
            return
        self.latest = msg
        if state_issue(msg, self.geometry, self.get_clock().now().nanoseconds) or not feedback_ok(msg, self.model, self.frame):
            self.pending = None

    def receive_paths(self, msg):
        if self.pending is None or self.job is not None:
            return
        state, policy, deadline = self.pending
        if (monotonic() >= deadline or msg.header != state.header or msg.geometry_id != state.geometry_id
                or msg.domain_id != self.domain_id or msg.plan_id != state.state_id
                or msg.state_id != state.state_id or msg.valid_until != policy.valid_until
                or [p.candidate_id for p in msg.paths] != list(range(1, 6))
                or not feedback_ok(state, self.model, self.frame)):
            return
        self.job = self.pool.submit(select, copy(self.selector), state, msg, self.controller,
                                    self.body, self.occupancy, self.model)

    def tick(self):
        now = self.get_clock().now()
        reason = state_issue(self.latest, self.geometry, now.nanoseconds)
        feedback = not reason and feedback_ok(self.latest, self.model, self.frame)
        target = None
        if self.job is not None and self.job.done():
            try:
                selector, result, finished = self.job.result()
                if (self.pending and self.pending[0].state_id == result.state_id and feedback
                        and finished <= self.pending[2] and now.nanoseconds < stamp_ns(result.valid_until)):
                    self.selector, target = selector, result
                    self.previous_plan_id = target.plan_id
            except (ValueError, ArithmeticError) as exc:
                self.get_logger().error(f'Rollout rejected: {exc}', throttle_duration_sec=1.)
            self.job = None
        if target is None:
            target = TargetPath(mode=TargetPath.STOP, reason=reason or (
                'missing_or_invalid_rollout_feedback' if not feedback else 'no_timely_plan'))
            target.header.frame_id = 'map'
            target.header.stamp = now.to_msg()
            target.valid_until = (now + Duration(seconds=PERIOD)).to_msg()
            if self.latest:
                target.state_id = self.latest.state_id
                target.geometry_id = self.latest.geometry_id
        self.target_pub.publish(target)
        self.pending = None
        if reason or self.job is not None:
            return
        state = self.latest
        stamp = stamp_ns(state.header.stamp)
        if stamp <= self.last_used:
            return
        self.last_used = stamp
        ego, lead = vehicles(state)
        policy = PlanningPolicy(header=state.header, state_id=state.state_id, geometry_id=state.geometry_id,
            previous_plan_id=self.previous_plan_id, valid_until=(now + Duration(seconds=PERIOD)).to_msg(),
            policy=(PlanningPolicy.CENTERLINE_RETURN if self.selector.plan_policy(lead, ego) == 'centerline_return'
                    else PlanningPolicy.REFERENCE_CENTERLINE))
        self.pending = state, policy, monotonic() + PERIOD
        self.policy_pub.publish(policy)


def main():
    rclpy.init()
    node = OvertakingDecision()
    try:
        rclpy.spin(node)
    finally:
        node.pool.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""Generate five candidates for a matched vehicle state and policy."""

from dataclasses import replace
import math
from pathlib import Path

import yaml
from f1tenth_overtaking_core.overtaking.mm_bspline_repair import BudgetedDomainGenerator
from f1tenth_overtaking_core.overtaking.session import make_plan
from f1tenth_overtaking_core.sampling import sample_candidate_xy
from geometry_msgs.msg import Point
from overtaking_msgs.msg import CandidatePath, CandidatePaths, MapAssets, PlanningPolicy, RelativeState
import rclpy
from rclpy.node import Node

from .planning import load_assets, map_qos, planning_state, stamp_ns


class PathGeneration(Node):
    def __init__(self):
        super().__init__('path_generation')
        self.declare_parameter('maps_root', '')
        self.declare_parameter('manifest_sha256', '')
        self.declare_parameter('model_file', '')
        model = yaml.safe_load(Path(self.get_parameter('model_file').value).read_text())
        self.curvature_limit = math.tan(min(model['s_max'], -model['s_min'])) / (model['lf'] + model['lr'])
        self.assets = None
        self.domain_id = ''
        self.states = {}
        self.policy = None
        self.completed_stamp = -1
        self.publisher = self.create_publisher(CandidatePaths, 'candidate_paths', 1)
        self.create_subscription(MapAssets, 'map_assets', self.receive_map, map_qos())
        self.create_subscription(RelativeState, 'relative_state', self.receive_state, 1)
        self.create_subscription(PlanningPolicy, 'planning_policy', self.receive_policy, 1)

    def receive_map(self, message):
        self.assets = None
        self.states.clear()
        self.policy = None
        self.completed_stamp = -1
        try:
            self.assets = load_assets(self, message, load_lookup=True)
        except (OSError, ValueError, KeyError) as exc:
            self.get_logger().error(f'Cannot load map: {exc}')
            return
        geo, domain, config, lookup = self.assets
        self.assets = geo, domain, replace(config, max_curvature_inv_m=min(
            config.max_curvature_inv_m, self.curvature_limit)), lookup
        self.domain_id = message.domain_id
        self.generator = BudgetedDomainGenerator(
            self.assets[1], budget_s=0.04, repair_candidates=(1, 2, 4, 5), lateral_inset_m=0.12)

    def receive_state(self, message):
        if stamp_ns(message.header.stamp) <= self.completed_stamp:
            return
        self.states[message.state_id] = message
        if len(self.states) > 8:
            del self.states[next(iter(self.states))]
        self.generate()

    def receive_policy(self, message):
        self.policy = message
        self.generate()

    def generate(self):
        if self.assets is None or self.policy is None:
            return
        geometry, _, config, lookup = self.assets
        policy = self.policy
        state = self.states.get(policy.state_id)
        if state is None:
            return
        stamp = stamp_ns(state.header.stamp)
        now = self.get_clock().now().nanoseconds
        if (stamp <= self.completed_stamp
                or state.geometry_id != geometry.geometry_id or policy.geometry_id != geometry.geometry_id
                or state.header.frame_id != 'map' or policy.header != state.header
                or not stamp <= now < stamp_ns(policy.valid_until)
                or policy.policy not in (PlanningPolicy.REFERENCE_CENTERLINE, PlanningPolicy.CENTERLINE_RETURN)):
            return
        self.completed_stamp = stamp
        target = ('centerline_return' if policy.policy == PlanningPolicy.CENTERLINE_RETURN
                  else 'reference_centerline')
        if self.generator.target_policy != target:
            self.generator.previous.clear()
            self.generator.target_policy = target
        plan = make_plan(geometry, planning_state(state.ego, state, now), config,
                         state.state_id, generator=self.generator, footprint_lookup=lookup)
        message = CandidatePaths()
        message.header = state.header
        message.plan_id = plan['plan_id']
        message.state_id = state.state_id
        message.geometry_id = geometry.geometry_id
        message.domain_id = self.domain_id
        message.valid_until = policy.valid_until
        if plan.get('result', {}).get('start') is not None:
            message.s_origin_m = plan['result']['start']['s']
        paths = []
        for path in plan['paths']:
            candidate = CandidatePath(
                candidate_id=path['candidate_id'], target_d_m=float(path.get('target_d_m') or 0.0),
                sampled_valid=bool(path['sampled_valid']),
                footprint_sampled_valid=bool(path.get('footprint_sampled_valid', False)),
                rejection_reason=path.get('reason') or '',
            )
            if candidate.sampled_valid and candidate.footprint_sampled_valid:
                try:
                    candidate.points = [
                        Point(x=float(x), y=float(y))
                        for x, y in sample_candidate_xy(geometry, path, config.lookahead_m)]
                except ValueError as exc:
                    candidate.rejection_reason = str(exc)
                    candidate.sampled_valid = candidate.footprint_sampled_valid = False
            paths.append(candidate)
        message.paths = paths
        if self.get_clock().now().nanoseconds < stamp_ns(message.valid_until):
            self.publisher.publish(message)


def main():
    rclpy.init()
    node = PathGeneration()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

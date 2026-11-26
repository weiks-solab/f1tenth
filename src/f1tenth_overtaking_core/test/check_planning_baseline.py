"""Compare the port with upstream on recorded my_map states and live simulation steps."""
import argparse
import gzip
import json
from pathlib import Path

import numpy as np

from f1tenth_benchmarks.research.overtaking import two_car_env
from f1tenth_benchmarks.research.overtaking.session import make_plan as upstream_plan
from f1tenth_benchmarks.research.run_mm_lap_comparison import DomainGenerator as UpstreamGenerator
from f1tenth_benchmarks.research.run_two_car_candidates import run

from f1tenth_overtaking_core.geometry import load_geometry
from f1tenth_overtaking_core.overtaking.domain_generator import DomainGenerator
from f1tenth_overtaking_core.overtaking.millimetre_domain import MillimetreDomain
from f1tenth_overtaking_core.overtaking.rollout import SingleTrackRollout
from f1tenth_overtaking_core.overtaking.session import SessionConfig, make_plan
from f1tenth_overtaking_core.overtaking.two_car_decision import OvertakingSelector


class ComparingSelector(two_car_env.OvertakingSelector):
    """Compare two rollout implementations from the same physical state."""

    checks = 0

    def __init__(self, geometry, sim, controller, cruise_speed, steer_ticks,
                 confirm_pass=False, curve_speed_control=False, measured_bend_caps=()):
        super().__init__(geometry, sim, controller, cruise_speed, steer_ticks, confirm_pass,
                         curve_speed_control, measured_bend_caps)
        self.port = OvertakingSelector(
            geometry, controller, cruise_speed, rollout=self.rollout, body=sim.bodies[0],
            timestep=sim.params.timestep, pair_clearance=sim.pair_clearance,
            wall_clearance=sim.wall_clearance, confirm_pass=confirm_pass,
            curve_speed_control=curve_speed_control, measured_bend_caps=measured_bend_caps)

    def rollout(self, *args, held_steering):
        vehicle = self.sim.cars[0]
        rollout = SingleTrackRollout(
            self.controller, self.body, self.sim.map, params=vehicle.params,
            timestep=self.dt, steer_ticks=self.steer_ticks,
            pair_clearance=self.sim.pair_clearance, wall_clearance=self.sim.wall_clearance,
            physics_tick=self.sim.physics_tick, state=vehicle.state,
            steer_buffer=vehicle.steer_buffer)
        return rollout(*args, held_steering=held_steering)

    def __call__(self, plan_event, ego, lead, **kwargs):
        self.port.held_steering = self.held_steering
        assert self.port.plan_policy(lead, ego) == self.plan_policy(lead, ego)
        upstream = super().__call__(plan_event, ego, lead, **kwargs)
        port = self.port(plan_event, ego, lead, **kwargs)
        for key in ('selected_candidate_id', 'target_speed_mps', 'selection_reason', 'costs'):
            assert upstream[key] == port[key], key
        assert self.completed_passes == self.port.completed_passes
        type(self).checks += 1
        return upstream


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('baseline_results', type=Path)
    parser.add_argument('comparison_output', type=Path)
    parser.add_argument('--check-esp', action='store_true')
    parser.add_argument('--esp-domain', type=Path)
    args = parser.parse_args()

    geometry = load_geometry(args.bundle / 'geometry', args.bundle / 'my_map.yaml', inspect=True)
    domain = MillimetreDomain.load(args.bundle / 'domain', geometry, expected_margin=0.185)
    settings = SessionConfig(**json.loads((args.baseline_results / 'report.json').read_text())['config'])
    with gzip.open(args.baseline_results / 'frames.json.gz', 'rt') as stream:
        frames = json.load(stream)
    upstream = UpstreamGenerator(domain, lateral_inset_m=0.12)
    port = DomainGenerator(domain, lateral_inset_m=0.12)
    plans = 0
    for frame in frames:
        if frame.get('plan') is None:
            continue
        state = frame['ego_state']
        reference = upstream_plan(geometry, state, settings, plans, generator=upstream)
        actual = make_plan(geometry, state, settings, plans, generator=port)
        assert len(actual['paths']) == len(reference['paths']) == 5
        for left, right in zip(actual['paths'], reference['paths']):
            for key in ('candidate_id', 'reason', 'sampled_valid', 'footprint_sampled_valid'):
                assert left[key] == right[key], key
            if left.get('spline') is not None:
                np.testing.assert_array_equal(left['spline']['coefficients'], right['spline']['coefficients'])
        plans += 1
    assert plans == 120
    print(f'{plans} five-slot plans match upstream, including footprint gates', flush=True)

    two_car_env.OvertakingSelector = ComparingSelector
    run(args.comparison_output, domain_dir=args.bundle / 'domain', phase=2, seconds=12.)
    assert ComparingSelector.checks == 120
    print(f'{ComparingSelector.checks} decisions and predicted clearances match using independent single-track rollout')
    if args.check_esp or args.esp_domain is not None:
        ComparingSelector.checks = 0
        output = args.comparison_output.with_name(args.comparison_output.name + '_esp')
        run(output, map_name='esp', domain_dir=args.esp_domain, phase=2, seconds=18.,
            ego_speed=1.35, lead_speed=0.85)
        report = json.loads((output / 'report.json').read_text())
        assert ComparingSelector.checks == 180
        assert report['completed_passes'] == 1, report['decision_counts']
        assert report['events'] == []
        print('ESP: 180 decisions agree, with one completed pass and no envelope event.')


if __name__ == '__main__':
    main()

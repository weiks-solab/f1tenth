"""Compare the lookup wall gate against exact checking of recorded candidates."""
import argparse
from copy import deepcopy
from dataclasses import replace
import gzip
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from f1tenth_overtaking_core.collision.discs import ThreeDiscBody
from f1tenth_overtaking_core.geometry import load_geometry
from f1tenth_overtaking_core.overtaking.footprint_lookup import FootprintLookup
from f1tenth_overtaking_core.overtaking.footprint_validation import check_candidate_footprints
from f1tenth_overtaking_core.overtaking.session import SessionConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('frames', type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.bundle / 'bundle.json').read_text())
    geometry = load_geometry(args.bundle / 'geometry', args.bundle / manifest['map_yaml'], inspect=True)
    config = SessionConfig(**json.loads((args.bundle / manifest['planning_report']).read_text())['config'])
    lookup = FootprintLookup(geometry, config, args.bundle / manifest['footprint_lookup'])
    try:
        FootprintLookup(geometry, replace(config, footprint_width_m=config.footprint_width_m + 0.01),
                        args.bundle / manifest['footprint_lookup'])
    except ValueError as exc:
        assert 'vehicle profile mismatch' in str(exc)
    else:
        raise AssertionError('accepted a footprint table made for a different car width')

    random = np.random.default_rng(503)
    dimensions = np.array([geometry.raster.width, geometry.raster.height]) * geometry.raster.resolution
    points = geometry.raster.local_to_map(random.uniform(-0.2, dimensions + 0.2, (2000, 2)))
    poses = np.column_stack((points, random.uniform(-np.pi, np.pi, len(points))))
    body = ThreeDiscBody(config.footprint_length_m, config.footprint_width_m)
    radius = body.radius + config.footprint_clearance_m
    safe = np.ones(len(poses), dtype=bool)
    safe[lookup.possible_hits(body, poses)] = False
    assert safe.any()
    assert all(not lookup.occupancy.intersects(body.centres(pose), radius) for pose in poses[safe])
    with gzip.open(args.frames, 'rt') as stream:
        frames = json.load(stream)

    timings = [[], []]
    counts = [0, 0]
    pairs = 0
    for frame in frames:
        plan = frame.get('plan')
        if plan is None:
            continue
        results = []
        for index, mode in enumerate((None, lookup)):
            paths = deepcopy(plan['paths'])
            for path in paths:
                path['sampled_valid'] = path['footprint_check']['checked']
            result = dict(paths=paths, start=plan.get('result', {}).get('start', {}))
            start = perf_counter()
            summary = check_candidate_footprints(geometry, result, plan['source_pose'], config, lookup=mode)
            timings[index].append((perf_counter() - start) * 1000)
            counts[index] += summary['exact_candidate_pose_checks']
            results.append([path['footprint_check'] for path in paths])
        assert results[0] == results[1], f'footprint result mismatch at {pairs}'
        pairs += 1
    assert pairs and counts[1] < counts[0]
    print(f'{manifest["map_id"]}: {pairs} batches match exactly, including first contact sample')
    for mode, values, count in zip(('exact', 'lookup'), timings, counts):
        print(f'{mode}: median {np.median(values):.2f} ms, P95 {np.percentile(values, 95):.2f} ms, '
              f'{count} exact checks')


if __name__ == '__main__':
    main()

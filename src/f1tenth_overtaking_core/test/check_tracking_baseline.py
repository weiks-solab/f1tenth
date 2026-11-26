"""Compare extracted tracking behavior against the pinned Phase 2 session."""
import argparse
import gzip
import json
import sys
from pathlib import Path

import numpy as np

from f1tenth_overtaking_core.geometry import load_geometry
from f1tenth_overtaking_core.sampling import sample_candidate_xy
from f1tenth_overtaking_core.tracking import OpenCandidatePP


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline_checkout', type=Path)
    parser.add_argument('frames', type=Path)
    args = parser.parse_args()
    maps = args.baseline_checkout / 'maps'
    geometry = load_geometry(maps / 'frenet/my_map/geometry', maps / 'my_map.yaml', inspect=True)
    with gzip.open(args.frames, 'rt') as stream:
        frames = json.load(stream)

    selection_count = 0
    for frame in frames:
        choice = frame.get('selected_candidate_id')
        if choice is None:
            continue
        plan = frame['plan']
        path = next(path for path in plan['paths'] if path['candidate_id'] == choice)
        points = sample_candidate_xy(geometry, path, 3.0)
        expected = frame['actions'][0]
        action, info = OpenCandidatePP(points, expected[1], 0.3, 0.33, 0.5148721293383273).plan(
            np.asarray(plan['source_pose']))
        np.testing.assert_allclose(action, expected, rtol=0, atol=1e-10)
        np.testing.assert_allclose(
            info['lookahead_xy'], frame['ego_control']['lookahead_xy'], rtol=0, atol=1e-10)
        selection_count += 1

    if selection_count != 118 or any(
        name.startswith('f1tenth_benchmarks.simulator') for name in sys.modules
    ):
        raise AssertionError('unexpected baseline selections or simulator import')
    print(f'{selection_count} selected paths match recorded steering, speed and lookahead')


if __name__ == '__main__':
    main()

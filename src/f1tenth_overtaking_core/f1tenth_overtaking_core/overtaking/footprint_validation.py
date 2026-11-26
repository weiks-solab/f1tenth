# SPDX-License-Identifier: Apache-2.0
# Adapted from f1tenth_benchmarks at 008907f3826ddf486f88dcd6079aa6d4c240084e.
"""Final three-disc gate on analytic candidate poses, including repaired paths.

This verifies sampled poses, not the continuous curve or tracking error tube.
The original trinary planning map is authoritative; unknown cells are blocked.
"""
import math

import numpy as np
from scipy.interpolate import BSpline

from ..collision.discs import ThreeDiscBody, OccupancyMap


def candidate_poses(reference, lateral, slope):
    xy, tangent, heading, curvature = reference
    jacobian = 1 - curvature * lateral
    positions = xy + lateral[:, None] * np.column_stack([-tangent[:, 1], tangent[:, 0]])
    return np.column_stack([positions, heading + np.arctan2(slope, jacobian)]), jacobian


def check_candidate_footprints(geometry, result, source_pose, config, *, lookup=None):
    body = ThreeDiscBody(config.footprint_length_m, config.footprint_width_m)
    settings = dict(model='three_discs', length_m=body.length, width_m=body.width,
                    radius_m=body.radius, longitudinal_offsets_m=[-body.length/3, 0., body.length/3],
                    clearance_m=config.footprint_clearance_m, sample_step_s_m=config.footprint_step_m,
                    reference_point='geometric_body_centre', unknown_is_obstacle=True,
                    scope='sampled_analytic_curve_poses', continuous_verified=False)
    occupancy = lookup.occupancy if lookup is not None else OccupancyMap.from_raster(geometry.raster)
    exact_sample = geometry.curve.sample
    radius = body.radius+config.footprint_clearance_m
    source_safe = not occupancy.intersects(body.centres(source_pose), radius)
    length = config.lookahead_m
    count = int(math.ceil(length/config.footprint_step_m))+1
    if count > 100001:
        raise ValueError('footprint sampling exceeds 100001 poses')
    distance = np.linspace(0., length, count)
    reference_samples = {}
    exact_checks = 0
    for path in result['paths']:
        path['footprint_sampled_valid'] = False
        path['footprint_verified'] = False
        if not path['sampled_valid']:
            path['footprint_check'] = dict(checked=False, reason='prior_gate_rejected')
            continue
        check = dict(checked=True, valid=False, source_pose_valid=source_safe, sample_count=0,
                     first_invalid_sample=None, reason=None)
        if not source_safe:
            check['reason'] = 'source_three_disc_clearance'
        else:
            try:
                spec = path['spline']
                spline = BSpline(spec['knots'], spec['coefficients'], spec['degree'], extrapolate=False)
                if spline.t[spline.k] > 0 or spline.t[-spline.k-1] < length:
                    raise ValueError('incomplete spline domain')
                origin = float(spec.get('source_s_origin', path.get('reference_s_origin',
                               result.get('start', {}).get('s', float('nan')))))
                sites = origin + distance
                if origin not in reference_samples:
                    sample = lookup.reference.sample if lookup is not None else exact_sample
                    reference_samples[origin] = sample(sites)
                lateral, slope = spline(distance), spline(distance, nu=1)
                poses, jacobian = candidate_poses(reference_samples[origin], lateral, slope)
                use_lookup = lookup is not None and np.isfinite(poses).all() and np.all(jacobian > 1e-3)
                if lookup is not None and not use_lookup:
                    poses, jacobian = candidate_poses(exact_sample(sites), lateral, slope)
                if not np.isfinite(poses).all() or np.any(jacobian <= 0):
                    raise ValueError('invalid curve pose')
                indices = range(len(poses))
                if use_lookup:
                    indices = lookup.possible_hits(body, poses)
                    if len(indices):
                        poses[indices], _ = candidate_poses(
                            exact_sample(sites[indices]), lateral[indices], slope[indices])
                for index in indices:
                    index = int(index)
                    pose = poses[index]
                    exact_checks += 1
                    if occupancy.intersects(body.centres(pose), radius):
                        check.update(sample_count=index+1, reason='three_disc_clearance', first_invalid_sample=dict(
                            index=index, distance_s_m=float(distance[index]), pose=pose.tolist()))
                        break
                else:
                    check.update(valid=True, sample_count=len(poses))
            except (KeyError, TypeError, ValueError, IndexError) as exc:
                check.update(reason='footprint_curve_unavailable', detail=str(exc))
        path['footprint_check'] = check
        path['footprint_sampled_valid'] = check['valid']
        if not check['valid']:
            path.update(sampled_valid=False, status='rejected', reason=check['reason'])
            if 'experimental_valid' in path:
                path['experimental_valid'] = False
    return dict(settings=settings, source_pose_valid=source_safe,
                exact_candidate_pose_checks=exact_checks,
                sampled_valid_count=sum(p['footprint_sampled_valid'] for p in result['paths']))

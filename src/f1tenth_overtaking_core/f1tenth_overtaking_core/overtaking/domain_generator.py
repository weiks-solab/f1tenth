# SPDX-License-Identifier: Apache-2.0
# Adapted from f1tenth_benchmarks at 008907f3826ddf486f88dcd6079aa6d4c240084e.
"""Candidate generator extracted from the offline simulation entry points."""
import math

import numpy as np

from .reference_path import REFERENCE_PATH
from .robust_adaptive_paths import direct_hermite_coefficients
from .session import reject_paths


def exact_start(geometry, pose):
    projection = geometry.to_frenet(pose[:2], yaw=pose[2])
    if not projection['valid']:
        return None, projection['reason']
    s, d = projection['s_wrapped'], projection['d']
    slope = (1-projection['kappa']*d)*math.tan(projection['e_psi'])
    if abs(projection['e_psi']) >= math.pi/2-1e-3:
        return None, 'heading_not_forward'
    # Keep measured pose exact; snapping the start would introduce a jump.
    return dict(s=float(s), d=float(d), d_prime=float(slope)), None


class DomainGenerator:
    def __init__(self, domain, target_policy='reference_centerline', lateral_inset_m=0.):
        self.domain = domain
        self.target_policy = target_policy
        self.lateral_inset_m = lateral_inset_m

    def __call__(self, geometry, x, y, yaw, **settings):
        start, reason = exact_start(geometry, [x, y, yaw])
        if start is None:
            raise ValueError('Invalid start: '+str(reason))
        length = settings['lookahead_m']
        plan = self.domain.plan(start['s'], start['d'], start['d_prime'], length=length,
            max_slope=settings['max_slope'], max_curvature=settings['max_curvature'],
            target_policy=self.target_policy, lateral_inset_m=self.lateral_inset_m)
        if not plan['paths']:
            return dict(start=start, paths=reject_paths(plan['reason']), **{k: v for k, v in plan.items() if k != 'paths'})
        # Reconstruct the mathematical start independently, so make_plan's
        # pose gate checks the curve rather than a copied source x/y/yaw.
        reference = geometry.reference(start['s'])
        nx, ny = -reference['tangent'][1], reference['tangent'][0]
        first = dict(x=reference['xy'][0]+start['d']*nx,
            y=reference['xy'][1]+start['d']*ny,
            yaw=reference['psi']+math.atan2(start['d_prime'], 1-reference['kappa']*start['d']))
        paths = []
        for proposal in plan['paths']:
            valid = proposal['experimental_valid']
            target = proposal['target_d_m']
            is_reference = proposal['path_type'] == REFERENCE_PATH
            coefficients = (np.zeros(6) if is_reference else
                            direct_hermite_coefficients(length, start['d'], start['d_prime'], target))
            path_first = dict(x=reference['xy'][0], y=reference['xy'][1], yaw=reference['psi']) if is_reference else dict(first)
            paths.append(dict(proposal, geometry_id=geometry.geometry_id,
                sampled_valid=valid, status='sampled_valid' if valid else 'rejected',
                points=[path_first], requested_target_d=target, endpoint_mode='fixed',
                reference_s_origin=start['s'], reference_d=0. if is_reference else None,
                spline=dict(knots=[0.]*6+[length]*6, degree=5, coefficients=coefficients.tolist()),
                output_representation='analytic_spline_and_single_start_point',
                footprint_verified=False))
        return dict(start=start, paths=paths, **{k: v for k, v in plan.items() if k != 'paths'})

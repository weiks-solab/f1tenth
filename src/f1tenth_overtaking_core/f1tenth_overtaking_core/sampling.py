# SPDX-License-Identifier: Apache-2.0
# Extracted from f1tenth_benchmarks at 008907f3826ddf486f88dcd6079aa6d4c240084e.

import math

import numpy as np
from scipy.interpolate import BSpline


def sample_candidate_xy(geometry, path, length_m, step_m=.02):
    """Sample an analytic candidate, or use its recorded polyline."""
    if not np.isfinite([length_m, step_m]).all() or min(length_m, step_m) <= 0:
        raise ValueError('invalid candidate sampling settings')
    spec = path.get('spline')
    if spec is not None:
        spline = BSpline(spec['knots'], spec['coefficients'], spec['degree'], extrapolate=False)
        origin = float(spec.get('source_s_origin', path.get('reference_s_origin', math.nan)))
        if not math.isfinite(origin) or spline.t[spline.k] > 0 or spline.t[-spline.k-1] < length_m:
            raise ValueError('invalid candidate spline domain')
        distance = np.linspace(0., length_m, max(2, math.ceil(length_m / step_m)) + 1)
        xy, tangent, _, _ = geometry.curve.sample(origin + distance)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        points = xy + spline(distance)[:, None] * normal
    else:
        points = np.array([[p['x'], p['y']] for p in path.get('points', [])], dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3 or not np.isfinite(points).all():
        raise ValueError('candidate requires at least three finite XY points')
    return points

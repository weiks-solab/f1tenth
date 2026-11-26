"""Offline tables for the sampled three-disc wall gate on closed tracks."""
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt

from ..collision.discs import OccupancyMap, ThreeDiscBody
from .fixed_arc_lookup import FixedArcLookup

STEP_M = 0.005
POLICY = 'gauss16_brackets_and_three_disc_center_grid_v1'


def footprint_profile(config):
    body = ThreeDiscBody(config.footprint_length_m, config.footprint_width_m)
    return np.array((body.length, body.width, config.footprint_clearance_m))


def build_footprint_lookup(geometry, config, output):
    """Persist numerical inverse seeds and conservative clearance bounds."""
    occupancy = OccupancyMap.from_raster(geometry.raster)
    factor = round(occupancy.resolution / STEP_M)
    if factor < 1 or not np.isclose(factor * STEP_M, occupancy.resolution, rtol=0, atol=1e-12):
        raise ValueError('5 mm clearance lookup requires an integer raster scale')
    curve = geometry.curve
    if not curve.closed:
        raise ValueError('offline footprint lookup requires a closed reference')
    sites = np.unique(np.r_[np.linspace(0, curve.L, int(np.ceil(curve.L / STEP_M)) + 1), curve.s_nodes])
    inverse = np.array([curve.u_at_s(s - curve.start_s) for s in sites])
    inverse[0], inverse[-1] = 0.0, 1.0
    free = (~occupancy.occupied).repeat(factor, axis=0).repeat(factor, axis=1)
    distance = distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1, 1:-1]
    # One half diagonal for each of the obstacle and query cells.
    clearance = np.maximum(0.0, distance * STEP_M - np.sqrt(2.0) * STEP_M - 1e-9)
    profile = footprint_profile(config)
    radius = ThreeDiscBody(*profile[:2]).radius + profile[2]
    # The extra guard keeps borderline comparisons on the exact raster path.
    safe_centres = clearance > radius + 1e-6
    with Path(output).open('xb') as stream:
        np.savez_compressed(stream, policy=POLICY, geometry_id=geometry.geometry_id, step_m=STEP_M,
                            arc_nodes=sites, u_nodes=inverse, profile=profile, safe_centres=safe_centres)


class FootprintLookup:
    def __init__(self, geometry, config, path):
        self.occupancy = OccupancyMap.from_raster(geometry.raster)
        self.reference = FixedArcLookup(geometry.curve, spacing=STEP_M)
        with np.load(path, allow_pickle=False) as data:
            if (data['policy'].item() != POLICY or data['geometry_id'].item() != geometry.geometry_id
                    or data['step_m'].item() != STEP_M
                    or not np.array_equal(data['profile'], footprint_profile(config))):
                raise ValueError('footprint lookup identity, policy or vehicle profile mismatch')
            safe_centres, sites, inverse = (data[name] for name in ('safe_centres', 'arc_nodes', 'u_nodes'))
        factor = round(self.occupancy.resolution / STEP_M)
        if (factor < 1 or not np.isclose(factor * STEP_M, self.occupancy.resolution, rtol=0, atol=1e-12)
                or safe_centres.shape != tuple(size * factor for size in self.occupancy.occupied.shape)
                or safe_centres.dtype != np.bool_):
            raise ValueError('invalid footprint centre grid')
        if (sites.ndim != 1 or len(sites) < 2 or inverse.shape != sites.shape
                or not np.isfinite(sites).all() or not np.isfinite(inverse).all()
                or np.any(np.diff(sites) <= 0) or np.any(np.diff(inverse) <= 0)
                or abs(sites[-1] - geometry.curve.L) > 1e-12
                or sites[0] != 0 or inverse[0] != 0 or inverse[-1] != 1):
            raise ValueError('invalid reference lookup brackets')
        # The batch inverse checks bracket signs and uses the original scalar
        # solver on unresolved roots. Cached values only seed the calculation.
        self.reference.arc_nodes = sites
        self.reference.u_nodes = inverse
        self.safe_centres = safe_centres

    def possible_hits(self, body, poses):
        offsets = np.array([-body.length / 3, 0.0, body.length / 3])
        direction = np.column_stack((np.cos(poses[:, 2]), np.sin(poses[:, 2])))
        centres = poses[:, None, :2] + direction[:, None, :] * offsets[None, :, None]
        grid = np.floor((centres - self.occupancy.origin[:2]) @ self.occupancy.rotation / STEP_M)
        inside = np.all((grid >= 0) & (grid < self.safe_centres.shape[::-1]), axis=(1, 2))
        cells = grid[inside].astype(int)
        safe = np.zeros(len(poses), dtype=bool)
        safe[inside] = np.all(self.safe_centres[cells[:, :, 1], cells[:, :, 0]], axis=1)
        return np.flatnonzero(~safe)

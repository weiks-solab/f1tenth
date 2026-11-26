"""Shared map and session settings for installed vehicle map bundles."""
import json
from pathlib import Path

from f1tenth_overtaking_core.geometry import load_geometry
from f1tenth_overtaking_core.overtaking.footprint_lookup import FootprintLookup
from f1tenth_overtaking_core.overtaking.millimetre_domain import MillimetreDomain
from f1tenth_overtaking_core.overtaking.session import SessionConfig
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .map_assets import verify_bundle


def map_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def load_assets(node, message, *, load_lookup=False):
    root = Path(node.get_parameter('maps_root').value)
    expected = node.get_parameter('manifest_sha256').value
    if message.header.frame_id != 'map' or message.manifest_sha256 != expected:
        raise ValueError('unexpected map')
    manifest = verify_bundle(root, message.map_id, message.bundle_id, expected)
    if (message.geometry_id != manifest['geometry_id']
            or message.domain_id != manifest['domain_id']):
        raise ValueError('map identity mismatch')
    bundle = root / message.map_id / message.bundle_id
    config = SessionConfig(**json.loads((bundle / manifest['planning_report']).read_text()))
    if config.footprint_check_enabled is not True:
        raise ValueError('vehicle planning requires footprint checks')
    # Bundled Stage B maps support on-car computation with isolated command output.
    geometry = load_geometry(bundle / 'geometry', bundle / manifest['map_yaml'], inspect=True)
    domain = MillimetreDomain.load(bundle / 'domain', geometry, expected_margin=config.safety_margin_m)
    lookup_file = manifest.get('footprint_lookup')
    lookup = FootprintLookup(geometry, config, bundle / lookup_file) if load_lookup and lookup_file else None
    node.get_logger().info(f'Loaded map {message.map_id}/{message.bundle_id}')
    return geometry, domain, config, lookup


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def planning_state(vehicle, message, now):
    return {
        'valid': vehicle.valid,
        'pose': [vehicle.pose.x, vehicle.pose.y, vehicle.pose.theta],
        'speed_mps': vehicle.speed_mps,
        'control_step': stamp_ns(message.header.stamp) // 100_000_000,
        'sim_time_s': now / 1e9,
        'frenet': {'valid': vehicle.valid, 's_wrapped': vehicle.s_m, 'd': vehicle.d_m,
                   'e_psi': vehicle.e_psi_rad, 'jacobian': vehicle.jacobian},
    }

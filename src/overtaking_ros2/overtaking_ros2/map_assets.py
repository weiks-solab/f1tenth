"""Announce the verified static bundle for vehicle integration."""

import hashlib
import json
from pathlib import Path

import rclpy
from overtaking_msgs.msg import MapAssets
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def verify_bundle(root, map_id, bundle_id, expected_manifest_sha):
    root = Path(root).resolve()
    bundle = (root / map_id / bundle_id).resolve()
    if bundle.parent.parent != root or bundle.parts[-2:] != (map_id, bundle_id):
        raise ValueError('invalid map or bundle ID')

    manifest_bytes = (bundle / 'bundle.json').read_bytes()
    if hashlib.sha256(manifest_bytes).hexdigest() != expected_manifest_sha:
        raise ValueError('bundle manifest hash mismatch')
    manifest = json.loads(manifest_bytes)
    if (manifest['schema_version'], manifest['scope'], manifest['frame_id'],
            manifest['map_id'], manifest['bundle_id']) != (
            1, 'vehicle_integration', 'map', map_id, bundle_id):
        raise ValueError('bundle identity, frame or scope mismatch')

    hashes = manifest['file_hashes']
    required = {manifest['map_yaml'], 'geometry/manifest.json', 'geometry/reference_curve.json',
                'domain/metadata.json', 'domain/domain.npz', manifest['footprint_lookup'],
                manifest['planning_report'], 'geometry/validation.json'}
    if not required.issubset(hashes):
        raise ValueError('incomplete map bundle')
    for name, expected in hashes.items():
        path = (bundle / name).resolve()
        if not path.is_relative_to(bundle):
            raise ValueError(f'asset escapes bundle: {name}')
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'asset hash mismatch: {name}')

    geometry = json.loads((bundle / 'geometry/manifest.json').read_text())
    domain = json.loads((bundle / 'domain/metadata.json').read_text())
    if (geometry['geometry_id'] != manifest['geometry_id']
            or domain['geometry_id'] != manifest['geometry_id']
            or domain['npz_sha256'] != manifest['domain_id']
            or hashes['domain/domain.npz'] != manifest['domain_id']):
        raise ValueError('geometry or domain identity mismatch')
    if 'domain_step_m' in manifest and not (
        domain['step_m'] == domain['d_step_m'] == manifest['domain_step_m']
    ):
        raise ValueError('map domain step mismatch')
    return manifest


class MapAssetsNode(Node):
    def __init__(self):
        super().__init__('map_assets')
        for name in ('maps_root', 'map_id', 'bundle_id', 'manifest_sha256'):
            self.declare_parameter(name, '')
        params = [self.get_parameter(name).value for name in
                  ('maps_root', 'map_id', 'bundle_id', 'manifest_sha256')]
        if not all(params):
            raise ValueError('map assets require explicit root, identity and manifest hash')
        manifest = verify_bundle(*params)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MapAssets, 'map_assets', qos)
        message = MapAssets()
        message.header.frame_id = manifest['frame_id']
        message.header.stamp = self.get_clock().now().to_msg()
        message.map_id = manifest['map_id']
        message.bundle_id = manifest['bundle_id']
        message.geometry_id = manifest['geometry_id']
        message.domain_id = manifest['domain_id']
        message.manifest_sha256 = params[-1]
        self.publisher.publish(message)
        self.get_logger().info(f'Published map {message.map_id}/{message.bundle_id}')


def main():
    rclpy.init()
    node = None
    try:
        node = MapAssetsNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""Build and pin a vehicle map bundle before starting ROS 2."""
import argparse
import json
from pathlib import Path
from shutil import copy2, copytree
from tempfile import TemporaryDirectory

from f1tenth_overtaking_core.geometry import load_geometry
from f1tenth_overtaking_core.geometry.map_raster import sha256
from f1tenth_overtaking_core.overtaking.footprint_lookup import build_footprint_lookup
from f1tenth_overtaking_core.overtaking.millimetre_domain import MillimetreDomain
from f1tenth_overtaking_core.overtaking.session import SessionConfig


def prepare(args):
    settings = json.loads(args.config.read_text())
    config = SessionConfig(**settings)
    geometry = load_geometry(args.geometry, args.map_yaml, inspect=True)
    target = args.output
    if target.parent.name != args.map_id or target.exists():
        raise ValueError('expected a new output directory under the map ID')

    image = Path(geometry.raster.config['image'])
    if image.is_absolute() or '..' in image.parts:
        raise ValueError('map YAML must refer to a relative image inside the bundle')

    target.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=target.parent) as temp:
        bundle = Path(temp) / 'bundle'
        copytree(args.geometry, bundle / 'geometry')
        (bundle / image).parent.mkdir(parents=True, exist_ok=True)
        copy2(args.map_yaml, bundle / args.map_yaml.name)
        copy2(geometry.raster.image_path, bundle / image)
        if args.domain is None:
            domain = MillimetreDomain.build(geometry, step=args.step, margin=config.safety_margin_m)
            domain.save(bundle / 'domain')
        else:
            domain = MillimetreDomain.load(args.domain, geometry, expected_margin=config.safety_margin_m)
            if domain.step != args.step:
                raise ValueError('offline domain spacing differs from requested spacing')
            copytree(args.domain, bundle / 'domain')
        build_footprint_lookup(geometry, config, bundle / 'footprint_lookup.npz')
        copy2(args.config, bundle / 'planning.json')
        # Domain.save adds the NPZ digest to the saved metadata.
        domain_id = sha256(bundle / 'domain/domain.npz')
        manifest = dict(
            schema_version=1, scope='vehicle_integration', frame_id='map',
            map_id=args.map_id, bundle_id=target.name, map_yaml=args.map_yaml.name,
            geometry_id=geometry.geometry_id, domain_id=domain_id,
            footprint_lookup='footprint_lookup.npz', planning_report='planning.json',
            domain_step_m=args.step,
            file_hashes={str(path.relative_to(bundle)): sha256(path)
                         for path in sorted(bundle.rglob('*')) if path.is_file()},
        )
        (bundle / 'bundle.json').write_text(json.dumps(manifest, indent=2) + '\n')
        bundle.rename(target)
    print(sha256(target / 'bundle.json'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('geometry', 'map-yaml', 'config', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--domain', type=Path, help='reuse a matching offline domain')
    parser.add_argument('--step', type=float, choices=(0.001, 0.005), default=0.005)
    prepare(parser.parse_args())


if __name__ == '__main__':
    main()

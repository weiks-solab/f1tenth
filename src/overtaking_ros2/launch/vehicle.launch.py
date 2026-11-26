"""On-car planning with live inputs. All defaults resolve inside the installation."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from overtaking_ros2.map_assets import verify_bundle


def nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    root, map_id, bundle_id = Path(value('maps_root')), value('map_id'), value('bundle_id')
    expected = value('manifest_sha256')
    if not expected:
        expected = {'my_map': 'ebb2883e948126a7744ddc61e26c4cd04b14b10647e33b62bd94836eccd33443',
                    'new': '8706f10171a15e9b5e6f6d86c9f48a63f5f9823184b0b849d8723379753f4932'}.get(map_id)
    manifest = verify_bundle(root, map_id, bundle_id, expected)
    bundle = root / map_id / bundle_id
    config = json.loads((bundle / manifest['planning_report']).read_text())
    model_file = value('model_file')
    model = yaml.safe_load(Path(model_file).read_text())
    frame, other = value('ego_body_frame'), value('opponent_body_frame')
    common = dict(maps_root=str(root), manifest_sha256=expected)
    geometry_id = manifest['geometry_id']

    def node(name, params, *, package='overtaking_ros2', executable=None):
        return Node(package=package, executable=executable or name, name=name, namespace='overtaking',
                    parameters=[params], output='screen')

    actions = [
        node('map_assets', dict(common, map_id=map_id, bundle_id=bundle_id)),
        node('path_generation', dict(common, model_file=model_file)),
        node('overtaking_decision', dict(common, model_file=model_file, ego_body_frame=frame)),
        node('pure_pursuit', dict(path_source='target_path', base_frame=frame,
            geometry_id=geometry_id, control_period=.05, target_max_age=.25, pose_max_age=.15,
            lookahead=config['pp_lookahead_m'], speed=config['speed_mps'],
            wheelbase=model['lf'] + model['lr'], max_steering=min(-model['s_min'], model['s_max'])),
            package='car_control', executable='pure_pursuit_tf'),
    ]
    if value('start_relative_state').lower() == 'true':
        actions.append(node('relative_state', dict(
            geometry_id=geometry_id, geometry_dir=str(bundle / 'geometry'),
            map_yaml=str(bundle / manifest['map_yaml']), ego_body_frame=frame,
            opponent_body_frame=other, ego_odom=value('ego_odom'), opponent_odom=value('opponent_odom'),
            steering_topic=value('steering_topic'), max_speed_mps=float(value('max_tracking_speed_mps')))))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('overtaking_ros2'))
    defaults = dict(maps_root=str(share / 'maps'), map_id='my_map', bundle_id='v1', manifest_sha256='',
        model_file=str(share / 'config/vehicle_model.yaml'), ego_body_frame='base_link',
        opponent_body_frame='opponent/base_link', ego_odom='/overtaking/input/ego_odom',
        opponent_odom='/overtaking/input/opponent_odom', steering_topic='/overtaking/input/steering',
        start_relative_state='true', max_tracking_speed_mps='3.0')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v,
        **({'choices': ['true', 'false']} if k == 'start_relative_state' else {}))
        for k, v in defaults.items()] + [OpaqueFunction(function=nodes)])

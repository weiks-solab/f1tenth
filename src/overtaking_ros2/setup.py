from glob import glob
from pathlib import Path
from setuptools import setup

package_name = 'overtaking_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ] + [(str(Path('share') / package_name / folder), [str(f) for f in folder.iterdir() if f.is_file()])
         for folder in sorted(Path('maps').rglob('*')) if folder.is_dir()],
    install_requires=['setuptools'],
    zip_safe=False,
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'prepare_map_assets = overtaking_ros2.prepare_map_assets:main',
        'map_assets = overtaking_ros2.map_assets:main',
        'relative_state = overtaking_ros2.relative_state:main',
        'path_generation = overtaking_ros2.path_generation:main',
        'overtaking_decision = overtaking_ros2.overtaking_decision:main',
    ]},
)

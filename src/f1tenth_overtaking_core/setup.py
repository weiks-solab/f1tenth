from setuptools import find_packages, setup

package_name = 'f1tenth_overtaking_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    package_data={package_name + '.overtaking': ['baseline_parameters/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
    ],
    install_requires=['setuptools', 'numpy', 'numba', 'scipy', 'Pillow', 'PyYAML'],
    zip_safe=False,
    license='Apache-2.0',
)

"""Setuptools/ament_python packaging for map_orientation_saver on ROS 2 Lyrical.

Installs the Python modules, ament resource marker, package metadata, launch
file, parameter YAMLs, license, attribution notice, and README into the ROS
package layout. Console entry points expose the continuous estimator as
map_orientation_node and the Nav2 wrapper as map_saver_with_orientation.

Build through colcon rather than running this file as a robot application:
    colcon build --packages-select map_orientation_saver
    source install/setup.bash

The test extra declares pytest for colcon test discovery. Runtime ROS package
dependencies are declared in package.xml; this file contains Python packaging
requirements and installation rules, not localization or estimator logic.
"""
from glob import glob
from setuptools import find_packages, setup

package_name = 'map_orientation_saver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE', 'NOTICE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Sergei Grichine',
    maintainer_email='slg@quakemap.com',
    description='Estimate geographic map orientation and save hash-bound metadata with Nav2 maps.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'map_orientation_node = map_orientation_saver.map_orientation_node:main',
        'map_saver_with_orientation = map_orientation_saver.map_orientation_save:main',
    ]},
)

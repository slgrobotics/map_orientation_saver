"""
Configure and launch the standalone map-orientation estimator.

This launch file creates map_orientation_node in an optional namespace and
remaps TF to that namespace. use_map_orientation_estimator defaults to true;
false returns without starting the node or reading configuration files.
No robot-model lookup or project-specific launch helper is required.

Parameters are layered in this order: package defaults, an optional external
NavSat YAML (declination converted from radians to degrees), an optional
estimator YAML, then explicit launch overrides. use_sim_time selects only the
clock; it does not imply that a simulator provides a true-ENU heading.

Examples:
    ros2 launch map_orientation_saver map_orientation.launch.py \
        magnetic_declination_degrees:=4.18
    ros2 launch map_orientation_saver map_orientation.launch.py \
        use_sim_time:=true heading_reference:=true_enu
    ros2 launch map_orientation_saver map_orientation.launch.py \
        namespace:=robot1 orientation_config_filepath:=/path/to/orientation.yaml

Other projects may include this launch with their own configuration paths.

When already inside a namespace group, leave namespace empty to avoid applying
it twice.

This launch starts the estimator only; map saving is a separate CLI.
"""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _number(value, name):
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a finite number')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f'{name} must be a finite number')
    return number


def _navsat_corrections(filename):
    """Read an explicitly selected NavSat config; never search robot projects."""
    path = Path(filename).expanduser()
    with path.open() as stream:
        settings = yaml.safe_load(stream)
    if not isinstance(settings, dict):
        raise ValueError('NavSat config must contain a navsat_transform node mapping')
    nodes = [value for name, value in settings.items()
             if str(name).rstrip('/').split('/')[-1] == 'navsat_transform']
    if len(nodes) != 1 or not isinstance(nodes[0], dict):
        raise ValueError('NavSat config must identify exactly one navsat_transform node')
    params = nodes[0].get('ros__parameters', {})
    if not isinstance(params, dict):
        raise ValueError('NavSat ros__parameters must be a mapping')
    corrections = {}
    if 'magnetic_declination_radians' in params:
        corrections['magnetic_declination_degrees'] = math.degrees(
            _number(params['magnetic_declination_radians'], 'magnetic_declination_radians'))
    if 'yaw_offset' in params:
        corrections['yaw_offset'] = _number(params['yaw_offset'], 'yaw_offset')
    return corrections


def _start(context):
    """Resolve configuration precedence and create the optional estimator node.
    """
    if not IfCondition(LaunchConfiguration('use_map_orientation_estimator')).evaluate(context):
        return []

    def argument(name):
        return LaunchConfiguration(name).perform(context)

    defaults = Path(get_package_share_directory('map_orientation_saver')) / 'config/map_orientation.yaml'
    parameters = [str(defaults)]
    if argument('navsat_config_filepath'):
        parameters.append(_navsat_corrections(argument('navsat_config_filepath')))
    if argument('orientation_config_filepath'):
        config = Path(argument('orientation_config_filepath')).expanduser()
        if not config.is_file():
            raise ValueError(f'Orientation config does not exist: {config}')
        parameters.append(str(config))

    overrides = {}
    for name in ('imu_topic', 'map_frame', 'robot_frame', 'heading_reference'):
        if argument(name):
            overrides[name] = argument(name)
    if 'heading_reference' in overrides and overrides['heading_reference'] not in ('magnetic', 'true_enu'):
        raise ValueError('heading_reference must be magnetic or true_enu')
    for name in ('magnetic_declination_degrees', 'yaw_offset'):
        if argument(name):
            overrides[name] = _number(argument(name), name)
    # A simulated clock alone says nothing about the IMU's geographic reference.
    overrides['use_sim_time'] = IfCondition(LaunchConfiguration('use_sim_time')).evaluate(context)
    parameters.append(overrides)
    return [Node(
        package='map_orientation_saver', executable='map_orientation_node',
        name='map_orientation_node', namespace=LaunchConfiguration('namespace'),
        parameters=parameters, output='screen',
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )]


def generate_launch_description():
    """Declare the public launch arguments and defer file/parameter resolution to launch time.
    """
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_map_orientation_estimator', default_value='true'),
        DeclareLaunchArgument('orientation_config_filepath', default_value='',
                              description='Optional estimator YAML overriding package defaults'),
        DeclareLaunchArgument('navsat_config_filepath', default_value='',
                              description='Optional external NavSat YAML; its radians are converted to degrees'),
        DeclareLaunchArgument('imu_topic', default_value=''),
        DeclareLaunchArgument('map_frame', default_value=''),
        DeclareLaunchArgument('robot_frame', default_value=''),
        DeclareLaunchArgument('heading_reference', default_value='',
                              description='magnetic or true_enu; empty uses the YAML setting'),
        DeclareLaunchArgument('magnetic_declination_degrees', default_value=''),
        DeclareLaunchArgument('yaw_offset', default_value='',
                              description='Optional heading-convention offset in radians'),
        OpaqueFunction(function=_start),
    ])

"""Tests for the independent estimator launch and configuration interface.

Loads the launch file without starting nodes, verifies default/disabled
behavior, namespace and frame/topic overrides, parameter precedence, and
conversion of explicitly selected NavSat radians to estimator degrees. It also
checks invalid/ambiguous configurations and the packaged ENU simulation YAML.

A simulation clock intentionally does not select a heading reference by itself;
that distinction is tested so a generic simulator is not silently labeled ENU.
Temporary YAML files and mocked node creation keep these tests independent of
robot projects, hardware, and running localization nodes.

After sourcing ROS 2 Lyrical, run from the repository root:
    python3 -m pytest test/test_launch.py -q
or use `colcon test --packages-select map_orientation_saver`.
"""
import importlib.util
import math
from pathlib import Path
from unittest.mock import Mock, patch

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

spec = importlib.util.spec_from_file_location(
    'map_orientation_launch', Path(__file__).parents[1] / 'launch/map_orientation.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def context():
    ctx = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    return ctx


def command(context):
    with patch.object(module, 'get_package_share_directory', return_value=str(Path(__file__).parents[1])), \
         patch.object(module, 'Node') as node:
        module._start(context)
    return node.call_args.kwargs


def test_defaults_are_independent_and_disabled_launch_reads_no_files(context):
    kwargs = command(context)
    assert kwargs['package'] == 'map_orientation_saver'
    assert kwargs['executable'] == 'map_orientation_node'
    assert kwargs['parameters'][-1] == {'use_sim_time': False}
    context.launch_configurations['use_map_orientation_estimator'] = 'false'
    with patch.object(module, 'get_package_share_directory') as share:
        assert module._start(context) == []
        share.assert_not_called()


def test_clock_does_not_imply_enu_and_heading_can_be_selected_explicitly(context):
    context.launch_configurations['use_sim_time'] = 'true'
    assert command(context)['parameters'][-1] == {'use_sim_time': True}
    context.launch_configurations['heading_reference'] = 'true_enu'
    assert command(context)['parameters'][-1] == {'use_sim_time': True, 'heading_reference': 'true_enu'}


def test_explicit_files_and_overrides_take_precedence(context, tmp_path):
    navsat = tmp_path / 'navsat.yaml'
    navsat.write_text(yaml.safe_dump({'/**/navsat_transform': {'ros__parameters': {
        'magnetic_declination_radians': .073, 'yaw_offset': .1}}}))
    config = tmp_path / 'orientation.yaml'
    config.write_text('/**/map_orientation_node:\n  ros__parameters:\n    magnetic_declination_degrees: 5.0\n')
    context.launch_configurations.update(
        navsat_config_filepath=str(navsat), orientation_config_filepath=str(config),
        magnetic_declination_degrees='6.0', namespace='dragger', imu_topic='sensors/imu',
        map_frame='outdoor_map', robot_frame='base_footprint')
    kwargs = command(context)
    assert kwargs['namespace'].perform(context) == 'dragger'
    assert kwargs['parameters'][1] == dict(magnetic_declination_degrees=math.degrees(.073), yaw_offset=.1)
    assert kwargs['parameters'][2] == str(config)
    assert kwargs['parameters'][3] == dict(magnetic_declination_degrees=6., use_sim_time=False,
        imu_topic='sensors/imu', map_frame='outdoor_map', robot_frame='base_footprint')
    assert kwargs['remappings'] == [('/tf', 'tf'), ('/tf_static', 'tf_static')]


@pytest.mark.parametrize('argument,value', [
    ('heading_reference', 'relative'), ('magnetic_declination_degrees', 'nan'),
    ('magnetic_declination_degrees', 'inf'), ('yaw_offset', 'invalid'),
    ('orientation_config_filepath', '/missing/orientation.yaml'),
])
def test_invalid_configuration_fails_clearly(context, argument, value):
    context.launch_configurations[argument] = value
    with pytest.raises(ValueError):
        command(context)


def test_ambiguous_navsat_config_is_rejected(tmp_path):
    path = tmp_path / 'navsat.yaml'
    path.write_text('robot_a/navsat_transform: {}\nrobot_b/navsat_transform: {}\n')
    with pytest.raises(ValueError, match='exactly one'):
        module._navsat_corrections(str(path))


def test_packaged_simulation_config_selects_true_enu():
    path = Path(__file__).parents[1] / 'config/map_orientation_sim.yaml'
    parameters = yaml.safe_load(path.read_text())['/**/map_orientation_node']['ros__parameters']
    assert parameters == dict(heading_reference='true_enu', magnetic_declination_degrees=0., yaw_offset=0.)

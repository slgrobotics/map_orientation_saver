"""Behavior tests for orientation math, collection, status, and sidecar saving.

Covers quaternion mounting correction, circular averaging, uncertainty and
reset rules, source timestamp validity, delayed-TF retries, and preservation
of a fresh estimate while newer samples wait. Saver tests exercise map hashes,
atomic sidecar replacement, unavailable orientation, failure handling, and
forwarding of Nav2/ROS command arguments.

Tests use synthetic messages, temporary map files, and mocked ROS/process
boundaries. They do not start robot hardware or require a running simulator,
DDS peer, or real magnetic heading source. They do not establish real-world
sensor calibration or accuracy.

After sourcing ROS 2 Lyrical, run from the repository root:
    python3 -m pytest test/test_map_orientation.py -q
or run the installed package's test workflow:
    colcon test --packages-select map_orientation_saver
"""
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
from collections import deque

import pytest
import yaml

from map_orientation_saver.map_orientation_math import OrientationWindow, map_heading, multiply
from map_orientation_saver.map_orientation_save import (
    fresh_estimate, map_fingerprint, nav2_arguments, write_sidecar,
)


def z(degrees):
    a = math.radians(degrees)/2
    return (0.,0.,math.sin(a),math.cos(a))


def estimate():
    return dict(valid=True,reference='true_north',map_frame='map',
                stamp=10.,published_at_ros_ns=10_000_000_000,
                map_x_heading_from_east_deg=30.,uncertainty_deg=2.,
                session_id='session',segment=0)


@pytest.mark.parametrize('heading,map_yaw,expected', [(0,0,0),(90,30,60),(-179,179,2)])
def test_map_geographic_heading(heading,map_yaw,expected):
    assert math.degrees(map_heading(z(heading),z(0),z(map_yaw))) == pytest.approx(expected)


def test_full_imu_mounting_quaternion():
    mounting = multiply((math.sin(.2),0.,0.,math.cos(.2)),z(40))
    inverse = (-mounting[0],-mounting[1],-mounting[2],mounting[3])
    imu = multiply(z(100),inverse)
    assert math.degrees(map_heading(imu,mounting,z(20),math.radians(5))) == pytest.approx(85)


def test_bad_quaternion_is_rejected():
    with pytest.raises(ValueError):
        map_heading((0,0,0,0),z(0),z(0))


def test_circular_average_and_sensor_uncertainty_floor():
    window=OrientationWindow(min_samples=2)
    window.add(10,math.radians(179),math.radians(5)**2)
    window.add(11,math.radians(-179),math.radians(5)**2)
    result=window.summary()
    assert abs(math.degrees(result['heading_rad'])) == pytest.approx(180)
    assert result['valid']
    assert math.degrees(result['uncertainty_rad']) >= 5


def test_window_expires_samples_and_resets_on_jump_gap_and_clock_reversal():
    window=OrientationWindow(window=1,min_samples=2)
    window.add(10,0,0)
    window.add(11,0,0)
    window.add(12,0,0)
    assert window.summary()['sample_count']==2
    for stamp, angle in [(13,1.),(20,1.),(19,1.)]:
        segment=window.segment
        window.add(stamp,angle,0)
        assert window.segment==segment+1
        assert window.summary()['sample_count']==1
        assert not window.summary()['valid']


@pytest.mark.parametrize('changes', [dict(valid=False),dict(reference='magnetic'),
    dict(map_frame='other'),dict(stamp=0),dict(published_at_ros_ns=0),
    dict(uncertainty_deg=float('nan'))])
def test_saver_rejects_unusable_estimates(changes):
    value=estimate();value.update(changes)
    assert not fresh_estimate(value,11_000_000_000,2.,'map')


def test_saver_rejects_future_and_stale_estimates():
    assert fresh_estimate(estimate(),11_000_000_000,2.,'map')
    assert not fresh_estimate(estimate(),9_000_000_000,2.,'map')
    assert not fresh_estimate(estimate(),13_000_000_000,2.,'map')
    assert not fresh_estimate(None,11_000_000_000,2.,'map')


def map_files(tmp_path):
    (tmp_path/'garden.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    path=tmp_path/'garden.yaml'
    path.write_text('image: garden.pgm\nresolution: 0.25\norigin: [-75, -75, 0]\n')
    return path


def test_sidecar_is_bound_to_both_map_files_and_overwrites_previous_metadata(tmp_path):
    path=map_files(tmp_path)
    sidecar=write_sidecar(path,estimate())
    data=yaml.safe_load(sidecar.read_text())
    assert data['orientation']['valid']
    assert data['map_content_hash']==map_fingerprint(path)[0]
    (tmp_path/'garden.pgm').write_bytes(b'changed image')
    assert data['map_content_hash'] != map_fingerprint(path)[0]
    write_sidecar(path,None,'No valid estimate')
    assert yaml.safe_load(sidecar.read_text())['orientation']==dict(valid=False,reason='No valid estimate')
    assert not list(tmp_path.glob('*.tmp'))


def test_failed_map_validation_does_not_overwrite_sidecar(tmp_path):
    path=map_files(tmp_path)
    sidecar=write_sidecar(path,estimate())
    original=sidecar.read_bytes()
    (tmp_path/'garden.pgm').unlink()
    with pytest.raises(FileNotFoundError):
        write_sidecar(path,None)
    assert sidecar.read_bytes()==original


def test_nav2_arguments_preserve_ros_namespace_and_params():
    args=['-f','garden','--orientation-timeout','3','--orientation-topic=x',
          '-t','robot/map','--ros-args','-r','__ns:=/dragger','--params-file','params.yaml']
    assert nav2_arguments(args,'/maps/garden') == [
        '-f','/maps/garden','-t','robot/map','--ros-args','-r','__ns:=/dragger',
        '--params-file','params.yaml']


def test_node_uses_imu_stamp_and_mounting_tf():
    from map_orientation_saver.map_orientation_node import MapOrientationNode
    from rclpy.time import Time
    from rclpy.clock import ClockType
    from builtin_interfaces.msg import Time as TimeMessage
    orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)
    transform=SimpleNamespace(transform=SimpleNamespace(rotation=orientation))
    window=OrientationWindow(min_samples=1)
    node=SimpleNamespace(settings=dict(map_frame='map',robot_frame='base_link',
        max_age=1.,sample_interval=.2,assumed_imu_uncertainty_degrees=15.,
        max_uncertainty_degrees=20.),calibrated=True,recovery=False,last_input=None,
        correction=0.,window=window,reason='',tf=Mock(),pending=deque(),
        last_enqueued=None,pending_tf_reason='',
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.5, clock_type=ClockType.ROS_TIME)))
    node.tf.lookup_transform.return_value=transform
    from sensor_msgs.msg import Imu
    message=Imu()
    message.header.stamp=TimeMessage(sec=10,nanosec=0)
    message.header.frame_id='imu_link'
    message.orientation.w=1.
    MapOrientationNode._try_imu(node,message)
    assert window.summary()['valid']
    assert node.variance_assumed is True
    calls=node.tf.lookup_transform.call_args_list
    assert [(c.args[0],c.args[1],c.args[2].nanoseconds) for c in calls]==[
        ('map','base_link',10_000_000_000),('imu_link','base_link',10_000_000_000)]
    MapOrientationNode._recovery(node,SimpleNamespace(data=True))
    assert node.window.summary() is None


@pytest.mark.parametrize('available,success',[(True,True),(False,True),(True,False)])
def test_cli_sidecar_only_after_successful_save(tmp_path,available,success):
    from map_orientation_saver.map_orientation_save import main
    filename=tmp_path/'garden'
    node=Mock()
    node.get_clock.return_value.now.return_value.nanoseconds=11_000_000_000
    def subscribe(kind,topic,callback,qos):
        if available:
            callback(SimpleNamespace(data=json.dumps(estimate())))
        return Mock()
    node.create_subscription.side_effect=subscribe
    process=Mock();process.poll.return_value=0 if success else 1
    process.returncode=0 if success else 1
    def start(command, **kwargs):
        map_files(tmp_path)
        assert command[:4]==['ros2','run','nav2_map_server','map_saver_cli']
        return process
    with patch('rclpy.init'),patch('rclpy.shutdown'),patch('rclpy.create_node',return_value=node), \
         patch('rclpy.spin_once'),patch('map_orientation_saver.map_orientation_save.subprocess.Popen',side_effect=start):
        if not success:
            with pytest.raises(SystemExit):
                main(['-f',str(filename),'--orientation-timeout','0.001'])
        else:
            main(['-f',str(filename),'--orientation-timeout','0.001'])
    sidecar=tmp_path/'garden.orientation.yaml'
    assert sidecar.exists()==success
    if success:
        assert yaml.safe_load(sidecar.read_text())['orientation']['valid']==available










def test_status_becomes_unavailable_when_inputs_stop():
    from map_orientation_saver.map_orientation_node import MapOrientationNode
    window=OrientationWindow(min_samples=1)
    window.add(10.,math.radians(30),math.radians(2)**2)
    node=SimpleNamespace(window=window,reason='',recovery=False,session_id='s',
        pending=deque(),pending_tf_reason='',
        settings=dict(max_age=1.,map_frame='map',robot_frame='base_link',imu_topic='imu/data',
                      heading_reference='true_enu'),correction=0.,publisher=Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_500_000_000)))
    MapOrientationNode._publish(node)
    result=json.loads(node.publisher.publish.call_args.args[0].data)
    assert result['valid']
    assert result['north_yaw_in_map_deg']==pytest.approx(60)
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=12_000_000_000))
    MapOrientationNode._publish(node)
    result=json.loads(node.publisher.publish.call_args.args[0].data)
    assert not result['valid']
    assert 'stale' in result['reason']





@pytest.mark.parametrize('reference,expected', [('magnetic',math.radians(10)+.1),('true_enu',.1)])
def test_node_converts_declination_degrees_to_internal_radians(reference,expected):
    from map_orientation_saver import map_orientation_node as module
    settings=dict(heading_reference=reference,magnetic_declination_degrees=10.,yaw_offset=.1)
    def declare(name,default):
        settings.setdefault(name,default)
    with patch.object(module.Node,'__init__',return_value=None), \
         patch.object(module.Node,'declare_parameter',side_effect=declare), \
         patch.object(module.Node,'get_parameter',side_effect=lambda name: SimpleNamespace(value=settings[name])), \
         patch.object(module.Node,'create_publisher'), \
         patch.object(module.Node,'create_subscription'), \
         patch.object(module.Node,'create_timer'), \
         patch.object(module,'Buffer'),patch.object(module,'TransformListener'):
        node=module.MapOrientationNode()
    assert node.calibrated
    assert node.correction==pytest.approx(expected)


def queued_node():
    from types import MethodType
    from map_orientation_saver.map_orientation_node import MapOrientationNode
    from rclpy.time import Time
    from rclpy.clock import ClockType
    clock=[10_500_000_000]
    node=SimpleNamespace(settings=dict(map_frame='map',robot_frame='base_link',imu_topic='imu/data',
        heading_reference='true_enu',max_age=1.,sample_interval=.2,
        assumed_imu_uncertainty_degrees=15.,max_uncertainty_degrees=20.),
        calibrated=True,recovery=False,last_input=None,last_enqueued=None,
        correction=0.,window=OrientationWindow(min_samples=1),reason='',tf=Mock(),
        pending=deque(maxlen=200),pending_tf_reason='',session_id='test',publisher=Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=clock[0],clock_type=ClockType.ROS_TIME)))
    node._process_pending=MethodType(MapOrientationNode._process_pending,node)
    node._try_imu=MethodType(MapOrientationNode._try_imu,node)
    return node,clock


def imu_at(seconds):
    from sensor_msgs.msg import Imu
    from rclpy.time import Time
    from rclpy.clock import ClockType
    message=Imu();message.header.frame_id='imu_link'
    message.header.stamp=Time(nanoseconds=round(seconds*1e9),clock_type=ClockType.ROS_TIME).to_msg()
    message.orientation.w=1.
    return message


def identity_transform():
    from geometry_msgs.msg import TransformStamped
    value=TransformStamped();value.transform.rotation.w=1.
    return value


def test_pending_imu_is_retried_at_its_original_stamp_when_tf_arrives():
    from map_orientation_saver.map_orientation_node import MapOrientationNode
    from tf2_ros import TransformException
    node,_=queued_node()
    node.tf.lookup_transform.side_effect=TransformException('future extrapolation')
    MapOrientationNode._imu(node,imu_at(10.2))
    assert len(node.pending)==1
    assert node.window.summary() is None
    node.tf.lookup_transform.side_effect=None
    node.tf.lookup_transform.return_value=identity_transform()
    node._process_pending()
    assert not node.pending
    assert node.window.summary()['valid']
    assert node.last_input==pytest.approx(10.2)
    assert all(call.args[2].nanoseconds==10_200_000_000
               for call in node.tf.lookup_transform.call_args_list)


def test_pending_newer_tf_does_not_invalidate_fresh_accepted_estimate():
    from map_orientation_saver.map_orientation_node import MapOrientationNode
    from tf2_ros import TransformException
    node,_=queued_node()
    node.tf.lookup_transform.return_value=identity_transform()
    MapOrientationNode._imu(node,imu_at(10.))
    node.tf.lookup_transform.side_effect=TransformException('future extrapolation')
    MapOrientationNode._imu(node,imu_at(10.3))
    MapOrientationNode._publish(node)
    result=json.loads(node.publisher.publish.call_args.args[0].data)
    assert result['valid']
    assert result['pending_samples']==1
    assert 'extrapolation' in result['pending_tf_reason']
    assert result['estimate_age_seconds']==pytest.approx(.5)


def test_pending_samples_expire_without_late_or_latest_tf_substitution():
    from map_orientation_saver.map_orientation_node import MapOrientationNode
    from tf2_ros import TransformException
    node,clock=queued_node()
    node.tf.lookup_transform.side_effect=TransformException('future extrapolation')
    MapOrientationNode._imu(node,imu_at(10.2))
    attempts=node.tf.lookup_transform.call_count
    clock[0]=12_000_000_000
    node._process_pending()
    assert not node.pending
    assert node.window.summary() is None
    assert node.tf.lookup_transform.call_count==attempts


def test_recovery_and_clock_reset_discard_pending_samples():
    from map_orientation_saver.map_orientation_node import MapOrientationNode
    from tf2_ros import TransformException
    node,_=queued_node()
    node.tf.lookup_transform.side_effect=TransformException('future extrapolation')
    MapOrientationNode._imu(node,imu_at(10.3))
    MapOrientationNode._imu(node,imu_at(10.1))
    assert len(node.pending)==1
    assert node.pending[0].header.stamp.nanosec==100_000_000
    MapOrientationNode._recovery(node,SimpleNamespace(data=True))
    assert not node.pending
    assert node.last_enqueued is None


@pytest.mark.parametrize('interrupted', [False, True, 'external'])
def test_estimator_exits_quietly_and_cleans_up_on_normal_shutdown(interrupted):
    from map_orientation_saver import map_orientation_node as module
    node = Mock()
    error = (KeyboardInterrupt() if interrupted is True else
             module.ExternalShutdownException() if interrupted == 'external' else None)
    with patch.object(module.rclpy, 'init'), \
         patch.object(module, 'MapOrientationNode', return_value=node), \
         patch.object(module.rclpy, 'spin', side_effect=error), \
         patch.object(module.rclpy, 'try_shutdown') as shutdown:
        module.main()
    node.destroy_node.assert_called_once()
    shutdown.assert_called_once()


def test_estimator_does_not_hide_unexpected_errors():
    from map_orientation_saver import map_orientation_node as module
    node = Mock()
    with patch.object(module.rclpy, 'init'), \
         patch.object(module, 'MapOrientationNode', return_value=node), \
         patch.object(module.rclpy, 'spin', side_effect=RuntimeError('unexpected')), \
         patch.object(module.rclpy, 'try_shutdown') as shutdown:
        with pytest.raises(RuntimeError, match='unexpected'):
            module.main()
    node.destroy_node.assert_called_once()
    shutdown.assert_called_once()


def test_estimator_releases_context_if_initialization_fails():
    from map_orientation_saver import map_orientation_node as module
    with patch.object(module.rclpy, 'init'), \
         patch.object(module, 'MapOrientationNode', side_effect=ValueError('bad parameters')), \
         patch.object(module.rclpy, 'try_shutdown') as shutdown:
        with pytest.raises(ValueError, match='bad parameters'):
            module.main()
    shutdown.assert_called_once()

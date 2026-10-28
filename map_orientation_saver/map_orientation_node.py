"""
Continuously estimate a map frame's geographic orientation from IMU and TF.

The node subscribes to an absolute ENU IMU orientation (default: imu/data),
compares it with map-to-robot TF at the same source timestamp, and corrects
for sensor mounting using TF. A bounded queue retries briefly delayed TF
without substituting a latest-time transform. Quality checks and a circular
window reject stale data and reset estimates after discontinuities or recovery.

Status is published once per second as JSON in std_msgs/String on the relative
localization/map_orientation topic. It includes validity/reason, map heading,
North direction, uncertainty, source time, and pending-TF diagnostics. The
optional localization/recovery_active Bool input pauses/resets estimation.
This node does not publish TF, control localization, or write map files.

Typical launch for magnetic heading (replace the example declination):
    ros2 launch map_orientation_saver map_orientation.launch.py \
        magnetic_declination_degrees:=4.18

For a simulator supplying true-ENU IMU orientation:
    ros2 launch map_orientation_saver map_orientation.launch.py \
        use_sim_time:=true heading_reference:=true_enu

Direct executable:
    ros2 run map_orientation_saver map_orientation_node`.

Without a true-heading reference or configured magnetic declination, the node
stays alive and explicitly reports orientation unavailable.
"""

from collections import deque
import json
import math
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException

from map_orientation_saver.map_orientation_math import OrientationWindow, map_heading


def q(message):
    """Extract a ROS quaternion message into the math library's (x, y, z, w) tuple.
    """
    return (message.x, message.y, message.z, message.w)


class MapOrientationNode(Node):
    """Collect time-aligned IMU/TF pairs and publish a geographic map heading.

    Startup parameters control the heading reference, correction, frames,
    sampling window, freshness, and quality limits. The node owns no map
    files and publishes unavailable status until its inputs are trustworthy.
    """
    def __init__(self):
        super().__init__('map_orientation_node')
        defaults = dict(imu_topic='imu/data', map_frame='map', robot_frame='base_link',
                        heading_reference='magnetic', magnetic_declination_degrees=float('nan'),
                        yaw_offset=0., max_age=1., sample_interval=.2, window_seconds=30.,
                        min_samples=20, reset_jump_degrees=25., max_uncertainty_degrees=20.,
                        assumed_imu_uncertainty_degrees=15.)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {k:self.get_parameter(k).value for k in defaults}
        p = self.settings
        if p['heading_reference'] not in ('magnetic','true_enu'):
            raise ValueError('heading_reference must be magnetic or true_enu (ENU convention)')
        for name in ('max_age','sample_interval','window_seconds','min_samples',
                     'reset_jump_degrees','max_uncertainty_degrees',
                     'assumed_imu_uncertainty_degrees'):
            if not math.isfinite(p[name]) or p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not math.isfinite(p['yaw_offset']):
            raise ValueError('yaw_offset must be finite')
        self.correction = p['yaw_offset']
        self.calibrated = p['heading_reference'] == 'true_enu'
        if not self.calibrated and math.isfinite(p['magnetic_declination_degrees']):
            self.correction += math.radians(p['magnetic_declination_degrees'])
            self.calibrated = True
        self.window = OrientationWindow(
            window=p['window_seconds'], min_samples=p['min_samples'],
            max_gap=max(3., p['max_age']), jump=math.radians(p['reset_jump_degrees']),
            max_uncertainty=math.radians(p['max_uncertainty_degrees']))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.session_id = str(uuid.uuid4())
        self.recovery = False
        self.last_input = None
        self.last_enqueued = None
        self.pending = deque(maxlen=200)
        self.pending_tf_reason = ''
        self.reason = 'Waiting for IMU and TF' if self.calibrated else 'True-heading correction not configured'
        self.publisher = self.create_publisher(String, 'localization/map_orientation',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Imu, p['imu_topic'], self._imu, qos_profile_sensor_data)
        self.create_subscription(Bool, 'localization/recovery_active', self._recovery,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05, self._process_pending)
        self.create_timer(1., self._publish)

    def _recovery(self, message):
        if message.data != self.recovery:
            self.window.reset()
            self.last_input = None
            self.last_enqueued = None
            self.pending.clear()
            self.pending_tf_reason = ''
        self.recovery = message.data
        if self.recovery:
            self.reason = 'Localization recovery active'

    def _imu(self, message):
        """Validate source time, decimate incoming IMU data, and queue it for TF pairing.
        """
        if not self.calibrated or self.recovery:
            return
        stamp = Time.from_msg(message.header.stamp)
        seconds = stamp.nanoseconds/1e9
        age = (self.get_clock().now()-stamp).nanoseconds/1e9
        if stamp.nanoseconds <= 0 or not 0 <= age <= self.settings['max_age']:
            self.reason = 'IMU timestamp missing, future, or stale'
            return
        if self.last_enqueued is not None:
            if seconds < self.last_enqueued:
                self.window.reset()
                self.pending.clear()
                self.last_input = None
                self.pending_tf_reason = ''
            elif seconds-self.last_enqueued < self.settings['sample_interval']:
                return
        self.last_enqueued = seconds
        self.pending.append(message)
        self._process_pending()

    def _process_pending(self):
        # Never block the executor waiting for TF: its callbacks must run to
        # fill the buffer. Retry the oldest retained source stamp on a timer.
        """Retry queued samples without blocking TF subscription callbacks.

        Process oldest-first, retaining a sample only while its exact timestamp
        may still become available within max_age. Never request latest-time TF.
        """
        if self.recovery or not self.calibrated:
            return
        while self.pending:
            if not self._try_imu(self.pending[0]):
                return
            self.pending.popleft()
        self.pending_tf_reason = ''

    def _try_imu(self, message):
        """Attempt one exact-time IMU/TF observation.

        Return False only when TF is unavailable and the queued sample should be
        retried. Return True after accepting or rejecting a sample so the queue
        can advance. A pending TF lookup does not invalidate a fresh estimate.
        """
        p = self.settings
        if not self.calibrated or self.recovery:
            return True
        stamp = Time.from_msg(message.header.stamp)
        seconds = stamp.nanoseconds / 1e9
        age = (self.get_clock().now()-stamp).nanoseconds/1e9
        if stamp.nanoseconds <= 0 or not 0 <= age <= p['max_age']:
            self.reason = 'IMU timestamp missing, future, or stale'
            return True
        if self.last_input is not None:
            if seconds < self.last_input:
                self.window.reset()
            elif seconds-self.last_input < p['sample_interval']:
                return True
        covariance = message.orientation_covariance
        if not message.header.frame_id or covariance[0] < 0:
            self.reason = 'IMU orientation or frame unavailable'
            return True
        variance = covariance[8]
        if not math.isfinite(variance) or variance < 0:
            self.reason = 'Invalid IMU yaw covariance'
            return True
        self.variance_assumed = bool(variance == 0)
        if self.variance_assumed:
            variance = math.radians(p['assumed_imu_uncertainty_degrees'])**2
        if variance > math.radians(p['max_uncertainty_degrees'])**2:
            self.reason = 'IMU heading uncertainty too large'
            return True
        try:
            pose = self.tf.lookup_transform(p['map_frame'],p['robot_frame'],stamp)
            mounting = self.tf.lookup_transform(message.header.frame_id,p['robot_frame'],stamp)
            angle = map_heading(q(message.orientation), q(mounting.transform.rotation),
                                q(pose.transform.rotation), self.correction)
            self.window.add(seconds, angle, variance)
        except TransformException as error:
            self.pending_tf_reason = f'Waiting for time-aligned TF: {error}'
            if self.last_input is None:
                self.reason = self.pending_tf_reason
            return False
        except ValueError as error:
            self.reason = f'Invalid orientation: {error}'
            return True
        self.last_input = seconds
        self.reason = ''
        self.pending_tf_reason = ''
        return True

    def _publish(self):
        """Publish current validity, geographic orientation, uncertainty, and queue diagnostics.
        """
        estimate = self.window.summary()
        reason = self.reason
        now = self.get_clock().now().nanoseconds/1e9
        valid = bool(estimate and estimate['valid'] and not reason and not self.recovery)
        if estimate and not 0 <= now-estimate['stamp'] <= self.settings['max_age']:
            valid = False
            reason = 'Orientation estimate stale'
            if self.pending_tf_reason:
                reason += '; '+self.pending_tf_reason
        if not valid and not reason:
            reason = 'Collecting samples or heading dispersion too large'
        payload = dict(schema_version=1, valid=valid, reason=reason,
                       session_id=self.session_id, map_frame=self.settings['map_frame'],
                       robot_frame=self.settings['robot_frame'], reference='true_north',
                       source='imu_and_tf', published_at_ros_ns=self.get_clock().now().nanoseconds,
                       imu_topic=self.settings['imu_topic'],
                       pending_samples=len(self.pending),
                       pending_tf_reason=self.pending_tf_reason,
                       heading_reference=self.settings['heading_reference'],
                       correction_radians=self.correction,
                       imu_uncertainty_assumed=getattr(self,'variance_assumed',False))
        if estimate:
            payload.update({key:value for key,value in estimate.items() if key != 'valid'})
            payload['estimate_age_seconds'] = now-estimate['stamp']
            payload['map_x_heading_from_east_deg'] = math.degrees(estimate['heading_rad'])
            payload['north_yaw_in_map_deg'] = math.degrees(math.pi/2-estimate['heading_rad']) % 360
            payload['uncertainty_deg'] = math.degrees(estimate['uncertainty_rad'])
        self.publisher.publish(String(data=json.dumps(payload, allow_nan=False)))


def main(args=None):
    """Initialize ROS, run the estimator executor, and release node/context resources.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = MapOrientationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C and a context stopped by ROS launch are normal shutdown paths.
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            # ROS signal handling may already have shut down this context.
            rclpy.try_shutdown()

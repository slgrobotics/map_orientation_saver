"""
Quaternion frame math and a bounded circular map-orientation estimate.

This ROS-independent module relates an absolute IMU orientation to robot yaw
in the map frame, including the IMU-to-robot mounting transform. Quaternions
use (x, y, z, w) order; angles are radians, positive counterclockwise from East
when expressed in true ENU. Circular statistics avoid the discontinuity at
-180/+180 degrees and retain a conservative sensor-uncertainty contribution.

OrientationWindow tracks a recent sampling window and starts a new segment
on a large heading jump, an input gap, or backward source time. The ROS node
handles timestamp/TF availability and decides when to publish its result.

This is a library, not a launchable node. It is imported by
map_orientation_saver.map_orientation_node and exercised by
`python3 -m pytest test/test_map_orientation.py` from the repository root.
"""

from collections import deque
import math


def wrap(angle):
    """Normalize an angle in radians to the signed -pi..pi circular range.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion(values):
    """Validate and normalize a quaternion supplied in (x, y, z, w) order.
    """
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite quaternion')
    norm = math.sqrt(sum(v*v for v in values))
    if norm < 1e-8:
        raise ValueError('Empty quaternion')
    return tuple(v / norm for v in values)


def multiply(a, b):
    """Compose two quaternions and normalize the resulting frame rotation.
    """
    x,y,z,w = a
    X,Y,Z,W = b
    return quaternion((w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                       w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z))


def yaw(q):
    """Extract yaw in radians from a normalized (x, y, z, w) orientation.
    """
    x,y,z,w = quaternion(q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def map_heading(world_from_imu, imu_from_base, map_from_base, correction=0.):
    """Return map +X yaw in true ENU, counterclockwise from East.

    Compose world-from-IMU with IMU-from-base before extracting heading, then
    apply the geographic correction and subtract the robot's map-frame yaw.
    The caller must provide synchronized transforms and correction in radians.
    """
    return wrap(yaw(multiply(world_from_imu, imu_from_base)) + correction
                - yaw(map_from_base))


class OrientationWindow:
    """Maintain a bounded source-time window of circular heading samples.

    Sample variances represent input heading uncertainty, in radians squared.
    Sensor uncertainty is not reduced by sqrt(sample_count), since repeated
    measurements can be correlated. Segment numbers identify discontinuities.
    """
    def __init__(self, window=30., min_samples=20, max_gap=3., jump=math.radians(25),
                 max_uncertainty=math.radians(20)):
        self.window, self.min_samples, self.max_gap = window, min_samples, max_gap
        self.jump, self.max_uncertainty = jump, max_uncertainty
        self.samples = deque()
        self.segment = 0

    def reset(self):
        """Discard accumulated samples and advance the discontinuity segment.
        """
        self.samples.clear()
        self.segment += 1

    def add(self, stamp, angle, variance):
        """Accept one heading sample and evict samples outside the time window.

        Reset the window on non-increasing source time, a long gap, or a large
        change from the current circular mean. Inputs must be finite; variance
        must be nonnegative.
        """
        angle = wrap(angle)
        if not all(math.isfinite(v) for v in (stamp, angle, variance)) or variance < 0:
            raise ValueError('Invalid orientation sample')
        if self.samples:
            last = self.samples[-1]
            if stamp <= last[0] or stamp-last[0] > self.max_gap:
                self.reset()
            elif abs(wrap(angle-self.summary()['heading_rad'])) > self.jump:
                self.reset()
        self.samples.append((stamp, angle, variance))
        while self.samples and stamp-self.samples[0][0] > self.window:
            self.samples.popleft()

    def summary(self):
        """Return the circular mean, uncertainty, source time, and validity.

        Return None before any samples exist. This checks count and uncertainty;
        the ROS node separately checks freshness against its current clock.
        """
        if not self.samples:
            return None
        n = len(self.samples)
        c = sum(math.cos(v[1]) for v in self.samples)/n
        s = sum(math.sin(v[1]) for v in self.samples)/n
        dispersion = math.sqrt(-2*math.log(min(1., max(1e-15, math.hypot(c,s)))))
        # Do not divide sensor uncertainty by sqrt(n): samples are correlated.
        uncertainty = math.sqrt(dispersion**2 + sum(v[2] for v in self.samples)/n)
        return dict(heading_rad=math.atan2(s,c), uncertainty_rad=uncertainty,
                    dispersion_rad=dispersion, sample_count=n,
                    stamp=self.samples[-1][0], segment=self.segment,
                    valid=n >= self.min_samples and uncertainty <= self.max_uncertainty)

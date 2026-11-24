"""Direction decisions without ROS dependencies."""

import math


class DirectionPolicy:
    def __init__(self, enabled=False, timeout=0.5, threshold_deg=90.0,
                 stop_band_deg=0.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('joint_state_timeout must be finite and positive')
        if not math.isfinite(threshold_deg) or not 0 < threshold_deg < 180:
            raise ValueError('reverse_threshold_deg must be between 0 and 180')
        if (not math.isfinite(stop_band_deg) or stop_band_deg < 0
                or stop_band_deg >= min(threshold_deg, 180 - threshold_deg)):
            raise ValueError('stop_band_deg must fit on both sides of the threshold')
        self.enabled = enabled
        self.timeout = timeout
        self.threshold = math.radians(threshold_deg)
        self.stop_band = math.radians(stop_band_deg)
        self.angle = None
        self.received = None
        self.stamp = None

    def update(self, angle, received, stamp):
        if angle is None or not all(math.isfinite(v) for v in (angle, received, stamp)):
            self.angle = self.received = self.stamp = None
            return
        self.angle = math.atan2(math.sin(angle), math.cos(angle))
        self.received = received
        self.stamp = stamp

    def state(self, monotonic_now, ros_now):
        if not self.enabled:
            return 'passthrough'
        if self.angle is None:
            return 'waiting_for_angle'
        # Check both reception age and source timestamp. Clock resets fail closed.
        if not (0 <= monotonic_now - self.received <= self.timeout
                and 0 <= ros_now - self.stamp <= self.timeout):
            return 'stale_angle'
        distance = abs(self.angle) - self.threshold
        if abs(distance) <= self.stop_band + 1e-12:
            return 'boundary_stop'
        return 'reverse' if distance > 0 else 'forward'

"""
State Estimation Primitives
Exports: DerivativeEstimator, ComplementaryFilter, LowPassFilter

These are used by main.py to build per-axis velocity and sway estimators.
The StateEstimator class is retained for backward compatibility / sim mode.
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class SensorReadings:
    accel_x: float = 0.0; accel_y: float = 0.0; accel_z: float = 0.0
    gyro_x: float = 0.0;  gyro_y: float = 0.0;  gyro_z: float = 0.0
    position: float = 0.0; timestamp: float = 0.0


class LowPassFilter:
    def __init__(self, cutoff_hz: float, dt: float):
        tau = 1.0 / (2 * np.pi * cutoff_hz)
        self.alpha = dt / (tau + dt)
        self._y = 0.0; self._init = False

    def reset(self, v=0.0):
        self._y = v; self._init = True

    def update(self, x: float) -> float:
        if not self._init:
            self._y = x; self._init = True
        else:
            self._y = (1 - self.alpha) * self._y + self.alpha * x
        return self._y

    @property
    def value(self): return self._y


class DerivativeEstimator:
    def __init__(self, dt: float, filter_cutoff_hz: Optional[float] = None):
        self.dt = dt
        self._last = 0.0; self._init = False
        self._filt = LowPassFilter(filter_cutoff_hz, dt) if filter_cutoff_hz else None

    def reset(self):
        self._last = 0.0; self._init = False
        if self._filt: self._filt.reset()

    def update(self, value: float) -> float:
        if not self._init:
            self._last = value; self._init = True; return 0.0
        d = (value - self._last) / self.dt
        self._last = value
        if self._filt: d = self._filt.update(d)
        return d


class ComplementaryFilter:
    """
    theta = alpha*(theta_prev + gyro*dt) + (1-alpha)*theta_accel
    Higher alpha trusts gyro more (rejects accel bias during motion).
    """
    def __init__(self, alpha: float = 0.98, dt: float = 0.005):
        self.alpha = alpha; self.dt = dt; self._theta = 0.0

    def reset(self, theta=0.0):
        self._theta = theta

    def update(self, gyro_rate: float, accel_angle: float) -> float:
        tg = self._theta + gyro_rate * self.dt
        self._theta = self.alpha * tg + (1 - self.alpha) * accel_angle
        return self._theta

    @property
    def angle(self): return self._theta


class StateEstimator:
    """Single-axis estimator — retained for sim / backward compat."""
    def __init__(self, dt, imu_cutoff_hz=10.0, velocity_cutoff_hz=20.0,
                 use_complementary=True, complementary_alpha=0.98,
                 theta_deadband_deg=0.40):
        self.dt = dt
        self.theta_deadband_rad = np.radians(theta_deadband_deg)
        self.theta_filter = LowPassFilter(imu_cutoff_hz, dt)
        self.theta_dot_filter = LowPassFilter(imu_cutoff_hz, dt)
        self.velocity_estimator = DerivativeEstimator(dt, velocity_cutoff_hz)
        self.use_complementary = use_complementary
        self.comp_filter = ComplementaryFilter(complementary_alpha, dt) if use_complementary else None
        self._position = 0.0; self._velocity = 0.0
        self._theta = 0.0; self._theta_dot = 0.0
        self.theta_unbanded = 0.0; self.theta_dot_unbanded = 0.0

    def reset(self):
        self.theta_filter.reset(); self.theta_dot_filter.reset()
        self.velocity_estimator.reset()
        if self.comp_filter: self.comp_filter.reset()
        self._position = 0.0; self._velocity = 0.0
        self._theta = 0.0; self._theta_dot = 0.0

    def update(self, readings: SensorReadings) -> Tuple[float, float, float]:
        self._position = readings.position
        self._velocity = self.velocity_estimator.update(readings.position)
        theta_accel = np.arctan2(readings.accel_z, readings.accel_y)
        theta_dot_raw = readings.gyro_x
        if self.use_complementary and self.comp_filter:
            self._theta = self.comp_filter.update(theta_dot_raw, theta_accel)
        else:
            self._theta = self.theta_filter.update(theta_accel)
        self._theta_dot = self.theta_dot_filter.update(theta_dot_raw)
        self.theta_unbanded = self._theta
        self.theta_dot_unbanded = self._theta_dot
        if abs(self._theta) < self.theta_deadband_rad:
            self._theta = 0.0; self._theta_dot = 0.0
        return self._velocity, self._theta, self._theta_dot

    @property
    def position(self): return self._position
    @property
    def velocity(self): return self._velocity
    @property
    def theta(self): return self._theta
    @property
    def theta_dot(self): return self._theta_dot
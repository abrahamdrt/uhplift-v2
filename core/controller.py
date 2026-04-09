"""
LQI Controller — State feedback with integral action
Directional anti-windup prevents permanent integrator lock.
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class ControllerState:
    integrator: float = 0.0
    last_error: float = 0.0
    last_output: float = 0.0
    saturated: bool = False


class LQIController:
    """
    LQI (LQR + Integral) state-feedback controller.

    u = -(K_vel*v + K_theta*theta + K_theta_dot*theta_dot) - K_int*integral(v_err)

    Directional anti-windup: when saturated, integration is allowed
    ONLY if it would reduce the saturation magnitude, preventing
    permanent lock that blocks operator recovery.
    """

    def __init__(self, gains, f_max: float = 13.0,
                 anti_windup: bool = True,
                 integrator_limit: Optional[float] = None):
        self.K = gains.as_array()
        self.f_max = f_max
        self.anti_windup = anti_windup
        self.integrator_limit = integrator_limit
        self.state = ControllerState()

    def set_gains(self, gains):
        """Update gains for gain scheduling (e.g., hoist length change)."""
        self.K = gains.as_array()

    def reset(self):
        self.state = ControllerState()

    def compute(self, v: float, theta: float, theta_dot: float,
                v_ref: float, dt: float) -> Tuple[float, dict]:

        v_error = v_ref - v

        # State feedback
        u_fb = -(self.K[0]*v + self.K[1]*theta + self.K[2]*theta_dot)

        # Integral term
        u_int = -self.K[3] * self.state.integrator

        u_total = u_fb + u_int
        u_sat = float(np.clip(u_total, -self.f_max, self.f_max))
        is_sat = abs(u_total) > self.f_max

        # Directional anti-windup
        if self.anti_windup and is_sat:
            would_reduce = (u_total > 0 and v_error < 0) or \
                           (u_total < 0 and v_error > 0)
            if would_reduce:
                self.state.integrator += v_error * dt
        else:
            self.state.integrator += v_error * dt

        if self.integrator_limit is not None:
            self.state.integrator = float(np.clip(
                self.state.integrator, -self.integrator_limit, self.integrator_limit))

        self.state.last_error = v_error
        self.state.last_output = u_sat
        self.state.saturated = is_sat

        info = {
            'u_feedback': u_fb, 'u_integral': u_int,
            'u_total': u_total, 'u_saturated': u_sat,
            'saturated': is_sat, 'integrator': self.state.integrator,
            'v_error': v_error,
        }
        return u_sat, info

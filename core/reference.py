"""
Reference Trajectory Generator
Generates trapezoidal velocity profiles and handles manual mode input

Profile structure:
    1. Idle (v=0) until t_start
    2. Ramp up at specified acceleration
    3. Cruise at v_target
    4. Ramp down at specified acceleration
    5. Idle (v=0) after stop

ManualModeGenerator:
    Converts joystick input [-1, 1] to smooth velocity commands
    with rate limiting (0.20g ramp limit from CDR)
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

from .config import G_IN_PER_S2


@dataclass
class TrapezoidalProfile:
    """Parameters defining a trapezoidal velocity profile"""
    v_target: float         # Target cruise velocity [in/s]
    accel_g: float          # Acceleration [g]
    t_start: float          # Time to start motion [s]
    cruise_duration: float  # Duration at cruise velocity [s]
    
    @property
    def accel(self) -> float:
        """Acceleration in [in/s²]"""
        return self.accel_g * G_IN_PER_S2
    
    @property
    def ramp_duration(self) -> float:
        """Duration of ramp up/down [s]"""
        return self.v_target / self.accel
    
    @property
    def t_cruise_start(self) -> float:
        """Time when cruise phase begins [s]"""
        return self.t_start + self.ramp_duration
    
    @property
    def t_cruise_end(self) -> float:
        """Time when cruise phase ends [s]"""
        return self.t_start + self.ramp_duration + self.cruise_duration
    
    @property
    def t_stop(self) -> float:
        """Time when motion stops [s]"""
        return self.t_cruise_end + self.ramp_duration
    
    @property
    def total_distance(self) -> float:
        """Total distance traveled [in]"""
        ramp_dist = self.v_target * self.ramp_duration
        cruise_dist = self.v_target * self.cruise_duration
        return ramp_dist + cruise_dist


class ReferenceGenerator:
    """
    Generates velocity reference signals for crane control.
    
    Supports:
    - Trapezoidal velocity profiles
    - Real-time profile generation
    """
    
    def __init__(self, profile: Optional[TrapezoidalProfile] = None):
        """
        Initialize reference generator.
        
        Args:
            profile: Initial profile (can be set later)
        """
        self.profile = profile
        self._t = 0.0
        self._active = False
    
    def set_profile(self, profile: TrapezoidalProfile):
        """Set new profile and reset time"""
        self.profile = profile
        self._t = 0.0
    
    def start(self):
        """Start profile execution"""
        self._active = True
        self._t = 0.0
    
    def stop(self):
        """Stop and reset"""
        self._active = False
        self._t = 0.0
    
    def get_reference(self, t: Optional[float] = None) -> Tuple[float, float]:
        """
        Get reference velocity and position at time t.
        
        Args:
            t: Time [s]. If None, uses internal time.
            
        Returns:
            Tuple of (v_ref [in/s], x_ref [in])
        """
        if self.profile is None:
            return 0.0, 0.0
        
        if t is None:
            t = self._t
        
        p = self.profile
        
        # Phase 1: Before start - idle
        if t < p.t_start:
            return 0.0, 0.0
        
        # Phase 2: Ramp up
        elif t < p.t_cruise_start:
            dt = t - p.t_start
            v = p.accel * dt
            x = 0.5 * p.accel * dt**2
            return v, x
        
        # Phase 3: Cruise
        elif t < p.t_cruise_end:
            dt_ramp = p.ramp_duration
            dt_cruise = t - p.t_cruise_start
            v = p.v_target
            x = 0.5 * p.accel * dt_ramp**2 + p.v_target * dt_cruise
            return v, x
        
        # Phase 4: Ramp down
        elif t < p.t_stop:
            dt_ramp = p.ramp_duration
            dt_down = t - p.t_cruise_end
            v = p.v_target - p.accel * dt_down
            x_ramp_up = 0.5 * p.accel * dt_ramp**2
            x_cruise = p.v_target * p.cruise_duration
            x_ramp_down = p.v_target * dt_down - 0.5 * p.accel * dt_down**2
            return v, x_ramp_up + x_cruise + x_ramp_down
        
        # Phase 5: After stop - idle
        else:
            return 0.0, p.total_distance
    
    def step(self, dt: float) -> Tuple[float, float]:
        """
        Advance internal time and return reference.
        
        Args:
            dt: Timestep [s]
            
        Returns:
            Tuple of (v_ref [in/s], x_ref [in])
        """
        v_ref, x_ref = self.get_reference(self._t)
        self._t += dt
        return v_ref, x_ref
    
    def is_complete(self) -> bool:
        """Check if profile execution is complete"""
        if self.profile is None:
            return True
        return self._t >= self.profile.t_stop
    
    @property
    def elapsed_time(self) -> float:
        """Current elapsed time [s]"""
        return self._t


class ManualModeGenerator:
    """
    Reference generator for manual (joystick) mode.
    
    Converts joystick input to smooth velocity commands
    with rate limiting for safety.
    
    This is the primary interface for USB game controller HMI.
    """
    
    def __init__(self,
                 v_max: float,
                 accel_max_g: float = 0.20):
        """
        Initialize manual mode generator.
        
        Args:
            v_max: Maximum velocity [in/s]
            accel_max_g: Maximum acceleration [g] (default 0.20 from CDR)
        """
        self.v_max = v_max
        self.accel_max = accel_max_g * G_IN_PER_S2
        self._v_current = 0.0
    
    def update(self, joystick_input: float, dt: float) -> float:
        """
        Update velocity reference based on joystick input.
        
        Args:
            joystick_input: Normalized input [-1, 1]
            dt: Timestep [s]
            
        Returns:
            Smoothed velocity reference [in/s]
        """
        # Target velocity from joystick
        v_target = joystick_input * self.v_max
        
        # Rate limit
        v_error = v_target - self._v_current
        max_change = self.accel_max * dt
        
        if abs(v_error) <= max_change:
            self._v_current = v_target
        else:
            self._v_current += np.sign(v_error) * max_change
        
        return self._v_current
    
    def reset(self):
        """Reset to zero velocity"""
        self._v_current = 0.0
    
    @property
    def current_velocity(self) -> float:
        """Current velocity reference [in/s]"""
        return self._v_current


def generate_profile_array(profile: TrapezoidalProfile,
                           dt: float,
                           t_final: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Pre-generate entire profile as arrays (for simulation).
    
    Args:
        profile: Profile parameters
        dt: Sample time [s]
        t_final: End time [s]
        
    Returns:
        Tuple of (t_array, v_ref_array, x_ref_array)
    """
    gen = ReferenceGenerator(profile)
    
    t = np.arange(0, t_final + dt, dt)
    v_ref = np.zeros_like(t)
    x_ref = np.zeros_like(t)
    
    for i, ti in enumerate(t):
        v_ref[i], x_ref[i] = gen.get_reference(ti)
    
    return t, v_ref, x_ref


def create_default_trolley_profile() -> TrapezoidalProfile:
    """Create default trolley profile matching MATLAB sims"""
    return TrapezoidalProfile(
        v_target=4.9,
        accel_g=0.20,
        t_start=1.0,
        cruise_duration=4.0
    )


def create_default_bridge_profile() -> TrapezoidalProfile:
    """Create default bridge profile matching MATLAB sims"""
    return TrapezoidalProfile(
        v_target=8.3,
        accel_g=0.20,
        t_start=1.0,
        cruise_duration=4.0
    )
"""
UHplift Crane Control System — Final Configuration
Team 11 Capstone II | University of Houston | Spring 2026

Coordinate System (immutable, matches physical frame):
    X-axis: Bridge  — North(-)→South(+), sway θx in XY plane
    Z-axis: Trolley — West(-)→East(+),  sway θz in ZY plane
    Y-axis: Hoist   — Up(+)/Down(-), cable length L increases downward

Units: lbm, inches, seconds
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Tuple, Optional

G_IN_PER_S2 = 386.09
G_C = 386.09


@dataclass
class AxisConfig:
    name: str
    m_t: float          # Moving mass [lbm]
    b_x: float          # Linear damping [lbf·s/in]
    v_target: float     # Reference velocity [in/s]
    f_max: float        # Force saturation [lbf]
    gear_ratio: float = 5.0
    wheel_diameter: float = 0.97
    steps_per_rev: float = 200.0
    microstepping: int = 4

    @property
    def pulses_per_rev(self) -> int:
        return int(self.steps_per_rev * self.microstepping)


@dataclass
class SystemConfig:
    # Load parameters (variable — updated by hoist position)
    m_l: float = 39.09          # Load mass [lbm]
    L: float = 41.138           # Cable length [in]
    c_theta: float = 0.05       # Rotational damping [lbf·in·s]

    trolley: AxisConfig = field(default_factory=lambda: AxisConfig(
        name="trolley",
        m_t=8.85,            # Trolley carriage mass [lbm] final trolley mass
        b_x=0.10,
        v_target=4.6,       # 17HS19-2004ME1K, 5:1 gear
        f_max=26.0,
        gear_ratio=5.0,
        wheel_diameter=0.97,
        microstepping=4,
    ))

    bridge: AxisConfig = field(default_factory=lambda: AxisConfig(
        name="bridge",
        m_t=17.9,            # Trolley + bridge combined [lbm]
        b_x=0.20,
        v_target=8.3,        # 17HS15-1504ME1K, 4:1 gear, dual motor
        f_max=27.36,         # 2x single motor limit
        gear_ratio=4.0,
        wheel_diameter=0.97,
        microstepping=4,
    ))

    # Control constraints
    a_slip_limit: float = 0.268
    sway_limit_deg: float = 2.0
    ss_error_limit: float = 1.0

    # Timing
    control_rate_hz: float = 200.0
    imu_filter_cutoff_hz: float = 10.0
    velocity_filter_cutoff_hz: float = 20.0

    # Safety limits
    theta_emergency_deg: float = 180.0
    position_limit_trolley: float = 48.0
    position_limit_bridge: float = 48.0
    position_limit_hoist_min: float = 12.0
    position_limit_hoist_max: float = 41.138

    @property
    def dt(self) -> float:
        return 1.0 / self.control_rate_hz

    @property
    def omega_n(self) -> float:
        return np.sqrt(G_IN_PER_S2 / self.L)

    @property
    def period(self) -> float:
        return 2 * np.pi / self.omega_n


# ── LQR Gain Sets ────────────────────────────────────────────────────────────

@dataclass
class GainSet:
    K_vel: float
    K_theta: float
    K_theta_dot: float
    K_int: float
    m_l: float
    L: float

    def as_array(self) -> np.ndarray:
        return np.array([self.K_vel, self.K_theta, self.K_theta_dot, self.K_int])

# Trolley System Gains (m_t = 8.85 lbm)
TROLLEY_GAINS = {
    (12.0, 12.0): GainSet(K_vel=0.416508, K_theta=-4.408728, K_theta_dot=0.833212, K_int=-0.868932, m_l=12.0, L=12.0),
    (12.0, 21.6): GainSet(K_vel=0.362439, K_theta=-5.309683, K_theta_dot=1.117304, K_int=-0.527999, m_l=12.0, L=21.6),
    (12.0, 31.3): GainSet(K_vel=1.266344, K_theta=-55.396850, K_theta_dot=5.548017, K_int=-1.430010, m_l=12.0, L=31.3),
    (12.0, 41.0): GainSet(K_vel=1.235986, K_theta=-72.782729, K_theta_dot=10.057725, K_int=-1.430010, m_l=12.0, L=41.0),
    (20.0, 12.0): GainSet(K_vel=0.503541, K_theta=-2.040418, K_theta_dot=1.764332, K_int=-1.211216, m_l=20.0, L=12.0),
    (20.0, 21.6): GainSet(K_vel=0.425856, K_theta=-3.038326, K_theta_dot=2.325337, K_int=-0.735984, m_l=20.0, L=21.6),
    (20.0, 31.3): GainSet(K_vel=0.383899, K_theta=-3.169761, K_theta_dot=2.663714, K_int=-0.527999, m_l=20.0, L=31.3),
    (20.0, 41.0): GainSet(K_vel=0.390490, K_theta=-4.804739, K_theta_dot=2.951219, K_int=-0.447214, m_l=20.0, L=41.0),
    (30.0, 12.0): GainSet(K_vel=0.760602, K_theta=-1.754593, K_theta_dot=2.430563, K_int=-1.688328, m_l=30.0, L=12.0),
    (30.0, 21.6): GainSet(K_vel=0.577953, K_theta=-2.256659, K_theta_dot=3.612170, K_int=-1.025897, m_l=30.0, L=21.6),
    (30.0, 31.3): GainSet(K_vel=0.462183, K_theta=-1.504810, K_theta_dot=4.749752, K_int=-0.735984, m_l=30.0, L=31.3),
    (30.0, 41.0): GainSet(K_vel=0.399341, K_theta=2.405183, K_theta_dot=4.653152, K_int=-0.527999, m_l=30.0, L=41.0),
    (40.0, 12.0): GainSet(K_vel=0.906937, K_theta=2.756381, K_theta_dot=3.002251, K_int=-1.993309, m_l=40.0, L=12.0),
    (40.0, 21.6): GainSet(K_vel=0.718338, K_theta=2.216566, K_theta_dot=4.178170, K_int=-1.211216, m_l=40.0, L=21.6),
    (40.0, 31.3): GainSet(K_vel=0.566707, K_theta=3.179175, K_theta_dot=5.557790, K_int=-0.868932, m_l=40.0, L=31.3),
    (40.0, 41.0): GainSet(K_vel=0.528542, K_theta=-0.089243, K_theta_dot=7.056531, K_int=-0.735984, m_l=40.0, L=41.0),
}

# Bridge System Gains (m_t = 17.90 lbm)
BRIDGE_GAINS = {
    (12.0, 12.0): GainSet(K_vel=1.976391, K_theta=-81.949578, K_theta_dot=1.257416, K_int=-3.280408, m_l=12.0, L=12.0),
    (12.0, 21.6): GainSet(K_vel=0.455047, K_theta=-11.231164, K_theta_dot=0.542875, K_int=-0.623377, m_l=12.0, L=21.6),
    (12.0, 31.3): GainSet(K_vel=1.101339, K_theta=-56.774449, K_theta_dot=3.959743, K_int=-1.211216, m_l=12.0, L=31.3),
    (12.0, 41.0): GainSet(K_vel=0.887126, K_theta=-49.125806, K_theta_dot=3.932262, K_int=-0.868932, m_l=12.0, L=41.0),
    (20.0, 12.0): GainSet(K_vel=1.967910, K_theta=-71.980504, K_theta_dot=3.013130, K_int=-3.280408, m_l=20.0, L=12.0),
    (20.0, 21.6): GainSet(K_vel=0.472873, K_theta=-5.142301, K_theta_dot=1.816830, K_int=-0.735984, m_l=20.0, L=21.6),
    (20.0, 31.3): GainSet(K_vel=0.421552, K_theta=-5.295005, K_theta_dot=2.110289, K_int=-0.527999, m_l=20.0, L=31.3),
    (20.0, 41.0): GainSet(K_vel=0.408947, K_theta=-8.573396, K_theta_dot=2.514614, K_int=-0.447214, m_l=20.0, L=41.0),
    (30.0, 12.0): GainSet(K_vel=1.970898, K_theta=-62.673135, K_theta_dot=2.321372, K_int=-3.280408, m_l=30.0, L=12.0),
    (30.0, 21.6): GainSet(K_vel=0.548672, K_theta=1.565625, K_theta_dot=3.006076, K_int=-0.868932, m_l=30.0, L=21.6),
    (30.0, 31.3): GainSet(K_vel=0.456203, K_theta=1.883961, K_theta_dot=3.782719, K_int=-0.623377, m_l=30.0, L=31.3),
    (30.0, 41.0): GainSet(K_vel=0.436215, K_theta=0.100998, K_theta_dot=4.365408, K_int=-0.527999, m_l=30.0, L=41.0),
    (40.0, 12.0): GainSet(K_vel=1.976717, K_theta=-54.025359, K_theta_dot=1.963894, K_int=-3.280408, m_l=40.0, L=12.0),
    (40.0, 21.6): GainSet(K_vel=0.731475, K_theta=1.142957, K_theta_dot=4.213284, K_int=-1.211216, m_l=40.0, L=21.6),
    (40.0, 31.3): GainSet(K_vel=0.545669, K_theta=4.953944, K_theta_dot=5.964091, K_int=-0.868932, m_l=40.0, L=31.3),
    (40.0, 41.0): GainSet(K_vel=0.483105, K_theta=4.050054, K_theta_dot=6.293301, K_int=-0.623377, m_l=40.0, L=41.0),
}


def get_gains(axis: str, m_l: float, L: float) -> GainSet:
    d = TROLLEY_GAINS if axis == "trolley" else BRIDGE_GAINS
    key = (m_l, L)
    if key in d:
        return d[key]
    # Nearest neighbor
    best_k, best_d2 = None, float('inf')
    for k in d:
        d2 = (k[0]-m_l)**2 + (k[1]-L)**2
        if d2 < best_d2:
            best_d2 = d2; best_k = k
    if best_k:
        return d[best_k]
    raise ValueError(f"No gains for axis={axis} m_l={m_l} L={L}")


# ── GPIO Pin Assignments (documentation only — main.py has authoritative PINS) ─

GPIO_PINS = {
    'trolley': {'pul': 22, 'dir': 27, 'ena': 17},
    'bridge':  {'pul': 23, 'dir': 24, 'ena': 25},
    'hoist':   {'pul': 5,  'dir': 6,  'ena': 13},
}

SPI_CONFIG = {
    'encoders': {'bus': 0, 'mode': 0, 'speed_hz': 500_000,
                 'cs_trolley': 8, 'cs_bridge': 7,
                 'cs_bridge_diag': 12, 'cs_hoist': 16},
    'imu':      {'bus': 1, 'ce': 2, 'mode': 0, 'speed_hz': 1_000_000},
}


class ControlMode:
    DISABLED = 0
    MANUAL = 1
    AUTO = 2
    HOMING = 3
    FAULT = 4


DEFAULT_CONFIG = SystemConfig()
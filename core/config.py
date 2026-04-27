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
    (5.0, 4.0): GainSet(K_vel=3.516941, K_theta=-95.136199, K_theta_dot=2.329195, K_int=-11.237304, m_l=5.0, L=4.0),
    (5.0, 5.0): GainSet(K_vel=0.757543, K_theta=-17.402416, K_theta_dot=0.368823, K_int=-2.280992, m_l=5.0, L=5.0),
    (5.0, 7.0): GainSet(K_vel=3.655403, K_theta=-132.015204, K_theta_dot=6.909888, K_int=-10.011300, m_l=5.0, L=7.0),
    (5.0, 12.0): GainSet(K_vel=1.903309, K_theta=-92.369617, K_theta_dot=6.573615, K_int=-4.251306, m_l=5.0, L=12.0),
    (5.0, 17.0): GainSet(K_vel=3.241605, K_theta=-193.335329, K_theta_dot=19.335821, K_int=-6.426169, m_l=5.0, L=17.0),
    (5.0, 18.0): GainSet(K_vel=3.016667, K_theta=-175.829247, K_theta_dot=17.473707, K_int=-5.725066, m_l=5.0, L=18.0),
    (5.0, 21.0): GainSet(K_vel=1.902471, K_theta=-122.293963, K_theta_dot=13.227039, K_int=-3.438170, m_l=5.0, L=21.0),
    (5.0, 23.0): GainSet(K_vel=3.593603, K_theta=-246.835964, K_theta_dot=31.102620, K_int=-6.426169, m_l=5.0, L=23.0),
    (5.0, 24.0): GainSet(K_vel=1.342125, K_theta=-90.382806, K_theta_dot=10.121238, K_int=-2.280992, m_l=5.0, L=24.0),
    (5.0, 29.0): GainSet(K_vel=1.732709, K_theta=-123.361280, K_theta_dot=16.194193, K_int=-2.735048, m_l=5.0, L=29.0),
    (5.0, 30.0): GainSet(K_vel=1.508688, K_theta=-110.830266, K_theta_dot=14.467936, K_int=-2.320259, m_l=5.0, L=30.0),
    (5.0, 36.0): GainSet(K_vel=2.387033, K_theta=-184.314967, K_theta_dot=28.361412, K_int=-3.442055, m_l=5.0, L=36.0),
    (5.0, 41.0): GainSet(K_vel=1.280809, K_theta=-102.063214, K_theta_dot=15.880118, K_int=-1.726869, m_l=5.0, L=41.0),
    (10.0, 4.0): GainSet(K_vel=2.520378, K_theta=-63.413594, K_theta_dot=1.301069, K_int=-7.954953, m_l=10.0, L=4.0),
    (10.0, 5.0): GainSet(K_vel=1.174876, K_theta=-25.830205, K_theta_dot=1.017952, K_int=-3.614721, m_l=10.0, L=5.0),
    (10.0, 7.0): GainSet(K_vel=0.381590, K_theta=-1.648122, K_theta_dot=0.565993, K_int=-1.087859, m_l=10.0, L=7.0),
    (10.0, 12.0): GainSet(K_vel=1.217863, K_theta=-51.540153, K_theta_dot=3.726544, K_int=-2.687244, m_l=10.0, L=12.0),
    (10.0, 17.0): GainSet(K_vel=2.345957, K_theta=-126.303912, K_theta_dot=12.300590, K_int=-4.549122, m_l=10.0, L=17.0),
    (10.0, 18.0): GainSet(K_vel=0.270756, K_theta=-3.046299, K_theta_dot=0.944412, K_int=-0.494322, m_l=10.0, L=18.0),
    (10.0, 21.0): GainSet(K_vel=0.261141, K_theta=-3.160379, K_theta_dot=1.023400, K_int=-0.440888, m_l=10.0, L=21.0),
    (10.0, 23.0): GainSet(K_vel=1.468904, K_theta=-92.527800, K_theta_dot=10.978656, K_int=-2.558882, m_l=10.0, L=23.0),
    (10.0, 24.0): GainSet(K_vel=1.661577, K_theta=-107.533503, K_theta_dot=13.372446, K_int=-2.869006, m_l=10.0, L=24.0),
    (10.0, 29.0): GainSet(K_vel=2.782940, K_theta=-188.336064, K_theta_dot=25.740771, K_int=-4.331823, m_l=10.0, L=29.0),
    (10.0, 30.0): GainSet(K_vel=1.408442, K_theta=-96.625836, K_theta_dot=13.196166, K_int=-2.170809, m_l=10.0, L=30.0),
    (10.0, 36.0): GainSet(K_vel=2.387769, K_theta=-179.512021, K_theta_dot=28.457374, K_int=-3.442055, m_l=10.0, L=36.0),
    (10.0, 41.0): GainSet(K_vel=1.281126, K_theta=-97.153974, K_theta_dot=16.048777, K_int=-1.726869, m_l=10.0, L=41.0),
    (12.0, 4.0): GainSet(K_vel=2.421927, K_theta=-65.163494, K_theta_dot=1.367371, K_int=-7.945974, m_l=12.0, L=4.0),
    (12.0, 5.0): GainSet(K_vel=0.880467, K_theta=-11.260182, K_theta_dot=0.603261, K_int=-2.555994, m_l=12.0, L=5.0),
    (12.0, 7.0): GainSet(K_vel=0.389771, K_theta=0.161724, K_theta_dot=0.681158, K_int=-1.087859, m_l=12.0, L=7.0),
    (12.0, 12.0): GainSet(K_vel=0.360359, K_theta=-2.543862, K_theta_dot=0.983872, K_int=-0.823119, m_l=12.0, L=12.0),
    (12.0, 17.0): GainSet(K_vel=0.290400, K_theta=-1.171843, K_theta_dot=1.149734, K_int=-0.554857, m_l=12.0, L=17.0),
    (12.0, 18.0): GainSet(K_vel=0.267938, K_theta=-0.452785, K_theta_dot=1.175575, K_int=-0.494322, m_l=12.0, L=18.0),
    (12.0, 21.0): GainSet(K_vel=1.084906, K_theta=-59.492513, K_theta_dot=6.658734, K_int=-1.937250, m_l=12.0, L=21.0),
    (12.0, 23.0): GainSet(K_vel=1.469512, K_theta=-90.662637, K_theta_dot=11.035936, K_int=-2.558882, m_l=12.0, L=23.0),
    (12.0, 24.0): GainSet(K_vel=1.662104, K_theta=-105.651381, K_theta_dot=13.424327, K_int=-2.869006, m_l=12.0, L=24.0),
    (12.0, 29.0): GainSet(K_vel=2.783300, K_theta=-186.425331, K_theta_dot=25.774434, K_int=-4.331823, m_l=12.0, L=29.0),
    (12.0, 30.0): GainSet(K_vel=1.009185, K_theta=-64.151572, K_theta_dot=8.603631, K_int=-1.537596, m_l=12.0, L=30.0),
    (12.0, 36.0): GainSet(K_vel=2.388067, K_theta=-177.591797, K_theta_dot=28.495754, K_int=-3.442055, m_l=12.0, L=36.0),
    (12.0, 41.0): GainSet(K_vel=1.281557, K_theta=-95.277505, K_theta_dot=16.114892, K_int=-1.726869, m_l=12.0, L=41.0),
    (15.0, 4.0): GainSet(K_vel=2.779203, K_theta=-68.530453, K_theta_dot=1.702963, K_int=-8.919054, m_l=15.0, L=4.0),
    (15.0, 5.0): GainSet(K_vel=1.021272, K_theta=-12.448984, K_theta_dot=0.846045, K_int=-3.012925, m_l=15.0, L=5.0),
    (15.0, 7.0): GainSet(K_vel=0.533835, K_theta=-0.917528, K_theta_dot=0.903995, K_int=-1.536728, m_l=15.0, L=7.0),
    (15.0, 12.0): GainSet(K_vel=0.355085, K_theta=1.341702, K_theta_dot=1.227772, K_int=-0.823119, m_l=15.0, L=12.0),
    (15.0, 17.0): GainSet(K_vel=0.317442, K_theta=0.883512, K_theta_dot=1.474526, K_int=-0.622806, m_l=15.0, L=17.0),
    (15.0, 18.0): GainSet(K_vel=0.292418, K_theta=1.418728, K_theta_dot=1.517529, K_int=-0.554857, m_l=15.0, L=18.0),
    (15.0, 21.0): GainSet(K_vel=0.311762, K_theta=-1.036643, K_theta_dot=1.680137, K_int=-0.554857, m_l=15.0, L=21.0),
    (15.0, 23.0): GainSet(K_vel=0.295407, K_theta=-0.583262, K_theta_dot=1.727068, K_int=-0.494322, m_l=15.0, L=23.0),
    (15.0, 24.0): GainSet(K_vel=0.298231, K_theta=-1.238468, K_theta_dot=1.804795, K_int=-0.494322, m_l=15.0, L=24.0),
    (15.0, 29.0): GainSet(K_vel=2.783905, K_theta=-183.575611, K_theta_dot=25.826601, K_int=-4.331823, m_l=15.0, L=29.0),
    (15.0, 30.0): GainSet(K_vel=0.637932, K_theta=-31.984343, K_theta_dot=5.074944, K_int=-0.970267, m_l=15.0, L=30.0),
    (15.0, 36.0): GainSet(K_vel=0.358742, K_theta=-10.618813, K_theta_dot=2.944697, K_int=-0.494043, m_l=15.0, L=36.0),
    (15.0, 41.0): GainSet(K_vel=1.282391, K_theta=-92.516655, K_theta_dot=16.220444, K_int=-1.726869, m_l=15.0, L=41.0),
    (20.0, 4.0): GainSet(K_vel=1.626756, K_theta=-28.347248, K_theta_dot=0.410032, K_int=-5.019803, m_l=20.0, L=4.0),
    (20.0, 5.0): GainSet(K_vel=1.245766, K_theta=-15.418263, K_theta_dot=1.221265, K_int=-3.796048, m_l=20.0, L=5.0),
    (20.0, 7.0): GainSet(K_vel=0.612742, K_theta=1.579704, K_theta_dot=1.221778, K_int=-1.811447, m_l=20.0, L=7.0),
    (20.0, 12.0): GainSet(K_vel=0.444998, K_theta=0.406029, K_theta_dot=1.820655, K_int=-1.087859, m_l=20.0, L=12.0),
    (20.0, 17.0): GainSet(K_vel=0.402893, K_theta=0.835698, K_theta_dot=2.090458, K_int=-0.823119, m_l=20.0, L=17.0),
    (20.0, 18.0): GainSet(K_vel=0.416508, K_theta=-0.033853, K_theta_dot=2.153071, K_int=-0.823119, m_l=20.0, L=18.0),
    (20.0, 21.0): GainSet(K_vel=0.294911, K_theta=2.931878, K_theta_dot=2.462583, K_int=-0.554857, m_l=20.0, L=21.0),
    (20.0, 23.0): GainSet(K_vel=0.314693, K_theta=2.476994, K_theta_dot=2.425940, K_int=-0.554857, m_l=20.0, L=23.0),
    (20.0, 24.0): GainSet(K_vel=0.321148, K_theta=1.894851, K_theta_dot=2.476307, K_int=-0.554857, m_l=20.0, L=24.0),
    (20.0, 29.0): GainSet(K_vel=0.376394, K_theta=-5.007073, K_theta_dot=3.019168, K_int=-0.582691, m_l=20.0, L=29.0),
    (20.0, 30.0): GainSet(K_vel=1.244347, K_theta=-72.910021, K_theta_dot=11.713161, K_int=-1.933971, m_l=20.0, L=30.0),
    (20.0, 36.0): GainSet(K_vel=0.232690, K_theta=2.150907, K_theta_dot=3.205127, K_int=-0.333218, m_l=20.0, L=36.0),
    (20.0, 41.0): GainSet(K_vel=0.217268, K_theta=1.975510, K_theta_dot=3.516978, K_int=-0.297199, m_l=20.0, L=41.0),
    (25.0, 4.0): GainSet(K_vel=3.186703, K_theta=-66.669940, K_theta_dot=1.872925, K_int=-10.011300, m_l=25.0, L=4.0),
    (25.0, 5.0): GainSet(K_vel=1.019104, K_theta=-2.726572, K_theta_dot=0.873179, K_int=-3.014627, m_l=25.0, L=5.0),
    (25.0, 7.0): GainSet(K_vel=0.763616, K_theta=0.555484, K_theta_dot=1.574491, K_int=-2.279705, m_l=25.0, L=7.0),
    (25.0, 12.0): GainSet(K_vel=0.555303, K_theta=2.007817, K_theta_dot=2.212370, K_int=-1.369843, m_l=25.0, L=12.0),
    (25.0, 17.0): GainSet(K_vel=0.503741, K_theta=-0.019961, K_theta_dot=2.685710, K_int=-1.035895, m_l=25.0, L=17.0),
    (25.0, 18.0): GainSet(K_vel=0.456306, K_theta=2.905415, K_theta_dot=2.659574, K_int=-0.923399, m_l=25.0, L=18.0),
    (25.0, 21.0): GainSet(K_vel=0.439905, K_theta=2.237745, K_theta_dot=2.882227, K_int=-0.823119, m_l=25.0, L=21.0),
    (25.0, 23.0): GainSet(K_vel=0.468553, K_theta=-2.430312, K_theta_dot=3.236858, K_int=-0.823119, m_l=25.0, L=23.0),
    (25.0, 24.0): GainSet(K_vel=0.398736, K_theta=3.062675, K_theta_dot=3.048384, K_int=-0.698288, m_l=25.0, L=24.0),
    (25.0, 29.0): GainSet(K_vel=0.351873, K_theta=3.005256, K_theta_dot=3.348521, K_int=-0.554857, m_l=25.0, L=29.0),
    (25.0, 30.0): GainSet(K_vel=0.351982, K_theta=2.978563, K_theta_dot=3.454576, K_int=-0.554857, m_l=25.0, L=30.0),
    (25.0, 36.0): GainSet(K_vel=0.389894, K_theta=-1.526830, K_theta_dot=3.997408, K_int=-0.554857, m_l=25.0, L=36.0),
    (25.0, 41.0): GainSet(K_vel=0.274182, K_theta=3.419324, K_theta_dot=4.188659, K_int=-0.374024, m_l=25.0, L=41.0),
    (30.0, 4.0): GainSet(K_vel=1.861885, K_theta=-22.060616, K_theta_dot=0.414351, K_int=-5.631357, m_l=30.0, L=4.0),
    (30.0, 5.0): GainSet(K_vel=1.577231, K_theta=-17.249710, K_theta_dot=1.929836, K_int=-5.016969, m_l=30.0, L=5.0),
    (30.0, 7.0): GainSet(K_vel=0.907534, K_theta=-0.692975, K_theta_dot=1.925436, K_int=-2.687244, m_l=30.0, L=7.0),
    (30.0, 12.0): GainSet(K_vel=0.627263, K_theta=3.715543, K_theta_dot=2.579925, K_int=-1.536728, m_l=30.0, L=12.0),
    (30.0, 17.0): GainSet(K_vel=0.594407, K_theta=-1.509176, K_theta_dot=3.318294, K_int=-1.221080, m_l=30.0, L=17.0),
    (30.0, 18.0): GainSet(K_vel=0.509454, K_theta=3.900783, K_theta_dot=3.211656, K_int=-1.035895, m_l=30.0, L=18.0),
    (30.0, 21.0): GainSet(K_vel=0.487199, K_theta=3.123699, K_theta_dot=3.520236, K_int=-0.923399, m_l=30.0, L=21.0),
    (30.0, 23.0): GainSet(K_vel=0.457425, K_theta=3.548524, K_theta_dot=3.642941, K_int=-0.823119, m_l=30.0, L=23.0),
    (30.0, 24.0): GainSet(K_vel=0.473404, K_theta=3.086445, K_theta_dot=3.640599, K_int=-0.823119, m_l=30.0, L=24.0),
    (30.0, 29.0): GainSet(K_vel=0.434418, K_theta=3.742108, K_theta_dot=4.001711, K_int=-0.698288, m_l=30.0, L=29.0),
    (30.0, 30.0): GainSet(K_vel=0.440909, K_theta=2.170262, K_theta_dot=4.203937, K_int=-0.698288, m_l=30.0, L=30.0),
    (30.0, 36.0): GainSet(K_vel=0.437673, K_theta=0.064321, K_theta_dot=4.658186, K_int=-0.622455, m_l=30.0, L=36.0),
    (30.0, 41.0): GainSet(K_vel=0.488458, K_theta=-6.902086, K_theta_dot=5.744919, K_int=-0.654049, m_l=30.0, L=41.0),
    (35.0, 4.0): GainSet(K_vel=2.543020, K_theta=-41.196055, K_theta_dot=0.914155, K_int=-7.954953, m_l=35.0, L=4.0),
    (35.0, 5.0): GainSet(K_vel=1.602922, K_theta=-11.266579, K_theta_dot=1.929634, K_int=-5.016969, m_l=35.0, L=5.0),
    (35.0, 7.0): GainSet(K_vel=0.889557, K_theta=3.728536, K_theta_dot=2.200603, K_int=-2.687244, m_l=35.0, L=7.0),
    (35.0, 12.0): GainSet(K_vel=0.752646, K_theta=0.921299, K_theta_dot=3.115747, K_int=-1.811447, m_l=35.0, L=12.0),
    (35.0, 17.0): GainSet(K_vel=0.591148, K_theta=4.101904, K_theta_dot=3.609682, K_int=-1.221080, m_l=35.0, L=17.0),
    (35.0, 18.0): GainSet(K_vel=0.596646, K_theta=3.069056, K_theta_dot=3.856662, K_int=-1.221080, m_l=35.0, L=18.0),
    (35.0, 21.0): GainSet(K_vel=0.551734, K_theta=3.485007, K_theta_dot=4.081076, K_int=-1.035895, m_l=35.0, L=21.0),
    (35.0, 23.0): GainSet(K_vel=0.575935, K_theta=1.237923, K_theta_dot=4.394984, K_int=-1.035895, m_l=35.0, L=23.0),
    (35.0, 24.0): GainSet(K_vel=0.521045, K_theta=4.325123, K_theta_dot=4.310456, K_int=-0.923399, m_l=35.0, L=24.0),
    (35.0, 29.0): GainSet(K_vel=0.507321, K_theta=3.197786, K_theta_dot=4.802326, K_int=-0.823119, m_l=35.0, L=29.0),
    (35.0, 30.0): GainSet(K_vel=0.522760, K_theta=1.524173, K_theta_dot=4.938219, K_int=-0.823119, m_l=35.0, L=30.0),
    (35.0, 36.0): GainSet(K_vel=0.433662, K_theta=4.198147, K_theta_dot=5.210778, K_int=-0.622455, m_l=35.0, L=36.0),
    (35.0, 41.0): GainSet(K_vel=0.457918, K_theta=2.058846, K_theta_dot=5.733681, K_int=-0.622806, m_l=35.0, L=41.0),
    (38.0, 4.0): GainSet(K_vel=2.034720, K_theta=-21.410751, K_theta_dot=0.656286, K_int=-6.313850, m_l=38.0, L=4.0),
    (38.0, 5.0): GainSet(K_vel=1.603288, K_theta=-8.311671, K_theta_dot=1.962020, K_int=-5.016969, m_l=38.0, L=5.0),
    (38.0, 7.0): GainSet(K_vel=1.008284, K_theta=3.611766, K_theta_dot=2.333981, K_int=-3.014627, m_l=38.0, L=7.0),
    (38.0, 12.0): GainSet(K_vel=0.750778, K_theta=4.220635, K_theta_dot=3.248607, K_int=-1.811447, m_l=38.0, L=12.0),
    (38.0, 17.0): GainSet(K_vel=0.661568, K_theta=4.563107, K_theta_dot=3.872262, K_int=-1.369843, m_l=38.0, L=17.0),
    (38.0, 18.0): GainSet(K_vel=0.680195, K_theta=1.039543, K_theta_dot=4.237540, K_int=-1.369843, m_l=38.0, L=18.0),
    (38.0, 21.0): GainSet(K_vel=0.658259, K_theta=0.525090, K_theta_dot=4.546505, K_int=-1.221080, m_l=38.0, L=21.0),
    (38.0, 23.0): GainSet(K_vel=0.573108, K_theta=4.843543, K_theta_dot=4.547198, K_int=-1.035895, m_l=38.0, L=23.0),
    (38.0, 24.0): GainSet(K_vel=0.594106, K_theta=3.481094, K_theta_dot=4.624766, K_int=-1.035895, m_l=38.0, L=24.0),
    (38.0, 29.0): GainSet(K_vel=0.584346, K_theta=1.092197, K_theta_dot=5.228050, K_int=-0.923399, m_l=38.0, L=29.0),
    (38.0, 30.0): GainSet(K_vel=0.515966, K_theta=4.122082, K_theta_dot=5.281883, K_int=-0.823119, m_l=38.0, L=30.0),
    (38.0, 36.0): GainSet(K_vel=0.481658, K_theta=4.703284, K_theta_dot=5.639423, K_int=-0.698288, m_l=38.0, L=36.0),
    (38.0, 41.0): GainSet(K_vel=0.462601, K_theta=3.254307, K_theta_dot=6.105300, K_int=-0.622455, m_l=38.0, L=41.0),
    (40.0, 4.0): GainSet(K_vel=1.813158, K_theta=-14.195039, K_theta_dot=0.606403, K_int=-5.631357, m_l=40.0, L=4.0),
    (40.0, 5.0): GainSet(K_vel=1.407176, K_theta=-2.964241, K_theta_dot=1.616432, K_int=-4.472136, m_l=40.0, L=5.0),
    (40.0, 7.0): GainSet(K_vel=0.985828, K_theta=4.545958, K_theta_dot=2.540417, K_int=-3.014627, m_l=40.0, L=7.0),
    (40.0, 12.0): GainSet(K_vel=0.734189, K_theta=4.862801, K_theta_dot=3.569822, K_int=-1.811447, m_l=40.0, L=12.0),
    (40.0, 17.0): GainSet(K_vel=0.649557, K_theta=5.028208, K_theta_dot=4.234226, K_int=-1.369843, m_l=40.0, L=17.0),
    (40.0, 18.0): GainSet(K_vel=0.694492, K_theta=4.526339, K_theta_dot=4.074696, K_int=-1.369843, m_l=40.0, L=18.0),
    (40.0, 21.0): GainSet(K_vel=0.653875, K_theta=3.434939, K_theta_dot=4.606550, K_int=-1.221080, m_l=40.0, L=21.0),
    (40.0, 23.0): GainSet(K_vel=0.682002, K_theta=-2.507671, K_theta_dot=5.366663, K_int=-1.221080, m_l=40.0, L=23.0),
    (40.0, 24.0): GainSet(K_vel=0.699598, K_theta=-4.038407, K_theta_dot=5.567749, K_int=-1.221080, m_l=40.0, L=24.0),
    (40.0, 29.0): GainSet(K_vel=0.573138, K_theta=3.383368, K_theta_dot=5.460509, K_int=-0.923399, m_l=40.0, L=29.0),
    (40.0, 30.0): GainSet(K_vel=0.584236, K_theta=0.696925, K_theta_dot=5.816145, K_int=-0.923399, m_l=40.0, L=30.0),
    (40.0, 36.0): GainSet(K_vel=0.641025, K_theta=-6.542753, K_theta_dot=7.116063, K_int=-0.923920, m_l=40.0, L=36.0),
    (40.0, 41.0): GainSet(K_vel=0.456673, K_theta=4.791479, K_theta_dot=6.439908, K_int=-0.622455, m_l=40.0, L=41.0),
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
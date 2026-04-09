"""
Crane Plant Model - State Space Representation
Matches the derivation in LQR_Control.m and Appendix C of the CDR

State vector (full): x = [x, v, θ, θ̇]ᵀ
  x     - position [in]
  v     - velocity [in/s]  
  θ     - sway angle [rad]
  θ̇     - sway angular rate [rad/s]

State vector (reduced, for control): x_red = [v, θ, θ̇]ᵀ
State vector (augmented with integrator): x_aug = [v, θ, θ̇, ∫e_v]ᵀ

Input: u = F (force) [lbf]
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple
from .config import G_IN_PER_S2, G_C, AxisConfig, SystemConfig


@dataclass
class PlantMatrices:
    """Container for state-space matrices"""
    A_full: np.ndarray      # 4x4 full state matrix
    B_full: np.ndarray      # 4x1 full input matrix
    A_red: np.ndarray       # 3x3 reduced state matrix (no position)
    B_red: np.ndarray       # 3x1 reduced input matrix
    A_aug: np.ndarray       # 4x4 augmented state matrix (with integrator)
    B_aug: np.ndarray       # 4x1 augmented input matrix


def build_plant_matrices(axis_config: AxisConfig, 
                         m_l: float, 
                         L: float,
                         c_theta: float = 0.05) -> PlantMatrices:
    """
    Build state-space matrices for given configuration.
    
    Args:
        axis_config: Trolley or bridge axis configuration
        m_l: Load mass [lbm]
        L: Cable length [in]
        c_theta: Rotational damping coefficient [lbf·in·s]
    
    Returns:
        PlantMatrices containing all state-space representations
    """
    m_t = axis_config.m_t
    b_x = axis_config.b_x
    
    # Effective rotational inertia
    J_eff = (m_l * L**2) / G_C
    
    # A matrix elements (from MATLAB derivation)
    a22 = -b_x / m_t
    a23 = (m_l * G_IN_PER_S2) / m_t
    a24 = (m_l * L * c_theta) / (m_t * J_eff)
    a42 = b_x / (m_t * L)
    a43 = -((m_l + m_t) * G_IN_PER_S2) / (m_t * L)
    a44 = -((m_l + m_t) * c_theta) / (m_t * J_eff)
    
    # B matrix elements
    b2 = G_C / m_t
    b4 = -G_C / (m_t * L)
    
    # Full state-space (4 states: x, v, θ, θ̇)
    A_full = np.array([
        [0,  1,   0,   0  ],
        [0,  a22, a23, a24],
        [0,  0,   0,   1  ],
        [0,  a42, a43, a44]
    ])
    
    B_full = np.array([[0], [b2], [0], [b4]])
    
    # Reduced system (remove position state for velocity tracking)
    A_red = A_full[1:4, 1:4]  # 3x3
    B_red = B_full[1:4]       # 3x1
    
    # Augmented system (add integrator for velocity error)
    # x_aug = [v, θ, θ̇, ∫(v_ref - v)dt]
    # The integrator row: d(∫e)/dt = v_ref - v = -v (when computing state derivative)
    A_aug = np.zeros((4, 4))
    A_aug[0:3, 0:3] = A_red
    A_aug[3, 0] = -1.0  # Integrator accumulates negative of velocity
    # A_aug[3, 1:4] = 0 (integrator not affected by θ, θ̇, or itself)
    
    B_aug = np.zeros((4, 1))
    B_aug[0:3, 0] = B_red.flatten()
    # B_aug[3] = 0 (force doesn't directly affect integrator)
    
    return PlantMatrices(
        A_full=A_full,
        B_full=B_full,
        A_red=A_red,
        B_red=B_red,
        A_aug=A_aug,
        B_aug=B_aug
    )


class CranePlant:
    """
    Crane plant model for simulation and state propagation.
    
    Can be used in two modes:
    1. Simulation: Full dynamics integration
    2. Real-time: Just provides matrices for control law
    """
    
    def __init__(self, config: SystemConfig, axis: str = "trolley"):
        """
        Initialize plant model.
        
        Args:
            config: System configuration
            axis: "trolley" or "bridge"
        """
        self.config = config
        self.axis = axis
        self.axis_config = config.trolley if axis == "trolley" else config.bridge
        
        # Build initial matrices
        self._update_matrices()
        
        # State: [x, v, θ, θ̇] for simulation
        # Augmented state: [v, θ, θ̇, ∫e_v] for control
        self.x_full = np.zeros(4)
        self.x_integrator = 0.0
        
    def _update_matrices(self):
        """Rebuild matrices (call when m_l or L changes)"""
        self.matrices = build_plant_matrices(
            self.axis_config,
            self.config.m_l,
            self.config.L,
            self.config.c_theta
        )
    
    def set_load_config(self, m_l: float, L: float):
        """Update load mass and cable length, rebuild matrices"""
        self.config.m_l = m_l
        self.config.L = L
        self._update_matrices()
    
    def reset(self):
        """Reset state to zero"""
        self.x_full = np.zeros(4)
        self.x_integrator = 0.0
    
    def get_augmented_state(self) -> np.ndarray:
        """Get augmented state vector [v, θ, θ̇, ∫e_v]"""
        return np.array([
            self.x_full[1],      # v
            self.x_full[2],      # θ
            self.x_full[3],      # θ̇
            self.x_integrator    # ∫e_v
        ])
    
    def _deriv(self, x: np.ndarray, u_sat: float) -> np.ndarray:
        """
        State derivative:  dx/dt = A·x + B·u
        
        Separated from step() so RK4 can evaluate it at intermediate points.
        """
        return self.matrices.A_full @ x + self.matrices.B_full.flatten() * u_sat
    
    def step(self, u: float, v_ref: float, dt: float) -> Tuple[np.ndarray, dict]:
        """
        Integrate one timestep using RK4 (zero-order hold on u).
        
        RK4 is required because the A matrix coupling term
        a23 = m_l·G/m_t ≈ 2252 makes Euler unstable at dt = 0.005 s.
        RK4's stability region covers |λ·dt| ≈ 2.8 vs Euler's 2.0,
        and more importantly handles the stiff off-diagonal coupling
        without the oscillatory blowup that Euler exhibits.
        
        Args:
            u: Control force [lbf]
            v_ref: Reference velocity [in/s]
            dt: Timestep [s]
            
        Returns:
            Tuple of (state, info_dict)
        """
        # Saturate input
        f_max = self.axis_config.f_max
        u_sat = np.clip(u, -f_max, f_max)
        
        # RK4 with zero-order hold on u (constant across the step)
        k1 = self._deriv(self.x_full, u_sat)
        k2 = self._deriv(self.x_full + 0.5 * dt * k1, u_sat)
        k3 = self._deriv(self.x_full + 0.5 * dt * k2, u_sat)
        k4 = self._deriv(self.x_full + dt * k3, u_sat)
        
        self.x_full = self.x_full + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)
        
        # Integrator update: accumulate velocity error
        v_error = v_ref - self.x_full[1]
        self.x_integrator = self.x_integrator + v_error * dt
        
        # Compute derived quantities
        accel_g = abs(k1[1]) / G_IN_PER_S2  # use k1 (start-of-step derivative)
        sway_deg = np.degrees(self.x_full[2])
        
        info = {
            'position': self.x_full[0],
            'velocity': self.x_full[1],
            'theta_rad': self.x_full[2],
            'theta_deg': sway_deg,
            'theta_dot': self.x_full[3],
            'accel_g': accel_g,
            'force_applied': u_sat,
            'saturated': abs(u) > f_max,
            'integrator': self.x_integrator
        }
        
        return self.x_full.copy(), info


def compute_natural_frequency(L: float) -> float:
    """Compute natural frequency [rad/s] for given cable length"""
    return np.sqrt(G_IN_PER_S2 / L)


def compute_period(L: float) -> float:
    """Compute swing period [s] for given cable length"""
    return 2 * np.pi / compute_natural_frequency(L)


def compute_damping_ratio(m_l: float, m_t: float, L: float, 
                          b_x: float, c_theta: float) -> float:
    """
    Estimate damping ratio from system parameters.
    This is approximate for the coupled system.
    """
    omega_n = compute_natural_frequency(L)
    # Simplified: treat as SDOF pendulum with equivalent damping
    J_eff = (m_l * L**2) / G_C
    c_eq = c_theta + b_x * L  # Rough equivalent
    zeta = c_eq / (2 * np.sqrt((m_l + m_t) * G_IN_PER_S2 * L))
    return zeta
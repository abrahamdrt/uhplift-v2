"""
Stepper Motor Driver - pigpio DMA Waveforms
Non-blocking continuous velocity interface for 200 Hz control loop

Architecture:
    set_velocity_continuous(v_in_s)   ← called every control tick (5ms)
        Builds a repeating DMA waveform at the target pulse frequency.
        Uses wave_send_repeat() for gapless output.  NEVER blocks.

    _send_pulses_ramped_blocking()    ← retained for homing / --test-motors ONLY
        Original trapezoidal profile with blocking wave_tx_busy() wait.

Waveform swap protocol (avoids glitches):
    1. wave_add_generic(new_pulses)
    2. new_wid = wave_create()
    3. wave_send_repeat(new_wid)     ← atomically replaces current TX
    4. wave_delete(old_wid)          ← safe: old is no longer in DMA chain

Hardware interface:
    CL42T-V41 drivers via ULN2003AN level shifter
    PUL/DIR/ENA control signals, 24V logic
    
Signal polarity (with ULN2003):
    GPIO HIGH → ULN sinks → opto ON
    ENA: GPIO LOW = disabled, GPIO HIGH = enabled
"""

import pigpio
import time
import math
from typing import Optional, Tuple
from dataclasses import dataclass


# ── CL42T Timing Requirements ─────────────────────────────────────────────────
# From CL42T V4.1 manual sequence chart
T1_ENA_DIR_MS = 250     # ENA must precede DIR by ≥200ms
T2_DIR_PUL_US = 10      # DIR must precede PUL rising edge by ≥2µs
MIN_PULSE_WIDTH_US = 2  # Minimum HIGH or LOW time ≥1µs (use 2µs margin)


@dataclass
class StepperConfig:
    """Configuration for a single stepper axis."""
    name: str
    pul_pin: int          # BCM GPIO for pulse
    dir_pin: int          # BCM GPIO for direction
    ena_pin: int          # BCM GPIO for enable
    
    steps_per_rev: int = 200      # Motor steps per revolution
    microstepping: int = 4        # Microstep divisor (DIP switch setting)
    gear_ratio: float = 4.0       # Motor:output shaft
    wheel_radius_in: float = 0.5  # Drive wheel/drum radius [in]
    
    # ── Continuous-mode parameters ────────────────────────────────────────
    repeat_chunk_pulses: int = 20   # Pulses per repeating waveform
    v_min_in_s: float = 0.01       # Below this |velocity|, stop TX [in/s]
    
    # ── Blocking-ramp parameters (homing / test-motors only) ─────────────
    start_ratio: float = 0.10
    start_min_hz: float = 80.0
    start_max_hz: float = 400.0
    accel_scale: float = 3.0
    accel_min_hz_s: float = 300.0
    accel_max_hz_s: float = 12000.0
    decel_scale: float = 3.0
    decel_min_hz_s: float = 300.0
    decel_max_hz_s: float = 12000.0
    ramp_chunk_pulses: int = 200
    
    @property
    def pulses_per_rev(self) -> int:
        """Pulses per motor shaft revolution."""
        return self.steps_per_rev * self.microstepping
    
    @property
    def pulses_per_output_rev(self) -> int:
        """Pulses per output shaft revolution (after gearbox)."""
        return int(self.pulses_per_rev * self.gear_ratio)
    
    @property
    def inches_per_pulse(self) -> float:
        """Linear travel per pulse [in/pulse]."""
        circumference = 2 * math.pi * self.wheel_radius_in
        return circumference / self.pulses_per_output_rev


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp value to range [lo, hi]."""
    return max(lo, min(hi, x))


class StepperDriver:
    """
    Stepper motor driver using pigpio DMA waveforms.
    
    Primary API for closed-loop control:
        set_velocity_continuous(velocity_in_s)   ← non-blocking, call every tick
    
    Blocking API for homing / motor checkout only:
        _send_pulses_ramped_blocking(total_pulses, target_hz)
    """
    
    def __init__(self,
                 config: StepperConfig,
                 simulation_mode: bool = False):
        self.config = config
        self.simulation_mode = simulation_mode
        
        self._pi: Optional[pigpio.pi] = None
        self._initialized = False
        self._enabled = False
        
        # ── Continuous-mode state ─────────────────────────────────────────
        self._active_wid: int = -1          # Currently transmitting waveform ID
        self._direction_forward: bool = True
        self._last_half_period_us: int = 0  # Avoid redundant rebuilds
        self._transmitting: bool = False
        
        # ── Position tracking ─────────────────────────────────────────────
        self._commanded_pulses: int = 0
    
    # ══════════════════════════════════════════════════════════════════════════
    # Lifecycle
    # ══════════════════════════════════════════════════════════════════════════
    
    def initialize(self) -> bool:
        """Initialize pigpio and configure GPIO pins."""
        if self.simulation_mode:
            print(f"[STEPPER:{self.config.name}] Running in simulation mode")
            self._initialized = True
            return True
        
        self._pi = pigpio.pi()
        if not self._pi.connected:
            print(f"[STEPPER:{self.config.name}] ERROR: pigpio daemon not running")
            print("  → Run: sudo pigpiod")
            return False
        
        self._pi.set_mode(self.config.pul_pin, pigpio.OUTPUT)
        self._pi.set_mode(self.config.dir_pin, pigpio.OUTPUT)
        self._pi.set_mode(self.config.ena_pin, pigpio.OUTPUT)
        
        # Safe initial state (disabled)
        self._pi.write(self.config.pul_pin, 0)
        self._pi.write(self.config.dir_pin, 0)
        self._pi.write(self.config.ena_pin, 0)
        
        print(f"[STEPPER:{self.config.name}] Initialized via pigpio")
        print(f"  PUL=GPIO{self.config.pul_pin}, DIR=GPIO{self.config.dir_pin}, "
              f"ENA=GPIO{self.config.ena_pin}")
        
        self._initialized = True
        return True
    
    def enable(self):
        """Enable motor driver.  Blocks for T1 ENA→DIR settle time."""
        if self.simulation_mode or self._pi is None:
            self._enabled = True
            return
        
        self._pi.write(self.config.ena_pin, 0) # active low
        print(f"[STEPPER:{self.config.name}] ENA asserted, waiting {T1_ENA_DIR_MS}ms...")
        time.sleep(T1_ENA_DIR_MS / 1000.0)
        
        # Sync DIR pin with internal state so the first velocity command
        # doesn't run in the wrong direction.  initialize() sets DIR=0
        # but _direction_forward defaults to True — this mismatch caused
        # the first-move direction glitch.
        self._direction_forward = True
        self._pi.write(self.config.dir_pin, 1)
        time.sleep(T2_DIR_PUL_US / 1_000_000)  # DIR settle
        
        self._enabled = True
        print(f"[STEPPER:{self.config.name}] Driver ENABLED")
    
    def disable(self):
        """Disable motor driver and stop any active transmission."""
        self._stop_transmission()
        if self._pi is not None:
            self._pi.write(self.config.ena_pin, 1) 
        self._enabled = False
        print(f"[STEPPER:{self.config.name}] Driver DISABLED")
    
    def close(self):
        """Release all resources."""
        if self._pi is not None:
            self._stop_transmission()
            self.disable()
            self._pi.write(self.config.pul_pin, 0)
            self._pi.write(self.config.dir_pin, 0)
            self._pi.stop()
            self._pi = None
        self._initialized = False
    
    # ══════════════════════════════════════════════════════════════════════════
    # Non-blocking continuous velocity API  (called every control tick)
    # ══════════════════════════════════════════════════════════════════════════
    
    def set_velocity_continuous(self, velocity_in_s: float) -> None:
        """
        Command a continuous linear velocity via repeating DMA waveform.
        
        Call this once per control tick (every 5ms at 200 Hz).
        NEVER blocks.  The DMA engine streams pulses between ticks.
        
        Args:
            velocity_in_s: Signed trolley/bridge velocity [in/s].
                           Positive = forward, negative = reverse.
        """
        # ── Simulation path ───────────────────────────────────────────────
        if self.simulation_mode:
            return
        
        if self._pi is None or not self._enabled:
            return
        
        # ── Zero-velocity dead-band: stop transmission entirely ───────────
        if abs(velocity_in_s) < self.config.v_min_in_s:
            self._stop_transmission()
            return
                
        # ── Determine desired direction and frequency ─────────────────────
        desired_forward = (velocity_in_s > 0.0)
        freq_hz = abs(velocity_in_s) / self.config.inches_per_pulse
        half_period_us = self._hz_to_half_period_us(freq_hz)
        
        # ── Direction change: stop → flip DIR → settle → resume ───────────
        if desired_forward != self._direction_forward:
            self._stop_transmission()
            self._direction_forward = desired_forward
            self._pi.write(self.config.dir_pin, 1 if desired_forward else 0)
            # CL42T requires ≥2µs DIR→PUL setup.  time.sleep resolves to
            # ~50-100µs on Linux, well within the ≤100µs budget and well
            # above the 2µs minimum.
            time.sleep(T2_DIR_PUL_US / 1_000_000)


        # ── Skip rebuild if frequency unchanged ───────────────────────────
        if half_period_us == self._last_half_period_us and self._transmitting:
            return
        
        # ── Build new repeating waveform ──────────────────────────────────
        new_wid = self._build_repeat_waveform(
            self.config.repeat_chunk_pulses, half_period_us
        )
        if new_wid < 0:
            return  # wave_create failed (resource exhaustion)
        
        # ── Atomic swap: start new, then delete old ───────────────────────
        old_wid = self._active_wid
        self._pi.wave_send_repeat(new_wid)
        if old_wid >= 0:
            try:
                self._pi.wave_delete(old_wid)
            except pigpio.error:
                pass  # Already deleted or invalid — not critical
        
        self._active_wid = new_wid
        self._last_half_period_us = half_period_us
        self._transmitting = True
    
    def _stop_transmission(self) -> None:
        """Stop DMA transmission and clean up the active waveform."""
        if self._pi is None:
            self._transmitting = False
            return
        
        if self._transmitting:
            self._pi.wave_tx_stop()
            # Drive PUL low to leave output in a known state
            self._pi.write(self.config.pul_pin, 0)
        
        if self._active_wid >= 0:
            try:
                self._pi.wave_delete(self._active_wid)
            except pigpio.error:
                pass
            self._active_wid = -1
        
        self._last_half_period_us = 0
        self._transmitting = False
    
    def _build_repeat_waveform(self, n_pulses: int, half_period_us: int) -> int:
        """
        Build a pigpio waveform suitable for wave_send_repeat().
        
        Does NOT call wave_clear() — that would destroy the currently
        transmitting waveform.  wave_create() consumes and clears the
        internal pulse buffer without affecting existing waveform IDs.
        
        Returns:
            Waveform ID (≥0), or negative on failure.
        """
        pin_mask = 1 << self.config.pul_pin
        pulse_list = []
        for _ in range(n_pulses):
            pulse_list.append(pigpio.pulse(pin_mask, 0, half_period_us))
            pulse_list.append(pigpio.pulse(0, pin_mask, half_period_us))
        
        self._pi.wave_add_generic(pulse_list)
        wid = self._pi.wave_create()
        return wid
    
    # ══════════════════════════════════════════════════════════════════════════
    # Blocking ramp API  (homing / --test-motors ONLY)
    # ══════════════════════════════════════════════════════════════════════════
    
    def set_direction(self, forward: bool) -> None:
        """Set motor direction with CL42T settle time (blocking)."""
        self._direction_forward = forward
        if self._pi is not None:
            self._pi.write(self.config.dir_pin, 1 if forward else 0)
            time.sleep(T2_DIR_PUL_US / 1_000_000)
    
    def _send_pulses_ramped_blocking(self,
                                     total_pulses: int,
                                     target_hz: float) -> None:
        """
        Send pulses with smooth trapezoidal/triangular velocity profile.
        
        *** BLOCKS until all pulses are sent. ***
        Use ONLY in homing sequences or --test-motors mode.
        NEVER call from the 200 Hz control loop.
        
        Args:
            total_pulses: Number of pulses to send
            target_hz: Target cruise frequency [Hz]
        """
        if self.simulation_mode:
            if self._direction_forward:
                self._commanded_pulses += total_pulses
            else:
                self._commanded_pulses -= total_pulses
            return
        
        if self._pi is None or total_pulses <= 0:
            return
        
        f_start, accel, decel = self._compute_ramp_params(target_hz)
        chunk = self.config.ramp_chunk_pulses
        
        f0 = max(float(f_start), 1.0)
        fT = max(float(target_hz), f0)
        
        # Calculate profile phases
        pulses_accel = self._ramp_pulses_needed(f0, fT, accel)
        pulses_decel = self._ramp_pulses_needed(f0, fT, decel)
        
        if pulses_accel + pulses_decel >= total_pulses:
            f_peak = self._solve_peak_frequency(total_pulses, f0, accel, decel)
            cruise_pulses = 0
        else:
            f_peak = fT
            cruise_pulses = int(round(total_pulses - pulses_accel - pulses_decel))
        
        accel_pulses = int(round(self._ramp_pulses_needed(f0, f_peak, accel)))
        decel_pulses = int(round(self._ramp_pulses_needed(f0, f_peak, decel)))
        
        used = accel_pulses + cruise_pulses + decel_pulses
        if used != total_pulses:
            cruise_pulses += (total_pulses - used)
            cruise_pulses = max(cruise_pulses, 0)
        
        # Acceleration phase
        f = f0
        remaining = accel_pulses
        while remaining > 0:
            dp = min(chunk, remaining)
            half_us = self._hz_to_half_period_us(f)
            wid = self._build_blocking_waveform(dp, half_us)
            self._send_waveform_blocking(wid)
            f = math.sqrt(max(f * f + 2.0 * accel * float(dp), f0 * f0))
            f = min(f, f_peak)
            remaining -= dp
        
        # Cruise phase
        remaining = cruise_pulses
        if remaining > 0:
            f = f_peak
            while remaining > 0:
                dp = min(chunk, remaining)
                half_us = self._hz_to_half_period_us(f)
                wid = self._build_blocking_waveform(dp, half_us)
                self._send_waveform_blocking(wid)
                remaining -= dp
        
        # Deceleration phase
        remaining = decel_pulses
        f = f_peak
        while remaining > 0:
            dp = min(chunk, remaining)
            half_us = self._hz_to_half_period_us(f)
            wid = self._build_blocking_waveform(dp, half_us)
            self._send_waveform_blocking(wid)
            f_sq_new = f * f - 2.0 * decel * float(dp)
            f = math.sqrt(max(f_sq_new, f0 * f0))
            remaining -= dp
        
        # Track commanded position
        if self._direction_forward:
            self._commanded_pulses += total_pulses
        else:
            self._commanded_pulses -= total_pulses
    
    def _send_pulses_constant_blocking(self, n_pulses: int,
                                       frequency_hz: float) -> None:
        """
        Send pulses at constant frequency (no ramping).  BLOCKS.
        For low-speed homing or testing only.
        """
        if self.simulation_mode:
            if self._direction_forward:
                self._commanded_pulses += n_pulses
            else:
                self._commanded_pulses -= n_pulses
            return
        
        if self._pi is None or n_pulses <= 0:
            return
        
        chunk = self.config.ramp_chunk_pulses
        half_us = self._hz_to_half_period_us(frequency_hz)
        
        remaining = n_pulses
        while remaining > 0:
            dp = min(chunk, remaining)
            wid = self._build_blocking_waveform(dp, half_us)
            self._send_waveform_blocking(wid)
            remaining -= dp
        
        if self._direction_forward:
            self._commanded_pulses += n_pulses
        else:
            self._commanded_pulses -= n_pulses
    
    # ══════════════════════════════════════════════════════════════════════════
    # Shared helpers
    # ══════════════════════════════════════════════════════════════════════════
    
    def velocity_to_frequency(self, velocity_in_s: float) -> float:
        """Convert velocity [in/s] to pulse frequency [Hz]."""
        return abs(velocity_in_s) / self.config.inches_per_pulse
    
    def get_commanded_position(self) -> float:
        """Get commanded position from pulse counting [in]."""
        return self._commanded_pulses * self.config.inches_per_pulse
    
    def zero_position(self) -> None:
        """Reset commanded position to zero."""
        self._commanded_pulses = 0
    
    @property
    def is_enabled(self) -> bool:
        return self._enabled
    
    @property
    def is_transmitting(self) -> bool:
        return self._transmitting
    
    # ══════════════════════════════════════════════════════════════════════════
    # Internal helpers
    # ══════════════════════════════════════════════════════════════════════════
    
    def _hz_to_half_period_us(self, hz: float) -> int:
        """Convert frequency to half-period in microseconds."""
        hz = max(float(hz), 1.0)
        half = int(1_000_000 / (2.0 * hz))
        return max(half, MIN_PULSE_WIDTH_US)
    
    def _build_blocking_waveform(self, n_pulses: int, half_period_us: int) -> int:
        """
        Build waveform for blocking send (wave_send_once).
        Calls wave_clear() first — safe because nothing else is transmitting
        during a blocking ramp sequence.
        """
        if self._pi is None:
            return -1
        
        self._pi.wave_clear()
        pin_mask = 1 << self.config.pul_pin
        pulse_list = []
        for _ in range(n_pulses):
            pulse_list.append(pigpio.pulse(pin_mask, 0, half_period_us))
            pulse_list.append(pigpio.pulse(0, pin_mask, half_period_us))
        self._pi.wave_add_generic(pulse_list)
        return self._pi.wave_create()
    
    def _send_waveform_blocking(self, wid: int) -> None:
        """Send waveform and block until complete."""
        if self._pi is None or wid < 0:
            return
        self._pi.wave_send_once(wid)
        while self._pi.wave_tx_busy():
            time.sleep(0.005)
        self._pi.wave_delete(wid)
    
    def _compute_ramp_params(self, target_hz: float) -> Tuple[float, float, float]:
        """Compute (start_hz, accel, decel) for blocking ramp."""
        t = max(float(target_hz), 1.0)
        c = self.config
        start_hz = clamp(t * c.start_ratio, c.start_min_hz, c.start_max_hz)
        accel = clamp(t * c.accel_scale, c.accel_min_hz_s, c.accel_max_hz_s)
        decel = clamp(t * c.decel_scale, c.decel_min_hz_s, c.decel_max_hz_s)
        if t < start_hz:
            start_hz = max(1.0, t * 0.5)
        return start_hz, accel, decel
    
    @staticmethod
    def _ramp_pulses_needed(f0: float, f1: float, slope: float) -> float:
        slope = max(float(slope), 1e-6)
        return (f1 * f1 - f0 * f0) / (2.0 * slope)
    
    @staticmethod
    def _solve_peak_frequency(total_pulses: int, f_start: float,
                              accel: float, decel: float) -> float:
        a = max(float(accel), 1e-6)
        d = max(float(decel), 1e-6)
        f0_sq = f_start * f_start
        denom = (1.0 / a) + (1.0 / d)
        fpeak_sq = f0_sq + (2.0 * float(total_pulses)) / denom
        return math.sqrt(max(fpeak_sq, f0_sq))


# ── Pre-configured Axes ───────────────────────────────────────────────────────
# Pin assignments match config.py GPIO_PINS (validated)

TROLLEY_STEPPER = StepperConfig(
    name="trolley",
    pul_pin=22, dir_pin=27, ena_pin=17,
    steps_per_rev=200, microstepping=4,
    gear_ratio=5.0, wheel_radius_in=0.485,
)

BRIDGE_STEPPER = StepperConfig(
    name="bridge",
    pul_pin=23, dir_pin=24, ena_pin=25,
    steps_per_rev=200, microstepping=4,
    gear_ratio=4.0, wheel_radius_in=0.485,
)

HOIST_STEPPER = StepperConfig(
    name="hoist",
    pul_pin=5,
    dir_pin=6,
    ena_pin=13,
    steps_per_rev=200,
    microstepping=4,
    gear_ratio=10.0,
    wheel_radius_in=0.75,  # Drum radius [in]
)


def create_stepper(axis: str, simulation_mode: bool = False) -> StepperDriver:
    """Factory function to create stepper driver for specified axis."""
    configs = {
        "trolley": TROLLEY_STEPPER,
        "bridge": BRIDGE_STEPPER,
        "hoist": HOIST_STEPPER,
    }
    if axis not in configs:
        raise ValueError(f"Unknown axis: {axis}. Use 'trolley', 'bridge', or 'hoist'.")
    return StepperDriver(configs[axis], simulation_mode)
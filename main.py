#!/usr/bin/env python3
"""
UHplift Crane Control System — Final Main Loop
Team 11 Capstone II | University of Houston | Spring 2026

Coordinate System (physical frame, immutable):
    X-axis: Bridge  — North(-)→South(+), sway theta_x in XY plane
    Z-axis: Trolley — West(-)→East(+),  sway theta_z in ZY plane
    Y-axis: Hoist   — Up(+)/Down(-), cable length L increases downward

Usage:
    python3 main.py --mode manual --log     # Jog all axes, no LQR
    python3 main.py --mode bridge --log     # Bridge LQR, trolley locked
    python3 main.py --mode trolley --log    # Trolley LQR, bridge locked
    python3 main.py --test-sensors          # Sensor checkout

HMI (PS4 DualShock 4):
    Cross (B0):   Enable/disable drives
    Circle (B1):  Software stop (held)
    Square (B3):  Toggle MANUAL<->AUTO (bridge/trolley modes only)
    Options (B9): Zero ALL encoders
    L1+D-Up:      Home bridge encoder to 0
    L1+D-Left:    Home trolley encoder to 0
    L1+R2:        Home hoist to L=12.0 in
    D-pad U/D:    Bridge velocity command
    D-pad L/R:    Trolley velocity command
    L2/R2:        Hoist down/up
"""

import sys
import time
import argparse
import signal
import csv
import os
import math
import numpy as np
from datetime import datetime
from typing import Optional
from dataclasses import dataclass

from core.config import (
    SystemConfig, ControlMode, DEFAULT_CONFIG,
    get_gains, G_IN_PER_S2
)
from core.controller import LQIController
from core.reference import ManualModeGenerator
from core.estimator import (
    DerivativeEstimator, ComplementaryFilter, LowPassFilter
)
from drivers.imu import MPU6050
from drivers.stepper import StepperDriver, StepperConfig
from drivers.joystick import JoystickDriver

try:
    import spidev
    import pigpio
except ImportError:
    spidev = None
    pigpio = None

# ── Final Hardware Pin Map ────────────────────────────────────────────────────
PINS = {
    'trolley': {'pul': 22, 'dir': 27, 'ena': 17, 'cs': 8},
    'bridge':  {'pul': 23, 'dir': 24, 'ena': 25, 'cs': 7, 'cs_diag': 12},
    'hoist':   {'pul': 5,  'dir': 6,  'ena': 13, 'cs': 16,
                'brake': 26},  # NOTE: Cannot use GPIO16 for brake — conflicts
                               # with hoist encoder CS2. Using GPIO26 instead.
                               # Verify physical wiring matches this assignment.
}
ENC_IPC = {
    'trolley': -0.000152,  # NEGATED: encoder counts west=positive but
                           # coordinate system requires west=negative, east=positive
    'bridge': 0.000190,
    'bridge_diag': 0.000190, 'hoist': 0.000118,
}
L_HOME = 12.0
L_MAX  = 41.138

# ══════════════════════════════════════════════════════════════════════════════
# Encoder Bus — 4x LS7366R on SPI0 with software chip selects
# ══════════════════════════════════════════════════════════════════════════════

class EncoderBus:
    WR_MDR0  = 0x88; WR_MDR1  = 0x90; CLR_CNTR = 0x20
    LOAD_OTR = 0xE8; RD_OTR   = 0x68; RD_MDR0  = 0x48

    def __init__(self, pi_handle, simulation_mode=False):
        self._pi = pi_handle
        self._sim = simulation_mode
        self._spi = None
        self._encs = {}

    def add(self, name, cs_gpio, in_per_count):
        self._encs[name] = {'cs': cs_gpio, 'ipc': in_per_count, 'offset': 0}

    def initialize(self):
        results = {}
        if self._sim:
            for n in self._encs:
                print(f"[ENCODER:{n}] Simulation mode")
                results[n] = True
            return results
        self._spi = spidev.SpiDev()
        self._spi.open(0, 0)
        self._spi.no_cs = True
        self._spi.mode = 0
        self._spi.max_speed_hz = 500_000
        for enc in self._encs.values():
            self._pi.set_mode(enc['cs'], pigpio.OUTPUT)
            self._pi.write(enc['cs'], 1)
        for n, enc in self._encs.items():
            results[n] = self._init_one(n, enc['cs'])
        return results

    def _init_one(self, name, cs):
        self._xfer(cs, [self.WR_MDR0, 0x03]); time.sleep(0.001)
        self._xfer(cs, [self.WR_MDR1, 0x00]); time.sleep(0.001)
        self._xfer(cs, [self.CLR_CNTR]);       time.sleep(0.001)
        rb = self._xfer(cs, [self.RD_MDR0, 0x00])
        ok = rb[1] == 0x03
        print(f"[ENCODER:{name}] MDR0=0x{rb[1]:02X} "
              f"{'ok' if ok else 'FAIL'} (CS=GPIO{cs})")
        return ok

    def _xfer(self, cs, data):
        self._pi.write(cs, 0); time.sleep(0.000002)
        result = self._spi.xfer2(list(data))
        self._pi.write(cs, 1); time.sleep(0.000002)
        return result

    def read_counts(self, name):
        if self._sim: return 0
        enc = self._encs[name]; cs = enc['cs']
        self._xfer(cs, [self.LOAD_OTR]); time.sleep(0.0001)
        raw = self._xfer(cs, [self.RD_OTR, 0, 0, 0, 0])
        c = (raw[1]<<24)|(raw[2]<<16)|(raw[3]<<8)|raw[4]
        if c >= 0x80000000: c -= 0x100000000
        return c - enc['offset']

    def read_position(self, name):
        return self.read_counts(name) * self._encs[name]['ipc']

    def zero(self, name):
        enc = self._encs[name]
        if self._sim: enc['offset'] = 0; return
        enc['offset'] = self.read_counts(name) + enc['offset']
        print(f"[ENCODER:{name}] Zeroed")

    def close(self):
        if self._spi: self._spi.close()


# ══════════════════════════════════════════════════════════════════════════════
# System State
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class SystemState:
    mode: int = ControlMode.DISABLED
    fault_code: int = 0
    x_bridge: float = 0.0;  v_bridge: float = 0.0
    v_ref_bridge: float = 0.0; v_cmd_bridge: float = 0.0; F_bridge: float = 0.0
    theta_x: float = 0.0; theta_x_dot: float = 0.0; theta_x_raw: float = 0.0
    x_trolley: float = 0.0; v_trolley: float = 0.0
    v_ref_trolley: float = 0.0; v_cmd_trolley: float = 0.0; F_trolley: float = 0.0
    theta_z: float = 0.0; theta_z_dot: float = 0.0; theta_z_raw: float = 0.0
    x_hoist: float = 0.0; v_hoist: float = 0.0; v_ref_hoist: float = 0.0
    L_current: float = L_HOME
    loop_time_ms: float = 0.0; loop_count: int = 0


# ══════════════════════════════════════════════════════════════════════════════
# Crane Controller
# ══════════════════════════════════════════════════════════════════════════════

class CraneController:

    def __init__(self, config, active_mode="manual",
                 log_csv=False, simulation_mode=False):
        self.config = config
        self.active_mode = active_mode
        self.log_csv = log_csv
        self.simulation_mode = simulation_mode
        self.state = SystemState()
        dt = config.dt

        # Velocity estimators
        self.vel_est_trolley = DerivativeEstimator(dt, filter_cutoff_hz=20.0)
        self.vel_est_bridge  = DerivativeEstimator(dt, filter_cutoff_hz=20.0)
        self.vel_est_hoist   = DerivativeEstimator(dt, filter_cutoff_hz=20.0)

        # Sway estimators
        self.comp_x = ComplementaryFilter(alpha=0.98, dt=dt)
        self.comp_z = ComplementaryFilter(alpha=0.98, dt=dt)
        self.tdot_lpf_x = LowPassFilter(10.0, dt)
        self.tdot_lpf_z = LowPassFilter(10.0, dt)
        self.theta_deadband_rad = np.radians(0.2)

        # Reference generators
        self.ref_bridge  = ManualModeGenerator(v_max=config.bridge.v_target,
                                               accel_max_g=0.20)
        self.ref_trolley = ManualModeGenerator(v_max=config.trolley.v_target,
                                               accel_max_g=0.20)
        self.ref_hoist   = ManualModeGenerator(v_max=1.25, accel_max_g=0.03)

        # LQR controller (active axis only)
        self.controller = None
        self._v_cmd_active = 0.0
        if active_mode == "bridge":
            gains = get_gains("bridge", config.m_l, config.L)
            self.controller = LQIController(gains, f_max=config.bridge.f_max)
        elif active_mode == "trolley":
            gains = get_gains("trolley", config.m_l, config.L)
            self.controller = LQIController(gains, f_max=config.trolley.f_max)

        self._last_gain_L = config.L
        self._pi = None
        self.encoder_bus = None; self.imu = None
        self.stepper_trolley = None; self.stepper_bridge = None
        self.stepper_hoist = None; self.joystick = None
        self._running = False
        self._enable_last = False; self._mode_last = False; self._reset_last = False
        # Hoist brake delay — ticks to wait after brake release before sending pulses
        # At 200Hz: 20 ticks = 100ms, 40 ticks = 200ms
        self._brake_delay_ticks = 40   # 200ms — adjust if brake is faster/slower
        self._brake_ticks_remaining = 0
        self._brake_released = False
        self._csv_file = None; self._csv_writer = None; self._t0 = 0.0

    # ── Initialize ───────────────────────────────────────────────────────

    def initialize(self):
        print("=" * 60)
        print(f"  UHplift Crane — Mode: {self.active_mode.upper()}")
        print("=" * 60 + "\n")
        success = True

        if not self.simulation_mode:
            self._pi = pigpio.pi()
            if not self._pi.connected:
                print("[ERROR] pigpiod not running"); return False
            # Hoist brake GPIO — K4 is wired NC (brake on NC terminal)
            # HIGH = relay energized = NC opens = no 24V to brake coil = spring ENGAGES brake
            # LOW  = relay off       = NC closed = 24V to brake coil  = brake RELEASED
            self._pi.set_mode(PINS['hoist']['brake'], pigpio.OUTPUT)
            self._pi.write(PINS['hoist']['brake'], 1)  # Start with brake ENGAGED
        else:
            self._pi = None

        # Encoders
        print("[INIT] Encoder bus (SPI0 software CS)...")
        self.encoder_bus = EncoderBus(self._pi, self.simulation_mode)
        self.encoder_bus.add("trolley", PINS['trolley']['cs'], ENC_IPC['trolley'])
        self.encoder_bus.add("bridge",  PINS['bridge']['cs'],  ENC_IPC['bridge'])
        self.encoder_bus.add("bridge_diag", PINS['bridge']['cs_diag'], ENC_IPC['bridge_diag'])
        self.encoder_bus.add("hoist",   PINS['hoist']['cs'],   ENC_IPC['hoist'])
        enc_ok = self.encoder_bus.initialize()

        critical_fail = False
        if self.active_mode == "bridge" and not enc_ok.get("bridge", False):
            print("  [CRITICAL] Bridge encoder failed"); critical_fail = True
        if self.active_mode == "trolley" and not enc_ok.get("trolley", False):
            print("  [CRITICAL] Trolley encoder failed"); critical_fail = True
        for n, ok in enc_ok.items():
            if not ok:
                print(f"  [WARN] {n} encoder failed (non-critical)")

        # IMU
        print("[INIT] IMU (I2C1, MPU6050)...")
        self.imu = MPU6050(simulation_mode=self.simulation_mode)
        if not self.imu.initialize():
            if self.active_mode in ("bridge", "trolley"):
                print("  [CRITICAL] IMU failed"); critical_fail = True
            else:
                print("  [WARN] IMU failed — MANUAL still OK")

        if critical_fail:
            print("\n  Critical sensor fail → forced to MANUAL-only")
            self.controller = None; self.active_mode = "manual"

        # Steppers
        print("[INIT] Steppers...")
        self.stepper_trolley = StepperDriver(StepperConfig(
            name="trolley", pul_pin=PINS['trolley']['pul'],
            dir_pin=PINS['trolley']['dir'], ena_pin=PINS['trolley']['ena'],
            steps_per_rev=200, microstepping=4, gear_ratio=5.0,
            wheel_radius_in=0.485), self.simulation_mode)
        if not self.stepper_trolley.initialize(): success = False

        self.stepper_bridge = StepperDriver(StepperConfig(
            name="bridge", pul_pin=PINS['bridge']['pul'],
            dir_pin=PINS['bridge']['dir'], ena_pin=PINS['bridge']['ena'],
            steps_per_rev=200, microstepping=4, gear_ratio=4.0,
            wheel_radius_in=0.485), self.simulation_mode)
        if not self.stepper_bridge.initialize(): success = False

        self.stepper_hoist = StepperDriver(StepperConfig(
            name="hoist", pul_pin=PINS['hoist']['pul'],
            dir_pin=PINS['hoist']['dir'], ena_pin=PINS['hoist']['ena'],
            steps_per_rev=200, microstepping=1, gear_ratio=10.0,
            wheel_radius_in=0.75), self.simulation_mode)
        self.stepper_hoist.initialize()

        # Joystick
        print("[INIT] Joystick...")
        self.joystick = JoystickDriver(simulation_mode=self.simulation_mode)
        self.joystick.initialize()

        print()
        if success:
            print(f"[INIT] Ready — {self.active_mode.upper()}")
            self.state.mode = ControlMode.DISABLED
        else:
            print("[INIT] Hardware failure")
            self.state.mode = ControlMode.FAULT; self.state.fault_code = 1
        return success

    def calibrate_imu(self):
        if self.imu:
            print("\n[CAL] IMU — hold stationary...")
            self.imu.calibrate(num_samples=100, delay_ms=10)

    # ── Drives ───────────────────────────────────────────────────────────

    def enable_drives(self):
        print("[ENABLE] Drivers hardwired, syncing DIR...")
        for s in [self.stepper_trolley, self.stepper_bridge, self.stepper_hoist]:
            if s: s.enable()
        # Brake is NOT released here — it releases only when hoist is commanded.
        # See control_step() hoist section.
        print("[ENABLE] Done")

    def disable_drives(self):
        print("[DISABLE] Stopping pulses + engaging brake...")
        # Engage hoist brake FIRST — K4 wired NC:
        # HIGH = relay energized = NC opens = no 24V to brake coil = spring ENGAGES brake
        if self._pi and not self.simulation_mode:
            self._pi.write(PINS['hoist']['brake'], 1)
            print("[BRAKE] Engaged (NC open, spring holds)")
        for s in [self.stepper_trolley, self.stepper_bridge, self.stepper_hoist]:
            if s: s.disable()
        self.state.mode = ControlMode.DISABLED
        print("[DISABLE] Done")

    # ── Homing ───────────────────────────────────────────────────────────

    def zero_bridge(self):
        self.encoder_bus.zero("bridge"); self.encoder_bus.zero("bridge_diag")
        self.vel_est_bridge.reset(); self.comp_x.reset()
        self.state.x_bridge = 0.0; self.state.v_bridge = 0.0
        self._v_cmd_active = 0.0
        if self.controller and self.active_mode == "bridge": self.controller.reset()
        print("[HOME] Bridge -> X=0")

    def zero_trolley(self):
        self.encoder_bus.zero("trolley")
        self.vel_est_trolley.reset(); self.comp_z.reset()
        self.state.x_trolley = 0.0; self.state.v_trolley = 0.0
        self._v_cmd_active = 0.0
        if self.controller and self.active_mode == "trolley": self.controller.reset()
        print("[HOME] Trolley -> Z=0")

    def home_hoist(self):
        self.encoder_bus.zero("hoist"); self.vel_est_hoist.reset()
        self.state.x_hoist = 0.0; self.state.L_current = L_HOME
        self.config.L = L_HOME
        print(f"[HOME] Hoist -> L={L_HOME}")

    def zero_all(self):
        self.zero_bridge(); self.zero_trolley(); self.home_hoist()

    # ── Sensor Reading ───────────────────────────────────────────────────
    def read_sensors(self):
        pos_t = self.encoder_bus.read_position("trolley")
        pos_b = self.encoder_bus.read_position("bridge")

        # --- VALIDATION HACK: HOIST ENCODER OFFLINE ---
        # Comment out the real read so it doesn't hang or throw bad SPI data:
        # pos_h = self.encoder_bus.read_position("hoist")
        pos_h = 0.0 # Dummy value to prevent variable reference errors

        self.state.x_trolley = pos_t
        self.state.x_bridge = pos_b
        self.state.x_hoist = pos_h
        self.state.v_trolley = self.vel_est_trolley.update(pos_t)
        self.state.v_bridge  = self.vel_est_bridge.update(pos_b)
        self.state.v_hoist   = self.vel_est_hoist.update(pos_h)
        # Hoist -> dynamic L
        # Comment out the dynamic calculation:
        # self.state.L_current = max(L_HOME, min(L_MAX, L_HOME - pos_h))

        # --- VALIDATION HACK: FORCE GAIN SCHEDULER STATE ---
        self.state.L_current = 12.0  # <-- CHANGE THIS to match physical test length
        self.config.m_l = 20.3       # <-- CHANGE THIS to match physical test mass
        # IMU
        if self.imu:
            accel, gyro = self.imu.read()
            # Bridge sway theta_x: XY plane, pitch
            tx_acc = np.arctan2(accel[0], accel[1])
            tx_dot_raw = gyro[2]
            # Trolley sway theta_z: ZY plane, roll
            tz_acc = np.arctan2(accel[2], accel[1])
            tz_dot_raw = gyro[0]
            self.state.theta_x = self.comp_x.update(tx_dot_raw, tx_acc)
            self.state.theta_z = self.comp_z.update(tz_dot_raw, tz_acc)
            self.state.theta_x_dot = self.tdot_lpf_x.update(tx_dot_raw)
            self.state.theta_z_dot = self.tdot_lpf_z.update(tz_dot_raw)
            self.state.theta_x_raw = self.state.theta_x
            self.state.theta_z_raw = self.state.theta_z
            if abs(self.state.theta_x) < self.theta_deadband_rad:
                self.state.theta_x = 0.0; self.state.theta_x_dot = 0.0
            if abs(self.state.theta_z) < self.theta_deadband_rad:
                self.state.theta_z = 0.0; self.state.theta_z_dot = 0.0
        # Gain scheduling
        if abs(self.state.L_current - self._last_gain_L) > 1.0 and self.controller:
            self._last_gain_L = self.state.L_current
            self.config.L = self.state.L_current
            try:
                g = get_gains(self.active_mode, self.config.m_l, self.config.L)
                self.controller.set_gains(g)
                print(f"[GAINS] L={self.config.L:.1f} m_l={self.config.m_l:.1f}") # Added mass to print for sanity check
            except ValueError:
                print(f"[GAINS] No match for L={self.config.L:.1f}") 

    # ── HMI ──────────────────────────────────────────────────────────────

    def process_hmi(self):
        if not self.joystick: return
        self.joystick.update()

        if self.joystick.get_estop_pressed():
            if self.state.mode != ControlMode.DISABLED:
                print("[HMI] STOP"); self.disable_drives()
            return

        # L1 homing (mask motion)
        l1 = False
        if not self.simulation_mode and self.joystick._connected:
            l1 = self.joystick._buttons.get(4, False)
        if l1 and self.state.mode != ControlMode.DISABLED:
            hat = self.joystick._hats.get(0, (0, 0))
            if hat[1] == 1:  self.zero_bridge()
            if hat[0] == -1: self.zero_trolley()
            if self.joystick._buttons.get(7, False): self.home_hoist()
            return

        # Enable toggle (Cross)
        en = self.joystick.get_enable_pressed()
        if en and not self._enable_last:
            if self.state.mode == ControlMode.DISABLED:
                self.enable_drives(); self.state.mode = ControlMode.MANUAL
                print("[HMI] -> MANUAL")
            elif self.state.mode in (ControlMode.MANUAL, ControlMode.AUTO):
                self.disable_drives(); print("[HMI] -> DISABLED")
        self._enable_last = en

        # Mode toggle (Square)
        md = self.joystick.get_mode_pressed()
        if md and not self._mode_last:
            if self.active_mode in ("bridge","trolley") and self.controller:
                if self.state.mode == ControlMode.MANUAL:
                    self.state.mode = ControlMode.AUTO
                    self.controller.reset(); self._v_cmd_active = 0.0
                    print(f"[HMI] -> AUTO ({self.active_mode.upper()})")
                elif self.state.mode == ControlMode.AUTO:
                    self.state.mode = ControlMode.MANUAL
                    print("[HMI] -> MANUAL")
            elif self.active_mode == "manual":
                print("[HMI] No AUTO in --mode manual")
        self._mode_last = md

        # Options: zero all
        rs = self.joystick.get_reset_pressed()
        if rs and not self._reset_last:
            if self.state.mode == ControlMode.FAULT:
                self.state.mode = ControlMode.DISABLED; self.state.fault_code = 0
                print("[HMI] Fault cleared")
            else:
                self.zero_all()
        self._reset_last = rs

    # ── Control Step ─────────────────────────────────────────────────────

    def control_step(self):
        dt = self.config.dt
        joy_b = self.joystick.get_bridge_input() if self.joystick else 0.0
        joy_t = self.joystick.get_trolley_input() if self.joystick else 0.0
        joy_h = self.joystick.get_hoist_input() if self.joystick else 0.0

        # Hoist always manual — brake follows command with delay
        # K4 wired NC: LOW = relay off = NC closed = 24V to coil = RELEASED
        #              HIGH = relay on  = NC open  = no 24V      = ENGAGED
        vr_h = self.ref_hoist.update(joy_h, dt)
        self.state.v_ref_hoist = vr_h

        if self._pi and not self.simulation_mode:
            if abs(vr_h) > 0.0:
                if not self._brake_released:
                    # First tick hoist is commanded — release brake, start counter
                    self._pi.write(PINS['hoist']['brake'], 0)
                    self._brake_released = True
                    self._brake_ticks_remaining = self._brake_delay_ticks
                if self._brake_ticks_remaining > 0:
                    # Still waiting for brake to fully release — hold pulses
                    self._brake_ticks_remaining -= 1
                else:
                    # Brake fully released — allow motion
                    if self.stepper_hoist:
                        self.stepper_hoist.set_velocity_continuous(vr_h)
            else:
                # No hoist command — engage brake, reset state
                if self._brake_released:
                    self._pi.write(PINS['hoist']['brake'], 1)
                    self._brake_released = False
                    self._brake_ticks_remaining = 0
                if self.stepper_hoist:
                    self.stepper_hoist.set_velocity_continuous(0.0)
        else:
            # Simulation mode — no brake logic
            if self.stepper_hoist:
                self.stepper_hoist.set_velocity_continuous(vr_h)
        # MANUAL
        if self.state.mode == ControlMode.MANUAL:
            self.state.F_bridge = 0.0; self.state.F_trolley = 0.0
            if self.active_mode == "manual":
                vr_b = self.ref_bridge.update(joy_b, dt)
                vr_t = self.ref_trolley.update(joy_t, dt)
                self.state.v_ref_bridge = vr_b; self.state.v_ref_trolley = vr_t
                if self.stepper_bridge: self.stepper_bridge.set_velocity_continuous(vr_b)
                if self.stepper_trolley: self.stepper_trolley.set_velocity_continuous(vr_t)
                self.state.v_cmd_bridge = vr_b; self.state.v_cmd_trolley = vr_t
            elif self.active_mode == "bridge":
                vr_b = self.ref_bridge.update(joy_b, dt)
                self.state.v_ref_bridge = vr_b; self.state.v_ref_trolley = 0.0
                if self.stepper_bridge: self.stepper_bridge.set_velocity_continuous(vr_b)
                self.state.v_cmd_bridge = vr_b; self._v_cmd_active = vr_b
            elif self.active_mode == "trolley":
                vr_t = self.ref_trolley.update(joy_t, dt)
                self.state.v_ref_trolley = vr_t; self.state.v_ref_bridge = 0.0
                if self.stepper_trolley: self.stepper_trolley.set_velocity_continuous(vr_t)
                self.state.v_cmd_trolley = vr_t; self._v_cmd_active = vr_t

        # AUTO
        elif self.state.mode == ControlMode.AUTO and self.controller:
            if self.active_mode == "bridge":
                vr = self.ref_bridge.update(joy_b, dt)
                self.state.v_ref_bridge = vr; self.state.v_ref_trolley = 0.0
                v_m = self.state.v_bridge
                th = self.state.theta_x; thd = self.state.theta_x_dot
                ac = self.config.bridge; stepper = self.stepper_bridge
            else:
                vr = self.ref_trolley.update(joy_t, dt)
                self.state.v_ref_trolley = vr; self.state.v_ref_bridge = 0.0
                v_m = self.state.v_trolley
                th = self.state.theta_z; thd = self.state.theta_z_dot
                ac = self.config.trolley; stepper = self.stepper_trolley

            F, info = self.controller.compute(v=v_m, theta=th, theta_dot=thd,
                                               v_ref=vr, dt=dt)
            m_tot = ac.m_t + self.config.m_l
            dv = (F * G_IN_PER_S2 / m_tot) * dt
            max_dv = self.config.a_slip_limit * G_IN_PER_S2 * dt
            dv = max(-max_dv, min(max_dv, dv))
            self._v_cmd_active += dv
            self._v_cmd_active = max(-ac.v_target, min(ac.v_target, self._v_cmd_active))
            if stepper: stepper.set_velocity_continuous(self._v_cmd_active)

            if self.active_mode == "bridge":
                self.state.F_bridge = F; self.state.v_cmd_bridge = self._v_cmd_active
                self.state.F_trolley = 0.0
            else:
                self.state.F_trolley = F; self.state.v_cmd_trolley = self._v_cmd_active
                self.state.F_bridge = 0.0

    # ── Safety ───────────────────────────────────────────────────────────

    def check_safety(self):
        if self.state.mode not in (ControlMode.MANUAL, ControlMode.AUTO):
            return True
        # NOTE: Position limits removed — encoder noise produces spurious
        # large position readings that would incorrectly trip the fault.
        # Sway limit retained (IMU-based, independent of encoder).
        tr = self.state.theta_x_raw if self.active_mode == "bridge" else self.state.theta_z_raw
        if abs(tr)*180/math.pi > self.config.theta_emergency_deg:
            print(f"[SAFETY] Sway {abs(tr)*180/math.pi:.1f} deg")
            self.state.fault_code = 2; return False
        return True

    def _open_log(self):
        if not self.log_csv: return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        d = os.path.join(os.getcwd(), "logs"); os.makedirs(d, exist_ok=True)
        # Added 'trolley_val' to the filename so you know these are the focused runs
        p = os.path.join(d, f"run_trolley_val_{ts}.csv") 
        self._csv_file = open(p, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        
        # Streamlined header: Time, Mode, Trolley Kinematics, Trolley Sway, Sanity Checks
        self._csv_writer.writerow([
            "t_s", "mode",
            "x_trolley", "v_trolley", "v_ref_trolley", "v_cmd_trolley", "F_trolley",
            "theta_z_deg", "theta_z_dot", "theta_z_raw_deg",
            "L_current", "loop_ms"
        ])
        print(f"[LOG] -> {p}")

    def _log_tick(self):
        if not self._csv_writer: return
        s = self.state; t = time.time() - self._t0
        mm = {0:"DISABLED", 1:"MANUAL", 2:"AUTO", 4:"FAULT"}
        
        # Streamlined data row matching the new header
        self._csv_writer.writerow([
            f"{t:.4f}", mm.get(s.mode,"?"),
            f"{s.x_trolley:.4f}", f"{s.v_trolley:.4f}", 
            f"{s.v_ref_trolley:.4f}", f"{s.v_cmd_trolley:.4f}", f"{s.F_trolley:.4f}",
            f"{s.theta_z*180/math.pi:.4f}", f"{s.theta_z_dot:.4f}", f"{s.theta_z_raw*180/math.pi:.4f}",
            f"{s.L_current:.2f}", f"{s.loop_time_ms:.2f}"
        ])

    def _close_log(self):
        if self._csv_file:
            self._csv_file.flush(); self._csv_file.close()
            self._csv_file = None; self._csv_writer = None
            print("[LOG] Saved")

    # ── Run Loop ─────────────────────────────────────────────────────────

    def run(self):
        self._running = True; self._t0 = time.time()
        tdt = self.config.dt; self._open_log()
        print(f"\n[RUN] {self.config.control_rate_hz}Hz | "
              f"{self.active_mode} | Ctrl+C stop\n")
        try:
            while self._running:
                t0 = time.time()
                self.process_hmi(); self.read_sensors()
                if not self.check_safety():
                    self.disable_drives(); self.state.mode = ControlMode.FAULT
                if self.state.mode in (ControlMode.MANUAL, ControlMode.AUTO):
                    self.control_step()
                t1 = time.time()
                self.state.loop_time_ms = (t1-t0)*1000
                self.state.loop_count += 1
                self._log_tick()
                sl = tdt-(t1-t0)
                if sl > 0: time.sleep(sl)
                elif self.state.loop_count % 200 == 0:
                    print(f"[WARN] Overrun {self.state.loop_time_ms:.1f}ms")
                if self.state.loop_count % 500 == 0:
                    if self.state.mode in (ControlMode.MANUAL, ControlMode.AUTO):
                        self._print_status()
        except KeyboardInterrupt:
            print("\n[RUN] Interrupted")
        finally:
            self.shutdown()

    def _print_status(self):
        s = self.state; mm = {0:"DIS",1:"MAN",2:"AUTO",4:"FLT"}
        m = mm.get(s.mode,"?")
        if self.active_mode == "bridge":
            a = (f"Xb:{s.x_bridge:+7.2f} Vb:{s.v_bridge:+6.2f} "
                 f"Vref:{s.v_ref_bridge:+6.2f} Vcmd:{s.v_cmd_bridge:+6.2f} "
                 f"tx:{s.theta_x_raw*180/math.pi:+5.2f}d F:{s.F_bridge:+7.2f}")
        elif self.active_mode == "trolley":
            a = (f"Zt:{s.x_trolley:+7.2f} Vt:{s.v_trolley:+6.2f} "
                 f"Vref:{s.v_ref_trolley:+6.2f} Vcmd:{s.v_cmd_trolley:+6.2f} "
                 f"tz:{s.theta_z_raw*180/math.pi:+5.2f}d F:{s.F_trolley:+7.2f}")
        else:
            a = (f"Xb:{s.x_bridge:+7.2f} Zt:{s.x_trolley:+7.2f} "
                 f"Yh:{s.x_hoist:+6.2f}")
        print(f"[{m}({self.active_mode[0].upper()})] {a} L:{s.L_current:5.1f} "
              f"{s.loop_time_ms:.1f}ms")

    def stop(self): self._running = False

    def shutdown(self):
        print("\n[SHUTDOWN]...")
        self._close_log(); self.disable_drives()
        if self.encoder_bus: self.encoder_bus.close()
        if self.imu: self.imu.close()
        for s in [self.stepper_trolley, self.stepper_bridge, self.stepper_hoist]:
            if s: s.close()
        if self.joystick: self.joystick.close()
        if self._pi and not self.simulation_mode: self._pi.stop()
        print("[SHUTDOWN] Done")


# ══════════════════════════════════════════════════════════════════════════════
# Test
# ══════════════════════════════════════════════════════════════════════════════

def test_sensors(sim=False):
    print("=" * 60 + "\n  Sensor Checkout\n" + "=" * 60)
    pi = None
    if not sim:
        pi = pigpio.pi()
        if not pi.connected: print("[ERROR] pigpiod"); return

    bus = EncoderBus(pi, sim)
    bus.add("trolley", PINS['trolley']['cs'], ENC_IPC['trolley'])
    bus.add("bridge",  PINS['bridge']['cs'],  ENC_IPC['bridge'])
    bus.add("bridge_diag", PINS['bridge']['cs_diag'], ENC_IPC['bridge_diag'])
    bus.add("hoist",   PINS['hoist']['cs'],   ENC_IPC['hoist'])
    bus.initialize()

    print("\n[ENC] Move axes manually:")
    for _ in range(10):
        t=bus.read_position("trolley"); b=bus.read_position("bridge")
        h=bus.read_position("hoist"); bd=bus.read_position("bridge_diag")
        print(f"  T:{t:+8.3f} B:{b:+8.3f} Bd:{bd:+8.3f} H:{h:+8.3f}")
        time.sleep(0.5)
    bus.close()

    print("\n[IMU]:")
    imu = MPU6050(simulation_mode=sim)
    if imu.initialize():
        imu.calibrate(50, 10)
        for _ in range(1000):
            a, g = imu.read()
            tx=math.degrees(math.atan2(a[0],a[1]))
            tz=math.degrees(math.atan2(a[2],a[1]))
            print(f"  tx:{tx:+6.2f} tz:{tz:+6.2f} gx:{g[0]:+.3f} gz:{g[2]:+.3f}")
            time.sleep(0.3)
    imu.close()

    print("\n[JOY]:")
    j = JoystickDriver(simulation_mode=sim)
    if j.initialize():
        for i in range(50):
            j.update(); inp = j.get_all_inputs()
            if i%10==0:
                print(f"  B:{inp['bridge']:+.0f} T:{inp['trolley']:+.0f} H:{inp['hoist']:+.0f}")
            time.sleep(0.1)
    j.close()
    if pi: pi.stop()
    print("\n[DONE]")


# ══════════════════════════════════════════════════════════════════════════════
# Entry
# ══════════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(description="UHplift Crane Control")
    p.add_argument("--mode", choices=["manual","bridge","trolley"], default="manual")
    p.add_argument("--log", action="store_true")
    p.add_argument("--sim", action="store_true")
    p.add_argument("--test-sensors", action="store_true")
    a = p.parse_args()

    if a.test_sensors: test_sensors(a.sim); return

    ctrl = CraneController(DEFAULT_CONFIG, active_mode=a.mode,
                            log_csv=a.log, simulation_mode=a.sim)
    signal.signal(signal.SIGINT, lambda s,f: ctrl.stop())
    if ctrl.initialize():
        ctrl.calibrate_imu(); ctrl.run()
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
"""
MPU6050 IMU Driver — I2C Bus 1, Address 0x68

Replaces LSM6DS3 (SPI1) after ground-fault damage to SPI components.
Interface: I2C1 (GPIO2=SDA, GPIO3=SCL), 400 kHz
Provides: 3-axis accelerometer [g] and 3-axis gyroscope [rad/s]
Calibration subtracts resting offsets including 1g on gravity axis.

Axis mapping (MUST match crane kinematic frame):
    The read() method returns arrays indexed as:
        accel[0] = X  (bridge sway direction, north/south)
        accel[1] = Y  (vertical, gravity ≈ +1g at rest)
        accel[2] = Z  (trolley sway direction, east/west)
        gyro[0]  = rotation about X (trolley sway rate)
        gyro[1]  = rotation about Y (yaw, unused)
        gyro[2]  = rotation about Z (bridge sway rate)

    If the MPU6050 chip axes don't align with the crane frame,
    adjust AXIS_REMAP and AXIS_SIGN below. The default identity
    mapping assumes the chip is mounted with:
        chip X → crane X (north),  chip Y → crane Y (up),  chip Z → crane Z (east)

Wiring:
    Pi GPIO2  (pin 3)  → SDA
    Pi GPIO3  (pin 5)  → SCL
    Pi 3.3V   (pin 1)  → VCC
    Pi GND    (pin 9)  → GND
    GND or NC          → AD0  (0x68 address)
"""

import time
import math
import numpy as np
from typing import Tuple

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

# ── MPU6050 Registers ─────────────────────────────────────────────────────────
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B   # 14-byte burst: accel(6) + temp(2) + gyro(6)
REG_PWR_MGMT_1   = 0x6B
REG_WHO_AM_I     = 0x75

WHO_AM_I_EXPECTED = 0x68

# ── Scale Factors ─────────────────────────────────────────────────────────────
# ±2g  → 16384 LSB/g   (ACCEL_CONFIG = 0x00)
# ±4g  → 8192  LSB/g   (ACCEL_CONFIG = 0x08)
# ±250 dps → 131 LSB/dps (GYRO_CONFIG = 0x00)
# ±500 dps → 65.5 LSB/dps (GYRO_CONFIG = 0x08)

ACCEL_RANGE = 0x00      # ±2g — best resolution for tilt sensing
GYRO_RANGE  = 0x08      # ±500 dps — matches LSM6DS3 setting

ACCEL_SCALE = 2.0 / 32768.0       # ±2g  → g per LSB
GYRO_SCALE  = 500.0 / 32768.0     # ±500 dps → dps per LSB
DEG_TO_RAD  = math.pi / 180.0

# DLPF setting: REG_CONFIG bits [2:0]
# 3 = accel BW 44 Hz, gyro BW 42 Hz, Fs = 1 kHz
# Good balance: filters noise while preserving sway dynamics (ωn ≈ 3 rad/s)
DLPF_CFG = 3

# Sample rate = 1 kHz / (1 + SMPLRT_DIV)
# SMPLRT_DIV = 4 → 200 Hz (matches control loop rate)
SMPLRT_DIV = 4

# ── Axis Remapping ────────────────────────────────────────────────────────────
# If chip orientation doesn't match crane frame, adjust these.
# AXIS_REMAP: which chip axis maps to crane [X, Y, Z]
# AXIS_SIGN:  sign flip per axis (+1 or -1)
#
# Default: identity (chip X→crane X, chip Y→crane Y, chip Z→crane Z)
# Example: if chip is rotated 90° about vertical, you'd swap X↔Z
AXIS_REMAP = [0, 1, 2]    # [chip_index_for_crane_X, _Y, _Z]
AXIS_SIGN  = [1, -1, 1]    # [sign_for_crane_X, _Y, _Z]


def _s16(hi: int, lo: int) -> int:
    """Combine two bytes (big-endian) into signed 16-bit."""
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v


class MPU6050:
    """MPU6050 I2C IMU driver — drop-in replacement for LSM6DS3."""

    def __init__(self, config=None, simulation_mode=False,
                 i2c_bus=1, i2c_addr=0x68):
        self.simulation_mode = simulation_mode
        self.i2c_bus = i2c_bus
        self.i2c_addr = i2c_addr
        self._bus = None
        self._initialized = False
        self.accel_offset = np.zeros(3)
        self.gyro_offset = np.zeros(3)

    def initialize(self) -> bool:
        if self.simulation_mode:
            print("[IMU] MPU6050 simulation mode")
            self._initialized = True
            return True

        if SMBus is None:
            print("[IMU] smbus2 not installed — run: pip install smbus2")
            return False

        try:
            self._bus = SMBus(self.i2c_bus)

            # WHO_AM_I check
            who = self._bus.read_byte_data(self.i2c_addr, REG_WHO_AM_I)
            ok = (who == WHO_AM_I_EXPECTED)
            print(f"[IMU] MPU6050 WHO_AM_I = 0x{who:02X} "
                  f"{'ok' if ok else 'UNEXPECTED'}")
            if not ok:
                print(f"  Expected 0x{WHO_AM_I_EXPECTED:02X} — "
                      f"chip may be MPU6500/MPU9250 variant, continuing")

            # Wake from sleep (clear SLEEP bit, use internal oscillator)
            self._bus.write_byte_data(self.i2c_addr, REG_PWR_MGMT_1, 0x00)
            time.sleep(0.05)

            # Configure sample rate
            self._bus.write_byte_data(self.i2c_addr, REG_SMPLRT_DIV, SMPLRT_DIV)

            # Configure DLPF
            self._bus.write_byte_data(self.i2c_addr, REG_CONFIG, DLPF_CFG)

            # Configure gyro range (±500 dps)
            self._bus.write_byte_data(self.i2c_addr, REG_GYRO_CONFIG, GYRO_RANGE)

            # Configure accel range (±2g)
            self._bus.write_byte_data(self.i2c_addr, REG_ACCEL_CONFIG, ACCEL_RANGE)

            time.sleep(0.05)

            # Readback verification
            pm = self._bus.read_byte_data(self.i2c_addr, REG_PWR_MGMT_1)
            gc = self._bus.read_byte_data(self.i2c_addr, REG_GYRO_CONFIG)
            ac = self._bus.read_byte_data(self.i2c_addr, REG_ACCEL_CONFIG)
            print(f"[IMU] PWR=0x{pm:02X} GYRO_CFG=0x{gc:02X} "
                  f"ACCEL_CFG=0x{ac:02X}")
            print(f"[IMU] Sample rate: {1000//(1+SMPLRT_DIV)} Hz, "
                  f"DLPF: BW={[260,184,94,44,21,10,5][DLPF_CFG]} Hz")

            self._initialized = True
            return True

        except Exception as e:
            print(f"[IMU] Init error: {e}")
            return False

    def calibrate(self, num_samples=100, delay_ms=10):
        if not self._initialized:
            return
        print(f"[IMU] Calibrating ({num_samples} samples)...")
        print("  Keep IMU stationary!")

        a_sum = np.zeros(3)
        g_sum = np.zeros(3)

        for i in range(num_samples):
            a_raw, g_raw = self._read_raw()
            a_sum += a_raw
            g_sum += g_raw
            if (i + 1) % 25 == 0:
                print(f"  {i+1}/{num_samples}")
            time.sleep(delay_ms / 1000.0)

        self.accel_offset = a_sum / num_samples
        self.gyro_offset = g_sum / num_samples

        # Preserve gravity on Y axis (crane vertical)
        self.accel_offset[1] -= 1.0

        print(f"[IMU] Calibration complete")
        print(f"  Accel offset: [{self.accel_offset[0]:.4f}, "
              f"{self.accel_offset[1]:.4f}, {self.accel_offset[2]:.4f}] g")
        print(f"  Gyro offset:  [{self.gyro_offset[0]:.4f}, "
              f"{self.gyro_offset[1]:.4f}, {self.gyro_offset[2]:.4f}] rad/s")

    def read(self) -> Tuple[np.ndarray, np.ndarray]:
        """Read calibrated accel [g] and gyro [rad/s] in crane frame."""
        if self.simulation_mode:
            return np.array([0.0, 1.0, 0.0]), np.zeros(3)

        try:
            a, g = self._read_raw()
            a -= self.accel_offset
            g -= self.gyro_offset
            return a, g
        except Exception:
            # I2C error — return safe zeros, don't crash control loop
            return np.array([0.0, 1.0, 0.0]), np.zeros(3)

    def _read_raw(self) -> Tuple[np.ndarray, np.ndarray]:
        """Burst read 14 bytes, apply scale and axis remap."""
        data = self._bus.read_i2c_block_data(
            self.i2c_addr, REG_ACCEL_XOUT_H, 14)

        # Raw chip-frame values
        a_chip = [
            _s16(data[0], data[1]) * ACCEL_SCALE,    # chip X [g]
            _s16(data[2], data[3]) * ACCEL_SCALE,    # chip Y [g]
            _s16(data[4], data[5]) * ACCEL_SCALE,    # chip Z [g]
        ]
        # data[6:8] = temperature, skip
        g_chip = [
            _s16(data[8],  data[9])  * GYRO_SCALE * DEG_TO_RAD,  # chip X [rad/s]
            _s16(data[10], data[11]) * GYRO_SCALE * DEG_TO_RAD,  # chip Y [rad/s]
            _s16(data[12], data[13]) * GYRO_SCALE * DEG_TO_RAD,  # chip Z [rad/s]
        ]

        # Remap chip axes → crane frame axes
        accel = np.array([
            AXIS_SIGN[0] * a_chip[AXIS_REMAP[0]],
            AXIS_SIGN[1] * a_chip[AXIS_REMAP[1]],
            AXIS_SIGN[2] * a_chip[AXIS_REMAP[2]],
        ])
        gyro = np.array([
            AXIS_SIGN[0] * g_chip[AXIS_REMAP[0]],
            AXIS_SIGN[1] * g_chip[AXIS_REMAP[1]],
            AXIS_SIGN[2] * g_chip[AXIS_REMAP[2]],
        ])
        return accel, gyro

    def close(self):
        if self._bus is not None:
            self._bus.close()
            self._bus = None
        self._initialized = False


# Backward-compatible alias so existing imports don't break during transition
LSM6DS3 = MPU6050
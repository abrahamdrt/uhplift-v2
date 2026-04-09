"""
LSM6DS3 IMU Driver — SPI1, CE0 (GPIO 18), Mode 0

Interface: SPI1 auxiliary controller, Mode 0 (aux rejects Mode 3)
Provides: 3-axis accelerometer [g] and 3-axis gyroscope [rad/s]
Calibration subtracts resting offsets including 1g on Y (vertical).

Axis mapping (per physical mounting):
    accel[0] = X (bridge sway direction)
    accel[1] = Y (vertical / gravity)
    accel[2] = Z (trolley sway direction)
    gyro[0]  = rotation about X (trolley sway rate)
    gyro[1]  = rotation about Y (yaw)
    gyro[2]  = rotation about Z (bridge sway rate)
"""

import time
import numpy as np
from typing import Optional, Tuple

try:
    import spidev
except ImportError:
    spidev = None

# ── LSM6DS3 Registers ────────────────────────────────────────────────────────
REG_WHO_AM_I   = 0x0F
REG_CTRL1_XL   = 0x10
REG_CTRL2_G    = 0x11
REG_CTRL3_C    = 0x12
REG_OUT_TEMP_L = 0x20
REG_OUTX_L_G   = 0x22
REG_OUTX_L_XL  = 0x28

WHO_AM_I_VALUE = 0x69

# CTRL1_XL: 208 Hz, ±4g, anti-alias 100 Hz
CTRL1_XL_SETUP = 0x58
# CTRL2_G: 208 Hz, ±500 dps
CTRL2_G_SETUP  = 0x54

ACCEL_SCALE = 4.0 / 32768.0    # ±4g
GYRO_SCALE  = 500.0 / 32768.0  # ±500 dps → dps
DEG_TO_RAD  = np.pi / 180.0


class LSM6DS3:
    def __init__(self, config=None, simulation_mode=False,
                 spi_bus=1, spi_ce=0):
        self.simulation_mode = simulation_mode
        self.spi_bus = spi_bus
        self.spi_ce = spi_ce
        self._spi = None
        self._initialized = False
        self.accel_offset = np.zeros(3)
        self.gyro_offset = np.zeros(3)

    def initialize(self) -> bool:
        if self.simulation_mode:
            print("[IMU] Simulation mode"); self._initialized = True; return True
        try:
            self._spi = spidev.SpiDev()
            self._spi.open(self.spi_bus, self.spi_ce)
            self._spi.mode = 0b00
            self._spi.max_speed_hz = 1_000_000

            who = self._read_reg(REG_WHO_AM_I)
            print(f"[IMU] WHO_AM_I = 0x{who:02X} "
                  f"{'✓' if who == WHO_AM_I_VALUE else '✗'}")
            if who != WHO_AM_I_VALUE:
                return False

            self._write_reg(REG_CTRL1_XL, CTRL1_XL_SETUP)
            self._write_reg(REG_CTRL2_G, CTRL2_G_SETUP)
            time.sleep(0.05)

            c1 = self._read_reg(REG_CTRL1_XL)
            c2 = self._read_reg(REG_CTRL2_G)
            print(f"[IMU] CTRL1_XL=0x{c1:02X}, CTRL2_G=0x{c2:02X}")
            self._initialized = True
            return True
        except Exception as e:
            print(f"[IMU] Init error: {e}"); return False

    def calibrate(self, num_samples=100, delay_ms=10):
        if not self._initialized: return
        print(f"[IMU] Calibrating with {num_samples} samples...")
        print("  → Keep IMU stationary!")
        ax_sum = np.zeros(3); gx_sum = np.zeros(3)
        for i in range(num_samples):
            a, g = self._read_raw()
            ax_sum += a; gx_sum += g
            if (i+1) % 25 == 0: print(f"  → {i+1}/{num_samples}")
            time.sleep(delay_ms / 1000.0)
        self.accel_offset = ax_sum / num_samples
        self.gyro_offset = gx_sum / num_samples
        # Keep gravity on Y axis (vertical)
        self.accel_offset[1] -= 1.0
        print(f"[IMU] Calibration complete")
        print(f"  Accel offset: [{self.accel_offset[0]:.4f}, "
              f"{self.accel_offset[1]:.4f}, {self.accel_offset[2]:.4f}] g")
        print(f"  Gyro offset:  [{self.gyro_offset[0]:.4f}, "
              f"{self.gyro_offset[1]:.4f}, {self.gyro_offset[2]:.4f}] rad/s")

    def read(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.simulation_mode:
            return np.array([0.0, 1.0, 0.0]), np.zeros(3)
        a, g = self._read_raw()
        a -= self.accel_offset
        g -= self.gyro_offset
        return a, g

    def _read_raw(self) -> Tuple[np.ndarray, np.ndarray]:
        # Burst read: 12 bytes starting at OUTX_L_G (gyro then accel)
        data = self._read_burst(REG_OUTX_L_G, 12)
        gx = self._to_int16(data[0], data[1]) * GYRO_SCALE * DEG_TO_RAD
        gy = self._to_int16(data[2], data[3]) * GYRO_SCALE * DEG_TO_RAD
        gz = self._to_int16(data[4], data[5]) * GYRO_SCALE * DEG_TO_RAD
        # Accel is at offset +6
        data2 = self._read_burst(REG_OUTX_L_XL, 6)
        ax = self._to_int16(data2[0], data2[1]) * ACCEL_SCALE
        ay = self._to_int16(data2[2], data2[3]) * ACCEL_SCALE
        az = self._to_int16(data2[4], data2[5]) * ACCEL_SCALE
        return np.array([ax, ay, az]), np.array([gx, gy, gz])

    def _read_reg(self, reg):
        r = self._spi.xfer2([reg | 0x80, 0x00])
        return r[1]

    def _write_reg(self, reg, val):
        self._spi.xfer2([reg & 0x7F, val])

    def _read_burst(self, start_reg, count):
        tx = [start_reg | 0x80] + [0x00] * count
        rx = self._spi.xfer2(tx)
        return rx[1:]

    @staticmethod
    def _to_int16(lo, hi):
        v = (hi << 8) | lo
        return v - 65536 if v >= 32768 else v

    def close(self):
        if self._spi:
            self._spi.close(); self._spi = None
        self._initialized = False

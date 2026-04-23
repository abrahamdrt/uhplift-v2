"""
MPU6050 IMU Driver — I2C Bus 1, Address 0x68
Hardened for 200 Hz control loop over long cable runs.

Interface: I2C1 (GPIO2=SDA, GPIO3=SCL), 400 kHz recommended
Provides: 3-axis accelerometer [g] and 3-axis gyroscope [rad/s]

CRITICAL: Set I2C bus to 400 kHz in /boot/firmware/config.txt:
    dtparam=i2c_arm=on,i2c_arm_baudrate=400000

Axis mapping (crane kinematic frame):
    accel[0] = X  (bridge sway, north/south)
    accel[1] = Y  (vertical, gravity +1g at rest)
    accel[2] = Z  (trolley sway, east/west)
    gyro[0]  = rotation about X (trolley sway rate)
    gyro[1]  = rotation about Y (yaw, unused)
    gyro[2]  = rotation about Z (bridge sway rate)

Wiring:
    Pi GPIO2 (pin 3)  -> SDA
    Pi GPIO3 (pin 5)  -> SCL
    Pi 3.3V  (pin 1)  -> VCC
    Pi GND   (pin 9)  -> GND
    GND or NC         -> AD0 (0x68 address)
"""

import time
import math
import struct
import numpy as np
from typing import Tuple, Optional

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

# ── MPU6050 Registers ─────────────────────────────────────────────────────────
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1   = 0x6B
REG_WHO_AM_I     = 0x75

WHO_AM_I_EXPECTED = 0x68

# ── Configuration ─────────────────────────────────────────────────────────────
ACCEL_RANGE = 0x00       # +/-2g
GYRO_RANGE  = 0x08       # +/-500 dps
ACCEL_SCALE = 2.0 / 32768.0
GYRO_SCALE  = 500.0 / 32768.0
DEG_TO_RAD  = math.pi / 180.0
DLPF_CFG    = 3          # Accel BW 44 Hz, Gyro BW 42 Hz
SMPLRT_DIV  = 4          # 1000/(1+4) = 200 Hz

# I2C kernel timeout (units of 10ms). 2 = 20ms max per transaction.
I2C_TIMEOUT_IOCTL = 0x0702
I2C_TIMEOUT_VALUE = 2

# Axis remapping: chip axis index -> crane frame [X, Y, Z]
AXIS_REMAP = [0, 1, 2]
AXIS_SIGN  = [1, 1, 1]


def _s16(hi: int, lo: int) -> int:
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v


class MPU6050:
    """MPU6050 I2C IMU driver — hardened for 200 Hz control loop."""

    def __init__(self, config=None, simulation_mode=False,
                 i2c_bus=1, i2c_addr=0x68):
        self.simulation_mode = simulation_mode
        self.i2c_bus = i2c_bus
        self.i2c_addr = i2c_addr
        self._bus = None
        self._initialized = False
        self.accel_offset = np.zeros(3)
        self.gyro_offset = np.zeros(3)

        # Last-known-good values (returned on read failure)
        self._last_accel = np.array([0.0, 1.0, 0.0])
        self._last_gyro  = np.zeros(3)

        # Error rate limiting
        self._error_count = 0
        self._last_error_print = 0.0
        self._consecutive_fails = 0

    def initialize(self) -> bool:
        if self.simulation_mode:
            print("[IMU] MPU6050 simulation mode")
            self._initialized = True
            return True

        if SMBus is None:
            print("[IMU] smbus2 not installed — pip install smbus2")
            return False

        try:
            self._bus = SMBus(self.i2c_bus)

            # Set kernel I2C timeout to prevent indefinite blocking
            try:
                import fcntl
                fcntl.ioctl(self._bus.fd, I2C_TIMEOUT_IOCTL, I2C_TIMEOUT_VALUE)
                print(f"[IMU] I2C timeout set to {I2C_TIMEOUT_VALUE * 10}ms")
            except Exception:
                print("[IMU] Could not set I2C timeout (non-critical)")

            # WHO_AM_I
            who = self._bus.read_byte_data(self.i2c_addr, REG_WHO_AM_I)
            ok = (who == WHO_AM_I_EXPECTED)
            print(f"[IMU] MPU6050 WHO_AM_I = 0x{who:02X} "
                  f"{'ok' if ok else 'UNEXPECTED'}")

            # Wake (clear SLEEP, use internal 8 MHz clock)
            self._bus.write_byte_data(self.i2c_addr, REG_PWR_MGMT_1, 0x00)
            time.sleep(0.05)

            # Configure
            self._bus.write_byte_data(self.i2c_addr, REG_SMPLRT_DIV, SMPLRT_DIV)
            self._bus.write_byte_data(self.i2c_addr, REG_CONFIG, DLPF_CFG)
            self._bus.write_byte_data(self.i2c_addr, REG_GYRO_CONFIG, GYRO_RANGE)
            self._bus.write_byte_data(self.i2c_addr, REG_ACCEL_CONFIG, ACCEL_RANGE)
            time.sleep(0.05)

            # Verify
            gc = self._bus.read_byte_data(self.i2c_addr, REG_GYRO_CONFIG)
            ac = self._bus.read_byte_data(self.i2c_addr, REG_ACCEL_CONFIG)
            print(f"[IMU] GYRO_CFG=0x{gc:02X} ACCEL_CFG=0x{ac:02X}")
            print(f"[IMU] {1000//(1+SMPLRT_DIV)} Hz sample rate, "
                  f"DLPF BW={[260,184,94,44,21,10,5][DLPF_CFG]} Hz")

            # Quick test read to confirm data path works
            test = self._bus.read_i2c_block_data(self.i2c_addr, REG_ACCEL_XOUT_H, 14)
            if len(test) == 14:
                print("[IMU] Test read OK (14 bytes)")
            else:
                print(f"[IMU] Test read returned {len(test)} bytes (expected 14)")

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
        good = 0

        for i in range(num_samples):
            try:
                a_raw, g_raw = self._read_raw()
                a_sum += a_raw
                g_sum += g_raw
                good += 1
            except Exception:
                pass  # Skip failed reads during calibration
            if (i + 1) % 25 == 0:
                print(f"  {i+1}/{num_samples} ({good} good)")
            time.sleep(delay_ms / 1000.0)

        if good < 10:
            print(f"[IMU] Calibration failed — only {good}/{num_samples} reads succeeded")
            return

        self.accel_offset = a_sum / good
        self.gyro_offset = g_sum / good
        self.accel_offset[1] -= 1.0  # Preserve gravity on Y

        print(f"[IMU] Calibration complete ({good}/{num_samples} samples)")
        print(f"  Accel offset: [{self.accel_offset[0]:.4f}, "
              f"{self.accel_offset[1]:.4f}, {self.accel_offset[2]:.4f}] g")
        print(f"  Gyro offset:  [{self.gyro_offset[0]:.4f}, "
              f"{self.gyro_offset[1]:.4f}, {self.gyro_offset[2]:.4f}] rad/s")

    def read(self) -> Tuple[np.ndarray, np.ndarray]:
        """Read calibrated accel [g] and gyro [rad/s] in crane frame.
        Returns last-known-good values on I2C failure. Never blocks >20ms."""
        if self.simulation_mode:
            return np.array([0.0, 1.0, 0.0]), np.zeros(3)

        try:
            a, g = self._read_raw()
            a -= self.accel_offset
            g -= self.gyro_offset

            # Cache as last-known-good
            self._last_accel = a.copy()
            self._last_gyro = g.copy()
            self._consecutive_fails = 0
            return a, g

        except Exception as e:
            self._error_count += 1
            self._consecutive_fails += 1

            # Rate-limited warning (max once per second)
            now = time.time()
            if now - self._last_error_print > 5.0:
                print(f"[IMU] I2C read failed ({self._consecutive_fails} consecutive, "
                      f"{self._error_count} total): {e}")
                self._last_error_print = now

            # Try to recover bus if many consecutive failures
            if self._consecutive_fails == 50:
                self._try_bus_recovery()

            return self._last_accel.copy(), self._last_gyro.copy()

    def _read_raw(self) -> Tuple[np.ndarray, np.ndarray]:
        """Burst read 14 bytes, apply scale and axis remap."""
        data = self._bus.read_i2c_block_data(
            self.i2c_addr, REG_ACCEL_XOUT_H, 14)

        a_chip = [
            _s16(data[0], data[1]) * ACCEL_SCALE,
            _s16(data[2], data[3]) * ACCEL_SCALE,
            _s16(data[4], data[5]) * ACCEL_SCALE,
        ]
        g_chip = [
            _s16(data[8],  data[9])  * GYRO_SCALE * DEG_TO_RAD,
            _s16(data[10], data[11]) * GYRO_SCALE * DEG_TO_RAD,
            _s16(data[12], data[13]) * GYRO_SCALE * DEG_TO_RAD,
        ]

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

    def _try_bus_recovery(self):
        """Attempt to recover a hung I2C bus by re-opening it."""
        print("[IMU] Attempting I2C bus recovery...")
        try:
            if self._bus:
                self._bus.close()
            time.sleep(0.01)
            self._bus = SMBus(self.i2c_bus)

            # Re-set kernel timeout
            try:
                import fcntl
                fcntl.ioctl(self._bus.fd, I2C_TIMEOUT_IOCTL, I2C_TIMEOUT_VALUE)
            except Exception:
                pass

            # Re-wake the MPU6050
            self._bus.write_byte_data(self.i2c_addr, REG_PWR_MGMT_1, 0x00)
            time.sleep(0.01)
            self._bus.write_byte_data(self.i2c_addr, REG_SMPLRT_DIV, SMPLRT_DIV)
            self._bus.write_byte_data(self.i2c_addr, REG_CONFIG, DLPF_CFG)
            self._bus.write_byte_data(self.i2c_addr, REG_GYRO_CONFIG, GYRO_RANGE)
            self._bus.write_byte_data(self.i2c_addr, REG_ACCEL_CONFIG, ACCEL_RANGE)

            self._consecutive_fails = 0
            print("[IMU] Bus recovery succeeded")
        except Exception as e:
            print(f"[IMU] Bus recovery failed: {e}")

    def close(self):
        if self._bus is not None:
            self._bus.close()
            self._bus = None
        self._initialized = False


# Backward-compatible alias
LSM6DS3 = MPU6050
"""
imu_diag.py
===========
Live IMU diagnostic for axis convention verification.

Run this with the IMU mounted in its intended position on the hoist,
then tilt the board by hand to confirm sign and axis conventions
BEFORE committing any changes to estimator.py.

Usage:
    python3 imu_diag.py

Controls:
    Ctrl+C to exit

What to check:
    1. Stationary: accel_y should read ~1.0g (gravity on Y = hoist axis)
                   accel_x and accel_z should read ~0g
    2. Tilt in trolley sway direction (rotate in YZ plane):
       → theta_trolley should increase positively
       → gyro_x should show rate
    3. Tilt in bridge sway direction (rotate in XY plane):
       → theta_bridge should increase positively
       → gyro_z should show rate

If signs are flipped, note which axis to negate in estimator.py.

SPI config: bus=1, ce=0 (SPI1, GPIO18) — matches confirmed hardware setup.
"""

import sys
import os
import time
import numpy as np

# Allow running from any directory in the repo
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from imu import LSM6DS3, LSM6DS3Config

# ── Configuration ─────────────────────────────────────────────────────────────
SPI_BUS     = 1       # SPI1 (auxiliary controller)
SPI_CE      = 0       # CE0 = GPIO18
PRINT_HZ    = 10      # Display update rate [Hz]
CAL_SAMPLES = 100     # Calibration samples

ALPHA       = 0.98    # Complementary filter coefficient
DT          = 1.0 / PRINT_HZ

# ── Colour helpers ────────────────────────────────────────────────────────────
def _c(val, thresh_warn, thresh_bad, fmt="{:+.4f}"):
    """Colour a value green/yellow/red based on thresholds."""
    s = fmt.format(val)
    if abs(val) > thresh_bad:
        return f"\033[91m{s}\033[0m"   # red
    if abs(val) > thresh_warn:
        return f"\033[93m{s}\033[0m"   # yellow
    return f"\033[92m{s}\033[0m"       # green


def main():
    print("=" * 65)
    print("  UHplift IMU Diagnostic")
    print(f"  SPI bus={SPI_BUS}, CE={SPI_CE}")
    print("=" * 65)

    imu = LSM6DS3(
        config=LSM6DS3Config(),
        simulation_mode=False,
        spi_bus=SPI_BUS,
        spi_ce=SPI_CE,
    )

    if not imu.initialize():
        print("\n[ERROR] IMU failed to initialize. Check wiring and SPI config.")
        sys.exit(1)

    # ── Calibration ───────────────────────────────────────────────────────────
    print(f"\n[CAL] Keep IMU stationary — collecting {CAL_SAMPLES} samples...")
    imu.calibrate(num_samples=CAL_SAMPLES)
    print("[CAL] Done.\n")

    # ── Complementary filter state for both axes ───────────────────────────────
    theta_trolley = 0.0   # YZ plane — trolley sway
    theta_bridge  = 0.0   # XY plane — bridge sway

    print("  Tilt the board and verify signs match expected sway direction.")
    print("  Press Ctrl+C to exit.\n")

    header = (
        f"{'accel_x':>9} {'accel_y':>9} {'accel_z':>9}  "
        f"{'gyro_x':>9} {'gyro_y':>9} {'gyro_z':>9}  "
        f"{'θ_trol°':>9} {'θ_brdg°':>9}"
    )
    print(header)
    print("-" * len(header))

    period = 1.0 / PRINT_HZ

    try:
        while True:
            t0 = time.time()

            accel, gyro = imu.read()
            ax, ay, az = accel
            gx, gy, gz = gyro

            # ── Angle from accelerometer ──────────────────────────────────────
            # Trolley sway: rotation in YZ plane, gravity reference is Y
            theta_trol_accel = np.arctan2(az, ay)
            # Bridge sway: rotation in XY plane, gravity reference is Y
            theta_brdg_accel = np.arctan2(ax, ay)

            # ── Complementary filter ──────────────────────────────────────────
            theta_trolley = ALPHA * (theta_trolley + gx * DT) + (1 - ALPHA) * theta_trol_accel
            theta_bridge  = ALPHA * (theta_bridge  + gz * DT) + (1 - ALPHA) * theta_brdg_accel

            theta_trol_deg = np.degrees(theta_trolley)
            theta_brdg_deg = np.degrees(theta_bridge)

            # ── Print row ─────────────────────────────────────────────────────
            row = (
                f"  {_c(ax, 0.05, 0.3):>9}g"
                f" {_c(ay - 1.0, 0.05, 0.2):>9}g"   # offset by 1g — deviation from vertical
                f" {_c(az, 0.05, 0.3):>9}g  "
                f" {_c(gx, 0.05, 0.5, '{:+.3f}'):>9}"
                f" {_c(gy, 0.05, 0.5, '{:+.3f}'):>9}"
                f" {_c(gz, 0.05, 0.5, '{:+.3f}'):>9}  "
                f" {_c(theta_trol_deg, 0.5, 2.0, '{:+.2f}'):>9}°"
                f" {_c(theta_brdg_deg, 0.5, 2.0, '{:+.2f}'):>9}°"
            )
            print(row)

            # ── Rate limit display ────────────────────────────────────────────
            elapsed = time.time() - t0
            sleep_t = max(0, period - elapsed)
            time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n\n[EXIT] Stopped.")
        print("\nSummary — update estimator.py with confirmed axes:")
        print(f"  theta_trolley: arctan2(accel_z, accel_y),  rate: gyro_x")
        print(f"  theta_bridge:  arctan2(accel_x, accel_y),  rate: gyro_z")
        print("\nIf signs were flipped, negate the relevant accel argument.")
        imu.close()


if __name__ == "__main__":
    main()
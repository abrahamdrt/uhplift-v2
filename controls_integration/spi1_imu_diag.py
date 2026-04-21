#!/usr/bin/env python3
"""
SPI1 IMU Diagnostic — Isolates the EINVAL root cause
UHplift Team 11

The Pi 4's SPI1 uses the auxiliary SPI controller (spi-bcm2835aux),
which has different capabilities than SPI0 (spi-bcm2835):
  - Mode 3 may not be supported on some kernel versions
  - Clock speeds are limited to system_clock / (2*n) where n >= 1
  - CS polarity flags may be restricted

This script tests each parameter individually to find which one
throws [Errno 22] Invalid argument.

Wiring (SPI1 CE0 defaults):
    SCL  → GPIO21 (SCLK)
    SDA  → GPIO20 (MOSI)
    SA0  → GPIO19 (MISO)
    CS   → GPIO18 (CE0)
    Vin  → 3.3V
    GND  → GND

Run:
    python3 spi1_imu_diag.py
"""

import os
import sys

# ── Step 1: Check dtoverlay and device nodes ──────────────────────────────────
print("=" * 60)
print("  SPI1 IMU Diagnostic")
print("=" * 60)

print("\n[1] Checking device nodes...")
for dev in ["/dev/spidev0.0", "/dev/spidev0.1", "/dev/spidev1.0", "/dev/spidev1.1", "/dev/spidev1.2"]:
    exists = os.path.exists(dev)
    marker = "  ✓" if exists else "  -"
    print(f"  {marker} {dev}")

if not os.path.exists("/dev/spidev1.0"):
    print("\n  [ERROR] /dev/spidev1.0 not found.")
    print("  Add to /boot/firmware/config.txt (or /boot/config.txt):")
    print("    dtoverlay=spi1-1cs,cs0_pin=18")
    print("  Then reboot.")
    sys.exit(1)

print("\n[2] Checking dtoverlay configuration...")
for cfg_path in ["/boot/firmware/config.txt", "/boot/config.txt"]:
    if os.path.exists(cfg_path):
        with open(cfg_path, 'r') as f:
            lines = f.readlines()
        spi1_lines = [l.strip() for l in lines if 'spi1' in l.lower()]
        if spi1_lines:
            print(f"  Found in {cfg_path}:")
            for l in spi1_lines:
                print(f"    {l}")
        else:
            print(f"  [WARN] No spi1 overlay found in {cfg_path}")
        break

# ── Step 2: Test SPI1 open ────────────────────────────────────────────────────
import spidev

print("\n[3] Testing spidev(1, 0) open...")
try:
    spi = spidev.SpiDev()
    spi.open(1, 0)
    print("  ✓ spi.open(1, 0) succeeded")
except Exception as e:
    print(f"  [ERROR] spi.open(1, 0) failed: {e}")
    sys.exit(1)

# ── Step 3: Test each parameter independently ────────────────────────────────
print("\n[4] Testing SPI mode settings...")
for mode in [0, 1, 2, 3]:
    try:
        spi.mode = mode
        print(f"  ✓ Mode {mode} (0b{mode:02b}) accepted")
    except Exception as e:
        print(f"  ✗ Mode {mode} (0b{mode:02b}) REJECTED: {e}")

print("\n[5] Testing clock speeds...")
for speed in [100_000, 500_000, 1_000_000, 2_000_000, 4_000_000]:
    try:
        spi.max_speed_hz = speed
        actual = spi.max_speed_hz
        print(f"  ✓ {speed:>10,} Hz requested → {actual:>10,} Hz actual")
    except Exception as e:
        print(f"  ✗ {speed:>10,} Hz REJECTED: {e}")

# ── Step 4: Try WHO_AM_I read at each working mode ───────────────────────────
print("\n[6] WHO_AM_I read attempts (0x8F → expect 0x69)...")
spi.close()

for mode in [0, 3]:
    for speed in [100_000, 1_000_000]:
        try:
            spi = spidev.SpiDev()
            spi.open(1, 0)
            spi.max_speed_hz = speed
            spi.mode = mode
            resp = spi.xfer2([0x8F, 0x00])
            result = "✓ WHO_AM_I" if resp[1] == 0x69 else f"got 0x{resp[1]:02X}"
            print(f"  Mode {mode}, {speed//1000}kHz: [{hex(resp[0])}, {hex(resp[1])}] {result}")
            spi.close()
        except Exception as e:
            print(f"  Mode {mode}, {speed//1000}kHz: FAILED — {e}")
            try:
                spi.close()
            except:
                pass

# ── Step 5: If Mode 3 fails, test the Mode 0 + CPOL inversion workaround ────
print("\n[7] Workaround test: Mode 0 with manual CPOL idle...")
print("  (The aux SPI may work in Mode 0 if the LSM6DS3 also accepts it)")
print("  Note: LSM6DS3 datasheet specifies Mode 3, but some breakout boards")
print("  include a level shifter that may tolerate Mode 0.")

try:
    spi = spidev.SpiDev()
    spi.open(1, 0)
    spi.max_speed_hz = 1_000_000
    spi.mode = 0
    
    # Try WHO_AM_I
    resp = spi.xfer2([0x8F, 0x00])
    if resp[1] == 0x69:
        print(f"  ✓ MODE 0 WORKS! WHO_AM_I = 0x{resp[1]:02X}")
        print("  → The LSM6DS3 is responding in Mode 0 on SPI1.")
        print("  → Update imu.py: set spi.mode = 0 when using SPI1.")
    else:
        print(f"  ✗ Mode 0 returned 0x{resp[1]:02X} (not 0x69)")
    spi.close()
except Exception as e:
    print(f"  ✗ Mode 0 test failed: {e}")

# ── Step 6: Fallback recommendation ──────────────────────────────────────────
print("\n" + "=" * 60)
print("  RECOMMENDATIONS")
print("=" * 60)
print("""
If Mode 3 is REJECTED on SPI1:
  The Pi 4 auxiliary SPI (spi-bcm2835aux) driver may not support
  Mode 3 on your kernel version. You have two options:

  Option A (recommended): Keep IMU on SPI0 CE1 (GPIO7)
    The encoder chain uses SPI0 through the HCT244 level shifter
    with CS1-CS4. The IMU uses SPI0 CE1 (GPIO7) directly. Since
    SPI0 uses hardware chip-select and only one CS is active at
    a time, there is NO bus contention — the transactions are
    electrically independent. This is already validated working.
    
    In imu.py: spi.open(0, 1)  # SPI0, CE1

  Option B: Use SPI1 in Mode 0 (if WHO_AM_I passes above)
    Some LSM6DS3 breakouts tolerate Mode 0 despite the datasheet
    specifying Mode 3. If step [7] returned 0x69, this works.
    
    In imu.py: spi.open(1, 0); spi.mode = 0

  Option C: Upgrade kernel and retry
    sudo apt update && sudo apt full-upgrade
    sudo reboot
    The spi-bcm2835aux Mode 3 support was improved in recent kernels.
""")
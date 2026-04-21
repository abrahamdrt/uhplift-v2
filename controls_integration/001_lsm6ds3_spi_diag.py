#!/usr/bin/env python3
"""
IMU SPI Diagnostic
Checks: spidev device exists, CE1 responds, bus not stuck, MISO not floating
"""
import os, sys

# ── Step 1: Check spidev devices ─────────────────────────────────────────────
print("=== Step 1: spidev devices ===")
for dev in ["/dev/spidev0.0", "/dev/spidev0.1"]:
    exists = os.path.exists(dev)
    print(f"  {dev}: {'FOUND' if exists else 'MISSING'}")

if not os.path.exists("/dev/spidev0.1"):
    print("\n  /dev/spidev0.1 missing — CE1 not exposed.")
    print("  Add to /boot/config.txt:  dtparam=spi=on")
    print("  Then reboot and re-run.")
    sys.exit()

# ── Step 2: Raw WHO_AM_I read — try both modes ─────────────────────────────
import spidev

print("\n=== Step 2: WHO_AM_I raw read (0x8F 0x00) ===")
for mode in [0, 3]:
    spi = spidev.SpiDev()
    spi.open(0, 1)
    spi.max_speed_hz = 100_000   # slow speed for debug
    spi.mode = mode
    resp = spi.xfer2([0x8F, 0x00])
    spi.close()
    print(f"  Mode {mode}: sent [0x8F, 0x00]  got {[hex(b) for b in resp]}  "
          f"byte[1] = {hex(resp[1])}  {'OK' if resp[1] == 0x69 else '---'}")

# ── Step 3: Loopback — short MOSI(GPIO10) to MISO(GPIO9) ──────────────────
print("\n=== Step 3: SPI loopback (short GPIO10→GPIO9) ===")
print("  If you have MOSI and MISO shorted, the bytes below should match.")
spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz = 100_000
spi.mode = 3
test_bytes = [0xA5, 0x5A, 0xFF, 0x00]
resp = spi.xfer2(test_bytes)
spi.close()
print(f"  Sent: {[hex(b) for b in test_bytes]}")
print(f"  Got:  {[hex(b) for b in resp]}")
match = test_bytes == resp
print(f"  Loopback: {'PASS' if match else 'FAIL (remove short after test)'}")

print("\n=== Done ===")
print("Expected IMU result: Mode 3, byte[1] = 0x69")
print("If loopback fails: MISO line issue (check SA0 wire and shared bus contention)")
print("If loopback passes but IMU fails: CS (side 2) not tied to GND, or IMU not powered")
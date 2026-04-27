#!/usr/bin/env python3
"""
MPU6050 I2C validation script for Raspberry Pi 4.

Wiring (Pi 4 I2C1 -> MPU6050 / GY-521 breakout):
    Pi 3.3V (pin 1)  -> VCC  (most GY-521 clones run fine on 3.3V; see notes)
    Pi GND  (pin 9)  -> GND
    GPIO2   (pin 3)  -> SDA   (I2C1 data)
    GPIO3   (pin 5)  -> SCL   (I2C1 clock)
    Pi GND  or 3.3V  -> AD0   (select addr: GND = 0x68, 3.3V = 0x69)
    (INT, XDA, XCL unused)

Prereqs:
    - I2C enabled:  sudo raspi-config -> Interface Options -> I2C -> Enable
    - /dev/i2c-1 exists after reboot
    - `pip install smbus2`

What this does:
    1. Opens /dev/i2c-1 at the default 100 kHz.
    2. Reads WHO_AM_I (register 0x75). Expected value: 0x68.
    3. Wakes the chip by clearing the sleep bit in PWR_MGMT_1 (0x6B).
    4. Reads 5 samples of accel X/Y/Z, gyro X/Y/Z, and die temperature.
"""

import sys
import time
import argparse

try:
    from smbus2 import SMBus
except ImportError:
    sys.exit("smbus2 not installed. Run:  pip install smbus2")

# --- MPU6050 register map (subset) ---
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B   # accel X/Y/Z, temp, gyro X/Y/Z are 14 bytes starting here
REG_TEMP_OUT_H   = 0x41
REG_GYRO_XOUT_H  = 0x43
REG_PWR_MGMT_1   = 0x6B
REG_WHO_AM_I     = 0x75

WHO_AM_I_EXPECTED = 0x68

# Scale factors for default ranges (+/-2g, +/-250 deg/s)
ACCEL_LSB_PER_G   = 16384.0
GYRO_LSB_PER_DPS  = 131.0
# Temperature in celsius = (raw / 340) + 36.53
TEMP_SCALE  = 340.0
TEMP_OFFSET = 36.53


def s16(hi, lo):
    """Combine two bytes (big-endian, MPU6050 order) into signed 16-bit."""
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v


def scan_i2c(bus_num=1):
    """Lightweight i2cdetect equivalent. Returns list of responding addresses."""
    found = []
    with SMBus(bus_num) as bus:
        for addr in range(0x03, 0x78):
            try:
                bus.read_byte(addr)
                found.append(addr)
            except OSError:
                pass
    return found


def read_all(bus, addr):
    """Burst-read 14 bytes: accel XYZ, temp, gyro XYZ."""
    data = bus.read_i2c_block_data(addr, REG_ACCEL_XOUT_H, 14)
    ax = s16(data[0],  data[1])
    ay = s16(data[2],  data[3])
    az = s16(data[4],  data[5])
    t  = s16(data[6],  data[7])
    gx = s16(data[8],  data[9])
    gy = s16(data[10], data[11])
    gz = s16(data[12], data[13])
    return ax, ay, az, t, gx, gy, gz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus",  type=int, default=1, help="I2C bus number (default 1)")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x68,
                    help="MPU6050 address: 0x68 (AD0=GND) or 0x69 (AD0=3V3)")
    ap.add_argument("--scan", action="store_true",
                    help="Scan the bus and exit (like i2cdetect)")
    args = ap.parse_args()

    if args.scan:
        print(f"Scanning /dev/i2c-{args.bus} ...")
        found = scan_i2c(args.bus)
        if not found:
            print("  No devices responded. Check wiring, power, and that I2C is enabled.")
        else:
            for a in found:
                note = ""
                if a == 0x68:
                    note = "  <- MPU6050 default (AD0=GND)"
                elif a == 0x69:
                    note = "  <- MPU6050 alternate (AD0=3V3)"
                print(f"  0x{a:02X}{note}")
        return

    print(f"Opening /dev/i2c-{args.bus}, device at 0x{args.addr:02X} ...")
    bus = SMBus(args.bus)

    # --- Step 1: WHO_AM_I ---
    print("Reading WHO_AM_I (0x75) ...")
    try:
        whoami = bus.read_byte_data(args.addr, REG_WHO_AM_I)
    except OSError as e:
        print(f"\n[FAIL] I2C read errored: {e}")
        print("  No device is ACKing at this address. Try:")
        print(f"    python3 {sys.argv[0]} --scan")
        print("  and see which addresses (if any) show up.")
        sys.exit(1)

    print(f"  WHO_AM_I = 0x{whoami:02X}  (expected 0x{WHO_AM_I_EXPECTED:02X})")
    if whoami != WHO_AM_I_EXPECTED:
        print("\n[WARN] WHO_AM_I is not 0x68. Some clones return other values")
        print("  (e.g. MPU6500 returns 0x70, MPU9250 returns 0x71). If you're")
        print("  sure this is an MPU6050, try power-cycling the module.")
        print("  Continuing anyway since the chip is responding ...")
    else:
        print("[OK] MPU6050 identified.\n")

    # --- Step 2: Wake from sleep ---
    # Writing 0x00 clears the SLEEP bit and uses the internal 8 MHz oscillator.
    # Writing 0x01 instead selects the PLL with X gyro as clock source, which
    # is slightly more stable, but 0x00 is simpler and fine for bring-up.
    print("Waking MPU6050 (clear PWR_MGMT_1.SLEEP) ...")
    bus.write_byte_data(args.addr, REG_PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    pm = bus.read_byte_data(args.addr, REG_PWR_MGMT_1)
    print(f"  PWR_MGMT_1 = 0x{pm:02X}  (sleep bit 6 should be 0)")

    # --- Step 3: Read samples ---
    print("\nReading 5 samples (accel in g, gyro in deg/s, temp in C):")
    hdr = f"{'idx':>4} {'aX':>7} {'aY':>7} {'aZ':>7} {'gX':>7} {'gY':>7} {'gZ':>7} {'T':>6}"
    print(hdr)
    for i in range(5):
        ax, ay, az, t, gx, gy, gz = read_all(bus, args.addr)
        ax_g = ax / ACCEL_LSB_PER_G
        ay_g = ay / ACCEL_LSB_PER_G
        az_g = az / ACCEL_LSB_PER_G
        gx_d = gx / GYRO_LSB_PER_DPS
        gy_d = gy / GYRO_LSB_PER_DPS
        gz_d = gz / GYRO_LSB_PER_DPS
        t_c  = t / TEMP_SCALE + TEMP_OFFSET
        print(f"{i:>4} {ax_g:>7.3f} {ay_g:>7.3f} {az_g:>7.3f} "
              f"{gx_d:>7.2f} {gy_d:>7.2f} {gz_d:>7.2f} {t_c:>6.1f}")
        time.sleep(0.1)

    print("\nSanity checks:")
    print("  - At rest, flat, Z up: aZ should be ~+1.0 g, aX and aY near 0.")
    print("  - Gyro values should be near 0 when the board is not moving.")
    print("  - Temperature should read a few degrees above ambient.")
    bus.close()


if __name__ == "__main__":
    main()
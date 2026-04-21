#!/usr/bin/env python3
"""
UHplift — LSM6DS3 IMU Diagnostic
Team 11 Capstone II

Datasheet references (DocID026899 Rev 10):
    Page 34: Device compatible with SPI Mode 0 and Mode 3
    Page 34: CS LOW = SPI enabled, CS HIGH = I2C mode
    Page 37: RW bit (bit 0) = 1 for read, bits 1-7 = register address
    Page 19: SDO/SA0 = MISO, SDA/SDI = MOSI, SCL/SPC = SCLK
    Page 56: CTRL3_C IF_INC defaults to 1 (burst read enabled)

Wiring (SPI1, CE0):
    IMU Vin  -> Pi 3.3V     (Pin 17)
    IMU GND  -> Pi GND      (Pin 39)
    IMU SCL  -> GPIO21      (Pin 40)  SCLK
    IMU SDA  -> GPIO20      (Pin 38)  MOSI
    IMU SA0  -> GPIO19      (Pin 35)  MISO
    IMU CS   -> GPIO18      (Pin 12)  CE0 — driven LOW by spidev during xfer

CS behavior (page 34):
    CS LOW  = SPI communication mode, I2C disabled
    CS HIGH = SPI idle, I2C enabled
    spidev hardware CE0 handles this automatically per datasheet requirement.
    python3 test_imu_2.py

Expected WHO_AM_I: 0x69
"""

import spidev
import time

# ── SPI Configuration ─────────────────────────────────────────────────────────
SPI_BUS      = 1        # SPI1 (auxiliary SPI, /dev/spidev1.0)
SPI_CE       = 0        # CE0 = GPIO18
SPI_MODE     = 0b00     # Mode 0 — confirmed compatible per datasheet page 34
                        # Mode 3 rejected by Pi 4 aux SPI (Errno 22)
SPI_SPEED_HZ = 1_000_000  # 1 MHz — well within 10 MHz datasheet max (page 25)

# ── Register Definitions (page 37, 56) ────────────────────────────────────────
REG_WHO_AM_I = 0x0F     # WHO_AM_I register address
RW_READ      = 0x80     # Bit 7 set = read operation (page 37: bit 0 of command byte)
WHO_AM_I_VAL = 0x69     # Expected response (LSM6DS3 fixed value)

REG_CTRL3_C  = 0x12     # Control register 3
REG_CTRL4_C  = 0x13     # Control register 4
CTRL3_C_VAL  = 0x44     # BDU=1, IF_INC=1 (page 56)
CTRL4_C_VAL  = 0x04     # I2C_disable=1 (page 34 — locks device into SPI mode)


def read_register(spi, reg):
    """
    Read a single register.
    Command byte: bit7=1 (read) | bits[6:0]=address (page 37)
    Returns byte[1] — byte[0] is don't-care during address phase.
    """
    resp = spi.xfer2([reg | RW_READ, 0x00])
    return resp[1]


def write_register(spi, reg, value):
    """
    Write a single register.
    Command byte: bit7=0 (write) | bits[6:0]=address (page 37)
    """
    spi.xfer2([reg & 0x7F, value])


def test_who_am_i(spi):
    """Step 1 — bare WHO_AM_I read. No init required, register is read-only."""
    val = read_register(spi, REG_WHO_AM_I)
    if val == WHO_AM_I_VAL:
        print(f"  WHO_AM_I = 0x{val:02X} ✓")
        return True
    else:
        print(f"  WHO_AM_I = 0x{val:02X} ✗  (expected 0x{WHO_AM_I_VAL:02X})")
        if val == 0x00:
            print("  → 0x00: MISO not driven — check SA0 solder joint and wire")
        elif val == 0xFF:
            print("  → 0xFF: MISO pulled high — check for short to VDD")
        return False


def init_imu(spi):
    """
    Step 2 — write CTRL3_C and CTRL4_C per datasheet.
    CTRL3_C (page 56): BDU=1 (block data update), IF_INC=1 (auto-increment)
    CTRL4_C (page 34): I2C_disable=1 (locks into SPI mode permanently)
    """
    write_register(spi, REG_CTRL3_C, CTRL3_C_VAL)
    time.sleep(0.001)
    write_register(spi, REG_CTRL4_C, CTRL4_C_VAL)
    time.sleep(0.001)

    # Verify write
    rb3 = read_register(spi, REG_CTRL3_C)
    rb4 = read_register(spi, REG_CTRL4_C)
    print(f"  CTRL3_C = 0x{rb3:02X} (wrote 0x{CTRL3_C_VAL:02X}) "
          f"{'✓' if rb3 == CTRL3_C_VAL else '✗'}")
    print(f"  CTRL4_C = 0x{rb4:02X} (wrote 0x{CTRL4_C_VAL:02X}) "
          f"{'✓' if rb4 == CTRL4_C_VAL else '✗'}")
    return rb3 == CTRL3_C_VAL and rb4 == CTRL4_C_VAL


def run_loop(spi):
    """Step 3 — continuous WHO_AM_I loop. Wiggle wires to find loose connections."""
    print("\n  Looping — wiggle wires to find loose connection. Ctrl+C to stop.\n")
    count = 0
    found = 0
    try:
        while True:
            val = read_register(spi, REG_WHO_AM_I)
            count += 1
            ok = val == WHO_AM_I_VAL
            if ok:
                found += 1
            print(f"  [{count:05d}] 0x{val:02X} {'✓' if ok else '✗'}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"\n  Stopped. {found}/{count} reads returned 0x69.")
        if found > 0 and found < count:
            print("  → Intermittent contact — loose wire or cold solder joint.")
        elif found == 0:
            print("  → Never responded — check SA0 wire continuity and solder.")
        else:
            print("  → Stable. IMU communicating reliably.")


def main():
    print("=" * 60)
    print("  LSM6DS3 IMU Diagnostic")
    print("=" * 60)
    print(f"  Bus={SPI_BUS} CE={SPI_CE} Mode={SPI_MODE} Speed={SPI_SPEED_HZ//1000}kHz")
    print(f"  CS=GPIO18 SCLK=GPIO21 MOSI=GPIO20 MISO=GPIO19\n")

    try:
        spi = spidev.SpiDev()
        spi.open(SPI_BUS, SPI_CE)
        spi.mode = SPI_MODE
        spi.max_speed_hz = SPI_SPEED_HZ
    except FileNotFoundError:
        print("[ERROR] /dev/spidev1.0 not found.")
        print("  Add to /boot/firmware/config.txt under [all]:")
        print("    dtoverlay=spi1-1cs")
        print("  Then: sudo reboot")
        return
    except Exception as e:
        print(f"[ERROR] SPI open failed: {e}")
        return

    # Step 1 — WHO_AM_I
    print("[1] WHO_AM_I read:")
    ok = test_who_am_i(spi)

    if not ok:
        print("\n[2] Running loop — wiggle wires while watching:")
        run_loop(spi)
        spi.close()
        return

    # Step 2 — Init registers
    print("\n[2] Writing CTRL3_C and CTRL4_C:")
    init_imu(spi)

    # Step 3 — Confirm stable
    print("\n[3] Stability loop:")
    run_loop(spi)

    spi.close()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
BMI160 SPI1 validation script for Raspberry Pi 4.

Wiring (Pi 4 SPI1 -> BMI160 breakout):
    Pi 3V3  (pin 1 or 17) -> 3V3  (VDD/VDDIO)
    Pi GND  (e.g. pin 9)  -> GND
    GPIO21  (pin 40)      -> SCL  (SCK)
    GPIO20  (pin 38)      -> SDA  (MOSI)
    GPIO19  (pin 35)      -> SA0  (MISO / SDO)
    GPIO18  (pin 12)      -> CS   (CSB)

Prereqs:
    - /boot/firmware/config.txt contains:   dtoverlay=spi1-1cs
    - /dev/spidev1.0 exists after reboot
    - `pip install spidev`

What this does:
    1. Opens /dev/spidev1.0 at 1 MHz, SPI mode 0.
    2. Forces the chip into SPI mode by doing a dummy read of 0x7F
       (BMI160 boots in I2C mode; a CSB rising edge locks it into SPI).
    3. Reads CHIP_ID (register 0x00). Expected value: 0xD1.
    4. If that works, powers on the accelerometer and reads a few samples.
"""

import time
import sys

try:
    import spidev
except ImportError:
    sys.exit("spidev not installed. Run:  pip install spidev")

# --- BMI160 register map (subset) ---
REG_CHIP_ID   = 0x00
REG_ERR       = 0x02
REG_PMU_STAT  = 0x03
REG_DATA_ACC  = 0x12   # ACC_X LSB; 6 bytes total for X,Y,Z
REG_ACC_RANGE = 0x41
REG_CMD       = 0x7E
REG_DUMMY     = 0x7F   # dummy read here to switch to SPI after boot

CMD_ACC_NORMAL = 0x11  # set accel to normal power mode
CHIP_ID_EXPECTED = 0xD1 

# Read bit: BMI160 sets bit 7 of the address byte high for reads.
READ_FLAG = 0x80


def open_spi(bus=1, device=0, hz=500000):
    spi = spidev.SpiDev()
    spi.open(bus, device)
    spi.max_speed_hz = hz
    spi.mode = 0b11        # CPOL=0, CPHA=0 (BMI160 supports mode 0 or 3)
    spi.bits_per_word = 8
    return spi


def read_reg(spi, reg, length=1):
    """Read `length` bytes starting at `reg`."""
    tx = [reg | READ_FLAG] + [0x00] * length
    rx = spi.xfer2(tx)
    return rx[1:]  # first byte is junk clocked out during the address phase


def write_reg(spi, reg, value):
    """Write one byte to `reg`."""
    spi.xfer2([reg & 0x7F, value & 0xFF])


def enter_spi_mode(spi):
    """
    BMI160 powers up in I2C mode. A rising edge on CSB switches it to SPI.
    Bosch recommends a single dummy read at 0x7F to complete the switch.
    """
    read_reg(spi, REG_DUMMY, 1)
    time.sleep(0.001)


def s16(lsb, msb):
    """Combine two bytes into a signed 16-bit int (little-endian)."""
    v = (msb << 8) | lsb
    return v - 0x10000 if v & 0x8000 else v


def main():
    print("Opening /dev/spidev1.0 ...")
    spi = open_spi(bus=1, device=0, hz=1_000_000)

    print("Switching BMI160 into SPI mode (dummy read @ 0x7F) ...")
    enter_spi_mode(spi)

    print("Reading CHIP_ID (0x00) ...")
    chip_id = read_reg(spi, REG_CHIP_ID, 1)[0]
    print(f"  CHIP_ID = 0x{chip_id:02X}  (expected 0x{CHIP_ID_EXPECTED:02X})")

    if chip_id != CHIP_ID_EXPECTED:
        print("\n[FAIL] CHIP_ID mismatch. Things to check:")
        print("  - VDD at chip pin 8 actually reads ~3.3V")
        print("  - MOSI/MISO not swapped (SDA->MOSI, SA0->MISO)")
        print("  - SCK and CS going to the right Pi pins")
        print("  - spi1-1cs overlay enabled and /dev/spidev1.0 present")
        print("  - Try lowering max_speed_hz to 500000")
        spi.close()
        sys.exit(1)

    print("[OK] BMI160 responded correctly over SPI.\n")

    # Check ERR_REG just to make sure nothing is flagged.
    err = read_reg(spi, REG_ERR, 1)[0]
    print(f"ERR_REG (0x02) = 0x{err:02X}  (0x00 = no error)")

    # Power on the accelerometer.
    print("Turning on accelerometer (CMD 0x11) ...")
    write_reg(spi, REG_CMD, CMD_ACC_NORMAL)
    time.sleep(0.05)  # datasheet: ~3.8 ms start-up, give it margin

    pmu = read_reg(spi, REG_PMU_STAT, 1)[0]
    print(f"PMU_STATUS (0x03) = 0x{pmu:02X}  (acc bits 5:4 should be 01)")

    # Read 5 accel samples.
    print("\nReading 5 accelerometer samples (raw, +/-2g default):")
    print(f"{'idx':>4} {'X':>8} {'Y':>8} {'Z':>8}")
    for i in range(5):
        raw = read_reg(spi, REG_DATA_ACC, 6)
        ax = s16(raw[0], raw[1])
        ay = s16(raw[2], raw[3])
        az = s16(raw[4], raw[5])
        print(f"{i:>4} {ax:>8} {ay:>8} {az:>8}")
        time.sleep(0.1)

    print("\nAt rest with Z pointing up, Z should read ~16384 (1g at +/-2g range).")
    spi.close()


if __name__ == "__main__":
    main()
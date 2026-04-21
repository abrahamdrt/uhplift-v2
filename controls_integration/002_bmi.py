#!/usr/bin/env python3
"""
BMI160 SPI1 validation script for Raspberry Pi 4 (v2 with diagnostics).

Wiring (Pi 4 SPI1 -> BMI160 breakout):
    Pi 3V3  (pin 1 or 17) -> VIN  (recommended) or 3V3 pad
    Pi GND  (e.g. pin 9)  -> GND
    GPIO21  (pin 40)      -> SCL  (SCK)
    GPIO20  (pin 38)      -> SDA  (MOSI)
    GPIO19  (pin 35)      -> SA0  (MISO / SDO)
    GPIO18  (pin 12)      -> CS   (CSB)

Run modes:
    python3 bmi160_spi_test.py              # full BMI160 test
    python3 bmi160_spi_test.py --loopback   # pure SPI peripheral sanity check
                                            # (jumper MOSI<->MISO, disconnect IMU)

NOTE: Pi aux SPI (SPI1) only supports mode 0. Don't try mode 3 on spidev1.x.
"""

import sys
import time
import argparse

try:
    import spidev
except ImportError:
    sys.exit("spidev not installed. Run:  pip install spidev")

# --- BMI160 register map (subset) ---
REG_CHIP_ID   = 0x00
REG_ERR       = 0x02
REG_PMU_STAT  = 0x03
REG_DATA_ACC  = 0x12
REG_ACC_RANGE = 0x41
REG_CMD       = 0x7E
REG_DUMMY     = 0x7F

CMD_SOFTRESET  = 0xB6
CMD_ACC_NORMAL = 0x11
CHIP_ID_EXPECTED = 0xD1

READ_FLAG = 0x80


def open_spi(bus=1, device=0, hz=1_000_000):
    spi = spidev.SpiDev()
    spi.open(bus, device)
    spi.max_speed_hz = hz
    spi.mode = 0b00  # aux SPI (SPI1) ONLY supports mode 0
    spi.bits_per_word = 8
    return spi


def read_reg(spi, reg, length=1):
    tx = [reg | READ_FLAG] + [0x00] * length
    rx = spi.xfer2(tx)
    return rx[1:]


def write_reg(spi, reg, value):
    spi.xfer2([reg & 0x7F, value & 0xFF])


def enter_spi_mode(spi):
    """
    Force several CSB toggles then do the dummy read at 0x7F.
    Multiple small transactions give the chip clean CSB rising edges
    to latch the SPI mode selection.
    """
    for _ in range(3):
        spi.xfer2([0x7F | READ_FLAG, 0x00])
        time.sleep(0.001)


def s16(lsb, msb):
    v = (msb << 8) | lsb
    return v - 0x10000 if v & 0x8000 else v


def loopback_test(bus=1, device=0, hz=1_000_000):
    """
    Pi SPI peripheral sanity check. Jumper MOSI (GPIO20) directly to
    MISO (GPIO19), disconnect the IMU, then run this.
    If the Pi's SPI1 is working, whatever we send is what we get back.
    """
    print(f"Loopback test on spidev{bus}.{device} @ {hz} Hz, mode 0")
    print("Jumper MOSI (GPIO20, pin 38) <-> MISO (GPIO19, pin 35).")
    print("Disconnect the IMU for this test.\n")

    spi = open_spi(bus, device, hz)
    patterns = [
        [0xAA, 0x55, 0xFF, 0x00, 0xDE, 0xAD, 0xBE, 0xEF],
        [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80],
        [0xD1, 0xD1, 0xD1, 0xD1],
    ]
    all_ok = True
    for tx in patterns:
        rx = spi.xfer2(list(tx))
        ok = rx == list(tx)
        all_ok &= ok
        print(f"  TX: {[f'0x{b:02X}' for b in tx]}")
        print(f"  RX: {[f'0x{b:02X}' for b in rx]}  {'OK' if ok else 'MISMATCH'}")
    spi.close()

    print()
    if all_ok:
        print("[OK] Loopback passed. The Pi's SPI1 peripheral is healthy.")
        print("     If BMI160 still fails, it's a wiring or power issue on")
        print("     the IMU side, not the Pi.")
    else:
        print("[FAIL] Loopback failed. Either:")
        print("       - MOSI/MISO jumper not actually connected")
        print("       - SPI1 overlay not loaded (check /boot/firmware/config.txt)")
        print("       - Wrong spidev bus/device")
    return all_ok


def bmi160_test():
    print("Opening /dev/spidev1.0 ...")
    spi = open_spi(bus=1, device=0, hz=1_000_000)

    print("Switching BMI160 into SPI mode (CSB toggles + dummy reads) ...")
    enter_spi_mode(spi)

    print("Reading CHIP_ID (0x00) ...")
    chip_id = read_reg(spi, REG_CHIP_ID, 1)[0]
    print(f"  CHIP_ID = 0x{chip_id:02X}  (expected 0x{CHIP_ID_EXPECTED:02X})")

    if chip_id != CHIP_ID_EXPECTED:
        print("\n[FAIL] CHIP_ID mismatch. Diagnosis by return value:")
        if chip_id == 0xFF:
            print("  0xFF usually means MISO is idling high and no data is")
            print("  coming back. Most likely causes:")
            print("   1. Chip is not actually powered. Try moving the 3.3V")
            print("      supply wire from the '3V3' pad to the 'VIN' pad.")
            print("      The '3V3' pad on many breakouts is the LDO OUTPUT,")
            print("      not an input, and back-feeding it may not power VDD.")
            print("   2. Chip is stuck in I2C mode. Check CS is being driven")
            print("      (should go low during transfer, high at idle).")
            print("   3. MISO is disconnected or the trace is broken.")
        elif chip_id == 0x00:
            print("  0x00 usually means MISO is being held low -- likely the")
            print("  MISO line is floating and reading as ground, or MOSI")
            print("  and MISO are swapped so the chip never drives MISO.")
        else:
            print(f"  0x{chip_id:02X} is unexpected. Possible bit-alignment")
            print("  issue or a different chip entirely. Try re-running a few")
            print("  times -- if the value changes between runs, it's noise")
            print("  on a floating line.")
        print("\n  Also try:  python3 bmi160_spi_test.py --loopback")
        print("  to verify the Pi's SPI1 peripheral itself works.")
        spi.close()
        sys.exit(1)

    print("[OK] BMI160 responded correctly over SPI.\n")

    err = read_reg(spi, REG_ERR, 1)[0]
    print(f"ERR_REG (0x02) = 0x{err:02X}  (0x00 = no error)")

    print("Turning on accelerometer (CMD 0x11) ...")
    write_reg(spi, REG_CMD, CMD_ACC_NORMAL)
    time.sleep(0.05)

    pmu = read_reg(spi, REG_PMU_STAT, 1)[0]
    print(f"PMU_STATUS (0x03) = 0x{pmu:02X}  (acc bits 5:4 should be 01)")

    print("\nReading 5 accelerometer samples (raw, +/-2g default):")
    print(f"{'idx':>4} {'X':>8} {'Y':>8} {'Z':>8}")
    for i in range(5):
        raw = read_reg(spi, REG_DATA_ACC, 6)
        ax = s16(raw[0], raw[1])
        ay = s16(raw[2], raw[3])
        az = s16(raw[4], raw[5])
        print(f"{i:>4} {ax:>8} {ay:>8} {az:>8}")
        time.sleep(0.1)

    print("\nAt rest with Z pointing up, Z should read ~16384 (1g at +/-2g).")
    spi.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--loopback", action="store_true",
                    help="Run SPI1 loopback test instead of BMI160 test")
    args = ap.parse_args()

    if args.loopback:
        loopback_test()
    else:
        bmi160_test()


if __name__ == "__main__":
    main()
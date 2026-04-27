#!/usr/bin/env python3
"""
BMI160 SPI1 validation script for Raspberry Pi 4 (Continuous Wiggle Test).

Wiring (Pi 4 SPI1 -> BMI160 breakout):
    Pi 3V3  (pin 1 or 17) -> VIN  (recommended) or 3V3 pad
    Pi GND  (e.g. pin 9)  -> GND
    GPIO21  (pin 40)      -> SCL  (SCK)
    GPIO20  (pin 38)      -> SDA  (MOSI)
    GPIO19  (pin 35)      -> SA0  (MISO / SDO)
    GPIO18  (pin 12)      -> CS   (CSB)
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
    else:
        print("[FAIL] Loopback failed. Check connections.")
    return all_ok


def bmi160_test():
    print("Opening /dev/spidev1.0 ...")
    spi = open_spi(bus=1, device=0, hz=1_000_000)

    print("Switching BMI160 into SPI mode (CSB toggles + dummy reads) ...")
    enter_spi_mode(spi)

    # Initial check before entering the loop
    chip_id = read_reg(spi, REG_CHIP_ID, 1)[0]
    if chip_id != CHIP_ID_EXPECTED:
        print(f"\n[WARNING] Initial CHIP_ID mismatch: 0x{chip_id:02X} (Expected 0x{CHIP_ID_EXPECTED:02X})")
        print("Continuing anyway so you can wiggle the wires to find the connection...")
    else:
        print("[OK] BMI160 responded correctly over SPI.\n")

    print("Attempting to turn on accelerometer (CMD 0x11) ...")
    write_reg(spi, REG_CMD, CMD_ACC_NORMAL)
    time.sleep(0.05)

    print("\nStarting Continuous Wiggle Test.")
    print("Press Ctrl+C to stop.\n")
    print(f"{'idx':>6} {'X':>8} {'Y':>8} {'Z':>8}  {'STATUS':>15}")
    
    try:
        i = 0
        while True:
            # Re-read chip ID every loop to instantly detect connection loss
            current_chip_id = read_reg(spi, REG_CHIP_ID, 1)[0]

            if current_chip_id != CHIP_ID_EXPECTED:
                # If SPI drops, alert the user and print the garbage ID
                print(f"{i:>6} {'-':>8} {'-':>8} {'-':>8}  [SPI LOST (0x{current_chip_id:02X})]")
            else:
                # Connection is solid, grab the data
                raw = read_reg(spi, REG_DATA_ACC, 6)
                ax = s16(raw[0], raw[1])
                ay = s16(raw[2], raw[3])
                az = s16(raw[4], raw[5])
                print(f"{i:>6} {ax:>8} {ay:>8} {az:>8}  [CONNECTED]")

            i += 1
            time.sleep(0.1) # 10Hz loop gives you plenty of time to read the output
            
    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
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
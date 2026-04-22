#!/usr/bin/env python3
"""
UHplift — Continuous Sensor Diagnostic
Prints encoder positions, IMU angles, and joystick state every 100ms.
Runs until Ctrl+C.

Usage:
    python3 diag_sensors.py              # All sensors
    python3 diag_sensors.py --enc-only   # Encoders only (fast debug)
    python3 diag_sensors.py --imu-only   # IMU only
"""

import sys
import time
import math
import argparse
import signal

try:
    import spidev
    import pigpio
except ImportError:
    print("[ERROR] spidev/pigpio not available"); sys.exit(1)

# ── Pin map (must match main.py) ─────────────────────────────────────────────
CS_TROLLEY     = 8   # CS1
CS_HOIST       = 16  # CS2
CS_BRIDGE      = 7   # CS3
CS_BRIDGE_DIAG = 12  # CS4

IPC_TROLLEY = -0.000152  # Negated for coordinate convention
IPC_BRIDGE  = 0.000190
IPC_HOIST   = 0.000118

# LS7366R commands
WR_MDR0 = 0x88; WR_MDR1 = 0x90; CLR_CNTR = 0x20
LOAD_OTR = 0xE8; RD_OTR = 0x68; RD_MDR0 = 0x48

running = True

def sigint(s, f):
    global running
    running = False

signal.signal(signal.SIGINT, sigint)


def xfer(spi, pi, cs, data):
    pi.write(cs, 0); time.sleep(0.000002)
    r = spi.xfer2(list(data))
    pi.write(cs, 1); time.sleep(0.000002)
    return r


def init_enc(spi, pi, name, cs):
    pi.set_mode(cs, pigpio.OUTPUT)
    pi.write(cs, 1)
    xfer(spi, pi, cs, [WR_MDR0, 0x03]); time.sleep(0.001)
    xfer(spi, pi, cs, [WR_MDR1, 0x00]); time.sleep(0.001)
    xfer(spi, pi, cs, [CLR_CNTR]);       time.sleep(0.001)
    rb = xfer(spi, pi, cs, [RD_MDR0, 0x00])
    ok = rb[1] == 0x03
    print(f"  [{name:>12}] CS=GPIO{cs:>2}  MDR0=0x{rb[1]:02X}  "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def read_enc(spi, pi, cs, ipc):
    xfer(spi, pi, cs, [LOAD_OTR]); time.sleep(0.0001)
    raw = xfer(spi, pi, cs, [RD_OTR, 0, 0, 0, 0])
    c = (raw[1]<<24)|(raw[2]<<16)|(raw[3]<<8)|raw[4]
    if c >= 0x80000000: c -= 0x100000000
    return c, c * ipc


def main():
    parser = argparse.ArgumentParser(description="UHplift Sensor Diag")
    parser.add_argument("--enc-only", action="store_true")
    parser.add_argument("--imu-only", action="store_true")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Print rate [Hz] (default: 10)")
    args = parser.parse_args()

    pi = pigpio.pi()
    if not pi.connected:
        print("[ERROR] pigpiod not running"); return

    print("=" * 72)
    print("  UHplift Continuous Sensor Diagnostic — Ctrl+C to stop")
    print("=" * 72)

    spi_enc = None
    enc_ok = {}

    if not args.imu_only:
        print("\n[ENCODERS] SPI0 software CS:")
        spi_enc = spidev.SpiDev()
        spi_enc.open(0, 0)
        spi_enc.no_cs = True
        spi_enc.mode = 0
        spi_enc.max_speed_hz = 500_000

        for name, cs in [("trolley", CS_TROLLEY), ("hoist", CS_HOIST),
                         ("bridge", CS_BRIDGE), ("bridge_diag", CS_BRIDGE_DIAG)]:
            enc_ok[name] = init_enc(spi_enc, pi, name, cs)

    imu_bus = None
    a_off = [0,0,0]; g_off = [0,0,0]
    IMU_ADDR = 0x68
    if not args.enc_only:
        print("\n[IMU] MPU6050 I2C1 (0x68):")
        try:
            from smbus2 import SMBus as _SMBus
            imu_bus = _SMBus(1)
            who = imu_bus.read_byte_data(IMU_ADDR, 0x75)
            print(f"  WHO_AM_I = 0x{who:02X} {'OK' if who == 0x68 else 'UNEXPECTED'}")
            # Wake + configure
            imu_bus.write_byte_data(IMU_ADDR, 0x6B, 0x00)  # Wake
            imu_bus.write_byte_data(IMU_ADDR, 0x19, 4)     # 200 Hz
            imu_bus.write_byte_data(IMU_ADDR, 0x1A, 3)     # DLPF 44 Hz
            imu_bus.write_byte_data(IMU_ADDR, 0x1B, 0x08)  # ±500 dps
            imu_bus.write_byte_data(IMU_ADDR, 0x1C, 0x00)  # ±2g
            time.sleep(0.05)
            # Quick calibration
            print("  Calibrating (50 samples)...")
            ax_s = [0,0,0]; gx_s = [0,0,0]
            ASCL = 2.0/32768.0; GSCL = 500.0/32768.0 * math.pi/180
            def _s16(h, l):
                v = (h<<8)|l
                return v-0x10000 if v&0x8000 else v
            for _ in range(50):
                d = imu_bus.read_i2c_block_data(IMU_ADDR, 0x3B, 14)
                for j in range(3):
                    ax_s[j] += _s16(d[j*2], d[j*2+1]) * ASCL
                    gx_s[j] += _s16(d[8+j*2], d[9+j*2]) * GSCL
                time.sleep(0.01)
            a_off = [x/50 for x in ax_s]
            g_off = [x/50 for x in gx_s]
            a_off[1] -= 1.0  # Subtract gravity on Y
            print(f"  Accel offset: [{a_off[0]:.4f}, {a_off[1]:.4f}, {a_off[2]:.4f}]")
            print(f"  Gyro offset:  [{g_off[0]:.4f}, {g_off[1]:.4f}, {g_off[2]:.4f}]")
        except Exception as e:
            print(f"  IMU error: {e}")
            imu_bus = None

    # ── Joystick (optional) ──────────────────────────────────────────────
    joy = None
    if not args.enc_only and not args.imu_only:
        try:
            import pygame
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                joy = pygame.joystick.Joystick(0)
                joy.init()
                print(f"\n[JOY] {joy.get_name()}")
            else:
                print("\n[JOY] No controller found")
        except Exception:
            pass

    # ── Print header ─────────────────────────────────────────────────────
    dt = 1.0 / args.rate
    count = 0
    print(f"\n{'─'*72}")

    header_parts = []
    if spi_enc:
        header_parts.append("  TROLLEY       BRIDGE        HOIST       BR_DIAG")
    if imu_bus:
        header_parts.append("   θx(brg)   θz(trl)   gx       gz")
    if joy:
        header_parts.append(" JOY:B T H")
    print("".join(header_parts))
    print(f"{'─'*72}")

    # ── Main loop ────────────────────────────────────────────────────────
    while running:
        parts = []

        if spi_enc:
            enc_data = {}
            for name, cs, ipc in [("trolley", CS_TROLLEY, IPC_TROLLEY),
                                   ("bridge", CS_BRIDGE, IPC_BRIDGE),
                                   ("hoist", CS_HOIST, IPC_HOIST),
                                   ("br_diag", CS_BRIDGE_DIAG, IPC_BRIDGE)]:
                if enc_ok.get(name if name != "br_diag" else "bridge_diag", False):
                    c, p = read_enc(spi_enc, pi, cs, ipc)
                    enc_data[name] = (c, p)
                else:
                    enc_data[name] = (0, 0.0)

            t_c, t_p = enc_data["trolley"]
            b_c, b_p = enc_data["bridge"]
            h_c, h_p = enc_data["hoist"]
            d_c, d_p = enc_data["br_diag"]
            parts.append(f"T:{t_p:+8.3f}  B:{b_p:+8.3f}  "
                        f"H:{h_p:+8.3f}  D:{d_p:+8.3f}")

        if imu_bus:
            try:
                ASCL = 2.0/32768.0; GSCL = 500.0/32768.0 * math.pi/180
                d = imu_bus.read_i2c_block_data(IMU_ADDR, 0x3B, 14)
                a = [0.0]*3; g = [0.0]*3
                for j in range(3):
                    a[j] = _s16(d[j*2], d[j*2+1]) * ASCL - a_off[j]
                    g[j] = _s16(d[8+j*2], d[9+j*2]) * GSCL - g_off[j]
                tx_deg = math.degrees(math.atan2(a[0], a[1]))
                tz_deg = math.degrees(math.atan2(a[2], a[1]))
                parts.append(f"  θx:{tx_deg:+6.2f}° θz:{tz_deg:+6.2f}° "
                            f"gx:{g[0]:+.3f} gz:{g[2]:+.3f}")
            except Exception:
                parts.append("  IMU: read error")

        if joy:
            try:
                import pygame
                pygame.event.pump()
                hat = joy.get_hat(0) if joy.get_numhats() > 0 else (0,0)
                btns = [joy.get_button(i) for i in range(min(joy.get_numbuttons(), 14))]
                parts.append(f"  hat:{hat} btn:{''.join(str(b) for b in btns)}")
            except Exception:
                pass

        print(" | ".join(parts), flush=True)
        count += 1
        time.sleep(dt)

    # ── Cleanup ──────────────────────────────────────────────────────────
    print(f"\n[DONE] {count} samples")
    if spi_enc: spi_enc.close()
    if imu_bus: imu_bus.close()
    if joy:
        import pygame; pygame.quit()
    pi.stop()


if __name__ == "__main__":
    main()
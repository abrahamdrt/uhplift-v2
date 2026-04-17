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

    imu_spi = None
    if not args.enc_only:
        print("\n[IMU] SPI1 CE0 Mode 0:")
        imu_spi = spidev.SpiDev()
        imu_spi.open(1, 2)
        imu_spi.mode = 0
        imu_spi.max_speed_hz = 1_000_000

        who = imu_spi.xfer2([0x0F | 0x80, 0x00])[1]
        print(f"  WHO_AM_I = 0x{who:02X} {'OK' if who == 0x69 else 'FAIL'}")
        if who == 0x69:
            imu_spi.xfer2([0x10, 0x58])  # CTRL1_XL: 208Hz ±4g
            imu_spi.xfer2([0x11, 0x54])  # CTRL2_G:  208Hz ±500dps
            time.sleep(0.05)
            # Quick calibration
            print("  Calibrating (50 samples)...")
            ax_s = [0,0,0]; gx_s = [0,0,0]
            for _ in range(50):
                # Read accel
                tx = [0x28|0x80]+[0]*6
                rd = imu_spi.xfer2(tx)
                for j in range(3):
                    v = (rd[2*j+2]<<8)|rd[2*j+1]
                    if v>=32768: v-=65536
                    ax_s[j] += v * 4.0/32768.0
                # Read gyro
                tx = [0x22|0x80]+[0]*6
                rd = imu_spi.xfer2(tx)
                for j in range(3):
                    v = (rd[2*j+2]<<8)|rd[2*j+1]
                    if v>=32768: v-=65536
                    gx_s[j] += v * 500.0/32768.0 * math.pi/180
                time.sleep(0.01)
            a_off = [x/50 for x in ax_s]
            g_off = [x/50 for x in gx_s]
            a_off[1] -= 1.0  # Subtract gravity on Y
            print(f"  Accel offset: [{a_off[0]:.4f}, {a_off[1]:.4f}, {a_off[2]:.4f}]")
            print(f"  Gyro offset:  [{g_off[0]:.4f}, {g_off[1]:.4f}, {g_off[2]:.4f}]")
        else:
            imu_spi.close(); imu_spi = None
            print("  IMU not available")

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
    if imu_spi:
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

        if imu_spi:
            # Accel
            tx = [0x28|0x80]+[0]*6
            rd = imu_spi.xfer2(tx)
            a = [0.0]*3
            for j in range(3):
                v = (rd[2*j+2]<<8)|rd[2*j+1]
                if v>=32768: v-=65536
                a[j] = v*4.0/32768.0 - a_off[j]
            # Gyro
            tx = [0x22|0x80]+[0]*6
            rd = imu_spi.xfer2(tx)
            g = [0.0]*3
            for j in range(3):
                v = (rd[2*j+2]<<8)|rd[2*j+1]
                if v>=32768: v-=65536
                g[j] = v*500.0/32768.0*math.pi/180 - g_off[j]

            tx_deg = math.degrees(math.atan2(a[0], a[1]))  # Bridge sway
            tz_deg = math.degrees(math.atan2(a[2], a[1]))  # Trolley sway
            parts.append(f"  θx:{tx_deg:+6.2f}° θz:{tz_deg:+6.2f}° "
                        f"gx:{g[0]:+.3f} gz:{g[2]:+.3f}")

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
    if imu_spi: imu_spi.close()
    if joy:
        import pygame; pygame.quit()
    pi.stop()


if __name__ == "__main__":
    main()
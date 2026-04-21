#!/usr/bin/env python3
"""
UHplift — Drivetrain Calibration v3
Team 11 Capstone II

PURPOSE:
    Validates and corrects the theoretical inches-per-pulse ratio against
    physical reality. The stepper config uses a calculated value based on
    gear ratio and wheel radius — this script measures the actual distance
    traveled per pulse and tells you what to update in the stack.

ENCODER FEEDBACK AVAILABILITY:
    Trolley (CS=GPIO8):  Encoder confirmed working — counts verified against motion.
    Bridge  (CS=GPIO7):  No encoder feedback — distance must be measured manually.
    Hoist   (CS=GPIO16): No encoder feedback — distance must be measured manually.

THEORETICAL VALUES (from current stepper.py):
    Trolley: gear=5.0, r=0.485in → inches_per_pulse = 2π×0.485 / (200×4×5.0) = 0.000381 in/pulse
    Bridge:  gear=4.0, r=0.485in → inches_per_pulse = 2π×0.485 / (200×4×4.0) = 0.000476 in/pulse
    Hoist:   gear=10.0, r=0.750in → inches_per_pulse = 2π×0.750 / (200×4×10.0) = 0.000589 in/pulse

    ENC_IPC (encoder inches per count, from main.py):
    Trolley: -0.000152 in/count (negated for coordinate convention)
    Bridge:   0.000190 in/count
    Hoist:    0.000118 in/count

WHAT TO UPDATE AFTER CALIBRATION:
    If measured distance differs from theoretical:
    1. Update wheel_radius_in in stepper.py for the relevant axis
       (or update ENC_IPC in main.py for encoder-based axes)
    2. The script computes the corrected values directly

USAGE:
    python3 calibrate_drivetrain_v3.py --axis trolley --dist 10.0 --hz 1500
    python3 calibrate_drivetrain_v3.py --axis bridge  --dist 10.0 --hz 1500
    python3 calibrate_drivetrain_v3.py --axis hoist   --dist 5.0  --hz 800

    --axis   : trolley / bridge / hoist
    --dist   : distance to command [inches] — use a ruler or tape measure
    --hz     : pulse frequency (default 1500) — keep low for calibration
    --no-rev : skip the return trip (useful if axis has limited travel)
"""

import pigpio
import spidev
import time
import math
import argparse
import sys

# ══════════════════════════════════════════════════════════════════════════════
# Hardware Map — matches main.py exactly
# ══════════════════════════════════════════════════════════════════════════════

PINS = {
    'trolley': {
        'pul': 22, 'dir': 27, 'ena': 17, 'cs': 8,
        'gear': 5.0, 'radius': 0.485,
        'enc': True,   # encoder feedback available
        'enc_ipc': -0.000152,  # in/count (signed per coordinate convention)
        'counts_per_rev': 4000,
    },
    'bridge': {
        'pul': 23, 'dir': 24, 'ena': 25, 'cs': 7,
        'gear': 4.0, 'radius': 0.485,
        'enc': False,  # no encoder feedback
        'enc_ipc': 0.000190,
        'counts_per_rev': 4000,
    },
    'hoist': {
        'pul': 5, 'dir': 6, 'ena': 13, 'cs': 16,
        'gear': 10.0, 'radius': 0.750,
        'enc': False,  # no encoder feedback
        'enc_ipc': 0.000118,
        'counts_per_rev': 4000,
        'brake': 26,   # GPIO26 — NC relay, LOW=released HIGH=engaged
    },
}

STEPS_PER_REV = 200
MICROSTEP     = 4
PULSES_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEP  # 800

# ══════════════════════════════════════════════════════════════════════════════
# Theoretical Calculations
# ══════════════════════════════════════════════════════════════════════════════

def theoretical_ipp(gear, radius):
    """Theoretical inches per pulse from geometry."""
    circumference = 2 * math.pi * radius
    pulses_per_output_rev = PULSES_PER_MOTOR_REV * gear
    return circumference / pulses_per_output_rev

def dist_to_pulses(dist_in, gear, radius):
    """Convert a target distance [in] to pulse count."""
    ipp = theoretical_ipp(gear, radius)
    return int(round(dist_in / ipp))

# ══════════════════════════════════════════════════════════════════════════════
# Encoder Bus — mirrors main.py software CS logic
# ══════════════════════════════════════════════════════════════════════════════

class EncoderBusDiag:
    WR_MDR0  = 0x88; WR_MDR1 = 0x90; CLR_CNTR = 0x20
    LOAD_OTR = 0xE8; RD_OTR  = 0x68; RD_MDR0  = 0x48

    def __init__(self, pi_handle, cs_pin):
        self._pi  = pi_handle
        self.cs   = cs_pin
        self._spi = spidev.SpiDev()
        self._spi.open(0, 0)
        self._spi.no_cs       = True
        self._spi.mode        = 0
        self._spi.max_speed_hz = 500_000
        self._pi.set_mode(self.cs, pigpio.OUTPUT)
        self._pi.write(self.cs, 1)

    def _xfer(self, data):
        self._pi.write(self.cs, 0); time.sleep(0.000002)
        r = self._spi.xfer2(list(data))
        self._pi.write(self.cs, 1); time.sleep(0.000002)
        return r

    def initialize(self):
        self._xfer([self.WR_MDR0, 0x03]); time.sleep(0.001)
        self._xfer([self.WR_MDR1, 0x00]); time.sleep(0.001)
        self._xfer([self.CLR_CNTR]);       time.sleep(0.001)
        rb = self._xfer([self.RD_MDR0, 0x00])
        ok = rb[1] == 0x03
        status = '✓' if ok else '✗ FAIL — counts unreliable'
        print(f"  [ENC] MDR0=0x{rb[1]:02X} {status} (CS=GPIO{self.cs})")
        return ok

    def read_counts(self):
        self._xfer([self.LOAD_OTR]); time.sleep(0.0001)
        raw = self._xfer([self.RD_OTR, 0, 0, 0, 0])
        c = (raw[1]<<24)|(raw[2]<<16)|(raw[3]<<8)|raw[4]
        if c >= 0x80000000: c -= 0x100000000
        return c

    def clear(self):
        self._xfer([self.CLR_CNTR]); time.sleep(0.001)

    def close(self):
        self._spi.close()


# ══════════════════════════════════════════════════════════════════════════════
# Waveform — blocking ramp (calibration only, never in control loop)
# ══════════════════════════════════════════════════════════════════════════════

def send_pulses_ramped(pi, pul_pin, total, target_hz):
    if total <= 0: return
    t        = float(max(target_hz, 1))
    start_hz = max(min(t * 0.10, 400.0), 80.0)
    accel    = max(min(t * 3.0, 12000.0), 300.0)
    chunk    = 200
    min_hus  = 2

    def hz_to_hus(hz): return max(int(1_000_000 / (2 * max(hz, 1))), min_hus)

    def send_chunk(n, hus):
        pi.wave_clear()
        mask = 1 << pul_pin
        pi.wave_add_generic(
            [pigpio.pulse(mask, 0, hus), pigpio.pulse(0, mask, hus)] * n)
        wid = pi.wave_create()
        pi.wave_send_once(wid)
        while pi.wave_tx_busy(): time.sleep(0.005)
        pi.wave_delete(wid)

    ramp_p = int((t*t - start_hz*start_hz) / (2 * accel))
    if ramp_p * 2 >= total:
        ramp_p = total // 2
    cruise = total - 2 * ramp_p

    f = start_hz
    for rem, phase in [(ramp_p, 'up'), (cruise, 'cruise'), (ramp_p, 'down')]:
        while rem > 0:
            n = min(chunk, rem)
            send_chunk(n, hz_to_hus(f))
            if phase == 'up':
                f = min(math.sqrt(f*f + 2*accel*n), t)
            elif phase == 'down':
                f = max(math.sqrt(max(f*f - 2*accel*n, start_hz**2)), start_hz)
            rem -= n


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="UHplift Drivetrain Calibration v3")
    parser.add_argument("--axis",   choices=["trolley","bridge","hoist"], required=True)
    parser.add_argument("--dist",   type=float, default=10.0,
                        help="Distance to command [inches] (default: 10.0)")
    parser.add_argument("--hz",     type=int,   default=1500,
                        help="Pulse frequency (default: 1500)")
    parser.add_argument("--no-rev", action="store_true",
                        help="Skip return trip")
    args = parser.parse_args()

    cfg         = PINS[args.axis]
    ipp_theory  = theoretical_ipp(cfg['gear'], cfg['radius'])
    n_pulses    = dist_to_pulses(args.dist, cfg['gear'], cfg['radius'])
    has_enc     = cfg['enc']
    has_brake   = 'brake' in cfg

    print("=" * 64)
    print(f"  UHplift Drivetrain Calibration — {args.axis.upper()}")
    print("=" * 64)
    print(f"  Target distance  : {args.dist:.3f} in")
    print(f"  Theoretical IPP  : {ipp_theory:.8f} in/pulse")
    print(f"  Pulses to send   : {n_pulses}")
    print(f"  Pulse frequency  : {args.hz} Hz")
    print(f"  Encoder feedback : {'YES (CS=GPIO' + str(cfg['cs']) + ')' if has_enc else 'NO — measure manually'}")
    print(f"  Brake            : {'YES (GPIO' + str(cfg['brake']) + ')' if has_brake else 'no'}")
    print(f"  Pins             : PUL={cfg['pul']} DIR={cfg['dir']} (ENA hardwired)")
    print()

    pi = pigpio.pi()
    if not pi.connected:
        print("[ERROR] pigpiod not running — sudo pigpiod")
        sys.exit(1)

    # GPIO init
    pi.set_mode(cfg['pul'], pigpio.OUTPUT)
    pi.set_mode(cfg['dir'], pigpio.OUTPUT)
    pi.write(cfg['pul'], 0)
    pi.write(cfg['dir'], 1)

    # Brake init — engage by default
    if has_brake:
        pi.set_mode(cfg['brake'], pigpio.OUTPUT)
        pi.write(cfg['brake'], 1)
        print("[BRAKE] Engaged at startup")

    # Encoder init
    enc = None
    if has_enc:
        print("[ENC] Initializing...")
        enc = EncoderBusDiag(pi, cfg['cs'])
        enc_ok = enc.initialize()
        if not enc_ok:
            print("  [WARN] Encoder init failed — will proceed without count verification")
            enc = None

    try:
        input(f"\nPlace a mark at the current {args.axis} position. Press Enter to begin...")

        # Release brake if hoist
        if has_brake:
            pi.write(cfg['brake'], 0)
            print("[BRAKE] Released — waiting 300ms...")
            time.sleep(0.3)

        if enc: enc.clear()
        counts_pre = enc.read_counts() if enc else None

        # ── FORWARD ───────────────────────────────────────────────────────────
        print(f"\n[FWD] Sending {n_pulses} pulses at {args.hz} Hz...")
        send_pulses_ramped(pi, cfg['pul'], n_pulses, args.hz)
        time.sleep(0.3)

        counts_post = enc.read_counts() if enc else None

        print(f"\n[RESULT — FORWARD]")
        if enc and counts_pre is not None and counts_post is not None:
            delta_counts = counts_post - counts_pre
            dist_enc     = abs(delta_counts) * abs(cfg['enc_ipc'])
            cal_enc_ipc  = args.dist / abs(delta_counts) if delta_counts != 0 else 0
            print(f"  Encoder counts   : {delta_counts}")
            print(f"  Distance (enc)   : {dist_enc:.4f} in  (using current ENC_IPC)")
            print(f"  Tracking         : {100.0 * dist_enc / args.dist:.2f}%")

        # Manual measurement input
        print(f"\n  >>> Measure actual distance traveled with a ruler <<<")
        dist_str = input("  Enter measured distance [inches] (Enter to skip): ").strip()

        if dist_str:
            d_actual    = float(dist_str)
            cal_ipp     = d_actual / n_pulses
            cal_radius  = cal_ipp * PULSES_PER_MOTOR_REV * cfg['gear'] / (2 * math.pi)
            error_pct   = 100.0 * (d_actual - args.dist) / args.dist

            print(f"\n  ┌─────────────────────────────────────────────────────┐")
            print(f"  │  Commanded        : {args.dist:.4f} in                    │")
            print(f"  │  Actual measured  : {d_actual:.4f} in                    │")
            print(f"  │  Error            : {error_pct:+.2f}%                       │")
            print(f"  ├─────────────────────────────────────────────────────┤")
            print(f"  │  UPDATE stepper.py:                                 │")
            print(f"  │    wheel_radius_in = {cal_radius:.5f}  (was {cfg['radius']:.5f})  │")
            print(f"  │  Corrected IPP    = {cal_ipp:.8f} in/pulse        │")
            if enc and dist_str and delta_counts != 0:
                cal_ipc = d_actual / abs(delta_counts)
                print(f"  ├─────────────────────────────────────────────────────┤")
                print(f"  │  UPDATE main.py ENC_IPC:                            │")
                sign = '-' if cfg['enc_ipc'] < 0 else ' '
                print(f"  │    '{args.axis}': {sign}{cal_ipc:.8f}              │")
            print(f"  └─────────────────────────────────────────────────────┘")

        # ── RETURN ────────────────────────────────────────────────────────────
        if not args.no_rev:
            input("\nPress Enter to run reverse (return to start)...")
            pi.write(cfg['dir'], 0)
            time.sleep(0.00001)

            print(f"[REV] Sending {n_pulses} pulses...")
            send_pulses_ramped(pi, cfg['pul'], n_pulses, args.hz)
            time.sleep(0.3)

            if enc:
                counts_final = enc.read_counts()
                net = counts_final - counts_pre
                print(f"\n  Net residual counts: {net}  (should be ~0)")
                if abs(net) <= 10:
                    print(f"  ✓ Return verified")
                else:
                    print(f"  [NOTE] Residual > 10 counts — possible backlash or missed steps")

    except KeyboardInterrupt:
        print("\n[ABORT]")
    finally:
        # Always engage brake and stop pulses on exit
        pi.wave_tx_stop()
        pi.write(cfg['pul'], 0)
        pi.write(cfg['dir'], 0)
        if has_brake:
            pi.write(cfg['brake'], 1)
            print("[BRAKE] Engaged")
        if enc: enc.close()
        pi.stop()
        print("[CLEANUP] Done")

if __name__ == "__main__":
    main()
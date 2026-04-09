#!/usr/bin/env python3
"""
UHplift — Unified Drivetrain Calibration (v2)
Team 11 Capstone II

A safe, command-line driven calibration script that uses the exact
Software Chip Select and GPIO mapping from the final production stack.

Usage:
    python3 calibrate_drivetrain_v2.py --axis bridge --revs 1.0 --hz 2000
    python3 calibrate_drivetrain_v2.py --axis trolley --revs 2.0
    python3 calibrate_drivetrain_v2.py --axis hoist --hz 1000
"""

import pigpio
import spidev
import time
import math
import argparse
import sys

# ══════════════════════════════════════════════════════════════════════════════
# Authoritative Hardware Map (Matches main.py and config.py)
# ══════════════════════════════════════════════════════════════════════════════

PINS = {
    'trolley': {'pul': 22, 'dir': 27, 'ena': 17, 'cs': 8,  'gear': 5.0},
    'bridge':  {'pul': 23, 'dir': 24, 'ena': 25, 'cs': 7,  'gear': 4.0},
    'hoist':   {'pul': 5,  'dir': 6,  'ena': 13, 'cs': 16, 'gear': 10.0},
}

STEPS_PER_REV = 200
MICROSTEP = 4
PULSES_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEP  # 800
COUNTS_PER_MOTOR_REV = 4000  # 1000 PPR × 4 quadrature

# CL42T Timing
T1_ENA_DIR_MS = 250
T2_DIR_PUL_US = 10
RAMP_CHUNK = 200
MIN_HALF_PERIOD_US = 2

# ══════════════════════════════════════════════════════════════════════════════
# Encoder Bus (Mirrors main.py Software CS Logic)
# ══════════════════════════════════════════════════════════════════════════════

class EncoderBusDiag:
    WR_MDR0  = 0x88; WR_MDR1  = 0x90; CLR_CNTR = 0x20
    LOAD_OTR = 0xE8; RD_OTR   = 0x68; RD_MDR0  = 0x48

    def __init__(self, pi_handle, cs_pin):
        self._pi = pi_handle
        self.cs = cs_pin
        self._spi = spidev.SpiDev()
        self._spi.open(0, 0)
        self._spi.no_cs = True
        self._spi.mode = 0
        self._spi.max_speed_hz = 500_000
        
        # Init GPIO for Software CS
        self._pi.set_mode(self.cs, pigpio.OUTPUT)
        self._pi.write(self.cs, 1)

    def _xfer(self, data):
        self._pi.write(self.cs, 0); time.sleep(0.000002)
        result = self._spi.xfer2(list(data))
        self._pi.write(self.cs, 1); time.sleep(0.000002)
        return result

    def initialize(self):
        self._xfer([self.WR_MDR0, 0x03]); time.sleep(0.001)
        self._xfer([self.WR_MDR1, 0x00]); time.sleep(0.001)
        self._xfer([self.CLR_CNTR]);      time.sleep(0.001)
        rb = self._xfer([self.RD_MDR0, 0x00])
        ok = rb[1] == 0x03
        print(f"[ENC] MDR0 readback: 0x{rb[1]:02X} {'✓' if ok else '✗ FAIL'} (CS=GPIO{self.cs})")
        return ok

    def read_counts(self):
        self._xfer([self.LOAD_OTR]); time.sleep(0.0001)
        raw = self._xfer([self.RD_OTR, 0, 0, 0, 0])
        c = (raw[1]<<24)|(raw[2]<<16)|(raw[3]<<8)|raw[4]
        if c >= 0x80000000: c -= 0x100000000
        return c

    def clear(self):
        self._xfer([self.CLR_CNTR])
        time.sleep(0.001)

    def close(self):
        self._spi.close()

# ══════════════════════════════════════════════════════════════════════════════
# pigpio Waveform Helpers
# ══════════════════════════════════════════════════════════════════════════════

def hz_to_half_us(hz):
    return max(int(1_000_000 / (2 * max(hz, 1))), MIN_HALF_PERIOD_US)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def send_pulses_ramped(pi, pul_pin, total, target_hz):
    """Blocking trapezoidal ramp."""
    if total <= 0: return
    
    t = max(float(target_hz), 1.0)
    start_hz = clamp(t * 0.10, 80.0, 400.0)
    accel = clamp(t * 3.0, 300.0, 12000.0)
    decel = clamp(t * 3.0, 300.0, 12000.0)
    if t < start_hz: start_hz = max(1.0, t * 0.5)
    
    f0 = max(start_hz, 1.0)
    fT = max(t, f0)
    
    def ramp_p(fa, fb, s): return (fb*fb - fa*fa) / (2.0 * max(s, 1e-6))
    
    pa = ramp_p(f0, fT, accel)
    pd = ramp_p(f0, fT, decel)
    
    if pa + pd >= total:
        denom = (1.0/accel) + (1.0/decel)
        fp = math.sqrt(max(f0*f0 + 2.0*total/denom, f0*f0))
        cruise = 0
    else:
        fp = fT
        cruise = int(round(total - pa - pd))
    
    ap = int(round(ramp_p(f0, fp, accel)))
    dp = int(round(ramp_p(f0, fp, decel)))
    used = ap + cruise + dp
    if used != total:
        cruise += (total - used)
        cruise = max(cruise, 0)
    
    def send_chunk(n, half_us):
        pi.wave_clear()
        mask = 1 << pul_pin
        pulses = [pigpio.pulse(mask, 0, half_us), pigpio.pulse(0, mask, half_us)] * n
        pi.wave_add_generic(pulses)
        wid = pi.wave_create()
        pi.wave_send_once(wid)
        while pi.wave_tx_busy(): time.sleep(0.005)
        pi.wave_delete(wid)
    
    f = f0; rem = ap
    while rem > 0:
        dp_n = min(RAMP_CHUNK, rem)
        send_chunk(dp_n, hz_to_half_us(f))
        f = min(math.sqrt(max(f*f + 2*accel*dp_n, f0*f0)), fp)
        rem -= dp_n
    
    rem = cruise
    while rem > 0:
        dp_n = min(RAMP_CHUNK, rem)
        send_chunk(dp_n, hz_to_half_us(fp))
        rem -= dp_n
    
    f = fp; rem = dp
    while rem > 0:
        dp_n = min(RAMP_CHUNK, rem)
        send_chunk(dp_n, hz_to_half_us(f))
        f = math.sqrt(max(f*f - 2*decel*dp_n, f0*f0))
        rem -= dp_n

# ══════════════════════════════════════════════════════════════════════════════
# Main Execution
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="Unified Drivetrain Calibration")
    parser.add_argument("--axis", choices=["trolley", "bridge", "hoist"], required=True,
                        help="Which axis to calibrate (sets pins/gear ratio automatically)")
    parser.add_argument("--revs", type=float, default=1.0,
                        help="Output shaft revolutions to command (default: 1)")
    parser.add_argument("--hz", type=int, default=2000,
                        help="Target pulse frequency (default: 2000)")
    args = parser.parse_args()
    
    cfg = PINS[args.axis]
    total_pulses = int(PULSES_PER_MOTOR_REV * cfg['gear'] * args.revs)
    expected_counts = int(COUNTS_PER_MOTOR_REV * cfg['gear'] * args.revs)
    
    print("=" * 64)
    print(f"  UHplift Drivetrain Calibration: {args.axis.upper()}")
    print("=" * 64)
    print(f"  Gear ratio:  {cfg['gear']}:1")
    print(f"  Commanding:  {args.revs} output rev = {total_pulses} pulses")
    print(f"  Expected:    {expected_counts} encoder counts")
    print(f"  Ramp speed:  {args.hz} Hz")
    print(f"  Pins:        PUL={cfg['pul']}, DIR={cfg['dir']}, ENA={cfg['ena']}, CS={cfg['cs']}")
    print()
    
    pi = pigpio.pi()
    if not pi.connected:
        print("[ERROR] pigpio daemon not running — run: sudo pigpiod")
        sys.exit(1)
    
    # Init GPIO
    pi.set_mode(cfg['pul'], pigpio.OUTPUT)
    pi.set_mode(cfg['dir'], pigpio.OUTPUT)
    pi.set_mode(cfg['ena'], pigpio.OUTPUT)
    pi.write(cfg['pul'], 0)
    pi.write(cfg['dir'], 1)  # Forward
    pi.write(cfg['ena'], 1)  # Disabled
    
    # Init Encoder using correct Software CS
    enc = EncoderBusDiag(pi, cfg['cs'])
    if not enc.initialize():
        print("[WARN] Encoder readback failed — counts may be unreliable")
    
    try:
        input(f"\nPlace a reference mark on the {args.axis} shaft/rail. Press Enter to enable...")
        
        pi.write(cfg['ena'], 0)
        print(f"[ENA] Waiting {T1_ENA_DIR_MS}ms for brake/driver...")
        time.sleep(T1_ENA_DIR_MS / 1000)
        
        pi.write(cfg['dir'], 1)
        time.sleep(T2_DIR_PUL_US / 1_000_000)
        
        enc.clear()
        enc_pre = enc.read_counts()
        
        # ── FWD ───────────────────────────────────────────────────────
        print(f"\n[FWD] Sending {total_pulses} pulses...")
        send_pulses_ramped(pi, cfg['pul'], total_pulses, args.hz)
        time.sleep(0.3)
        
        enc_fwd = enc.read_counts()
        delta_fwd = enc_fwd - enc_pre
        
        print(f"  Encoder:  {delta_fwd} counts")
        if expected_counts > 0:
            print(f"  Tracking: {100.0 * delta_fwd / expected_counts:.2f}%")
        
        print(f"\n  >>> Measure the distance traveled <<<")
        dist_str = input("  Enter measured distance [inches] (or press Enter to skip): ")
        
        if dist_str.strip():
            dist_in = float(dist_str.strip())
            cal_ipc = dist_in / abs(delta_fwd) if delta_fwd != 0 else 0
            pulses_per_out = PULSES_PER_MOTOR_REV * cfg['gear']
            cal_ipp = dist_in / total_pulses if total_pulses > 0 else 0
            implied_r = cal_ipp * pulses_per_out / (2 * math.pi)
            
            print(f"\n  ┌─────────────────────────────────────────┐")
            print(f"  │  in_per_count    = {cal_ipc:.8f}         │ (For main.py ENC_IPC)")
            print(f"  │  inches_per_pulse = {cal_ipp:.8f}        │")
            print(f"  │  wheel_radius    = {implied_r:.4f} in            │ (For stepper.py config)")
            print(f"  └─────────────────────────────────────────┘")
        
        # ── REV ───────────────────────────────────────────────────────
        input("\nPress Enter to run reverse (same distance back)...")
        pi.write(cfg['dir'], 0)
        time.sleep(T2_DIR_PUL_US / 1_000_000)
        
        print(f"[REV] Sending {total_pulses} pulses...")
        send_pulses_ramped(pi, cfg['pul'], total_pulses, args.hz)
        time.sleep(0.3)
        
        enc_rev = enc.read_counts()
        net = enc_rev - enc_pre
        
        print(f"  Net residual counts (should be ~0): {net}")
        if abs(net) > 10:
            print(f"  [NOTE] May indicate mechanical backlash or missed steps.")
        else:
            print(f"  ✓ Return-to-origin verified")
            
    except KeyboardInterrupt:
        print("\n[ABORT]")
    finally:
        pi.write(cfg['ena'], 1)  # Disable
        pi.write(cfg['pul'], 0)
        pi.write(cfg['dir'], 0)
        pi.stop()
        enc.close()
        print("\n[CLEANUP] Done")

if __name__ == "__main__":
    main()
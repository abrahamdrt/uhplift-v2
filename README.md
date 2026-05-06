# UHplift — Active Sway Suppression Control System

**Team 11 | MECE 4341 Capstone Design II | University of Houston | Spring 2026**

Abraham Duarte · Eric Cabrera · Kevin Holbrook · Astrid Ruiz

---

## Overview

UHplift is the control software stack for a 1:8 dynamically similar CMAA Class C overhead bridge crane testbed with active load sway suppression. The system runs a 200 Hz closed-loop LQR controller on a Raspberry Pi 4B, using quadrature encoder feedback for position/velocity and an IMU complementary filter for real-time sway angle estimation.

The controller targets simultaneous velocity tracking and sway suppression across a 16-point operating envelope (payload mass × hoist length), with gain scheduling handled at runtime from a precomputed table.

---

## Repository Structure

```
uhplift-v2/
├── core/
│   ├── config.py        # System parameters, LQR gain tables, coordinate system
│   ├── controller.py    # LQI control law with directional anti-windup
│   ├── estimator.py     # DerivativeEstimator, ComplementaryFilter, LowPassFilter
│   ├── plant.py         # State-space matrices, open-loop analysis, RK4 simulation
│   └── reference.py     # Slew-rate-limited velocity reference generator
├── drivers/
│   ├── encoder.py       # LS7366R SPI quadrature counter driver
│   ├── imu.py           # MPU6050 I2C driver with fault recovery
│   ├── joystick.py      # PS4 DualShock 4 HMI driver
│   └── stepper.py       # pigpio DMA waveform stepper driver (CL42T)
├── main.py              # Top-level control loop and state machine
├── diag_sensors.py      # Standalone sensor health check utility
└── plot_run.py          # Post-run CSV log plotter
```

---

## Coordinate System

All axes are fixed to the physical crane frame and immutable throughout the codebase.

| Axis | Physical Direction | Quantity |
|------|--------------------|----------|
| X    | Bridge — North(−) → South(+) | Position, velocity, sway θ_x in XY plane |
| Z    | Trolley — West(−) → East(+)  | Position, velocity, sway θ_z in ZY plane |
| Y    | Hoist — Up(+) / Down(−)      | Hoist length L increases downward |

Units throughout: **lbm, inches, seconds**.

---

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| Compute | Raspberry Pi 4B | — |
| Stepper drivers | CL42T-V4.1 (×3) | GPIO PUL/DIR/ENA via ULN2003AN |
| Encoders | LS7366R 32-bit quadrature counter (×4) | SPI0, software CS |
| Encoder line receiver | AM26LS32 differential line receiver | — |
| Logic level translation | SN74HCT244 octal buffer (3.3V ↔ 5V) | — |
| IMU | MPU6050 | I2C @ 400 kHz, address 0x68 |
| HMI | PS4 DualShock 4 | USB |
| Hoist brake | Relay-controlled electromagnetic brake | GPIO 26 |

**GPIO pin assignments** (defined in `main.py`):

| Axis | PUL | DIR | ENA | Encoder CS |
|------|-----|-----|-----|------------|
| Trolley | 22 | 27 | 17 | 8 |
| Bridge | 23 | 24 | 25 | 7 (primary), 12 (diag) |
| Hoist | 5 | 6 | 13 | 16 |

> **Relay polarity:** Motor enable relays (K1/K2/K3) are wired to NO terminals (active-HIGH). The hoist brake relay (K4) is wired to NC — opposite polarity. This is handled in software but must be respected if the relay board is rewired.

---

## Installation

**Target platform:** Raspberry Pi 4B running Raspberry Pi OS (64-bit).

### 1. System dependencies

```bash
sudo apt update
sudo apt install python3-pip pigpio python3-pigpio i2c-tools
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### 2. Python dependencies

```bash
pip3 install numpy scipy spidev smbus2 matplotlib
```

### 3. I2C and SPI configuration

Add to `/boot/firmware/config.txt`:

```
dtparam=spi=on
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```

Then reboot. The 400 kHz I2C rate is required for reliable communication over the ~10 ft harness to the trolley IMU.

### 4. Clone and deploy

```bash
git clone https://github.com/abrahamdrt/uhplift-v2.git
cd uhplift-v2
```

---

## Usage

### Running the control loop

```bash
# Manual jog — all axes, no LQR (good for first power-on verification)
python3 main.py --mode manual --log

# Trolley LQR active, bridge locked at zero
python3 main.py --mode trolley --log

# Bridge LQR active, trolley locked at zero
python3 main.py --mode bridge --log
```

The `--log` flag writes a timestamped CSV to `./logs/run_YYYYMMDD_HHMMSS.csv`.

### Sensor checkout (no control loop)

```bash
# All sensors — encoder positions, IMU angles, joystick inputs
python3 diag_sensors.py

# Encoders only
python3 diag_sensors.py --enc-only

# IMU only
python3 diag_sensors.py --imu-only
```

Run this before every test session to confirm all sensor channels are live.

### Post-run plotting

```bash
# Plot most recent log in ./logs/
python3 plot_run.py

# Plot a specific file
python3 plot_run.py logs/run_20260501_143022.csv
```

Generates 5 PNGs: velocity tracking, sway angle, sway rate, control force, and commanded vs. actual velocity.

---

## Operator Interface (PS4 DualShock 4)

| Input | Action |
|-------|--------|
| Cross (B0) | Enable / disable all drives |
| Circle (B1, held) | Software stop |
| Square (B3) | Toggle MANUAL ↔ AUTO (bridge/trolley modes only) |
| Options (B9) | Zero all encoders |
| L1 + D-Up | Home bridge encoder to 0 |
| L1 + D-Left | Home trolley encoder to 0 |
| L1 + R2 | Home hoist to L = 12.0 in |
| D-pad U/D | Bridge velocity command |
| D-pad L/R | Trolley velocity command |
| L2 / R2 | Hoist down / up |

In **AUTO** mode, the joystick continues to set the velocity reference. The LQI controller augments that reference with a sway-correcting force term — it does not override the operator command.

---

## Control Architecture

### Plant model

The crane-load system is modeled as a linearized cart-pendulum (small-angle, θ ≤ 15°). The state vector for the trolley and bridge axes is:

```
x = [v, θ, θ_dot, ∫e_v]ᵀ
```

where `v` is carriage velocity, `θ` is sway angle, `θ_dot` is sway rate, and `∫e_v` is the accumulated velocity error. Position is excluded to avoid the uncontrollable integrator mode; integral action is added explicitly for zero steady-state velocity error.

State-space matrices are computed in `core/plant.py`. Open-loop analysis at the nominal operating point (max payload, max hoist length) gives ζ < 0.4% and a free-decay settling time > 3 minutes, establishing the need for active control.

### LQR gain computation

Gains were computed offline using `tune_trolley_gains.py` via `scipy.linalg.solve_continuous_are` (equivalent to MATLAB's `lqr()`). A grid search over the Q weight space was used to select the cost function weighting that minimizes a three-term objective: peak sway (40%), sway settling (40%), and velocity settling (20%). The resulting gain sets are stored in `config.py` as `GainSet` objects indexed by `(m_l, L)`.

### Gain scheduling

At runtime, `get_gains(m_l, L)` in `config.py` finds the nearest entry in the gain table by Euclidean distance in the (mass, length) space and loads it into the controller via `set_gains()`. The scheduler re-triggers whenever the estimated hoist length changes by more than 1 inch.

### Encoder inches-per-count calibration

| Axis | IPC [in/count] | Notes |
|------|---------------|-------|
| Trolley | −0.000152 | Negated for coordinate convention |
| Bridge | +0.000190 | |
| Hoist | +0.000118 | |

These were determined experimentally by commanding a known displacement and comparing commanded vs. measured counts. Update these values in `main.py` if the drivetrain is modified.

---

## Known Limitations and Open Items

- **Hoist encoder feedback:** Hoist position is measured but hoist length L is currently initialized from the hoist home position and tracked by dead-reckoning from step count. A closed-loop L estimate from the hoist encoder is a TODO flagged in `main.py` near gain initialization.
- **Bridge axis validation:** Trolley single-axis LQR is validated on hardware. Bridge axis LQR has been verified in simulation; hardware closed-loop validation is the next step.
- **Dual-motor bridge synchronization:** Both bridge motors share a single PUL/DIR/ENA signal set from one Pi source with one encoder for feedback. Synchronization relies on matched mechanical load; independent current monitoring per motor is recommended for future iterations.
- **Velocity LPF:** A low-pass filter for velocity estimation is defined in `estimator.py` but the cutoff tuning should be verified under actual load — encoder differentiation at low speeds produces artifacts that the current 20 Hz cutoff may not fully reject.

---

## Lessons Learned — Hardware Integration

These are documented here to protect future teams using similar architectures.

**Pi GPIO current limits:** The Raspberry Pi 4B GPIO pins source a maximum of ~16 mA at 3.3V. Relay coil loads (typically 60–80 mA total across multiple coils) must never be driven directly from GPIO. Use a ULN2003AN, ULN2803A, or equivalent driver IC at every relay interface.

**Floating ground references:** All 5V and 3.3V supply commons must share a single reference point tied to protective earth. Floating ground references across mixed-supply boards cause ground potential divergence under load and can produce destructive voltage transients on SPI/I2C lines. This failure mode destroyed four LS7366R ICs, the SN74HCT244, and the original Pi 4B during commissioning.

**Level shifting at every logic boundary:** Every 3.3V ↔ 5V signal boundary requires a buffer or level shifter. The SN74HCT244 between the LS7366R outputs and the Pi SPI bus is non-optional.

**I2C on long harnesses:** At cable lengths above ~1 m, use 400 kHz fast mode with explicit pull-up resistors. Enforce a kernel-level I2C timeout (via `fcntl` ioctl) to prevent any single failed transaction from blocking the control loop tick.

---

## License

Academic project. Not licensed for commercial use.  
Contact: `aduarte3@cougarnet.uh.edu`

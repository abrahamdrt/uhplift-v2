"""
LS7366R Quadrature Encoder Counter — Standalone SPI Driver
For standalone diagnostics only. Main control loop uses EncoderBus in main.py.

Hardware chain:
    Encoder (1000 PPR) -> AM26LS32ACN -> LS7366R -> CD74HCT244E -> Pi SPI0

Software CS mapping:
    CS1: GPIO 8  (Trolley)
    CS2: GPIO 16 (Hoist)
    CS3: GPIO 7  (Bridge master)
    CS4: GPIO 12 (Bridge diagnostic)
"""

import spidev
import time
from dataclasses import dataclass

CMD_WR_MDR0  = 0x88; CMD_WR_MDR1 = 0x90; CMD_CLR_CNTR = 0x20
CMD_LOAD_OTR = 0xE8; CMD_RD_OTR  = 0x68; CMD_RD_MDR0  = 0x48


@dataclass
class EncoderConfig:
    name: str
    spi_bus: int = 0
    spi_ce: int = 0
    spi_speed_hz: int = 500_000
    counts_per_rev: int = 4000
    gear_ratio: float = 4.0
    wheel_radius_in: float = 0.485
    in_per_count: float = 0.000190


class LS7366R:
    def __init__(self, config, simulation_mode=False):
        self.config = config
        self.simulation_mode = simulation_mode
        self._spi = None; self._initialized = False; self._offset = 0

    def initialize(self):
        if self.simulation_mode:
            self._initialized = True; return True
        try:
            self._spi = spidev.SpiDev()
            self._spi.open(self.config.spi_bus, self.config.spi_ce)
            self._spi.mode = 0; self._spi.max_speed_hz = self.config.spi_speed_hz
            self._spi.xfer2([CMD_WR_MDR0, 0x03]); time.sleep(0.001)
            self._spi.xfer2([CMD_WR_MDR1, 0x00]); time.sleep(0.001)
            self._spi.xfer2([CMD_CLR_CNTR]);       time.sleep(0.001)
            rb = self._spi.xfer2([CMD_RD_MDR0, 0x00])
            ok = rb[1] == 0x03
            print(f"[ENCODER:{self.config.name}] MDR0=0x{rb[1]:02X} "
                  f"{'ok' if ok else 'FAIL'}")
            self._initialized = ok; return ok
        except Exception as e:
            print(f"[ENCODER:{self.config.name}] {e}"); return False

    def read_counts(self):
        if self.simulation_mode or not self._initialized: return 0
        self._spi.xfer2([CMD_LOAD_OTR]); time.sleep(0.0001)
        raw = self._spi.xfer2([CMD_RD_OTR, 0, 0, 0, 0])
        c = (raw[1]<<24)|(raw[2]<<16)|(raw[3]<<8)|raw[4]
        if c >= 0x80000000: c -= 0x100000000
        return c - self._offset

    def read_position(self):
        return self.read_counts() * self.config.in_per_count

    def zero(self):
        self._offset = self.read_counts() + self._offset

    def close(self):
        if self._spi: self._spi.close()


# Pre-configured (for standalone use — main.py uses EncoderBus instead)
TROLLEY_ENCODER = EncoderConfig(name="trolley", spi_ce=0, gear_ratio=5.0,
                                 in_per_count=0.000152)
BRIDGE_ENCODER  = EncoderConfig(name="bridge", spi_ce=1, gear_ratio=4.0,
                                 in_per_count=0.000190)
HOIST_ENCODER   = EncoderConfig(name="hoist", spi_ce=0, gear_ratio=10.0,
                                 in_per_count=0.000118)

def create_encoder(axis, simulation_mode=False):
    c = {"trolley":TROLLEY_ENCODER, "bridge":BRIDGE_ENCODER, "hoist":HOIST_ENCODER}
    return LS7366R(c[axis], simulation_mode)

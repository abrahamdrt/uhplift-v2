"""
USB Game Controller HMI Driver
Pendant-style interface for UHplift crane control

Design intent:
    Matches the original AC pendant behavior — discrete directional commands
    at v_target, not proportional speed. The D-pad provides four digital
    directions for trolley and bridge. L2/R2 buttons provide digital hoist
    up/down. All motion commands feed ManualModeGenerator which applies 0.20g
    rate limiting, so motion is always smooth regardless of input being digital.

Validated hardware:
    Sony DualShock 4 (PS4) via USB, tested under Linux/pygame.
    D-pad reports as Hat 0 with (x, y) tuples — NOT individual buttons.
    L2/R2 report as buttons B6/B7 — NOT analog axes.

Confirmed PS4 mapping (verified via map_controller.py):
    Trolley  <- D-pad Left/Right  -> Hat 0 x-component  (-1, 0, +1)
    Bridge   <- D-pad Up/Down     -> Hat 0 y-component  (-1, 0, +1)
    Hoist down <- L2              -> Button 6            (0 or 1)
    Hoist up   <- R2              -> Button 7            (0 or 1)
    Enable   <- Cross    (B0)     -> rising edge in main.py
    Soft stop<- Circle   (B1)     -> held state in main.py
    Mode     <- Square   (B3)     -> rising edge in main.py
    Reset    <- Options  (B9)     -> rising edge in main.py

Note: Circle (B1) is a SOFTWARE disable only. The hardware E-stop is a
separate normally-closed AC line interlock upstream of the PSU, independent
of software.

Headless Pi:
    SDL_VIDEODRIVER and SDL_AUDIODRIVER are set to "dummy" before pygame.init()
    so the driver works on a Pi without a display.
"""

import os
from typing import Optional, Dict, Tuple
from dataclasses import dataclass


# ── Configuration ─────────────────────────────────────────────────────────────

@dataclass
class JoystickConfig:
    """
    Controller mapping for pendant-style crane HMI.

    Trolley and bridge use the D-pad hat — returns discrete -1, 0, +1.
    Hoist uses L2/R2 as digital buttons — 0 at rest, 1 when pressed.
    """

    # ── D-pad hat ─────────────────────────────────────────────────────────────
    # D-pad reports as a hat with (x, y) tuple.
    #   x: -1 = Left (trolley reverse), +1 = Right (trolley forward)
    #   y: -1 = Down (bridge reverse),  +1 = Up    (bridge forward)
    dpad_hat_index: int = 0

    # ── Hoist buttons (digital: 0 at rest, 1 when pressed) ───────────────────
    hoist_down_button: int = 6  # L2
    hoist_up_button:   int = 7  # R2

    # ── Face buttons ──────────────────────────────────────────────────────────
    enable_button: int = 0      # Cross   — toggle drives on/off (edge)
    estop_button:  int = 1      # Circle  — software disable (held state)
    mode_button:   int = 3      # Square  — toggle MANUAL <-> AUTO (edge)
    reset_button:  int = 9      # Options — fault reset / zero encoders (edge)


# ── Driver ────────────────────────────────────────────────────────────────────

class JoystickDriver:
    """
    Pendant-style USB game controller driver for crane HMI.

    Primary interface for the 200 Hz control loop:
        update()               -- call once per tick to latch current state
        get_trolley_input()    -- returns -1, 0, or +1  (D-pad Left/Right)
        get_bridge_input()     -- returns -1, 0, or +1  (D-pad Down/Up)
        get_hoist_input()      -- returns -1, 0, or +1  (L2 down / R2 up)
        get_*_pressed()        -- button state booleans (edge detection in main.py)
    """

    def __init__(self,
                 config: Optional[JoystickConfig] = None,
                 simulation_mode: bool = False):
        """
        Args:
            config: Axis/button mapping. Defaults to validated PS4 mapping.
            simulation_mode: Skip pygame; use set_sim_inputs() for testing.
        """
        self.config = config or JoystickConfig()
        self.simulation_mode = simulation_mode

        self._pygame_initialized: bool = False
        self._joystick = None
        self._connected: bool = False

        # Latched hardware state (updated each update() call)
        self._axes:    Dict[int, float]           = {}
        self._buttons: Dict[int, bool]            = {}
        self._hats:    Dict[int, Tuple[int, int]] = {}

        # Simulation overrides
        self._sim_trolley: float = 0.0
        self._sim_bridge:  float = 0.0
        self._sim_hoist:   float = 0.0

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def initialize(self) -> bool:
        """
        Initialize pygame and connect to the first available joystick.

        Sets SDL environment variables for headless Pi operation before
        calling pygame.init().

        Returns:
            True if a joystick was found and initialized.
        """
        if self.simulation_mode:
            print("[JOYSTICK] Simulation mode — no hardware required")
            return True

        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

        try:
            import pygame
            pygame.init()
            pygame.joystick.init()
            self._pygame_initialized = True

            count = pygame.joystick.get_count()
            if count == 0:
                print("[JOYSTICK] No controller found")
                print("  -> Connect a USB gamepad and re-run")
                return False

            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()

            name        = self._joystick.get_name()
            num_axes    = self._joystick.get_numaxes()
            num_buttons = self._joystick.get_numbuttons()
            num_hats    = self._joystick.get_numhats()

            print(f"[JOYSTICK] Connected: {name}")
            print(f"  Axes: {num_axes} | Buttons: {num_buttons} | Hats: {num_hats}")

            if num_hats == 0:
                print("  [WARN] No hat/D-pad detected. "
                      "Trolley and bridge will not respond.")

            self._connected = True
            return True

        except ImportError:
            print("[JOYSTICK] pygame not installed -- run: pip install pygame")
            return False
        except Exception as e:
            print(f"[JOYSTICK] Initialization failed: {e}")
            return False

    def close(self):
        """Release pygame resources."""
        if self._pygame_initialized:
            try:
                import pygame
                pygame.joystick.quit()
                pygame.quit()
            except Exception:
                pass
        self._connected = False

    # ── Per-tick update ───────────────────────────────────────────────────────

    def update(self) -> None:
        """
        Pump pygame events and latch current controller state.

        Call exactly once per control loop tick (every 5ms at 200 Hz).
        All get_*() methods read from this latched snapshot.
        """
        if self.simulation_mode or not self._connected or self._joystick is None:
            return

        try:
            import pygame
            pygame.event.pump()

            for i in range(self._joystick.get_numaxes()):
                self._axes[i] = self._joystick.get_axis(i)

            for i in range(self._joystick.get_numbuttons()):
                self._buttons[i] = bool(self._joystick.get_button(i))

            for i in range(self._joystick.get_numhats()):
                self._hats[i] = self._joystick.get_hat(i)

        except Exception as e:
            print(f"[JOYSTICK] Read error: {e}")

    # ── Motion inputs ─────────────────────────────────────────────────────────

    def get_trolley_input(self) -> float:
        """
        Trolley velocity command from D-pad Left/Right.

        Returns:
            +1.0  D-pad Right pressed  (forward, +X)
            -1.0  D-pad Left  pressed  (reverse, -X)
             0.0  D-pad neutral
        """
        if self.simulation_mode:
            return self._sim_trolley

        hat = self._hats.get(self.config.dpad_hat_index, (0, 0))
        return float(hat[0])

    def get_bridge_input(self) -> float:
        """
        Bridge velocity command from D-pad Up/Down.

        Returns:
            +1.0  D-pad Up   pressed  (forward, +Y)
            -1.0  D-pad Down pressed  (reverse, -Y)
             0.0  D-pad neutral
        """
        if self.simulation_mode:
            return self._sim_bridge

        hat = self._hats.get(self.config.dpad_hat_index, (0, 0))
        return float(hat[1])

    def get_hoist_input(self) -> float:
        """
        Hoist velocity command from L2 (down) and R2 (up).

        Both are digital buttons: 0 at rest, 1 when pressed.

        Returns:
            +1.0  R2 (B7) pressed  (hoist up,   +Z)
            -1.0  L2 (B6) pressed  (hoist down, -Z)
             0.0  neither or both pressed
        """
        if self.simulation_mode:
            return self._sim_hoist

        down = 1.0 if self._buttons.get(self.config.hoist_down_button, False) else 0.0
        up   = 1.0 if self._buttons.get(self.config.hoist_up_button,   False) else 0.0
        return up - down

    # ── Button state ──────────────────────────────────────────────────────────

    def get_enable_pressed(self) -> bool:
        """Cross (B0) -- enable/disable drives. Edge-detected in main.py."""
        if self.simulation_mode:
            return False
        return self._buttons.get(self.config.enable_button, False)

    def get_estop_pressed(self) -> bool:
        """
        Circle (B1) -- software disable. Held state: active while held.
        NOTE: NOT the hardware E-stop. Hardware E-stop is a separate
        normally-closed AC line interlock independent of software.
        """
        if self.simulation_mode:
            return False
        return self._buttons.get(self.config.estop_button, False)

    def get_mode_pressed(self) -> bool:
        """Square (B3) -- toggle MANUAL <-> AUTO. Edge-detected in main.py."""
        if self.simulation_mode:
            return False
        return self._buttons.get(self.config.mode_button, False)

    def get_reset_pressed(self) -> bool:
        """Options (B9) -- clear FAULT / zero encoders. Edge-detected in main.py."""
        if self.simulation_mode:
            return False
        return self._buttons.get(self.config.reset_button, False)

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def get_all_inputs(self) -> Dict:
        """Snapshot of all crane inputs. Useful for logging and --test-sensors."""
        return {
            'trolley': self.get_trolley_input(),
            'bridge':  self.get_bridge_input(),
            'hoist':   self.get_hoist_input(),
            'enable':  self.get_enable_pressed(),
            'estop':   self.get_estop_pressed(),
            'mode':    self.get_mode_pressed(),
            'reset':   self.get_reset_pressed(),
        }

    def get_raw_state(self) -> Dict:
        """Full raw hardware state for debugging."""
        return {
            'axes':    dict(self._axes),
            'buttons': dict(self._buttons),
            'hats':    dict(self._hats),
        }

    # ── Simulation helpers ────────────────────────────────────────────────────

    def set_sim_inputs(self,
                       trolley: float = 0.0,
                       bridge:  float = 0.0,
                       hoist:   float = 0.0) -> None:
        """Inject simulated motion commands for software-only testing."""
        self._sim_trolley = float(trolley)
        self._sim_bridge  = float(bridge)
        self._sim_hoist   = float(hoist)

    @property
    def is_connected(self) -> bool:
        return self._connected or self.simulation_mode


# ── Controller profile ────────────────────────────────────────────────────────

def detect_controller_type(name: str) -> JoystickConfig:
    """
    Return a JoystickConfig for the detected controller name string.

    Only the PS4 DualShock 4 mapping has been hardware-validated.
    Run map_controller.py to verify before use on any other controller.
    """
    name_lower = name.lower()

    if any(k in name_lower for k in
           ['playstation', 'dualshock', 'dualsense', 'wireless controller']):
        # ── Sony DualShock 4 / PS4 ── VALIDATED ──────────────────────────────
        # Confirmed via map_controller.py on Linux/pygame:
        #   D-pad   -> Hat 0  (x = trolley, y = bridge)
        #   L2/R2   -> Buttons 6 / 7  (digital: 0 at rest, 1 pressed)
        #   Cross   -> Button 0    Circle  -> Button 1
        #   Square  -> Button 3    Options -> Button 9
        return JoystickConfig(
            dpad_hat_index=0,
            hoist_down_button=6,
            hoist_up_button=7,
            enable_button=0,
            estop_button=1,
            mode_button=3,
            reset_button=9,
        )

    else:
        print(f"[JOYSTICK] Unknown controller '{name}' -- using PS4 defaults. "
              f"Run map_controller.py to verify mapping.")
        return JoystickConfig()
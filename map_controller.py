"""
Standalone controller mapping script.
Run this BEFORE anything else to confirm axis indices and trigger behavior.

Usage:
    pip install pygame
    python3 map_controller.py

What to look for:
    - Trolley stick (left stick up/down)  → which axis index changes? rest value = 0.0? REMOVED TO SIMPLIFY -> DPAD
    - Bridge stick (right stick up/down)  → which axis index changes? REMOVED TO SIMPLIFY -> DPAD
    - L2 trigger                          → which axis index? rest value = -1.0 or 0.0?
    - R2 trigger                          → which axis index? rest value = -1.0 or 0.0?
    - Each button press                   → which button index lights up?
    - D-Pad Mapping                       → maps to hat, H0 = (0,0)
"""

import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

import pygame
import time

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
if count == 0:
    print("No controller found. Plug in your PS4 controller and re-run.")
    exit(1)

joy = pygame.joystick.Joystick(0)
joy.init()

print(f"\nController : {joy.get_name()}")
print(f"Axes       : {joy.get_numaxes()}")
print(f"Buttons    : {joy.get_numbuttons()}")
print(f"Hats(DPad) : {joy.get_numhats()}")
print("\nMove each stick/trigger and press each button. Ctrl+C to quit.\n")
print("-" * 90)

prev_buttons = [0] * joy.get_numbuttons()
prev_hats = [(0, 0)] * joy.get_numhats()

try:
    while True:
        pygame.event.pump()

        axes    = [round(joy.get_axis(i), 2) for i in range(joy.get_numaxes())]
        buttons = [joy.get_button(i)         for i in range(joy.get_numbuttons())]
        hats    = [joy.get_hat(i)            for i in range(joy.get_numhats())]

        # Highlight non-zero axes
        axes_str = "  ".join(f"\033[93mA{i}={v:+.2f}\033[0m" if abs(v) > 0.05 else f"A{i}={v:+.2f}" for i, v in enumerate(axes))
        # Highlight pressed buttons
        btn_str = "  ".join(f"\033[92mB{i}=1\033[0m" if b else f"B{i}=0" for i, b in enumerate(buttons))
        # Highlight pressed hats (D-pad)
        hat_str = "  ".join(f"\033[96mH{i}={v}\033[0m" if v != (0,0) else f"H{i}={v}" for i, v in enumerate(hats))

        print(f"{axes_str:40s} {btn_str:30s} {hat_str}", end="\r")

        # Print newlines for clean logging
        for i, (now, prev) in enumerate(zip(buttons, prev_buttons)):
            if now and not prev:
                print(f"\n  >>> Button {i} pressed")
        prev_buttons = buttons[:]
        
        for i, (now, prev) in enumerate(zip(hats, prev_hats)):
            if now != prev and now != (0, 0):
                print(f"\n  >>> D-Pad Hat {i} pressed: {now}")
        prev_hats = hats[:]

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\nDone.")
    pygame.quit()
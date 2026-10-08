"""Gamepad layouts for upstream ``XBoxController``: run a PS4 pad through the Xbox code path.

Upstream reads pygame joystick indices fixed to its Xbox layout. ``RemappedJoystick`` sits between
it and pygame and translates those Xbox indices to the pad's own, so the walk logic is untouched.

Indices as seen by pygame on the Pi (SDL 2.28, Linux hid-playstation), verified 6 Oct 2026 on
Jero's DualShock 4: axes 0/1 left stick, 2 L2 (rests at -1), 3/4 right stick, 5 R2 (rests at -1).
Buttons follow the kernel order: 0 cross, 1 circle, 2 triangle, 3 square, 4 L1, 5 R1.

What the walk does with them (upstream v2_rl_walk_mujoco.py):
    left stick   walk / strafe          right stick X   turn
    A  (PS4 x)   pause / resume          LB (PS4 L1)     hold for faster stepping
    Y  (PS4 tri) toggle head mode        D-pad up/down   gait frequency trim
"""

from __future__ import annotations

import logging

log = logging.getLogger("jero.pads")

# upstream Xbox index -> pad index
XBOX = {"axes": {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5}, "buttons": {0: 0, 1: 1, 3: 3, 4: 4, 6: 6, 7: 7}}
PS4 = {
    # upstream: 0 l_x, 1 l_y, 2 r_x (turn), 3 r_y, 4 right trigger, 5 left trigger
    "axes": {0: 0, 1: 1, 2: 3, 3: 4, 4: 5, 5: 2},
    # upstream: 0 A, 1 B, 3 X, 4 Y, 6 LB, 7 RB
    "buttons": {0: 0, 1: 1, 3: 3, 4: 2, 6: 4, 7: 5},
}
LAYOUTS = {"xbox": XBOX, "ps4": PS4}
# what upstream's Xbox button names are on each pad (for logs)
BUTTON_NAMES = {
    "xbox": {},
    "ps4": {"A": "cross", "B": "circle", "X": "square", "Y": "triangle", "LB": "L1", "RB": "R1"},
}
PS4_NAMES = ("wireless controller", "dualshock", "ps4", "sony")


def detect(name: str) -> str:
    n = name.lower()
    if "xbox" in n or "microsoft" in n:
        return "xbox"
    return "ps4" if any(k in n for k in PS4_NAMES) else "xbox"


class RemappedJoystick:
    """pygame Joystick look-alike that answers upstream's Xbox indices from another layout."""

    def __init__(self, joystick, layout: dict):
        self._js = joystick
        self._axes = layout["axes"]
        self._buttons = layout["buttons"]

    def get_axis(self, i):
        return self._js.get_axis(self._axes.get(i, i))

    def get_button(self, i):
        j = self._buttons.get(i)
        return False if j is None else self._js.get_button(j)

    def __getattr__(self, name):  # init, get_hat, get_numaxes, get_name, ...
        return getattr(self._js, name)


def make_controller(command_freq: int, layout: str = "auto"):
    """Build upstream XBoxController with the pad's layout in place from its first read."""
    import pygame
    from mini_bdx_runtime import xbox_controller as xc

    pygame.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("no gamepad connected (pair it, then check /dev/input/js0)")
    probe = pygame.joystick.Joystick(0)
    probe.init()
    name = probe.get_name()
    chosen = detect(name) if layout == "auto" else layout
    log.info("gamepad %r: %s layout", name, chosen)

    real = xc.pygame.joystick.Joystick
    xc.pygame.joystick.Joystick = lambda i: RemappedJoystick(real(i), LAYOUTS[chosen])
    try:
        ctrl = xc.XBoxController(command_freq)  # its worker thread starts here, already remapped
        ctrl.jero_button_names = BUTTON_NAMES[chosen]
        return ctrl
    finally:
        xc.pygame.joystick.Joystick = real

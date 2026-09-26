"""Drop-in replacement for upstream XBoxController that merges the Xbox pad with the Jero link.

RLWalk only needs ``get_last_command() -> (commands[7], Buttons, left_trigger, right_trigger)``.

Priority, evaluated every control tick (50 Hz):
  1. Xbox sticks outside the deadband  -> Xbox drives (a human can always take over).
  2. Fresh Jero link command            -> link drives.
  3. Otherwise                          -> zero command (stand / step in place).
Buttons and triggers from both sources are OR-ed / max-ed.
"""

from __future__ import annotations

import logging
import time

import numpy as np
from mini_bdx_runtime.buttons import Buttons

log = logging.getLogger("jero.controller")

ZERO = [0.0] * 7


class MuxController:
    def __init__(self, xbox=None, link=None, deadband: float = 0.02, log_every_s: float = 5.0):
        self.xbox = xbox
        self.link = link
        self.deadband = deadband
        self.buttons = Buttons()
        self.source = "idle"
        self._last_log = 0.0
        self._log_every_s = log_every_s

    def _xbox(self):
        if self.xbox is None:
            return None
        cmds, btns, lt, rt = self.xbox.get_last_command()
        pressed = {
            "A": btns.A.is_pressed,
            "B": btns.B.is_pressed,
            "X": btns.X.is_pressed,
            "Y": btns.Y.is_pressed,
            "LB": btns.LB.is_pressed,
            "RB": btns.RB.is_pressed,
            "dpad_up": btns.dpad_up.is_pressed,
            "dpad_down": btns.dpad_down.is_pressed,
        }
        return list(np.asarray(cmds, dtype=float)), pressed, float(lt), float(rt)

    def get_last_command(self):
        commands, source = list(ZERO), "idle"
        pressed = {k: False for k in ("A", "B", "X", "Y", "LB", "RB", "dpad_up", "dpad_down")}
        lt = rt = 0.0

        pad = self._xbox()
        if pad is not None:
            pad_cmds, pad_pressed, lt, rt = pad
            pressed = {k: v or pad_pressed[k] for k, v in pressed.items()}
            if max(abs(c) for c in pad_cmds) > self.deadband:
                commands, source = pad_cmds, "xbox"

        if self.link is not None:
            link_cmd, link_pressed = self.link.snapshot()
            pressed = {k: v or link_pressed[k] for k, v in pressed.items()}
            if link_cmd is not None:
                lt, rt = max(lt, link_cmd.left_trigger), max(rt, link_cmd.right_trigger)
                if source == "idle":
                    commands, source = link_cmd.as_commands(), "link"

        self.buttons.update(
            pressed["A"],
            pressed["B"],
            pressed["X"],
            pressed["Y"],
            pressed["LB"],
            pressed["RB"],
            pressed["dpad_up"],
            pressed["dpad_down"],
        )
        self._maybe_log(source, commands)
        self.source = source
        return np.around(np.asarray(commands, dtype=float), 3), self.buttons, lt, rt

    def _maybe_log(self, source, commands):
        now = time.monotonic()
        if source != self.source or now - self._last_log > self._log_every_s:
            log.info("cmd source=%s vx=%.3f vy=%.3f wz=%.3f", source, *commands[:3])
            self._last_log = now

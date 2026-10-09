"""Run the upstream walk on a board that isn't a Raspberry Pi (e.g. Qualcomm RB3 Gen 2).

Upstream ``scripts/v2_rl_walk_mujoco.py`` imports four modules that need Adafruit Blinka's
``board`` (Raspberry Pi only): ``feet_contacts``, ``eyes``, ``antennas``, ``projector``
(``raw_imu`` too, but jero_walk.py already swaps that for robot/imu_mpu6050.py).
``install()`` puts replacements under the same module names in ``sys.modules`` before that
import, so upstream is not edited and the policy loop is unchanged.

Foot switches (the policy reads them as two of its 101 inputs):
  - ``none``:  always "not touching", exactly what unwired switches give on the Pi. The walk
              runs; in sim it recovers from pushes less well (5/8 vs 8/8 with real switches).
  - ``gpiod``: two GPIO lines via libgpiod (Linux character device), switch to GND, line
              pulled up, pressed = 0, same logic as upstream. Lines are set in
              ~/.config/jero/board.json or with --feet-chip / --feet-lines.
Eyes, antennas and the projector are expression features (duck_config.json: all false); their
stand-ins raise if anything tries to use them, rather than silently doing nothing.
"""

from __future__ import annotations

import json
import logging
import sys
import types
from pathlib import Path

log = logging.getLogger("jero.board")
CONFIG_PATH = Path.home() / ".config" / "jero" / "board.json"


def detect() -> str:
    """'pi' on a Raspberry Pi, else 'other' (RB3 Gen 2 and anything else)."""
    try:
        model = Path("/proc/device-tree/model").read_bytes().decode(errors="ignore")
    except OSError:
        return "other"
    return "pi" if "Raspberry Pi" in model else "other"


def load_config(path: Path = CONFIG_PATH) -> dict:
    try:
        return json.loads(Path(path).read_text())
    except FileNotFoundError:
        return {}


class NoFeetContacts:
    """Foot switches not wired: both feet always read 'not touching' (as on an unwired Pi)."""

    def get(self):
        return [False, False]

    def stop(self):
        pass


class GpiodFeetContacts:
    """Two foot switches on GPIO lines (libgpiod v2 Python bindings, ``pip install gpiod``).

    Wiring: each switch between its GPIO line and GND; the line is biased pull-up, so pressed
    reads 0, like upstream's digitalio version. Check the board's GPIO voltage first (the RB3
    Gen 2's low-speed connector is 1.8 V): a switch to GND is safe at any voltage.
    """

    def __init__(self, chip: str, left: int, right: int):
        import gpiod
        from gpiod.line import Bias, Direction, Value

        self.left, self.right = int(left), int(right)
        settings = gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)
        self.request = gpiod.request_lines(
            chip, consumer="jero-feet", config={(self.left, self.right): settings}
        )
        self._inactive = Value.INACTIVE

    def get(self):
        vals = self.request.get_values([self.left, self.right])
        return [v == self._inactive for v in vals]  # pulled up: pressed = low

    def stop(self):
        self.request.release()


def _unavailable(name: str):
    class Unavailable:
        def __init__(self, *args, **kwargs):
            raise RuntimeError(
                f"{name} needs Raspberry Pi GPIO (Adafruit board); on this board set "
                f"expression_features.{name.lower()} = false in duck_config.json"
            )

    Unavailable.__name__ = name
    return Unavailable


def _module(name: str, **attrs) -> types.ModuleType:
    mod = types.ModuleType(name)
    mod.__dict__.update(attrs)
    return mod


def make_feet(backend: str, chip: str | None = None, lines: tuple[int, int] | None = None):
    if backend == "none":
        return NoFeetContacts
    if backend == "gpiod":
        if not chip or not lines:
            raise SystemExit("--feet gpiod needs a chip and two lines (--feet-chip, --feet-lines L,R or board.json)")

        class FeetContacts(GpiodFeetContacts):
            def __init__(self):
                super().__init__(chip, *lines)

        return FeetContacts
    raise SystemExit(f"unknown feet backend {backend!r}")


def install(feet: str = "none", chip: str | None = None, lines: tuple[int, int] | None = None) -> None:
    """Replace upstream's Pi-only modules. Call before importing v2_rl_walk_mujoco."""
    feet_cls = make_feet(feet, chip, lines)
    sys.modules["mini_bdx_runtime.feet_contacts"] = _module(
        "mini_bdx_runtime.feet_contacts", FeetContacts=feet_cls
    )
    sys.modules["mini_bdx_runtime.eyes"] = _module("mini_bdx_runtime.eyes", Eyes=_unavailable("Eyes"))
    sys.modules["mini_bdx_runtime.antennas"] = _module(
        "mini_bdx_runtime.antennas", Antennas=_unavailable("Antennas")
    )
    sys.modules["mini_bdx_runtime.projector"] = _module(
        "mini_bdx_runtime.projector", Projector=_unavailable("Projector")
    )
    detail = f" ({chip} lines {lines[0]},{lines[1]})" if feet == "gpiod" else ""
    log.info("board shims: feet=%s%s; eyes/antennas/projector disabled", feet, detail)

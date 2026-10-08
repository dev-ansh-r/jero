#!/usr/bin/env python3
"""Show what the walk receives from the gamepad: same code path as jero_walk.py (robot/pads.py ->
upstream XBoxController, 20 Hz worker thread), next to the raw joystick values.

    python ~/Jero/tools/pad_test.py               # 20 s: move the sticks, press X/O/square/triangle
    python ~/Jero/tools/pad_test.py --background  # same, with SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1

Nothing moves on the robot (no servo access).
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
RUNTIME = [Path.home() / "Open_Duck_Mini_Runtime", REPO / "upstream" / "Open_Duck_Mini_Runtime"]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--background", action="store_true", help="set SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1")
    p.add_argument("--pad", choices=("auto", "xbox", "ps4"), default="auto")
    args = p.parse_args()
    if args.background:
        os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

    runtime = next((r for r in RUNTIME if (r / "mini_bdx_runtime").is_dir()), None)
    if runtime is None:
        sys.exit("Open_Duck_Mini_Runtime not found")
    sys.path.insert(0, str(runtime / "mini_bdx_runtime"))
    sys.path.insert(0, str(REPO / "robot"))
    import pads

    ctrl = pads.make_controller(20, args.pad)
    raw = ctrl.p1._js if hasattr(ctrl.p1, "_js") else ctrl.p1
    print(f"background events: {args.background}. Move the sticks and press buttons for {args.seconds:.0f} s.")
    print("walk sees: cmds [vx vy wz ...]  buttons            | raw axes")
    last, end = None, time.time() + args.seconds
    while time.time() < end:
        cmds, btns, _lt, _rt = ctrl.get_last_command()
        pressed = [n for n in ("A", "B", "X", "Y", "LB", "RB", "dpad_up", "dpad_down") if getattr(btns, n).is_pressed]
        axes = [round(raw.get_axis(i), 2) for i in range(raw.get_numaxes())]
        line = (tuple(round(float(c), 2) for c in cmds[:3]), tuple(pressed), tuple(axes))
        if line != last:
            print(f"  {list(line[0])}  {list(line[1])!s:18s} | {list(line[2])}")
            last = line
        time.sleep(0.05)
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Keyboard teleop over the Jero link. Works over SSH (plain terminal, no GUI).

    python brain/teleop.py jero.local

    w/s   forward / back          a/d   turn left / right      q/e  strafe left / right
    i/k   head pitch up / down    j/l   head yaw left / right
    space stop (zero velocity)    p     pause/resume (A)       x    E-STOP (pause)
    b     random sound (B)        f     projector (X)          ctrl-c quit
"""

import argparse
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jero_link" / "src"))

from jero_link import LIMITS, JeroClient, load_key

STEP = {"vx": 0.05, "vy": 0.05, "wz": 0.25, "head": 0.1}


def clamp(name, value):
    lo, hi = LIMITS[name]
    return max(lo, min(hi, value))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--key", default=os.path.expanduser("~/.config/jero/link.key"))
    args = p.parse_args()

    key = None if args.key.lower() == "none" else load_key(args.key)
    state = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "head_pitch": 0.0, "head_yaw": 0.0}

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    with JeroClient(args.host, args.port, key=key) as duck:
        try:
            tty.setcbreak(fd)
            print(__doc__)
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    if ch == "w":
                        state["vx"] = clamp("vx", state["vx"] + STEP["vx"])
                    elif ch == "s":
                        state["vx"] = clamp("vx", state["vx"] - STEP["vx"])
                    elif ch == "a":
                        state["wz"] = clamp("wz", state["wz"] + STEP["wz"])
                    elif ch == "d":
                        state["wz"] = clamp("wz", state["wz"] - STEP["wz"])
                    elif ch == "q":
                        state["vy"] = clamp("vy", state["vy"] + STEP["vy"])
                    elif ch == "e":
                        state["vy"] = clamp("vy", state["vy"] - STEP["vy"])
                    elif ch == "i":
                        state["head_pitch"] = clamp("head_pitch", state["head_pitch"] - STEP["head"])
                    elif ch == "k":
                        state["head_pitch"] = clamp("head_pitch", state["head_pitch"] + STEP["head"])
                    elif ch == "j":
                        state["head_yaw"] = clamp("head_yaw", state["head_yaw"] + STEP["head"])
                    elif ch == "l":
                        state["head_yaw"] = clamp("head_yaw", state["head_yaw"] - STEP["head"])
                    elif ch == " ":
                        state.update(vx=0.0, vy=0.0, wz=0.0)
                    elif ch == "p":
                        duck.press("A")
                    elif ch == "b":
                        duck.press("B")
                    elif ch == "f":
                        duck.press("X")
                    elif ch == "x":
                        state.update(vx=0.0, vy=0.0, wz=0.0)
                        duck.estop()
                    duck.drive(state["vx"], state["vy"], state["wz"])
                    duck.head(head_pitch=state["head_pitch"], head_yaw=state["head_yaw"])
                    sys.stdout.write(
                        "\rvx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f} head p={head_pitch:+.2f} y={head_yaw:+.2f}   ".format(
                            **state
                        )
                    )
                    sys.stdout.flush()
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            duck.stop()
            time.sleep(0.1)
    print("\nstopped")


if __name__ == "__main__":
    main()

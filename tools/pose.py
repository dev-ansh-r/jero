#!/usr/bin/env python3
"""Move all 14 servos smoothly to a pose and hold it, then release torque on Enter.

    python ~/Jero/tools/pose.py zero     # every joint at 0 rad (servo 2048): the horn-fitting pose
    python ~/Jero/tools/pose.py init     # upstream's standing start pose (init_pos)

Robot on a stand, feet off the ground. Moves over --seconds (default 3) from wherever the joints
are, at P gain --kp (default 16, half of the walk's 32), then holds. Enter releases torque, so
support the robot before pressing it. Ctrl+C also releases torque.

Raw servo angles: duck_config.json joint offsets are NOT applied (that's what you're checking).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Order and init pose: upstream mini_bdx_runtime/rustypot_position_hwi.py (HWI.joints / init_pos).
JOINTS = {
    "left_hip_yaw": (20, 0.002),
    "left_hip_roll": (21, 0.053),
    "left_hip_pitch": (22, -0.63),
    "left_knee": (23, 1.368),
    "left_ankle": (24, -0.784),
    "neck_pitch": (30, 0.0),
    "head_pitch": (31, 0.0),
    "head_yaw": (32, 0.0),
    "head_roll": (33, 0.0),
    "right_hip_yaw": (10, -0.003),
    "right_hip_roll": (11, -0.065),
    "right_hip_pitch": (12, 0.635),
    "right_knee": (13, 1.379),
    "right_ankle": (14, -0.796),
}


def retry(fn, *args, tries: int = 3):
    for i in range(tries):
        try:
            return fn(*args)
        except (KeyboardInterrupt, SystemExit):
            raise
        except BaseException:  # rustypot timeouts and panics: retry, then give up
            if i == tries - 1:
                raise
            time.sleep(0.05)
    return None


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("pose", choices=("zero", "init"))
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--kp", type=int, default=16)
    args = p.parse_args()

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
    import feetech_io  # same servo IO as the walk; no rustypot needed (e.g. on the RB3 Gen 2)

    io = feetech_io.open_bus(args.port)
    ids = [sid for sid, _ in JOINTS.values()]
    target = [0.0 if args.pose == "zero" else init for _, init in JOINTS.values()]

    start = retry(io.read_present_position, ids)
    print("from -> to (rad):")
    for (name, _), a, b in zip(JOINTS.items(), start, target):
        print(f"  {name:16s} {a:+.3f} -> {b:+.3f}")

    try:
        retry(io.set_kps, ids, [float(args.kp)] * len(ids))
        retry(io.write_goal_position, ids, list(start))  # hold where it is before enabling torque
        retry(io.enable_torque, ids)
        steps = max(1, int(args.seconds * 50))
        for k in range(1, steps + 1):
            f = k / steps
            retry(io.write_goal_position, ids, [a + (b - a) * f for a, b in zip(start, target)])
            time.sleep(0.02)
        time.sleep(0.5)
        now = retry(io.read_present_position, ids)
        print(f"\nholding '{args.pose}'. reached (rad), error = reached - target:")
        for (name, _), r, b in zip(JOINTS.items(), now, target):
            flag = "   <- off by more than 0.1 rad" if abs(r - b) > 0.1 else ""
            print(f"  {name:16s} {r:+.3f}  error {r - b:+.3f}{flag}")
        input("\nSupport the robot, then press Enter to release torque ")
    finally:
        retry(io.disable_torque, ids)
        print("torque released")
    return 0


if __name__ == "__main__":
    sys.exit(main())

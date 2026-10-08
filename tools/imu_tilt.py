#!/usr/bin/env python3
"""Measure the IMU tilt correction in the standing start pose and save it for the walk.

The walking policy learned what "standing level" looks like from the simulated IMU. If the real
IMU reads differently in the same pose (mounting tilt, a slightly different crouch), the policy
believes the robot is tipping and steps to catch itself: the robot drifts with no command.

Servos powered, robot on the floor. This tool:
  1. moves the robot into upstream's start pose (init_pos) over 3 s, like the walk does;
  2. you let it stand on the floor (steady it with a finger if needed, don't push);
  3. averages the IMU for 3 s and compares it with the sim reference below;
  4. saves the difference as accel_offset in ~/.config/jero/imu.json, then releases torque.

    python ~/Jero/tools/imu_tilt.py            # measure + save
    python ~/Jero/tools/imu_tilt.py --no-save  # measure only

Run tools/imu_check.py first (mounting + gyro bias); re-run this after any change to the IMU
mounting or the joint offsets.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "robot"))

# Sim IMU (robot frame, m/s^2) standing still in the home/init pose on flat ground, measured with
# upstream Open_Duck_Playground's scene_flat_terrain_backlash (STS3215 and STS3235 models give the
# same value). This is what the policy saw as "level" in training.
SIM_STANDING_ACCEL = np.array([0.134, -0.004, 9.809])
MAX_OFFSET = 2.5  # m/s^2 (~15 deg): more than this is a wrong pose or mounting, not a tilt to correct

# upstream rustypot_position_hwi.py HWI.joints / init_pos (same as tools/pose.py)
JOINTS = {
    "left_hip_yaw": (20, 0.002), "left_hip_roll": (21, 0.053), "left_hip_pitch": (22, -0.63),
    "left_knee": (23, 1.368), "left_ankle": (24, -0.784), "neck_pitch": (30, 0.0),
    "head_pitch": (31, 0.0), "head_yaw": (32, 0.0), "head_roll": (33, 0.0),
    "right_hip_yaw": (10, -0.003), "right_hip_roll": (11, -0.065), "right_hip_pitch": (12, 0.635),
    "right_knee": (13, 1.379), "right_ankle": (14, -0.796),
}  # fmt: skip


def retry(fn, *args, tries: int = 5):
    for i in range(tries):
        try:
            return fn(*args)
        except OSError:
            if i == tries - 1:
                raise
            time.sleep(0.05)
    return None


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--kp", type=int, default=32, help="servo P gain while standing (the walk uses 32)")
    p.add_argument("--no-save", action="store_true")
    args = p.parse_args()

    import feetech_io
    import imu_mpu6050 as m

    cfg = m.load_config()
    if not m.CONFIG_PATH.is_file():
        sys.exit(f"{m.CONFIG_PATH} missing: run tools/imu_check.py first")
    imu = m.Imu(50, config=dict(cfg, calibrate_seconds=0), start_thread=False)

    io = feetech_io.open_bus(args.port)
    ids = [sid for sid, _ in JOINTS.values()]
    target = [init for _, init in JOINTS.values()]
    start = retry(io.read_present_position, ids)
    try:
        retry(io.set_kps, ids, [float(args.kp)] * len(ids))
        retry(io.write_goal_position, ids, list(start))
        retry(io.enable_torque, ids)
        for k in range(1, 151):  # 3 s ramp
            f = k / 150
            retry(io.write_goal_position, ids, [a + (b - a) * f for a, b in zip(start, target)])
            time.sleep(0.02)

        input("\nRobot is in the start pose. Stand it on the floor, let go (steady it lightly if it\n"
              "wobbles, don't push), then press Enter to measure for 3 s ")  # fmt: skip
        samples = []
        t_end = time.time() + 3.0
        while time.time() < t_end:
            try:
                accel, _ = imu.read_robot()
                samples.append(accel)
            except OSError:
                pass
            time.sleep(0.01)
    finally:
        retry(io.disable_torque, ids)
        print("torque released (support the robot)")

    a = np.array(samples)
    real, spread = a.mean(axis=0), a.std(axis=0)
    offset = real - SIM_STANDING_ACCEL
    offset[2] = 0.0  # only correct the tilt (x/y); gravity's size isn't the policy's concern
    tilt = np.degrees(np.arctan2(offset[:2], 9.81))
    print(f"\nreal accel (robot frame): {np.round(real, 3)}   spread {np.round(spread, 3)}")
    print(f"sim reference:            {SIM_STANDING_ACCEL}")
    print(f"correction (subtracted):  {np.round(offset, 3)}  = about {tilt[0]:+.1f} deg pitch, {tilt[1]:+.1f} deg roll")
    if np.abs(offset).max() > MAX_OFFSET or spread.max() > 0.5:
        print("NOT saved: tilt too large or the robot moved while measuring. Check the pose/mounting, retry.")
        return 1
    if args.no_save:
        print("--no-save: nothing written")
        return 0
    m.save_config({"accel_offset": [round(float(x), 4) for x in offset]})
    print(f"saved accel_offset to {m.CONFIG_PATH}; jero_walk.py --imu mpu6050 applies it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

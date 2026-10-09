#!/usr/bin/env python3
"""Measure the IMU mounting (MPU6050 on I2C, or --backend iio: RB3 Gen 2 onboard), verify signs, save it.

Run on the Pi with the IMU fitted in the trunk and the robot in your hands (servos off):

    python ~/Jero/tools/imu_check.py

Steps (about 1 minute):
  1. hold upright and still        -> finds robot +z (gravity) and the gyro bias
  2. tip nose DOWN 30-45 deg, hold -> finds robot +x
  3. tip LEFT side down, hold      -> checks accel y and gyro x signs
  4. turn LEFT (counter-clockwise seen from above) ~90 deg -> checks gyro z sign
Writes axes + gyro bias to ~/.config/jero/imu.json. Exit code 0 only if every check passes.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
import imu_mpu6050 as m

RATE = 100.0


def record(imu: m.Imu, seconds: float, robot_frame: bool):
    acc, gyr = [], []
    for _ in range(int(seconds * RATE)):
        a, g = imu.dev.read_si()
        g = g - imu.gyro_bias
        if robot_frame:
            a, g = imu.to_robot(a), imu.to_robot(g)
        acc.append(a)
        gyr.append(g)
        time.sleep(1 / RATE)
    return np.array(acc), np.array(gyr)


def prompt(msg: str, settle: float = 0.0):
    input(f"\n>> {msg}\n   press Enter when ready ")
    if settle:
        time.sleep(settle)


def peak(series: np.ndarray) -> float:
    """Signed value of the largest excursion."""
    return float(series[np.argmax(np.abs(series))])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bus", type=int, default=1)
    p.add_argument("--address", type=lambda s: int(s, 0), default=0x68)
    p.add_argument("--backend", choices=("mpu6050", "iio"), default="mpu6050",
                   help="iio = RB3 Gen 2 onboard IMU (robot/imu_iio.py)")
    p.add_argument("--no-save", action="store_true")
    args = p.parse_args()

    cfg = dict(m.DEFAULTS, bus=args.bus, address=args.address, axes="x,y,z", gyro_bias=[0, 0, 0], calibrate_seconds=0,
               backend=args.backend)
    imu = m.Imu(50, config=cfg, start_thread=False)
    if args.backend == "iio":
        print(f"connected: {imu.dev.name}, {imu.dev.rates()[0]:g} / {imu.dev.rates()[1]:g} Hz")
        print("WARNING: this is the RealSense's IMU on the HEAD: the walk can't use it (head moves)")
    else:
        print(f"connected: WHO_AM_I 0x{imu.dev.whoami:02x} ({m.KNOWN_WHOAMI.get(imu.dev.whoami, 'unknown')})")

    results = []

    def check(name, ok, detail):
        results.append(ok)
        print(f"   [{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    # 1. upright + still: gravity axis and gyro bias
    prompt("1/4  Hold the robot UPRIGHT and perfectly STILL", settle=0.5)
    bias = imu.measure_gyro_bias(2.0)
    if bias is None:
        sys.exit("   moved during calibration: run again and keep it still")
    imu.gyro_bias = bias
    still, _ = record(imu, 1.0, robot_frame=False)
    still_mean = still.mean(axis=0)
    check(
        "gravity magnitude",
        8.8 < np.linalg.norm(still_mean) < 10.8,
        f"|a| = {np.linalg.norm(still_mean):.2f} m/s^2 (want ~9.8)",
    )

    # 2. nose down: forward axis
    prompt("2/4  Tip the NOSE DOWN about 30-45 deg and HOLD it there", settle=0.7)
    nose, _ = record(imu, 1.0, robot_frame=False)
    try:
        axes = m.derive_axes(still_mean, nose.mean(axis=0))
    except ValueError as exc:
        sys.exit(f"   {exc}")
    imu.idx, imu.sign = m.parse_axes(axes)
    print(f'   measured mounting: axes = "{axes}"')
    up = imu.to_robot(still_mean)
    check("upright reads +z", up[2] > 8.8 and abs(up[0]) < 1.5 and abs(up[1]) < 1.5, f"accel = {np.round(up, 2)}")
    nd = imu.to_robot(nose.mean(axis=0))
    check("nose down -> accel x negative", nd[0] < -3.0, f"accel x = {nd[0]:.2f}")

    # 3. left side down: accel y and gyro x
    prompt("3/4  Back UPRIGHT. After Enter, tip the LEFT side DOWN about 30-45 deg and HOLD")
    a3, g3 = record(imu, 3.0, robot_frame=True)
    check("left down -> accel y negative", a3[-50:, 1].mean() < -3.0, f"accel y = {a3[-50:, 1].mean():.2f}")
    check("left down -> gyro x negative", peak(g3[:, 0]) < -0.3, f"gyro x peak = {peak(g3[:, 0]):.2f} rad/s")

    # 4. yaw left: gyro z. (Gyro y needs no own step: gyro and accel share the chip's axes,
    #    so a right-handed accel mapping that passes 1-3 fixes all gyro signs too.)
    prompt("4/4  Back UPRIGHT. After Enter, TURN it LEFT (counter-clockwise from above) ~90 deg")
    _, g4 = record(imu, 3.0, robot_frame=True)
    check("turn left -> gyro z positive", peak(g4[:, 2]) > 0.3, f"gyro z peak = {peak(g4[:, 2]):.2f} rad/s")

    ok = all(results)
    print("\nRESULT:", "ALL PASS" if ok else "FAILED: fix mounting/wiring and run again (nothing saved)")
    if ok and not args.no_save:
        m.save_config(
            {
                "backend": args.backend,
                "bus": args.bus,
                "address": args.address,
                "axes": axes,
                "gyro_bias": [round(float(b), 6) for b in bias],
                "accel_offset": [0.0, 0.0, 0.0],  # a new mounting invalidates the old tilt correction
            }
        )
        print(f"saved to {m.CONFIG_PATH}. jero_walk.py --imu mpu6050 will use it.")
        print("next: tools/imu_tilt.py (measures the tilt correction in the standing pose)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

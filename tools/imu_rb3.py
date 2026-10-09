#!/usr/bin/env python3
"""Test the RB3 Gen 2's onboard IMU (TDK ICM-42688 on the sensor hub), then calibrate it for the walk.

Nothing moves; servos can be off. Board mounted in the robot for --calibrate.

    python ~/Jero/tools/imu_rb3.py --install     # once: Qualcomm's sensor tool + its permission (sudo)
    python ~/Jero/tools/imu_rb3.py               # test: rate, gravity, gyro noise (keep it still)
    python ~/Jero/tools/imu_rb3.py --calibrate   # 4 guided poses -> ~/.config/jero/imu.json ("backend": "qsh")
    python ~/Jero/tools/imu_rb3.py --live        # watch the values while you tilt the robot

The test checks (held still):
  rate      >= 50 fresh samples/s for both accel and gyro (the walk runs at 50 Hz)
  gravity   |accel| is 9.8 m/s^2 +- 0.8
  gyro      bias < 0.05 rad/s, noise < 0.01 rad/s
After --calibrate run tools/imu_tilt.py on the robot (standing tilt correction), as for the MPU; then
robot/jero.sh uses the onboard IMU. (Not the IIO accel_3d/gyro_3d devices: those are the RealSense's.)
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot"))
import imu_qsh

TOOL_PATH = "/usr/bin/see_workhorse"


def sh(*cmd: str) -> bool:
    print("  $", " ".join(cmd))
    return subprocess.run(cmd, check=False).returncode == 0


def install() -> int:
    ok = sh("sudo", "apt-get", "install", "-y", "-qq", "qcom-sensors-test-apps")
    ok &= sh("sudo", "setcap", "cap_wake_alarm+ep", TOOL_PATH)  # its wake-alarm timer, without sudo
    ok &= sh("systemctl", "is-active", "--quiet", "sscrpcd")
    print("installed: run without --install to test" if ok else "install FAILED (see above)")
    return 0 if ok else 1


def test(dev: imu_qsh.QshImu, seconds: float) -> int:
    print(f"device: {dev.name}, {dev.rate_hz:g} Hz requested")
    print(f"hold the robot/board STILL for {seconds:g} s ...")
    a0, g0 = dev.accel.parser.count, dev.gyro.parser.count
    acc, gyr = [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        a, g = dev.read_si()
        acc.append(a)
        gyr.append(g)
        time.sleep(0.01)
    dt = time.monotonic() - t0
    ra, rg = (dev.accel.parser.count - a0) / dt, (dev.gyro.parser.count - g0) / dt
    acc, gyr = np.array(acc), np.array(gyr)
    g = np.linalg.norm(acc.mean(axis=0))
    bias, noise = np.abs(gyr.mean(axis=0)).max(), gyr.std(axis=0).max()
    axis = "xyz"[int(np.argmax(np.abs(acc.mean(axis=0))))]
    print(
        f"  accel mean {np.round(acc.mean(axis=0), 2)} m/s^2 (gravity on sensor {axis}), std {np.round(acc.std(axis=0), 3)}"
    )
    print(f"  gyro  mean {np.round(gyr.mean(axis=0), 4)} rad/s, std {np.round(gyr.std(axis=0), 4)}")
    checks = [
        ("rate", min(ra, rg) >= 50, f"accel {ra:.0f}/s, gyro {rg:.0f}/s (want >= 50)"),
        ("gravity", 9.0 < g < 10.6, f"|a| = {g:.2f} m/s^2 (want 9.8 +- 0.8)"),
        ("gyro", bias < 0.05 and noise < 0.01, f"bias {bias:.4f} rad/s (< 0.05), noise {noise:.4f} (< 0.01)"),
    ]
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    ok = all(c[1] for c in checks)
    print("RESULT:", "ALL PASS. next: --calibrate" if ok else "FAILED (moved? see above)")
    return 0 if ok else 1


def live(dev: imu_qsh.QshImu) -> int:
    import imu_mpu6050 as m

    cfg = m.load_config()
    robot = cfg.get("backend") == "qsh"
    idx, sign = m.parse_axes(cfg["axes"]) if robot else (np.arange(3), np.ones(3))
    bias = np.asarray(cfg.get("gyro_bias") or [0, 0, 0], float) if robot else np.zeros(3)
    print("robot frame (from imu.json): +x forward, +y left, +z up" if robot else "SENSOR frame (not calibrated yet)")
    print("upright and still: accel ~ (0, 0, +9.8). nose down: accel x < 0. left down: accel y < 0. Ctrl+C stops")
    try:
        while True:
            a, g = dev.read_si()
            a, g = sign * a[idx], sign * (g - bias)[idx]
            print(
                f"\r  accel x {a[0]:+6.2f} y {a[1]:+6.2f} z {a[2]:+6.2f} m/s^2 | "
                f"gyro x {g[0]:+5.2f} y {g[1]:+5.2f} z {g[2]:+5.2f} rad/s   ",
                end="",
                flush=True,
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--install", action="store_true", help="install Qualcomm's sensor tool + permission (sudo)")
    p.add_argument("--live", action="store_true", help="print values continuously")
    p.add_argument("--calibrate", action="store_true", help="run tools/imu_check.py --backend qsh")
    p.add_argument("--seconds", type=float, default=3.0)
    args = p.parse_args()

    if args.install:
        return install()
    if args.calibrate:
        return subprocess.call([sys.executable, str(ROOT / "tools" / "imu_check.py"), "--backend", "qsh"])
    try:
        dev = imu_qsh.QshImu()
    except (FileNotFoundError, OSError) as exc:
        sys.exit(f"{exc}\n(first time? python {Path(__file__).name} --install)")
    try:
        return live(dev) if args.live else test(dev, args.seconds)
    finally:
        dev.close()


if __name__ == "__main__":
    sys.exit(main())

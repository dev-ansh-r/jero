#!/usr/bin/env python3
"""Test the RB3 Gen 2's onboard IMU, then calibrate it for the walk. Nothing moves; servos can be off.

    python ~/Jero/tools/imu_rb3.py --install-udev   # once: 100 Hz at every boot (asks for sudo)
    python ~/Jero/tools/imu_rb3.py                  # test: rate, gravity, gyro noise (board still)
    python ~/Jero/tools/imu_rb3.py --live           # watch the values while you tilt the robot
    python ~/Jero/tools/imu_rb3.py --calibrate      # mounting + gyro bias -> ~/.config/jero/imu.json

The test checks (robot or board held still):
  rate      the sensor hub delivers >= 50 fresh samples/s (the walk runs at 50 Hz)
  read      one accel+gyro read takes < 15 ms (it runs in the IMU thread, not the policy loop)
  gravity   |accel| is 9.8 m/s^2 +- 0.8 (scale and units right)
  gyro      still: bias < 0.05 rad/s and noise < 0.01 rad/s
--calibrate is tools/imu_check.py --backend iio (4 guided poses), then run tools/imu_tilt.py on the
robot as for the MPU. After that robot/jero.sh uses the onboard IMU (imu.json has "backend": "iio").
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
import imu_iio

UDEV_RULE = "/etc/udev/rules.d/99-jero-imu-rate.rules"
RULES = """# Jero: RB3 Gen 2 onboard IMU at 100 Hz (default 10 Hz is too slow for the 50 Hz walk)
ACTION=="add", SUBSYSTEM=="iio", ATTR{name}=="accel_3d", ATTR{in_accel_sampling_frequency}="100"
ACTION=="add", SUBSYSTEM=="iio", ATTR{name}=="gyro_3d", ATTR{in_anglvel_sampling_frequency}="100"
"""


def install_udev() -> int:
    print(f"installing {UDEV_RULE} (sudo)")
    ok = subprocess.run(["sudo", "tee", UDEV_RULE], input=RULES.encode(), stdout=subprocess.DEVNULL, check=False).returncode == 0
    for name, kind in (("accel_3d", "accel"), ("gyro_3d", "anglvel")):  # and now, without a reboot
        f = imu_iio.find_device(name) / f"in_{kind}_sampling_frequency"
        ok &= subprocess.run(["sudo", "tee", str(f)], input=b"100\n", stdout=subprocess.DEVNULL, check=False).returncode == 0
        print(f"  {name}: {f.read_text().strip()} Hz")
    return 0 if ok else 1


def test(dev: imu_iio.IioImu, seconds: float) -> int:
    print(f"device: {dev.name}; set rate accel {dev.rates()[0]:g} Hz, gyro {dev.rates()[1]:g} Hz")
    print(f"hold the board/robot STILL for {seconds:g} s ...")
    acc, gyr, t_read, stamps = [], [], [], []
    t_end = time.perf_counter() + seconds
    while time.perf_counter() < t_end:
        t0 = time.perf_counter()
        a, g = dev.read_si()
        t_read.append(time.perf_counter() - t0)
        acc.append(a)
        gyr.append(g)
        stamps.append(t0)
        time.sleep(0.004)
    acc, gyr = np.array(acc), np.array(gyr)
    fresh = 1 + sum(1 for k in range(1, len(acc)) if not np.array_equal(acc[k], acc[k - 1]))
    rate = fresh / (stamps[-1] - stamps[0])
    g = np.linalg.norm(acc.mean(axis=0))
    bias, noise = np.abs(gyr.mean(axis=0)).max(), gyr.std(axis=0).max()
    axis = "xyz"[int(np.argmax(np.abs(acc.mean(axis=0))))]
    print(
        f"  accel mean {np.round(acc.mean(axis=0), 2)} m/s^2 (gravity on sensor {axis}), std {np.round(acc.std(axis=0), 3)}"
    )
    print(f"  gyro  mean {np.round(gyr.mean(axis=0), 4)} rad/s, std {np.round(gyr.std(axis=0), 4)}")
    checks = [
        ("rate", rate >= 50, f"{rate:.0f} fresh samples/s (want >= 50)"),
        (
            "read",
            np.percentile(t_read, 95) < 0.015,
            f"{np.mean(t_read) * 1000:.1f} ms mean, {np.percentile(t_read, 95) * 1000:.1f} ms p95 (want < 15)",
        ),
        ("gravity", 9.0 < g < 10.6, f"|a| = {g:.2f} m/s^2 (want 9.8 +- 0.8)"),
        ("gyro", bias < 0.05 and noise < 0.01, f"bias {bias:.4f} rad/s (< 0.05), noise {noise:.4f} (< 0.01)"),
    ]
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    ok = all(c[1] for c in checks)
    if dev.rates()[0] < 50:
        print("  hint: rate is set below 50 Hz: run with --install-udev once")
    print("RESULT:", "ALL PASS. next: --calibrate" if ok else "FAILED (moved? rate too low? see above)")
    return 0 if ok else 1


def live(dev: imu_iio.IioImu) -> int:
    sys.path.insert(0, str(ROOT / "robot"))
    import imu_mpu6050 as m

    cfg = m.load_config()
    robot = cfg.get("backend") == "iio"
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
    p.add_argument("--install-udev", action="store_true", help="100 Hz at every boot (sudo), then exit")
    p.add_argument("--live", action="store_true", help="print values continuously")
    p.add_argument("--calibrate", action="store_true", help="run tools/imu_check.py --backend iio")
    p.add_argument("--seconds", type=float, default=3.0)
    args = p.parse_args()

    if args.install_udev:
        return install_udev()
    if args.calibrate:
        return subprocess.call([sys.executable, str(ROOT / "tools" / "imu_check.py"), "--backend", "iio"])
    try:
        dev = imu_iio.IioImu()
    except FileNotFoundError as exc:
        sys.exit(str(exc))
    return live(dev) if args.live else test(dev, args.seconds)


if __name__ == "__main__":
    sys.exit(main())

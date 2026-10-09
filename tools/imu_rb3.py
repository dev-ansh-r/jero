#!/usr/bin/env python3
"""Test the RB3 Gen 2's onboard IMU, then calibrate it for the walk. Nothing moves; servos can be off.

    python ~/Jero/tools/imu_rb3.py --install        # once: stream the IMU at every boot (sudo)
    python ~/Jero/tools/imu_rb3.py                  # test: rate, gravity, gyro noise (board still)
    python ~/Jero/tools/imu_rb3.py --live           # watch the values while you tilt the robot
    python ~/Jero/tools/imu_rb3.py --calibrate      # mounting + gyro bias -> ~/.config/jero/imu.json

The test checks (robot or board held still):
  rate      the sensor hub delivers >= 50 fresh samples/s (the walk runs at 50 Hz)
  read      one accel+gyro read takes < 2 ms (buffer mode; sysfs mode is ~14 ms: run --install)
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

UNIT = ROOT / "robot" / "systemd" / "jero-imu.service"
SETUP = ROOT / "robot" / "jero-imu-setup.sh"
OLD_RULE = "/etc/udev/rules.d/99-jero-imu-rate.rules"  # earlier sysfs-only setup, replaced by the service


def sh(*cmd: str) -> bool:
    print("  $", " ".join(cmd))
    return subprocess.run(cmd, check=False).returncode == 0


def install() -> int:
    """Stream the IMU at every boot: jero-imu.service runs robot/jero-imu-setup.sh (sudo)."""
    ok = sh("sudo", "install", "-m", "755", str(SETUP), "/usr/local/bin/jero-imu-setup")
    ok &= sh("sudo", "install", "-m", "644", str(UNIT), "/etc/systemd/system/jero-imu.service")
    sh("sudo", "rm", "-f", OLD_RULE)
    sh("sudo", "systemctl", "mask", "--now", "iio-sensor-proxy")  # screen rotation: not on a robot
    ok &= sh("sudo", "systemctl", "daemon-reload")
    ok &= sh("sudo", "systemctl", "enable", "--now", "jero-imu.service")
    ok &= sh("sudo", "systemctl", "restart", "jero-imu.service")
    subprocess.run(["journalctl", "-u", "jero-imu", "-n", "2", "--no-pager", "-o", "cat"], check=False)
    print(
        "installed: the onboard IMU streams at every boot (undo: sudo systemctl disable jero-imu;"
        " sudo systemctl unmask iio-sensor-proxy)"
        if ok
        else "install FAILED (see above)"
    )
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
            f"{np.mean(t_read) * 1000:.1f} ms mean, {np.percentile(t_read, 95) * 1000:.1f} ms p95 (want < 2)",
        ),
        ("gravity", 9.0 < g < 10.6, f"|a| = {g:.2f} m/s^2 (want 9.8 +- 0.8)"),
        ("gyro", bias < 0.05 and noise < 0.01, f"bias {bias:.4f} rad/s (< 0.05), noise {noise:.4f} (< 0.01)"),
    ]
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    ok = all(c[1] for c in checks)
    if "sysfs" in dev.name:
        print("  hint: IMU buffers not streaming: run with --install once")
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
    p.add_argument("--install", action="store_true", help="stream the IMU at every boot (sudo), then exit")
    p.add_argument("--live", action="store_true", help="print values continuously")
    p.add_argument("--calibrate", action="store_true", help="run tools/imu_check.py --backend iio")
    p.add_argument("--seconds", type=float, default=3.0)
    args = p.parse_args()

    if args.install:
        return install()
    if args.calibrate:
        return subprocess.call([sys.executable, str(ROOT / "tools" / "imu_check.py"), "--backend", "iio"])
    try:
        dev = imu_iio.IioImu()
    except FileNotFoundError as exc:
        sys.exit(str(exc))
    return live(dev) if args.live else test(dev, args.seconds)


if __name__ == "__main__":
    sys.exit(main())

"""RB3 Gen 2 onboard IMU through Linux IIO (sensor hub: accel_3d + gyro_3d), MPU6050-driver compatible.

The board's accelerometer and gyro appear as /sys/bus/iio/devices/iio:deviceN with names accel_3d and
gyro_3d (HID sensor hub). This class has the same ``read_si()`` / ``init()`` / ``whoami`` as
``imu_mpu6050.Mpu6050``, so ``imu_mpu6050.Imu`` uses it when ~/.config/jero/imu.json says
``"backend": "iio"`` (tools/imu_rb3.py writes that): mounting axes, gyro bias, tilt correction,
tools/imu_check.py, tools/imu_tilt.py and the walk all work unchanged.

Conventions: HID sensors report gravity (board flat, face up: z = -1 g); the walk wants specific force
(+1 g up, like the MPU6050), so accel is negated (``accel_sign``). Gyro is rad/s, right-handed.
Sampling frequency defaults to 10 Hz; writing it needs root (tools/imu_rb3.py --install-udev sets 100 Hz
at every boot).
"""

from __future__ import annotations

import logging
import os
from pathlib import Path

import numpy as np

log = logging.getLogger("jero.imu.iio")
IIO_ROOT = Path("/sys/bus/iio/devices")
RATE_HZ = 100
DEVICE_GLOB = "iio:device*"  # tests use another pattern (":" is not allowed in Windows file names)


def _read0(fd: int) -> bytes:
    """Re-read a sysfs attribute from the start (a fresh sample each time)."""
    if hasattr(os, "pread"):
        return os.pread(fd, 32, 0)
    os.lseek(fd, 0, os.SEEK_SET)  # Windows (tests only)
    return os.read(fd, 32)


def find_device(name: str, root: Path = IIO_ROOT) -> Path:
    for d in sorted(Path(root).glob(DEVICE_GLOB)):
        try:
            if (d / "name").read_text().strip() == name:
                return d
        except OSError:
            continue
    raise FileNotFoundError(f"no IIO device named {name!r} under {root} (is this an RB3 Gen 2?)")


class _Triple:
    """x/y/z raw channels of one IIO device, kept open; one read = three preads at offset 0."""

    def __init__(self, dev: Path, kind: str):
        self.dev, self.kind = dev, kind
        self.scale = float((dev / f"in_{kind}_scale").read_text())
        off = dev / f"in_{kind}_offset"
        self.offset = float(off.read_text()) if off.exists() else 0.0
        self.fds = [os.open(dev / f"in_{kind}_{ax}_raw", os.O_RDONLY) for ax in "xyz"]
        self.freq_file = dev / f"in_{kind}_sampling_frequency"

    def read(self) -> np.ndarray:
        return (np.array([int(_read0(fd)) for fd in self.fds], float) + self.offset) * self.scale

    def rate(self) -> float:
        try:
            return float(self.freq_file.read_text())
        except (OSError, ValueError):
            return 0.0

    def set_rate(self, hz: float) -> bool:
        if self.rate() >= hz:
            return True
        try:
            self.freq_file.write_text(f"{hz:g}\n")
            return True
        except OSError:
            log.warning(
                "%s: can't set %g Hz (now %g Hz): run tools/imu_rb3.py --install-udev once",
                self.dev.name,
                hz,
                self.rate(),
            )
            return False

    def close(self):
        for fd in self.fds:
            os.close(fd)


class IioImu:
    """Onboard accel + gyro in SI units (m/s^2 specific force, rad/s), sensor frame."""

    def __init__(self, root: Path = IIO_ROOT, rate_hz: float = RATE_HZ, accel_sign: float = -1.0):
        self.root, self.rate_hz, self.accel_sign = Path(root), rate_hz, accel_sign
        self.whoami = None  # imu_check prints it for the MPU; IIO has no chip ID register
        self.name = "RB3 onboard IMU (IIO accel_3d + gyro_3d)"
        self.accel = _Triple(find_device("accel_3d", self.root), "accel")
        self.gyro = _Triple(find_device("gyro_3d", self.root), "anglvel")
        self.init()

    def init(self) -> None:
        for t in (self.accel, self.gyro):
            t.set_rate(self.rate_hz)

    def rates(self) -> tuple[float, float]:
        return self.accel.rate(), self.gyro.rate()

    def read_si(self) -> tuple[np.ndarray, np.ndarray]:
        return self.accel_sign * self.accel.read(), self.gyro.read()

    def close(self) -> None:
        self.accel.close()
        self.gyro.close()

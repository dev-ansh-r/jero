"""RB3 Gen 2 onboard IMU through Linux IIO (sensor hub: accel_3d + gyro_3d), MPU6050-driver compatible.

The board's accelerometer and gyro appear as /sys/bus/iio/devices/iio:deviceN with names accel_3d and
gyro_3d (HID sensor hub). This class has the same ``read_si()`` / ``init()`` / ``whoami`` as
``imu_mpu6050.Mpu6050``, so ``imu_mpu6050.Imu`` uses it when ~/.config/jero/imu.json says
``"backend": "iio"`` (tools/imu_check.py --backend iio writes that): mounting axes, gyro bias, tilt
correction, tools/imu_tilt.py and the walk all work unchanged.

Two ways to read, picked automatically:
  buffer  /dev/iio:deviceN streams every sample (~450/s from the hub); a read drains it and keeps the
          newest: ~0.05 ms. Needs the buffers enabled as root at boot: robot/jero-imu-setup.sh via
          jero-imu.service (tools/imu_rb3.py --install), which also stops iio-sensor-proxy (screen
          rotation) so nothing else takes samples from the same buffer.
  sysfs   in_*_raw files: each value is a request to the sensor hub, ~14 ms for accel + gyro. Fallback.

Conventions: HID sensors report gravity (board flat, face up: z = -1 g); the walk wants specific force
(+1 g up, like the MPU6050), so accel is negated (``accel_sign``). Gyro is rad/s, right-handed.
"""

from __future__ import annotations

import logging
import os
import struct
import time
from pathlib import Path

import numpy as np

log = logging.getLogger("jero.imu.iio")
IIO_ROOT = Path("/sys/bus/iio/devices")
DEV_DIR = Path("/dev")
RATE_HZ = 100
STALE_S = 0.1  # buffer silent this long -> read sysfs instead
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


def scan_layout(dev: Path) -> tuple[int, dict]:
    """(bytes per scan, {channel: (offset, struct format, shift, mask)}) of the enabled scan elements."""
    chans = []
    for en in sorted((dev / "scan_elements").glob("*_en")):
        if en.read_text().strip() != "1":
            continue
        name = en.name[:-3]
        index = int((dev / "scan_elements" / f"{name}_index").read_text())
        spec = (dev / "scan_elements" / f"{name}_type").read_text().strip()  # e.g. le:s32/32>>0
        endian, rest = spec.split(":")
        sign, rest = rest[0], rest[1:]
        bits, rest = rest.split("/")
        storage, shift = rest.split(">>")
        chans.append((index, name, endian, sign, int(bits), int(storage), int(shift)))
    layout, off, align = {}, 0, 1
    for _, name, endian, sign, bits, storage, shift in sorted(chans):
        size = storage // 8
        off = (off + size - 1) // size * size  # each element aligned to its own size
        fmt = ("<" if endian == "le" else ">") + {1: "b", 2: "h", 4: "i", 8: "q"}[size]
        if sign == "u":
            fmt = fmt.upper()
        layout[name] = (off, fmt, shift, (1 << bits) - 1, sign == "s", bits)
        off += size
        align = max(align, size)
    return (off + align - 1) // align * align, layout


class _Triple:
    """x/y/z of one IIO device: streamed from its buffer when enabled, else read from sysfs."""

    def __init__(self, dev: Path, kind: str, dev_dir: Path = DEV_DIR, clock=time.monotonic):
        self.dev, self.kind, self.clock = dev, kind, clock
        self.scale = float((dev / f"in_{kind}_scale").read_text())
        off = dev / f"in_{kind}_offset"
        self.offset = float(off.read_text()) if off.exists() else 0.0
        self.fds = [os.open(dev / f"in_{kind}_{ax}_raw", os.O_RDONLY) for ax in "xyz"]
        self.freq_file = dev / f"in_{kind}_sampling_frequency"
        self.buf_fd, self.last, self.last_t, self.mode = None, None, 0.0, "sysfs"
        self._open_buffer(dev_dir / dev.name)

    def _open_buffer(self, node: Path) -> None:
        try:
            if (self.dev / "buffer" / "enable").read_text().strip() != "1":
                return
            self.size, layout = scan_layout(self.dev)
            self.chans = [layout[f"in_{self.kind}_{ax}"] for ax in "xyz"]
            self.buf_fd = os.open(node, os.O_RDONLY | getattr(os, "O_NONBLOCK", 0))
            self.mode = "buffer"
        except (OSError, KeyError, ValueError) as exc:
            log.warning("%s: buffer not usable (%s): reading sysfs (~7 ms per read)", self.dev.name, exc)
            self.buf_fd = None

    def _drain(self) -> np.ndarray | None:
        """Newest complete scan in the buffer (raw counts), or None if nothing new."""
        data = b""
        while True:
            try:
                chunk = os.read(self.buf_fd, self.size * 256)
            except BlockingIOError:
                break
            if not chunk:
                break
            data += chunk
        n = len(data) // self.size
        if n == 0:
            return None
        rec = data[(n - 1) * self.size : n * self.size]
        out = []
        for off, fmt, shift, mask, signed, bits in self.chans:
            v = (struct.unpack_from(fmt, rec, off)[0] >> shift) & mask
            if signed and v >= 1 << (bits - 1):
                v -= 1 << bits
            out.append(v)
        return np.array(out, float)

    def read(self) -> np.ndarray:
        if self.buf_fd is not None:
            raw = self._drain()
            now = self.clock()
            if raw is not None:
                self.last, self.last_t = raw, now
            if self.last is not None and now - self.last_t < STALE_S:
                return (self.last + self.offset) * self.scale
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
                "%s: can't set %g Hz (now %g Hz): run tools/imu_rb3.py --install once", self.dev.name, hz, self.rate()
            )
            return False

    def close(self):
        for fd in self.fds + ([self.buf_fd] if self.buf_fd is not None else []):
            os.close(fd)


class IioImu:
    """Onboard accel + gyro in SI units (m/s^2 specific force, rad/s), sensor frame."""

    def __init__(
        self, root: Path = IIO_ROOT, rate_hz: float = RATE_HZ, accel_sign: float = -1.0, dev_dir: Path = DEV_DIR
    ):
        self.root, self.rate_hz, self.accel_sign = Path(root), rate_hz, accel_sign
        self.whoami = None  # imu_check prints it for the MPU; IIO has no chip ID register
        self.accel = _Triple(find_device("accel_3d", self.root), "accel", Path(dev_dir))
        self.gyro = _Triple(find_device("gyro_3d", self.root), "anglvel", Path(dev_dir))
        self.name = f"RB3 onboard IMU (IIO accel_3d + gyro_3d, {self.accel.mode}/{self.gyro.mode})"
        self.init()

    def init(self) -> None:
        for t in (self.accel, self.gyro):
            if t.mode == "sysfs":
                t.set_rate(self.rate_hz)

    def rates(self) -> tuple[float, float]:
        return self.accel.rate(), self.gyro.rate()

    def read_si(self) -> tuple[np.ndarray, np.ndarray]:
        return self.accel_sign * self.accel.read(), self.gyro.read()

    def close(self) -> None:
        self.accel.close()
        self.gyro.close()

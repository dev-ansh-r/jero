"""MPU6050 (GY-521) drop-in for upstream ``mini_bdx_runtime.raw_imu.Imu``.

The walking policy only consumes raw gyro [rad/s] and accelerometer [m/s^2, gravity
included] in the robot frame (x forward, y left, z up: the ``imu`` site in the sim trunk).
This module produces exactly that from an MPU6050 / MPU6500-family chip, with the same
``Imu(sampling_freq, user_pitch_bias, calibrate, upside_down).get_data()`` interface.

Settings chosen to sit close to the BNO055 fusion-mode defaults the policy saw:
gyro +/-2000 deg/s, accel +/-4 g, on-chip low-pass ~42-44 Hz, 200 Hz internal rate.

Mounting is handled by ``axes``: one entry per robot axis saying which sensor axis (and
sign) it comes from, e.g. ``"x,y,z"`` (chip flat, X arrow forward) or ``"-y,x,z"``.
Don't guess it: run ``tools/imu_check.py``, which measures it and saves it to
``~/.config/jero/imu.json`` together with the gyro bias.

``jero_walk.py --imu mpu6050`` installs this module in place of upstream ``raw_imu``.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from pathlib import Path
from queue import Empty, Full, Queue

import numpy as np

log = logging.getLogger("jero.imu")

CONFIG_PATH = Path.home() / ".config" / "jero" / "imu.json"

# registers (same addresses on MPU6050 and MPU6500/9250/9255)
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_CONFIG2 = 0x1D  # MPU6500 family only (reserved on MPU6050)
ACCEL_XOUT_H = 0x3B
PWR_MGMT_1 = 0x6B
WHO_AM_I = 0x75

WHOAMI_MPU6050 = 0x68
KNOWN_WHOAMI = {
    0x68: "MPU6050",
    0x70: "MPU6500",
    0x71: "MPU9250",
    0x73: "MPU9255",
    0x72: "MPU6050 clone",
    0x98: "MPU6050 clone",
}

GYRO_LSB_PER_DPS = 16.4  # FS_SEL=3, +/-2000 deg/s
ACCEL_LSB_PER_G = 8192.0  # AFS_SEL=1, +/-4 g
G = 9.80665

# Defaults that jero_walk.py / imu_check.py may override before Imu() is constructed.
DEFAULTS = {"bus": 1, "address": 0x68, "axes": "x,y,z", "gyro_bias": None, "calibrate_seconds": 2.0}
# Policy-frame accel correction (robot frame, m/s^2), subtracted from every reading. Measured by
# tools/imu_tilt.py: real reading in the standing start pose minus what the sim's IMU reads there.
DEFAULTS["accel_offset"] = [0.0, 0.0, 0.0]


def load_config(path: Path = CONFIG_PATH) -> dict:
    """Merged config: DEFAULTS <- ~/.config/jero/imu.json (if present)."""
    cfg = dict(DEFAULTS)
    try:
        cfg.update(json.loads(Path(path).read_text()))
    except FileNotFoundError:
        pass
    return cfg


def save_config(update: dict, path: Path = CONFIG_PATH) -> None:
    path = Path(path)
    cfg = {}
    if path.exists():
        cfg = json.loads(path.read_text())
    cfg.update(update)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(cfg, indent=2) + "\n")


def parse_axes(spec: str) -> tuple[np.ndarray, np.ndarray]:
    """``"-y,x,z"`` -> (index array, sign array) so robot = sign * sensor[index]."""
    parts = [p.strip().lower() for p in spec.split(",")]
    if len(parts) != 3:
        raise ValueError(f"axes must have 3 entries, got {spec!r}")
    idx, sign = [], []
    for p in parts:
        s = -1.0 if p.startswith("-") else 1.0
        name = p.lstrip("+-")
        if name not in ("x", "y", "z"):
            raise ValueError(f"bad axis {p!r} in {spec!r}")
        idx.append("xyz".index(name))
        sign.append(s)
    if sorted(idx) != [0, 1, 2]:
        raise ValueError(f"axes must use each of x, y, z once: {spec!r}")
    m = np.zeros((3, 3))
    for r, (i, s) in enumerate(zip(idx, sign)):
        m[r, i] = s
    if round(np.linalg.det(m)) != 1:
        raise ValueError(f"axes {spec!r} is a mirror image (left-handed); one sign is wrong")
    return np.array(idx), np.array(sign)


def axes_to_string(idx, sign) -> str:
    return ",".join(("-" if s < 0 else "") + "xyz"[i] for i, s in zip(idx, sign))


def derive_axes(still_accel, nose_down_accel) -> str:
    """Mounting from two sensor-frame accelerometer means.

    still: robot upright -> specific force points robot +z.
    nose_down: robot pitched forward -> gravity reaction gains a robot -x component.
    """
    up = np.asarray(still_accel, float)
    tilt = np.asarray(nose_down_accel, float)
    zi = int(np.argmax(np.abs(up)))
    z = np.zeros(3)
    z[zi] = math.copysign(1.0, up[zi])
    horiz = tilt - np.dot(tilt, z) * z  # part of the tilt not along up
    horiz[zi] = 0.0
    if np.linalg.norm(horiz) < 0.25 * G:
        raise ValueError("nose-down tilt too small to read (tip it 30-45 degrees)")
    xi = int(np.argmax(np.abs(horiz)))
    x = np.zeros(3)
    x[xi] = -math.copysign(1.0, horiz[xi])  # robot +x is opposite to where gravity reaction moved
    y = np.cross(z, x)
    rows = [x, y, z]
    idx = [int(np.argmax(np.abs(r))) for r in rows]
    sign = [float(np.sign(r[i])) for r, i in zip(rows, idx)]
    return axes_to_string(idx, sign)


def _int16(hi: int, lo: int) -> int:
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


class Mpu6050:
    """Register-level driver. ``bus`` is anything with smbus2's read/write_byte_data + read_i2c_block_data."""

    def __init__(self, bus=None, address: int = 0x68, bus_number: int = 1):
        if bus is None:
            from smbus2 import SMBus  # imported lazily so tests run without it

            bus = SMBus(bus_number)
        self.bus = bus
        self.address = address
        self.whoami = None
        self.init()

    def init(self) -> None:
        a = self.address
        self.bus.write_byte_data(a, PWR_MGMT_1, 0x80)  # reset
        time.sleep(0.1)
        self.bus.write_byte_data(a, PWR_MGMT_1, 0x01)  # wake, PLL on gyro X
        time.sleep(0.05)
        self.whoami = self.bus.read_byte_data(a, WHO_AM_I)
        name = KNOWN_WHOAMI.get(self.whoami)
        if name is None:
            log.warning("unknown WHO_AM_I 0x%02x at 0x%02x: continuing with MPU6050 register map", self.whoami, a)
        else:
            log.info("IMU %s (WHO_AM_I 0x%02x) at 0x%02x", name, self.whoami, a)
        self.bus.write_byte_data(a, CONFIG, 0x03)  # DLPF: ~42 Hz gyro (/44 Hz accel on MPU6050), 1 kHz internal
        self.bus.write_byte_data(a, SMPLRT_DIV, 4)  # 1 kHz / 5 = 200 Hz
        self.bus.write_byte_data(a, GYRO_CONFIG, 0x18)  # +/-2000 deg/s
        self.bus.write_byte_data(a, ACCEL_CONFIG, 0x08)  # +/-4 g
        if self.whoami != WHOAMI_MPU6050:
            self.bus.write_byte_data(a, ACCEL_CONFIG2, 0x03)  # MPU6500 family accel DLPF ~44.8 Hz
        time.sleep(0.05)

    def read_si(self) -> tuple[np.ndarray, np.ndarray]:
        """(accel m/s^2, gyro rad/s) in the SENSOR frame."""
        d = self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 14)
        ax, ay, az = (_int16(d[i], d[i + 1]) for i in (0, 2, 4))
        gx, gy, gz = (_int16(d[i], d[i + 1]) for i in (8, 10, 12))
        accel = np.array([ax, ay, az], float) * (G / ACCEL_LSB_PER_G)
        gyro = np.array([gx, gy, gz], float) * (math.pi / 180.0 / GYRO_LSB_PER_DPS)
        return accel, gyro


class Imu:
    """Same interface as upstream ``raw_imu.Imu``: ``get_data() -> {"gyro", "accelero"}`` (robot frame)."""

    def __init__(
        self,
        sampling_freq,
        user_pitch_bias=0,
        calibrate=False,
        upside_down=False,
        *,
        bus=None,
        config: dict | None = None,
        start_thread: bool = True,
    ):
        cfg = config if config is not None else load_config()
        self.sampling_freq = sampling_freq
        if upside_down:
            log.warning("imu_upside_down is ignored for the MPU6050: mounting is set by 'axes' (tools/imu_check.py)")
        backend = cfg.get("backend", "mpu6050")
        if backend == "qsh":  # RB3 Gen 2 onboard ICM-42688 via the sensor hub (robot/imu_qsh.py)
            from imu_qsh import QshImu

            self.dev = QshImu()
        elif backend == "iio":  # RealSense D455 IMU (robot/imu_iio.py): camera on the head
            from imu_iio import IioImu

            self.dev = IioImu(accel_sign=float(cfg.get("accel_sign", -1.0)))
        else:
            self.dev = Mpu6050(bus=bus, address=int(cfg["address"]), bus_number=int(cfg["bus"]))
        self.idx, self.sign = parse_axes(cfg["axes"])
        self.x_offset = 0.0  # same knob as upstream (accel x tare)
        self.accel_offset = np.asarray(cfg.get("accel_offset") or [0.0, 0.0, 0.0], float)
        if np.any(self.accel_offset):
            log.info("IMU accel offset (robot frame, m/s^2): %s", np.round(self.accel_offset, 3))
        self.errors = 0

        bias = cfg.get("gyro_bias")
        if calibrate or bias is None or cfg.get("calibrate_seconds", 0) > 0:
            measured = self.measure_gyro_bias(cfg.get("calibrate_seconds") or 2.0)
            if measured is not None:
                bias = measured
            elif bias is not None:
                log.warning("robot moved during gyro calibration: using saved bias %s", np.round(bias, 4))
            else:
                log.warning("robot moved during gyro calibration and no saved bias: gyro bias NOT removed")
        self.gyro_bias = np.zeros(3) if bias is None else np.asarray(bias, float)

        self.last_imu_data = {"gyro": np.zeros(3), "accelero": np.zeros(3)}
        self.imu_queue: Queue = Queue(maxsize=1)
        if start_thread:
            threading.Thread(target=self.imu_worker, daemon=True).start()

    # -- frames ------------------------------------------------------------------------
    def to_robot(self, v: np.ndarray) -> np.ndarray:
        return self.sign * v[self.idx]

    def read_robot(self) -> tuple[np.ndarray, np.ndarray]:
        accel, gyro = self.dev.read_si()
        gyro = gyro - self.gyro_bias  # bias lives in the sensor frame
        return self.to_robot(accel), self.to_robot(gyro)

    # -- calibration -------------------------------------------------------------------
    def measure_gyro_bias(self, seconds: float = 2.0, rate_hz: float = 100.0, max_std: float = 0.02):
        """Mean sensor-frame gyro while still; None if it moved (std > max_std rad/s)."""
        n = max(20, int(seconds * rate_hz))
        samples = []
        for _ in range(n):
            try:
                samples.append(self.dev.read_si()[1])
            except OSError:
                pass
            time.sleep(1.0 / rate_hz)
        if len(samples) < n // 2:
            return None
        s = np.array(samples)
        if s.std(axis=0).max() > max_std:
            return None
        bias = s.mean(axis=0)
        log.info("gyro bias (sensor frame, rad/s): %s", np.round(bias, 4))
        return bias

    # -- worker (mirrors upstream raw_imu) -----------------------------------------------
    def sample(self) -> dict:
        """One reading in the policy's terms: robot frame, gyro bias and tilt correction removed."""
        accel, gyro = self.read_robot()
        accel = accel - self.accel_offset
        accel[0] -= self.x_offset
        return {"gyro": gyro, "accelero": accel}

    def imu_worker(self):
        period = 1.0 / self.sampling_freq
        while True:
            t0 = time.time()
            try:
                data = self.sample()
                self.errors = 0
            except OSError as exc:
                self.errors += 1
                if self.errors in (1, 10) or self.errors % 100 == 0:
                    log.error("[IMU] read failed x%d: %s", self.errors, exc)
                if self.errors % 25 == 0:
                    try:
                        self.dev.init()
                    except OSError:
                        pass
                time.sleep(period)
                continue
            try:
                self.imu_queue.put_nowait(data)
            except Full:  # keep the newest sample, unlike upstream's blocking put
                try:
                    self.imu_queue.get_nowait()
                except Empty:
                    pass
                self.imu_queue.put_nowait(data)
            time.sleep(max(0.0, period - (time.time() - t0)))

    def get_data(self):
        try:
            self.last_imu_data = self.imu_queue.get(False)
        except Empty:
            pass
        return self.last_imu_data


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    imu = Imu(50)
    while True:
        d = imu.get_data()
        print("gyro", np.around(d["gyro"], 3), " accelero", np.around(d["accelero"], 3))
        time.sleep(1 / 25)

"""MPU6050 drop-in: scaling, mounting derivation for every orientation, bias, upstream hook."""

import itertools
import json
import logging
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot"))

import imu_mpu6050 as m

G = m.G


def rotations():
    """All 24 proper axis-aligned rotations (sensor-from-robot)."""
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            r = np.zeros((3, 3))
            for i, (p, s) in enumerate(zip(perm, signs)):
                r[i, p] = s
            if round(np.linalg.det(r)) == 1:
                yield r


class FakeBus:
    """Serves raw registers for a chip mounted with rotation R (sensor = R @ robot)."""

    def __init__(self, R, accel_robot, gyro_robot, gyro_bias_sensor=(0, 0, 0), whoami=0x68):
        self.R = np.asarray(R, float)
        self.accel_robot = np.asarray(accel_robot, float)
        self.gyro_robot = np.asarray(gyro_robot, float)
        self.bias = np.asarray(gyro_bias_sensor, float)
        self.whoami = whoami
        self.writes = {}

    def write_byte_data(self, addr, reg, val):
        self.writes[reg] = val

    def read_byte_data(self, addr, reg):
        assert reg == m.WHO_AM_I
        return self.whoami

    def read_i2c_block_data(self, addr, reg, n):
        assert (reg, n) == (m.ACCEL_XOUT_H, 14)
        a = self.R @ self.accel_robot / G * m.ACCEL_LSB_PER_G
        g = (self.R @ self.gyro_robot + self.bias) * 180 / math.pi * m.GYRO_LSB_PER_DPS
        out = []
        for v in [*a, 0.0, *g]:
            iv = int(round(v)) & 0xFFFF  # noqa: RUF046  (v can be a numpy float)
            out += [iv >> 8, iv & 0xFF]
        return out


def cfg(**kw):
    return {**m.DEFAULTS, "calibrate_seconds": 0, "gyro_bias": [0, 0, 0], **kw}


def test_scaling_flat_identity():
    bus = FakeBus(np.eye(3), [0, 0, G], [0.1, -0.2, 0.3])
    imu = m.Imu(50, bus=bus, config=cfg(), start_thread=False)
    a, g = imu.read_robot()
    assert np.allclose(a, [0, 0, G], atol=0.01)
    assert np.allclose(g, [0.1, -0.2, 0.3], atol=0.003)
    assert bus.writes[m.GYRO_CONFIG] == 0x18 and bus.writes[m.ACCEL_CONFIG] == 0x08
    assert m.ACCEL_CONFIG2 not in bus.writes  # reserved on a genuine MPU6050


def test_tilt_correction_is_subtracted_from_accel_only():
    bus = FakeBus(np.eye(3), [0.9, 0.1, G], [0.1, -0.2, 0.3])
    imu = m.Imu(50, bus=bus, config=cfg(accel_offset=[0.766, 0.104, 0.0]), start_thread=False)
    d = imu.sample()
    assert np.allclose(d["accelero"], [0.134, -0.004, G], atol=0.01)  # = the sim's standing reading
    assert np.allclose(d["gyro"], [0.1, -0.2, 0.3], atol=0.003)  # gyro untouched


def test_no_accel_offset_in_config_means_none():
    bus = FakeBus(np.eye(3), [0.9, 0.1, G], [0, 0, 0])
    imu = m.Imu(50, bus=bus, config={k: v for k, v in cfg().items() if k != "accel_offset"}, start_thread=False)
    assert np.allclose(imu.sample()["accelero"], [0.9, 0.1, G], atol=0.01)


def test_mpu6500_clone_gets_accel_dlpf():
    bus = FakeBus(np.eye(3), [0, 0, G], [0, 0, 0], whoami=0x70)
    m.Imu(50, bus=bus, config=cfg(), start_thread=False)
    assert bus.writes[m.ACCEL_CONFIG2] == 0x03


@pytest.mark.parametrize("R", list(rotations()))
def test_derive_axes_recovers_every_mounting(R):
    th = math.radians(40)
    still = R @ np.array([0, 0, G])
    nose_down = R @ np.array([-G * math.sin(th), 0, G * math.cos(th)])  # robot pitched forward
    spec = m.derive_axes(still, nose_down)
    idx, sign = m.parse_axes(spec)
    rng = np.random.default_rng(0)
    for _ in range(5):
        v_robot = rng.normal(size=3)
        assert np.allclose(sign * (R @ v_robot)[idx], v_robot)


@pytest.mark.parametrize("R", [r for i, r in enumerate(rotations()) if i % 5 == 0])
def test_end_to_end_signs_match_sim_convention(R):
    """Turn left (+z), roll right-side-down (+x), pitch nose-down (+y): robot-frame gyro after mapping."""
    th = math.radians(40)
    spec = m.derive_axes(R @ [0, 0, G], R @ [-G * math.sin(th), 0, G * math.cos(th)])
    bias = np.array([0.02, -0.01, 0.015])
    bus = FakeBus(R, [0, 0, G], [0.0, 0.0, 0.0], gyro_bias_sensor=bias)
    imu = m.Imu(50, bus=bus, config=cfg(axes=spec, calibrate_seconds=0.2, gyro_bias=None), start_thread=False)
    assert np.allclose(imu.gyro_bias, bias, atol=0.002)
    for w in ([0, 0, 1.0], [1.0, 0, 0], [0, 1.0, 0]):
        bus.gyro_robot = np.array(w)
        _, g = imu.read_robot()
        assert np.allclose(g, w, atol=0.01)


def test_moving_during_calibration_falls_back_to_saved_bias():
    class Shaky(FakeBus):
        k = 0

        def read_i2c_block_data(self, addr, reg, n):
            self.k += 1
            self.gyro_robot = np.array([0.5 if self.k % 2 else -0.5, 0, 0])
            return super().read_i2c_block_data(addr, reg, n)

    bus = Shaky(np.eye(3), [0, 0, G], [0, 0, 0])
    imu = m.Imu(50, bus=bus, config=cfg(calibrate_seconds=0.2, gyro_bias=[0.01, 0.02, 0.03]), start_thread=False)
    assert np.allclose(imu.gyro_bias, [0.01, 0.02, 0.03])


@pytest.mark.parametrize("bad", ["x,y", "x,x,z", "x,y,-z", "q,y,z"])
def test_bad_axes_rejected(bad):
    with pytest.raises(ValueError):
        m.parse_axes(bad)


def test_jero_walk_installs_driver_in_place_of_upstream(tmp_path, monkeypatch):
    import jero_walk

    conf = tmp_path / "imu.json"
    conf.write_text(json.dumps({"axes": "-y,x,z", "gyro_bias": [0, 0, 0]}))
    monkeypatch.setattr(m, "CONFIG_PATH", conf)
    monkeypatch.delitem(sys.modules, "mini_bdx_runtime.raw_imu", raising=False)
    jero_walk.install_mpu6050(logging.getLogger("test"))
    assert sys.modules["mini_bdx_runtime.raw_imu"] is m
    monkeypatch.delitem(sys.modules, "mini_bdx_runtime.raw_imu")

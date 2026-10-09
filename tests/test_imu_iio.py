"""RB3 onboard IMU (Linux IIO) driver on a fake sysfs tree, and its use through imu_mpu6050.Imu."""

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot"))

import imu_iio
import imu_mpu6050 as m

DEV = "iio_device{}"  # sysfs uses "iio:device{}"; ":" is not a legal Windows file name


@pytest.fixture(autouse=True)
def _glob(monkeypatch):
    monkeypatch.setattr(imu_iio, "DEVICE_GLOB", "iio_device*")


def make_sysfs(root: Path, accel=(-4, 89, -949), gyro=(1525, -915, 2746), rate="10.000000"):
    for n, (name, kind, raw, scale) in enumerate(
        (("accel_3d", "accel", accel, "0.009806650"), ("gyro_3d", "anglvel", gyro, "0.000001745"))
    ):
        d = root / DEV.format(n)
        d.mkdir(parents=True)
        (d / "name").write_text(name + "\n")
        (d / f"in_{kind}_scale").write_text(scale + "\n")
        (d / f"in_{kind}_offset").write_text("0\n")
        (d / f"in_{kind}_sampling_frequency").write_text(rate + "\n")
        for ax, v in zip("xyz", raw):
            (d / f"in_{kind}_{ax}_raw").write_text(f"{v}\n")
    (root / DEV.format(2)).mkdir()
    (root / DEV.format(2) / "name").write_text("pmic adc\n")
    return root


def test_reads_si_units_with_hid_gravity_flipped_to_specific_force(tmp_path):
    dev = imu_iio.IioImu(root=make_sysfs(tmp_path))
    a, g = dev.read_si()
    assert a == pytest.approx(np.array([4, -89, 949]) * 0.00980665)  # flat, face up: +9.3 on z
    assert g == pytest.approx(np.array([1525, -915, 2746]) * 1.745e-6)
    assert dev.rates() == (100.0, 100.0)  # raised from 10 Hz (writable here)


def test_rereads_fresh_values(tmp_path):
    root = make_sysfs(tmp_path)
    dev = imu_iio.IioImu(root=root)
    (root / DEV.format(0) / "in_accel_z_raw").write_text("-1000\n")
    assert dev.read_si()[0][2] == pytest.approx(1000 * 0.00980665)


def test_missing_device_says_so(tmp_path):
    (tmp_path / DEV.format(0)).mkdir()
    (tmp_path / DEV.format(0) / "name").write_text("something else\n")
    with pytest.raises(FileNotFoundError, match="accel_3d"):
        imu_iio.IioImu(root=tmp_path)


def test_imu_uses_iio_backend_from_config(tmp_path, monkeypatch):
    root = make_sysfs(tmp_path)
    real = imu_iio.IioImu
    monkeypatch.setattr(imu_iio, "IioImu", lambda **kw: real(root=root, **kw))
    cfg = dict(m.DEFAULTS, backend="iio", axes="x,y,z", gyro_bias=[0, 0, 0], calibrate_seconds=0)
    imu = m.Imu(50, config=cfg, start_thread=False)
    d = imu.sample()
    assert d["accelero"][2] == pytest.approx(949 * 0.00980665) and d["gyro"][0] == pytest.approx(1525 * 1.745e-6)

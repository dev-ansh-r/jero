"""Bench mode with absent servos (robot/missing_servos.py), against upstream's real HWI class."""

import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot"))
sys.path.insert(0, str(ROOT / "upstream" / "Open_Duck_Mini_Runtime" / "mini_bdx_runtime"))

import missing_servos as ms

HEAD = [30, 31, 32, 33]


class FakeBus:
    """rustypot.feetech stand-in: every ID except ``absent`` answers; absent IDs time out."""

    def __init__(self, port=None, baud=None, absent=HEAD):
        self.absent = set(absent)
        self.pos = {}
        self.calls = []

    def _check(self, ids):
        if self.absent & set(ids):
            raise OSError("Operation timed out")

    def read_present_position(self, ids):
        self._check(ids)
        return [self.pos.get(i, 0.1 * i) for i in ids]

    def read_present_velocity(self, ids):
        self._check(ids)
        return [0.5 for _ in ids]

    def write_goal_position(self, ids, values):
        self._check(ids)
        self.pos.update(zip(ids, values))

    def set_kps(self, ids, kps):
        self._check(ids)
        self.calls.append(("set_kps", list(ids), list(kps)))

    def set_kds(self, ids, kds):
        self._check(ids)

    def disable_torque(self, ids):
        self._check(ids)
        self.calls.append(("disable_torque", list(ids)))


def test_reads_fill_missing_with_last_goal_and_zero_velocity():
    io = ms.MissingServoIO(FakeBus(), HEAD)
    io.write_goal_position([10, 30], [0.2, -0.4])
    assert io.read_present_position([10, 30, 31]) == [0.2, -0.4, 0.0]
    assert io.read_present_velocity([10, 30]) == [0.5, 0.0]


def test_writes_skip_missing_ids_including_generic_calls():
    bus = FakeBus()
    io = ms.MissingServoIO(bus, HEAD)
    io.set_kps([10, 30, 11], [32, 32, 33])  # generic (__getattr__) path with values
    io.disable_torque([31, 12])  # generic path without values
    io.set_kps([30, 31], [1, 1])  # only missing IDs: nothing sent
    assert bus.calls == [("set_kps", [10, 11], [32, 33]), ("disable_torque", [12])]


def test_parse_ids():
    assert ms.parse_ids("30, 31,32,33") == HEAD
    assert ms.parse_ids("") == []


def test_upstream_hwi_runs_with_head_chain_absent(monkeypatch):
    """install() must take effect for HWI objects built afterwards, like RLWalk's."""
    monkeypatch.setitem(sys.modules, "rustypot", types.SimpleNamespace(feetech=FakeBus))
    monkeypatch.delitem(sys.modules, "mini_bdx_runtime.rustypot_position_hwi", raising=False)
    try:
        import mini_bdx_runtime.rustypot_position_hwi as hwi_mod
    except ImportError as exc:  # submodule not checked out
        pytest.skip(f"upstream runtime not available: {exc}")

    ms.install(HEAD)
    joints = [
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
        "neck_pitch", "head_pitch", "head_yaw", "head_roll",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
    ]  # fmt: skip
    cfg = types.SimpleNamespace(joints_offset=dict.fromkeys(joints, 0.0))
    hwi = hwi_mod.HWI(cfg, "/dev/null")
    hwi.set_position_all(hwi.init_pos)

    pos = hwi.get_present_positions()
    vel = hwi.get_present_velocities()
    assert pos is not None and len(pos) == 14
    assert vel is not None and len(vel) == 14
    # head joints read back exactly where they were commanded (init pose), legs from the bus
    for name in ("neck_pitch", "head_pitch", "head_yaw", "head_roll"):
        assert pos[joints.index(name)] == pytest.approx(hwi.init_pos[name], abs=1e-3)
    assert pos[joints.index("left_knee")] == pytest.approx(hwi.init_pos["left_knee"], abs=1e-3)
    assert vel[joints.index("head_yaw")] == 0.0 and vel[joints.index("left_knee")] == 0.5

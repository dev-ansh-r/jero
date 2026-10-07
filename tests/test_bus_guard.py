"""robot/bus_guard.py: a rustypot panic becomes one skipped read, against upstream's real HWI."""

import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot"))
sys.path.insert(0, str(ROOT / "upstream" / "Open_Duck_Mini_Runtime" / "mini_bdx_runtime"))

import bus_guard


class PanicException(BaseException):
    """Stands in for pyo3_runtime.PanicException (a BaseException)."""


class FakeBus:
    opened = 0
    panic_next = False

    def __init__(self, port=None, baud=None):
        FakeBus.opened += 1

    def read_present_position(self, ids):
        if FakeBus.panic_next:
            FakeBus.panic_next = False
            raise PanicException("assertion failed: self.is_input_buffer_empty(port)?")
        return [0.0 for _ in ids]

    read_present_velocity = read_present_position

    def write_goal_position(self, ids, values):
        pass

    set_kps = set_kds = write_goal_position


@pytest.fixture(autouse=True)
def reset():
    FakeBus.opened = 0
    FakeBus.panic_next = False


def test_panic_becomes_bus_error_and_port_is_reopened():
    io = bus_guard.GuardedIO(FakeBus, "/dev/ttyACM0", 1000000)
    FakeBus.panic_next = True
    with pytest.raises(bus_guard.BusError):
        io.read_present_position([10, 11])
    assert FakeBus.opened == 2 and io.panics == 1
    assert io.read_present_position([10, 11]) == [0.0, 0.0]  # next read works on the new port


def test_ordinary_errors_and_ctrl_c_pass_through():
    class Bus(FakeBus):
        def read_present_position(self, ids):
            raise OSError("Operation timed out")

        def read_present_velocity(self, ids):
            raise KeyboardInterrupt

    io = bus_guard.GuardedIO(Bus, "p", 1)
    with pytest.raises(OSError, match="timed out"):
        io.read_present_position([1])
    with pytest.raises(KeyboardInterrupt):
        io.read_present_velocity([1])
    assert Bus.opened == 1  # no reopen for these


def test_upstream_hwi_skips_a_read_instead_of_crashing(monkeypatch):
    monkeypatch.setitem(sys.modules, "rustypot", types.SimpleNamespace(feetech=FakeBus))
    monkeypatch.delitem(sys.modules, "mini_bdx_runtime.rustypot_position_hwi", raising=False)
    try:
        import mini_bdx_runtime.rustypot_position_hwi as hwi_mod
    except ImportError as exc:
        pytest.skip(f"upstream runtime not available: {exc}")

    bus_guard.install()
    joints = [
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
        "neck_pitch", "head_pitch", "head_yaw", "head_roll",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
    ]  # fmt: skip
    hwi = hwi_mod.HWI(types.SimpleNamespace(joints_offset=dict.fromkeys(joints, 0.0)), "/dev/null")
    FakeBus.panic_next = True
    assert hwi.get_present_positions() is None  # walk loop: obs None -> skip this step
    assert len(hwi.get_present_positions()) == 14  # and carries on

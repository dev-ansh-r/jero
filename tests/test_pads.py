"""PS4 pad through upstream's real XBoxController (robot/pads.py), with a fake pygame."""

import sys
import types
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot"))
sys.path.insert(0, str(ROOT / "upstream" / "Open_Duck_Mini_Runtime" / "mini_bdx_runtime"))

import pads


class FakePS4:
    """Raw DualShock 4 as pygame reports it on the Pi (measured 6 Oct 2026)."""

    def __init__(self, i=0):
        self.axes = [0.0, 0.0, -1.0, 0.0, 0.0, -1.0]  # L2 (2) and R2 (5) rest at -1
        self.buttons = [False] * 13
        self.hat = (0, 0)

    def init(self):
        pass

    def get_name(self):
        return "Sony Interactive Entertainment Wireless Controller"

    def get_numaxes(self):
        return 6

    def get_axis(self, i):
        return self.axes[i]

    def get_button(self, i):
        return self.buttons[i]

    def get_hat(self, i):
        return self.hat


@pytest.fixture
def xc(monkeypatch):
    pad = FakePS4()
    fake = types.SimpleNamespace(
        init=lambda: None,
        JOYBUTTONDOWN=1,
        JOYBUTTONUP=2,
        joystick=types.SimpleNamespace(get_count=lambda: 1, Joystick=lambda i: pad),
        event=types.SimpleNamespace(get=list, pump=lambda: None),
    )
    monkeypatch.setitem(sys.modules, "pygame", fake)
    monkeypatch.delitem(sys.modules, "mini_bdx_runtime.xbox_controller", raising=False)
    try:
        import mini_bdx_runtime.xbox_controller  # noqa: F401
    except ImportError as exc:
        pytest.skip(f"upstream runtime not available: {exc}")
    return pad


def test_detects_ps4_by_name():
    assert pads.detect("Sony Interactive Entertainment Wireless Controller") == "ps4"
    assert pads.detect("Xbox Wireless Controller") == "xbox"


def test_ps4_at_rest_commands_nothing(xc):
    ctrl = pads.make_controller(20, "auto")
    cmds = np.asarray(ctrl.get_commands()[0])
    assert np.allclose(cmds, 0.0)  # raw axis 2 is L2 at -1: unmapped this would be a full-speed turn


def test_ps4_sticks_and_triggers_map_like_xbox(xc):
    ctrl = pads.make_controller(20, "ps4")
    xc.axes = [0.0, -1.0, 1.0, 1.0, 0.0, 1.0]  # left stick up, L2 + R2 fully pressed, right stick right
    out = ctrl.get_commands()
    cmds, left_trigger, right_trigger = np.asarray(out[0]), out[7], out[8]
    assert cmds[0] == pytest.approx(0.15)  # forward, X_RANGE max
    assert cmds[1] == pytest.approx(0.0)
    assert cmds[2] == pytest.approx(-1.0)  # right stick right -> turn right (negative yaw)
    assert left_trigger == pytest.approx(1.0) and right_trigger == pytest.approx(1.0)


def test_ps4_buttons(xc):
    js = pads.RemappedJoystick(xc, pads.PS4)
    xc.buttons[0] = True  # cross
    xc.buttons[2] = True  # triangle
    xc.buttons[4] = True  # L1
    assert js.get_button(0)  # upstream A: pause
    assert js.get_button(4)  # upstream Y: head mode
    assert js.get_button(6)  # upstream LB: faster stepping
    assert not js.get_button(3) and not js.get_button(7)

"""Tests for the Pi-side controller mux and the brain's follow controller (no hardware needed).

Uses the real upstream ``mini_bdx_runtime.buttons`` from the pinned submodule.
"""

import sys
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "jero_link" / "src"))
sys.path.insert(0, str(ROOT / "robot"))
sys.path.insert(0, str(ROOT / "brain"))
sys.path.insert(0, str(ROOT / "upstream" / "Open_Duck_Mini_Runtime" / "mini_bdx_runtime"))

pytest.importorskip("mini_bdx_runtime.buttons", reason="git submodule update --init upstream/Open_Duck_Mini_Runtime")

from follow_control import FollowController, Target
from jero_controller import MuxController
from mini_bdx_runtime.buttons import Buttons

from jero_link import Command, Receiver, encode

KEY = bytes.fromhex("33" * 32)


class FakeXbox:
    """Mimics upstream XBoxController.get_last_command()."""

    def __init__(self):
        self.cmds = [0.0] * 7
        self.buttons = Buttons()
        self.raw = dict.fromkeys(("A", "B", "X", "Y", "LB", "RB", "up", "down"), False)
        self.lt = self.rt = 0.0

    def get_last_command(self):
        r = self.raw
        self.buttons.update(r["A"], r["B"], r["X"], r["Y"], r["LB"], r["RB"], r["up"], r["down"])
        return np.array(self.cmds), self.buttons, self.lt, self.rt


class Clock:
    t = 50.0

    def __call__(self):
        return self.t


def link_with(cmd, clock, seq=1):
    rx = Receiver(key=KEY, clock=clock)
    rx.handle(encode(cmd, seq=seq, session="brain", key=KEY))
    return rx


def test_idle_is_zero():
    cmds, buttons, lt, rt = MuxController().get_last_command()
    assert cmds.tolist() == [0.0] * 7 and (lt, rt) == (0.0, 0.0)
    assert isinstance(buttons, Buttons)


def test_link_drives_when_pad_is_centered():
    clock = Clock()
    mux = MuxController(xbox=FakeXbox(), link=link_with(Command(vx=0.1, wz=0.4), clock))
    cmds, *_ = mux.get_last_command()
    assert cmds[:3].tolist() == [0.1, 0.0, 0.4] and mux.source == "link"


def test_pad_overrides_link():
    clock = Clock()
    pad = FakeXbox()
    pad.cmds = [0.0, 0.0, -0.8, 0, 0, 0, 0]
    mux = MuxController(xbox=pad, link=link_with(Command(vx=0.1), clock))
    cmds, *_ = mux.get_last_command()
    assert cmds[:3].tolist() == [0.0, 0.0, -0.8] and mux.source == "xbox"


def test_stale_link_falls_back_to_zero():
    clock = Clock()
    mux = MuxController(link=link_with(Command(vx=0.1, ttl_ms=200), clock))
    assert mux.get_last_command()[0][0] == 0.1
    clock.t += 0.3
    assert mux.get_last_command()[0].tolist() == [0.0] * 7 and mux.source == "idle"


def test_link_press_triggers_upstream_button_once():
    clock = Clock()
    rx = link_with(Command(buttons=("A",)), clock)
    mux = MuxController(link=rx)
    time.sleep(0.25)  # upstream Button debounces on wall-clock time since construction
    triggers = 0
    for _ in range(10):  # 10 control ticks at 50 Hz = 0.2 s
        _, buttons, *_ = mux.get_last_command()
        triggers += buttons.A.triggered
        clock.t += 0.02
    assert triggers == 1


def test_pad_buttons_pass_through():
    pad = FakeXbox()
    mux = MuxController(xbox=pad)
    pad.raw["B"] = True
    _, buttons, *_ = mux.get_last_command()
    assert buttons.B.is_pressed


def test_triggers_take_max():
    clock = Clock()
    pad = FakeXbox()
    pad.lt = 0.2
    mux = MuxController(xbox=pad, link=link_with(Command(left_trigger=0.7, right_trigger=0.1), clock))
    _, _, lt, rt = mux.get_last_command()
    assert (lt, rt) == (0.7, 0.1)


def test_follow_controller():
    ctrl = FollowController()
    assert ctrl.update(None) == (0.0, 0.0)
    assert ctrl.update(Target(cx=0.5, area=0.12)) == (0.0, 0.0)  # centred, at distance
    vx, wz = ctrl.update(Target(cx=0.5, area=0.02))
    assert vx > 0 and wz == 0.0  # far -> walk forward
    vx, wz = ctrl.update(Target(cx=0.1, area=0.02))
    assert vx == 0.0 and wz > 0  # far left -> turn left in place first
    vx, wz = ctrl.update(Target(cx=0.5, area=0.9))
    assert vx == ctrl.cfg.min_vx  # too close -> slow back-off, capped
    vx, wz = ctrl.update(Target(cx=0.0, area=0.12))
    assert wz == ctrl.cfg.max_wz

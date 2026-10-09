"""robot/board_shims.py: upstream's walk script imports on a non-Pi board (no Adafruit `board`)."""

import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "upstream" / "Open_Duck_Mini_Runtime"
sys.path.insert(0, str(ROOT / "robot"))

import board_shims


def test_no_feet_backend_reads_not_touching():
    feet = board_shims.make_feet("none")()
    assert feet.get() == [False, False]


def test_gpiod_backend_pressed_is_low(monkeypatch):
    class Value:
        ACTIVE, INACTIVE = "active", "inactive"

    class Request:
        def __init__(self):
            self.vals = {22: Value.INACTIVE, 27: Value.ACTIVE}  # left pressed (pulled low), right not

        def get_values(self, lines):
            return [self.vals[i] for i in lines]

        def release(self):
            pass

    calls = {}

    def request_lines(chip, consumer, config):
        calls["chip"], calls["lines"] = chip, next(iter(config))
        return Request()

    line = types.SimpleNamespace(
        Bias=types.SimpleNamespace(PULL_UP="pull-up"),
        Direction=types.SimpleNamespace(INPUT="input"),
        Value=Value,
    )
    gpiod = types.SimpleNamespace(LineSettings=lambda **kw: kw, request_lines=request_lines, line=line)
    monkeypatch.setitem(sys.modules, "gpiod", gpiod)
    monkeypatch.setitem(sys.modules, "gpiod.line", line)

    feet = board_shims.make_feet("gpiod", "/dev/gpiochip4", (22, 27))()
    assert feet.get() == [True, False]
    assert calls == {"chip": "/dev/gpiochip4", "lines": (22, 27)}


def test_gpiod_backend_needs_chip_and_lines():
    with pytest.raises(SystemExit):
        board_shims.make_feet("gpiod")


def test_upstream_walk_imports_without_pi_gpio(monkeypatch):
    """With the shims (and the MPU driver as raw_imu), v2_rl_walk_mujoco imports with no Blinka."""
    if not (RUNTIME / "scripts" / "v2_rl_walk_mujoco.py").is_file():
        pytest.skip("upstream runtime submodule not checked out")
    for name in ("pygame", "onnxruntime", "rustypot"):  # third-party libs a CI box may not have
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "board", None)  # importing Adafruit `board` must not happen
    monkeypatch.syspath_prepend(str(RUNTIME / "mini_bdx_runtime"))
    monkeypatch.syspath_prepend(str(RUNTIME / "scripts"))
    for name in list(sys.modules):
        if name.startswith("mini_bdx_runtime") or name == "v2_rl_walk_mujoco":
            monkeypatch.delitem(sys.modules, name, raising=False)

    import imu_mpu6050

    monkeypatch.setitem(sys.modules, "mini_bdx_runtime.raw_imu", imu_mpu6050)
    board_shims.install("none")
    import v2_rl_walk_mujoco as walk

    assert walk.FeetContacts().get() == [False, False]
    with pytest.raises(RuntimeError, match="expression_features"):
        walk.Eyes()
    with pytest.raises(RuntimeError):
        walk.Antennas()

"""robot/feetech_io.py against a simulated Pico bridge, and through upstream's real HWI."""

import math
import struct
import sys
import time
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot"))
sys.path.insert(0, str(ROOT / "upstream" / "Open_Duck_Mini_Runtime" / "mini_bdx_runtime"))

import feetech_io as fio

IDS = [20, 21, 22, 23, 24, 30, 31, 32, 33, 10, 11, 12, 13, 14]


def status(sid, data: bytes) -> bytes:
    body = bytes([sid, len(data) + 2, 0]) + data
    return b"\xff\xff" + body + bytes([fio.checksum(body)])


class FakeBridge:
    """Answers like firmware/pico_bridge: whole batch in one chunk, or a marker, or nothing."""

    def __init__(self):
        self.pos = {i: 2048 + 10 * i for i in IDS}  # raw servo units
        self.speed = {i: 0 for i in IDS}
        self.queue = []  # [ready_at, bytes]
        self.drop_next = False
        self.late_next = 0.0  # seconds
        self.marker_next = False
        self.writes = []
        self.sync_reads = 0

    def write(self, data: bytes):
        assert data[:2] == b"\xff\xff" and data[-1] == fio.checksum(data[2:-1])
        inst, params = data[4], data[5:-1]
        if inst == fio.INST_SYNC_WRITE:
            addr, n = params[0], params[1]
            for k in range(2, len(params), n + 1):
                sid, val = params[k], params[k + 1 : k + 1 + n]
                self.writes.append((addr, sid, val))
                if addr == fio.REG_GOAL_POSITION:
                    self.pos[sid] = struct.unpack("<h", val)[0]
            return
        assert inst == fio.INST_SYNC_READ
        addr, n, ids = params[0], params[1], list(params[2:])
        self.sync_reads += 1
        if self.drop_next:
            self.drop_next = False
            return
        if self.marker_next:
            self.marker_next = False
            self.queue.append([time.monotonic(), status(0, b"")])
            return
        if addr == fio.REG_PRESENT_POSITION and n == 4:
            batch = b"".join(status(i, struct.pack("<hH", self.pos[i], self.speed[i])) for i in ids)
        elif addr == fio.REG_PRESENT_POSITION:
            batch = b"".join(status(i, struct.pack("<h", self.pos[i])) for i in ids)
        else:
            batch = b"".join(status(i, struct.pack("<H", self.speed[i])) for i in ids)
        self.queue.append([time.monotonic() + self.late_next, batch])
        self.late_next = 0.0

    def read(self, timeout: float) -> bytes:
        end = time.monotonic() + timeout
        while True:
            now = time.monotonic()
            if self.queue and self.queue[0][0] <= now:
                return self.queue.pop(0)[1]
            if now >= end:
                return b""
            time.sleep(min(0.001, end - now))


@pytest.fixture
def bridge():
    return FakeBridge()


@pytest.fixture
def io(bridge):
    return fio.FeetechIO(bridge, timeout_s=0.025, quiet_after_error_s=0.04)


def test_conversions_match_rustypot():
    assert fio.pos_to_rad(2048) == pytest.approx(0.0)
    assert fio.rad_to_pos(0.0) == 2048
    assert fio.pos_to_rad(fio.rad_to_pos(1.0)) == pytest.approx(1.0, abs=2 * math.pi / 4096)
    assert fio.speed_to_rads(100) == pytest.approx(2 * math.pi * 100 / 4095)
    assert fio.speed_to_rads((1 << 15) + 100) == pytest.approx(-2 * math.pi * 100 / 4095)


def test_write_then_read_round_trip(io):
    goal = [0.1 * (k - 7) for k in range(len(IDS))]
    io.write_goal_position(IDS, goal)
    got = io.read_present_position(IDS)
    assert got == pytest.approx(goal, abs=2 * math.pi / 4096)
    assert io.read_present_velocity(IDS) == [0.0] * len(IDS)


def test_lost_reply_fails_fast_and_next_read_works(io, bridge):
    io.read_present_position(IDS)  # first call pays the one-off quiet drain
    bridge.drop_next = True
    t0 = time.monotonic()
    with pytest.raises(fio.BusError):
        io.read_present_position(IDS)
    assert time.monotonic() - t0 < 0.1  # rustypot would take 1 s
    assert len(io.read_present_position(IDS)) == len(IDS)


def test_bridge_error_marker_fails_immediately(io, bridge):
    io.read_present_position(IDS)
    bridge.marker_next = True
    t0 = time.monotonic()
    with pytest.raises(fio.BusError, match="bridge reported"):
        io.read_present_position(IDS)
    assert time.monotonic() - t0 < 0.015


def test_late_position_reply_is_never_read_as_velocity(io, bridge):
    """The dangerous case: same IDs and length, so only draining keeps the data honest."""
    io.read_present_position(IDS)
    bridge.speed = {i: 7 for i in IDS}
    bridge.late_next = 0.06  # arrives well after the ~32 ms deadline (Windows sleeps are coarse)
    with pytest.raises(fio.BusError):
        io.read_present_position(IDS)
    vel = io.read_present_velocity(IDS)  # the late position batch must have been drained
    assert vel == pytest.approx([fio.speed_to_rads(7)] * len(IDS))


def test_sync_writes_encode_registers(io, bridge):
    io.set_kps(IDS[:2], [32.0, 16.0])
    io.enable_torque(IDS[:1])
    io.disable_torque(IDS[:1])
    assert (fio.REG_P, 20, b"\x20") in bridge.writes and (fio.REG_P, 21, b"\x10") in bridge.writes
    assert (fio.REG_TORQUE_ENABLE, 20, b"\x01") in bridge.writes
    assert (fio.REG_TORQUE_ENABLE, 20, b"\x00") in bridge.writes


def test_upstream_hwi_uses_it_and_skips_a_lost_read(monkeypatch, bridge):
    monkeypatch.setitem(sys.modules, "rustypot", types.SimpleNamespace(feetech=None))
    monkeypatch.delitem(sys.modules, "mini_bdx_runtime.rustypot_position_hwi", raising=False)
    try:
        import mini_bdx_runtime.rustypot_position_hwi as hwi_mod
    except ImportError as exc:
        pytest.skip(f"upstream runtime not available: {exc}")
    monkeypatch.setattr(fio, "SerialTransport", lambda port: bridge)
    monkeypatch.setattr(fio, "resolve_port", lambda port: port)

    fio.install()
    joints = [
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
        "neck_pitch", "head_pitch", "head_yaw", "head_roll",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
    ]  # fmt: skip
    hwi = hwi_mod.HWI(types.SimpleNamespace(joints_offset=dict.fromkeys(joints, 0.0)), "/dev/ttyACM0")
    hwi.set_position_all(hwi.init_pos)
    pos = hwi.get_present_positions()
    assert pos[joints.index("left_knee")] == pytest.approx(hwi.init_pos["left_knee"], abs=2e-3)

    bridge.drop_next = True
    t0 = time.monotonic()
    assert hwi.get_present_positions() is None  # walk loop: obs None -> skip this step
    assert time.monotonic() - t0 < 0.15
    assert len(hwi.get_present_velocities()) == 14


def test_port_lost_then_reopened(bridge):
    """The Pico reboots (watchdog): the tty vanishes, reads fail fast, then the IO reopens itself."""

    class Vanishing(FakeBridge):
        gone = False

        def write(self, data):
            if self.gone:
                raise fio.PortLost("bus bridge port lost: [Errno 5] Input/output error")
            super().write(data)

        def read(self, timeout):
            if self.gone:
                raise fio.PortLost("bus bridge port lost: end of file (device gone)")
            return super().read(timeout)

        def close(self):
            pass

    first = Vanishing()
    reopened = []

    def reopen():
        new = Vanishing()
        reopened.append(new)
        return new

    io = fio.FeetechIO(first, timeout_s=0.025, quiet_after_error_s=0.01, reopen=reopen)
    io.read_present_position(IDS)
    first.gone = True
    with pytest.raises(fio.BusError, match="port lost"):
        io.read_present_position(IDS)
    assert io.t is None  # closed right away, so the device can come back under the same name
    assert len(io.read_present_position(IDS)) == len(IDS)  # reopened on the next call
    assert len(reopened) == 1


def test_reopen_is_rate_limited_while_the_pico_is_away():
    calls = []

    def reopen():
        calls.append(1)
        raise FileNotFoundError("/dev/ttyACM0 not found and no Pico on USB")

    io = fio.FeetechIO(None, reopen=reopen)
    for _ in range(5):
        with pytest.raises(fio.BusError):
            io.read_present_position(IDS)
    assert len(calls) == 1  # at most one attempt per 0.2 s, each failing read stays fast


def test_position_then_velocity_is_one_bus_transaction(io, bridge):
    bridge.speed = {i: 100 + i for i in IDS}
    n0 = bridge.sync_reads
    pos = io.read_present_position(IDS)
    vel = io.read_present_velocity(IDS)
    assert bridge.sync_reads - n0 == 1  # one combined read (registers 56-59)
    assert len(pos) == len(vel) == len(IDS)
    assert vel == pytest.approx([fio.speed_to_rads(100 + i) for i in IDS])
    io.read_present_velocity(IDS)  # cache is single-use: a second call reads the bus again
    assert bridge.sync_reads - n0 == 2


def test_velocity_cache_not_used_for_other_ids_or_when_old(io, bridge):
    io.read_present_position(IDS)
    n0 = bridge.sync_reads
    io.read_present_velocity(IDS[:3])  # different IDs: fresh read
    assert bridge.sync_reads - n0 == 1
    io.read_present_position(IDS)
    time.sleep(0.03)  # older than vel_cache_s (15 ms)
    io.read_present_velocity(IDS)
    assert bridge.sync_reads - n0 == 3

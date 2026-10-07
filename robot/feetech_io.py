"""Feetech STS servo bus IO for upstream HWI, with a short timeout instead of rustypot's 1 s.

Drop-in for the ``rustypot.feetech(port, baudrate)`` object that upstream
``mini_bdx_runtime.rustypot_position_hwi.HWI`` uses: same methods, same units (radians, rad/s),
same conversions (rustypot device/feetech_sts3215.rs ``conv``).

Why: the USB link between the Pi Zero and the Pico bridge loses about 0.3-0.5 % of transactions.
rustypot waits 1 s for each, which freezes the 50 Hz walk. Here a lost reply raises ``BusError``
after ~25 ms; upstream HWI turns that into ``None`` and the walk skips one step.

Stale data is the real danger: a late *position* reply looks exactly like the next *velocity*
reply (same IDs, same length). So every transaction starts by draining the input, and after a
failure the next one first waits out a quiet window for late bytes.

``install()`` makes upstream's HWI use this instead of rustypot; nothing upstream is edited.
"""

from __future__ import annotations

import logging
import math
import os
import struct
import time

log = logging.getLogger("jero.feetech")

BROADCAST = 0xFE
INST_READ, INST_WRITE, INST_SYNC_READ, INST_SYNC_WRITE = 0x02, 0x03, 0x82, 0x83
# registers (rustypot device/feetech_sts3215.rs)
REG_MODE, REG_P, REG_D, REG_I = 33, 21, 22, 23
REG_TORQUE_ENABLE, REG_GOAL_POSITION, REG_GOAL_TIME = 40, 42, 44
REG_PRESENT_POSITION, REG_PRESENT_SPEED = 56, 58


class BusError(IOError):
    pass


def checksum(body: bytes) -> int:
    return ~sum(body) & 0xFF


def packet(sid: int, inst: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, inst]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


# rustypot conv: 2048 = 0 rad; speed has a sign bit (15)
def pos_to_rad(raw: int) -> float:
    return 2.0 * math.pi * raw / 4096.0 - math.pi


def rad_to_pos(rad: float) -> int:
    return int(4096.0 * (math.pi + rad) / (2.0 * math.pi))  # truncates toward 0, like Rust `as i16`


def speed_to_rads(raw: int) -> float:
    if raw > (1 << 15):
        raw = -(raw - (1 << 15))
    return 2.0 * math.pi * raw / 4095.0


class SerialTransport:
    """Raw, non-blocking access to the bridge's tty (Linux)."""

    def __init__(self, port: str):
        import termios

        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        speed = getattr(termios, "B1000000", termios.B115200)  # never 1200: reboots the Pico
        attrs[4] = attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def write(self, data: bytes) -> None:
        import select

        view = memoryview(data)
        deadline = time.monotonic() + 0.05
        while view:
            try:
                n = os.write(self.fd, view)
                view = view[n:]
            except BlockingIOError:
                if time.monotonic() > deadline:
                    raise BusError("write to the bus bridge timed out") from None
                select.select([], [self.fd], [], 0.005)

    def read(self, timeout: float) -> bytes:
        """Whatever arrives within ``timeout`` seconds (b"" if nothing)."""
        import select

        r, _, _ = select.select([self.fd], [], [], max(0.0, timeout))
        if not r:
            return b""
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""

    def close(self) -> None:
        os.close(self.fd)


class FeetechIO:
    def __init__(self, transport, timeout_s: float = 0.025, quiet_after_error_s: float = 0.04):
        self.t = transport
        self.timeout_s = timeout_s
        self.quiet_after_error_s = quiet_after_error_s
        self.errors = 0
        self._dirty = True  # drain carefully before the first transaction too
        self._last_log = 0.0

    # -- plumbing --------------------------------------------------------------------------
    def _drain(self) -> None:
        if self._dirty:
            # a late answer to a failed request may still be on its way: wait until it's quiet
            while self.t.read(self.quiet_after_error_s):
                pass
            self._dirty = False
        while self.t.read(0):
            pass

    def _fail(self, msg: str) -> None:
        self._dirty = True
        self.errors += 1
        now = time.monotonic()
        if now - self._last_log > 1.0:
            log.warning("servo bus: %s (errors so far: %d)", msg, self.errors)
            self._last_log = now
        raise BusError(msg)

    def _send(self, data: bytes) -> None:
        self._drain()
        self.t.write(data)

    def _read_replies(self, ids: list[int], datalen: int) -> list[bytes]:
        """Status packets from ``ids`` in that order, each with ``datalen`` data bytes."""
        size = 6 + datalen
        need = size * len(ids)
        buf = b""
        deadline = time.monotonic() + self.timeout_s + 0.0005 * len(ids)
        while len(buf) < need:
            left = deadline - time.monotonic()
            if left <= 0:
                self._fail(f"timeout: {len(buf)}/{need} bytes from IDs {ids}")
            buf += self.t.read(left)
            if len(buf) >= 6 and buf[:2] == b"\xff\xff" and buf[2] != ids[0]:
                # the Pico bridge's error marker (a packet from an ID nobody asked) or garbage
                self._fail(f"bridge reported a failed transaction for IDs {ids}")
        out = []
        for k, sid in enumerate(ids):
            p = buf[k * size : (k + 1) * size]
            if p[:2] != b"\xff\xff" or p[3] != datalen + 2:
                self._fail(f"bad reply framing for ID {sid}: {p.hex(' ')}")
            if p[2] != sid:
                self._fail(f"reply from ID {p[2]} where ID {sid} was expected")
            if p[-1] != checksum(p[2:-1]):
                self._fail(f"bad reply checksum for ID {sid}")
            out.append(p[5:-1])
        if len(buf) > need:
            self._dirty = True  # extra bytes: drain carefully next time
        return out

    def _sync_read(self, ids, addr: int, datalen: int) -> list[bytes]:
        ids = [int(i) for i in ids]
        self._send(packet(BROADCAST, INST_SYNC_READ, bytes([addr, datalen, *ids])))
        return self._read_replies(ids, datalen)

    def _sync_write(self, ids, addr: int, values: list[bytes]) -> None:
        datalen = len(values[0])
        params = bytes([addr, datalen]) + b"".join(bytes([int(i)]) + v for i, v in zip(ids, values))
        self._send(packet(BROADCAST, INST_SYNC_WRITE, params))

    # -- rustypot-compatible API (what upstream HWI and tools call) --------------------------
    def read_present_position(self, ids) -> list[float]:
        return [pos_to_rad(struct.unpack("<h", d)[0]) for d in self._sync_read(ids, REG_PRESENT_POSITION, 2)]

    def read_present_velocity(self, ids) -> list[float]:
        return [speed_to_rads(struct.unpack("<H", d)[0]) for d in self._sync_read(ids, REG_PRESENT_SPEED, 2)]

    def write_goal_position(self, ids, goal_position) -> None:
        vals = [struct.pack("<h", rad_to_pos(float(r))) for r in goal_position]
        self._sync_write(ids, REG_GOAL_POSITION, vals)

    def set_kps(self, ids, kps) -> None:
        self._sync_write(ids, REG_P, [bytes([int(k) & 0xFF]) for k in kps])

    def set_kds(self, ids, kds) -> None:
        self._sync_write(ids, REG_D, [bytes([int(k) & 0xFF]) for k in kds])

    def set_mode(self, ids, mode: int) -> None:
        self._sync_write(ids, REG_MODE, [bytes([int(mode)])] * len(ids))

    def enable_torque(self, ids) -> None:
        self._sync_write(ids, REG_TORQUE_ENABLE, [b"\x01"] * len(ids))

    def disable_torque(self, ids) -> None:
        self._sync_write(ids, REG_TORQUE_ENABLE, [b"\x00"] * len(ids))

    def set_goal_time(self, ids, goal_time) -> None:
        self._sync_write(ids, REG_GOAL_TIME, [struct.pack("<H", int(t)) for t in goal_time])


def open_bus(port: str, baudrate: int = 1000000) -> FeetechIO:
    timeout_ms = float(os.environ.get("JERO_BUS_TIMEOUT_MS", "25"))
    return FeetechIO(SerialTransport(port), timeout_s=timeout_ms / 1000.0)


def install() -> None:
    """Make every HWI built after this use FeetechIO (call before missing_servos.install)."""
    import mini_bdx_runtime.rustypot_position_hwi as hwi_mod

    class _Bus:  # stands in for the rustypot module inside hwi_mod only
        @staticmethod
        def feetech(port, baudrate):
            return open_bus(port, baudrate)

    hwi_mod.rustypot = _Bus

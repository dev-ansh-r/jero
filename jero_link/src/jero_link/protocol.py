"""Jero link wire protocol: one UDP datagram = one command.

Frame layout (all big-endian)::

    magic   3 bytes   b"JR1"
    flags   1 byte    bit0 = signed
    mac    32 bytes   HMAC-SHA256(key, body) when signed, zeros otherwise
    body    N bytes   UTF-8 JSON (see ``encode``)

The robot keeps executing the last command until ``ttl_ms`` expires, then
falls back to zero velocity. Senders therefore stream at 10-20 Hz.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import math
import time
from dataclasses import dataclass, field, replace
from typing import Optional, Tuple

MAGIC = b"JR1"
VERSION = 1
DEFAULT_PORT = 5005
MAX_DATAGRAM = 1024
_MAC_LEN = 32
_HEADER_LEN = len(MAGIC) + 1 + _MAC_LEN
_FLAG_SIGNED = 0x01

# Same ranges the upstream Xbox mapping uses (mini_bdx_runtime/xbox_controller.py).
LIMITS = {
    "vx": (-0.15, 0.15),  # m/s, forward +
    "vy": (-0.2, 0.2),  # m/s, left +
    "wz": (-1.0, 1.0),  # rad/s, counter-clockwise +
    "neck_pitch": (-0.34, 1.1),  # rad
    "head_pitch": (-0.78, 0.3),  # rad
    "head_yaw": (-0.5, 0.5),  # rad
    "head_roll": (-0.5, 0.5),  # rad
}
AXES = tuple(LIMITS)  # order == RLWalk.last_commands order
BUTTONS = ("A", "B", "X", "Y", "LB", "RB", "dpad_up", "dpad_down")
TTL_MS_RANGE = (50, 2000)


class ProtocolError(ValueError):
    """Raised for any datagram that must be dropped."""


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


@dataclass(frozen=True)
class Command:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    neck_pitch: float = 0.0
    head_pitch: float = 0.0
    head_yaw: float = 0.0
    head_roll: float = 0.0
    buttons: Tuple[str, ...] = field(default_factory=tuple)
    left_trigger: float = 0.0
    right_trigger: float = 0.0
    estop: bool = False
    ttl_ms: int = 500

    def clamped(self) -> "Command":
        axes = {name: _clamp(getattr(self, name), *LIMITS[name]) for name in AXES}
        return replace(
            self,
            **axes,
            left_trigger=_clamp(self.left_trigger, 0.0, 1.0),
            right_trigger=_clamp(self.right_trigger, 0.0, 1.0),
            ttl_ms=int(_clamp(self.ttl_ms, *TTL_MS_RANGE)),
        )

    def as_commands(self) -> list:
        """The 7-vector RLWalk feeds to the policy: [vx, vy, wz, neck, head_pitch, yaw, roll]."""
        return [float(getattr(self, name)) for name in AXES]


@dataclass(frozen=True)
class Message:
    session: str
    seq: int
    sent_at: float
    command: Command


def encode(
    command: Command,
    seq: int,
    session: str,
    key: Optional[bytes] = None,
) -> bytes:
    cmd = command.clamped()
    body = json.dumps(
        {
            "v": VERSION,
            "sid": session,
            "seq": int(seq),
            "t": round(time.time(), 3),
            "cmd": {name: round(getattr(cmd, name), 4) for name in AXES},
            "btn": sorted(set(cmd.buttons)),
            "trig": [round(cmd.left_trigger, 3), round(cmd.right_trigger, 3)],
            "estop": bool(cmd.estop),
            "ttl": cmd.ttl_ms,
        },
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    if key:
        flags, mac = _FLAG_SIGNED, hmac.new(key, body, hashlib.sha256).digest()
    else:
        flags, mac = 0, bytes(_MAC_LEN)
    frame = MAGIC + bytes([flags]) + mac + body
    if len(frame) > MAX_DATAGRAM:
        raise ProtocolError(f"frame too large ({len(frame)} bytes)")
    return frame


def _finite(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{name} is not a number")
    value = float(value)
    if not math.isfinite(value):
        raise ProtocolError(f"{name} is not finite")
    return value


def decode(frame: bytes, key: Optional[bytes] = None) -> Message:
    """Parse and validate a frame. With ``key`` set, unsigned or badly signed frames are rejected."""
    if len(frame) > MAX_DATAGRAM:
        raise ProtocolError("frame too large")
    if len(frame) <= _HEADER_LEN or frame[: len(MAGIC)] != MAGIC:
        raise ProtocolError("bad magic")
    flags = frame[len(MAGIC)]
    mac = frame[len(MAGIC) + 1 : _HEADER_LEN]
    body = frame[_HEADER_LEN:]

    if key:
        if not flags & _FLAG_SIGNED:
            raise ProtocolError("unsigned frame rejected")
        expected = hmac.new(key, body, hashlib.sha256).digest()
        if not hmac.compare_digest(mac, expected):
            raise ProtocolError("bad signature")

    try:
        data = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"bad json: {exc}") from exc
    if not isinstance(data, dict) or data.get("v") != VERSION:
        raise ProtocolError("unsupported version")

    session = data.get("sid")
    seq = data.get("seq")
    if not isinstance(session, str) or not 0 < len(session) <= 64:
        raise ProtocolError("bad session id")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
        raise ProtocolError("bad seq")

    raw_cmd = data.get("cmd", {})
    if not isinstance(raw_cmd, dict) or set(raw_cmd) - set(AXES):
        raise ProtocolError("bad cmd")
    axes = {name: _finite(raw_cmd.get(name, 0.0), name) for name in AXES}

    buttons = data.get("btn", [])
    if not isinstance(buttons, list) or any(b not in BUTTONS for b in buttons):
        raise ProtocolError("bad buttons")

    trig = data.get("trig", [0.0, 0.0])
    if not isinstance(trig, list) or len(trig) != 2:
        raise ProtocolError("bad triggers")

    estop = data.get("estop", False)
    if not isinstance(estop, bool):
        raise ProtocolError("bad estop")

    command = Command(
        **axes,
        buttons=tuple(buttons),
        left_trigger=_finite(trig[0], "trig[0]"),
        right_trigger=_finite(trig[1], "trig[1]"),
        estop=estop,
        ttl_ms=int(_finite(data.get("ttl", 500), "ttl")),
    ).clamped()
    return Message(
        session=session,
        seq=seq,
        sent_at=_finite(data.get("t", 0.0), "t"),
        command=command,
    )


def load_key(path: Optional[str]) -> Optional[bytes]:
    """Read a hex key file written by tools/gen_link_key.sh. ``None`` -> unsigned mode."""
    if not path:
        return None
    with open(path, "r", encoding="ascii") as fh:
        text = fh.read().strip()
    try:
        key = bytes.fromhex(text)
    except ValueError as exc:
        raise ValueError(f"{path}: key must be hex") from exc
    if len(key) < 16:
        raise ValueError(f"{path}: key must be at least 16 bytes")
    return key

"""Brain-side client: streams the current command to the robot at a fixed rate."""

from __future__ import annotations

import logging
import secrets
import socket
import threading
from dataclasses import replace
from typing import Optional

from .protocol import BUTTONS, DEFAULT_PORT, Command, encode

log = logging.getLogger("jero_link.client")

# A press is repeated in this many consecutive frames so one lost datagram doesn't eat it.
PRESS_REPEAT = 2


class JeroClient:
    """Usage::

        with JeroClient("jero.local", key=load_key("link.key")) as duck:
            duck.drive(vx=0.1)
            ...
            duck.stop()
    """

    def __init__(
        self,
        host: str,
        port: int = DEFAULT_PORT,
        key: Optional[bytes] = None,
        rate_hz: float = 20.0,
        ttl_ms: int = 500,
    ):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")
        if 1000.0 / rate_hz >= ttl_ms:
            raise ValueError("ttl_ms must be longer than the send period")
        self.addr = (host, port)
        self.key = key
        self.period = 1.0 / rate_hz
        self.session = secrets.token_hex(8)
        self._seq = 0
        self._cmd = Command(ttl_ms=ttl_ms)
        self._pending_presses: dict = {}
        self._estop_frames = 0
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # -- command setters -------------------------------------------------
    def drive(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> None:
        with self._lock:
            self._cmd = replace(self._cmd, vx=vx, vy=vy, wz=wz)

    def head(self, neck_pitch=None, head_pitch=None, head_yaw=None, head_roll=None) -> None:
        updates = {
            k: v
            for k, v in dict(
                neck_pitch=neck_pitch, head_pitch=head_pitch, head_yaw=head_yaw, head_roll=head_roll
            ).items()
            if v is not None
        }
        with self._lock:
            self._cmd = replace(self._cmd, **updates)

    def antennas(self, left: float, right: float) -> None:
        with self._lock:
            self._cmd = replace(self._cmd, left_trigger=left, right_trigger=right)

    def press(self, *buttons: str) -> None:
        """Momentary press. 'A' toggles pause/walk on the robot (upstream behaviour)."""
        for b in buttons:
            if b not in BUTTONS:
                raise ValueError(f"unknown button {b!r}; expected one of {BUTTONS}")
        with self._lock:
            for b in buttons:
                self._pending_presses[b] = PRESS_REPEAT
        self._wake.set()

    def stop(self) -> None:
        """Zero velocity (robot keeps stepping in place)."""
        self.drive(0.0, 0.0, 0.0)
        self._wake.set()

    def estop(self) -> None:
        """Pause the walk loop on the robot. Resume with press('A')."""
        with self._lock:
            self._cmd = replace(self._cmd, vx=0.0, vy=0.0, wz=0.0)
            self._estop_frames = PRESS_REPEAT
        self._wake.set()

    # -- transport -------------------------------------------------------
    def _next_frame(self) -> bytes:
        with self._lock:
            buttons = tuple(b for b, n in self._pending_presses.items() if n > 0)
            self._pending_presses = {b: n - 1 for b, n in self._pending_presses.items() if n > 1}
            estop = self._estop_frames > 0
            self._estop_frames = max(0, self._estop_frames - 1)
            cmd = replace(self._cmd, buttons=buttons, estop=estop)
            self._seq += 1
            seq = self._seq
        return encode(cmd, seq=seq, session=self.session, key=self.key)

    def send_once(self) -> None:
        self.sock.sendto(self._next_frame(), self.addr)

    def _loop(self) -> None:
        while self._running:
            try:
                self.send_once()
            except OSError as exc:  # network blips must not kill the stream
                log.warning("send failed: %s", exc)
            self._wake.wait(self.period)
            self._wake.clear()

    def start(self) -> "JeroClient":
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._loop, name="jero-link-tx", daemon=True)
            self._thread.start()
        return self

    def close(self) -> None:
        if self._running:
            self.stop()
            self.send_once()
            self._running = False
            self._wake.set()
            if self._thread:
                self._thread.join(timeout=1.0)
        self.sock.close()

    def __enter__(self) -> "JeroClient":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.close()

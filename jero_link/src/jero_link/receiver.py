"""Robot-side receiver: validates frames, tracks freshness, holds momentary buttons."""

from __future__ import annotations

import logging
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional

from .protocol import BUTTONS, DEFAULT_PORT, MAX_DATAGRAM, Command, Message, ProtocolError, decode

log = logging.getLogger("jero_link.receiver")

# How long a button stays "pressed" after the packet that carried it.
# Upstream Button.triggered debounces at 0.2 s, so 0.1 s gives exactly one trigger.
BUTTON_HOLD_S = 0.1


@dataclass
class LinkState:
    command: Command
    received_at: float
    session: str
    seq: int
    pressed_until: Dict[str, float] = field(default_factory=dict)

    def fresh(self, now: float) -> bool:
        return (now - self.received_at) * 1000.0 <= self.command.ttl_ms

    def pressed(self, now: float) -> Dict[str, bool]:
        return {b: self.pressed_until.get(b, 0.0) > now for b in BUTTONS}


class Receiver:
    """Accepts frames (from a socket or directly via ``handle``) and exposes the latest state.

    Session rule: the most recent session id wins; within a session, seq must increase.
    """

    def __init__(
        self,
        key: Optional[bytes] = None,
        on_estop: Optional[Callable[[], None]] = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.key = key
        self.on_estop = on_estop
        self.clock = clock
        self._lock = threading.Lock()
        self._state: Optional[LinkState] = None
        self.dropped = 0
        self.accepted = 0

    def handle(self, frame: bytes) -> bool:
        try:
            msg = decode(frame, self.key)
        except ProtocolError as exc:
            self.dropped += 1
            log.debug("dropped frame: %s", exc)
            return False
        return self._accept(msg)

    def _accept(self, msg: Message) -> bool:
        now = self.clock()
        with self._lock:
            prev = self._state
            if prev is not None and prev.session == msg.session and msg.seq <= prev.seq:
                self.dropped += 1
                return False
            pressed_until = dict(prev.pressed_until) if prev and prev.session == msg.session else {}
            for b in msg.command.buttons:
                pressed_until[b] = now + BUTTON_HOLD_S
            self._state = LinkState(msg.command, now, msg.session, msg.seq, pressed_until)
            self.accepted += 1
        if msg.command.estop and self.on_estop is not None:
            self.on_estop()
        return True

    def snapshot(self):
        """Returns (command or None if stale/absent, pressed-buttons dict)."""
        now = self.clock()
        with self._lock:
            state = self._state
            if state is None:
                return None, {b: False for b in BUTTONS}
            return (state.command if state.fresh(now) else None), state.pressed(now)


class UdpReceiver(Receiver):
    """Receiver bound to a UDP port, serviced by a daemon thread."""

    def __init__(self, bind: str = "0.0.0.0", port: int = DEFAULT_PORT, **kwargs):
        super().__init__(**kwargs)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.sock.settimeout(0.5)
        self.port = self.sock.getsockname()[1]
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="jero-link-rx", daemon=True)
        self._thread.start()
        log.info("jero link listening on %s:%d (%s)", bind, self.port, "signed" if self.key else "UNSIGNED")

    def _loop(self):
        while self._running:
            try:
                frame, _addr = self.sock.recvfrom(MAX_DATAGRAM + 1)
            except socket.timeout:
                continue
            except OSError:
                break
            self.handle(frame)

    def close(self):
        self._running = False
        self.sock.close()
        self._thread.join(timeout=1.0)

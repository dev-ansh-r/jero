"""Keep a servo-bus hiccup from killing the walk.

rustypot reports a corrupted or late reply as a Rust panic ("assertion failed:
self.is_input_buffer_empty"). pyo3 raises that as ``PanicException``, a ``BaseException``, so
upstream HWI's ``except Exception`` can't catch it and the whole walk dies. A panic can also
poison the port's lock inside rustypot, so the port object is no good afterwards.

``GuardedIO`` wraps the rustypot IO: on a panic it drops the port, opens a fresh one and raises
an ordinary ``BusError``. Upstream HWI then returns None for that read and the walk loop skips
one step (``if obs is None: continue``) instead of crashing.

``install()`` patches upstream's HWI module before RLWalk is built; nothing upstream is edited.
"""

from __future__ import annotations

import gc
import logging
import time

log = logging.getLogger("jero.bus")


class BusError(IOError):
    pass


class GuardedIO:
    def __init__(self, factory, port: str, baudrate: int):
        self._factory = factory
        self._port = port
        self._baudrate = baudrate
        self.panics = 0
        self._last_log = 0.0
        self._io = factory(port, baudrate)

    def _reopen(self) -> None:
        self._io = None
        gc.collect()  # rustypot keeps the port open exclusively until the old object is freed
        try:
            self._io = self._factory(self._port, self._baudrate)
        except Exception as exc:  # noqa: BLE001  retried on the next call
            log.error("servo bus: reopening %s failed: %s", self._port, exc)

    def __getattr__(self, name):
        if self._io is None:
            self._reopen()
            if self._io is None:
                raise BusError(f"servo bus {self._port} is not open")
        attr = getattr(self._io, name)
        if not callable(attr):
            return attr

        def call(*args, **kwargs):
            try:
                return attr(*args, **kwargs)
            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception:
                raise  # rustypot timeouts etc.: upstream HWI already handles these
            except BaseException as exc:  # pyo3 PanicException
                self.panics += 1
                now = time.monotonic()
                if now - self._last_log > 1.0:
                    log.warning("servo bus panic in %s (#%d): %s; reopening port", name, self.panics, exc)
                    self._last_log = now
                self._reopen()
                raise BusError(f"servo bus error in {name}: {exc}") from exc

        return call


def install() -> None:
    """Make every HWI built after this talk through GuardedIO (call before missing_servos.install)."""
    import mini_bdx_runtime.rustypot_position_hwi as hwi_mod

    real_feetech = hwi_mod.rustypot.feetech

    class _Rustypot:  # stands in for the rustypot module inside hwi_mod only
        @staticmethod
        def feetech(port, baudrate):
            return GuardedIO(real_feetech, port, baudrate)

    hwi_mod.rustypot = _Rustypot

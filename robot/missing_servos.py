"""Bench mode: run the walk with some servos absent (e.g. the head chain not wired yet).

``MissingServoIO`` wraps the rustypot Feetech IO used by upstream ``rustypot_position_hwi.HWI``:
- writes (goal position, kps, kds, torque on/off) skip the missing IDs;
- reads return the real values for present IDs and stand-ins for missing ones: the last goal
  position written to that ID (perfect tracking) and 0 for velocity / anything else.

So the policy sees the missing joints exactly where it commanded them. That is NOT what an
unpowered joint does, so use this on a stand only, never for floor walking.

``install(missing_ids)`` patches upstream's HWI module before RLWalk is built (RLWalk turns the
servos on inside its constructor). Nothing in upstream is edited.
"""

from __future__ import annotations

import logging
from collections.abc import Iterable

log = logging.getLogger("jero.missing_servos")


class MissingServoIO:
    def __init__(self, io, missing: Iterable[int]):
        self._io = io
        self.missing = frozenset(int(i) for i in missing)
        self.goal: dict[int, float] = {}

    def _split(self, ids, values=None):
        keep = [k for k, i in enumerate(ids) if int(i) not in self.missing]
        kept_ids = [ids[k] for k in keep]
        kept_values = None if values is None else [values[k] for k in keep]
        return keep, kept_ids, kept_values

    def _read(self, name: str, ids, fake):
        keep, kept_ids, _ = self._split(ids)
        real = getattr(self._io, name)(kept_ids) if kept_ids else []
        out = [fake(int(i)) for i in ids]
        for k, v in zip(keep, real):
            out[k] = v
        return out

    def read_present_position(self, ids):
        return self._read("read_present_position", ids, lambda i: self.goal.get(i, 0.0))

    def read_present_velocity(self, ids):
        return self._read("read_present_velocity", ids, lambda i: 0.0)

    def write_goal_position(self, ids, values):
        for i, v in zip(ids, values):
            if int(i) in self.missing:
                self.goal[int(i)] = float(v)
        return self._write("write_goal_position", ids, values)

    def _write(self, name: str, ids, values=None):
        _, kept_ids, kept_values = self._split(ids, values)
        if not kept_ids:
            return None
        fn = getattr(self._io, name)
        return fn(kept_ids) if values is None else fn(kept_ids, kept_values)

    def __getattr__(self, name):
        """Any other rustypot call: reads get stand-ins, writes skip missing IDs."""
        attr = getattr(self._io, name)
        if not callable(attr):
            return attr

        def call(ids, *rest):
            if name.startswith("read_"):
                return self._read(name, ids, lambda i: 0.0)
            return self._write(name, ids, rest[0] if rest else None)

        return call


def parse_ids(spec: str) -> list[int]:
    return [int(x) for x in spec.replace(" ", "").split(",") if x]


def install(missing: Iterable[int]) -> None:
    """Make every HWI built after this talk through MissingServoIO."""
    import mini_bdx_runtime.rustypot_position_hwi as hwi_mod

    real_feetech = hwi_mod.rustypot.feetech
    missing = sorted({int(i) for i in missing})

    class _Rustypot:  # stands in for the rustypot module inside hwi_mod only
        @staticmethod
        def feetech(port, baudrate):
            return MissingServoIO(real_feetech(port, baudrate), missing)

    hwi_mod.rustypot = _Rustypot
    log.warning("BENCH MODE: servos %s treated as absent (stand only, never on the floor)", missing)

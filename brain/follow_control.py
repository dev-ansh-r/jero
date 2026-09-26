"""Follow-a-person control law: normalised bbox -> (vx, wz). No I/O, unit-tested."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jero_link" / "src"))

from jero_link import LIMITS  # noqa: E402


@dataclass
class Target:
    cx: float  # horizontal centre, normalised 0..1 (0 = left edge)
    area: float  # bbox area / frame area, 0..1 (proxy for distance)


@dataclass
class FollowConfig:
    target_area: float = 0.12  # stop when the person fills ~12 % of the frame
    area_deadband: float = 0.03
    center_deadband: float = 0.06
    k_turn: float = 1.6  # rad/s per unit of normalised horizontal error
    k_forward: float = 0.8  # m/s per unit of area error
    max_vx: float = 0.10  # stay below the policy's 0.15 limit for a crowd
    min_vx: float = -0.05  # back off slowly when someone steps too close
    max_wz: float = 0.6


class FollowController:
    def __init__(self, cfg: Optional[FollowConfig] = None):
        self.cfg = cfg or FollowConfig()

    def update(self, target: Optional[Target]) -> Tuple[float, float]:
        """Returns (vx, wz)."""
        c = self.cfg
        if target is None:
            return 0.0, 0.0  # lost or brief dropout: stand still rather than guess

        err_x = 0.5 - target.cx  # + => target is left => turn left (+wz)
        wz = 0.0 if abs(err_x) < c.center_deadband else c.k_turn * err_x

        err_a = c.target_area - target.area  # + => too far => walk forward
        vx = 0.0 if abs(err_a) < c.area_deadband else c.k_forward * err_a
        if abs(err_x) > 0.25:  # turn in place first when the target is far off-centre
            vx = 0.0

        vx = max(max(LIMITS["vx"][0], c.min_vx), min(c.max_vx, vx))
        wz = max(-c.max_wz, min(c.max_wz, wz))
        return round(vx, 3), round(wz, 3)

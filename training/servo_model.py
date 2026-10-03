"""Servo profiles for the MuJoCo model: STS3215 (upstream, 7.4 V) or STS3235 (Jero, 12 V / 3S).

Upstream's servo class in ``open_duck_mini_v2*.xml`` was identified on a real 7.4 V STS3215.
We don't have a test rig for the STS3235 yet, so its values are that identified model scaled by
datasheet ratios (stall torque, no-load speed, mass). Replace them with measured values once a
servo has been characterised.

Datasheet values (Waveshare / Feetech):
  STS3215 7.4 V: 19.5 kg.cm stall, 0.192 s/60 deg no-load
  STS3235 12 V:  30.0 kg.cm stall, 0.222 s/60 deg no-load, 70.5 g, 1:345 steel gears

``apply(xml_text, profile)`` rewrites one upstream robot XML; nothing in ``upstream/`` is edited.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

UPSTREAM_MAX_MOTOR_VELOCITY = 5.24  # rad/s, upstream joystick.py / mujoco_infer.py
SERVO_CLASS = "sts3215"  # class name in the upstream XML (kept even for the STS3235)
ROBOT_XMLS = ("open_duck_mini_v2.xml", "open_duck_mini_v2_backlash.xml")
TRUNK = "trunk_assembly"


@dataclass(frozen=True)
class ServoProfile:
    name: str
    stall_kgcm: float
    s_per_60deg: float
    mass_kg: float
    extra_trunk_kg: float = 0.0  # e.g. a 3S pack + buck converter instead of 2S


STS3215 = ServoProfile("sts3215", stall_kgcm=19.5, s_per_60deg=0.192, mass_kg=0.055)
# extra_trunk_kg: one more 18650 (~46 g) plus holder and 12 V -> 5 V buck. Weigh it and update.
STS3235 = ServoProfile("sts3235", stall_kgcm=30.0, s_per_60deg=0.222, mass_kg=0.0705, extra_trunk_kg=0.06)
PROFILES = {p.name: p for p in (STS3215, STS3235)}

# Servo case size (STS3215 outline, 45.2 x 24.7 x 35 mm): inertia of the added mass.
_CASE = (0.0452, 0.0247, 0.035)


def ratios(profile: ServoProfile, base: ServoProfile = STS3215) -> dict[str, float]:
    return {
        "torque": profile.stall_kgcm / base.stall_kgcm,
        "speed": base.s_per_60deg / profile.s_per_60deg,
        "dmass_kg": profile.mass_kg - base.mass_kg,
    }


def max_motor_velocity(profile: ServoProfile) -> float:
    return UPSTREAM_MAX_MOTOR_VELOCITY * ratios(profile)["speed"]


def _vec(s: str) -> np.ndarray:
    return np.array([float(x) for x in s.split()])


def _fmt(v) -> str:
    return " ".join(f"{x:.9g}" for x in v)


def _full_to_mat(fi: np.ndarray) -> np.ndarray:
    ixx, iyy, izz, ixy, ixz, iyz = fi
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])


def _mat_to_full(m: np.ndarray) -> np.ndarray:
    return np.array([m[0, 0], m[1, 1], m[2, 2], m[0, 1], m[0, 2], m[1, 2]])


def _shift(r: np.ndarray) -> np.ndarray:
    """Parallel-axis term for a unit mass at offset r."""
    return np.dot(r, r) * np.eye(3) - np.outer(r, r)


def add_point_mass(inertial: ET.Element, dm: float, at: np.ndarray, box=_CASE) -> None:
    """Add mass ``dm`` (a servo-sized box) at ``at`` (body frame) to an ``<inertial fullinertia=...>``."""
    m = float(inertial.get("mass"))
    c = _vec(inertial.get("pos"))
    inertia = _full_to_mat(_vec(inertial.get("fullinertia")))
    m2 = m + dm
    c2 = (m * c + dm * at) / m2
    a, b, h = box
    own = dm / 12.0 * np.diag([b * b + h * h, a * a + h * h, a * a + b * b])
    inertia2 = inertia + m * _shift(c - c2) + own + dm * _shift(at - c2)
    inertial.set("mass", f"{m2:.9g}")
    inertial.set("pos", _fmt(c2))
    inertial.set("fullinertia", _fmt(_mat_to_full(inertia2)))


def apply(xml_text: str, profile: ServoProfile) -> str:
    """Return the robot XML with ``profile`` in place of the upstream STS3215 model."""
    if profile == STS3215:
        return xml_text
    r = ratios(profile)
    root = ET.fromstring(xml_text)

    pos = root.find(f".//default[@class='{SERVO_CLASS}']/position")
    if pos is None:
        raise ValueError(f"no <default class='{SERVO_CLASS}'><position> in this XML")
    kp = float(pos.get("kp"))
    lo, hi = _vec(pos.get("forcerange"))
    # Same register P-gain on the robot, stronger motor -> stiffer joint: scale kp with torque.
    pos.set("kp", f"{kp * r['torque']:.6g}")
    pos.set("forcerange", _fmt((lo * r["torque"], hi * r["torque"])))

    # Each servo sits at its joint, fixed to the parent link; its mass is folded into that
    # link's CAD inertial. Add the mass difference there.
    parent = {child: p for p in root.iter() for child in p}
    joints = [j for j in root.iter("joint") if j.get("class") == SERVO_CLASS]
    for j in joints:
        body = parent[j]
        link = parent[body]
        if link.tag != "body":
            raise ValueError(f"joint {j.get('name')}: parent of {body.get('name')} is not a body")
        anchor = _vec(body.get("pos", "0 0 0")) + _vec(j.get("pos", "0 0 0"))  # j.pos is ~0 here
        add_point_mass(link.find("inertial"), r["dmass_kg"], anchor)

    if profile.extra_trunk_kg:
        trunk = root.find(f".//body[@name='{TRUNK}']/inertial")
        add_point_mass(trunk, profile.extra_trunk_kg, _vec(trunk.get("pos")), box=(0.07, 0.04, 0.02))

    return ET.tostring(root, encoding="unicode")


def add_payload(xml_text: str, kg: float, offset=(0.0, 0.0, 0.0), size=(0.10, 0.08, 0.02)) -> str:
    """Add a rigid ``kg`` payload to the trunk, ``offset`` metres from the trunk's centre of mass
    (x forward, y left, z up). Default size is a board-sized box."""
    root = ET.fromstring(xml_text)
    trunk = root.find(f".//body[@name='{TRUNK}']/inertial")
    add_point_mass(trunk, kg, _vec(trunk.get("pos")) + np.asarray(offset, dtype=float), box=size)
    return ET.tostring(root, encoding="unicode")


def describe(profile: ServoProfile) -> str:
    r = ratios(profile)
    return (
        f"{profile.name}: torque x{r['torque']:.3f}, speed x{r['speed']:.3f} "
        f"(max_motor_velocity {max_motor_velocity(profile):.2f} rad/s), "
        f"+{r['dmass_kg'] * 1000:.1f} g per servo, +{profile.extra_trunk_kg * 1000:.0f} g trunk"
    )

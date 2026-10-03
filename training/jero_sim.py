#!/usr/bin/env python3
"""Run upstream Open_Duck_Playground with Jero's servo profile (training/servo_model.py).

Upstream loads its robot XML from an in-memory assets dict; this swaps in the rewritten XML
and the matching joint speed limit, then runs the upstream entry point unchanged.

    python training/jero_sim.py check                         # print the model the sim will use
    python training/jero_sim.py train --task flat_terrain_backlash --num_timesteps 50000000 ...
    python training/jero_sim.py view -o policy.onnx           # upstream MuJoCo viewer
    python training/jero_sim.py gate -o policy.onnx [--push]  # headless 60 s walk test (gate 1)

Profile: --servo sts3235 (default) or sts3215 (upstream, for A/B), or env JERO_SERVO.
Runs from upstream/Open_Duck_Playground (its scripts use relative paths); this script cds there.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PG = HERE.parent / "upstream" / "Open_Duck_Playground"
SCENE = "playground/open_duck_mini_v2/xmls/scene_flat_terrain_backlash.xml"
REFERENCE = "playground/open_duck_mini_v2/data/polynomial_coefficients.pkl"

sys.path.insert(0, str(HERE))
import servo_model

# Gate 1 command schedule: (seconds, [vx, vy, wz]), limits as in upstream joystick.py.
SCHEDULE = [
    (5, [0.0, 0.0, 0.0]),
    (12, [0.15, 0.0, 0.0]),
    (8, [-0.15, 0.0, 0.0]),
    (8, [0.0, 0.2, 0.0]),
    (8, [0.0, -0.2, 0.0]),
    (7, [0.0, 0.0, 1.0]),
    (7, [0.0, 0.0, -1.0]),
    (5, [0.0, 0.0, 0.0]),
]
FALL_UPRIGHT = 0.5  # trunk z-axis . world z below this (tilt > 60 deg) counts as a fall


def install(profile: servo_model.ServoProfile, payload_kg: float = 0.0, payload_at=(0.0, 0.0, 0.0)) -> None:
    """Patch upstream modules in place so every model they load uses ``profile`` (+ payload)."""
    os.chdir(PG)
    sys.path.insert(0, str(PG))
    from playground.open_duck_mini_v2 import base, joystick

    upstream_get_assets = base.get_assets

    def get_assets():
        assets = upstream_get_assets()
        for name in servo_model.ROBOT_XMLS:
            xml = servo_model.apply(assets[name].decode(), profile)
            if payload_kg:
                xml = servo_model.add_payload(xml, payload_kg, payload_at)
            assets[name] = xml.encode()
        return assets

    base.get_assets = get_assets  # base.py and mujoco_infer_base.py look it up at call time

    vmax = servo_model.max_motor_velocity(profile)
    upstream_default_config = joystick.default_config

    def default_config():
        cfg = upstream_default_config()
        cfg.max_motor_velocity = vmax
        return cfg

    joystick.default_config = default_config
    # Joystick(config=default_config()) was evaluated at import: patch that instance too.
    for default in joystick.Joystick.__init__.__defaults__:
        if hasattr(default, "max_motor_velocity"):
            default.max_motor_velocity = vmax


def cmd_check(profile, _args) -> int:
    import mujoco
    import numpy as np
    from playground.open_duck_mini_v2 import base, joystick

    print(servo_model.describe(profile))
    for scene in ("scene_flat_terrain.xml", "scene_flat_terrain_backlash.xml"):
        text = (PG / "playground/open_duck_mini_v2/xmls" / scene).read_text()
        m = mujoco.MjModel.from_xml_string(text, assets=base.get_assets())
        dofs = [m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in range(m.nu)]
        print(
            f"  {scene:34s} kp={sorted(set(np.round(m.actuator_gainprm[:, 0], 3)))} "
            f"force={sorted(set(np.round(m.actuator_forcerange[:, 1], 3)))} "
            f"armature={sorted(set(np.round(m.dof_armature[dofs], 4)))} mass={m.body_mass.sum():.3f} kg"
        )
    print(f"  joystick max_motor_velocity={joystick.default_config().max_motor_velocity:.3f} rad/s")
    return 0


def cmd_train(_profile, args) -> int:
    from playground.open_duck_mini_v2 import runner

    sys.argv = ["runner.py", *args.rest]
    runner.main()
    return 0


def _infer(profile, onnx: str):
    from playground.open_duck_mini_v2.mujoco_infer import MjInfer

    sim = MjInfer(SCENE, REFERENCE, onnx, standing=False)
    sim.max_motor_velocity = servo_model.max_motor_velocity(profile)
    return sim


def cmd_view(profile, args) -> int:
    _infer(profile, args.onnx).run()
    return 0


def cmd_gate(profile, args) -> int:
    import mujoco
    import numpy as np
    from playground.open_duck_mini_v2 import mujoco_infer

    sim = _infer(profile, args.onnx)
    m, d = sim.model, sim.data
    trunk = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, servo_model.TRUNK)
    ctrl_dt = sim.sim_dt * sim.decimation
    period = sim.PRM.nb_steps_in_period
    rng = np.random.default_rng(args.seed)
    push_every = round(args.push_interval / ctrl_dt)

    t_end = np.cumsum([s for s, _ in SCHEDULE])
    n_ctrl = round(float(t_end[-1]) / ctrl_dt)
    seg_v = {i: [] for i in range(len(SCHEDULE))}
    min_upright, pushes = 1.0, 0
    torque_use = []  # |actuator force| / force limit, per control step
    limit = m.actuator_forcerange[:, 1]
    for k in range(n_ctrl):
        t = k * ctrl_dt
        seg = int(np.searchsorted(t_end, t, side="right"))
        sim.commands[:3] = SCHEDULE[seg][1]
        if args.push and k > 0 and k % push_every == 0:
            theta = rng.uniform(0, 2 * np.pi)
            d.qvel[sim._floating_base_qvel_addr : sim._floating_base_qvel_addr + 2] = args.push_mag * np.array(
                [np.cos(theta), np.sin(theta)]
            )
            pushes += 1

        sim.imitation_i = (sim.imitation_i + sim.phase_frequency_factor) % period
        a = sim.imitation_i / period * 2 * np.pi
        sim.imitation_phase = np.array([np.cos(a), np.sin(a)])
        action = sim.policy.infer(sim.get_obs(d, sim.commands))
        sim.last_last_last_action = sim.last_last_action.copy()
        sim.last_last_action = sim.last_action.copy()
        sim.last_action = action.copy()
        targets = sim.default_actuator + action * sim.action_scale
        if mujoco_infer.USE_MOTOR_SPEED_LIMITS:
            step = sim.max_motor_velocity * ctrl_dt
            targets = np.clip(targets, sim.prev_motor_targets - step, sim.prev_motor_targets + step)
            sim.prev_motor_targets = targets.copy()
        sim.motor_targets = targets
        d.ctrl = targets.copy()
        for _ in range(sim.decimation):
            mujoco.mj_step(m, d)

        torque_use.append(np.abs(d.actuator_force) / limit)
        upright = d.xmat[trunk].reshape(3, 3)[2, 2]
        min_upright = min(min_upright, upright)
        if upright < FALL_UPRIGHT:
            print(f"FAIL: fell at t={t:.1f}s (segment {seg}, cmd {SCHEDULE[seg][1]})")
            return 1
        # body-frame planar velocity + yaw rate, for command tracking
        rot = d.xmat[trunk].reshape(3, 3)
        v = rot.T @ d.cvel[trunk][3:]  # cvel is [ang, lin] in world orientation at the body com
        w = rot.T @ d.cvel[trunk][:3]
        seg_v[seg].append([v[0], v[1], w[2]])

    print(f"{profile.name}: no fall in {t_end[-1]} s ({pushes} pushes of {args.push_mag} m/s), "
          f"min upright {min_upright:.2f}")
    print("  segment  cmd [vx vy wz]          mean [vx vy wz]")
    for i, (_, c) in enumerate(SCHEDULE):
        mean = np.mean(seg_v[i][len(seg_v[i]) // 3 :], axis=0)  # skip the transient
        print(f"  {i:7d}  {c!s:24s} [{mean[0]:+.3f} {mean[1]:+.3f} {mean[2]:+.3f}]")
    use = np.array(torque_use)
    p95 = np.percentile(use, 95, axis=0)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
    top = np.argsort(p95)[::-1][:4]
    print("  torque use (p95 / max of limit): " + ", ".join(
        f"{names[i]} {p95[i]:.0%}/{use[:, i].max():.0%}" for i in top))
    print("PASS")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--servo", choices=sorted(servo_model.PROFILES), default=os.environ.get("JERO_SERVO", "sts3235"))
    p.add_argument("--payload-kg", type=float, default=0.0, help="extra rigid mass on the trunk (kg)")
    p.add_argument(
        "--payload-at", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("X", "Y", "Z"),
        help="payload offset from the trunk centre of mass, metres (x forward, y left, z up)",
    )
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    t = sub.add_parser("train", help="upstream runner.py; remaining args are passed through")
    t.add_argument("rest", nargs=argparse.REMAINDER)
    v = sub.add_parser("view")
    v.add_argument("-o", "--onnx", required=True)
    g = sub.add_parser("gate")
    g.add_argument("-o", "--onnx", required=True)
    g.add_argument("--push", action="store_true", help="kick the base like training does")
    g.add_argument("--push-mag", type=float, default=0.5, help="m/s (training samples 0.1-1.0)")
    g.add_argument("--push-interval", type=float, default=6.0, help="s")
    g.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    profile = servo_model.PROFILES[args.servo]
    if getattr(args, "onnx", None):
        args.onnx = str(Path(args.onnx).resolve())  # before install() changes directory
    print(f"servo profile: {servo_model.describe(profile)}", file=sys.stderr)
    if args.payload_kg:
        print(f"payload: {args.payload_kg * 1000:.0f} g at {list(args.payload_at)} m from trunk CoM", file=sys.stderr)
    install(profile, args.payload_kg, args.payload_at)
    return {"check": cmd_check, "train": cmd_train, "view": cmd_view, "gate": cmd_gate}[args.cmd](profile, args)


if __name__ == "__main__":
    sys.exit(main())

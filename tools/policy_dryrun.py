#!/usr/bin/env python3
"""Run the walking policy on the Pi without servos: live IMU in, actions out, timing measured.

Proves the Pi + IMU + onnxruntime + policy chain before the servo bus exists. Builds the
observation exactly like upstream ``scripts/v2_rl_walk_mujoco.py`` (RLWalk.get_obs), except that
the joints are assumed to follow the targets perfectly and both feet are on the ground.

    workon open-duck-mini-runtime
    python ~/Jero/tools/policy_dryrun.py                      # MPU6050/6500/9250 via ~/.config/jero/imu.json
    python ~/Jero/tools/policy_dryrun.py --imu none           # no IMU wired: robot "upright and still"
    python ~/Jero/tools/policy_dryrun.py --vx 0.1 --seconds 30

Tilt the IMU while it runs: the actions should change. Pass: no deadline misses at 50 Hz, no NaN.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

HOME = Path.home()
REPO = Path(__file__).resolve().parents[1]
RUNTIMES = [HOME / "Open_Duck_Mini_Runtime", REPO / "upstream" / "Open_Duck_Mini_Runtime"]  # Pi image, submodule
POLICIES = [HOME / "BEST_WALK_ONNX_2.onnx", REPO / "upstream" / "Open_Duck_Mini" / "BEST_WALK_ONNX_2.onnx"]

# Joint order and init pose: upstream mini_bdx_runtime/rustypot_position_hwi.py (HWI.joints / init_pos).
INIT_POS = np.array(
    [0.002, 0.053, -0.63, 1.368, -0.784, 0.0, 0.0, 0.0, 0.0, -0.003, -0.065, 0.635, 1.379, -0.796]
)
JOINTS = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]  # fmt: skip
ACTION_SCALE = 0.25  # upstream default


class StillImu:
    """Stand-in when no IMU is wired: upright, not rotating."""

    def get_data(self):
        return {"gyro": np.zeros(3), "accelero": np.array([0.0, 0.0, 9.81])}


def make_imu(kind: str, freq: int):
    if kind == "none":
        return StillImu()
    sys.path.insert(0, str(REPO / "robot"))
    import imu_mpu6050

    if not imu_mpu6050.CONFIG_PATH.is_file():
        sys.exit(f"{imu_mpu6050.CONFIG_PATH} missing: run tools/imu_check.py first (or use --imu none)")
    print("IMU: keep it still for 2 s (gyro bias)...")
    return imu_mpu6050.Imu(freq)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--onnx", default=next((str(c) for c in POLICIES if c.is_file()), str(POLICIES[0])))
    p.add_argument("--imu", choices=("mpu", "none"), default="mpu")
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--vx", type=float, default=0.0, help="forward command, m/s (upstream range +/-0.15)")
    p.add_argument("--freq", type=int, default=50)
    args = p.parse_args()

    runtime = next((r for r in RUNTIMES if (r / "scripts" / "polynomial_coefficients.pkl").is_file()), None)
    if runtime is None:
        sys.exit("Open_Duck_Mini_Runtime not found (expected ~/Open_Duck_Mini_Runtime on the Duck image)")
    sys.path.append(str(runtime / "mini_bdx_runtime"))  # no-op on the Pi, where the package is installed
    from mini_bdx_runtime.onnx_infer import OnnxInfer
    from mini_bdx_runtime.poly_reference_motion import PolyReferenceMotion

    policy = OnnxInfer(args.onnx, awd=True)
    prm = PolyReferenceMotion(str(runtime / "scripts" / "polynomial_coefficients.pkl"))
    imu = make_imu(args.imu, args.freq)

    dt = 1.0 / args.freq
    cmds = np.array([args.vx, 0, 0, 0, 0, 0, 0], dtype=float)
    targets = INIT_POS.copy()
    prev_targets = INIT_POS.copy()
    a1 = a2 = a3 = np.zeros(len(JOINTS))
    imitation_i = 0.0
    loop_ms, infer_ms, misses, n = [], [], 0, int(args.seconds * args.freq)

    print(f"policy {args.onnx}: running {args.seconds:.0f} s at {args.freq} Hz, cmd vx={args.vx}")
    next_t = time.perf_counter()
    for k in range(n):
        t0 = time.perf_counter()
        imu_data = imu.get_data()
        imitation_i = (imitation_i + 1) % prm.nb_steps_in_period
        ph = imitation_i / prm.nb_steps_in_period * 2 * np.pi
        dof_pos = targets  # assume perfect tracking
        dof_vel = (targets - prev_targets) / dt
        obs = np.concatenate(
            [
                imu_data["gyro"],
                imu_data["accelero"],
                cmds,
                dof_pos - INIT_POS,
                dof_vel * 0.05,
                a1,
                a2,
                a3,
                targets,
                [1.0, 1.0],  # both feet on the ground
                [np.cos(ph), np.sin(ph)],
            ]
        )
        t1 = time.perf_counter()
        action = np.asarray(policy.infer(obs))
        infer_ms.append((time.perf_counter() - t1) * 1000)
        if not np.all(np.isfinite(action)):
            print(f"FAIL: non-finite action at step {k}")
            return 1
        a1, a2, a3 = action.copy(), a1, a2
        prev_targets, targets = targets, INIT_POS + action * ACTION_SCALE

        if k % (args.freq // 2) == 0:
            g, acc = np.round(imu_data["gyro"], 2), np.round(imu_data["accelero"], 2)
            print(
                f"t={k * dt:5.1f}s gyro={g} acc={acc} action[min,max]=[{action.min():+.2f},{action.max():+.2f}] "
                f"knees L/R={targets[3]:+.2f}/{targets[12]:+.2f} rad"
            )
        loop_ms.append((time.perf_counter() - t0) * 1000)
        next_t += dt
        sleep = next_t - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            misses += 1
            next_t = time.perf_counter()

    infer, loop = np.array(infer_ms), np.array(loop_ms)
    print(
        f"\nobs size {obs.size}, action size {action.size}\n"
        f"inference: mean {infer.mean():.2f} ms, p99 {np.percentile(infer, 99):.2f} ms, max {infer.max():.2f} ms\n"
        f"loop work: mean {loop.mean():.2f} ms, p99 {np.percentile(loop, 99):.2f} ms (budget {dt * 1000:.0f} ms)\n"
        f"deadline misses: {misses}/{n}"
    )
    ok = misses <= n * 0.01 and np.percentile(loop, 99) < dt * 1000 * 0.5
    print("PASS" if ok else "WARN: loop is slow; the real runtime also reads 14 servos each step")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

# Training (GPU server)

The pinned `BEST_WALK_ONNX_2.onnx` should walk a stock build, so retraining is **not on the critical path**
for 10 Oct. Set the pipeline up anyway, so it's ready if our masses differ or we want new behaviours.

## Setup

Requires an NVIDIA GPU with a CUDA 12 driver. The Playground pins `jax[cuda12]`.

```bash
git clone --recurse-submodules=upstream/Open_Duck_Playground --shallow-submodules https://github.com/dev-ansh-r/Jero
cd Jero && training/setup_server.sh        # installs uv, runs uv sync, asserts JAX sees the GPU
```

## Train

```bash
STEPS=50000000 training/train.sh          # smoke run: check the reward climbs and ONNX files appear
training/train.sh                          # upstream "current win": flat_terrain_backlash, 300M steps
```

- Output goes to `training/runs/<timestamp>_<task>/`, which holds the checkpoints, an `.onnx` per checkpoint, `train.log`
  and `run_info.txt` (the commits used and the GPU).
- Monitor with `cd upstream/Open_Duck_Playground && uv run tensorboard --logdir ../../training/runs`.
- Resume with `RESTORE=<checkpoint dir> training/train.sh`.
- The repo doesn't document training time; record the wall-clock time of the first full run here.

## Validate → deploy

```bash
training/eval_mujoco.sh training/runs/<run>/<ckpt>.onnx      # needs a display
training/deploy_policy.sh training/runs/<run>/<ckpt>.onnx bdxv2@jero.local
```

A policy only goes on the robot after it walks in MuJoCo, and the first on-robot run is done on the stand.

## When we'd actually retrain

- The measured part masses differ noticeably from the MJCF (see the weigh-in log in printing.md). Update
  `upstream/.../xmls` in a Jero-owned copy, not in the submodule.
- We add payload (Jetson onboard, a bigger battery).
- We want different command ranges or gaits (e.g., slower, steadier walking for crowds).

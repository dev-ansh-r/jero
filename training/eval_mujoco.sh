#!/usr/bin/env bash
# Watch a policy in the MuJoCo viewer before it touches hardware (needs a display: run on a laptop,
# or over ssh -X / VNC). Defaults to the pinned upstream policy.
#   training/eval_mujoco.sh [path/to/policy.onnx]
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PG="$REPO/upstream/Open_Duck_Playground"
ONNX="${1:-$REPO/upstream/Open_Duck_Mini/BEST_WALK_ONNX_2.onnx}"
ONNX="$(realpath "$ONNX")"
[ -f "$ONNX" ] || { echo "not found: $ONNX (tools/fetch_policy.sh downloads the upstream one)" >&2; exit 1; }
cd "$PG"
uv run playground/open_duck_mini_v2/mujoco_infer.py -o "$ONNX"

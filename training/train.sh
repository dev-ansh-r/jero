#!/usr/bin/env bash
# Train a walking policy with the upstream recipe ("current win" in Open_Duck_Playground README).
# ONNX files are exported automatically next to each checkpoint in the run directory.
#
#   training/train.sh                                   # 300M steps, flat_terrain_backlash
#   STEPS=50000000 training/train.sh                    # quick smoke run
#   RESTORE=runs/<run>/<ckpt> training/train.sh         # resume
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PG="$REPO/upstream/Open_Duck_Playground"

TASK="${TASK:-flat_terrain_backlash}"
STEPS="${STEPS:-300000000}"
ENV_NAME="${ENV_NAME:-joystick}"
RUN="${RUN:-$(date +%Y%m%d_%H%M%S)_${TASK}}"
OUT="$REPO/training/runs/$RUN"
mkdir -p "$OUT"

args=(--env "$ENV_NAME" --task "$TASK" --num_timesteps "$STEPS" --output_dir "$OUT")
[ -n "${RESTORE:-}" ] && args+=(--restore_checkpoint_path "$RESTORE")

{
  echo "jero_commit=$(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo none)"
  echo "playground_commit=$(git -C "$PG" rev-parse HEAD)"
  echo "task=$TASK steps=$STEPS env=$ENV_NAME started=$(date -Is)"
  nvidia-smi --query-gpu=name --format=csv,noheader
} > "$OUT/run_info.txt"

cd "$PG"
echo "run dir: $OUT   (tensorboard: cd $PG && uv run tensorboard --logdir $OUT)"
uv run playground/open_duck_mini_v2/runner.py "${args[@]}" 2>&1 | tee "$OUT/train.log"
echo "finished $(date -Is)" >> "$OUT/run_info.txt"
find "$OUT" -maxdepth 1 -name '*.onnx' -printf '%T@ %p\n' | sort -rn | head -3 | cut -d' ' -f2-

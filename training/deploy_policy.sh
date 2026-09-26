#!/usr/bin/env bash
# Copy a trained policy to the robot, keeping the previous one as a rollback.
#   training/deploy_policy.sh runs/<run>/<ckpt>.onnx bdxv2@jero.local
set -euo pipefail
ONNX="${1:?usage: deploy_policy.sh <policy.onnx> <user@host>}"
HOST="${2:?usage: deploy_policy.sh <policy.onnx> <user@host>}"
NAME="jero_$(date +%Y%m%d_%H%M%S).onnx"
scp "$ONNX" "$HOST:~/$NAME"
# shellcheck disable=SC2029  # $NAME is meant to expand locally
ssh "$HOST" "cd ~ && { [ -e jero_policy.onnx ] && cp -P jero_policy.onnx jero_policy.prev.onnx || true; } && ln -sfn $NAME jero_policy.onnx && ls -l jero_policy*.onnx"
echo "On the robot: python ~/Jero/robot/jero_walk.py --onnx_model_path ~/jero_policy.onnx"
echo "Rollback:     ssh $HOST 'ln -sfn \$(readlink ~/jero_policy.prev.onnx) ~/jero_policy.onnx'"

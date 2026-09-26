#!/usr/bin/env bash
# Download the pinned upstream walking policy without cloning the 170 MB Open_Duck_Mini repo.
#   tools/fetch_policy.sh [dest_dir]
set -euo pipefail
DEST="${1:-$HOME}"
SHA=b23317a485b3cec7d8417f352478778b3475173c
URL="https://raw.githubusercontent.com/apirrone/Open_Duck_Mini/$SHA/BEST_WALK_ONNX_2.onnx"
mkdir -p "$DEST"
curl -fL --retry 3 -o "$DEST/BEST_WALK_ONNX_2.onnx" "$URL"
ls -l "$DEST/BEST_WALK_ONNX_2.onnx"

#!/usr/bin/env bash
# One-time setup of a CUDA 12 GPU box for Open_Duck_Playground (MuJoCo MJX + JAX + Brax PPO).
#   git clone --recurse-submodules=upstream/Open_Duck_Playground https://github.com/dev-ansh-r/Jero
#   Jero/training/setup_server.sh
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PG="$REPO/upstream/Open_Duck_Playground"

command -v nvidia-smi >/dev/null || { echo "nvidia-smi not found: need an NVIDIA GPU + driver" >&2; exit 1; }
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv

if [ ! -f "$PG/pyproject.toml" ]; then
  git -C "$REPO" submodule update --init --depth 1 upstream/Open_Duck_Playground
fi

if ! command -v uv >/dev/null; then
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.local/bin:$PATH"
fi

cd "$PG"
uv sync
uv run python - <<'EOF'
import jax
devs = jax.devices()
print("JAX devices:", devs)
assert any(d.platform == "gpu" for d in devs), "JAX does not see the GPU (check CUDA 12 / driver)"
EOF
echo "OK: run training/train.sh"

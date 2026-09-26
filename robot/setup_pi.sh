#!/usr/bin/env bash
# Set up Jero on a Pi Zero 2 W that runs the pre-built Open Duck image (runtime v2 branch).
# Idempotent: re-running only fills in what is missing. Never overwrites duck_config.json or keys.
#
#   ssh bdxv2@bdxv2.local
#   git clone https://github.com/dev-ansh-r/Jero ~/Jero     # no submodules needed: the image has the runtime
#   ~/Jero/robot/setup_pi.sh [--replace-autostart]
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${VENV:-$HOME/.virtualenvs/open-duck-mini-runtime}"
RUNTIME="${JERO_RUNTIME_DIR:-$HOME/Open_Duck_Mini_Runtime}"
KEY_DIR="$HOME/.config/jero"
KEY_FILE="$KEY_DIR/link.key"
POLICY_URL="https://raw.githubusercontent.com/apirrone/Open_Duck_Mini/b23317a485b3cec7d8417f352478778b3475173c/BEST_WALK_ONNX_2.onnx"
REPLACE_AUTOSTART=0

for arg in "$@"; do
  case "$arg" in
    --replace-autostart) REPLACE_AUTOSTART=1 ;;
    -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

step() { printf '\n==> %s\n' "$*"; }
warn() { printf 'WARN: %s\n' "$*" >&2; }

[ "$(uname -m)" = "aarch64" ] || warn "not aarch64: this script targets the Pi Zero 2 W image"

step "Checking runtime + venv"
[ -f "$RUNTIME/scripts/v2_rl_walk_mujoco.py" ] || { echo "runtime not found at $RUNTIME (set JERO_RUNTIME_DIR)" >&2; exit 1; }
[ -x "$VENV/bin/python" ] || { echo "venv not found at $VENV (set VENV=...)" >&2; exit 1; }
echo "runtime: $RUNTIME"
echo "venv:    $VENV"

step "Installing jero_link into the runtime venv"
"$VENV/bin/pip" install --quiet -e "$REPO/jero_link"
"$VENV/bin/python" -c "import jero_link; print('jero_link', jero_link.__version__)"

step "Link key"
if [ -f "$KEY_FILE" ]; then
  echo "exists: $KEY_FILE"
else
  mkdir -p "$KEY_DIR" && chmod 700 "$KEY_DIR"
  "$REPO/tools/gen_link_key.sh" "$KEY_FILE"
  echo "Copy this key to the brain (Jetson/laptop) as ~/.config/jero/link.key:"
  cat "$KEY_FILE"
fi

step "duck_config.json"
if [ -f "$HOME/duck_config.json" ]; then
  echo "exists: $HOME/duck_config.json (not touched)"
else
  cp "$REPO/robot/duck_config.json" "$HOME/duck_config.json"
  echo "copied template -> $HOME/duck_config.json (run find_soft_offsets.py and fill joints_offsets)"
fi

step "Walk policy"
if [ -f "$HOME/BEST_WALK_ONNX_2.onnx" ]; then
  echo "exists: $HOME/BEST_WALK_ONNX_2.onnx"
elif [ -f "$REPO/upstream/Open_Duck_Mini/BEST_WALK_ONNX_2.onnx" ]; then
  cp "$REPO/upstream/Open_Duck_Mini/BEST_WALK_ONNX_2.onnx" "$HOME/"
else
  curl -fL --retry 3 -o "$HOME/BEST_WALK_ONNX_2.onnx" "$POLICY_URL"
fi

step "USB-serial latency timer (upstream README)"
RULE=/etc/udev/rules.d/99-usb-serial.rules
LINE='SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"'
if [ -f "$RULE" ] && grep -qF "$LINE" "$RULE"; then
  echo "present: $RULE"
else
  echo "$LINE" | sudo tee -a "$RULE" >/dev/null && sudo udevadm control --reload-rules
  echo "added: $RULE"
fi

step "systemd unit jero-walk.service"
sed -e "s|@USER@|$USER|g" -e "s|@HOME@|$HOME|g" -e "s|@REPO@|$REPO|g" -e "s|@VENV@|$VENV|g" \
  "$REPO/robot/systemd/jero-walk.service" | sudo tee /etc/systemd/system/jero-walk.service >/dev/null
if [ ! -f /etc/default/jero-walk ]; then
  echo 'JERO_WALK_ARGS=""' | sudo tee /etc/default/jero-walk >/dev/null
fi
sudo systemctl daemon-reload
echo "installed (not enabled). Start manually: sudo systemctl start jero-walk"

if [ "$REPLACE_AUTOSTART" = 1 ]; then
  step "Replacing upstream foot-switch autostart with jero-walk"
  sudo systemctl disable --now open-duck-walk.service 2>/dev/null || true
  sudo systemctl enable jero-walk.service
fi

step "Security checklist (event Wi-Fi is hostile)"
cat <<'EOF'
  [ ] passwd                                  # default is bdxv2 / ilovemyduck
  [ ] sudo nmcli connection modify Openduck wifi-sec.psk "<new strong pass>"
  [ ] delete or change the 'Duckspot' / 12345678 fallback network
  [ ] keep the link signed (never run jero_walk with --link-key none at the venue)
EOF
echo
echo "Done."

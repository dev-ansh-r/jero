#!/usr/bin/env bash
# Set up Jero on a Qualcomm RB3 Gen 2 running Ubuntu (aarch64), replacing the Pi Zero 2 W.
# Same robot software as the Pi; the Pi-only GPIO parts are swapped by robot/board_shims.py.
# Idempotent: re-running only fills in what is missing. See docs/rb3gen2.md.
#
#   git clone -b rb3gen2 https://github.com/dev-ansh-r/jero ~/Jero
#   ~/Jero/robot/setup_rb3.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${VENV:-$HOME/.virtualenvs/open-duck-mini-runtime}"
RUNTIME="${JERO_RUNTIME_DIR:-$HOME/Open_Duck_Mini_Runtime}"
RUNTIME_SHA=376de65435c93486347cd601a31aa96951799896
KEY_DIR="$HOME/.config/jero"
KEY_FILE="$KEY_DIR/link.key"
POLICY_URL="https://raw.githubusercontent.com/apirrone/Open_Duck_Mini/b23317a485b3cec7d8417f352478778b3475173c/BEST_WALK_ONNX_2.onnx"

step() { printf '\n==> %s\n' "$*"; }
warn() { printf 'WARN: %s\n' "$*" >&2; }

[ "$(uname -m)" = "aarch64" ] || warn "not aarch64: this script targets the RB3 Gen 2"
command -v apt-get >/dev/null || {
  echo "no apt-get: this script needs Ubuntu on the RB3 Gen 2 (Qualcomm Linux images have no package manager)" >&2
  exit 1
}

step "System packages"
sudo apt-get update
sudo apt-get install -y git curl python3-venv python3-dev build-essential i2c-tools gpiod bluez usbutils
for g in dialout i2c input gpio; do
  if getent group "$g" >/dev/null && ! id -nG "$USER" | tr ' ' '\n' | grep -qx "$g"; then
    sudo usermod -aG "$g" "$USER"
    echo "added $USER to group $g (log out and back in for it to apply)"
  fi
done

step "Python environment ($VENV)"
if ! command -v uv >/dev/null; then
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.local/bin:$PATH"
fi
[ -x "$VENV/bin/python" ] || uv venv --seed --python /usr/bin/python3 "$VENV"
grep -qF "$VENV/bin/activate" "$HOME/.bashrc" || echo "source $VENV/bin/activate" >>"$HOME/.bashrc"

step "Open_Duck_Mini_Runtime at $RUNTIME_SHA"
[ -d "$RUNTIME/.git" ] || git clone https://github.com/apirrone/Open_Duck_Mini_Runtime "$RUNTIME"
git -C "$RUNTIME" fetch -q origin "$RUNTIME_SHA" 2>/dev/null || true
git -C "$RUNTIME" checkout -q "$RUNTIME_SHA"
# --no-deps: its list pulls Pi-only Adafruit/Blinka GPIO and pypot; install only what the walk uses.
uv pip install --python "$VENV/bin/python" -e "$RUNTIME" --no-deps
uv pip install --python "$VENV/bin/python" numpy onnxruntime scipy pygame smbus2
uv pip install --python "$VENV/bin/python" rustypot || warn "rustypot not installed: fine, the walk uses robot/feetech_io.py"
uv pip install --python "$VENV/bin/python" gpiod || warn "python gpiod not installed: only needed for wired foot switches (--feet gpiod)"
uv pip install --python "$VENV/bin/python" -e "$REPO/jero_link"
"$VENV/bin/python" -c "import onnxruntime, numpy, pygame, jero_link; print('ok: onnxruntime', onnxruntime.__version__, 'numpy', numpy.__version__)"

step "Link key"
if [ -f "$KEY_FILE" ]; then
  echo "exists: $KEY_FILE"
else
  mkdir -p "$KEY_DIR" && chmod 700 "$KEY_DIR"
  "$REPO/tools/gen_link_key.sh" "$KEY_FILE"
  echo "copy it to the brain/laptop as ~/.config/jero/link.key"
fi

step "duck_config.json and the walk policy"
[ -f "$HOME/duck_config.json" ] || cp "$REPO/robot/duck_config.json" "$HOME/duck_config.json"
[ -f "$HOME/BEST_WALK_ONNX_2.onnx" ] || curl -fL --retry 3 -o "$HOME/BEST_WALK_ONNX_2.onnx" "$POLICY_URL"
ls -l "$HOME/duck_config.json" "$HOME/BEST_WALK_ONNX_2.onnx"

step "Pico bus bridge: stable name /dev/jero-servo"
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", SYMLINK+="jero-servo", GROUP="dialout", MODE="0660"' |
  sudo tee /etc/udev/rules.d/99-jero-servo.rules >/dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger

step "Bluetooth for the PS4 pad (reconnects with the PS button after a reboot)"
sudo sed -i 's/^#\?AutoEnable=.*/AutoEnable=true/' /etc/bluetooth/main.conf
sudo sed -i 's/^#\?ClassicBondedOnly=.*/ClassicBondedOnly=false/' /etc/bluetooth/input.conf
sudo install -m 755 "$REPO/robot/jero-bt-setup.sh" /usr/local/bin/jero-bt-setup
sudo install -m 644 "$REPO/robot/systemd/jero-bt-connectable.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now jero-bt-connectable.service || warn "bluetooth setup didn't finish: check journalctl -u jero-bt-connectable"

step "systemd unit jero-walk.service (installed, not enabled)"
sed -e "s|@USER@|$USER|g" -e "s|@HOME@|$HOME|g" -e "s|@REPO@|$REPO|g" -e "s|@VENV@|$VENV|g" \
  "$REPO/robot/systemd/jero-walk.service" | sudo tee /etc/systemd/system/jero-walk.service >/dev/null
if [ ! -f /etc/default/jero-walk ]; then
  echo 'JERO_WALK_ARGS="--board other --imu mpu6050 --serial-port /dev/jero-servo"' | sudo tee /etc/default/jero-walk >/dev/null
fi
sudo systemctl daemon-reload

step "Done. Next (docs/rb3gen2.md):"
cat <<'EOF'
  1. i2cdetect -l; i2cdetect -y <bus>          find the IMU (68), then: python ~/Jero/tools/imu_check.py --bus <bus>
  2. python ~/Jero/tools/bus_test.py --port /dev/jero-servo
  3. python ~/Jero/tools/policy_dryrun.py      (policy + IMU, no servos)
  4. pair the PS4 pad (btmgmt ssp on / bluetoothctl pair, trust, connect / btmgmt ssp off)
  5. python ~/Jero/robot/jero_walk.py --board other --imu mpu6050 --no-link --serial-port /dev/jero-servo
EOF

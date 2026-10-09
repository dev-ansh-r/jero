#!/usr/bin/env bash
# RealSense + dashboard environment on the RB3 Gen 2 (Ubuntu aarch64), separate from the walk's:
#   ~/Jero/robot/setup_vision.sh
# Installs Intel's pyrealsense2 wheel (aarch64, no librealsense build needed), OpenCV and numpy into
# ~/.virtualenvs/jero-vision, and the RealSense udev rules (camera usable without sudo).
# Then: ~/.virtualenvs/jero-vision/bin/python ~/Jero/brain/eyes.py   ->  http://<robot>:8080
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${VISION_VENV:-$HOME/.virtualenvs/jero-vision}"
RS_VERSION="${RS_VERSION:-2.58.4}"

step() { printf '\n==> %s\n' "$*"; }

step "Python environment ($VENV)"
if [ ! -x "$VENV/bin/python" ]; then
  if command -v uv >/dev/null || [ -x "$HOME/.local/bin/uv" ]; then
    "$(command -v uv || echo "$HOME/.local/bin/uv")" venv -q -p /usr/bin/python3 "$VENV"
  else
    python3 -m venv "$VENV"
  fi
fi
if [ -x "$HOME/.local/bin/uv" ] || command -v uv >/dev/null; then
  "$(command -v uv || echo "$HOME/.local/bin/uv")" pip install -q --python "$VENV/bin/python" \
    "pyrealsense2==$RS_VERSION.*" numpy opencv-python-headless
else
  "$VENV/bin/pip" install -q "pyrealsense2==$RS_VERSION.*" numpy opencv-python-headless
fi
"$VENV/bin/python" -c "import pyrealsense2 as rs, cv2; print('pyrealsense2', rs.__version__, 'opencv', cv2.__version__)"

step "RealSense udev rules"
RULES=/etc/udev/rules.d/99-realsense-libusb.rules
if [ -f "$RULES" ]; then
  echo "present: $RULES"
else
  tmp="$(mktemp)"
  curl -fsSL -o "$tmp" "https://raw.githubusercontent.com/IntelRealSense/librealsense/v$RS_VERSION/config/99-realsense-libusb.rules"
  sudo install -m 644 "$tmp" "$RULES" && rm -f "$tmp"
  sudo udevadm control --reload-rules && sudo udevadm trigger
  echo "installed: $RULES (re-plug the camera)"
fi

step "Camera"
"$VENV/bin/python" - <<'EOF'
import pyrealsense2 as rs
devs = rs.context().query_devices()
for d in devs:
    print(d.get_info(rs.camera_info.name), "USB", d.get_info(rs.camera_info.usb_type_descriptor))
print(f"{len(devs)} RealSense camera(s)" + ("" if devs else ": plug it into a USB 3 port (blue)"))
EOF

step "Done. Dashboard: $VENV/bin/python $REPO/brain/eyes.py   ->  http://$(hostname -I | awk '{print $1}'):8080"

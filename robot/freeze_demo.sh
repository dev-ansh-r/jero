#!/usr/bin/env bash
# Freeze the Pi for the demo, once, AFTER Jero has walked well with the pad on this Pi:
#   ~/Jero/robot/freeze_demo.sh               # record + back up + stop the OS changing itself
#   ~/Jero/robot/freeze_demo.sh --autostart   # also: start Jero at boot (waits for the pad)
#
# 1. records the code commit, the Python packages and a checksum of the policy, ~/duck_config.json
#    and ~/.config/jero/* (calibration, link key) in ~/jero-demo/; robot/demo.sh warns if any change;
# 2. copies those files to ~/jero-demo/backup/ (restore: cp -a ~/jero-demo/backup/... back);
# 3. turns off automatic apt updates/upgrades (no dpkg lock or surprise upgrade at the venue);
# 4. makes sure the pad reconnect service (jero-bt-connectable) is enabled;
# 5. --autostart: installs robot/systemd/jero-demo.service: at boot it runs robot/jero.sh, waits for
#    the pad as long as it takes, starts paused, and restarts it if it ever exits.
#    Undo: sudo systemctl disable --now jero-demo
# Run it again after any change you mean to keep (new calibration, new commit).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${VENV:-$HOME/.virtualenvs/open-duck-mini-runtime}"
FREEZE="$HOME/jero-demo"

step() { printf '\n== %s\n' "$*"; }

autostart=0
[ "${1:-}" = "--autostart" ] && autostart=1

step "code"
rev="$(git -C "$REPO" rev-parse --short HEAD)"
if [ -n "$(git -C "$REPO" status --porcelain --untracked-files=no)" ]; then
  echo "WARNING: uncommitted changes in $REPO (frozen as they are, but not in git):"
  git -C "$REPO" status --short --untracked-files=no
fi
mkdir -p "$FREEZE/backup"
echo "$rev" >"$FREEZE/git-rev"
date '+%Y-%m-%d %H:%M' >"$FREEZE/frozen-at"
echo "commit $rev ($(git -C "$REPO" log -1 --format=%s))"

step "Python packages"
"$VENV/bin/python" -m pip freeze >"$FREEZE/pip-freeze.txt" 2>/dev/null \
  || uv pip freeze --python "$VENV/bin/python" >"$FREEZE/pip-freeze.txt"
echo "$(wc -l <"$FREEZE/pip-freeze.txt") packages -> $FREEZE/pip-freeze.txt"

step "policy, config, calibration: checksums + backup"
files=()
for f in "$HOME/BEST_WALK_ONNX_2.onnx" "$HOME/duck_config.json" "$HOME"/.config/jero/*; do
  [ -f "$f" ] && files+=("$f")
done
sha256sum "${files[@]}" >"$FREEZE/manifest.sha256"
for f in "${files[@]}"; do
  mkdir -p "$FREEZE/backup$(dirname "$f")"
  cp -a "$f" "$FREEZE/backup$f"
done
chmod -R go-rwx "$FREEZE" # the link key is in the backup
sed "s|  $HOME/|  ~/|" "$FREEZE/manifest.sha256" | awk '{print "  " $2}'

step "stop automatic OS updates"
for unit in apt-daily.timer apt-daily-upgrade.timer unattended-upgrades.service; do
  if systemctl cat "$unit" >/dev/null 2>&1; then
    sudo systemctl disable --now "$unit" >/dev/null 2>&1 || true
    echo "disabled $unit"
  fi
done
echo "(undo after the event: sudo systemctl enable --now apt-daily.timer apt-daily-upgrade.timer)"

step "pad reconnect after reboot"
if systemctl cat jero-bt-connectable >/dev/null 2>&1; then
  sudo systemctl enable jero-bt-connectable >/dev/null 2>&1
  echo "jero-bt-connectable: $(systemctl is-enabled jero-bt-connectable), $(systemctl is-active jero-bt-connectable)"
else
  echo "WARNING: jero-bt-connectable not installed: run robot/setup_pi.sh (Bluetooth step)"
fi

if [ "$autostart" = 1 ]; then
  step "autostart at boot (jero-demo.service)"
  grep -q '"start_paused": *false' "$HOME/duck_config.json" 2>/dev/null \
    && { echo "refusing: start_paused is false in ~/duck_config.json (it would step at boot)"; exit 1; }
  sed -e "s|@USER@|$USER|g" -e "s|@HOME@|$HOME|g" -e "s|@REPO@|$REPO|g" -e "s|@VENV@|$VENV|g" \
    "$REPO/robot/systemd/jero-demo.service" | sudo tee /etc/systemd/system/jero-demo.service >/dev/null
  sudo systemctl daemon-reload
  sudo systemctl disable --now jero-walk.service open-duck-walk.service >/dev/null 2>&1 || true
  sudo systemctl enable jero-demo.service
  echo "enabled: from the next boot, power on -> PS on the pad -> Cross."
  echo "start it now without rebooting: sudo systemctl start jero-demo; watch: journalctl -fu jero-demo"
fi

step "frozen at $rev in $FREEZE. On the day: $REPO/robot/demo.sh --check"

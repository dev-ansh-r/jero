#!/usr/bin/env bash
# Demo day: check everything without moving anything, then start Jero with the PS4 pad.
#   ~/Jero/robot/demo.sh            # checks, then robot/jero.sh (extra arguments go to it)
#   ~/Jero/robot/demo.sh --check    # checks only
#
# If the autostart service is on (robot/freeze_demo.sh --autostart), there is nothing to run:
# power on, press PS on the pad, press Cross. This script then only says so.
#
# Checks: frozen files unchanged (robot/freeze_demo.sh), Pico present, all 14 servos answer
# (reads only), IMU answers on I2C, Bluetooth powered + connectable service, pad connected.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${VENV:-$HOME/.virtualenvs/open-duck-mini-runtime}"
PY="$VENV/bin/python"
FREEZE="$HOME/jero-demo"
fails=0

say() { printf '[demo] %s\n' "$*"; }
ok() { printf '[demo]   OK    %s\n' "$*"; }
warn() { printf '[demo]   WARN  %s\n' "$*"; }
bad() { printf '[demo]   FAIL  %s\n' "$*"; fails=$((fails + 1)); }

check_only=0
if [ "${1:-}" = "--check" ]; then check_only=1; shift; fi

# -- already running as a service? ----------------------------------------------------------
for unit in jero-demo jero-walk; do
  if systemctl is-active --quiet "$unit" 2>/dev/null; then
    say "Jero is already running ($unit.service): press PS on the pad, then Cross."
    say "watch it:  journalctl -fu $unit      run by hand instead:  sudo systemctl stop $unit"
    exit 0
  fi
done

[ -x "$PY" ] || { say "no Python environment at $VENV (run robot/setup_pi.sh)"; exit 1; }

# -- frozen state ---------------------------------------------------------------------------
say "frozen state"
if [ -f "$FREEZE/manifest.sha256" ]; then
  if (cd / && sha256sum --quiet -c "$FREEZE/manifest.sha256" >/dev/null 2>&1); then
    ok "policy, configs, calibration unchanged since $(cat "$FREEZE/frozen-at" 2>/dev/null)"
  else
    warn "changed since the freeze: $(cd / && sha256sum --quiet -c "$FREEZE/manifest.sha256" 2>&1 | tr '\n' ' ')"
    warn "the frozen copies are in $FREEZE/backup/"
  fi
  rev="$(git -C "$REPO" rev-parse --short HEAD)"
  if [ "$rev" = "$(cat "$FREEZE/git-rev" 2>/dev/null)" ]; then
    ok "code at the frozen commit $rev"
  else
    warn "code at $rev, frozen at $(cat "$FREEZE/git-rev" 2>/dev/null): git -C $REPO checkout $(cat "$FREEZE/git-rev" 2>/dev/null)"
  fi
else
  warn "not frozen yet (robot/freeze_demo.sh)"
fi

# -- servo bridge + servos ------------------------------------------------------------------
say "servos"
port=""
for cand in /dev/jero-servo /dev/serial/by-id/*Pico* /dev/ttyACM*; do
  if [ -e "$cand" ]; then port="$cand"; break; fi
done
if [ -z "$port" ]; then
  bad "Pico not found: check its USB cable (lsusb | grep 2e8a)"
elif command -v fuser >/dev/null && fuser "$port" >/dev/null 2>&1; then
  bad "$port in use by PID(s):$(fuser "$port" 2>/dev/null) (another walk or tool still running?)"
elif out="$("$PY" "$REPO/tools/bus_test.py" --port "$port" --rounds 50 2>&1)"; then
  ok "all 14 servos answer on $port"
else
  bad "servo bus check failed on $port (servo power on? battery charged?):"
  printf '%s\n' "$out" | tail -8 | sed 's/^/           /'
fi

# -- IMU ------------------------------------------------------------------------------------
say "IMU"
if [ ! -f "$HOME/.config/jero/imu.json" ]; then
  bad "no IMU calibration (~/.config/jero/imu.json)"
elif imu="$("$PY" - <<'EOF' 2>&1
import json, pathlib
from smbus2 import SMBus
c = json.loads((pathlib.Path.home() / ".config/jero/imu.json").read_text())
bus, addr = int(c.get("bus", 1)), int(c.get("address", 0x68))
with SMBus(bus) as b:
    who = b.read_byte_data(addr, 0x75)
print(f"bus {bus} addr {addr:#x} WHO_AM_I {who:#04x}, tilt correction {'yes' if 'accel_offset' in c else 'NO'}")
EOF
)"; then
  ok "$imu"
else
  bad "IMU not answering: $(printf '%s' "$imu" | tail -1) (check its 4 wires)"
fi

# -- Bluetooth + pad ------------------------------------------------------------------------
say "PS4 pad"
if bluetoothctl show 2>/dev/null | grep -q "Powered: yes"; then ok "Bluetooth powered"; else bad "Bluetooth off: sudo systemctl restart bluetooth jero-bt-connectable"; fi
if systemctl is-active --quiet jero-bt-connectable 2>/dev/null; then
  ok "pad reconnect service active"
else
  warn "jero-bt-connectable not active: sudo systemctl restart jero-bt-connectable"
fi
if [ -e /dev/input/js0 ]; then ok "pad connected"; else warn "pad not connected yet: press its PS button"; fi

# -- result ---------------------------------------------------------------------------------
if [ "$fails" -gt 0 ]; then
  say "$fails check(s) failed: fix them first"
  exit 1
fi
if [ "$check_only" = 1 ]; then
  say "all good. Start with: $REPO/robot/demo.sh"
  exit 0
fi
say "all good: starting Jero. Cross (X) starts/pauses, left stick walks, right stick turns, Ctrl+C stops."
exec "$REPO/robot/jero.sh" "$@"

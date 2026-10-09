#!/usr/bin/env bash
# One shot: start Jero with the PS4 pad, on the Pi Zero 2 W or the RB3 Gen 2.
#   ~/Jero/robot/jero.sh            # extra arguments go to jero_walk.py, e.g. --no-link
#
# 1. waits for the pad (/dev/input/js0): press PS if it isn't connected yet;
# 2. finds the Pico servo bridge (/dev/jero-servo, /dev/serial/by-id/*Pico*, /dev/ttyACM*) and
#    refuses if another program holds it (it never kills anything);
# 3. checks the IMU calibration, the link key and start_paused;
# 4. runs robot/jero_walk.py with the arguments that walked on the Pi (+ --board other on non-Pi),
#    Wi-Fi link on (voice commands + tools/estop.py), output also saved under ~/jero-logs/;
# 5. gives the walk real-time priority on the fastest core if passwordless sudo allows it
#    (speech and the camera share the CPU on the RB3).
# Cross (X) starts/pauses, left stick walks, right stick turns, Ctrl+C stops.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${VENV:-$HOME/.virtualenvs/open-duck-mini-runtime}"
PAD_WAIT_S="${JERO_PAD_WAIT_S:-60}"
LOG_DIR="$HOME/jero-logs"

say() { printf '[jero] %s\n' "$*"; }
die() { printf '[jero] ERROR: %s\n' "$*" >&2; exit 1; }

# -- python ---------------------------------------------------------------------------------
PY="$VENV/bin/python"
[ -x "$PY" ] || die "no Python environment at $VENV (run robot/setup_pi.sh or robot/setup_rb3.sh)"

# -- board ----------------------------------------------------------------------------------
board_args=()
if grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
  say "board: $(tr -d '\0' </proc/device-tree/model)"
else
  board_args=(--board other)
  say "board: $(tr -d '\0' </proc/device-tree/model 2>/dev/null || echo 'not a Raspberry Pi') (non-Pi shims)"
fi

# -- servo bridge ---------------------------------------------------------------------------
port=""
for cand in /dev/jero-servo /dev/serial/by-id/*Pico* /dev/ttyACM*; do
  if [ -e "$cand" ]; then port="$cand"; break; fi
done
[ -n "$port" ] || die "Pico servo bridge not found (no /dev/jero-servo or /dev/ttyACM*): check its USB cable"
if command -v fuser >/dev/null && fuser "$port" >/dev/null 2>&1; then
  die "$port is in use by PID(s):$(fuser "$port" 2>/dev/null) - stop that program first (fuser -v $port)"
fi
say "servo bridge: $port"

# -- calibration, key, config ---------------------------------------------------------------
[ -f "$HOME/.config/jero/imu.json" ] || die "no IMU calibration: run tools/imu_check.py (and tools/imu_tilt.py)"
grep -q accel_offset "$HOME/.config/jero/imu.json" || say "note: no tilt correction yet (tools/imu_tilt.py)"
[ -f "$HOME/.config/jero/link.key" ] || die "no link key (~/.config/jero/link.key): run the setup script"
if grep -q '"start_paused": *false' "$HOME/duck_config.json" 2>/dev/null; then
  say "note: start_paused is false in ~/duck_config.json: the robot steps as soon as it starts"
else
  say "starts paused: press Cross (X) to walk"
fi

# -- gamepad --------------------------------------------------------------------------------
if [ ! -e /dev/input/js0 ]; then
  say "waiting up to ${PAD_WAIT_S}s for the PS4 pad: press its PS button"
  for _ in $(seq 1 "$PAD_WAIT_S"); do
    [ -e /dev/input/js0 ] && break
    sleep 1
  done
fi
[ -e /dev/input/js0 ] || die "no pad (/dev/input/js0). Check: bluetoothctl show | grep Powered; sudo btmgmt info"
say "pad connected"

# -- priority: once the walk is running, give it the fastest core and real-time scheduling ----
boost() {
  local pid=""
  for _ in $(seq 1 20); do
    pid="$(pgrep -n -f "robot/jero_walk.py" || true)"
    [ -n "$pid" ] && break
    sleep 0.5
  done
  [ -n "$pid" ] || return 0
  sleep 8 # let it start its threads (IMU, pad, link) so all of them get the boost
  local best="" cap max=0 f
  for f in /sys/devices/system/cpu/cpu[0-9]*/cpu_capacity; do
    [ -r "$f" ] || continue
    cap="$(cat "$f")"
    if [ "$cap" -gt "$max" ]; then max="$cap"; best="$(basename "$(dirname "$f")")"; fi
  done
  if sudo -n true 2>/dev/null; then
    sudo -n chrt -a -f -p 20 "$pid" >/dev/null 2>&1 && say "walk (PID $pid): real-time priority" || true
    if [ -n "$best" ]; then
      sudo -n taskset -a -cp "${best#cpu}" "$pid" >/dev/null 2>&1 && say "walk pinned to $best (fastest core)" || true
    fi
  else
    say "note: no passwordless sudo, walk runs at normal priority"
  fi
}

mkdir -p "$LOG_DIR"
log="$LOG_DIR/walk-$(date +%Y%m%d-%H%M%S).log"
say "log: $log"
boost &
exec > >(tee -a "$log") 2>&1
exec "$PY" "$REPO/robot/jero_walk.py" "${board_args[@]}" --imu mpu6050 --serial-port "$port" "$@"

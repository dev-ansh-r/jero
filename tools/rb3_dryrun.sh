#!/usr/bin/env bash
# Whole-robot dry run on the RB3 Gen 2: checks every part, nothing moves.
#   ~/Jero/tools/rb3_dryrun.sh
# Board health, servo bridge (+ a read-only bus test when the servos are powered), onboard IMU,
# policy timing with the real IMU while the camera dashboard runs, PS4 pad, camera + dashboard,
# speaker/mic, voice, network. Prints PASS / WARN / FAIL per line; exit 1 if anything FAILs.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${VENV:-$HOME/.virtualenvs/open-duck-mini-runtime}/bin/python"
fails=0 warns=0
pass() { printf '  PASS  %s\n' "$*"; }
warn() { printf '  WARN  %s\n' "$*"; warns=$((warns + 1)); }
fail() { printf '  FAIL  %s\n' "$*"; fails=$((fails + 1)); }
section() { printf '\n== %s\n' "$*"; }

section "board"
pass "$(tr -d '\0' </proc/device-tree/model 2>/dev/null), up $(uptime -p | sed 's/up //')"
hot=$(for z in /sys/class/thermal/thermal_zone*/temp; do cat "$z"; done 2>/dev/null | sort -n | tail -1)
if [ "${hot:-0}" -lt 75000 ]; then pass "hottest zone $((hot / 1000)) C"; else warn "hottest zone $((hot / 1000)) C"; fi
avail=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)
if [ "$avail" -gt 1500 ]; then pass "memory available ${avail} MB"; else warn "memory available ${avail} MB"; fi
for s in sscrpcd jero-bt-connectable; do
  if systemctl is-active --quiet "$s"; then pass "service $s active"; else fail "service $s not active"; fi
done

section "servo bridge (Pico)"
port=""
for cand in /dev/jero-servo /dev/ttyACM*; do if [ -e "$cand" ]; then port="$cand"; break; fi; done
if [ -z "$port" ]; then
  fail "Pico not on USB (lsusb | grep 2e8a)"
elif fuser "$port" >/dev/null 2>&1; then
  warn "$port in use (walk running?): bus test skipped"
elif out=$("$PY" "$REPO/tools/bus_test.py" --port "$port" --rounds 50 2>&1); then
  pass "Pico on $port, all 14 servos answer"
else
  warn "Pico on $port, servos not answering: servo battery off? ($(printf '%s' "$out" | tail -1))"
fi

section "IMU (RB3 onboard ICM-42688)"
cfg="$HOME/.config/jero/imu.json"
if ! grep -q '"backend": "qsh"' "$cfg" 2>/dev/null; then
  fail "not calibrated for the onboard IMU: tools/imu_rb3.py --calibrate"
else
  pass "calibrated: axes $(sed -n 's/.*"axes": "\(.*\)".*/\1/p' "$cfg")"
  if sed -n '/accel_offset/,/]/p' "$cfg" | grep -Eq '[1-9]'; then pass "tilt correction set"; else warn "no tilt correction yet: tools/imu_tilt.py (servos on, standing)"; fi
  if out=$("$PY" "$REPO/tools/imu_rb3.py" --seconds 2 2>&1); then
    pass "$(printf '%s' "$out" | grep -E 'rate:' | sed 's/.*rate: //'), still and quiet"
  else
    fail "IMU test: $(printf '%s' "$out" | grep -E 'FAIL|Error|error' | head -1)"
  fi
fi

section "policy with the real IMU (dashboard running alongside)"
if out=$(cd "$REPO" && timeout 60 "$PY" tools/policy_dryrun.py --seconds 10 2>&1); then
  pass "$(printf '%s' "$out" | grep 'loop work' | sed 's/^ *//')"
  pass "$(printf '%s' "$out" | grep 'deadline misses' | sed 's/^ *//')"
else
  fail "policy dry run: $(printf '%s' "$out" | tail -2 | tr '\n' ' ')"
fi

section "PS4 pad"
if [ -e /dev/input/js0 ]; then pass "pad connected (js0)"; else warn "pad not connected: press PS"; fi

section "camera + dashboard"
if lsusb | grep -q 8086:0b5c; then pass "RealSense D455 on USB"; else fail "RealSense not on USB"; fi
if st=$(curl -s -m 3 http://127.0.0.1:8080/status.json); then
  pass "dashboard up: $(printf '%s' "$st" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d["camera"] + ", " + str(d["fps"]) + " fps, mode " + d["mode"])')"
else
  warn "dashboard not running: ~/.virtualenvs/jero-vision/bin/python $REPO/brain/eyes.py"
fi
if pgrep -f "[b]rain/show.py" >/dev/null; then pass "show controller running$(pgrep -af '[b]rain/show.py' | grep -q dry-run && echo ' (dry run: motions only logged)')"; else warn "show controller not running (dashboard buttons won't speak/move)"; fi

section "speaker + mic + voice"
if aplay -l 2>/dev/null | grep -q '^card'; then pass "speaker: $(aplay -l | grep -m1 '^card' | cut -d: -f2 | cut -d, -f1 | sed 's/^ //')"; else fail "no sound card (display driver blacklisted?)"; fi
if arecord -l 2>/dev/null | grep -q '^card'; then pass "mic present"; else warn "no capture device"; fi
if [ -s "$HOME/.config/jero/elevenlabs.env" ]; then pass "ElevenLabs key configured"; else warn "no ElevenLabs key: lines shown, not spoken"; fi
n=$(find "$HOME/.cache/jero/show-tts" -name '*.wav' 2>/dev/null | wc -l)
if [ "$n" -ge 6 ]; then pass "$n show lines cached (play offline)"; else warn "only $n show lines cached: start brain/show.py once online"; fi
if curl -s -m 4 -o /dev/null https://api.elevenlabs.io; then pass "internet: ElevenLabs reachable"; else warn "ElevenLabs unreachable: cached lines only"; fi

section "network"
pass "addresses: $(hostname -I | tr ' ' '\n' | grep -v ':' | grep -v '^$' | tr '\n' ' ')"
ssid=$(nmcli -t -f active,ssid dev wifi 2>/dev/null | sed -n 's/^yes://p' | head -1)
if [ -n "$ssid" ]; then pass "Wi-Fi: $ssid"; else warn "no Wi-Fi connection (wired only?): set up the venue hotspot"; fi

printf '\n== RESULT: %d fail, %d warn\n' "$fails" "$warns"
[ "$fails" -eq 0 ]

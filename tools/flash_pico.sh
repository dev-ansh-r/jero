#!/usr/bin/env bash
# Flash the Pico bus bridge from the Pi, without unplugging anything.
#   tools/flash_pico.sh firmware.uf2 [/dev/ttyACM0]
#
# 1. a "1200 baud touch" on the Pico's port reboots it into its USB-drive bootloader (RPI-RP2);
# 2. the .uf2 is copied onto that drive; the Pico reboots into the new firmware by itself;
# 3. waits for the serial port to come back.
# Stop the walk first (only one program can use the port). Servo power can stay on.
set -euo pipefail

UF2="${1:?usage: tools/flash_pico.sh firmware.uf2 [/dev/ttyACM0]}"
PORT="${2:-/dev/ttyACM0}"
DISK=/dev/disk/by-label/RPI-RP2
[ -f "$UF2" ] || { echo "no such file: $UF2" >&2; exit 1; }

if [ ! -e "$DISK" ]; then
  [ -e "$PORT" ] || { echo "$PORT not found and no RPI-RP2 drive: is the Pico plugged in?" >&2; exit 1; }
  if fuser "$PORT" >/dev/null 2>&1; then
    echo "$PORT is in use (walk running?): stop it first" >&2
    exit 1
  fi
  echo "rebooting the Pico into its bootloader (1200 baud touch on $PORT)"
  stty -F "$PORT" 1200 || true
fi

for _ in $(seq 1 50); do
  [ -e "$DISK" ] && break
  sleep 0.2
done
[ -e "$DISK" ] || {
  echo "RPI-RP2 drive didn't appear. Fallback: hold BOOTSEL while re-plugging the Pico's USB, then re-run." >&2
  exit 1
}

MNT="$(mktemp -d)"
sudo mount "$DISK" "$MNT"
echo "copying $UF2"
sudo cp "$UF2" "$MNT/" && sync
sudo umount "$MNT" 2>/dev/null || true   # the Pico may already have rebooted and taken the drive away
rmdir "$MNT" 2>/dev/null || true

for _ in $(seq 1 50); do
  [ -e "$PORT" ] && break
  sleep 0.2
done
if [ -e "$PORT" ]; then
  echo "done: $PORT is back with the new firmware"
else
  echo "copied, but $PORT hasn't come back yet: check 'dmesg | tail'" >&2
  exit 1
fi

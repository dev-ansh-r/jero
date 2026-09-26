#!/usr/bin/env bash
# Assign IDs to all 14 STS3215 servos, one at a time, using upstream configure_motor.py.
# Each new servo ships as ID 1, so ONLY ONE servo may be on the bus while it is configured.
# After each one: fit the horn at the zero position and stick the label on the servo.
#
# Run where the runtime is installed (Pi image, or a laptop with the runtime venv):
#   tools/configure_servos.sh [--port /dev/ttyACM0] [--from right_knee]
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${JERO_RUNTIME_DIR:-$HOME/Open_Duck_Mini_Runtime}"
[ -d "$RUNTIME" ] || RUNTIME="$REPO/upstream/Open_Duck_Mini_Runtime"
PORT=/dev/ttyACM0
FROM=""

while [ $# -gt 0 ]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --from) FROM="$2"; shift 2 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

# name:id, same map as upstream rustypot_position_hwi.py
SERVOS=(
  right_hip_yaw:10 right_hip_roll:11 right_hip_pitch:12 right_knee:13 right_ankle:14
  left_hip_yaw:20  left_hip_roll:21  left_hip_pitch:22  left_knee:23  left_ankle:24
  neck_pitch:30    head_pitch:31     head_yaw:32        head_roll:33
)

started=0
[ -z "$FROM" ] && started=1
done_list=()
for entry in "${SERVOS[@]}"; do
  name="${entry%%:*}"; id="${entry##*:}"
  if [ "$started" = 0 ]; then
    if [ "$name" = "$FROM" ]; then started=1; else continue; fi
  fi
  echo
  echo "=== $name  ->  ID $id ==="
  read -r -p "Connect ONLY the '$name' servo to the adapter, power on, then press Enter (s = skip) " ans
  [ "$ans" = "s" ] && continue
  (cd "$RUNTIME/scripts" && python configure_motor.py --port "$PORT" --id "$id")
  echo ">> Fit the horn at zero, label the servo '$id $name', then disconnect it."
  done_list+=("$id:$name")
done

echo
echo "Configured: ${done_list[*]:-none}"
echo "Next: assemble, then run check_motors.py with all servos daisy-chained."

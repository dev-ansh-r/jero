#!/usr/bin/env bash
# At boot (jero-imu.service, root): stream the RB3 Gen 2's onboard IMU for the walk (robot/imu_iio.py).
# Stops iio-sensor-proxy (screen rotation; it would take samples from the same buffers), then for the
# accel_3d and gyro_3d IIO devices: 100 Hz, x/y/z + timestamp in the scan, buffer on.
# /dev/iio:deviceN is already readable by everyone on this image.
set -u

systemctl stop iio-sensor-proxy 2>/dev/null || true

for pair in accel_3d:accel gyro_3d:anglvel; do
  name="${pair%%:*}" kind="${pair#*:}" dev=""
  for d in /sys/bus/iio/devices/iio:device*; do
    if [ "$(cat "$d/name" 2>/dev/null)" = "$name" ]; then dev="$d"; fi
  done
  if [ -z "$dev" ]; then
    echo "jero-imu: no IIO device $name" >&2
    continue
  fi
  echo 0 >"$dev/buffer/enable"
  echo 100 >"$dev/in_${kind}_sampling_frequency"
  for ax in x y z; do echo 1 >"$dev/scan_elements/in_${kind}_${ax}_en"; done
  echo 1 >"$dev/scan_elements/in_timestamp_en"
  echo 128 >"$dev/buffer/length"
  echo 1 >"$dev/buffer/enable"
  echo "jero-imu: $name ($(basename "$dev")) $(cat "$dev/in_${kind}_sampling_frequency") Hz, buffer $(cat "$dev/buffer/enable")"
done

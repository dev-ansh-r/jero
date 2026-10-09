"""Walk telemetry for the dashboard: commanded velocity, gyro yaw rate and pause state over UDP.

A daemon thread reads what the walk already keeps (``RLWalk.last_commands``, the IMU driver's
``last_imu_data``, ``RLWalk.paused``) 10 times a second and sends one JSON datagram to the dashboard
(brain/eyes.py integrates it into the "covered path" card). Fire and forget: never blocks the walk,
never takes IMU samples (it reads the cached reading), silent if nothing listens.
"""

from __future__ import annotations

import json
import logging
import socket
import threading
import time

log = logging.getLogger("jero.telemetry")
DEFAULT_ADDR = ("127.0.0.1", 5006)


def snapshot(rl) -> dict:
    cmds = list(getattr(rl, "last_commands", None) or [0.0] * 7)
    imu = getattr(getattr(rl, "imu", None), "last_imu_data", None) or {}
    gyro = imu.get("gyro")
    return {
        "t": time.time(),
        "vx": float(cmds[0]),
        "vy": float(cmds[1]),
        "wz": float(cmds[2]),
        "gz": None if gyro is None else float(gyro[2]),
        "paused": bool(getattr(rl, "paused", False)),
    }


def start(rl, addr: tuple = DEFAULT_ADDR, rate_hz: float = 10.0) -> threading.Thread:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def loop():
        period, errors = 1.0 / rate_hz, 0
        while True:
            try:
                sock.sendto(json.dumps(snapshot(rl)).encode(), addr)
            except OSError as exc:  # nobody listening is fine; anything else: say so once
                errors += 1
                if errors == 1:
                    log.debug("telemetry send failed: %s", exc)
            except Exception as exc:  # noqa: BLE001  never take the walk down
                log.warning("telemetry: %s", exc)
            time.sleep(period)

    th = threading.Thread(target=loop, name="jero-telemetry", daemon=True)
    th.start()
    log.info("telemetry -> udp://%s:%d at %g Hz (dashboard path card)", addr[0], addr[1], rate_hz)
    return th

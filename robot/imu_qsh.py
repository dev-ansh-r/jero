"""RB3 Gen 2 onboard IMU (TDK ICM-42688, "icm4x6xx") through Qualcomm's sensor hub, MPU6050-driver compatible.

The IMU is wired to the sensing hub, not to Linux I2C. Qualcomm's ``see_workhorse`` (package
qcom-sensors-test-apps) streams it: one process for accel, one for gyro, printing every sample as
JSON-like text, which reader threads parse, keeping the newest. Measured on the RB3: samples arrive
evenly every 10 ms at 100 Hz (no buffering when run under ``stdbuf -oL``).

Units as the walk wants them: accel m/s^2 specific force (flat, face up: z = +9.8), gyro rad/s.
Same ``read_si()`` / ``init()`` / ``whoami`` as ``imu_mpu6050.Mpu6050``: ``imu_mpu6050.Imu`` uses it when
~/.config/jero/imu.json says ``"backend": "qsh"`` (tools/imu_rb3.py --calibrate writes that).

Setup (robot/setup_rb3.sh): ``sudo apt install qcom-sensors-test-apps`` and
``sudo setcap cap_wake_alarm+ep /usr/bin/see_workhorse`` (it needs the wake-alarm timer; this avoids sudo).
"""

from __future__ import annotations

import atexit
import logging
import shutil
import subprocess
import threading
import time

import numpy as np

log = logging.getLogger("jero.imu.qsh")
TOOL = "see_workhorse"
RATE_HZ = 200
STALE_S = 0.2


class SampleParser:
    """Feed see_workhorse output lines; ``latest`` is the newest 3-value "data" array."""

    def __init__(self):
        self.latest: np.ndarray | None = None
        self.count = 0
        self._grab, self._cur = 0, []

    def feed(self, line: str) -> bool:
        """True when this line completed a sample."""
        if '"data" : [' in line:
            self._grab, self._cur = 3, []
            return False
        if self._grab:
            try:
                self._cur.append(float(line.strip().rstrip(",")))
            except ValueError:  # not a sample after all
                self._grab = 0
                return False
            self._grab -= 1
            if not self._grab:
                self.latest, self.count = np.array(self._cur), self.count + 1
                return True
        return False


class _Stream:
    def __init__(self, sensor: str, rate_hz: float, tool: str = TOOL, clock=time.monotonic):
        self.sensor, self.rate_hz, self.tool, self.clock = sensor, rate_hz, tool, clock
        self.parser, self.t = SampleParser(), 0.0
        self.proc: subprocess.Popen | None = None
        self.start()

    def start(self) -> None:
        cmd = [
            self.tool,
            f"-sensor={self.sensor}",
            f"-sample_rate={self.rate_hz:g}",
            "-duration=864000",
            "-display_events=1",
        ]
        if shutil.which("stdbuf"):
            cmd = ["stdbuf", "-oL", *cmd]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._read, args=(self.proc,), name=f"jero-imu-{self.sensor}", daemon=True).start()

    def _read(self, proc: subprocess.Popen) -> None:
        for line in proc.stdout:
            if self.parser.feed(line):
                self.t = self.clock()

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def fresh(self) -> bool:
        return self.parser.latest is not None and self.clock() - self.t < STALE_S

    def close(self) -> None:
        if self.alive():
            self.proc.terminate()
            try:
                self.proc.wait(2)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class QshImu:
    """Onboard accel + gyro in SI units (m/s^2 specific force, rad/s), sensor frame."""

    def __init__(self, rate_hz: float = RATE_HZ, wait_s: float = 4.0, tool: str = TOOL, stream=_Stream):
        if shutil.which(tool) is None and stream is _Stream:
            raise FileNotFoundError(f"{tool} not found: sudo apt install qcom-sensors-test-apps (robot/setup_rb3.sh)")
        self.rate_hz, self.whoami = rate_hz, None
        self.name = "RB3 onboard ICM-42688 (Qualcomm sensor hub)"
        self.accel = stream("accel", rate_hz, tool)
        self.gyro = stream("gyro", rate_hz, tool)
        atexit.register(self.close)
        end = time.monotonic() + wait_s
        while time.monotonic() < end and not (self.accel.fresh() and self.gyro.fresh()):
            time.sleep(0.05)
        if not (self.accel.fresh() and self.gyro.fresh()):
            self.close()
            raise OSError(
                f"no samples from {tool} in {wait_s:g} s: check `sudo getcap /usr/bin/{tool}` shows "
                "cap_wake_alarm (sudo setcap cap_wake_alarm+ep /usr/bin/see_workhorse) and sscrpcd is active"
            )

    def init(self) -> None:
        """Restart a stream that died (called by the IMU worker after repeated errors)."""
        for s in (self.accel, self.gyro):
            if not s.alive():
                log.warning("IMU %s stream ended: restarting %s", s.sensor, TOOL)
                s.start()

    def rates(self) -> tuple[float, float]:
        return float(self.rate_hz), float(self.rate_hz)

    def read_si(self) -> tuple[np.ndarray, np.ndarray]:
        if not (self.accel.fresh() and self.gyro.fresh()):
            raise OSError("onboard IMU samples stale")
        return self.accel.parser.latest.copy(), self.gyro.parser.latest.copy()

    def close(self) -> None:
        self.accel.close()
        self.gyro.close()

"""RB3 onboard IMU via the sensor hub (robot/imu_qsh.py): see_workhorse output parsing, staleness."""

import sys
import time
from pathlib import Path
from types import MappingProxyType

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot"))

import imu_mpu6050 as m
import imu_qsh

# trimmed real output of `see_workhorse -sensor=accel -sample_rate=100 -display_events=1` on the RB3
OUTPUT = """18:37:45.710 stream_sensor( accel)
"sns_client_event_msg" : {
 "events" : [
  {
   "msg_id" : 768,
   "payload" : {
    "sample_rate" : 100.000000,
    "range" : [
     -156.906403,
     156.906403
    ],
  }
  {
   "msg_id" : 1025,
   "timestamp" : 663892922241,
   "payload" : {
    "data" : [
     -0.171185,
     -0.202310,
     9.956287
    ],
    "status" : "SNS_STD_SENSOR_SAMPLE_STATUS_UNRELIABLE"
   }
  }
  {
   "msg_id" : 1025,
   "payload" : {
    "data" : [
     0.116119,
     -0.102951,
     9.908403
    ],
"""


def test_parser_reads_data_arrays_not_ranges():
    p = imu_qsh.SampleParser()
    done = [p.feed(line) for line in OUTPUT.splitlines()]
    assert sum(done) == 2 and p.count == 2
    assert p.latest == pytest.approx([0.116119, -0.102951, 9.908403])


class FakeStream:
    VALUES = MappingProxyType({"accel": (0.1, -0.2, 9.9), "gyro": (0.001, 0.002, -0.003)})

    def __init__(self, sensor, rate_hz, tool):
        self.sensor, self.parser, self.t, self.dead = sensor, imu_qsh.SampleParser(), time.monotonic(), False
        self.parser.latest = np.array(self.VALUES[sensor])
        self.parser.count = imu_qsh.WARMUP_SAMPLES + 1

    def fresh(self):
        return time.monotonic() - self.t < imu_qsh.STALE_S

    def alive(self):
        return not self.dead

    def start(self):
        self.dead, self.t = False, time.monotonic()
        self.parser.count = imu_qsh.WARMUP_SAMPLES + 1

    def close(self):
        self.dead = True


def test_qsh_imu_reads_and_flags_stale():
    dev = imu_qsh.QshImu(stream=FakeStream, wait_s=0.5)
    a, g = dev.read_si()
    assert a == pytest.approx([0.1, -0.2, 9.9]) and g == pytest.approx([0.001, 0.002, -0.003])
    dev.accel.t -= 1.0  # no sample for a second
    with pytest.raises(OSError):
        dev.read_si()
    dev.accel.dead = True
    dev.init()  # restarts the dead stream
    assert dev.accel.alive() and dev.read_si()[0][2] == pytest.approx(9.9)


def test_imu_uses_qsh_backend(monkeypatch):
    real = imu_qsh.QshImu
    monkeypatch.setattr(imu_qsh, "QshImu", lambda **kw: real(stream=FakeStream, wait_s=0.5, **kw))
    cfg = dict(m.DEFAULTS, backend="qsh", axes="x,y,z", gyro_bias=[0, 0, 0], calibrate_seconds=0)
    d = m.Imu(50, config=cfg, start_thread=False).sample()
    assert d["accelero"][2] == pytest.approx(9.9) and d["gyro"][2] == pytest.approx(-0.003)

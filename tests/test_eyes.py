"""Jero's eyes: nearest-target detection, radar, head follow, and the dashboard server (fake camera)."""

import json
import math
import sys
import threading
import urllib.request
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "brain"))

import eyes
from skills import HEAD_UP

K = eyes.Intrinsics(fx=384.0, fy=384.0, cx=320.0, cy=240.0, width=640, height=480)


def scene(u=160, v=240, dist=0.8, radius=40, wall=2.5):
    yy, xx = np.mgrid[0:480, 0:640]
    depth = np.full((480, 640), wall, np.float32)
    depth[(xx - u) ** 2 + (yy - v) ** 2 < radius**2] = dist
    return depth


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_finds_nearest_blob_with_distance_box_and_bearing():
    t = eyes.find_target(scene(u=160, v=200, dist=0.8), K)
    assert t is not None and t.distance == pytest.approx(0.8, abs=0.01)
    x0, y0, x1, y1 = t.box
    assert x0 <= 160 <= x1 and y0 <= 200 <= y1 and (x1 - x0) < 110
    assert t.yaw > 0 and t.yaw == pytest.approx(math.atan((320 - 160) / 384), abs=0.05)  # left of centre
    assert t.pitch > 0  # above the image centre


def test_nothing_in_range_or_only_noise_means_no_target():
    assert eyes.find_target(np.full((480, 640), 4.5, np.float32), K) is None  # beyond FAR_M
    assert eyes.find_target(np.zeros((480, 640), np.float32), K) is None  # no depth at all
    speck = np.full((480, 640), 4.5, np.float32)
    speck[100:108, 100:108] = 0.7  # one cell: noise
    assert eyes.find_target(speck, K) is None


def test_radar_points_left_and_right():
    depth = np.full((480, 640), 0.0, np.float32)
    depth[:, :100] = 1.0  # obstacle on the left of the image
    pts = eyes.radar(depth, K)
    assert pts and all(x < 0 and z == pytest.approx(1.0) for x, z in pts)


def test_head_follow_turns_toward_target_clamps_and_returns():
    clock = Clock()
    f = eyes.HeadFollow(clock=clock)
    left = eyes.find_target(scene(u=100), K)
    for _ in range(100):
        pose = f.update(left)
    assert pose["head_yaw"] == pytest.approx(eyes.HeadFollow.YAW[1])  # clamped
    centred = eyes.Target(1.0, 320, 240, (0, 0, 1, 1), yaw=math.radians(1), pitch=0.0)
    assert f.update(centred)["head_yaw"] == pose["head_yaw"]  # inside the deadband: no twitch
    up = eyes.Target(1.0, 320, 100, (0, 0, 1, 1), yaw=0.0, pitch=0.3)
    assert f.update(up)["head_pitch"] * HEAD_UP > 0  # looks up
    clock.t += 1.0
    assert f.update(None)["head_yaw"] > 0.4  # just lost: hold
    clock.t += 5.0
    for _ in range(80):
        pose = f.update(None)
    assert abs(pose["head_yaw"]) < 0.01 and abs(pose["head_pitch"]) < 0.01  # back to straight ahead


def test_rotated_intrinsics():
    r = K.rotated_180()
    assert (r.cx, r.cy) == (639 - 320, 479 - 240)


def test_fake_source_has_a_target_moving_in_range():
    src = eyes.FakeSource()
    seen = [eyes.find_target(src.scene(t)[1], src.k) for t in np.linspace(0, 14, 15)]
    assert all(s is not None and 0.4 < s.distance < 1.9 for s in seen)
    assert len({round(s.yaw, 1) for s in seen}) > 3  # it wanders


cv2 = pytest.importorskip("cv2")


def test_dashboard_serves_page_status_and_mjpeg():
    class Fast(eyes.FakeSource):
        def read(self):
            return self.scene(0.5)

    sent = []

    class Client:
        def head(self, **kw):
            sent.append(kw)

    e = eyes.Eyes(Fast(), follow=eyes.HeadFollow(), client=Client())
    for _ in range(3):
        assert e.step()
    server = eyes.ThreadingHTTPServer(("127.0.0.1", 0), eyes.make_handler(e))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        page = urllib.request.urlopen(base + "/", timeout=5).read().decode()
        assert "/color.mjpg" in page and "/status.json" in page
        st = json.loads(urllib.request.urlopen(base + "/status.json", timeout=5).read())
        assert st["target"]["distance"] > 0 and st["follow"] and st["frame"] == 3 and st["radar"]
        jpg = urllib.request.urlopen(base + "/depth.jpg", timeout=5).read()
        assert jpg[:2] == b"\xff\xd8"
        with urllib.request.urlopen(base + "/color.mjpg", timeout=5) as r:
            assert r.headers["Content-Type"].startswith("multipart/x-mixed-replace")
            head = r.read(200)
            assert head.startswith(b"--frame") and b"image/jpeg" in head
    finally:
        server.shutdown()
        server.server_close()
    assert sent and set(sent[-1]) == {"neck_pitch", "head_pitch", "head_yaw", "head_roll"}

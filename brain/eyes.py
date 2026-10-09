#!/usr/bin/env python3
"""Jero's eyes: RealSense colour + depth as a live web dashboard, and optional head-follow.

Open http://<robot>:8080 in any browser on the same network (laptop on a projector, phones):
colour with the nearest person/object boxed and its distance, depth heatmap, a top-down radar and
the numbers. Runs in its own environment (setup: robot/setup_vision.sh), next to the walk:

    ~/.virtualenvs/jero-vision/bin/python ~/Jero/brain/eyes.py              # camera -> dashboard
    ~/.virtualenvs/jero-vision/bin/python ~/Jero/brain/eyes.py --fake       # no camera: test scene
    ~/.virtualenvs/jero-vision/bin/python ~/Jero/brain/eyes.py --follow     # + head turns to the nearest target
    ~/.virtualenvs/jero-vision/bin/python ~/Jero/brain/eyes.py --follow --dry-run   # log head commands only

--follow sends head poses over the Jero link (the walk must run with the link on and be unpaused);
the walk keeps balancing, vision only moves the head.

Commands card: the page's buttons (capture image, say hi, dance, stop) POST /api/command. Capture is
done here (3-2-1 on the feed, saved to ~/jero-photos/); the others go to brain/show.py, which speaks
and moves the robot and posts what Jero said back (/api/say, robot only, brain/show_api.py).
Path card: the walk sends commanded velocity + gyro yaw rate over UDP (robot/telemetry.py, port
5006); the dashboard integrates it into the covered path (an estimate: commands, not measured steps).
Camera upside down? --rotate 180.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import re
import socket
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(1, str(HERE.parent / "jero_link" / "src"))

from skills import HEAD_UP, NEUTRAL

log = logging.getLogger("jero.eyes")
PAGE = HERE / "eyes.html"
NEAR_M, FAR_M = 0.25, 3.0  # target search range
CELL = 8  # detection grid: 8x8 pixel cells
MIN_CELLS = 6  # smaller blobs are noise


# -- detection (numpy only) -----------------------------------------------------------------
@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def rotated_180(self) -> Intrinsics:
        return Intrinsics(
            self.fx, self.fy, self.width - 1 - self.cx, self.height - 1 - self.cy, self.width, self.height
        )


@dataclass
class Target:
    distance: float  # m, median over the blob
    u: float  # pixel centre
    v: float
    box: tuple  # x0, y0, x1, y1 pixels
    yaw: float  # rad, + = target is to the robot's left
    pitch: float  # rad, + = target is above the camera axis


def grid(depth: np.ndarray, cell: int = CELL) -> np.ndarray:
    """Mean depth of each cell over its valid pixels (inf where under half the pixels are valid)."""
    h, w = depth.shape[0] // cell * cell, depth.shape[1] // cell * cell
    blocks = depth[:h, :w].reshape(h // cell, cell, w // cell, cell).swapaxes(1, 2).reshape(h // cell, w // cell, -1)
    valid = (blocks > NEAR_M) & (blocks < FAR_M)
    count = valid.sum(axis=2)
    mean = np.where(valid, blocks, 0).sum(axis=2) / np.maximum(count, 1)
    return np.where(count >= cell * cell // 2, mean, np.inf)


def find_target(depth: np.ndarray, k: Intrinsics, band: float = 0.3) -> Target | None:
    """The nearest blob: cells within ``band`` m of the nearest cell, connected to it."""
    g = grid(depth)
    if not np.isfinite(g).any():
        return None
    start = np.unravel_index(np.argmin(g), g.shape)
    near = g <= g[start] + band
    seen = np.zeros_like(near)
    stack, cells = [start], []
    seen[start] = True
    while stack:  # flood fill (the grid is 60x80: cheap)
        r, c = stack.pop()
        cells.append((r, c))
        for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= rr < g.shape[0] and 0 <= cc < g.shape[1] and near[rr, cc] and not seen[rr, cc]:
                seen[rr, cc] = True
                stack.append((rr, cc))
    if len(cells) < MIN_CELLS:
        return None
    rows, cols = np.array(cells).T
    u, v = (cols.mean() + 0.5) * CELL, (rows.mean() + 0.5) * CELL
    box = (int(cols.min() * CELL), int(rows.min() * CELL), int((cols.max() + 1) * CELL), int((rows.max() + 1) * CELL))
    return Target(
        distance=float(np.median(g[rows, cols])),
        u=float(u),
        v=float(v),
        box=box,
        yaw=float(-math.atan((u - k.cx) / k.fx)),
        pitch=float(-math.atan((v - k.cy) / k.fy)),
    )


def radar(depth: np.ndarray, k: Intrinsics, columns: int = 48) -> list:
    """Top-down obstacle points [x (m, + = right), z (m, forward)]: nearest in each image column band."""
    h, w = depth.shape
    mid = depth[h // 3 : 2 * h // 3]
    pts = []
    for i in range(columns):
        u0, u1 = i * w // columns, (i + 1) * w // columns
        strip = mid[:, u0:u1]
        ok = strip[(strip > NEAR_M) & (strip < 5.0)]
        if ok.size < strip.size // 10:
            continue
        z = float(np.percentile(ok, 5))
        x = ((u0 + u1) / 2 - k.cx) / k.fx * z
        pts.append([round(x, 3), round(z, 3)])
    return pts


# -- head follow ----------------------------------------------------------------------------
class HeadFollow:
    """Turns the head (which carries the camera) toward the target, a little each frame."""

    YAW = (-0.45, 0.45)
    PITCH_UP, PITCH_DOWN = 0.5, 0.25  # rad of head tilt allowed up / down

    def __init__(
        self,
        gain: float = 0.35,
        deadband: float = math.radians(3),
        max_step: float = 0.05,
        lost_s: float = 2.0,
        clock=time.monotonic,
    ):
        self.gain, self.deadband, self.max_step, self.lost_s, self.clock = gain, deadband, max_step, lost_s, clock
        self.yaw = 0.0
        self.up = 0.0  # + = looking up
        self.last_seen = -1e9

    def _step(self, err: float) -> float:
        if abs(err) < self.deadband:
            return 0.0
        return max(-self.max_step, min(self.max_step, self.gain * err))

    def update(self, target: Target | None) -> dict:
        now = self.clock()
        if target is not None:
            self.last_seen = now
            self.yaw = min(max(self.yaw + self._step(target.yaw), self.YAW[0]), self.YAW[1])
            self.up = min(max(self.up + self._step(target.pitch), -self.PITCH_DOWN), self.PITCH_UP)
        elif now - self.last_seen > self.lost_s:  # nobody there: drift back to straight ahead
            self.yaw *= 0.92
            self.up *= 0.92
        return {**NEUTRAL, "head_yaw": round(self.yaw, 3), "head_pitch": round(self.up * HEAD_UP, 3)}


# -- frame sources --------------------------------------------------------------------------
class RealSenseSource:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 15):
        import pyrealsense2 as rs

        self.rs = rs
        self.pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        prof = self.pipe.start(cfg)
        dev = prof.get_device()
        self.name = dev.get_info(rs.camera_info.name)
        self.scale = dev.first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)
        i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.k = Intrinsics(i.fx, i.fy, i.ppx, i.ppy, i.width, i.height)

    def read(self):
        fs = self.align.process(self.pipe.wait_for_frames(2000))
        d, c = fs.get_depth_frame(), fs.get_color_frame()
        if not d or not c:
            return None
        return np.asanyarray(c.get_data()).copy(), np.asanyarray(d.get_data()).astype(np.float32) * self.scale

    def close(self):
        self.pipe.stop()


class FakeSource:
    """A wall at 2.5 m, a floor, and a 'person' wandering between 0.5 and 1.8 m. No camera needed."""

    name = "test scene (--fake)"

    def __init__(self, width: int = 640, height: int = 480, fps: int = 15, clock=time.monotonic):
        self.k = Intrinsics(width * 0.6, width * 0.6, width / 2, height / 2, width, height)
        self.period, self.clock, self.t0 = 1.0 / fps, clock, clock()
        self.yy, self.xx = np.mgrid[0:height, 0:width].astype(np.float32)

    def scene(self, t: float):
        w, h = self.k.width, self.k.height
        depth = np.full((h, w), 2.5, np.float32) + 0.15 * np.sin(self.xx / 90)
        floor = self.yy > h * 0.7
        depth[floor] = (1.2 * h / (self.yy[floor] - h * 0.5 + 1)).clip(0.6, 4)
        pu, pv = w * (0.5 + 0.33 * math.sin(t * 0.7)), h * (0.45 + 0.12 * math.sin(t * 1.3))
        pd = 1.15 + 0.65 * math.sin(t * 0.45)
        r = 70 / pd
        body = (self.xx - pu) ** 2 + ((self.yy - pv) / 1.6) ** 2 < r * r
        depth[body] = pd
        color = np.zeros((h, w, 3), np.uint8)
        color[..., 0] = (40 + 50 * self.yy / h).astype(np.uint8)
        color[..., 1] = (35 + 30 * self.xx / w).astype(np.uint8)
        color[..., 2] = 30
        color[floor] = (60, 70, 80)
        color[body] = (60, 140, 230)
        return color, depth

    def read(self):
        time.sleep(self.period)
        return self.scene(self.clock() - self.t0)

    def close(self):
        pass


# -- rendering (OpenCV) ---------------------------------------------------------------------
def render(color: np.ndarray, depth: np.ndarray, target: Target | None, quality: int = 70):
    """JPEG bytes of the annotated colour image and the depth heatmap."""
    import cv2

    near_red = cv2.convertScaleAbs((4 - np.clip(depth, 0, 4)) * (255 / 4))  # near = red, far = blue
    heat = cv2.applyColorMap(near_red, cv2.COLORMAP_TURBO)
    heat[(depth <= 0) | ~np.isfinite(depth)] = (18, 18, 18)
    col = color.copy()
    h, w = col.shape[:2]
    cv2.drawMarker(col, (w // 2, h // 2), (200, 200, 200), cv2.MARKER_CROSS, 18, 1)
    for img in (col, heat):
        if target is not None:
            x0, y0, x1, y1 = target.box
            cv2.rectangle(img, (x0, y0), (x1, y1), (40, 220, 255), 2)
            label = f"{target.distance:.2f} m"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            ty = max(y0 - 8, th + 6)
            cv2.rectangle(img, (x0, ty - th - 6), (x0 + tw + 10, ty + 4), (40, 220, 255), -1)
            cv2.putText(img, label, (x0 + 5, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 2, cv2.LINE_AA)
    enc = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    return cv2.imencode(".jpg", col, enc)[1].tobytes(), cv2.imencode(".jpg", heat, enc)[1].tobytes()


# -- show events (speech, captions, photos, ...) ----------------------------------------------
class Events:
    """Recent dashboard events with increasing ids; the page follows them over /api/events (SSE)."""

    def __init__(self, maxlen: int = 100, clock=time.time):
        self.cond = threading.Condition()
        self.items: deque = deque(maxlen=maxlen)
        self.seq = 0
        self.clock = clock

    def push(self, kind: str, **data) -> dict:
        with self.cond:
            self.seq += 1
            ev = {"id": self.seq, "type": kind, "t": round(self.clock(), 3), **data}
            self.items.append(ev)
            self.cond.notify_all()
        log.info("event %s %s", kind, {k: v for k, v in data.items() if k != "url"})
        return ev

    def since(self, after: int, timeout: float = 15.0) -> list:
        with self.cond:
            self.cond.wait_for(lambda: self.seq > after, timeout=timeout)
            return [e for e in self.items if e["id"] > after]


PHOTO_NAME = re.compile(r"^jero-\d{8}-\d{6}(-\d+)?\.jpg$")
EVENT_KINDS = ("say", "caption", "mode", "photo")
COMMANDS = ("capture", "hi", "dance", "stop")  # the page's buttons; anyone viewing may press them


class PathTracker:
    """Covered path from walk telemetry: heading from the gyro (commanded turn rate if none),
    position from the commanded velocity. An estimate: the robot may walk slower than commanded."""

    def __init__(self, speed_scale: float = 1.0, min_step_m: float = 0.02, max_points: int = 3000):
        self.speed_scale, self.min_step_m = speed_scale, min_step_m
        self.lock = threading.Lock()
        self.max_points = max_points
        self.reset()

    def reset(self) -> None:
        with getattr(self, "lock", threading.Lock()):
            self.x = self.y = self.heading = self.distance = 0.0
            self.points = deque([(0.0, 0.0)], maxlen=self.max_points)
            self.last_t = None
            self.last_msg = 0.0
            self.paused = True

    def update(self, msg: dict, now: float | None = None) -> None:
        t = float(msg.get("t", time.time()))
        with self.lock:
            self.last_msg = time.time() if now is None else now
            self.paused = bool(msg.get("paused"))
            dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.5)
            self.last_t = t
            if self.paused or dt == 0.0:
                return
            wz = msg.get("gz")
            wz = float(msg.get("wz", 0.0)) if wz is None else float(wz)
            self.heading += wz * dt
            vx, vy = float(msg.get("vx", 0.0)) * self.speed_scale, float(msg.get("vy", 0.0)) * self.speed_scale
            c, s_ = math.cos(self.heading), math.sin(self.heading)
            dx, dy = (vx * c - vy * s_) * dt, (vx * s_ + vy * c) * dt
            self.x += dx
            self.y += dy
            self.distance += math.hypot(dx, dy)
            px, py = self.points[-1]
            if math.hypot(self.x - px, self.y - py) >= self.min_step_m:
                self.points.append((round(self.x, 3), round(self.y, 3)))

    def state(self, max_points: int = 600, now: float | None = None) -> dict:
        with self.lock:
            pts = list(self.points) + [(round(self.x, 3), round(self.y, 3))]
            step = max(1, len(pts) // max_points)
            now = time.time() if now is None else now
            return {
                "points": pts[::step] + ([pts[-1]] if (len(pts) - 1) % step else []),
                "pose": [round(self.x, 3), round(self.y, 3), round(self.heading, 3)],
                "distance": round(self.distance, 2),
                "live": now - self.last_msg < 1.0,
                "paused": self.paused,
            }


def listen_telemetry(tracker: PathTracker, port: int = 5006, host: str = "127.0.0.1") -> threading.Thread:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))

    def loop():
        while True:
            data, _ = sock.recvfrom(2048)
            try:
                tracker.update(json.loads(data))
            except (ValueError, TypeError) as exc:
                log.debug("bad telemetry: %s", exc)

    th = threading.Thread(target=loop, name="jero-telemetry-rx", daemon=True)
    th.start()
    log.info("path card: listening for walk telemetry on udp://%s:%d", host, port)
    return th


# -- the loop + shared state ----------------------------------------------------------------
class Eyes:
    def __init__(
        self,
        source,
        rotate: int = 0,
        follow: HeadFollow | None = None,
        client=None,
        photos_dir: Path | None = None,
        path: PathTracker | None = None,
        clock=time.monotonic,
    ):
        self.source, self.rotate, self.follow, self.client = source, rotate, follow, client
        self.k = source.k.rotated_180() if rotate == 180 else source.k
        self.photos_dir = Path(photos_dir or Path.home() / "jero-photos")
        self.path, self.clock = path or PathTracker(), clock
        self.events = Events()
        self.cond = threading.Condition()
        self.seq = 0
        self.jpeg = {"color": b"", "depth": b""}
        self.last_color = None
        self.status = {
            "camera": source.name,
            "fps": 0.0,
            "target": None,
            "radar": [],
            "follow": bool(follow),
            "head": None,
            "frame": 0,
            "size": [self.k.width, self.k.height],
            "mode": "standby",
            "say": None,
        }
        self._times = deque(maxlen=30)
        self._running = False

    def step(self) -> bool:
        frames = self.source.read()
        if frames is None:
            return False
        color, depth = frames
        if self.rotate == 180:
            color, depth = color[::-1, ::-1], depth[::-1, ::-1]
        target = find_target(depth, self.k)
        head = None
        if self.follow is not None:
            head = self.follow.update(target)
            if self.client is not None:
                self.client.head(**head)
        cj, dj = render(color, depth, target)
        self._times.append(time.monotonic())
        fps = (len(self._times) - 1) / (self._times[-1] - self._times[0]) if len(self._times) > 1 else 0.0
        with self.cond:
            self.seq += 1
            self.jpeg = {"color": cj, "depth": dj}
            self.last_color = color
            self.status.update(
                fps=round(fps, 1),
                frame=self.seq,
                radar=radar(depth, self.k),
                head=head,
                target=None
                if target is None
                else {
                    **asdict(target),
                    "yaw_deg": round(math.degrees(target.yaw), 1),
                    "pitch_deg": round(math.degrees(target.pitch), 1),
                },
            )
            self.cond.notify_all()
        return True

    # -- show actions (from /api/*: the show controller, voice, a terminal) -----------------
    def say(self, text: str) -> dict:
        with self.cond:
            self.status["say"] = text
        return self.events.push("say", text=text)

    def caption(self, text: str, seconds: float = 4.0, style: str = "info") -> dict:
        return self.events.push("caption", text=text, seconds=seconds, style=style)

    def command(self, name: str) -> dict:
        """A button on the page. Capture is done here; the rest is for brain/show.py."""
        if name == "capture":
            return self.photo(countdown=3)
        return self.events.push("command", name=name)

    def set_mode(self, mode: str) -> dict:
        with self.cond:
            self.status["mode"] = mode
        return self.events.push("mode", mode=mode)

    def photo(self, countdown: int = 3, wait: bool = False) -> dict:
        """Countdown on the page, then save the plain colour frame to photos_dir and show it."""
        countdown = max(0, min(int(countdown), 10))
        ev = self.events.push("countdown", seconds=countdown)

        def shoot():
            time.sleep(countdown)
            with self.cond:
                frame = self.last_color
            if frame is None:
                self.events.push("caption", text="No picture: camera not ready", seconds=3, style="warn")
                return
            name = self.save_photo(frame)
            self.events.push("photo", name=name, url=f"/photos/{name}")

        if wait:
            shoot()
        else:
            threading.Thread(target=shoot, name="jero-photo", daemon=True).start()
        return ev

    def save_photo(self, frame: np.ndarray) -> str:
        import cv2

        self.photos_dir.mkdir(parents=True, exist_ok=True)
        stem = time.strftime("jero-%Y%m%d-%H%M%S")
        name, k = f"{stem}.jpg", 1
        while (self.photos_dir / name).exists():
            name, k = f"{stem}-{k}.jpg", k + 1
        cv2.imwrite(str(self.photos_dir / name), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        return name

    def photos(self, limit: int = 24) -> list:
        if not self.photos_dir.is_dir():
            return []
        files = sorted((f for f in self.photos_dir.glob("jero-*.jpg") if PHOTO_NAME.match(f.name)), reverse=True)
        return [{"name": f.name, "url": f"/photos/{f.name}"} for f in files[:limit]]

    def run(self):
        self._running = True
        while self._running:
            try:
                self.step()
            except Exception as exc:  # noqa: BLE001  keep serving; a USB hiccup shouldn't kill the page
                log.warning("frame failed: %s", exc)
                time.sleep(0.5)

    def stop(self):
        self._running = False

    def snapshot(self) -> dict:
        with self.cond:
            return {**self.status, "path": self.path.state()}

    def wait_frame(self, after: int, timeout: float = 2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq > after, timeout=timeout)
            return self.seq, self.jpeg


def make_handler(eyes: Eyes, allow_remote: bool = False):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # quiet: one line per client, not per request
            pass

        def _send(self, body: bytes, ctype: str, code: int = 200):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code: int = 200):
            self._send(json.dumps(obj).encode(), "application/json", code)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self._send(PAGE.read_bytes(), "text/html; charset=utf-8")
            elif path == "/status.json":
                self._send(json.dumps(eyes.snapshot()).encode(), "application/json")
            elif path in ("/color.mjpg", "/depth.mjpg"):
                self._stream(path[1:6])
            elif path in ("/color.jpg", "/depth.jpg"):
                _, jpeg = eyes.wait_frame(0)
                self._send(jpeg[path[1:6]], "image/jpeg")
            elif path == "/api/events":
                self._events()
            elif path == "/api/photos":
                self._json(eyes.photos())
            elif path.startswith("/photos/") and PHOTO_NAME.match(path[8:]):
                f = eyes.photos_dir / path[8:]
                if f.is_file():
                    self._send(f.read_bytes(), "image/jpeg")
                else:
                    self.send_error(404)
            else:
                self.send_error(404)

        def do_POST(self):
            kind = self.path.split("?")[0].removeprefix("/api/")
            local = self.client_address[0] in ("127.0.0.1", "::1")
            if kind not in ("command", "path/reset") and not (allow_remote or local):
                self._json({"error": "only the robot itself can post this"}, 403)
                return
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(min(n, 8192)) or b"{}") if n else {}
            except ValueError:
                self._json({"error": "body must be JSON"}, 400)
                return
            text = str(body.get("text", ""))[:300]
            if kind == "command" and body.get("name") in COMMANDS:
                ev = eyes.command(body["name"])
            elif kind == "path/reset":
                eyes.path.reset()
                ev = eyes.events.push("path_reset")
            elif kind == "say" and text:
                ev = eyes.say(text)
            elif kind == "caption" and text:
                ev = eyes.caption(text, float(body.get("seconds", 4.0)), str(body.get("style", "info"))[:16])
            elif kind == "mode" and body.get("mode"):
                ev = eyes.set_mode(str(body["mode"])[:40])
            elif kind == "photo":
                ev = eyes.photo(int(body.get("countdown", 3)))
            else:
                self._json(
                    {
                        "error": "POST /api/command {name: capture|hi|dance|stop}, /api/path/reset, "
                        "/api/say|caption|mode|photo"
                    },
                    400,
                )
                return
            self._json(ev)

        def _events(self):
            query = self.path.partition("?")[2]
            after = None
            for part in query.split("&"):
                if part.startswith("after="):
                    try:
                        after = int(part[6:])
                    except ValueError:
                        pass
            if after is None:
                after = eyes.events.seq  # new viewers: only what happens from now on
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(b"retry: 1500\n\n")
                while True:
                    items = eyes.events.since(after, timeout=15.0)
                    if not items:
                        self.wfile.write(b": ping\n\n")
                    for ev in items:
                        after = ev["id"]
                        self.wfile.write(f"id: {ev['id']}\ndata: {json.dumps(ev)}\n\n".encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _stream(self, which: str):
            log.info("viewer %s: %s stream", self.client_address[0], which)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = 0
            try:
                while True:
                    seq, jpeg = eyes.wait_frame(seq)
                    data = jpeg[which]
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(data))
                    self.wfile.write(data + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def avoid_fastest_core() -> None:
    """Keep off the core robot/jero.sh pins the walk to (highest cpu_capacity)."""
    caps = {}
    for f in Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpu_capacity"):
        try:
            caps[int(f.parent.name[3:])] = int(f.read_text())
        except (OSError, ValueError):
            pass
    if len(caps) > 1 and hasattr(os, "sched_setaffinity"):
        fastest = max(caps, key=caps.get)
        os.sched_setaffinity(0, set(caps) - {fastest})
        log.info("vision on CPUs %s (cpu%d left to the walk)", sorted(set(caps) - {fastest}), fastest)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--fake", action="store_true", help="synthetic scene instead of the camera")
    p.add_argument("--bind", default="0.0.0.0", help="dashboard address (0.0.0.0 = every network)")
    p.add_argument("--http-port", type=int, default=8080)
    p.add_argument("--fps", type=int, default=15, choices=(5, 15, 30))
    p.add_argument("--rotate", type=int, default=0, choices=(0, 180))
    p.add_argument("--follow", action="store_true", help="turn the head toward the nearest target (Jero link)")
    p.add_argument("--dry-run", action="store_true", help="--follow: log head commands, don't send them")
    p.add_argument("--telemetry-port", type=int, default=5006, help="walk telemetry (UDP) for the path card")
    p.add_argument("--path-scale", type=float, default=1.0, help="path card: real speed / commanded speed")
    p.add_argument("--photos", default=str(Path.home() / "jero-photos"), help="where photos are saved")
    p.add_argument("--allow-remote", action="store_true", help="accept show actions (POST) from other machines")
    p.add_argument("--host", default="127.0.0.1", help="robot running jero_walk.py with the link on")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--key", default=os.path.expanduser("~/.config/jero/link.key"))
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    avoid_fastest_core()
    source = FakeSource(fps=args.fps) if args.fake else RealSenseSource(fps=args.fps)
    log.info("camera: %s, %dx%d", source.name, source.k.width, source.k.height)
    client = None
    if args.follow and not args.dry_run:
        from jero_link import JeroClient, load_key

        client = JeroClient(args.host, port=args.port, key=load_key(args.key)).start()
    elif args.follow:

        class LogClient:
            def head(self, **kw):
                log.info("head yaw=%+.2f pitch=%+.2f", kw["head_yaw"], kw["head_pitch"])

        client = LogClient()
    path = PathTracker(speed_scale=args.path_scale)
    try:
        listen_telemetry(path, args.telemetry_port)
    except OSError as exc:
        log.warning("path card off: can't listen on udp %d (%s)", args.telemetry_port, exc)
    eyes = Eyes(
        source,
        rotate=args.rotate,
        follow=HeadFollow() if args.follow else None,
        client=client,
        photos_dir=Path(args.photos).expanduser(),
        path=path,
    )
    threading.Thread(target=eyes.run, name="jero-eyes", daemon=True).start()

    server = ThreadingHTTPServer((args.bind, args.http_port), make_handler(eyes, allow_remote=args.allow_remote))
    server.daemon_threads = True
    shown = "<this machine's IP>" if args.bind == "0.0.0.0" else args.bind
    log.info("dashboard: http://%s:%d/  (photos: %s)", shown, args.http_port, eyes.photos_dir)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        eyes.stop()
        server.server_close()
        if client is not None and hasattr(client, "close"):
            client.head(**NEUTRAL)
            client.close()
        source.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

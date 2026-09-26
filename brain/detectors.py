"""Person detectors for brain/follow.py.

A detector is ``callable(frame_bgr) -> (Target | None, (x, y, w, h) | None)``.
``hog`` needs nothing beyond OpenCV and runs anywhere, but it is slow (~5-10 FPS on CPU)
and weak at a distance. Swap in a TensorRT detector on the Orin for the event.
"""

from __future__ import annotations

from follow_control import Target


def _largest(boxes):
    return max(boxes, key=lambda b: b[2] * b[3]) if len(boxes) else None


def hog_detector():
    import cv2

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect(frame):
        h, w = frame.shape[:2]
        scale = 480.0 / max(h, w)
        small = cv2.resize(frame, (int(w * scale), int(h * scale))) if scale < 1 else frame
        s = scale if scale < 1 else 1.0
        boxes, _ = hog.detectMultiScale(small, winStride=(8, 8), padding=(8, 8), scale=1.05)
        box = _largest(boxes)
        if box is None:
            return None, None
        x, y, bw, bh = (int(v / s) for v in box)
        return Target(cx=(x + bw / 2) / w, area=(bw * bh) / float(w * h)), (x, y, bw, bh)

    return detect


def make_detector(name: str):
    if name == "hog":
        return hog_detector()
    raise ValueError(f"unknown detector {name!r}")

#!/usr/bin/env python3
"""Follow-a-person behaviour for the demo: camera on the Jetson -> detector -> Jero link.

The control law (``FollowController``) is plain Python and unit-tested; the detector is
pluggable (see ``detectors.py``). Start with ``--dry-run`` to watch commands without moving.

    python brain/follow.py jero.local --camera 0 --dry-run
    python brain/follow.py jero.local --camera "nvarguscamerasrc ! ... ! appsink"   # CSI via GStreamer
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jero_link" / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from follow_control import FollowController  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("host")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--key", default=os.path.expanduser("~/.config/jero/link.key"))
    p.add_argument("--camera", default="0", help="index, file, or GStreamer pipeline")
    p.add_argument("--detector", default="hog", choices=["hog"], help="add yours in detectors.py")
    p.add_argument("--dry-run", action="store_true", help="print commands, don't send")
    p.add_argument("--show", action="store_true", help="show an OpenCV window")
    args = p.parse_args()

    import cv2  # JetPack ships OpenCV; on a laptop: pip install opencv-python

    from detectors import make_detector

    src = int(args.camera) if args.camera.isdigit() else args.camera
    backend = cv2.CAP_GSTREAMER if isinstance(src, str) and "!" in src else cv2.CAP_ANY
    cap = cv2.VideoCapture(src, backend)
    if not cap.isOpened():
        sys.exit(f"cannot open camera {args.camera!r}")

    detector = make_detector(args.detector)
    ctrl = FollowController()
    duck = None
    if not args.dry_run:
        from jero_link import JeroClient, load_key

        key = None if args.key.lower() == "none" else load_key(args.key)
        duck = JeroClient(args.host, args.port, key=key).start()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            target, box = detector(frame)
            vx, wz = ctrl.update(target)
            if duck:
                duck.drive(vx=vx, wz=wz)
            else:
                print(f"target={target} -> vx={vx:+.3f} wz={wz:+.3f}")
            if args.show:
                if box is not None:
                    x, y, w, h = box
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.imshow("jero follow", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if duck:
            duck.close()
        cap.release()


if __name__ == "__main__":
    main()

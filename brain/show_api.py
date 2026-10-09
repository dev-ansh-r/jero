#!/usr/bin/env python3
"""Drive the dashboard (brain/eyes.py) from the robot: speech bubble, captions, mode, photos.

    python ~/Jero/brain/show_api.py say "Hello everyone! Jero, I am."
    python ~/Jero/brain/show_api.py caption "DANCE MODE" --seconds 6 --style party
    python ~/Jero/brain/show_api.py mode dancing
    python ~/Jero/brain/show_api.py photo --countdown 3

From Python: ``from show_api import Dashboard; Dashboard().say("Hi!")``. Every call is best effort:
if the dashboard isn't running it logs once and returns None, so the show never stops for it.
Only works from the robot itself (the dashboard refuses show actions from other machines).
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import urllib.request

log = logging.getLogger("jero.show_api")
STYLES = ("info", "warn", "party", "system")


class Dashboard:
    def __init__(self, base: str = "http://127.0.0.1:8080", timeout_s: float = 1.0, urlopen=urllib.request.urlopen):
        self.base, self.timeout_s, self._urlopen = base.rstrip("/"), timeout_s, urlopen
        self._warned = False

    def _post(self, kind: str, body: dict) -> dict | None:
        req = urllib.request.Request(
            f"{self.base}/api/{kind}",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with self._urlopen(req, timeout=self.timeout_s) as resp:
                self._warned = False
                return json.loads(resp.read() or b"{}")
        except Exception as exc:  # noqa: BLE001  the dashboard is optional
            if not self._warned:
                log.warning("dashboard not reachable at %s (%s): carrying on without it", self.base, exc)
                self._warned = True
            return None

    def say(self, text: str):
        return self._post("say", {"text": text})

    def caption(self, text: str, seconds: float = 4.0, style: str = "info"):
        return self._post("caption", {"text": text, "seconds": seconds, "style": style})

    def mode(self, mode: str):
        return self._post("mode", {"mode": mode})

    def photo(self, countdown: int = 3):
        return self._post("photo", {"countdown": countdown})


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("action", choices=("say", "caption", "mode", "photo"))
    p.add_argument("text", nargs="?", default="", help="say/caption text, or the mode name")
    p.add_argument("--seconds", type=float, default=4.0)
    p.add_argument("--style", choices=STYLES, default="info")
    p.add_argument("--countdown", type=int, default=3)
    p.add_argument("--url", default="http://127.0.0.1:8080")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    d = Dashboard(args.url, timeout_s=3.0)
    if args.action in ("say", "caption", "mode") and not args.text:
        p.error(f"{args.action} needs text")
    out = {
        "say": lambda: d.say(args.text),
        "caption": lambda: d.caption(args.text, args.seconds, args.style),
        "mode": lambda: d.mode(args.text),
        "photo": lambda: d.photo(args.countdown),
    }[args.action]()
    print(json.dumps(out) if out else "failed")
    return 0 if out else 1


if __name__ == "__main__":
    sys.exit(main())

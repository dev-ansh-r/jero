#!/usr/bin/env python3
"""Emergency pause from any laptop on the robot's network. Keep this one command ready at the demo.

    python tools/estop.py jero.local            # pause the walk (resume with Xbox A or --resume)
    python tools/estop.py jero.local --resume
"""

import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jero_link" / "src"))

from jero_link import JeroClient, load_key


def main():
    p = argparse.ArgumentParser()
    p.add_argument("host")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--key", default=os.path.expanduser("~/.config/jero/link.key"))
    p.add_argument("--resume", action="store_true", help="send an 'A' press (toggles pause)")
    args = p.parse_args()

    key = None if args.key.lower() == "none" else load_key(args.key)
    client = JeroClient(args.host, args.port, key=key)
    if args.resume:
        client.press("A")
    else:
        client.estop()
    for _ in range(3):  # a few frames so one lost datagram doesn't matter
        client.send_once()
        time.sleep(0.05)
    client.sock.close()
    print("resume sent" if args.resume else "E-STOP sent")


if __name__ == "__main__":
    main()

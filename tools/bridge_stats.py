#!/usr/bin/env python3
"""Read the Pico bridge's error counters (firmware/pico_bridge, counters since its last reset).

Run it right after tools/bus_test.py (not while the walk is running: only one program can own the
port). Linux only, standard library only.

    python ~/Jero/tools/bridge_stats.py

The bridge answers a broadcast packet with instruction 0xF0 itself; it never reaches the servos.
"""

from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import termios
import time

FIELDS = (
    ("requests", "valid packets from the Pi"),
    ("badChecksum", "packets from the Pi with a wrong checksum -> marker sent"),
    ("stalled", "packets from the Pi that stopped half way -> marker sent"),
    ("junkBytes", "bytes from the Pi outside any packet"),
    ("replyErrors", "servo reply missing/corrupt -> marker sent"),
    ("okBatches", "transactions answered in full"),
    ("usbShort", "USB writes that lost bytes"),
    ("watchdogResets", "times the Pico rebooted itself because its loop got stuck (since power-on)"),
)


def checksum(body: bytes) -> int:
    return ~sum(body) & 0xFF


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    args = p.parse_args()

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0  # raw: no input/output/local processing
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = termios.B115200  # never 1200: that reboots the Pico into its bootloader
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)

        body = bytes([0xFE, 0x02, 0xF0])
        os.write(fd, b"\xff\xff" + body + bytes([checksum(body)]))

        data, deadline = b"", time.monotonic() + 0.5
        while time.monotonic() < deadline:
            r, _, _ = select.select([fd], [], [], 0.05)
            if r:
                data += os.read(fd, 256)
            if len(data) >= 4 and len(data) >= 4 + data[3]:
                break
    finally:
        os.close(fd)

    n = 4 * len(FIELDS)
    if len(data) < 6 + n or data[:3] != b"\xff\xff\xfe" or data[3] != n + 2:
        print(f"no stats reply (got {data.hex(' ') or 'nothing'}): is the new bridge firmware flashed?")
        return 1
    if data[5 + n] != checksum(data[2 : 5 + n]):
        print("stats reply has a bad checksum")
        return 1
    values = struct.unpack("<" + "I" * len(FIELDS), data[5 : 5 + n])
    for (name, meaning), v in zip(FIELDS, values):
        print(f"  {name:12s} {v:8d}   {meaning}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

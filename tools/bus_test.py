#!/usr/bin/env python3
"""Servo bus check through the Pico bridge, with the same servo IO the walk uses (--io).

Reads only: nothing moves. Servos powered, Pico on USB.

    python ~/Jero/tools/bus_test.py                         # all 14 servos
    python ~/Jero/tools/bus_test.py --ids 10,11,12,13,14,20,21,22,23,24   # legs only

1. reliability: N rounds of sync-read position + velocity (what the walk does every step).
   --io jero (default, robot/feetech_io.py): pass = no read slower than 100 ms, errors <= 1 %
   (a lost USB reply costs one 20 ms step). --io rustypot: pass = 0 errors (each costs 1 s).
   Both: median well under 10 ms (the whole control step is 20 ms).
2. missing servo: one sync-read that includes an ID nobody has (99). With the Jero bridge it must
   fail FAST (well under 1 s) and the very next normal read must still work, with no panic.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ALL_IDS = [20, 21, 22, 23, 24, 30, 31, 32, 33, 10, 11, 12, 13, 14]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--ids", default=",".join(map(str, ALL_IDS)))
    p.add_argument("--rounds", type=int, default=300)
    p.add_argument(
        "--io", choices=("jero", "rustypot"), default="jero",
        help="jero = robot/feetech_io.py (what the walk uses, 25 ms timeout); rustypot = upstream (1 s)",
    )
    args = p.parse_args()
    ids = [int(x) for x in args.ids.split(",") if x]

    if args.io == "jero":
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot"))
        import feetech_io

        def open_io():
            return feetech_io.open_bus(args.port)
    else:
        import rustypot

        def open_io():
            return rustypot.feetech(args.port, 1000000)

    io = open_io()

    def reopen():
        nonlocal io
        io = None
        import gc

        gc.collect()
        io = open_io()

    # 1. reliability
    errors, panics, times = 0, 0, []
    for n in range(args.rounds):
        t0 = time.perf_counter()
        try:
            io.read_present_position(ids)
            io.read_present_velocity(ids)
        except (KeyboardInterrupt, SystemExit):
            raise
        except Exception as e:  # noqa: BLE001  counting every bus error is the point
            errors += 1
            if errors <= 5:
                print(f"  round {n}: {type(e).__name__}: {e}")
        except BaseException as e:  # noqa: BLE001  rustypot panics are BaseException
            panics += 1
            if panics <= 5:
                print(f"  round {n}: PANIC {e}")
            reopen()
        times.append((time.perf_counter() - t0) * 1000)
    times.sort()
    med, p95, worst = times[len(times) // 2], times[int(len(times) * 0.95)], times[-1]
    print(
        f"1. {args.rounds} rounds, {len(ids)} servos: errors {errors}, panics {panics}; "
        f"pos+vel median {med:.1f} ms, p95 {p95:.1f} ms, worst {worst:.1f} ms"
    )
    if args.io == "jero":
        # a lost USB reply is expected now and costs one 20 ms step: what matters is no long stall
        ok1 = panics == 0 and med < 10 and worst < 100 and errors <= args.rounds * 0.01
        print(f"   (jero io: pass = no panics, worst < 100 ms, errors <= 1 %; error rate {errors / args.rounds:.2%})")
    else:
        ok1 = errors == 0 and panics == 0 and med < 10

    # 2. missing servo fails fast, and the bus recovers
    t0 = time.perf_counter()
    try:
        io.read_present_position([*ids[:3], 99])
        print("2. read with missing ID 99 unexpectedly succeeded")
        failed_fast = False
    except (KeyboardInterrupt, SystemExit):
        raise
    except BaseException as e:  # noqa: BLE001  error or panic, both are reported
        dt = (time.perf_counter() - t0) * 1000
        failed_fast = dt < 200 and isinstance(e, Exception)
        kind = type(e).__name__ if isinstance(e, Exception) else "PANIC"
        print(f"2. read with missing ID 99 failed in {dt:.0f} ms ({kind}: {e})")
        if not isinstance(e, Exception):
            reopen()
    try:
        io.read_present_position(ids)
        recovered = True
        print("   next normal read: OK")
    except BaseException as e:  # noqa: BLE001
        recovered = False
        print(f"   next normal read FAILED: {type(e).__name__}: {e}")

    ok = ok1 and failed_fast and recovered
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

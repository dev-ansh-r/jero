#!/usr/bin/env python3
"""Run the upstream Open Duck Mini v2 RL walk with Jero's controller mux.

Same policy loop as upstream ``scripts/v2_rl_walk_mujoco.py``; the only change is where
commands come from: Xbox pad (priority) + Jero link over Wi-Fi (Jetson / laptop).

    python robot/jero_walk.py                       # Xbox + link, signed with ~/.config/jero/link.key
    python robot/jero_walk.py --no-xbox             # link only (keep a laptop ready with tools/estop)
    python robot/jero_walk.py --imu mpu6050         # GY-521 instead of BNO055 (run tools/imu_check.py first)
    python robot/jero_walk.py --missing-servos 30,31,32,33   # bench: head chain not wired (stand only)
    python robot/jero_walk.py --pad ps4             # DualShock 4 (auto-detected by name otherwise)
    python robot/jero_walk.py --board other --imu mpu6050   # RB3 Gen 2 etc. (auto-detected); see docs/rb3gen2.md
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

HOME = Path.home()
REPO = Path(__file__).resolve().parents[1]
DEFAULT_KEY = HOME / ".config" / "jero" / "link.key"


def resolve_runtime(explicit: str | None) -> Path:
    candidates = [
        explicit,
        os.environ.get("JERO_RUNTIME_DIR"),
        HOME / "Open_Duck_Mini_Runtime",  # pre-built Duck image location
        REPO / "upstream" / "Open_Duck_Mini_Runtime",  # submodule
    ]
    for c in candidates:
        if c and (Path(c) / "scripts" / "v2_rl_walk_mujoco.py").is_file():
            return Path(c).resolve()
    sys.exit("Open_Duck_Mini_Runtime not found: pass --runtime-dir or set JERO_RUNTIME_DIR")


def resolve_policy(explicit: str | None) -> Path:
    candidates = [
        explicit,
        HOME / "BEST_WALK_ONNX_2.onnx",
        REPO / "upstream" / "Open_Duck_Mini" / "BEST_WALK_ONNX_2.onnx",
    ]
    for c in candidates:
        if c and Path(c).is_file():
            return Path(c).resolve()
    sys.exit("policy .onnx not found: pass --onnx_model_path (see tools/fetch_policy.sh)")


def install_mpu6050(log) -> None:
    """Make upstream's ``from mini_bdx_runtime.raw_imu import Imu`` load our MPU6050 driver."""
    import imu_mpu6050

    cfg = imu_mpu6050.load_config()
    if not imu_mpu6050.CONFIG_PATH.is_file():
        sys.exit(f"{imu_mpu6050.CONFIG_PATH} missing: run tools/imu_check.py once (measures the mounting)")
    imu_mpu6050.parse_axes(cfg["axes"])  # fail early on a bad file
    sys.modules["mini_bdx_runtime.raw_imu"] = imu_mpu6050
    backend = "RB3 onboard (IIO)" if cfg.get("backend") == "iio" else "MPU6050"
    log.info("IMU: %s axes=%s (keep the robot still for 2 s at start: gyro bias)", backend, cfg["axes"])


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--onnx_model_path", default=None)
    p.add_argument("--duck_config_path", default=str(HOME / "duck_config.json"))
    p.add_argument("--runtime-dir", default=None)
    p.add_argument("--serial-port", default="/dev/ttyACM0")
    p.add_argument(
        "--bus",
        choices=("jero", "rustypot"),
        default=os.environ.get("JERO_BUS", "jero"),
        help="servo IO: jero = robot/feetech_io.py (25 ms timeout), rustypot = upstream (1 s timeout)",
    )
    p.add_argument(
        "--imu",
        choices=("bno055", "mpu6050"),
        default=os.environ.get("JERO_IMU", "bno055"),
        help="bno055 = upstream driver; mpu6050 = robot/imu_mpu6050.py (env JERO_IMU)",
    )
    p.add_argument(
        "--board",
        choices=("auto", "pi", "other"),
        default=os.environ.get("JERO_BOARD", "auto"),
        help="pi = upstream GPIO code; other (RB3 Gen 2, ...) = robot/board_shims.py; auto = detect",
    )
    p.add_argument(
        "--feet",
        choices=("auto", "none", "gpiod", "pi"),
        default=os.environ.get("JERO_FEET", "auto"),
        help="foot switches: none (Jero has none: both read 'not touching'), gpiod, or pi (upstream "
        "Pi GPIO 22/27); auto = ~/.config/jero/board.json, else none",
    )
    p.add_argument("--feet-chip", default=None, help="gpiod chip, e.g. /dev/gpiochip4")
    p.add_argument("--feet-lines", default=None, help="gpiod line offsets LEFT,RIGHT, e.g. 22,27")
    # upstream knobs, same defaults as v2_rl_walk_mujoco.py
    p.add_argument("-a", "--action_scale", type=float, default=0.25)
    p.add_argument("-p", type=int, default=30)
    p.add_argument("-i", type=int, default=0)
    p.add_argument("-d", type=int, default=0)
    p.add_argument("-c", "--control_freq", type=int, default=50)
    p.add_argument("--pitch_bias", type=float, default=0, help="deg")
    p.add_argument("--cutoff_frequency", type=float, default=None)
    # Jero
    p.add_argument("--no-xbox", action="store_true", help="don't require a paired gamepad")
    p.add_argument("--pad", choices=("auto", "xbox", "ps4"), default="auto", help="gamepad layout (auto: by name)")
    p.add_argument("--no-link", action="store_true", help="disable the Wi-Fi command link")
    p.add_argument(
        "--telemetry",
        default=os.environ.get("JERO_TELEMETRY", "127.0.0.1:5006"),
        help="host:port for the dashboard's path card (UDP, 10 Hz); 'none' = off",
    )
    p.add_argument("--link-port", type=int, default=5005)
    p.add_argument("--link-bind", default="0.0.0.0")
    p.add_argument("--link-key", default=str(DEFAULT_KEY), help="hex key file; 'none' = unsigned (bench only)")
    p.add_argument(
        "--missing-servos",
        default="",
        metavar="IDS",
        help="bench only: comma-separated servo IDs that aren't connected, e.g. 30,31,32,33 (head chain)",
    )
    p.add_argument("-v", "--verbose", action="store_true")
    return p.parse_args()


def main():
    args = parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    log = logging.getLogger("jero.walk")

    runtime = resolve_runtime(args.runtime_dir)
    policy = resolve_policy(args.onnx_model_path)
    duck_config = Path(args.duck_config_path).expanduser().resolve()
    if not duck_config.is_file():
        sys.exit(f"{duck_config} missing: copy robot/duck_config.json and fill in your offsets")

    sys.path.insert(0, str(runtime / "mini_bdx_runtime"))
    sys.path.insert(0, str(runtime / "scripts"))
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    # upstream loads ./polynomial_coefficients.pkl and ../mini_bdx_runtime/assets relative to scripts/
    os.chdir(runtime / "scripts")

    import board_shims

    board = board_shims.detect() if args.board == "auto" else args.board
    cfg = board_shims.load_config().get("feet", {})
    feet = args.feet if args.feet != "auto" else cfg.get("backend", "none")
    chip = args.feet_chip or cfg.get("chip")
    lines = args.feet_lines.split(",") if args.feet_lines else cfg.get("lines")
    lines = tuple(int(x) for x in lines) if lines else None
    if board != "pi":
        if args.imu != "mpu6050":
            sys.exit("not a Raspberry Pi: upstream's BNO055 driver needs Pi GPIO; use --imu mpu6050")
        if feet == "pi":
            sys.exit("--feet pi needs a Raspberry Pi; use none or gpiod")
        board_shims.install(feet, chip, lines)
        log.info("board: %s (non-Pi shims installed)", board)
    elif feet != "pi":
        board_shims.install_feet(feet, chip, lines)  # no foot switches on Jero: skip Pi GPIO for them
    log.info("foot switches: %s", feet)

    if args.imu == "mpu6050":
        install_mpu6050(log)

    if args.bus == "jero":
        import feetech_io

        feetech_io.install()  # short timeout: a lost reply skips one step instead of freezing 1 s
    else:
        import bus_guard

        bus_guard.install()  # rustypot: a bus panic skips one step instead of killing the walk
    if args.missing_servos:
        import missing_servos

        missing_servos.install(missing_servos.parse_ids(args.missing_servos))

    from jero_controller import MuxController
    from v2_rl_walk_mujoco import RLWalk

    link = None
    if not args.no_link:
        from jero_link import UdpReceiver, load_key

        key_path = os.path.expanduser(args.link_key)
        if args.link_key.lower() != "none" and not os.path.isfile(key_path):
            sys.exit(f"link key {key_path} missing: run robot/setup_pi.sh or tools/gen_link_key.sh")
        key = None if args.link_key.lower() == "none" else load_key(key_path)
        if key is None:
            log.warning("Jero link is UNSIGNED: anyone on this Wi-Fi can drive the robot")
        link = UdpReceiver(bind=args.link_bind, port=args.link_port, key=key)

    xbox = None
    if not args.no_xbox:
        import pads

        try:
            xbox = pads.make_controller(20, args.pad)
        except Exception as exc:  # noqa: BLE001  pygame raises if no joystick is connected
            if link is None:
                sys.exit(f"No gamepad ({exc}) and link disabled: nothing can command the robot")
            log.warning("No gamepad (%s): link-only mode", exc)

    log.info("runtime=%s policy=%s config=%s", runtime, policy, duck_config)
    rl = RLWalk(
        str(policy),
        duck_config_path=str(duck_config),
        serial_port=args.serial_port,
        action_scale=args.action_scale,
        pid=[args.p, args.i, args.d],
        control_freq=args.control_freq,
        commands=False,  # we attach our own controller below
        pitch_bias=args.pitch_bias,
        cutoff_frequency=args.cutoff_frequency,
    )
    if link is not None:

        def estop():
            if not rl.paused:
                log.warning("E-STOP from link: walk paused (press A to resume)")
            rl.paused = True

        link.on_estop = estop

    rl.xbox_controller = MuxController(xbox=xbox, link=link)
    rl.commands = True
    if args.telemetry != "none":
        import telemetry

        host, _, port = args.telemetry.rpartition(":")
        telemetry.start(rl, (host or "127.0.0.1", int(port)))
    try:
        rl.run()
    finally:
        if link is not None:
            link.close()


if __name__ == "__main__":
    main()

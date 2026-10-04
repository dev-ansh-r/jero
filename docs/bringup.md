# Bring-up runbook

Work through the steps in order, and don't start a step until the previous one passes.
Put Jero on a stand with its feet off the ground until step 6.

## 0. Pi setup (bench, no servos)

The pre-built Duck image the runtime README mentions (v0.2.3) is not published: the runtime's only
release (V2 Classic) has no image attached. Install on plain Raspberry Pi OS instead. This is what
Jero's Pi runs (verified 4 Oct 2026: Raspberry Pi OS Lite 64-bit, Debian 13 trixie, Python 3.13).

1. Raspberry Pi Imager → Raspberry Pi OS Lite (64-bit). Edit settings: hostname `jero`, your user,
   2.4 GHz Wi-Fi (the Zero 2 W has no 5 GHz), country IN, **Services → Enable SSH**.
   No monitor needed: once it boots, `ssh <user>@jero.local`.
2. System packages (run long ones in `tmux`; if you're on Raspberry Pi Connect and dpkg gets
   interrupted, finish it with `sudo systemd-run --unit=fix-dpkg --collect dpkg --configure -a`):
   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo apt install -y git i2c-tools build-essential python3-dev tmux
   sudo raspi-config nonint do_i2c 0
   ```
3. Python environment + runtime at the pinned commit. On Python 3.13 three runtime pins have no
   ARM64 wheels, so override them (onnxruntime ≥ 1.20, numpy 2.x, pygame 2.6.1; the policy gives
   identical actions):
   ```bash
   curl -LsSf https://astral.sh/uv/install.sh | sh && source ~/.local/bin/env
   uv venv --seed --python /usr/bin/python3 ~/.virtualenvs/open-duck-mini-runtime
   echo 'source ~/.virtualenvs/open-duck-mini-runtime/bin/activate' >> ~/.bashrc && source ~/.bashrc
   git clone https://github.com/apirrone/Open_Duck_Mini_Runtime ~/Open_Duck_Mini_Runtime
   cd ~/Open_Duck_Mini_Runtime && git checkout 376de65435c93486347cd601a31aa96951799896
   printf 'onnxruntime>=1.20\nnumpy>=2.1\npygame==2.6.1\n' > ~/py313-overrides.txt
   uv pip install -e . --override ~/py313-overrides.txt
   ```
4. Jero:
   ```bash
   git clone https://github.com/dev-ansh-r/jero ~/Jero && ~/Jero/robot/setup_pi.sh
   python ~/Jero/tools/policy_dryrun.py --imu none --seconds 10     # policy on the Pi, no hardware
   ```
5. Copy `~/.config/jero/link.key` to the brain machine(s).

**Pass:** `setup_pi.sh` prints `Done.`; `policy_dryrun.py` prints `PASS` (measured on the Zero 2 W:
~1 ms per inference, p99 loop ~3 ms of the 20 ms budget).

The docs below say `workon open-duck-mini-runtime`; with this setup the venv is already active
from `~/.bashrc` (or `source ~/.virtualenvs/open-duck-mini-runtime/bin/activate`).

## 1. Servo IDs (before assembly)

```bash
~/Jero/tools/configure_servos.sh          # one servo on the bus at a time (--port /dev/ttyUSB0 for a USB-TTL adapter)
```

**Pass:** 14 labelled servos, each with its horn fitted at the zero position.

## 2. IMU

```bash
cd ~/Open_Duck_Mini_Runtime && python3 mini_bdx_runtime/mini_bdx_runtime/raw_imu.py
python3 scripts/imu_server.py        # laptop: python3 scripts/imu_client.py --ip jero.local
```

**Pass:** tilting the body forward, back and sideways moves the frame the same way. If it's mirrored,
set `imu_upside_down` in `~/duck_config.json`. Then run `scripts/calibrate_imu.py`.

### 2b. IMU: MPU6050 (GY-521) instead of BNO055

The policy only reads raw gyro + accelerometer, so an MPU6050 works. Mount it **rigidly** where the
BNO055 goes (screws or a printed clip, no foam tape), wired to the same pins (VCC to 3V3, AD0 to GND).

```bash
i2cdetect -y 1                                   # expect 68
python ~/Jero/tools/imu_check.py                 # guided, ~1 min, robot in your hands
```

Tips from our first run (an "MPU9250" board that reports `WHO_AM_I 0x70 (MPU6500)`: same accel/gyro, fine):
- Step 1: put it **down on the table**; a hand is never still enough (gyro std limit 0.02 rad/s).
- Hold it in the orientation it will have in the robot, with a FRONT mark, and look **from behind** it:
  "left side down" is the robot's left. Nose down reads accel x **negative**, left down accel y **negative**.
- Step 4: turn it briskly (~1 s); each step records 3 s right after Enter.

**Pass:** `ALL PASS`. That saves the measured mounting + gyro bias to `~/.config/jero/imu.json`.
From then on run the walk with `--imu mpu6050` (or `JERO_IMU=mpu6050` in `/etc/default/jero-walk`).
Re-run the check whenever the IMU is unscrewed. At every start, hold the robot still for 2 s:
the gyro bias is re-measured (falls back to the saved one if it moved).

## 3. Motors

```bash
python3 scripts/check_voltage.py && python3 scripts/check_motors.py
```

**Pass:** all 14 IDs respond and the bus voltage reads 7.4–8.4 V.

## 4. Joint offsets

```bash
cd scripts && python find_soft_offsets.py       # write the results into ~/duck_config.json
```

**Pass:** in the init pose (`turn_on.py`), the legs look symmetric and the feet sit parallel to the floor.

## 5. Foot switches

```bash
python3 scripts/fc_test.py
```

**Pass:** pressing each foot flips the matching value.

## 6. Walk (Xbox pad, upstream path first)

Pair the pad (see the upstream `docs/INSTALL.md` §4), then:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts && python v2_rl_walk_mujoco.py --onnx_model_path ~/BEST_WALK_ONNX_2.onnx
```

**Pass:** it stands, steps in place, walks and turns on the sticks, and `A` pauses. If it leans, use `--pitch_bias`
(degrees), or D-pad up/down to trim the gait frequency.

## 7. Walk through Jero (pad + link)

```bash
python ~/Jero/robot/jero_walk.py                  # pad still has priority
# brain:  python brain/teleop.py jero.local
# any laptop:  python tools/estop.py jero.local
```

**Pass:** teleop drives it, touching the pad sticks takes over immediately, killing teleop brings it to a
stop within 0.5 s, and `estop.py` pauses it.

## 8. Autostart (only once 7 passes)

```bash
echo 'JERO_WALK_ARGS="--pitch_bias 0"' | sudo tee /etc/default/jero-walk
~/Jero/robot/setup_pi.sh --replace-autostart && sudo reboot
journalctl -u jero-walk -f
```

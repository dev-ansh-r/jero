# Pico servo bus bridge

Raspberry Pi Pico (RP2040) firmware that replaces the Waveshare Bus Servo Adapter. The Pico shows up
on the Pi Zero 2 W as `/dev/ttyACM0` and carries Feetech STS packets between USB and the servo bus
(1 Mbps), so the upstream runtime (rustypot) and pypot work unchanged.

Based on the bridge in [linuxchunk/zero_driver_pico](https://github.com/linuxchunk/zero_driver_pico)
(same wiring, same line release after each packet), rewritten to work per transaction. The servo
library on the Pi aborts on any byte that is late or out of place, so this firmware:

1. reads one complete, checksummed instruction packet from USB;
2. works out the replies it must produce: none for a sync write or other broadcast, one per listed
   ID for a sync read, one for anything addressed to a single ID;
3. sends it, releases the line, and skips its own echo;
4. collects whole reply packets (header, expected ID, length, checksum) until all arrive or a
   deadline passes;
5. answers USB in one write: all replies in the requested order, or, if any is missing or corrupt,
   one short error marker and nothing else.

The error marker is a valid status packet from an ID that wasn't asked for. The Pi rejects it at
once, so a missing servo costs one control step instead of a 1 s timeout, and nothing is left
behind in the Pi's buffer. It never carries servo data.

## Wiring

| Pico | Connects to |
|---|---|
| GP16 (pin 21), UART0 TX | servo DATA, direct |
| GP17 (pin 22), UART0 RX | servo DATA through ~5 kΩ |
| GND (pin 23) | servo supply GND (common ground, star point at the battery −) |

Servo power never goes through the Pico. See [docs/wiring.md](../../docs/wiring.md).

## Build and flash

```sh
cd firmware/pico_bridge
pio run                     # -> .pio/build/pico/firmware.uf2
```

Hold BOOTSEL while plugging the Pico into USB, then copy `firmware.uf2` to the `RPI-RP2` drive.
From the Pi you can also put it in bootloader mode by opening the port at 1200 baud
(so never open `/dev/ttyACM0` at 1200 baud by accident).

## LED

| LED | Meaning |
|---|---|
| short flicker | bus traffic |
| fast blink for 1 s | a transaction failed (missing or corrupt reply, error marker sent) |

## Check it on the Pi

```bash
python ~/Jero/tools/bus_test.py          # reads only, nothing moves
```

Pass: 0 errors and 0 panics over 300 rounds, median well under 10 ms, and a read that includes a
missing ID fails in well under 1 s with the next read still working.

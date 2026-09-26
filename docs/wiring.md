# Wiring

Reference: the upstream [wiring diagram](https://github.com/apirrone/Open_Duck_Mini/blob/b23317a485b3cec7d8417f352478778b3475173c/docs/open_duck_mini_v2_wiring_diagram.png).

## Power

```
2× 18650 (2S, 7.4 V nom / 8.4 V full) ─ 2S BMS ─ switch ─┬─ Bus Servo Adapter (A) 5.5×2.1 jack ─ servo bus
                  USB-C 2S charger ───────┘                 └─ UBEC 5 V/3 A ─ Pi 5 V (header pin 2) + GND (pin 6)
```

- Match the two cell voltages before putting them in the holder.
- Check 5.0–5.2 V at the Pi and 7.4–8.4 V at the servo adapter **before** connecting either one.
- Leave out the expression features until walking works (set `duck_config.json` → `expression_features: false`).

## Servo bus (IDs)

The servos are daisy-chained. The adapter connects to the Pi over USB (micro-USB OTG → USB-C), and shows up as `/dev/ttyACM0`.

| Chain | Joint | ID |
|---|---|---|
| Right leg | hip_yaw / hip_roll / hip_pitch / knee / ankle | 10 / 11 / 12 / 13 / 14 |
| Left leg | hip_yaw / hip_roll / hip_pitch / knee / ankle | 20 / 21 / 22 / 23 / 24 |
| Neck/head | neck_pitch / head_pitch / head_yaw / head_roll | 30 / 31 / 32 / 33 |

## Pi Zero 2 W header

| Signal | Pin | GPIO |
|---|---|---|
| BNO055 VIN / GND / SDA / SCL | 1 / 9 / 3 / 5 | 3V3 / GND / GPIO2 / GPIO3 |
| Left foot switch / Right foot switch / GND | 15 / 13 / 9 | GPIO22 / GPIO27 / GND |
| Left eye + / Right eye + / Projector + / common − | 16 / 18 / 22 / 6 | GPIO23 / 24 / 25 / GND |
| Antenna L PWM / R PWM / 5 V / GND | 32 / 33 / 2 / 6 | GPIO12 / GPIO13 |
| MAX98357A LRC / BCLK / DIN / VIN / GND | 35 / 12 / 40 / 2 / 6 | GPIO19 / 18 / 21 |

The foot switches are active-low with internal pull-ups (upstream `feet_contacts.py`).

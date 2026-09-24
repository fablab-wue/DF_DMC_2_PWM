# DF_DMC_2_PWM

Dragonframe **DMC v2** on a **Raspberry Pi Pico**. Servos on PWM pins are motors. The remaining PWM pins mirror the first DMX channels. There is no SliderMC UART and no NeoPixel.

Shared framing, the path table, DMX, and GIO come from the sibling checkout [DF_DMC_Common](https://github.com/fablab-wue/DF_DMC_Common). Clone it as `../DF_DMC_Common` before building.

## Pins

| Signal | GPIO |
|--------|------|
| PWM_1..16 | 0..15 |
| DMX_TX | 16 |
| CAMERA | 17, open-collector, active low |
| SW_1..4 | 18..21, pull-up, ON = low |
| BUZZER | 22, active high |
| Onboard LED | 25 |
| GIO_OUT_1..2 | 26..27, open-collector, active low |
| GIO_IN | 28, pull-up, low = bit 0 |

The DIP is read once at startup. Changing it later has no effect until the next reset.

## Onboard LED

| State | LED |
|-------|-----|
| Boot | off for about 0.5 s |
| Waiting for Dragonframe | toggle at about 4 Hz |
| Connected | one very short pulse every 1 s |
| DMC packet | one short pulse per parsed frame, then back to waiting or connected |

## DIP switches

SW1 is the high bit, SW3 the low bit. ON = low.

| SW1 | SW2 | SW3 | PWM use |
|-----|-----|-----|---------|
| OFF | OFF | OFF | no motors, PWM_1..16 = DMX 1..16 |
| OFF | OFF | ON | motors 1..2, PWM_3..16 = DMX 1..14 |
| OFF | ON | OFF | motors 1..4, PWM_5..16 = DMX 1..12 |
| OFF | ON | ON | motors 1..6, PWM_7..16 = DMX 1..10 |
| ON | OFF | OFF | motors 1..8, PWM_9..16 = DMX 1..8 |
| ON | OFF | ON | motors 1..10, PWM_11..16 = DMX 1..6 |
| ON | ON | OFF | motors 1..12, PWM_13..16 = DMX 1..4 |
| ON | ON | ON | motors 1..16, no DMX on PWM |

DMX_TX still sends the 512-channel universe in every mode, including all-motors.

| SW4 | PWM mirror |
|-----|------------|
| OFF | linear (DMX level / 255) |
| ON | square curve for LEDs: 0 = 0%, 128 ≈ 25%, 255 = 100% |

The square curve is only on the PWM mirror. The DMX wire stays linear.

## Dragonframe

Connect as device type **dmc-lite**. Hello name: `DF PWM V1 (dmc-lite)`.

Use the step integers directly. Do not set steps per unit to 1000.

- 1 step = 1 µs from center
- Position 0 = 1.5 ms pulse
- Range −1000..+1000 (0.5 ms..2.5 ms)
- Servo PWM is 100 Hz. DMX PWM is 18 kHz
- A move or jog sets the pulse immediately. Speed and acceleration are ignored
- Soft limits from Arc are clamped on top of ±1000 µs. Until a limit is enabled, only ±1000 applies
- Reset position does not move the 1.5 ms zero
- Realtime preview downloads the path. Each frame is held until the next frame, with no ramp
- A motor with the enable flag clear goes limp (PWM level 0)

## Build

1. Install [VS Code](https://code.visualstudio.com/) and the PlatformIO IDE extension.
2. Clone this repo and `DF_DMC_Common` side by side.
3. **File → Open Folder** → this repository.
4. PlatformIO: **Build** / **Upload** (`board = pico`).

USB CDC is binary DMC. Do not use the serial monitor as a console.

## License

Copyright (c) 2026 Jochen Krapf \<jk@nerd2nerd.org\>

Licensed under the [MIT License](LICENSE).

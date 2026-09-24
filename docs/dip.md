# DIP switches

[← Index](README.md)

SW1..SW4 are GPIO 18..21, inputs with pull-up. **ON = low**, **OFF = high**. They are read once in `setup()`. Changing them later does nothing until the next reset.

SW1 is the high bit, SW3 the low bit. Code 0..6 → `motors = code × 2`. Code 7 → 16 motors. DMX PWM pins are the rest of PWM_1..16, starting at the next pin. DMX channel 1 is the first of those pins.

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

Motor counts are even so each RP2040 PWM slice (GPIO pairs 0–1, 2–3, …) is entirely servo rate or entirely 18 kHz.

| SW4 | PWM mirror |
|-----|------------|
| OFF | linear (DMX level / 255) |
| ON | square curve: duty = level × level / 255 (0 = 0%, 128 ≈ 25%, 255 = 100%) |

The square curve is only on the PWM pins, for LEDs that should look linear. The DMX wire stays raw 0–255 in every mode, including all-motors.

Hello reports the motor count from this table. With zero motors, motor commands get a range error and all 16 pins are DMX PWM.

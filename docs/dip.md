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

Motor counts are even so each RP2040 PWM slice (GPIO pairs 0–1, 2–3, …) is entirely servo rate or entirely the DMX mirror rate.

| SW4 | PWM mirror |
|-----|------------|
| OFF | 18 kHz, wrap 254, compare `L` (0 = low, 255 = high) |
| ON | about 2 kHz, wrap `254 × 255`, compare `L × L` (level 1 is one count, 128 is about 25%, 255 is full on) |

SW4 on needs the wide counter so dim steps are not rounded to zero. The clock divider cannot go below 1, so 64771 counts at 133 MHz is about 2 kHz.

The square curve is only on the PWM pins, for LEDs that should look linear. The DMX wire stays raw 0–255 in every mode, including all-motors.

Hello reports the motor count from this table. With zero motors, motor commands get a range error and all 16 pins are DMX PWM.

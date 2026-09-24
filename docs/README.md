# DF_DMC_2_PWM

Dragonframe **DMC v2** on a **Raspberry Pi Pico**. Servo PWM pins are motors. The remaining PWM pins mirror the first DMX channels. There is no SliderMC UART and no NeoPixel.

**Start here:** [overview.md](overview.md)

| Document | Topic |
|----------|-------|
| [overview.md](overview.md) | What it is, LED, what a move does |
| [pins.md](pins.md) | GPIO map, pinout image, DMX / camera / buzzer wiring |
| [dip.md](dip.md) | SW1..SW4, motor count, DMX curve |
| [dragonframe.md](dragonframe.md) | Step scale, soft limits, realtime path |
| [build.md](build.md) | PlatformIO, sibling library |

Connect steps that both boards share: [DF_DMC_Common dragonframe.md](https://github.com/fablab-wue/DF_DMC_Common/blob/main/docs/dragonframe.md).

**Code:** this repository. **Library:** [DF_DMC_Common](https://github.com/fablab-wue/DF_DMC_Common).

**Official DMC protocol (Dragonframe, not this project):** [DMC-Protocol-2024-08-13.pdf](https://www.dragonframe.com/download/dmcproto/DMC-Protocol-2024-08-13.pdf)

# DF_DMC_2_PWM

Dragonframe **DMC v2** on a **Raspberry Pi Pico**. Servo PWM pins are motors. The remaining PWM pins mirror the first DMX channels. There is no SliderMC UART and no NeoPixel.

**Documentation:** [docs/README.md](docs/README.md)

Shared framing, the path table, DMX, and GIO come from the sibling checkout [DF_DMC_Common](https://github.com/fablab-wue/DF_DMC_Common). Clone it as `../DF_DMC_Common` before a source build.

## Flash a release

No compiler and no PlatformIO. Board: **Raspberry Pi Pico**.

1. Download `DF_DMC_2_PWM-<tag>-pico.uf2` from the [Releases](https://github.com/fablab-wue/DF_DMC_2_PWM/releases) page.
2. Hold **BOOTSEL**, plug in USB, then release BOOTSEL.
3. Copy the UF2 onto the `RPI-RP2` drive. The board reboots into the new firmware.

USB CDC is binary DMC, not a text console. A new file is built when a `v*` tag is pushed. That tag's commit must already contain this workflow. Rebuild an existing tag from the Actions page with **Run workflow**.

Source build steps: [docs/build.md](docs/build.md).

## License

Copyright (c) 2026 Jochen Krapf \<jk@nerd2nerd.org\>

Licensed under the [MIT License](LICENSE).

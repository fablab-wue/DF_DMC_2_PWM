# DF_DMC_2_PWM build

[← Index](README.md)

Board: **Raspberry Pi Pico**. Core: earlephilhower Arduino Pico via PlatformIO (`env:rpipico`, `board = pico`).

## Flash a release

No compiler and no PlatformIO.

1. Download `DF_DMC_2_PWM-<tag>-pico.uf2` from the [Releases](https://github.com/fablab-wue/DF_DMC_2_PWM/releases) page.
2. Hold **BOOTSEL**, plug in USB, then release BOOTSEL.
3. Copy the UF2 onto the `RPI-RP2` drive. The board reboots into the new firmware.

A new file is built when a `v*` tag is pushed. Rebuild an existing tag from the Actions page with **Run workflow**.

## Prerequisites

1. Install [VS Code](https://code.visualstudio.com/).
2. Install the **PlatformIO IDE** extension.
3. Clone this repo and [DF_DMC_Common](https://github.com/fablab-wue/DF_DMC_Common) as siblings (`../DF_DMC_Common`).
4. **File → Open Folder** → this repository.
5. Connect the Pico over USB.

```bash
python -m platformio run
python -m platformio run --target upload
```

`platformio.ini` sets `-DFDMC_MAX_AXES=16` and `lib_deps = symlink://../DF_DMC_Common`.

USB CDC is **binary DMC**. `pio device monitor` will not show a useful text console. Close Dragonframe before Upload — the port must be free.

## Dragonframe

Scene → Connections → **dmc-lite** → this COM port. Do not set steps per unit to 1000. Scale: [dragonframe.md](dragonframe.md).

Official wire format: [DMC-Protocol-2024-08-13.pdf](https://www.dragonframe.com/download/dmcproto/DMC-Protocol-2024-08-13.pdf).

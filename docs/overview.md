# DF_DMC_2_PWM overview

[← Index](README.md)

**DF_DMC_2_PWM** is a USB **DMC v2** device for [Dragonframe](https://www.dragonframe.com/) Arc. It runs on a **Raspberry Pi Pico**. Dragonframe talks binary DMC on USB. Servos are the motors. There is no motion-controller UART.

```text
Dragonframe (PC)
        USB CDC  — binary DMC v2  (device type dmc-lite)
DF_DMC_2_PWM  (Raspberry Pi Pico)
        PWM_1..16   servos and/or DMX mirror
        DMX_TX      DMX512 universe
```

Pins: [pins.md](pins.md). DIP: [dip.md](dip.md). Dragonframe scale: [dragonframe.md](dragonframe.md). Build: [build.md](build.md).

## Hello

Dragonframe starts with `MSG_HI` (`0x0001`). This board replies as **`jDF-PWM V1 <servos>S+<lights>L+2O+1I+CT+DMX`**. `<servos>` is the DIP motor count and `<lights>` is how many of the remaining pins mirror DMX. SW1 on, SW2 and SW3 off is **`jDF-PWM V1 8S+8L+2O+1I+CT+DMX`**. The name is at most 32 bytes. An unsolicited hello is also sent when the USB serial port opens.

Hello fields, fixed at boot from the DIP:

- motor count 0, 2, 4, 6, 8, 10, 12, or 16
- DMX count **512**
- GIO out **2** / GIO in **1**
- upload frame count **2048**
- capabilities `REAL_TIME` + `REAL_TIME_CAMERA`
- protocol version **2**, firmware `1.0.0`

## What a move does

A `MOVE` or jog slews toward the target at the speed Dragonframe sent. Speed 1 is the slow end, 10000 is the axis max velocity (`MOTOR_SET_SPEED`). Until that arrives, max velocity is 4000 steps/s. There is no acceleration ramp. While a realtime path is playing, each frame is held until the next frame (no ramp between frames). Position reports go out about every 100 ms while a motor is moving. `MSG_RT_END` is sent when playback finishes.

A motor with the enable flag clear goes limp (PWM level 0). Enable restores the last pulse.

## LED (GP25)

The onboard LED is one color. There is no NeoPixel.

| State | LED |
|-------|-----|
| Boot | off for about 0.5 s |
| Waiting for Dragonframe | toggle at about 4 Hz |
| Connected | one very short pulse every 1 s |
| DMC packet | one short pulse per parsed frame, then back to waiting or connected. A packet restarts the 1 s connected timer |

## Firmware `src/` map

| File | Role |
|------|------|
| `config.h` | Pins, 306.35 Hz servo scale |
| `servo_bank` | Servo PWM and path playback |
| `status_led` | Onboard LED |
| `bridge` | USB DMC dispatch |
| `main.cpp` | `setup()` / `loop()` |

Framing, the path table, DMX, and GIO come from [DF_DMC_Common](https://github.com/fablab-wue/DF_DMC_Common).

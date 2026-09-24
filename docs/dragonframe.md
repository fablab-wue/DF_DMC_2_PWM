# Dragonframe scale

[← Index](README.md)

Connect as device type **dmc-lite**. The shared steps (Scene → Connections, COM port, binary USB) are in [DF_DMC_Common — Dragonframe](https://github.com/fablab-wue/DF_DMC_Common/blob/main/docs/dragonframe.md).

Use the step integers directly. Do **not** set steps per unit to 1000.

## Pulse

Servo PWM is **306.35 Hz** (clock divider 6.625, top 65535) on a 133 MHz Pico. One count is 49.812 ns. There is no scale, only an offset:

`pwm = clamp(steps + 30113, 10113, 50113)`

| Steps | Counts | Pulse |
|-------|--------|-------|
| −20000 | 10113 | 504 µs |
| 0 | 30113 | 1500 µs |
| +20000 | 50113 | 2497 µs |

DMX PWM on the remaining pins stays **18 kHz**.

## Limits and enable

`MOTOR_SET_LIMITS` is stored per motor. Move, jog, and each path frame are clamped to the enabled soft limits and also to ±20000. Until a limit is enabled, only ±20000 applies. The board does not publish limits back to Arc.

Reset position does not move the 30113 zero. The reply is the fixed-scale position.

A motor with the enable flag clear goes limp (PWM level 0). Enable restores the last pulse.

## Realtime preview

Dragonframe downloads the path (`MSG_RT_UPLOAD_*`). Playback holds each frame’s steps until the next frame. There is no interpolation and no accel. Position reports go out about every 100 ms while a path is playing. `MSG_RT_END` is sent when playback finishes.

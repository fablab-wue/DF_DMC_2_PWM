# Generate Pico pinout ASCII + PNG for DF_DMC_2_PWM (stdlib only).
# python tools/render_pinout.py

from __future__ import annotations

from pathlib import Path

from pinout_common import (
    C_BUZZER,
    C_CAMERA,
    C_CTRL_PIN,
    C_DRV,
    C_EXT,
    C_FREE,
    C_GND,
    C_GP,
    C_LED,
    C_PINNUM,
    C_PWR_3V3,
    C_PWR_5V,
    C_SERVO,
    C_SW,
    C_UART,
    OUT_PNG,
    Canvas,
    color_for,
    crop_rgb,
    load_png_rgb,
    text_width,
)

ROOT = Path(__file__).resolve().parents[1]
OUT_PNG_PATH = ROOT / "docs" / "img" / "pinout.png"
OUT_TXT_PATH = ROOT / "docs" / "pinout.txt"

LEFT = [
    ("GP0", "PWM_1"),
    ("GP1", "PWM_2"),
    ("GND", "GND"),
    ("GP2", "PWM_3"),
    ("GP3", "PWM_4"),
    ("GP4", "PWM_5"),
    ("GP5", "PWM_6"),
    ("GND", "GND"),
    ("GP6", "PWM_7"),
    ("GP7", "PWM_8"),
    ("GP8", "PWM_9"),
    ("GP9", "PWM_10"),
    ("GND", "GND"),
    ("GP10", "PWM_11"),
    ("GP11", "PWM_12"),
    ("GP12", "PWM_13"),
    ("GP13", "PWM_14"),
    ("GND", "GND"),
    ("GP14", "PWM_15"),
    ("GP15", "PWM_16"),
]

RIGHT = [
    ("VBUS", "VBUS"),
    ("VSYS", "VSYS"),
    ("GND", "GND"),
    ("3V3_EN", "3V3_EN"),
    ("3V3", "3V3 OUT"),
    ("ADC_VREF", "ADC_VREF"),
    ("GP28", "GIO_IN"),
    ("AGND", "ADC GND"),
    ("GP27", "GIO_OUT2"),
    ("GP26", "GIO_OUT1"),
    ("RUN", "RUN"),
    ("GP22", "BUZZER"),
    ("GND", "GND"),
    ("GP21", "SW_4"),
    ("GP20", "SW_3"),
    ("GP19", "SW_2"),
    ("GP18", "SW_1"),
    ("GND", "GND"),
    ("GP17", "CAMERA"),
    ("GP16", "DMX_TX"),
]

PICO_GPIO_PNG = OUT_PNG / "raspberry-pi-pico-gpio.png"
_BOARD_CROP = (411, 24, 658, 625)
_PIN_Y0_ABS = 39
_PIN_PITCH = 30


def render_ascii() -> str:
    lines = [
        "Raspberry Pi Pico — DF_DMC_2_PWM pinout (top view, USB at top)",
        "Defaults in src/config.h",
        "",
        " function         pin           pin  function",
        "                +--- USB ---+",
    ]
    for i, ((lg, ll), (rg, rl)) in enumerate(zip(LEFT, RIGHT)):
        pn_l = i + 1
        pn_r = 40 - i
        lines.append(
            " %-16s %-7s %2d |o o| %-2d %-8s %s"
            % (ll, lg, pn_l, pn_r, rg, rl)
        )
    lines.extend(
        [
            "                +-----------+",
            "",
            "PWM_1..16 on GP0..15. DMX_TX GP16. CAMERA GP17 (open-collector, active low).",
            "SW_1..4 on GP18..21, pull-up, ON = low, read once at startup.",
            "BUZZER GP22 active high. GIO_OUT1..2 GP26..27 open-collector. GIO_IN GP28 pull-up.",
            "GP25 is the onboard LED (not on the header). GP23/GP24 are Pico power pins, unused here.",
        ]
    )
    return "\n".join(lines) + "\n"


def load_pico_board():
    _w, _h, rows = load_png_rgb(PICO_GPIO_PNG)
    x0, y0, x1, y1 = _BOARD_CROP
    board_w, board_h, board = crop_rgb(rows, x0, y0, x1, y1)
    pin_ys = [_PIN_Y0_ABS - y0 + i * _PIN_PITCH for i in range(20)]
    return board_w, board_h, board, pin_ys


def render_png(path: Path):
    board_w, board_h, board_rows, pin_ys = load_pico_board()
    margin = 24
    title_h = 56
    gap = 4
    fun_w = 112
    gp_w = 52
    pin_w = 28
    box_h = min(22, _PIN_PITCH - 6)
    side_w = fun_w + gap + gp_w + gap + pin_w
    width = margin + side_w + board_w + side_w + margin
    legend = [
        ("PWM_*", C_SERVO),
        ("DMX_TX", C_DRV),
        ("GIO_OUT*", C_EXT),
        ("GIO_IN", C_SW),
        ("SW_*", C_SW),
        ("CAMERA", C_CAMERA),
        ("BUZZER", C_BUZZER),
        ("LED", C_LED),
        ("GND", C_GND),
        ("power 3V3", C_PWR_3V3),
        ("power 5V", C_PWR_5V),
    ]
    legend_rows = 1
    x = margin
    max_x = width - margin
    for name, _col in legend:
        item_w = 14 + text_width(name, 1) + 16
        if x + item_w > max_x and x > margin:
            legend_rows += 1
            x = margin
        x += item_w
    legend_h = legend_rows * 16 + 8
    height = margin + title_h + board_h + 28 + legend_h + margin
    text_c = (25, 25, 30)
    c = Canvas(width, height)

    def gp_box_color(pad, pin_num):
        if pad in ("GND", "AGND"):
            return C_GND
        if pad in ("VBUS", "VSYS") or pin_num in (39, 40):
            return C_PWR_5V
        if pad in ("3V3", "ADC_VREF") or pin_num in (35, 36):
            return C_PWR_3V3
        if pad in ("RUN", "3V3_EN"):
            return C_CTRL_PIN
        if pad.startswith("GP"):
            return C_GP
        return C_PINNUM

    c.text("Pico DF_DMC_2_PWM pinout", margin, margin, text_c, 2)
    c.text("Top view, USB at top. src/config.h", margin, margin + 28, (90, 90, 100), 1)

    board_x = margin + side_w
    board_y = margin + title_h
    c.blit_rgb(board_x, board_y, board_w, board_h, board_rows)

    for i in range(20):
        cy = board_y + pin_ys[i]
        by = cy - box_h // 2
        lg, ll = LEFT[i]
        rg, rl = RIGHT[i]
        pn_l = i + 1
        pn_r = 40 - i
        lc = color_for(ll, lg, pn_l)
        rc = color_for(rl, rg, pn_r)
        lx = margin
        c.label_box(ll, lx, by, fun_w, box_h, lc, "left")
        lx += fun_w + gap
        c.label_box(lg, lx, by, gp_w, box_h, gp_box_color(lg, pn_l), "center")
        lx += gp_w + gap
        c.label_box(str(pn_l), lx, by, pin_w, box_h, C_PINNUM, "center")
        rx = board_x + board_w
        c.label_box(str(pn_r), rx, by, pin_w, box_h, C_PINNUM, "center")
        rx += pin_w + gap
        c.label_box(rg, rx, by, gp_w, box_h, gp_box_color(rg, pn_r), "center")
        rx += gp_w + gap
        c.label_box(rl, rx, by, fun_w, box_h, rc, "left")

    c.text("GP25 = onboard LED (not on the header)", margin, board_y + board_h + 8, (90, 90, 100), 1)

    ly = board_y + board_h + 26
    x = margin
    for name, col in legend:
        item_w = 14 + text_width(name, 1) + 16
        if x + item_w > max_x and x > margin:
            x = margin
            ly += 16
        c.fill_rect(x, ly, 10, 10, col)
        c.text(name, x + 14, ly + 1, text_c, 1)
        x += item_w

    c.save(path)


def main():
    OUT_PNG_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_TXT_PATH.write_text(render_ascii(), encoding="utf-8")
    render_png(OUT_PNG_PATH)
    print("wrote", OUT_TXT_PATH)
    print("wrote", OUT_PNG_PATH)


if __name__ == "__main__":
    main()

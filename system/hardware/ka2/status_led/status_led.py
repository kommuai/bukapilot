#!/usr/bin/env python3
import subprocess
from typing import Optional

WS2812_SCRIPT_DEFAULT = "/usr/kommu/ws2812.py"

COLORS = {
  "WHITE": "FFFFFF",
  "RED": "00FF00",
  "GREEN": "0000FF",
  "BLUE": "FF0000",
  "ORANGE": "00FF25",
  "YELLOW": "00DD88",
}

def parse_color(color: str) -> str:
    """
    Accepts 'RED', 'red', 'ffffff', '#ffffff', etc.
    Returns a 6-char lowercase hex string like 'c92231'.
    Raises ValueError on invalid input.
    """
    if not color:
        raise ValueError("color must be provided")

    c = color.strip().lower()
    for name, hexv in COLORS.items():
        if c == name.lower():
            return hexv

    if c.startswith("#"):
        c = c[1:]
    if len(c) == 6 and all(ch in "0123456789abcdef" for ch in c):
        return c

    raise ValueError(f"Invalid color: {color}")

def set_led(
    a_color: str,
    b_color: Optional[str] = None,
    *,
    mode: str = "solid",
    rate: Optional[str] = None,
    duration: str = "1",
    brightness: str = "100",
    ws_script: str = WS2812_SCRIPT_DEFAULT,
    fire_and_forget: bool = True,
):
    """
    - a_color/b_color accept names ('RED') or hex ('#ff0000' / 'ff0000')
    - mode: 'solid' | 'blink' | 'run'
    - rate: optional, e.g. 'fast' | 'slow'
    - duration: only used for blink
    - brightness: '0'..'100'
    """
    a_hex = parse_color(a_color)
    b_hex = parse_color(b_color) if b_color else a_hex

    args = ["python", ws_script, mode, "--brightness", brightness]
    if mode in ("solid", "blink", "run"):
        args += ["--a-color", a_hex, "--b-color", b_hex]
        if mode == "blink" and rate:
            args += ["--rate", rate, "--duration", duration]

    if fire_and_forget:
        subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return None
    else:
        return subprocess.run(args, check=False)


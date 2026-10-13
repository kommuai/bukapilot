#!/usr/bin/env python3
"""
status_led.py — robust launcher with preemption and debug

- No --epoch requirement on ws2812.py.
- Preempts previous long-running worker (blink/run) via terminate/kill.
- Uses sys.executable to avoid "python" path issues.
- Optional DEBUG prints if env STATUS_LED_DEBUG=1.
"""

import os
import sys
import time
import shlex
import subprocess
import colorsys
import importlib.util
from typing import Optional

WS2812_SCRIPT_DEFAULT = "/usr/kommu/ws2812.py"

# Module-level handle to the currently running worker (for preemption)
_current_proc: Optional[subprocess.Popen] = None

COLORS = {
  "WHITE":  "FFFFFF",
  "RED":    "00FF00",
  "GREEN":  "0000FF",
  "BLUE":   "FF0000",
  "CYAN":   "FF0088",
  "ORANGE": "00FF25",
  "YELLOW": "00DD88",
}

def _dbg(msg: str):
    if os.getenv("STATUS_LED_DEBUG") == "1":
        print(f"[status_led] {msg}", file=sys.stderr)

def parse_color(color: str) -> str:
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

def _preempt_previous():
    global _current_proc
    if _current_proc is None:
        return
    if _current_proc.poll() is None:
        _dbg("Preempting previous ws2812.py (SIGTERM)...")
        try:
            _current_proc.terminate()
        except Exception as e:
            _dbg(f"terminate() error: {e!r}")
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline and _current_proc.poll() is None:
            time.sleep(0.01)
        if _current_proc.poll() is None:
            _dbg("Still running, sending SIGKILL...")
            try:
                _current_proc.kill()
            except Exception as e:
                _dbg(f"kill() error: {e!r}")
    _current_proc = None

def set_led(
    a_color: str,
    b_color: Optional[str] = None,
    *,
    mode: str = "solid",           # 'solid' | 'blink' | 'run'
    rate: Optional[str] = None,    # 'fast' | 'slow'
    duration: Optional[str] = None,# only for blink/run; None -> long sentinel
    brightness: str = "100",       # '0'..'100'
    ws_script: str = WS2812_SCRIPT_DEFAULT,
    fire_and_forget: bool = True,
):
    a_hex = parse_color(a_color)
    b_hex = parse_color(b_color) if b_color else a_hex

    if not os.path.exists(ws_script):
        raise FileNotFoundError(f"ws_script not found: {ws_script}")

    # Preempt any ongoing long-running worker
    _preempt_previous()

    py = sys.executable or "python3"
    args = [py, ws_script, mode, "--brightness", brightness, "--a-color", a_hex, "--b-color", b_hex]

    if mode in ("blink", "run"):
        if rate:
            args += ["--rate", rate]
        if duration is None:
            duration = "600"  # long sentinel; next call will preempt
        args += ["--duration", duration]

    # Debug print the exact command
    _dbg("exec: " + " ".join(shlex.quote(x) for x in args))

    if fire_and_forget:
        # Capture stderr to surface script errors in debug mode
        stderr = None if os.getenv("STATUS_LED_DEBUG") != "1" else subprocess.PIPE
        proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=stderr, text=True)
        if stderr is not None:
            # Read any immediate error without blocking the process lifetime
            try:
                time.sleep(0.02)
                if proc.poll() is not None:
                    err = proc.stderr.read() if proc.stderr else ""
                    if err:
                        _dbg(f"ws2812.py exited early with stderr:\n{err.strip()}")
            except Exception:
                pass
        if mode in ("blink", "run"):
            global _current_proc
            _current_proc = proc
        return None
    else:
        # Blocking call; show stderr to caller for troubleshooting
        return subprocess.run(args, check=False)


def _load_ws2812_module(ws_script: str):
    """Load the existing hardware driver without duplicating its SPI protocol."""
    spec = importlib.util.spec_from_file_location("kommu_ws2812", ws_script)
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load ws2812 driver: {ws_script}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run_rainbow_worker(
    ws_script: str,
    brightness: int = 200,
    frame_period: float = 0.08,
):
    """Continuously move a rainbow across the LEDs until this process is stopped."""
    ws2812 = _load_ws2812_module(ws_script)
    strip = ws2812.WS2812SPI(brightness=max(0, min(255, int(brightness))))
    strip.open()

    n_leds = max(1, int(strip.n_leds))
    frame = 0
    while True:
        phase = (frame * 0.02) % 1.0
        colors = []
        for led_index in range(n_leds):
            hue = (phase + led_index / n_leds) % 1.0
            red, green, blue = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
            colors.append((int(red * 255), int(green * 255), int(blue * 255)))

        # Reuse the existing driver's payload and SPI encoding so the animation
        # stays aligned with the hardware implementation in /usr/kommu.
        strip._write_payload(strip._build_payload(colors))
        frame += 1
        time.sleep(frame_period)


def set_rainbow(
    *,
    ws_script: str = WS2812_SCRIPT_DEFAULT,
    brightness: int = 200,
    fire_and_forget: bool = True,
):
    """Animate a moving rainbow until another LED state preempts it."""
    global _current_proc

    if not os.path.exists(ws_script):
        raise FileNotFoundError(f"ws_script not found: {ws_script}")

    _preempt_previous()
    py = sys.executable or "python3"
    args = [
        py,
        __file__,
        "--rainbow-worker",
        ws_script,
        str(max(0, min(255, int(brightness)))),
    ]
    _dbg("exec: " + " ".join(shlex.quote(x) for x in args))

    if fire_and_forget:
        _current_proc = subprocess.Popen(
            args,
            stdout=subprocess.DEVNULL,
            stderr=None if os.getenv("STATUS_LED_DEBUG") == "1" else subprocess.DEVNULL,
            text=True,
        )
        return None

    return subprocess.run(args, check=False)


if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "--rainbow-worker":
    _run_rainbow_worker(
        sys.argv[2] if len(sys.argv) > 2 else WS2812_SCRIPT_DEFAULT,
        int(sys.argv[3]) if len(sys.argv) > 3 else 200,
    )

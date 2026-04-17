#!/usr/bin/env python3
"""
Indicator LED Service (indicatord) — state-change only, uses process preemption for blink

- Subscribes to selfdriveState (active, alertType) and driverCameraState (integLines for brightness).
- Only updates LEDs when (color, mode, rate) changes.
- Blink runs "indefinitely" (duration=600s) but is preempted instantly when the state changes.
- Not-running fallback to solid YELLOW if selfdriveState stalls or is unavailable.
"""

import time
import threading
from typing import Callable, Dict, Optional, Tuple

from openpilot.system.hardware.ka2.status_led.status_led import set_led, WS2812_SCRIPT_DEFAULT
from cereal import messaging

StateTuple = Tuple[str, str, Optional[str], str]  # (color, mode, rate, brightness)

class AlertLEDService:
    def __init__(
        self,
        *,
        ws_script: str = WS2812_SCRIPT_DEFAULT,
        brightness_min: int = 50,
        brightness_max: int = 200,
        integ_min: int = 100,
        integ_max: int = 892,
        not_running_brightness: str = "200",
        topic_cs: str = "selfdriveState",
        topic_dc: str = "driverCameraState",
        base_poll_ms: int = 500,
        overrides: Optional[Dict[str, Tuple[str, str, Optional[str]]]] = None,
        on_change: Optional[Callable[[bool, str, StateTuple], None]] = None,
        not_running_timeout_s: float = 1.0,
        debug: bool = False,
    ):
        self.ws_script = ws_script
        self.brightness_min = int(brightness_min)
        self.brightness_max = int(brightness_max)
        self.integ_min = int(integ_min)
        self.integ_max = int(integ_max)
        self.not_running_brightness = str(not_running_brightness)
        self.topic_cs = topic_cs
        self.topic_dc = topic_dc
        self.base_poll_ms = max(1, int(base_poll_ms))
        self.overrides = {k.lower(): v for k, v in (overrides or {}).items()}
        self.on_change = on_change
        self.not_running_timeout_s = float(not_running_timeout_s)
        self.debug = debug

        self._stop = threading.Event()
        self._sm = messaging.SubMaster([self.topic_cs, self.topic_dc])

        self._last_key: Optional[Tuple[str, str, Optional[str]]] = None
        self._last_state: Optional[StateTuple] = None
        self._last_msg_time_cs: Optional[float] = None
        self._last_integ: int = (self.integ_min + self.integ_max) // 2

        # boot → not-running yellow (counts as a state)
        state = ("YELLOW", "solid", None, self.not_running_brightness)
        self._apply(state, active=False, alert_type="not_running", force=True)
        self._last_state = state
        self._last_key = (state[0], state[1], state[2])

    def stop(self):
        self._stop.set()

    def run_forever(self):
        while not self._stop.is_set():
            now = time.monotonic()
            self._sm.update(self.base_poll_ms)
            now = time.monotonic()

            if self._sm.updated[self.topic_dc]:
                dc = self._sm[self.topic_dc]
                v = getattr(dc, "integLines", None)
                if isinstance(v, (int, float)):
                    self._last_integ = int(v)

            if self._sm.updated[self.topic_cs]:
                cs = self._sm[self.topic_cs]
                self._last_msg_time_cs = now
                active = bool(getattr(cs, "active", False))
                alert_type = (getattr(cs, "alertType", "") or "")

                # Personality toggle is informational; keep current LED state unchanged.
                if alert_type.strip().lower() == "personalitychanged/warning":
                    if self.debug:
                        print("[indicatord] personalityChanged/warning -> keep previous LED state", flush=False)
                    continue

                color, mode, rate = self._classify_key(alert_type, active)
                key = (color, mode, rate)
                if key != self._last_key:
                    brightness = str(self._map_integ_to_brightness_inverse(self._last_integ))
                    desired: StateTuple = (color, mode, rate, brightness)
                    self._apply(desired, active=active, alert_type=alert_type, force=True)
                    self._last_state = desired
                    self._last_key = key
                    if self.on_change:
                        self._safe_on_change(active, alert_type, desired)
                else:
                    if self.debug:
                        print("[indicatord] CS updated but state unchanged", flush=False)

            # not-running fallback
            no_heartbeat = (self._last_msg_time_cs is None) or ((now - self._last_msg_time_cs) > self.not_running_timeout_s)
            if (not self._sm.alive[self.topic_cs]) or no_heartbeat:
                key = ("YELLOW", "solid", None)
                if self._last_key != key:
                    state = ("YELLOW", "solid", None, self.not_running_brightness)
                    self._apply(state, active=False, alert_type="not_running", force=True)
                    self._last_state = state
                    self._last_key = key
                    if self.on_change:
                        self._safe_on_change(False, "not_running", state)

    # mapping: alert_type format is "eventName/eventType" (e.g. cameraMalfunction/permanent, wrongGear/noEntry)
    # Event types (ET): enable, noEntry, warning, userDisable, softDisable, immediateDisable, permanent
    #
    # LED behavior summary:
    #   GREEN solid:  enable (engage), active with no alert
    #   RED blink:    immediateDisable when active (critical take-over)
    #   RED solid:    immediateDisable when not active, critical permanent (malfunction, faulted, etc.)
    #   ORANGE blink: softDisable, userDisable, warning (active or not)
    #   ORANGE solid: noEntry, softDisable when not active, warning permanent (dashcam, etc.)
    #   BLUE solid:   startup events, userDisable when not active, low-priority permanent, preEnable, overrides
    #   YELLOW solid: not_running fallback (selfdriveState unavailable)
    def _classify_key(self, alert_type: str, active: bool) -> Tuple[str, str, Optional[str]]:
        raw = (alert_type or "").strip()
        tl = raw.lower()
        parts = tl.split("/", 1)
        event_name = parts[0] if parts else ""
        event_type = parts[1] if len(parts) > 1 else ""

        if tl in self.overrides:
            color, mode, rate = self.overrides[tl]
            return (color, mode, rate)

        # Keep all startup alerts consistently blue (e.g. startup/noEntry, startup/permanent).
        if "startup" in event_name:
            return ("BLUE", "solid", None)

        # engage (buttonEnable/enable, pcmEnable/enable) — always green, matches soundd engage.wav
        if event_type == "enable":
            return ("GREEN", "solid", None)

        # --- Active (openpilot engaged) ---
        if active:
            if event_type == "immediatedisable":
                return ("RED", "blink", "fast")
            if event_type in ("softdisable", "userdisable"):
                return ("ORANGE", "blink", "fast")
            if event_type == "warning":
                return ("ORANGE", "blink", "fast")
            return ("GREEN", "solid", None)

        # --- Not active ---
        if event_type == "immediatedisable":
            return ("RED", "solid", None)
        if event_type in ("noentry", "softdisable"):
            return ("ORANGE", "solid", None)
        if event_type == "warning":
            return ("ORANGE", "blink", "fast")
        if event_type == "userdisable":
            return ("BLUE", "solid", None)

        # permanent: classify by event name (matches events.py EVENTS)
        if event_type == "permanent":
            if any(x in event_name for x in ("malfunction", "faulted", "error", "overheat", "outofspace",
                                             "relay", "accfaulted", "steerunavailable", "canerror", "canbus",
                                             "vehiclesensors", "sensorinvalid", "calibrationinvalid")):
                return ("RED", "solid", None)
            if any(x in event_name for x in ("dashcam", "calibration", "locationd", "paramsd",
                                             "unrecognized", "cameraframerate", "fan")):
                return ("ORANGE", "solid", None)
            return ("BLUE", "solid", None)

        # enable, preEnable, overrideLateral, overrideLongitudinal, or unknown
        return ("BLUE", "solid", None)

    def _map_integ_to_brightness_inverse(self, integ: int) -> int:
        if integ <= self.integ_min:
            return self.brightness_max
        if integ >= self.integ_max:
            return self.brightness_min
        t = (integ - self.integ_min) / (self.integ_max - self.integ_min)
        return int(round(self.brightness_max - t * (self.brightness_max - self.brightness_min)))

    def _apply(self, state: StateTuple, active: bool, alert_type: str, *, force: bool = False):
        color, mode, rate, brightness = state
        # For blink: set a long duration; preemption stops it on next state change
        kwargs = {"duration": "600"} if mode in ("blink", "run") else {}
        set_led(color, None, mode=mode, rate=rate, brightness=brightness,
                ws_script=self.ws_script, fire_and_forget=True, **kwargs)
        if self.debug:
            print(f"[indicatord]{' [FORCE]' if force else ''} -> {mode} {color} "
                  f"{('rate='+rate) if rate else ''} b={brightness}", flush=False)

    def _safe_on_change(self, active: bool, alert_type: str, state: StateTuple):
        try:
            self.on_change and self.on_change(active, alert_type, state)
        except Exception:
            pass

def main(*, debug: bool = False):
    svc = AlertLEDService(debug=debug)
    try:
        svc.run_forever()
    except KeyboardInterrupt:
        svc.stop()

if __name__ == "__main__":
    main(debug=False)

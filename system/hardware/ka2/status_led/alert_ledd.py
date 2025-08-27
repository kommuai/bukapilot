#!/usr/bin/env python3
"""
Alert LED Service (rewritten)

Goals:
- Blink once per alertType, then ALWAYS return to the previous solid state.
- Avoid the "blink script ends with OFF" race by (1) guarding past the blink end, and
  (2) doing a one-time solid reassert shortly afterward.
- Brightness = inverse map of driverCameraState.integLines (computed ONLY on controlsState updates).
- If controlsState is not running (no updates for > timeout), show solid YELLOW at fixed brightness.
- Low CPU: single loop with adaptive sleep to the next deadline.

StateTuple = (color, mode, rate, brightness)
"""

import time
import threading
from typing import Callable, Dict, Optional, Tuple

# --- robust imports for set_led ---
try:
    from .status_led import set_led, WS2812_SCRIPT_DEFAULT
except Exception:
    import os as _os, sys as _sys
    _pkg_root = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
    if _pkg_root not in _sys.path:
        _sys.path.insert(0, _pkg_root)
    from system.hardware.ka2.status_led.status_led import set_led, WS2812_SCRIPT_DEFAULT  # type: ignore

# --- cereal/messaging (required) ---
try:
    from cereal import messaging
except Exception as e:
    raise RuntimeError("cereal.messaging not found. Run inside the openpilot environment.") from e

StateTuple = Tuple[str, str, Optional[str], str]  # (color, mode, rate, brightness)


class AlertLEDService:
    """
    - Blink once per alertType, then force-restore previous solid state (with timing guard + reassert).
    - Brightness is inverse of driverCameraState.integLines:
        integ_min -> brightness_max
        integ_max -> brightness_min
      (Clamped; recomputed ONLY when controlsState updates.)
    - If controlsState not running for > not_running_timeout_s: show solid YELLOW at not_running_brightness.
    """

    # ------------------------- init -------------------------
    def __init__(
        self,
        *,
        ws_script: str = WS2812_SCRIPT_DEFAULT,
        brightness_min: int = 50,      # brightness at integ_max (darker)
        brightness_max: int = 200,     # brightness at integ_min (brighter)
        integ_min: int = 100,
        integ_max: int = 892,
        not_running_brightness: str = "200",
        topic_cs: str = "controlsState",
        topic_dc: str = "driverCameraState",
        base_poll_ms: int = 500,
        overrides: Optional[Dict[str, Tuple[str, str, Optional[str]]]] = None,
        on_change: Optional[Callable[[bool, str, StateTuple], None]] = None,
        blink_duration_s: float = 1.0,
        blink_guard_s: float = 0.12,           # guard AFTER blink end before restoring solid
        post_blink_reassert_s: float = 0.15,   # one-time reassert AFTER restore
        not_running_timeout_s: float = 1.0,
        debug: bool = False,
    ):
        # sanity
        assert 0 < brightness_min <= 255 and 0 < brightness_max <= 255
        assert integ_max > integ_min
        assert blink_duration_s > 0
        assert blink_guard_s >= 0
        assert post_blink_reassert_s >= 0

        # config
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
        self.blink_duration_s = float(blink_duration_s)
        self.blink_guard_s = float(blink_guard_s)
        self.post_blink_reassert_s = float(post_blink_reassert_s)
        self.not_running_timeout_s = float(not_running_timeout_s)
        self.debug = debug

        # runtime
        self._stop = threading.Event()
        self._sm = messaging.SubMaster([self.topic_cs, self.topic_dc])

        # last applied
        self._last: Optional[StateTuple] = None
        self._last_active: Optional[bool] = None

        # remembered non-blink solid (default)
        self._last_non_blink: StateTuple = ("BLUE", "solid", None, str(self.brightness_min))

        # blink timing + scheduling
        self._blink_latched_for: Optional[str] = None
        self._restore_at: float = 0.0              # when to restore solid after blink+guard
        self._post_reassert_at: float = 0.0        # when to reassert solid once
        self._pending_post_state: Optional[StateTuple] = None

        # CS heartbeat + fields
        self._last_msg_time_cs: Optional[float] = None
        self._last_seen_active: bool = False
        self._last_seen_alert_type: str = ""

        # latest integLines (used when CS updates)
        self._last_integ: int = (self.integ_min + self.integ_max) // 2

        # start with not-running yellow
        self._apply(("YELLOW", "solid", None, self.not_running_brightness),
                    active=False, alert_type="not_running", force=True)

    # ------------------------- public -------------------------
    def stop(self):
        self._stop.set()

    def run_forever(self):
        while not self._stop.is_set():
            now = time.monotonic()

            # compute adaptive wait to next deadline
            wait_ms = self._compute_wait_ms(now)
            self._sm.update(wait_ms)
            now = time.monotonic()

            # --- driverCameraState: cache integLines only (no LED apply here) ---
            if self._sm.updated[self.topic_dc]:
                dc = self._sm[self.topic_dc]
                v = getattr(dc, "integLines", None)
                if isinstance(v, (int, float)):
                    self._last_integ = int(v)

            # --- controlsState: main decision/update (only on CS updates) ---
            if self._sm.updated[self.topic_cs]:
                self._last_msg_time_cs = now
                cs = self._sm[self.topic_cs]
                self._last_seen_active = bool(getattr(cs, "active", False))
                self._last_seen_alert_type = (getattr(cs, "alertType", "") or "")
                ambient_brightness = str(self._map_integ_to_brightness_inverse(self._last_integ))
                desired = self._effective_state(self._last_seen_alert_type, self._last_seen_active, ambient_brightness)
                self._maybe_apply(desired, self._last_seen_active, self._last_seen_alert_type)

            # --- handle scheduled restore (past blink end + guard) ---
            if self._restore_at > 0 and now >= self._restore_at and self._pending_post_state:
                solid = self._pending_post_state
                self._pending_post_state = None
                self._restore_at = 0.0
                self._blink_latched_for = None
                self._maybe_apply(solid, self._last_seen_active, self._last_seen_alert_type, force=True)
                # schedule one-time reassert to stomp any trailing "off"
                if self.post_blink_reassert_s > 0:
                    self._post_reassert_at = now + self.post_blink_reassert_s

            # --- one-time reassert ---
            if self._post_reassert_at > 0 and now >= self._post_reassert_at:
                self._post_reassert_at = 0.0
                self._maybe_apply(self._last_non_blink, self._last_seen_active, self._last_seen_alert_type, force=True)

            # --- not-running fallback ---
            no_heartbeat = (self._last_msg_time_cs is None) or ((now - self._last_msg_time_cs) > self.not_running_timeout_s)
            if (not self._sm.alive[self.topic_cs]) or no_heartbeat:
                yellow = ("YELLOW", "solid", None, self.not_running_brightness)
                self._maybe_apply(yellow, active=False, alert_type="not_running")

    # ------------------------- mapping & policy -------------------------
    def _map_integ_to_brightness_inverse(self, integ: int) -> int:
        """Inverse linear map: integ_min -> brightness_max, integ_max -> brightness_min, clamped."""
        if integ <= self.integ_min:
            return self.brightness_max
        if integ >= self.integ_max:
            return self.brightness_min
        t = (integ - self.integ_min) / (self.integ_max - self.integ_min)
        return int(round(self.brightness_max - t * (self.brightness_max - self.brightness_min)))

    def classify(self, alert_type: str, active: bool, brightness: str) -> StateTuple:
        tl = alert_type.lower()
        if tl in self.overrides:
            color, mode, rate = self.overrides[tl]
            return (color, mode, rate, brightness)

        if active:
            if "immediate" in tl:
                return ("RED", "blink", "fast", brightness)
            if "soft" in tl or "warning" in tl:
                return ("ORANGE", "blink", "fast", brightness)
            return ("GREEN", "solid", None, brightness)

        if "noentry" in tl:
            if "permanent" in tl:
                return ("RED", "solid", None, brightness)
            return ("ORANGE", "solid", None, brightness)

        return ("BLUE", "solid", None, brightness)

    def _effective_state(self, alert_type: str, active: bool, brightness: str) -> StateTuple:
        """
        Decide the desired visual state.
        If it's a blink: schedule restore AFTER blink_duration + guard, then reassert once.
        If it's solid: remember as last_non_blink and clear timers.
        """
        base = self.classify(alert_type, active, brightness)
        now = time.monotonic()
        tl = alert_type.lower()

        # SOLID path: remember and clear blink/reassert timers
        if base[1] != "blink":
            self._last_non_blink = base
            self._blink_latched_for = None
            self._pending_post_state = None
            self._restore_at = 0.0
            self._post_reassert_at = 0.0
            return base

        # BLINK path: (re)latch per alert type and schedule a guarded restore
        if self._blink_latched_for != tl:
            self._blink_latched_for = tl
            # NOTE: we do NOT ask the LED script for its timing.
            # We *assume* blink_duration_s covers it and add blink_guard_s.
            self._restore_at = now + self.blink_duration_s + self.blink_guard_s
            self._pending_post_state = self._last_non_blink
            self._post_reassert_at = 0.0
            return base

        # still blinking: keep blinking
        if now < self._restore_at:
            return base

        # Safety fallback (normally the scheduled restore will handle this)
        return (base[0], "solid", None, brightness)

    # ------------------------- apply & change detection -------------------------
    def _maybe_apply(self, state: StateTuple, active: bool, alert_type: str, *, force: bool = False):
        if force or state != self._last or active != self._last_active:
            self._apply(state, active, alert_type, force=force)
            self._last = state
            self._last_active = active
            if self.on_change:
                try:
                    self.on_change(active, alert_type, state)
                except Exception:
                    pass

    def _apply(self, state: StateTuple, active: bool, alert_type: str, *, force: bool = False):
        color, mode, rate, brightness = state
        kwargs = {"duration": str(self.blink_duration_s)} if mode == "blink" else {}
        set_led(color, None, mode=mode, rate=rate, brightness=brightness,
                ws_script=self.ws_script, fire_and_forget=True, **kwargs)
        if self.debug and alert_type:
            print(f"[LED]{' [FORCE]' if force else ''} active={active} alert='{alert_type}' -> "
                  f"{mode} {color} {('rate='+rate) if rate else ''} brightness={brightness}", flush=False)

    # ------------------------- helpers -------------------------
    def _compute_wait_ms(self, now: float) -> int:
        """Sleep until the earliest of restore/reassert/base_poll deadlines."""
        deadlines = []
        if self._restore_at > 0:
            deadlines.append(self._restore_at)
        if self._post_blink_reassert_pending():
            deadlines.append(self._post_reassert_at)
        if deadlines:
            rem_ms = int((min(deadlines) - now) * 1000)
            return max(1, min(self.base_poll_ms, rem_ms if rem_ms > 0 else 1))
        return self.base_poll_ms

    def _post_blink_reassert_pending(self) -> bool:
        return self._post_reassert_at > 0

# ------------------------- script entry -------------------------
def main():
    svc = AlertLEDService()
    svc.run_forever()

if __name__ == "__main__":
    svc = AlertLEDService()
    try:
        svc.run_forever()
    except KeyboardInterrupt:
        svc.stop()


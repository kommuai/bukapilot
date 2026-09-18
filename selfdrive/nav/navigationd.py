#!/usr/bin/env python3
from __future__ import annotations
import json
from math import isfinite
from time import monotonic
from uuid import uuid4

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.nav.destination_store import (
  NAV_INSTRUCTION_STATE_KEY, NAV_ROUTE_DATA_KEY, NAV_ROUTE_FAILURE_KEY, NAV_ROUTE_REQUEST_KEY, NAV_SESSION_ID_KEY,
  NAV_STATUS_KEY, nav_stop_list_lock, parse_destination_json, read_stop_list, set_destination,
  NAV_STOP_VERSION_KEY, write_stop_list,
)
from openpilot.selfdrive.nav.route_engine import Coordinate, NavigationRoute, RouteProgress
from openpilot.selfdrive.appbridged.nav_route_transfer import ROUTE_TRANSFER_IDLE_TIMEOUT_SECONDS
from openpilot.selfdrive.nav.navigationd_policy import (
  ROUTE_FAILURE_RETRY_LIMIT, SAME_ROUTE_REROUTE_HOLD_SECONDS,
  route_failure_retry_ready, same_route_reroute_blocked,
)

NAVIGATIOND_HZ = 1
REROUTE_TRIGGER_SECONDS = 2.0
ARRIVAL_CLEAR_SECONDS = 3.0
SKIP_ARRIVAL_GRACE_SECONDS = 12.0  # phone MAX_STOPS queue / skip-episode
UNSURE_SUPPRESS_SECONDS = 3.0
ROUTE_REQUEST_TIMEOUT_SECONDS = 20.0
ROUTE_REQUEST_COOLDOWN_SECONDS = 30.0
REROUTE_REQUEST_DELAY_SECONDS = 10.0


class Navigationd:
  def __init__(self):
    self.params = Params()
    # Stops persist across reboot; route requests, status, and instruction state do not.
    had_request = bool(self.params.get(NAV_ROUTE_REQUEST_KEY))
    had_failure = bool(self.params.get(NAV_ROUTE_FAILURE_KEY))
    had_status = bool(self.params.get(NAV_STATUS_KEY))
    self.params.remove(NAV_ROUTE_REQUEST_KEY)
    self.params.remove(NAV_ROUTE_FAILURE_KEY)
    self.params.remove(NAV_STATUS_KEY)
    self.params.remove(NAV_INSTRUCTION_STATE_KEY)
    cloudlog.warning(
      f"navigationd boot_recovery transient_cleared request={int(had_request)} "
      f"failure={int(had_failure)} status={int(had_status)}"
    )
    self.pm = messaging.PubMaster(["navInstruction", "navRoute"])
    self.sm = messaging.SubMaster(["gpsLocation", "selfdriveState", "carState"])
    self.rk = Ratekeeper(NAVIGATIOND_HZ)
    self._route: NavigationRoute | None = None
    self._route_generation = 0
    self._published_route_generation = -1
    self._route_raw = ""
    self._last_position: Coordinate | None = None
    self._last_bearing: float | None = None
    self._gps_was_valid = False
    self._gps_regain_grace_until = 0.0
    self._off_route_started_at: float | None = None
    self._bearing_misaligned_started_at: float | None = None
    self._arrival_started_at: float | None = None
    self._last_nav_state: dict | None = None
    self._control_suppressed = False
    self._last_dest_key = ""
    self._route_dest_key = ""
    self._route_session_id = ''
    self._skip_arrival_until = 0.0
    self._last_status_log_at = 0.0
    self._last_reroute_needed: bool | None = None
    self._control_suppress_logged = False
    self._route_request_started_at = 0.0
    self._route_request_id = ''
    self._route_request_cooldown_until = 0.0
    self._reroute_request_not_before = 0.0
    self._same_route_reroute_not_before = 0.0
    self._same_route_accept_position: Coordinate | None = None
    self._same_route_guard_logged = False
    self._last_nav_status = ''
    self._last_session_id = ''
    self._route_request_failures = 0
    self._last_failed_request_id = ''
    self._boot_state_logged = False

  def _dest_key(self, dest: dict | None) -> str:
    if not dest:
      return ""
    try:
      return f"{float(dest['latitude']):.5f},{float(dest['longitude']):.5f}"
    except (KeyError, TypeError, ValueError):
      return ""

  def _stop_version(self) -> int:
    try:
      return max(0, int(self.params.get(NAV_STOP_VERSION_KEY) or 0))
    except (TypeError, ValueError, OverflowError):
      return 0

  def _clear_route(self, *, remove_destination: bool = False, reason: str = "") -> None:
    had_route = self._route is not None
    self._route = None
    self._route_raw = ""
    self._route_generation += 1
    self._off_route_started_at = None
    self._bearing_misaligned_started_at = None
    self._arrival_started_at = None
    self._control_suppressed = False
    self.params.remove(NAV_ROUTE_DATA_KEY)
    self.params.remove(NAV_INSTRUCTION_STATE_KEY)
    self.params.put_bool("NavRerouteNeeded", False)
    self.params.put_bool("NavHasRoute", False)
    self._route_dest_key = ""
    self._route_session_id = ''
    self._reroute_request_not_before = 0.0
    self._same_route_reroute_not_before = 0.0
    self._same_route_accept_position = None
    self._same_route_guard_logged = False
    self._last_nav_state = None
    if remove_destination:
      self.params.remove("NavDestination")
      if reason:
        self.params.put("NavDestinationWaypoints", json.dumps({"clearReason": reason}))
      else:
        self.params.remove("NavDestinationWaypoints")
    if had_route or remove_destination or reason:
      cloudlog.warning(
        f"navigationd route_clear reason={reason or 'none'} remove_dest={int(remove_destination)} "
        f"dest_key={self._last_dest_key}"
      )
    if remove_destination:
      self._clear_route_request('destination_cleared')
      self._set_nav_status('idle')

  def _set_nav_status(self, status: str) -> None:
    status = str(status or '')
    if status == self._last_nav_status:
      return
    self._last_nav_status = status
    self.params.put(NAV_STATUS_KEY, status)
    cloudlog.warning(f"navigationd status={status or 'none'} dest={self._last_dest_key}")

  def _clear_route_request(self, reason: str) -> None:
    if self._route_request_id:
      cloudlog.warning(f"navigationd route_request clear request={self._route_request_id} reason={reason}")
    self._route_request_id = ''
    self._route_request_started_at = 0.0
    self.params.remove(NAV_ROUTE_REQUEST_KEY)

  def _reset_route_acceptance_state(self, *, same_route: bool = False) -> None:
    self._off_route_started_at = None
    self._bearing_misaligned_started_at = None
    self._arrival_started_at = None
    self._control_suppressed = False
    self._control_suppress_logged = False
    self._last_reroute_needed = False
    self.params.put_bool('NavRerouteNeeded', False)
    self._reroute_request_not_before = monotonic() + REROUTE_REQUEST_DELAY_SECONDS
    self._same_route_accept_position = self._last_position if same_route else None
    self._same_route_reroute_not_before = (
      monotonic() + SAME_ROUTE_REROUTE_HOLD_SECONDS if same_route else 0.0
    )
    self._same_route_guard_logged = False

  def _same_route_reroute_blocked(self, now: float) -> bool:
    if (
      now >= self._same_route_reroute_not_before
      or self._same_route_accept_position is None
      or self._last_position is None
    ):
      return False
    if not same_route_reroute_blocked(
      now, self._same_route_reroute_not_before,
      self._last_position, self._same_route_accept_position,
    ):
      self._same_route_reroute_not_before = 0.0
      self._same_route_guard_logged = False
      return False
    try:
      moved_m = float(self._last_position.distance_to(self._same_route_accept_position))
    except (TypeError, ValueError):
      moved_m = 0.0
    if not self._same_route_guard_logged:
      self._same_route_guard_logged = True
      cloudlog.warning(
        f"navigationd reroute_guard same_route_hold=1 moved_m={moved_m:.0f} "
        f"dest={self._last_dest_key}"
      )
    return True

  def _maybe_request_route(self, dest: dict | None, location_valid: bool, now: float, *, reason: str) -> None:
    with nav_stop_list_lock():
      current_dest = parse_destination_json(self.params.get("NavDestination"))
      if self._dest_key(current_dest) != self._dest_key(dest):
        cloudlog.warning(
          f"navigationd route_request skipped stale_dest requested={self._dest_key(dest)} "
          f"active={self._dest_key(current_dest)}"
        )
        return
      self._maybe_request_route_locked(dest, location_valid, now, reason=reason)

  def _maybe_request_route_locked(self, dest: dict | None, location_valid: bool, now: float, *, reason: str) -> None:
    if not dest:
      self._set_nav_status('idle')
      return
    if self.params.get_bool('IsOffroad'):
      self._set_nav_status('paused_offroad')
      return
    if not location_valid or self._last_position is None:
      self._set_nav_status('waiting_for_gps')
      return
    if reason == 'reroute' and now < self._reroute_request_not_before:
      self._set_nav_status('route_active')
      return
    failure_raw = self.params.get(NAV_ROUTE_FAILURE_KEY) or ''
    if isinstance(failure_raw, bytes): failure_raw = failure_raw.decode('utf-8', errors='replace')
    if failure_raw:
      try:
        failure = json.loads(failure_raw)
      except (TypeError, ValueError, json.JSONDecodeError):
        failure = None
      failed_id = str(failure.get('requestId') or '') if isinstance(failure, dict) else ''
      if failed_id and failed_id != self._last_failed_request_id:
        self._last_failed_request_id = failed_id
        self._route_request_failures += 1
        self._route_request_cooldown_until = now + ROUTE_REQUEST_COOLDOWN_SECONDS
        cloudlog.warning(
          f"navigationd route_request failure request={failed_id} "
          f"attempt={self._route_request_failures} reason={failure.get('reason', '') if isinstance(failure, dict) else 'invalid'}"
        )
    if self._route_request_failures >= ROUTE_FAILURE_RETRY_LIMIT:
      if not route_failure_retry_ready(
        now, self._route_request_cooldown_until, self._route_request_failures,
      ):
        self._set_nav_status('route_unavailable')
        return
      cloudlog.warning(
        f"navigationd route_request retry_after_cooldown "
        f"attempt={self._route_request_failures}"
      )
      self._route_request_failures = 0
      self._last_failed_request_id = ''
      self._route_request_cooldown_until = 0.0
      self.params.remove(NAV_ROUTE_FAILURE_KEY)
    raw = self.params.get(NAV_ROUTE_REQUEST_KEY) or ''
    if isinstance(raw, bytes): raw = raw.decode('utf-8', errors='replace')
    if raw:
      try:
        request = json.loads(raw)
      except (TypeError, ValueError, json.JSONDecodeError):
        request = None
      request_id = str(request.get('requestId') or '') if isinstance(request, dict) else ''
      request_dest = parse_destination_json(request.get('destination')) if isinstance(request, dict) else None
      request_session = str(request.get('sessionId') or '') if isinstance(request, dict) else ''
      session_id = str(self.params.get(NAV_SESSION_ID_KEY) or 'legacy')
      request_stop_version = request.get('stopVersion') if isinstance(request, dict) else None
      if request_id and (
        self._dest_key(request_dest) != self._dest_key(dest)
        or request_session != session_id
        or (
          request_stop_version is not None
          and (type(request_stop_version) is not int or request_stop_version != self._stop_version())
        )
      ):
        self._clear_route_request('stale_destination_session_or_stops')
        raw = ''
        request_id = ''
      try:
        created_at = float(request.get('createdAtMonotonic') or 0.0) if isinstance(request, dict) else 0.0
      except (TypeError, ValueError, OverflowError):
        created_at = 0.0
      if not isfinite(created_at):
        created_at = 0.0
      try:
        transfer_started_at = float(request.get('transferStartedAtMonotonic') or 0.0) if isinstance(request, dict) else 0.0
        transfer_updated_at = float(request.get('transferUpdatedAtMonotonic') or 0.0) if isinstance(request, dict) else 0.0
      except (TypeError, ValueError, OverflowError):
        transfer_started_at = transfer_updated_at = 0.0
      transfer_active = bool(
        transfer_started_at and transfer_updated_at
        and isfinite(transfer_started_at) and isfinite(transfer_updated_at)
        and created_at <= transfer_started_at <= transfer_updated_at <= now
        and now - transfer_updated_at < ROUTE_TRANSFER_IDLE_TIMEOUT_SECONDS
      )
      if request_id and transfer_active:
        self._route_request_id = request_id
        self._route_request_started_at = created_at
        self._set_nav_status('receiving_route')
        return
      if request_id and transfer_started_at:
        self._route_request_cooldown_until = now + ROUTE_REQUEST_COOLDOWN_SECONDS
        if request_id != self._last_failed_request_id:
          self._route_request_failures += 1
          self._last_failed_request_id = request_id
        self.params.put(NAV_ROUTE_FAILURE_KEY, json.dumps({
          'requestId': request_id, 'reason': 'transfer_timeout',
        }))
        self._clear_route_request('transfer_timeout')
        self._set_nav_status('route_unavailable')
        return
      if request_id and created_at and now - created_at < ROUTE_REQUEST_TIMEOUT_SECONDS:
        self._route_request_id = request_id
        self._route_request_started_at = created_at
        self._set_nav_status('requesting_route')
        return
      if request_id:
        self._route_request_cooldown_until = now + ROUTE_REQUEST_COOLDOWN_SECONDS
        self._last_failed_request_id = request_id
        self._route_request_failures += 1
        self.params.put(NAV_ROUTE_FAILURE_KEY, json.dumps({'requestId': request_id, 'reason': 'timeout'}))
        self._clear_route_request('timeout')
        self._set_nav_status('phone_timeout')
        return
      self._clear_route_request('invalid_request')
    if now < self._route_request_cooldown_until:
      self._set_nav_status('route_unavailable')
      return
    session_id = str(self.params.get(NAV_SESSION_ID_KEY) or 'legacy')
    self._route_request_id = f"{session_id}:{self._route_generation}:{int(now * 1000)}"
    self._route_request_started_at = now
    request = {
      'requestId': self._route_request_id,
      'sessionId': session_id,
      'routeGeneration': self._route_generation,
      'stopVersion': self._stop_version(),
      'reason': reason,
      'createdAtMonotonic': now,
      'destination': dest,
      'start': {
        'latitude': self._last_position.latitude,
        'longitude': self._last_position.longitude,
      },
      'bearing': self._last_bearing,
    }
    self.params.put(NAV_ROUTE_REQUEST_KEY, json.dumps(request))
    self._set_nav_status('requesting_route')
    cloudlog.warning(
      f"navigationd route_request start request={self._route_request_id} reason={reason} "
      f"dest={self._last_dest_key} stop_version={self._stop_version()} "
      f"lat={self._last_position.latitude:.6f} lon={self._last_position.longitude:.6f}"
    )

  def _advance_stop(self, expected_dest_key: str, expected_session_id: str, *, reason: str) -> bool:
    with nav_stop_list_lock():
      stops = read_stop_list(self.params)
      current_dest = parse_destination_json(self.params.get("NavDestination"))
      current_key = self._dest_key(current_dest)
      current_session_id = str(self.params.get(NAV_SESSION_ID_KEY) or 'legacy')
      if (
        not stops or current_key != expected_dest_key or self._dest_key(stops[0]) != current_key
        or current_session_id != expected_session_id
      ):
        cloudlog.warning(
          f"navigationd stop_advance skipped stale_state expected={expected_dest_key}/{expected_session_id} "
          f"active={current_key}/{current_session_id} head={self._dest_key(stops[0]) if stops else 'none'}"
        )
        return False

      if len(stops) > 1:
        next_stops = write_stop_list(self.params, stops[1:], index=0)
        self._clear_route_request(f'{reason}_next_stop')
        self._clear_route(reason=f'{reason}_next_stop')
        self.params.put(NAV_SESSION_ID_KEY, f'nav-{uuid4().hex}')
        self.params.put_bool('NavRerouteNeeded', True)
        self._set_nav_status('advancing_stop')
        self._skip_arrival_until = max(self._skip_arrival_until, monotonic() + SKIP_ARRIVAL_GRACE_SECONDS)
        cloudlog.warning(
          f"navigationd stop_advance reason={reason} remaining={len(next_stops)} "
          f"next={self._dest_key(next_stops[0])}"
        )
        return True

      self._clear_route_request(f'{reason}_complete')
      self._clear_route(remove_destination=True, reason=reason)
      write_stop_list(self.params, [], index=0)
      self.params.remove(NAV_SESSION_ID_KEY)
      final_status = 'arrived' if reason == 'arrival' else 'idle'
      self._last_nav_status = final_status
      self.params.put(NAV_STATUS_KEY, final_status)
      cloudlog.warning(f"navigationd stop_advance reason={reason} remaining=0")
      return True

  def _load_pushed_route(self) -> None:
    with nav_stop_list_lock():
      self._load_pushed_route_locked()

  def _load_pushed_route_locked(self) -> None:
    raw = self.params.get(NAV_ROUTE_DATA_KEY) or ""
    if isinstance(raw, bytes): raw = raw.decode("utf-8", errors="replace")
    if not raw:
      if self._route is not None:
        self._clear_route(reason="route_data_empty")
      return
    if raw == self._route_raw:
      if self.params.get(NAV_ROUTE_REQUEST_KEY):
        self._clear_route_request('route_accepted')
        self._reset_route_acceptance_state(same_route=True)
        self._route_request_failures = 0
        self._last_failed_request_id = ''
        self.params.remove(NAV_ROUTE_FAILURE_KEY)
        self._set_nav_status('route_active')
      return
    try:
      data = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError) as e:
      cloudlog.warning(f"navigationd route_load json_error={type(e).__name__} bytes={len(raw)}")
      return
    if not isinstance(data, dict):
      cloudlog.warning("navigationd route_load invalid_payload type=non_dict")
      return
    try:
      route = NavigationRoute.from_mapbox_route(data)
    except (KeyError, TypeError, ValueError, IndexError, OverflowError) as e:
      cloudlog.warning(
        f"navigationd route_load parse_failed type={type(e).__name__} bytes={len(raw)}"
      )
      return
    if route is None:
      cloudlog.warning(f"navigationd route_load parse_failed bytes={len(raw)}")
      return
    if self._last_dest_key and self._route_dest_key and self._last_dest_key != self._route_dest_key:
      cloudlog.warning("navigationd ignored stale route for prior destination")
      self.params.remove(NAV_ROUTE_DATA_KEY)
      return
    self._route = route
    self._route_raw = raw
    self._route_dest_key = self._last_dest_key
    self._route_session_id = str(self.params.get(NAV_SESSION_ID_KEY) or 'legacy')
    self._route_generation += 1
    self._reset_route_acceptance_state()
    self.params.put_bool("NavHasRoute", True)
    self.params.remove("NavDestinationWaypoints")
    self._clear_route_request('route_accepted')
    self._same_route_reroute_not_before = 0.0
    self._same_route_accept_position = None
    self._same_route_guard_logged = False
    self._route_request_failures = 0
    self._last_failed_request_id = ''
    self.params.remove(NAV_ROUTE_FAILURE_KEY)
    self._set_nav_status('route_active')
    cloudlog.warning(
      f"navigationd route_load ok dest={self._last_dest_key} bytes={len(raw)} "
      f"points={len(route.geometry)} steps={len(route.steps)} gen={self._route_generation} "
      f"stop_version={self._stop_version()}"
    )

  def _update_location(self) -> tuple[bool, float]:
    """Keep last good fix across GPS outages (e.g. indoor car parks). Reject huge jumps after a gap."""
    self.sm.update(0)
    v_ego = float(self.sm["carState"].vEgo) if self.sm.seen["carState"] else 0.0
    if not self.sm.updated["gpsLocation"] and not self.sm.seen["gpsLocation"]:
      return False, v_ego
    gps = self.sm["gpsLocation"]
    if not self.sm.valid["gpsLocation"]:
      return False, v_ego
    lat, lon = float(gps.latitude), float(gps.longitude)
    if not isfinite(lat) or not isfinite(lon) or (abs(lat) < 1e-6 and abs(lon) < 1e-6):
      return False, v_ego
    cand = Coordinate(lat, lon)
    # After a GPS outage, ignore an absurd first fix (car-park multipath / teleport).
    if self._last_position is not None and self._gps_was_valid is False:
      try:
        jump_m = float(cand.distance_to(self._last_position))
      except (TypeError, ValueError):
        jump_m = 0.0
      if isfinite(jump_m) and jump_m > 250.0:
        cloudlog.warning(f"navigationd ignoring GPS jump after outage ({jump_m:.0f} m)")
        return False, max(0.0, v_ego)
    self._last_position = cand
    bearing = float(getattr(gps, "bearingDeg", getattr(gps, "bearing", 0.0)) or 0.0)
    self._last_bearing = bearing if isfinite(bearing) else None
    speed = float(getattr(gps, "speed", 0.0) or 0.0)
    if self.sm.seen["carState"]:
      speed = max(speed, v_ego)
    return True, max(0.0, speed)

  @staticmethod
  def _bump_timer(started_at: float | None, condition: bool, now: float) -> float | None:
    if not condition: return None
    return started_at if started_at is not None else now

  @staticmethod
  def _timer_expired(started_at: float | None, threshold: float, now: float) -> bool:
    return started_at is not None and (now - started_at) >= threshold

  def _build_progress(self, location_valid: bool, v_ego: float) -> tuple[RouteProgress | None, dict | None]:
    if self._route is None or not location_valid or self._last_position is None:
      self._off_route_started_at = self._bearing_misaligned_started_at = self._arrival_started_at = None
      return None, None
    if (progress := self._route.get_progress(self._last_position)) is None: return None, None
    now = monotonic()
    off_route = self._route.off_route_distance_exceeded(progress, v_ego)
    misaligned = self._route.route_bearing_misaligned(progress.closest_segment_index, self._last_bearing, v_ego)
    arrived = self._route.arrived(progress)
    self._off_route_started_at = self._bump_timer(self._off_route_started_at, off_route and not arrived, now)
    self._bearing_misaligned_started_at = self._bump_timer(self._bearing_misaligned_started_at, misaligned and not arrived, now)
    self._arrival_started_at = self._bump_timer(self._arrival_started_at, arrived, now)
    if not off_route: self._off_route_started_at = None
    if not misaligned: self._bearing_misaligned_started_at = None
    if not arrived: self._arrival_started_at = None
    return progress, {"offRoute": off_route, "misaligned": misaligned, "arrived": arrived, "now": now}

  def _maybe_flags(self, progress: RouteProgress | None, route_state: dict | None) -> bool:
    if self._route is None or self._route_dest_key != self._last_dest_key or progress is None or route_state is None:
      return False
    now = float(route_state["now"])
    need_reroute = False
    if progress.current_step_index < len(self._route.steps) - 1:
      need = self._timer_expired(self._off_route_started_at, REROUTE_TRIGGER_SECONDS, now)
      need = need or self._timer_expired(self._bearing_misaligned_started_at, REROUTE_TRIGGER_SECONDS, now)
      need_reroute = bool(need)
      if need_reroute and self._same_route_reroute_blocked(now):
        need_reroute = False
      if need_reroute != self._last_reroute_needed:
        self._last_reroute_needed = need_reroute
        cloudlog.warning(
          f"navigationd reroute_needed={int(need_reroute)} dest={self._last_dest_key} "
          f"off_route={int(bool(route_state.get('offRoute')))} "
          f"misaligned={int(bool(route_state.get('misaligned')))} "
          f"remaining_m={float(progress.distance_remaining):.0f}"
        )
      with nav_stop_list_lock():
        current_dest = parse_destination_json(self.params.get("NavDestination"))
        current_session_id = str(self.params.get(NAV_SESSION_ID_KEY) or 'legacy')
        if (
          self._dest_key(current_dest) != self._route_dest_key
          or current_session_id != self._route_session_id
        ):
          return False
        self.params.put_bool("NavRerouteNeeded", need_reroute)
      if need_reroute and self._timer_expired(self._off_route_started_at, UNSURE_SUPPRESS_SECONDS, now):
        if not self._control_suppressed and not self._control_suppress_logged:
          self._control_suppress_logged = True
          cloudlog.warning(f"navigationd control_suppressed dest={self._last_dest_key}")
        self._control_suppressed = True
    if self._timer_expired(self._arrival_started_at, ARRIVAL_CLEAR_SECONDS, now):
      if now < float(self._skip_arrival_until):
        self._arrival_started_at = None
      else:
        cloudlog.warning(
          f"navigationd skip mode=auto reason=arrival dest={self._last_dest_key} "
          f"remaining_m={float(progress.distance_remaining):.1f}"
        )
        advanced = self._advance_stop(self._last_dest_key, self._route_session_id, reason='arrival')
        if not advanced:
          self._arrival_started_at = None
        return advanced
    return False

  def _publish_instruction(self, progress: RouteProgress | None, location_valid: bool) -> None:
    msg = messaging.new_message("navInstruction")
    msg.valid = bool(
      self._route is not None and progress is not None and location_valid
      and not self._control_suppressed and not self.params.get_bool('IsOffroad')
    )
    if msg.valid and progress is not None and self._route is not None:
      payload = self._route.build_instruction_payload(progress)
      ni = msg.navInstruction
      ni.maneuverPrimaryText = payload["maneuverPrimaryText"]
      ni.maneuverSecondaryText = payload["maneuverSecondaryText"]
      ni.maneuverDistance = float(payload["maneuverDistance"])
      ni.maneuverType = str(payload["maneuverType"])
      ni.maneuverModifier = str(payload["maneuverModifier"])
      ni.distanceRemaining = float(payload["distanceRemaining"])
      ni.timeRemaining = float(payload["timeRemaining"])
      ni.timeRemainingTypical = float(payload["timeRemainingTypical"])
      ni.lanes = payload["lanes"]
      ni.showFull = bool(payload["showFull"])
      ni.allManeuvers = payload["allManeuvers"]
    self.pm.send("navInstruction", msg)

  def _publish_state(self, progress: RouteProgress | None, location_valid: bool) -> None:
    if self._route is None or progress is None or not location_valid or self._control_suppressed or self.params.get_bool('IsOffroad'):
      if self._last_nav_state is not None:
        cloudlog.warning("navigationd instruction_state cleared suppressed_or_no_route")
        self.params.remove(NAV_INSTRUCTION_STATE_KEY)
        self._last_nav_state = None
      return
    payload = self._route.build_instruction_payload(progress)
    all_maneuvers = payload.get("allManeuvers") or []
    next_maneuver = all_maneuvers[1] if len(all_maneuvers) > 1 and isinstance(all_maneuvers[1], dict) else {}
    lanes = payload.get("lanes") or []
    active_lane_direction, active_lane_index = "", -1
    for index, lane in enumerate(lanes):
      if not isinstance(lane, dict) or not lane.get("active"): continue
      candidate = str(lane.get("activeDirection") or "")
      if (not candidate or candidate == "none") and len(lane.get("directions") or []) == 1:
        candidate = str((lane.get("directions") or [""])[0] or "")
      if candidate and candidate != "none":
        active_lane_direction, active_lane_index = candidate, index
        break
    active_lane_side = "left" if active_lane_direction in ("slightLeft", "left", "sharpLeft") else (
      "right" if active_lane_direction in ("slightRight", "right", "sharpRight") else "")
    same_side_lane_count, active_lane_at_road_edge, has_shared = 0, False, False
    if active_lane_side:
      same = {"slightLeft", "left", "sharpLeft"} if active_lane_side == "left" else {"slightRight", "right", "sharpRight"}
      active_lane_at_road_edge = active_lane_index == 0 if active_lane_side == "left" else active_lane_index == len(lanes) - 1
      for lane in lanes:
        if not isinstance(lane, dict): continue
        directions = {str(d) for d in lane.get("directions") or [] if d}
        if directions & same:
          same_side_lane_count += 1
          has_shared |= len(directions - same) > 0
    state = {
      "valid": True,
      "maneuverModifier": str(payload.get("maneuverModifier") or ""),
      "maneuverType": str(payload.get("maneuverType") or ""),
      "laneCount": len(lanes),
      "activeLaneDirection": active_lane_direction,
      "activeLaneIndex": active_lane_index,
      "activeLaneAtRoadEdge": active_lane_at_road_edge,
      "hasSharedSameSideLane": has_shared,
      "sameSideLaneCount": same_side_lane_count,
      "maneuverPrimaryText": str(payload.get("maneuverPrimaryText") or ""),
      "maneuverSecondaryText": str(payload.get("maneuverSecondaryText") or ""),
      "maneuverDistance": float(payload.get("maneuverDistance") or 0.0),
      "nextManeuverType": str(next_maneuver.get("type") or ""),
      "nextManeuverModifier": str(next_maneuver.get("modifier") or ""),
      "nextManeuverDistance": float(next_maneuver.get("distance") or 0.0),
    }
    if state != self._last_nav_state:
      self.params.put(NAV_INSTRUCTION_STATE_KEY, json.dumps(state))
      self._last_nav_state = state
      cloudlog.warning(
        f"navigationd instruction_state type={state.get('maneuverType')} "
        f"mod={state.get('maneuverModifier')} dist_m={float(state.get('maneuverDistance') or 0):.0f} "
        f"lanes={int(state.get('laneCount') or 0)} active={state.get('activeLaneDirection')} "
        f"next={state.get('nextManeuverType')}/{state.get('nextManeuverModifier')}"
      )

  def _publish_route_if_needed(self) -> None:
    if self._route_generation == self._published_route_generation: return
    msg = messaging.new_message("navRoute")
    msg.valid = self._route is not None
    if self._route is not None:
      msg.navRoute.coordinates = [{"latitude": c.latitude, "longitude": c.longitude} for c in self._route.geometry]
    self.pm.send("navRoute", msg)
    self._published_route_generation = self._route_generation

  def run(self) -> None:
    cloudlog.warning("navigationd init")
    while True:
      location_valid, v_ego = self._update_location()
      now_mono = monotonic()
      if location_valid and not self._gps_was_valid:
        # Brief grace after GPS returns so multipath does not instantly trigger a reroute.
        position = self._last_position
        cloudlog.warning(
          f"navigationd gps_regained lat={position.latitude:.5f} lon={position.longitude:.5f} "
          f"speed_mps={v_ego:.2f} mono={now_mono:.3f} grace_sec=4"
        )
        self._gps_regain_grace_until = now_mono + 4.0
        self._off_route_started_at = self._bearing_misaligned_started_at = None
        self._control_suppressed = False
      elif not location_valid and self._gps_was_valid:
        cloudlog.warning(f"navigationd gps_lost mono={now_mono:.3f}")
      self._gps_was_valid = bool(location_valid)
      in_gps_grace = bool(location_valid) and now_mono < float(self._gps_regain_grace_until)
      with nav_stop_list_lock():
        stops = read_stop_list(self.params)
        dest = parse_destination_json(self.params.get("NavDestination"))
        is_offroad = self.params.get_bool('IsOffroad')
        session_id = str(self.params.get(NAV_SESSION_ID_KEY) or '')
        if not self._boot_state_logged:
          self._boot_state_logged = True
          cloudlog.warning(
            f"navigationd boot_state stops={len(stops)} dest={self._dest_key(dest)} "
            f"route_data={int(bool(self.params.get(NAV_ROUTE_DATA_KEY)))} "
            f"gps_valid={int(location_valid)} offroad={int(is_offroad)} session={int(bool(session_id))}"
          )
          if stops:
            self._clear_route_request('boot_recovery')
            self._clear_route(reason='boot_recovery')
            self.params.remove(NAV_ROUTE_FAILURE_KEY)
            self.params.put_bool('NavHasRoute', False)
            self.params.put_bool('NavRerouteNeeded', True)
            self._set_nav_status(
              'paused_offroad' if is_offroad else ('waiting_for_gps' if not location_valid else 'route_pending')
            )
        if stops and is_offroad:
          if dest is None:
            self._set_nav_status('paused_offroad')
        elif stops and (self._dest_key(dest) != self._dest_key(stops[0]) or not session_id):
          set_destination(self.params, stops[0])
          self._clear_route_request('restore_persistent_stops')
          self._clear_route(reason='restore_persistent_stops')
          self.params.remove(NAV_ROUTE_FAILURE_KEY)
          self.params.put(NAV_SESSION_ID_KEY, f'nav-{uuid4().hex}')
          self.params.put_bool('NavHasRoute', False)
          self.params.put_bool('NavRerouteNeeded', True)
          self._set_nav_status('route_pending')
          dest = stops[0]
          session_id = str(self.params.get(NAV_SESSION_ID_KEY) or '')
          cloudlog.warning(
            f"navigationd stops_restore source=device count={len(stops)} "
            f"active={self._dest_key(dest)}"
          )
      if session_id != self._last_session_id:
        if self._last_session_id:
          cloudlog.warning(
            f"navigationd session_changed {self._last_session_id} -> {session_id or 'none'}"
          )
          with nav_stop_list_lock():
            self._clear_route_request('session_changed')
        self._last_session_id = session_id
        self._route_request_failures = 0
        self._last_failed_request_id = ''
        self._route_request_cooldown_until = 0.0
      dest_key = self._dest_key(dest)
      if dest_key != self._last_dest_key:
        with nav_stop_list_lock():
          current_dest = parse_destination_json(self.params.get("NavDestination"))
          if self._dest_key(current_dest) == dest_key:
            self._clear_route_request('destination_changed')
        if self._last_dest_key and dest_key:
          # Phone skipped or advanced stop — drop stale geometry until fresh route arrives.
          cloudlog.warning(
            f"navigationd dest_changed {self._last_dest_key} -> {dest_key} "
            f"clearing_stale_route"
          )
          self._clear_route(remove_destination=False, reason="dest_changed")
          self.params.put_bool("NavRerouteNeeded", True)
          self._skip_arrival_until = max(float(self._skip_arrival_until), now_mono + SKIP_ARRIVAL_GRACE_SECONDS)
        elif dest_key and not self._last_dest_key:
          dest_name = str((dest or {}).get('name') or (dest or {}).get('place_name') or '')[:48]
          cloudlog.warning(f"navigationd dest_set key={dest_key} name={dest_name}")
        elif self._last_dest_key and not dest_key:
          cloudlog.warning(f"navigationd dest_cleared was={self._last_dest_key}")
        self._last_dest_key = dest_key
        self._route_request_failures = 0
        self._last_failed_request_id = ''
        self.params.remove(NAV_ROUTE_FAILURE_KEY)
      if dest is None and self._route is not None:
        self._clear_route(reason="dest_missing")
      else:
        if is_offroad:
          # Do not accept a route response that raced with the offroad transition.
          # The persistent stop list remains authoritative and will request a fresh
          # route after the next valid on-road fix.
          self._clear_route_request('paused_offroad')
          if self._route is None and self.params.get(NAV_ROUTE_DATA_KEY):
            self.params.remove(NAV_ROUTE_DATA_KEY)
            cloudlog.warning(
              f"navigationd route_response ignored_offroad dest={self._last_dest_key}"
            )
        else:
          self._load_pushed_route()
      # Route is kept while GPS is lost; progress simply pauses until a good fix returns.
      progress, route_state = self._build_progress(location_valid, v_ego)
      stop_advanced = False
      if not in_gps_grace:
        stop_advanced = self._maybe_flags(progress, route_state)
      if not stop_advanced:
        if dest is not None and self.params.get_bool('IsOffroad'):
          self._set_nav_status('paused_offroad')
        elif dest is not None and not location_valid:
          self._set_nav_status('waiting_for_gps')
        elif dest is not None and (self._route is None or self.params.get_bool('NavRerouteNeeded')):
          self._maybe_request_route(
            dest, location_valid, now_mono,
            reason='initial' if self._route is None else 'reroute',
          )
        elif self._route is not None and not self.params.get_bool('IsOffroad') and not self._control_suppressed:
          self._set_nav_status('route_active')
      # Re-read dest in case arrival cleared it mid-loop
      with nav_stop_list_lock():
        dest = parse_destination_json(self.params.get("NavDestination"))
      if self._route is None: progress = None
      if self._route is not None and (now_mono - self._last_status_log_at) >= 30.0:
        self._last_status_log_at = now_mono
        engaged = bool(
          self.sm.valid.get("selfdriveState") and self.sm["selfdriveState"].enabled
        )
        rem = float(progress.distance_remaining) if progress is not None else -1.0
        step = int(progress.current_step_index) if progress is not None else -1
        cloudlog.warning(
          f"navigationd status dest={self._last_dest_key} remaining_m={rem:.0f} step={step} "
          f"reroute={int(self.params.get_bool('NavRerouteNeeded'))} "
          f"engaged={int(engaged)} gps={int(location_valid)} "
          f"suppressed={int(self._control_suppressed)}"
        )
      with nav_stop_list_lock():
        current_dest = parse_destination_json(self.params.get("NavDestination"))
        current_session_id = str(self.params.get(NAV_SESSION_ID_KEY) or 'legacy')
        if self._route is not None and (
          self._dest_key(current_dest) != self._route_dest_key
          or current_session_id != self._route_session_id
        ):
          self._clear_route_request('destination_changed_before_publish')
          self._clear_route(reason='destination_changed_before_publish')
          self.params.put_bool('NavRerouteNeeded', bool(read_stop_list(self.params)))
          progress = None
        self._publish_instruction(progress, location_valid)
        self._publish_state(progress, location_valid)
        self._publish_route_if_needed()
      self.rk.keep_time()


def main() -> None:
  Navigationd().run()


if __name__ == "__main__":
  main()

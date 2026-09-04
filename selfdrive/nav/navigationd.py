#!/usr/bin/env python3
from __future__ import annotations
import json
from math import isfinite
from time import monotonic

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.nav.destination_store import (
  NAV_INSTRUCTION_STATE_KEY, NAV_ROUTE_DATA_KEY, parse_destination_json,
)
from openpilot.selfdrive.nav.route_engine import Coordinate, NavigationRoute, RouteProgress

NAVIGATIOND_HZ = 1
REROUTE_TRIGGER_SECONDS = 2.0
ARRIVAL_CLEAR_SECONDS = 3.0
SKIP_ARRIVAL_GRACE_SECONDS = 12.0  # phone MAX_STOPS queue / skip-episode
UNSURE_SUPPRESS_SECONDS = 3.0
ABANDON_HOLD_SECONDS = 35.0
ABANDON_AWAY_DELTA_M = 180.0
ABANDON_REMAINING_GROW_M = 250.0


class Navigationd:
  def __init__(self):
    self.params = Params()
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
    self._abandon_started_at: float | None = None
    self._min_dist_to_dest: float | None = None
    self._min_remaining: float | None = None
    self._last_nav_state: dict | None = None
    self._control_suppressed = False
    self._last_dest_key = ""
    self._route_dest_key = ""
    self._skip_arrival_until = 0.0
    self._last_reroute_needed: bool | None = None

  def _dest_key(self, dest: dict | None) -> str:
    if not dest:
      return ""
    try:
      return f"{float(dest['latitude']):.5f},{float(dest['longitude']):.5f}"
    except (KeyError, TypeError, ValueError):
      return ""

  def _clear_route(self, *, remove_destination: bool = False, reason: str = "") -> None:
    had_route = self._route is not None
    self._route = None
    self._route_raw = ""
    self._route_generation += 1
    self._off_route_started_at = None
    self._bearing_misaligned_started_at = None
    self._arrival_started_at = None
    self._abandon_started_at = None
    self._min_dist_to_dest = None
    self._min_remaining = None
    self._control_suppressed = False
    self.params.remove(NAV_ROUTE_DATA_KEY)
    self.params.remove(NAV_INSTRUCTION_STATE_KEY)
    self.params.put_bool("NavRerouteNeeded", False)
    self.params.put_bool("NavHasRoute", False)
    self._route_dest_key = ""
    self._last_nav_state = None
    if remove_destination:
      self.params.remove("NavDestination")
      if reason:
        self.params.put("NavDestinationWaypoints", json.dumps({"clearReason": reason}))
      else:
        self.params.remove("NavDestinationWaypoints")
    if had_route or remove_destination or reason:

  def _load_pushed_route(self) -> None:
    raw = self.params.get(NAV_ROUTE_DATA_KEY) or ""
    if isinstance(raw, bytes): raw = raw.decode("utf-8", errors="replace")
    if not raw:
      if self._route is not None:
        self._clear_route(reason="route_data_empty")
      return
    if raw == self._route_raw:
      return
    try:
      data = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError) as e:
      return
    if not isinstance(data, dict):
      return
    if (route := NavigationRoute.from_mapbox_route(data)) is None:
      return
    if self._last_dest_key and self._route_dest_key and self._last_dest_key != self._route_dest_key:
      self.params.remove(NAV_ROUTE_DATA_KEY)
      return
    self._route = route
    self._route_raw = raw
    self._route_dest_key = self._last_dest_key
    self._route_generation += 1
    self._off_route_started_at = None
    self._bearing_misaligned_started_at = None
    self._arrival_started_at = None
    self._abandon_started_at = None
    self._min_dist_to_dest = None
    self._min_remaining = None
    self._control_suppressed = False
    self.params.put_bool("NavHasRoute", True)
    self.params.put_bool("NavRerouteNeeded", False)
    self.params.remove("NavDestinationWaypoints")
    self._last_reroute_needed = False

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
    arrived = self._route.arrived(progress, v_ego)
    self._off_route_started_at = self._bump_timer(self._off_route_started_at, off_route and not arrived, now)
    self._bearing_misaligned_started_at = self._bump_timer(self._bearing_misaligned_started_at, misaligned and not arrived, now)
    self._arrival_started_at = self._bump_timer(self._arrival_started_at, arrived, now)
    if not off_route: self._off_route_started_at = None
    if not misaligned: self._bearing_misaligned_started_at = None
    if not arrived: self._arrival_started_at = None
    return progress, {"offRoute": off_route, "misaligned": misaligned, "arrived": arrived, "now": now}

  def _maybe_abandon(self, dest: dict | None, location_valid: bool, route_state: dict | None, progress: RouteProgress | None) -> None:
    """Clear pin anytime mid-drive when clearly leaving the destination; keep alternate approaches."""
    if dest is None or not location_valid or self._last_position is None:
      self._abandon_started_at = None
      return
    try:
      dist = float(self._last_position.distance_to(Coordinate(float(dest["latitude"]), float(dest["longitude"]))))
    except (KeyError, TypeError, ValueError):
      return
    if not isfinite(dist): return
    # Works from any point in the trip (not only near the destination).
    self._min_dist_to_dest = dist if self._min_dist_to_dest is None else min(self._min_dist_to_dest, dist)
    away = dist > (self._min_dist_to_dest + ABANDON_AWAY_DELTA_M)
    remaining_grew = False
    if progress is not None:
      try: remaining = float(progress.distance_remaining)
      except (TypeError, ValueError): remaining = None
      if remaining is not None and isfinite(remaining):
        self._min_remaining = remaining if self._min_remaining is None else min(self._min_remaining, remaining)
        remaining_grew = remaining > (self._min_remaining + ABANDON_REMAINING_GROW_M)
    off = bool(route_state and route_state.get("offRoute")) or self._route is None or self._control_suppressed
    # Still getting closer via another road → min_dist falls → away stays false (keep + reroute).
    # Leaving for real: farther from dest AND (off the corridor OR going back along the route).
    now = monotonic()
    considering = away and (off or remaining_grew)
    self._abandon_started_at = self._bump_timer(self._abandon_started_at, considering, now)
    if not considering:
      self._abandon_started_at = None
    if self._timer_expired(self._abandon_started_at, ABANDON_HOLD_SECONDS, now):
      self._clear_route(remove_destination=True, reason="abandon")

  def _maybe_flags(self, progress: RouteProgress | None, route_state: dict | None) -> None:
    if self._route is None or progress is None or route_state is None: return
    now = float(route_state["now"])
    if progress.current_step_index < len(self._route.steps) - 1:
      need = self._timer_expired(self._off_route_started_at, REROUTE_TRIGGER_SECONDS, now)
      need = need or self._timer_expired(self._bearing_misaligned_started_at, REROUTE_TRIGGER_SECONDS, now)
      need_reroute = bool(need)
      if need_reroute != self._last_reroute_needed:
        self._last_reroute_needed = need_reroute
      self.params.put_bool("NavRerouteNeeded", need_reroute)
      if need_reroute and self._timer_expired(self._off_route_started_at, UNSURE_SUPPRESS_SECONDS, now):
        self._control_suppressed = True
    if self._timer_expired(self._arrival_started_at, ARRIVAL_CLEAR_SECONDS, now):
      if now < float(self._skip_arrival_until):
        self._arrival_started_at = None
      else:
        self._clear_route(remove_destination=True, reason="arrival")

  def _publish_instruction(self, progress: RouteProgress | None, location_valid: bool) -> None:
    msg = messaging.new_message("navInstruction")
    msg.valid = bool(self._route is not None and progress is not None and location_valid and not self._control_suppressed)
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
    if self._route is None or progress is None or not location_valid or self._control_suppressed:
      if self._last_nav_state is not None:
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

  def _publish_route_if_needed(self) -> None:
    if self._route_generation == self._published_route_generation: return
    msg = messaging.new_message("navRoute")
    msg.valid = self._route is not None
    if self._route is not None:
      msg.navRoute.coordinates = [{"latitude": c.latitude, "longitude": c.longitude} for c in self._route.geometry]
    self.pm.send("navRoute", msg)
    self._published_route_generation = self._route_generation

  def run(self) -> None:
    while True:
      location_valid, v_ego = self._update_location()
      now_mono = monotonic()
      if location_valid and not self._gps_was_valid:
        # Brief grace after GPS returns so multipath does not instantly off-route/abandon.
        self._gps_regain_grace_until = now_mono + 4.0
        self._off_route_started_at = self._bearing_misaligned_started_at = None
        self._abandon_started_at = None
        self._control_suppressed = False
      self._gps_was_valid = bool(location_valid)
      in_gps_grace = bool(location_valid) and now_mono < float(self._gps_regain_grace_until)
      dest = parse_destination_json(self.params.get("NavDestination"))
      dest_key = self._dest_key(dest)
      if dest_key != self._last_dest_key:
        if self._last_dest_key and dest_key:
          # Phone skipped or advanced stop — drop stale geometry until fresh route arrives.
          self._clear_route(remove_destination=False, reason="dest_changed")
          self.params.put_bool("NavRerouteNeeded", True)
          self._skip_arrival_until = max(float(self._skip_arrival_until), now_mono + SKIP_ARRIVAL_GRACE_SECONDS)
        elif dest_key and not self._last_dest_key:
          dest_name = str((dest or {}).get('name') or (dest or {}).get('place_name') or '')[:48]
        elif self._last_dest_key and not dest_key:
        self._last_dest_key = dest_key
      if dest is None and self._route is not None:
        self._clear_route(reason="dest_missing")
      else:
        self._load_pushed_route()
      # Route is kept while GPS is lost; progress simply pauses until a good fix returns.
      progress, route_state = self._build_progress(location_valid, v_ego)
      if not in_gps_grace:
        self._maybe_flags(progress, route_state)
      # Re-read dest in case arrival cleared it mid-loop
      dest = parse_destination_json(self.params.get("NavDestination"))
      if not in_gps_grace:
        self._maybe_abandon(dest, location_valid, route_state, progress)
      if self._route is None: progress = None
      if self._route is not None and (now_mono - self._last_status_log_at) >= 30.0:
        self._last_status_log_at = now_mono
        engaged = bool(
          self.sm.valid.get("selfdriveState") and self.sm["selfdriveState"].enabled
        )
        rem = float(progress.distance_remaining) if progress is not None else -1.0
        step = int(progress.current_step_index) if progress is not None else -1
      self._publish_instruction(progress, location_valid)
      self._publish_state(progress, location_valid)
      self._publish_route_if_needed()
      self.rk.keep_time()


def main() -> None:
  Navigationd().run()


if __name__ == "__main__":
  main()

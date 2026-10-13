from math import isfinite

from openpilot.selfdrive.nav.destination_store import read_stop_list

SAME_ROUTE_REROUTE_HOLD_SECONDS = 30.0
SAME_ROUTE_REROUTE_DISTANCE_M = 40.0
ROUTE_FAILURE_RETRY_LIMIT = 3


def route_failure_retry_ready(now, cooldown_until, failures) -> bool:
  return failures >= ROUTE_FAILURE_RETRY_LIMIT and now >= cooldown_until


def has_persistent_nav_stops(params) -> bool:
  return bool(read_stop_list(params))


def same_route_reroute_blocked(now, hold_until, current_position, accepted_position) -> bool:
  if now >= hold_until or current_position is None or accepted_position is None:
    return False
  try:
    moved_m = float(current_position.distance_to(accepted_position))
  except (TypeError, ValueError):
    return False
  return isfinite(moved_m) and moved_m < SAME_ROUTE_REROUTE_DISTANCE_M

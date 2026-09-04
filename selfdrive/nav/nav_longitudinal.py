from __future__ import annotations
import json, math
from openpilot.common.constants import CV
from openpilot.common.params import Params

NAV_TURN_COMFORT_DECEL = 1.25
NAV_TURN_DISTANCE_BUFFER = 8.0
NAV_TURN_MIN_TARGET_DELTA = 0.25
NAV_TURN_TARGET_SPEEDS = {
  "uturn": 5.0 * CV.MPH_TO_MS,
  "sharpLeft": 10.0 * CV.MPH_TO_MS,
  "sharpRight": 10.0 * CV.MPH_TO_MS,
  "left": 14.0 * CV.MPH_TO_MS,
  "right": 14.0 * CV.MPH_TO_MS,
}
ROUNDABOUT_SPEED = 12.0 * CV.MPH_TO_MS


def _maneuver_target(maneuver_type: str, modifier: str) -> float | None:
  t, m = (maneuver_type or "").lower(), (modifier or "").lower()
  if m == "uturn" or "uturn" in t or "u-turn" in t: return NAV_TURN_TARGET_SPEEDS["uturn"]
  if "roundabout" in t or "rotary" in t: return ROUNDABOUT_SPEED
  if t == "turn": return NAV_TURN_TARGET_SPEEDS.get(modifier) or NAV_TURN_TARGET_SPEEDS.get(m)
  return None


def _target_for_distance(target_speed: float, maneuver_distance: float) -> float:
  remaining = max(maneuver_distance - NAV_TURN_DISTANCE_BUFFER, 0.0)
  return math.sqrt(target_speed * target_speed + 2.0 * NAV_TURN_COMFORT_DECEL * remaining)



def nav_turn_speed_ceiling(v_cruise: float, min_steer_speed: float = 0.0) -> float:
  """Return cruise ceiling (m/s); >= v_cruise means no change."""
  if not Params().get_bool("NavLongitudinalAllowed"): return v_cruise
  try: state = json.loads(Params().get("NavInstructionState") or "")
  except (TypeError, ValueError, json.JSONDecodeError): return v_cruise
  if not isinstance(state, dict) or not state.get("valid"): return v_cruise

  candidates = [
    (str(state.get("maneuverType") or ""), str(state.get("maneuverModifier") or ""), float(state.get("maneuverDistance") or 0.0)),
    (str(state.get("nextManeuverType") or ""), str(state.get("nextManeuverModifier") or ""), float(state.get("nextManeuverDistance") or 0.0)),
  ]
  best = None
  for mtype, mod, dist in candidates:
    if (target := _maneuver_target(mtype, mod)) is None: continue
    target = max(target, min_steer_speed)
    if target + NAV_TURN_MIN_TARGET_DELTA >= v_cruise: continue
    ceiling = max(target, _target_for_distance(target, dist))
    best = ceiling if best is None else min(best, ceiling)
  result = v_cruise if best is None else min(v_cruise, best)
  return _log_nav_ceiling(v_cruise, result, state)

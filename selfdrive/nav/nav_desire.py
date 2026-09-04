from __future__ import annotations
import json
from cereal import log
from openpilot.common.constants import CV
from openpilot.common.params import Params
from numpy import interp


LaneChangeDirection = log.LaneChangeDirection
TURN_IMMINENT_V = [0.0, 5.0, 10.0]
TURN_IMMINENT_D = [20.0, 25.0, 30.0]
KEEP_IMMINENT_V = [0.0, 15.0, 30.0]
KEEP_IMMINENT_D = [25.0, 90.0, 160.0]
AMBIGUOUS_SPLIT_SCALE = 0.6
LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS


def _load_state() -> dict:
  try: state = json.loads(Params().get("NavInstructionState") or "")
  except (TypeError, ValueError, json.JSONDecodeError): return {}
  return state if isinstance(state, dict) else {}


def _effective_modifier(state: dict, v_ego: float, maneuver_distance: float) -> str:
  modifier = str(state.get("maneuverModifier") or "")
  mtype = str(state.get("maneuverType") or "").lower()
  if mtype in ("off ramp", "fork") and any(x in modifier.lower() for x in ("left", "right")):
    keep_dist = float(interp(v_ego, KEEP_IMMINENT_V, KEEP_IMMINENT_D))
    if (int(state.get("sameSideLaneCount") or 0) > 1 and
        (int(state.get("laneCount") or 0) == 0 or int(state.get("laneCount") or 0) - int(state.get("sameSideLaneCount") or 0) <= 2)):
      keep_dist *= AMBIGUOUS_SPLIT_SCALE
    if maneuver_distance > keep_dist: return ""
    if state.get("activeLaneAtRoadEdge") and state.get("hasSharedSameSideLane"): return ""
    active = str(state.get("activeLaneDirection") or "")
    if active in ("slightLeft", "left"): return "slightLeft"
    if active in ("slightRight", "right"): return "slightRight"
    if active == "straight": return ""
    return ""
  return modifier



def _nav_context(carstate, lateral_active: bool):
  params = Params()
  if not params.get_bool("NavDesiresAllowed") or not lateral_active:
    return None
  if not (state := _load_state()).get("valid"):
    return None
  maneuver_distance = float(state.get("maneuverDistance") or 0.0)
  modifier = _effective_modifier(state, carstate.vEgo, maneuver_distance)
  if not modifier:
    return None
  return state, modifier, maneuver_distance


def navigation_suppresses_alc(carstate, lateral_active: bool) -> bool:
  """Block ALC while nav has an imminent maneuver so ALC cannot fight nav."""
  ctx = _nav_context(carstate, lateral_active)
  if ctx is None:
    return False
  _state, modifier, maneuver_distance = ctx
  if modifier in ("left", "sharpLeft", "right", "sharpRight"):
    if carstate.standstill or carstate.vEgo >= LANE_CHANGE_SPEED_MIN:
      return False
    return maneuver_distance <= float(interp(carstate.vEgo, TURN_IMMINENT_V, TURN_IMMINENT_D))
  if modifier == "slightLeft":
    if not Params().get_bool("NavLanePositioningAllowed"):
      return False
    return carstate.leftBlinker and not carstate.rightBlinker
  if modifier == "slightRight":
    if not Params().get_bool("NavLanePositioningAllowed"):
      return False
    return carstate.rightBlinker and not carstate.leftBlinker
  return False


def navigation_desire(carstate, lateral_active: bool) -> log.Desire:
  params = Params()
  ctx = _nav_context(carstate, lateral_active)
  if ctx is None:
    return log.Desire.none
  state, modifier, maneuver_distance = ctx

  lane_pos = params.get_bool("NavLanePositioningAllowed")
  if modifier == "slightLeft":
    if not lane_pos or carstate.rightBlinker or carstate.leftBlindspot:
      return log.Desire.none
    if carstate.steeringPressed and carstate.steeringTorque > 0:
      return log.Desire.keepLeft
    return log.Desire.none
  if modifier == "slightRight":
    if not lane_pos or carstate.leftBlinker or carstate.rightBlindspot:
      return log.Desire.none
    if carstate.steeringPressed and carstate.steeringTorque < 0:
      return log.Desire.keepRight
    return log.Desire.none
  if modifier in ("left", "sharpLeft"):
    if carstate.rightBlinker or carstate.leftBlindspot or carstate.standstill:
      return log.Desire.none
    if carstate.vEgo >= LANE_CHANGE_SPEED_MIN:
      return log.Desire.none
    if maneuver_distance <= float(interp(carstate.vEgo, TURN_IMMINENT_V, TURN_IMMINENT_D)):
      return log.Desire.turnLeft
  if modifier in ("right", "sharpRight"):
    if carstate.leftBlinker or carstate.rightBlindspot or carstate.standstill:
      return log.Desire.none
    if carstate.vEgo >= LANE_CHANGE_SPEED_MIN:
      return log.Desire.none
    if maneuver_distance <= float(interp(carstate.vEgo, TURN_IMMINENT_V, TURN_IMMINENT_D)):
      return log.Desire.turnRight
  return log.Desire.none

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.nav.nav_desire import navigation_desire, navigation_suppresses_alc
from numpy import clip
from enum import Enum, auto
import time

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
ALC_CANCEL_DELAY = 1.75 # In seconds

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}

def _desire_name(desire: log.Desire) -> str:
  return desire if isinstance(desire, str) else str(desire).split(".")[-1]



class Dir(Enum):
  LEFT = auto()
  RIGHT = auto()

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_blinker = None # Handle direction change
    self.desire = log.Desire.none
    self.last_alc_cancel = 0
    self.is_alc_enabled = Params().get_bool("IsAlcEnabled")
    self.blinker_below_lane_change_speed = False
    self._last_final_desire_log_key: tuple | None = None

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def is_road_edge_blinker(self, md, rightBlinker, leftBlinker):
    if md is None:
      return False

    # road_edge_stat calculation adapted from
    # https://github.com/kisapilot/openpilot/blob/93c8046/selfdrive/controls/lib/desire_helper.py

    # WARNING: No threshold can determine all road edges correctly. Driver check is still required.
    edge_threshold = 0.475  # Higher value filters out low-confidence road edges
    left_edge_prob = clip(1.0 - md.roadEdgeStds[0], 0.0, 1.0)
    right_edge_prob = clip(1.0 - md.roadEdgeStds[1], 0.0, 1.0)
    left_nearside_prob, right_nearside_prob = md.laneLineProbs[0], md.laneLineProbs[3]

    if right_edge_prob > edge_threshold and right_nearside_prob < 0.2 and left_nearside_prob >= right_nearside_prob:
      road_edge_stat = Dir.RIGHT
    elif left_edge_prob > edge_threshold and left_nearside_prob < 0.2 and right_nearside_prob >= left_nearside_prob:
      road_edge_stat = Dir.LEFT
    else:
      return False
    return (rightBlinker and road_edge_stat == Dir.RIGHT) or (leftBlinker and road_edge_stat == Dir.LEFT)

  def update(self, carstate, lateral_active, lane_change_prob, md=None):
    current_time = time.monotonic()
    one_blinker = (leftBlinker := carstate.leftBlinker) != (rightBlinker := carstate.rightBlinker)
    below_lane_change_speed = carstate.vEgo < LANE_CHANGE_SPEED_MIN

    blindspot_detected = ((carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right) or
                          (carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left))

    blinker_dir_changed = ((leftBlinker and self.prev_blinker == Dir.RIGHT) or
                           (rightBlinker and self.prev_blinker == Dir.LEFT))

    nav_allowed = lateral_active and Params().get_bool("NavDesiresAllowed")
    nav_suppresses_alc = nav_allowed and navigation_suppresses_alc(carstate, lateral_active)
    ready_for_lane_change = lateral_active and self.is_alc_enabled and self.lane_change_timer <= LANE_CHANGE_TIME_MAX and not carstate.lkaDisabled and not nav_suppresses_alc

    if one_blinker and self.prev_blinker is None:
      self.blinker_below_lane_change_speed = below_lane_change_speed # Check if blinker was on below lane change speed
    elif not one_blinker:
      self.blinker_below_lane_change_speed = False

    if not ready_for_lane_change:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none

    # If blinker off/blinker direction change during Assisted Lane Change, finish the lane change.
    elif self.lane_change_state == LaneChangeState.laneChangeStarting and (not one_blinker or blinker_dir_changed):
      # fade out over .5s
      self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

      # If not 75% certainty, move back to the original lane.
      if not (lane_change_prob < 0.25 and self.lane_change_ll_prob < 0.01):
        pass # Disable direction change temporarily because of KA2 strong steering pulling
        # self.lane_change_direction = (LaneChangeDirection.left if self.lane_change_direction == LaneChangeDirection.right
        #                               else LaneChangeDirection.right)
      self.lane_change_state = LaneChangeState.laneChangeFinishing
      self.last_alc_cancel = current_time

    else:
      can_start_lane_change = (one_blinker and not below_lane_change_speed and (current_time - self.last_alc_cancel >= ALC_CANCEL_DELAY)
                               and not self.is_road_edge_blinker(md, rightBlinker, leftBlinker))

      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and can_start_lane_change and not self.blinker_below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         (((steering_torque := carstate.steeringTorque) > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (steering_torque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        if not can_start_lane_change:
          self.lane_change_state = LaneChangeState.off
        elif torque_applied and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_state = LaneChangeState.preLaneChange if can_start_lane_change else LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_blinker = None if not one_blinker else (Dir.LEFT if leftBlinker else Dir.RIGHT)
    base_desire = DESIRES[self.lane_change_direction][self.lane_change_state]
    self.desire = base_desire
    nav_offered = log.Desire.none
    if nav_allowed:
      nav_offered = navigation_desire(carstate, lateral_active)
      if nav_offered != log.Desire.none:
        # Nav wins over ALC when both apply (latActive required; ALC may stay enabled).
        self.desire = nav_offered
        if self.lane_change_state != LaneChangeState.off:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0

    # ALC keep pulse only — do not suppress nav keep desires.
    keep_pulse_suppressed = False
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire == base_desire and self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        keep_pulse_suppressed = True
        self.desire = log.Desire.none

    final_name = _desire_name(self.desire)
    offered_name = _desire_name(nav_offered)
    base_name = _desire_name(base_desire)
    nav_took_over = offered_name != "none" and final_name == offered_name
    if nav_took_over:
      source = "nav"
    elif offered_name != "none":
      source = "nav_blocked_safety"
    elif final_name == base_name and base_name != "none":
      source = "alc"
    elif keep_pulse_suppressed and offered_name != "none":
      source = "nav_blocked_keep_pulse"
    else:
      source = "none"
    log_key = (
      final_name, offered_name, base_name, source,
      int(nav_took_over), int(one_blinker), self.lane_change_state,
      int(keep_pulse_suppressed),
    )
    if log_key != self._last_final_desire_log_key and (
      final_name != "none" or offered_name != "none" or source.startswith("nav_blocked")
    ):
      cloudlog.warning(
        f"desire_final={final_name} source={source} nav_offered={offered_name} "
        f"base={base_name} alc_state={self.lane_change_state} "
        f"nav_took_over={int(nav_took_over)} blinker={int(one_blinker)} "
        f"lateral={int(lateral_active)} keep_pulse_suppressed={int(keep_pulse_suppressed)}"
      )
      self._last_final_desire_log_key = log_key

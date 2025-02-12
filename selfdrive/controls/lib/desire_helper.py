from cereal import log
from common.realtime import DT_MDL
from selfdrive.config import Conversions as CV
from common.params import Params
from enum import Enum, auto
import time

LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 30 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
ALC_CANCEL_DELAY = 1.75 # In seconds

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeRight,
  },
}

class Dir(Enum):
  LEFT = auto()
  RIGHT = auto()

class DesireHelper:
  def is_road_edge_blinker(self, carstate, md):
    if md is None: return False

    # road_edge_stat calculation adapted from
    # https://github.com/kisapilot/openpilot/blob/93c8046/selfdrive/controls/lib/desire_helper.py

    # WARNING: No threshold can determine all road edges correctly. Driver check is still required.
    edge_threshold = 0.475  # Higher value filters out low-confidence road edges
    left_edge_prob = max(0.0, min(1.0 - md.roadEdgeStds[0], 1.0))
    right_edge_prob = max(0.0, min(1.0 - md.roadEdgeStds[1], 1.0))
    left_nearside_prob, right_nearside_prob = md.laneLineProbs[0], md.laneLineProbs[3]

    if right_edge_prob > edge_threshold and right_nearside_prob < 0.2 and left_nearside_prob >= right_nearside_prob:
      road_edge_stat = Dir.RIGHT
    elif left_edge_prob > edge_threshold and left_nearside_prob < 0.2 and right_nearside_prob >= left_nearside_prob:
      road_edge_stat = Dir.LEFT
    else:
      return False
    return (carstate.leftBlinker and road_edge_stat == Dir.LEFT) or (carstate.rightBlinker and road_edge_stat == Dir.RIGHT)

  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.last_alc_cancel = 0
    self.last_blinker_on = 0
    self.prev_blinker = None # Handle direction change
    self.desire = log.LateralPlan.Desire.none
    self.is_alc_enabled = Params().get_bool("IsAlcEnabled")
    self.blinker_below_lane_change_speed = False

  def update(self, carstate, active, lane_change_prob, md=None):
    current_time = time.monotonic()
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = carstate.vEgo < LANE_CHANGE_SPEED_MIN

    blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

    blinker_dir_changed = ((self.prev_blinker == Dir.LEFT and carstate.rightBlinker) or
                           (self.prev_blinker == Dir.RIGHT and carstate.leftBlinker))

    ready_for_lane_change = active and self.is_alc_enabled and not carstate.lkaDisabled and self.lane_change_timer <= LANE_CHANGE_TIME_MAX

    if one_blinker and not self.prev_blinker != None:
      self.last_blinker_on = current_time # Record time of the last blinker on
      self.blinker_below_lane_change_speed = below_lane_change_speed # Check if blinker was on below lane change speed
    elif not one_blinker:
      self.blinker_below_lane_change_speed = False

    if not ready_for_lane_change:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none

    # If blinker off/blinker direction change or blindspot detected during Assisted Lane Change, finish the lane change.
    elif self.lane_change_state == LaneChangeState.laneChangeStarting and (not one_blinker or blinker_dir_changed or blindspot_detected):
      # fade out over .5s
      self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

      # If not 75% certainty, move back to the original lane.
      if not (lane_change_prob < 0.25 and self.lane_change_ll_prob < 0.01):
        self.lane_change_direction = (LaneChangeDirection.left if self.lane_change_direction == LaneChangeDirection.right
                                      else LaneChangeDirection.right)
      self.lane_change_state = LaneChangeState.laneChangeFinishing
      self.last_alc_cancel = current_time

    else:
      can_start_lane_change = (one_blinker and not below_lane_change_speed and (current_time - self.last_alc_cancel >= ALC_CANCEL_DELAY)
                               and not self.is_road_edge_blinker(carstate, md))

      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and can_start_lane_change \
          and not self.blinker_below_lane_change_speed and round(current_time - self.last_blinker_on, 2) > 0.20:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if carstate.leftBlinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

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
          if can_start_lane_change:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_blinker = None if not one_blinker else (Dir.LEFT if carstate.leftBlinker else Dir.RIGHT)
    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.LateralPlan.Desire.keepLeft, log.LateralPlan.Desire.keepRight):
        self.desire = log.LateralPlan.Desire.none

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
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.last_alc_cancel = 0
    self.last_blinker_on = 0
    self.prev_one_blinker = False
    self.prev_blinker = None # Handle direction change
    self.desire = log.LateralPlan.Desire.none
    self.is_alc_enabled = Params().get_bool("IsAlcEnabled")
    self.blinker_below_lane_change_speed = False

  def update(self, carstate, active, lane_change_prob):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

    blinker_dir_changed = ((self.prev_blinker == Dir.LEFT and carstate.rightBlinker) or
                           (self.prev_blinker == Dir.RIGHT and carstate.leftBlinker))

    if one_blinker and not self.prev_one_blinker: # Record time of the last blinker on
      self.last_blinker_on = time.monotonic()
      # Check if blinker was on below lane change speed
      self.blinker_below_lane_change_speed = below_lane_change_speed
    elif not one_blinker:
      self.blinker_below_lane_change_speed = False

    # If ALC is disabled or LKA is disabled, do not start assisted lane change.
    if not active or self.lane_change_timer > LANE_CHANGE_TIME_MAX \
        or not self.is_alc_enabled or carstate.lkaDisabled:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none

    # If blinker off/blinker direction change or blindspot detected during Assisted Lane Change, finish the lane change.
    elif self.lane_change_state == LaneChangeState.laneChangeStarting and \
        ((not one_blinker) or blinker_dir_changed or blindspot_detected):
      # fade out over .5s
      self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

      # If not 75% certainty, move back to the original lane.
      if not (lane_change_prob < 0.25 and self.lane_change_ll_prob < 0.01):
        if self.lane_change_direction == LaneChangeDirection.right:
          self.lane_change_direction = LaneChangeDirection.left
        else:
          self.lane_change_direction = LaneChangeDirection.right
      self.lane_change_state = LaneChangeState.laneChangeFinishing
      self.last_alc_cancel = time.monotonic()

    else:
      wait_for_delay = time.monotonic() - self.last_alc_cancel < ALC_CANCEL_DELAY
      blinker_length_enough = round((time.monotonic() - self.last_blinker_on), 2) > 0.20

      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and blinker_length_enough and not below_lane_change_speed \
          and not wait_for_delay and not self.blinker_below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          carstate.leftBlinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        if not one_blinker or below_lane_change_speed:
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
          if one_blinker and not wait_for_delay:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    if not one_blinker:
      self.prev_blinker = None
    else:
      self.prev_blinker = Dir.LEFT if carstate.leftBlinker else Dir.RIGHT

    self.prev_one_blinker = one_blinker

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

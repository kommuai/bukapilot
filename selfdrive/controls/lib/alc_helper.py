from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from cereal import log
LaneChangeState = log.LaneChangeState

class ALCHelper:
  def __init__(self):
    self.is_alc_enabled = Params().get_bool("IsAlcEnabled")
    self.blinker_below_lane_change_speed = False  # Check if blinker was last on when below ALC speed or when lat not active
    self.prev_one_blinker = False
    self.alc_speed_below = False                  # If ALC was doing lane change when speed changed to below min speed
    self.prev_enough_lane_change_speed = False    # If the previous speed was enough for ALC
    self.blinker_has_lane_change = False          # If there was any ALC lane change while the blinker was on

  def update(self, carstate, one_blinker, lc_state, active):
    changing_lanes = lc_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
    below_lane_change_speed = carstate.vEgo < LANE_CHANGE_SPEED_MIN
    lat_active = active and not carstate.lkaDisabled

    # Check if blinker was on when below lane change speed for ALC or when lat not active
    if one_blinker:
      if changing_lanes: # If there is any ALC lane change while blinker is on
        self.blinker_has_lane_change = True
      if not self.prev_one_blinker:
        self.blinker_below_lane_change_speed = below_lane_change_speed or not lat_active
    else:
      self.blinker_below_lane_change_speed = False

    if not lat_active or not one_blinker: # If not lat active, reset check for lane change even if blinker is on
      self.blinker_has_lane_change = False
    self.prev_one_blinker = one_blinker

    # Check if there was any ALC lane change while blinker on/ALC was doing lane change, when speed changed to below min speed
    if below_lane_change_speed and self.prev_enough_lane_change_speed and (changing_lanes or self.blinker_has_lane_change):
      self.alc_speed_below = True
    elif not lat_active or (not one_blinker and lc_state == LaneChangeState.off):
      self.alc_speed_below = False
    self.prev_enough_lane_change_speed = not below_lane_change_speed

    # Check if ALC is active
    alc_active = ((lat_active and self.is_alc_enabled) and (self.alc_speed_below or
                  (not below_lane_change_speed and not self.blinker_below_lane_change_speed)))

    return alc_active

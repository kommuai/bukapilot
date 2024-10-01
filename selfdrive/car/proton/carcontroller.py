from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.proton.protoncan import create_can_steer_command, create_hud, create_lead_detect, send_buttons, create_acc_cmd
from selfdrive.car.proton.values import CAR, DBC
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from common.params import Params
import cereal.messaging as messaging

from common.features import Features

RES_INTERVAL = 300
RES_LEN = 2 # Press resume for 2 frames

def apply_proton_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):

  # limits due to driver torque
  driver_max_torque = LIMITS.STEER_MAX + driver_torque * 30
  driver_min_torque = -LIMITS.STEER_MAX + driver_torque * 30
  max_steer_allowed = max(min(LIMITS.STEER_MAX, driver_max_torque), 0)
  min_steer_allowed = min(max(-LIMITS.STEER_MAX, driver_min_torque), 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))

class CarControllerParams():
  def __init__(self, CP):

    self.STEER_MAX = CP.lateralParams.torqueV[0]
    # make sure Proton only has one max steer torque value
    assert(len(CP.lateralParams.torqueV) == 1)

    # for torque limit calculation
    self.STEER_DELTA_UP = 20                      # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = 30                    # torque decrease per refresh

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.disable_radar = Params().get_bool("DisableRadar")
    self.num_cruise_btn_sent = 0
    self.temp_lead_dist = 0       # The last lead distance before standstill
    self.last_res_press_frame = 0 # The frame where the last resume press was finished
    self.resume_counter = 0       # Counter for tracking the progress of a resume press

    f = Features()
    self.mads = f.has("StockAcc")

  def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, ldw, laneActive):
    can_sends = []
    lat_active = enabled and laneActive
    # tester present - w/ no response (keeps radar disabled)
    if CS.CP.openpilotLongitudinalControl and self.disable_radar:
      if (frame % 10) == 0:
        can_sends.append([0x7D0, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0])

    if frame <= 1000 and CS.out.cruiseState.available and self.num_cruise_btn_sent <= 5:
      self.num_cruise_btn_sent += 1
      can_sends.append(send_buttons(self.packer, frame % 16, True))

    # steer
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))

    if not lat_active and CS.stock_ldp: # Lane Departure Prevention
      steer_dir = -1 if CS.steer_dir else 1
      new_steer = CS.stock_ldp_cmd * steer_dir * 0.0002 # Reduce value because stock command was strong
      lat_active = True

    apply_steer = apply_proton_steer_torque_limits(new_steer, self.last_steer, 0, self.params)
    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)

    ts = frame * DT_CTRL
    self.is_alc_enabled = Params().get_bool("IsAlcEnabled")

    # CAN controlled lateral running at 50hz
    if (frame % 2) == 0:
      can_sends.append(create_can_steer_command(self.packer, apply_steer, lat_active, CS.hand_on_wheel_warning and CS.is_icc_on, (frame/2) % 16, CS.stock_lks_settings,  CS.stock_lks_settings2))

      #can_sends.append(create_hud(self.packer, apply_steer, enabled, ldw, rlane_visible, llane_visible))
      #can_sends.append(create_lead_detect(self.packer, lead_visible, enabled))
      #if CS.out.genericToggle:
      #  fake_enable = True
      #else:
      #  fake_enable = False
      #can_sends.append(create_acc_cmd(self.packer, actuators.accel, fake_enable, (frame/2) % 16))

    # For resume
    if CS.out.standstill and enabled and self.resume_counter == 0 and \
        frame > (self.last_res_press_frame + RES_INTERVAL) and CS.leadDistance > self.temp_lead_dist:
      # Only start a new resume if the last one was finished, with an interval
      self.resume_counter = 1 # Start a new resume press
    elif not enabled or (enabled and not CS.out.standstill):
      # cruise control not enabled or moving with cruise control, record the distance
      self.temp_lead_dist = CS.leadDistance

    if self.resume_counter > 0 and self.resume_counter <= RES_LEN and CS.out.standstill:
      if not self.mads or CS.acc_req: # Send resume press signal
          can_sends.append(send_buttons(self.packer, frame % 16, False))
      self.resume_counter += 1

    if self.resume_counter > RES_LEN or not CS.out.standstill:
       # If resume press is finished or car is moving
       self.last_res_press_frame = frame # Store the frame where last resume press was finished
       self.resume_counter = 0 # Reset resume counter

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX

    return new_actuators, can_sends

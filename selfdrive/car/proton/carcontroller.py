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
import time

RES_INTERVAL = 550
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
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.disable_radar = Params().get_bool("DisableRadar")
    self.num_cruise_btn_sent = 0
    self.temp_lead_dist = 0       # The last lead distance before standstill
    self.last_res_press_frame = 0 # The frame where the last resume press was finished
    self.resume_counter = 0       # Counter for tracking the progress of a resume press
    self.last_steer_disable = 0         # The time of last steer disable
    self.prev_steer_enabled = False

    f = Features()
    self.mads = f.has("StockAcc")

  def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, ldw):
    can_sends = []
    lat_active = enabled
    # tester present - w/ no response (keeps radar disabled)
    if CS.CP.openpilotLongitudinalControl and self.disable_radar:
      if (frame % 10) == 0:
        can_sends.append([0x7D0, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0])

    if frame <= 1000 and CS.out.cruiseState.available and self.num_cruise_btn_sent <= 5:
      self.num_cruise_btn_sent += 1
      can_sends.append(send_buttons(self.packer, frame % 16, True))

    # steer
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_proton_steer_torque_limits(new_steer, self.last_steer, 0, self.params)
    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)

    # Stock Lane Departure Prevention / Centering Control (LKS Auxiliary / Blue line)
    steer_enabled = enabled and not CS.out.lkaDisabled
    if not steer_enabled and self.prev_steer_enabled:
      self.last_steer_disable = time.monotonic()
    self.prev_steer_enabled = steer_enabled

    stock_cmd = CS.stock_ldp_cmd
    if not steer_enabled and stock_cmd > 0 and \
        not ((CS.out.leftBlinker and CS.stock_ldp_left) or (CS.out.rightBlinker and CS.stock_ldp_right)):
      apply_stock_dir = -1 if CS.steer_dir else 1

      # After disable, keep steering at 0 for the first 0.55 seconds, then increase from 0% to 100% over increase_duration.
      increase_duration = 0.5 # Duration in seconds for steering torque to increase from 0% to 100%
      disable_diff = time.monotonic() - self.last_steer_disable
      mul = max(0, min((disable_diff - 0.55) / increase_duration, 1))
      apply_steer = int(round(stock_cmd * apply_stock_dir * mul)) &~1 # Ensure LSB 0 for 11-bit cmd
      lat_active, self.steer_rate_limited = True, False

    # CAN controlled lateral running at 50hz
    if (frame % 2) == 0 and (CS.lks_audio is not None and CS.lks_tactile is not None): # Ensure LKS values are read
      can_sends.append(create_can_steer_command(self.packer, apply_steer, lat_active, \
      CS.hand_on_wheel_warning and CS.is_icc_on, CS.hand_on_wheel_warning_2 and CS.is_icc_on, \
      (frame/2) % 16, CS.lks_aux, CS.lks_audio, CS.lks_tactile, CS.lks_assist_mode, CS.lka_enable, CS.stock_ldw, steer_enabled))

      #can_sends.append(create_hud(self.packer, apply_steer, enabled, ldw, rlane_visible, llane_visible))
      #can_sends.append(create_lead_detect(self.packer, lead_visible, enabled))
      #if CS.out.genericToggle:
      #  fake_enable = True
      #else:
      #  fake_enable = False
      #can_sends.append(create_acc_cmd(self.packer, actuators.accel, fake_enable, (frame/2) % 16))

    # For resume
    if CS.out.standstill and enabled and self.resume_counter == 0 and \
        frame > (self.last_res_press_frame + RES_INTERVAL) and min(CS.leadDistance, 8) > self.temp_lead_dist:
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

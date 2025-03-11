from selfdrive.car.proton.protoncan import create_can_steer_command, send_buttons
from selfdrive.car.proton.values import DBC
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip
from common.params import Params
from common.features import Features
import time

RES_INTERVAL = 150
SNG_WAIT = 450
RES_LEN = 5

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
    self.last_steer_disable = 0   # The time of last steer disable
    self.prev_steer_enabled = False
    self.sng_next_press_frame = 0 # The frame where the next resume press is allowed
    self.resume_counter = 0       # Counter for tracking the progress of a resume press
    self.is_sng_check = False
    self.lead_valid = False
    self.prev_lead_dist = 0
    self.lead_moved = True

    f = Features()
    self.mads = f.has("StockAcc")
    self.always_lks_tactile = f.has("LKSTactile")
    # Set default warning type for LKS
    self.lks_tactile = True
    self.lks_audio = False

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

    stock_steer_cmd = CS.stock_ldp_cmd
    if not steer_enabled and stock_steer_cmd > 0 and \
       not ((CS.out.rightBlinker and CS.stock_ldp_right) or (CS.out.leftBlinker and CS.stock_ldp_left)):
      apply_stock_dir = -1 if CS.steer_dir else 1

      # After steer disable, keep steering at 0 for the first 0.55 seconds, then increase from 0% to 100% over 0.5 seconds.
      mul = clip((time.monotonic() - self.last_steer_disable - 0.55) / 0.5, 0, 1)
      apply_steer = int(round(stock_steer_cmd * apply_stock_dir * mul)) &~1 # Ensure LSB 0 for 11-bit cmd
      lat_active = True
      self.steer_rate_limited = False

    # CAN controlled lateral running at 50hz
    if frame % 2 == 0:
      raw_cnt = (frame // 2) % 16
      is_icc_on = CS.is_icc_on
      if is_icc_on is not None: # Ensure LKS values are read
        if not self.always_lks_tactile:
          self.lks_audio = CS.lks_audio
          self.lks_tactile = CS.lks_tactile

        can_sends.append(create_can_steer_command(self.packer, apply_steer, lat_active, \
        is_icc_on and CS.hand_on_wheel_warning, is_icc_on and CS.hand_on_wheel_warning_2, \
        raw_cnt, CS.lks_aux, self.lks_audio, self.lks_tactile, CS.lks_assist_mode, CS.lka_enable, CS.stock_ldw_steering, steer_enabled))

      #can_sends.append(create_hud(self.packer, apply_steer, enabled, ldw, rlane_visible, llane_visible))
      #can_sends.append(create_lead_detect(self.packer, lead_visible, enabled))
      #if CS.out.genericToggle:
      #  fake_enable = True
      #else:
      #  fake_enable = False
      #can_sends.append(create_acc_cmd(self.packer, actuators.accel, fake_enable, raw_cnt))

    # SNG auto resume
    auto_resume_allowed = enabled and CS.out.cruiseState.standstill

    if not auto_resume_allowed:
      self.is_sng_check = False
    else:
      self.lead_valid = self.lead_valid and lead_visible
      lead_dist = CS.leadDistance
      self.lead_moved = self.lead_valid and (self.lead_moved or lead_dist > max(1, self.prev_lead_dist))
      self.prev_lead_dist = lead_dist

      if not self.is_sng_check:
        # SNG auto resume check start
        self.is_sng_check = True
        self.lead_valid = True
        self.sng_next_press_frame = frame + SNG_WAIT
        self.resume_counter = 0
        self.lead_moved = False

      elif (CS.res_btn_pressed or CS.out.gasPressed) or self.resume_counter >= RES_LEN:
        # Manual press or auto resume finished
        self.sng_next_press_frame = max(self.sng_next_press_frame, frame + RES_INTERVAL)
        self.resume_counter = 0
        self.lead_moved = False

      elif self.lead_moved and frame > self.sng_next_press_frame:
        # Send resume press signal
        if not self.mads or CS.acc_req:
          can_sends.append(send_buttons(self.packer, frame % 16, False))
        self.resume_counter += 1

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX

    return new_actuators, can_sends

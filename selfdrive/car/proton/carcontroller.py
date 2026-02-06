from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.proton.protoncan import create_can_steer_command, send_buttons, create_acc_cmd
from openpilot.selfdrive.car.proton.values import DBC, CAR
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.common.features import Features
from time import monotonic

def apply_proton_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):

  # limits due to driver torque
  driver_offset = driver_torque * 30
  max_steer_allowed = clip(LIMITS.STEER_MAX + driver_offset, 0, LIMITS.STEER_MAX)
  min_steer_allowed = clip(-LIMITS.STEER_MAX + driver_offset, -LIMITS.STEER_MAX, 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return round(apply_torque)

class CarControllerParams():
  def __init__(self, CP):

    self.STEER_MAX = CP.lateralParams.torqueV[0]
    # make sure Proton only has one max steer torque value
    assert(len(CP.lateralParams.torqueV) == 1)

    # for torque limit calculation
    if CP.carFingerprint == CAR.X90:
      self.STEER_DELTA_UP = 4
      self.STEER_DELTA_DOWN = 8
    else:
      self.STEER_DELTA_UP = 15
      self.STEER_DELTA_DOWN = 35

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.params = CarControllerParams(self.CP)

    self.last_steer = 0
    self.steering_direction = False
    f = Features()
    self.always_lks_tactile = f.has("lks-tactile")
    self.openpilot_long = not f.has("stock-acc")

    self.prev_steer_enabled = False
    self.last_steer_disable = 0

    self.sng_next_press_frame = 0 # The frame where the next resume press is allowed
    self.resume_counter = 0       # Counter for tracking the progress of a resume press
    self.is_sng_check = False
    self.resume = False

  def update(self, CC, CS, now_nanos):
    can_sends = []

    lat_active = CC.latActive
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel
    accel_cmd = actuators.accel

    # steer
    new_steer = round(actuators.steer * self.params.STEER_MAX)
    apply_steer = apply_proton_steer_torque_limits(new_steer, self.last_steer, 0, self.params)

    if not (steer_enabled := CC.latActive) and self.prev_steer_enabled:
      self.last_steer_disable = monotonic()
    self.prev_steer_enabled = steer_enabled

    # Stock Lane Departure Prevention / Centering Control (LKS Auxiliary / Blue line)
    if not steer_enabled and (stock_steer := CS.stock_ldp_cmd) > 0 and \
       not ((CS.out.rightBlinker and CS.stock_ldp_right) or (CS.out.leftBlinker and CS.stock_ldp_left)):
      # Prevents sudden pull from LDP/ICC/LKA Centering after steer disable
      mul = clip((monotonic() - self.last_steer_disable - 0.55) / 0.5, 0, 1)
      apply_steer = round(stock_steer * (-1 if CS.stock_steer_dir else 1) * mul) &~1 # Ensure cmd LSB 0
      lat_active = True

    if (self.frame % 2) == 0:

      # stock lane departure settings
      ldw_steering = CS.stock_ldw_steering
      if self.always_lks_tactile:
        ldw_steering = ldw_steering or CS.has_audio_ldw
        lks_audio, lks_tactile = False, True
      else:
        lks_audio, lks_tactile = CS.lks_audio, CS.lks_tactile

      # standstill logic
      standstill_request = CS.out.standstill and CC.longActive

      # SNG
      if not (CS.cruise_standstill and CC.longActive):
        self.is_sng_check = False
      else:
        self.resume = CS.out.gasPressed or CS.res_btn_pressed
        if not self.is_sng_check:
          self.is_sng_check = True
          self.sng_next_press_frame = self.frame + 310
          self.resume_counter = 0

        elif self.resume or self.resume_counter >= 2:
          self.sng_next_press_frame = max(self.sng_next_press_frame, self.frame + 110)
          self.resume_counter = 0

        elif actuators.accel > 0 and self.frame > self.sng_next_press_frame:
          # to disengage from stock cruise standstill
          self.resume = True
          can_sends.append(send_buttons(self.packer, 0))
          self.resume_counter += 1

      # proton X90 steer values are even numbers if we don't want to edit the proton dbc
      is_x90 = self.CP.carFingerprint == CAR.X90
      steer_cmd = (round(apply_steer) * 2) if (is_x90 and CC.latActive) else apply_steer

      can_sends.append(create_can_steer_command(self.packer, steer_cmd, lat_active,
                       CS.hand_on_wheel_warning and CS.is_icc_on,
                       CS.hand_on_wheel_warning_2 and CS.is_icc_on,
                       CS.lks_aux, lks_audio, lks_tactile, CS.lks_assist_mode,
                       CS.lka_enable, ldw_steering, steer_enabled, is_x90))

      if self.openpilot_long:
        accel_cmd = accel_cmd * 15 if accel_cmd >= 0 else accel_cmd * 18
        if CS.out.gasPressed:
          accel_cmd = 0

        mult = interp(CS.out.vEgo, [0, 28.3], [1.0, 0.6])
        if CS.out.vEgo < 2.5:
          accel_cmd = (CS.stock_acc_cmd * mult + accel_cmd)/2
        else:
          accel_cmd = min(CS.stock_acc_cmd * mult, accel_cmd)

        can_sends.append(create_acc_cmd(self.packer, accel_cmd, CC.longActive, CS.out.gasPressed,
                                        standstill_request, self.resume))

      # to disengage from stock cruise standstill
      if (CC.enabled and CS.cruise_standstill and (self.frame % 20 == 0)):
        can_sends.append(send_buttons(self.packer, False))

    # cancel stock cruise if error at openpilot
    if pcm_cancel_cmd:
      can_sends.append(send_buttons(self.packer, 1))

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.accel = accel_cmd / 15 if accel_cmd >= 0 else accel_cmd / 18
    new_actuators.steer = apply_steer / self.params.STEER_MAX

    self.frame += 1
    return new_actuators, can_sends

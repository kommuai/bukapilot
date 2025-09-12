from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.proton.protoncan import create_can_steer_command, send_buttons, create_acc_cmd
from openpilot.selfdrive.car.proton.values import DBC
from openpilot.common.numpy_fast import clip

from openpilot.common.features import Features

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
    self.STEER_DELTA_UP = 15
    self.STEER_DELTA_DOWN = 35

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.params = CarControllerParams(self.CP)

    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False
    self.always_lks_tactile = Features().has("lks-tactile")

  def update(self, CC, CS, now_nanos):
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators
    lat_active = enabled
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # steer
    new_steer = round(actuators.steer * self.params.STEER_MAX)
    apply_steer = apply_proton_steer_torque_limits(new_steer, self.last_steer, 0, self.params)
    if not lat_active and (stock_steer_cmd := CS.stock_ldp_cmd) > 0 and \
       not ((CS.out.rightBlinker and CS.stock_ldp_right) or (CS.out.leftBlinker and CS.stock_ldp_left)):
      lat_active = True
      apply_steer = round(stock_steer_cmd * (-1 if CS.stock_steer_dir else 1))
      self.steer_rate_limited = False

    if (self.frame % 2) == 0:
      standstill_request = CS.out.standstill and CC.longActive and actuators.accel < -0.01

      ldw_steering = CS.stock_ldw_steering
      if self.always_lks_tactile:
        ldw_steering = ldw_steering or CS.has_audio_ldw
        lks_audio, lks_tactile = False, True
      else:
        lks_audio, lks_tactile = CS.lks_audio, CS.lks_tactile

      if CS.out.genericToggle:
        actuators.accel = 3

      can_sends.append(create_can_steer_command(self.packer, apply_steer, lat_active, \
                      CS.hand_on_wheel_warning and CS.is_icc_on, \
                      CS.is_icc_on and CS.hand_on_wheel_chime, \
                      CS.lks_aux, lks_audio, lks_tactile, CS.lks_assist_mode, \
                      CS.lka_enable, 0))
      can_sends.append(create_acc_cmd(self.packer, actuators.accel, CC.longActive, CS.out.gasPressed, standstill_request, CS.stock_acc_cmd, CS.out.vEgo))

    if pcm_cancel_cmd:
      can_sends.append(send_buttons(self.packer, 1))

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX

    self.frame += 1
    return new_actuators, can_sends

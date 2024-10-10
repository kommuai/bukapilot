from opendbc.can.packer import CANPacker

from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.proton.protoncan import create_can_steer_command, send_buttons, create_acc_cmd
from openpilot.selfdrive.car.proton.values import DBC
from openpilot.common.numpy_fast import clip

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

  def update(self, CC, CS, now_nanos):
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators
    #ldw = CC.hudControl.leftLaneDepart or CC.hudControl.rightLaneDepart

    lat_active = enabled

    # steer
    new_steer = round(actuators.steer * self.params.STEER_MAX)
    apply_steer = apply_proton_steer_torque_limits(new_steer, self.last_steer, 0, self.params)

    # CAN controlled lateral running at 50hz
    if (self.frame % 2) == 0:
      standstill_request = CS.out.standstill and CC.longActive and actuators.accel < -0.01
      can_sends.append(create_can_steer_command(self.packer, apply_steer, lat_active, \
                      CS.hand_on_wheel_warning and CS.is_icc_on, \
                      CS.is_icc_on and CS.hand_on_wheel_chime, \
                      CS.lks_aux, CS.lks_audio, CS.lks_tactile, CS.lks_assist_mode, \
                      CS.lka_enable))
      can_sends.append(create_acc_cmd(self.packer, actuators.accel, enabled, CS.out.gasPressed, standstill_request))

      #can_sends.append(create_hud(self.packer, apply_steer, enabled, ldw, CC.hudControl.rightLaneVisible, CC.hudControl.leftLaneVisible))
      #can_sends.append(create_lead_detect(self.packer, CC.hudControl.leadVisible, enabled))


    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX

    self.frame += 1
    return new_actuators, can_sends

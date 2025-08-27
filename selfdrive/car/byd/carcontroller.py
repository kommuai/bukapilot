import math
from opendbc.can.packer import CANPacker

from openpilot.selfdrive.car import apply_std_steer_angle_limits, AngleRateLimit
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.byd.bydcan import create_can_steer_command, send_buttons, create_lkas_hud, create_accel_command
from openpilot.selfdrive.car.byd.values import DBC
from openpilot.common.numpy_fast import clip

ECU_FAULT_ANGLE = 220 # degress
STEER_LOWPASS_HZ = 2

def lowpass_1pole(x, y_prev):
    """
    x:       current (raw) steer angle prediction [deg]
    y_prev:  previous filtered angle [deg]
    0.02:      timestep 50hz [s]
    STEER_LOWPASS_HZ: filter cutoff frequency [Hz] (lower = smoother)
    """
    if y_prev is None:
        return x
    alpha = math.exp(-2.0 * math.pi * STEER_LOWPASS_HZ * 0.02)
    return alpha * y_prev + (1.0 - alpha) * x


class CarControllerParams():
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[6., 4., 3.])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[8., 6., 4.])

  def __init__(self, CP):
    pass

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

    self.lka_active = False
    self.last_apply_angle = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators
    apply_angle = CS.out.steeringAngleDeg
    # lkas user activation, cannot tie to lka_on state because it may deactivate itself
    if CS.lka_on:
      self.lka_active = True
    if not CS.lka_on and CS.lkas_rdy_btn:
      self.lka_active = False

    lat_active = enabled and self.lka_active and not CS.out.standstill
      #and not CS.out.steeringPressed and abs(CS.out.steeringAngleDeg) < ECU_FAULT_ANGLE

    if (self.frame % 2) == 0:
      if lat_active:
        apply_angle = lowpass_1pole(actuators.steeringAngleDeg, self.last_apply_angle)
        apply_angle = apply_std_steer_angle_limits(apply_angle, \
          CS.out.steeringAngleDeg, CS.out.vEgo, CarControllerParams)

        # assumption why eps fault:
        # 1. steer rate too high
        # 2. met with resistance while steering
        # 3. applied steer too far away from current steeringAngleDeg
        apply_angle = clip(apply_angle, CS.out.steeringAngleDeg - 10, CS.out.steeringAngleDeg + 10)
        self.last_apply_angle = apply_angle

      can_sends.append(create_can_steer_command(self.packer, apply_angle, lat_active, CS.out.standstill, CS.lkas_healthy, CS.lkas_rdy_btn))
      can_sends.append(create_lkas_hud(self.packer, enabled, CS.lss_state, CS.lss_alert, CS.tsr, \
        CS.abh, CS.passthrough, CS.HMA, CS.pt2, CS.pt3, CS.pt4, CS.pt5, self.lka_active))

      if self.CP.openpilotLongitudinalControl:
        long_active = enabled and not CS.out.gasPressed
        brake_hold = CS.out.standstill and actuators.accel < 0

        if (CC.enabled and CS.out.standstill and actuators.accel > 0) or CS.out.genericToggle:
          can_sends.append(send_buttons(self.packer, 1))
          actuators.accel = 2
          long_active = True
          brake_hold = False

        can_sends.append(create_accel_command(self.packer, actuators.accel, long_active, brake_hold))

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends

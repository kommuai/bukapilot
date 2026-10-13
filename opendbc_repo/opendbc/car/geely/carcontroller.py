from cereal import car
from opendbc.can.packer import CANPacker

from opendbc.car.geely import geelycan
from opendbc.car.geely.values import (
  CarControllerParams,
  DBC,
  LKAS_STEP,
  geely_steering_deg_sign,
  lowpass_steer_cmd,
)
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.steer_sign = geely_steering_deg_sign(CP)
    self.last_apply_angle = None
    self.lkas_counter = 0
    self.lkas_counter_1bit_1 = 0
    self.lkas_counter_1bit_2 = 0

  def _compute_apply_angle(self, CS, actuators, lat_active):
    if not lat_active:
      return CS.out.steeringAngleDeg
    meas = CS.out.steeringAngleDeg
    prev = self.last_apply_angle if self.last_apply_angle is not None else meas
    filtered = lowpass_steer_cmd(actuators.steeringAngleDeg, self.last_apply_angle)
    self.last_apply_angle = apply_std_steer_angle_limits(
      filtered, prev, CS.out.vEgo, meas, True, CarControllerParams.ANGLE_LIMITS,
    )
    return self.last_apply_angle

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []
    lat_active = CC.latActive and not CS.out.standstill
    driver_over = CS.out.steeringPressed or CS.steer_related_intervention
    steer_req = lat_active and not driver_over
    apply_angle = CS.out.steeringAngleDeg

    if self.frame % LKAS_STEP == 0:
      if steer_req:
        apply_angle = self._compute_apply_angle(CS, CC.actuators, steer_req)
      can_sends.append(geelycan.create_lkas_command(
        self.packer, apply_angle, steer_req, CS.out.steeringAngleDeg,
        self.steer_sign, self.lkas_counter,
        self.lkas_counter_1bit_1, self.lkas_counter_1bit_2,
      ))
      self.lkas_counter = (self.lkas_counter + 1) % 16
      self.lkas_counter_1bit_1 ^= 1
      self.lkas_counter_1bit_2 ^= 1

    new_actuators = car.CarControl.Actuators.new_message()
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1
    return new_actuators, can_sends

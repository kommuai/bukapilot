from opendbc.can.packer import CANPacker

from opendbc.car import DT_CTRL
from opendbc.car.chery import cherycan
from opendbc.car.chery.values import (
  CarControllerParams,
  DBC,
  LANE_KEEP_STEP,
  LKAS_INFO_STEP,
  RESUME_BUTTON_INTERVAL_S,
  SPOOF_CYCLE_FRAMES,
  SPOOF_DURATION_FRAMES,
  chery_steering_deg_sign,
  lowpass_steer_cmd,
)
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.steer_sign = chery_steering_deg_sign(CP)
    self.last_apply_angle = None
    self.last_resume_button_frame = 0

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

    if self.frame % LANE_KEEP_STEP == 0:
      apply_angle = self._compute_apply_angle(CS, CC.actuators, steer_req)
      can_sends.append(cherycan.create_lane_keep_command(
        self.packer, apply_angle, steer_req, CS.out.steeringAngleDeg, self.steer_sign,
      ))

    if self.frame % LKAS_INFO_STEP == 0:
      can_sends.append(cherycan.create_lkas_info_torque_spoof(
        self.packer, lat_active, CS.out.steeringTorque,
        (self.frame % SPOOF_CYCLE_FRAMES) < SPOOF_DURATION_FRAMES,
        lkas_enable=steer_req, steer_related=CS.lkas_info_steer_related,
        apply_spoof_offset=not driver_over,
      ))

    if (not self.CP.openpilotLongitudinalControl and CC.cruiseControl.resume
        and not CS.out.brakePressed
        and (self.frame - self.last_resume_button_frame) * DT_CTRL > RESUME_BUTTON_INTERVAL_S):
      self.last_resume_button_frame = self.frame
      can_sends.append(cherycan.create_pcm_icc_toggle_press(self.packer, (CS.pcm_button_counter + 1) % 16))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1
    return new_actuators, can_sends

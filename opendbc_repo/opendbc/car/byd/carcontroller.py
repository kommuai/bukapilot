import math
import numpy as np

from opendbc.can.packer import CANPacker

from opendbc.car.byd.bydcan import (
  create_accel_command,
  create_can_steer_command,
  create_lkas_hud,
  create_steering_torque_spoof_camera,
  send_buttons,
)
from opendbc.car.byd.values import DBC, CAR, ACCEL_MULT, CANBUS
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

STEER_LOWPASS_HZ = 2
STEER_DT = 0.02
MAX_STEER_ANGLE_OFFSET_DEG = 10
# Degrees: warn on HUD when command hits angle safety envelope (meas offset or global max).
STEER_ANGLE_LIMIT_WARN_EPS_DEG = 0.08
LKA_COOLDOWN_MIN_FRAMES = 30
BUTTON_KEEPALIVE_FRAMES = 100
SPOOF_DURATION_FRAMES = 50
SPOOF_CYCLE_FRAMES = 150


def lowpass_1pole(x, y_prev):
  if y_prev is None:
    return x
  alpha = math.exp(-2.0 * math.pi * STEER_LOWPASS_HZ * STEER_DT)
  return alpha * y_prev + (1.0 - alpha) * x


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])

    self.lka_active = False
    self.last_apply_angle = 0
    self.accel_mult = ACCEL_MULT[CP.carFingerprint]
    self.lka_cooldown = 0
    self.prev_press = False
    self.lka_latched = False
    self.button_send_bus = CANBUS.cam_bus if CP.carFingerprint in (CAR.BYD_ATTO3, CAR.BYD_M6) else CANBUS.main_bus

  def _update_lka_latch_state(self, CS):
    if self.CP.carFingerprint in (CAR.BYD_M6, CAR.BYD_SEAL, CAR.BYD_SEALION7):
      rising_edge = CS.lkas_rdy_btn and not self.prev_press
      if rising_edge:
        if not self.lka_latched:
          self.lka_latched = True
        else:
          self.lka_latched = False
          self.lka_cooldown = 0
          self.lka_active = False
      self.prev_press = CS.lkas_rdy_btn

      if CS.out.brakePressed:
        self.lka_latched = False
        self.lka_cooldown = 0
        self.lka_active = False
      elif self.lka_latched:
        self.lka_active = True
        self.lka_cooldown += 1
    else:
      if CS.lka_on:
        self.lka_cooldown += 1
        self.lka_active = True
      if not CS.lka_on and CS.lkas_rdy_btn:
        self.lka_active = False
        self.lka_cooldown = 0

  def _compute_apply_angle(self, CS, actuators, lat_active):
    meas_deg = CS.out.steeringAngleDeg
    if not lat_active:
      return meas_deg, False

    limits = CarControllerParams.ANGLE_LIMITS
    after_lowpass = lowpass_1pole(actuators.steeringAngleDeg, self.last_apply_angle)
    after_std = apply_std_steer_angle_limits(
      after_lowpass,
      self.last_apply_angle,
      CS.out.vEgo,
      meas_deg,
      lat_active,
      limits,
    )
    lo = meas_deg - MAX_STEER_ANGLE_OFFSET_DEG
    hi = meas_deg + MAX_STEER_ANGLE_OFFSET_DEG
    apply_angle = float(np.clip(after_std, lo, hi))
    self.last_apply_angle = apply_angle

    eps = STEER_ANGLE_LIMIT_WARN_EPS_DEG
    meas_clip_limited = abs(after_std - apply_angle) > eps
    angle_max_limited = abs(apply_angle) >= limits.STEER_ANGLE_MAX - eps
    steer_angle_limited = meas_clip_limited or angle_max_limited
    return apply_angle, steer_angle_limited

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    self._update_lka_latch_state(CS)

    lat_active = (self.lka_cooldown > LKA_COOLDOWN_MIN_FRAMES) and enabled and self.lka_active and not CS.out.standstill

    apply_angle = CS.out.steeringAngleDeg
    hand_on_wheel_warning = False
    if (self.frame % 2) == 0:
      apply_angle, steer_angle_limited = self._compute_apply_angle(CS, actuators, lat_active)
      hand_on_wheel_warning = bool(lat_active and steer_angle_limited)
      can_sends.append(
        create_can_steer_command(
          self.packer,
          apply_angle,
          lat_active,
          CS.out.standstill,
          CS.lkas_healthy,
          CS.lkas_rdy_btn or CS.out.brakePressed,
        )
      )
      can_sends.append(
        create_lkas_hud(
          self.packer,
          lat_active,
          CS.lss_state,
          CS.lss_alert,
          CS.tsr,
          CS.HMA,
          CS.pt2,
          CS.pt3,
          CS.pt4,
          CS.pt5,
          CS.lkas_hud_status_passthrough,
          CS.lka_on,
          hand_on_wheel_warning,
        )
      )

      if self.CP.openpilotLongitudinalControl:
        long_active = CC.enabled and not CS.out.gasPressed
        brake_hold = CS.out.standstill and actuators.accel < 0
        can_sends.append(create_accel_command(self.packer, actuators.accel, long_active, self.accel_mult, brake_hold))
      else:
        if CS.out.genericToggle or (CS.out.standstill and CC.enabled and (self.frame % BUTTON_KEEPALIVE_FRAMES == 0)):
          can_sends.append(send_buttons(self.packer, 1, 0, self.button_send_bus))

    if self.CP.carFingerprint in (CAR.BYD_M6, CAR.BYD_SEAL, CAR.BYD_SEALION7):
      cycle_position = self.frame % SPOOF_CYCLE_FRAMES
      spoof_active = cycle_position < SPOOF_DURATION_FRAMES

      if (self.frame % 5) == 0:
        can_sends.append(
          create_steering_torque_spoof_camera(self.packer, lat_active, CS.out.steeringTorque, spoof_active)
        )

    if pcm_cancel_cmd:
      can_sends.append(send_buttons(self.packer, 0, 1, self.button_send_bus))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends

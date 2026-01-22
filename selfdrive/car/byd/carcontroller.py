import math
from opendbc.can.packer import CANPacker

from openpilot.selfdrive.car import apply_std_steer_angle_limits, AngleRateLimit
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.byd.bydcan import create_can_steer_command, send_buttons, create_lkas_hud, create_accel_command, create_steering_torque_spoof_camera
from openpilot.selfdrive.car.byd.values import DBC, CAR, ACCEL_MULT, CANBUS
from openpilot.common.numpy_fast import clip

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
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[6., 3., 1.])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[8., 7., 4.])

  def __init__(self, CP):
    pass

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

    self.lka_active = False
    self.last_apply_angle = 0
    self.accel_mult = ACCEL_MULT[CP.carFingerprint]
    self.lka_cooldown = 0
    self.prev_press = False
    self.lka_latched = False
    self.button_send_bus = CANBUS.cam_bus if (CP.carFingerprint in (CAR.ATTO3, CAR.M6)) else CANBUS.main_bus

  def update(self, CC, CS, now_nanos):
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators
    apply_angle = CS.out.steeringAngleDeg
    pcm_cancel_cmd = CC.cruiseControl.cancel

    if self.CP.carFingerprint in (CAR.M6, CAR.SEAL, CAR.SEALION7):
      rising_edge = CS.lkas_rdy_btn and not self.prev_press
      if rising_edge:
        if not self.lka_latched:
          # First rising edge: latch it
          self.lka_latched = True
        else:
          # Second rising edge: unlatch it
          self.lka_latched = False
          self.lka_cooldown = 0
          self.lka_active = False
      self.prev_press = CS.lkas_rdy_btn

      # Unlatch when brake is pressed
      if CS.out.brakePressed:
        self.lka_latched = False
        self.lka_cooldown = 0
        self.lka_active = False
      # When latched, activate LKA and increment cooldown
      elif self.lka_latched:
        self.lka_active = True
        self.lka_cooldown += 1
    else:
      # lkas user activation, cannot tie to lka_on state because it may deactivate itself
      if CS.lka_on:
        self.lka_cooldown += 1
        self.lka_active = True
      if not CS.lka_on and CS.lkas_rdy_btn:
        self.lka_active = False
        self.lka_cooldown = 0

    lat_active = (self.lka_cooldown > 30) and enabled and self.lka_active and not CS.out.standstill

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
      can_sends.append(create_can_steer_command(self.packer, apply_angle, lat_active, CS.out.standstill, CS.lkas_healthy, CS.lkas_rdy_btn or CS.out.brakePressed))
      can_sends.append(create_lkas_hud(self.packer, lat_active, CS.lss_state, CS.lss_alert, CS.tsr, \
        CS.abh, CS.passthrough, CS.HMA, CS.pt2, CS.pt3, CS.pt4, CS.pt5, CS.lka_on))

      if self.CP.openpilotLongitudinalControl:
        long_active = CC.enabled and not CS.out.gasPressed
        brake_hold = CS.out.standstill and actuators.accel < 0
        can_sends.append(create_accel_command(self.packer, actuators.accel, long_active, self.accel_mult, brake_hold))
      else:
        if CS.out.genericToggle or (CS.out.standstill and CC.enabled and (self.frame % 100 == 0)):
          can_sends.append(send_buttons(self.packer, 1, 0, self.button_send_bus))

    # Spoof steering torque to simulate hands on wheel
    if self.CP.carFingerprint in (CAR.M6, CAR.SEAL):
      # Time-based spoof: trigger every 3 seconds, sustain for 1 second
      # At 50 Hz: 1 second = 50 frames, 3 seconds = 150 frames
      SPOOF_DURATION_FRAMES = 50   # 1 second at 50 Hz
      SPOOF_CYCLE_FRAMES = 150     # 3 seconds at 50 Hz

      # Calculate position in 3-second cycle (0-149)
      cycle_position = self.frame % SPOOF_CYCLE_FRAMES
      spoof_active = cycle_position < SPOOF_DURATION_FRAMES

      if (self.frame % 5) == 0:
        can_sends.append(create_steering_torque_spoof_camera(self.packer, lat_active, CS.out.steeringTorque, spoof_active))

    if pcm_cancel_cmd:
      can_sends.append(send_buttons(self.packer, 0, 1, self.button_send_bus))

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends

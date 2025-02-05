from selfdrive.car.byd.bydcan import create_can_steer_command, send_buttons, create_lkas_hud
from selfdrive.car.byd.values import DBC
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip

RES_INTERVAL = 125
SNG_WAIT = 310
RES_LEN = 8

def apply_byd_steer_angle_limits(apply_angle, actual_angle, v_ego, LIMITS):
  # pick angle rate limits based on wind up/down
  steer_up = actual_angle * apply_angle >= 0. and abs(apply_angle) > abs(actual_angle)
  rate_limits = LIMITS.ANGLE_RATE_LIMIT_UP if steer_up else LIMITS.ANGLE_RATE_LIMIT_DOWN

  return clip(apply_angle, actual_angle - rate_limits, actual_angle + rate_limits)

class CarControllerParams():
  def __init__(self, CP):
    self.ANGLE_RATE_LIMIT_UP = 3       # maximum allow 150 degree per second, 100Hz loop means 1.5
    self.ANGLE_RATE_LIMIT_DOWN = 3

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.steer_rate_limited = False
    self.lka_active = False
    self.sng_next_press_frame = 0 # The frame where the next resume press is allowed
    self.resume_counter = 0       # Counter for tracking the progress of a resume press
    self.is_sng_check = False
    self.lead_valid = False

  def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, ldw):
    can_sends = []

    # steer
    apply_angle = apply_byd_steer_angle_limits(actuators.steeringAngleDeg, CS.out.steeringAngleDeg, CS.out.vEgo, self.params)
    self.steer_rate_limited = (abs(apply_angle - CS.out.steeringAngleDeg) > 2.5)

    # BYD CAN controlled lateral running at 50hz
    if (frame % 2) == 0:

      # logic to activate and deactivate lane keep, cannot tie to the lka_on state because it will occasionally deactivate itself
      if CS.lka_on:
        self.lka_active = True
      if not CS.lka_on and CS.lkas_rdy_btn:
        self.lka_active = False

      if CS.out.steeringTorqueEps > 15:
        apply_angle = CS.out.steeringAngleDeg

      lat_active = enabled and abs(CS.out.steeringAngleDeg) < 90 and self.lka_active and not CS.out.standstill # temporary hardcode 60 because if 90 degrees it will fault
      brake_hold = False
      can_sends.append(create_can_steer_command(self.packer, apply_angle, lat_active, CS.out.standstill, (frame/2) % 16))
#      can_sends.append(create_accel_command(self.packer, actuators.accel, enabled, brake_hold, (frame/2) % 16))
      can_sends.append(create_lkas_hud(self.packer, enabled, CS.lss_state, CS.lss_alert, CS.tsr, CS.abh, CS.passthrough, CS.HMA, CS.pt2, CS.pt3, CS.pt4, CS.pt5, self.lka_active, frame % 16))

    # SNG auto resume
    auto_resume_allowed = enabled and (CS.out.standstill or CS.out.cruiseState.standstill)

    if not auto_resume_allowed:
      self.is_sng_check = False
    else:
      self.lead_valid = self.lead_valid and lead_visible

      if not self.is_sng_check:
        # SNG auto resume check start
        self.is_sng_check = True
        self.lead_valid = True
        self.sng_next_press_frame = frame + SNG_WAIT
        self.resume_counter = 0

      elif (CS.res_btn_pressed or CS.out.gasPressed) or self.resume_counter >= RES_LEN:
        # Manual press or auto resume finished
        self.sng_next_press_frame = max(self.sng_next_press_frame, frame + RES_INTERVAL)
        self.resume_counter = 0

      elif self.lead_valid and frame > self.sng_next_press_frame:
        # Send resume press signal
        can_sends.append(send_buttons(self.packer, 1, (CS.counter_pcm_buttons + 1) % 16))
        self.resume_counter += 1

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    return new_actuators, can_sends

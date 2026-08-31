import numpy as np
from time import monotonic

from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_dist_to_meas_limits
from opendbc.car.proton.protoncan import create_acc_cmd, create_can_steer_command, send_buttons
from opendbc.car.proton.values import DBC, CAR, CarControllerParams

PROTON_DRIVER_TORQUE_FACTOR = 30
STEER_DISABLE_BLEND_DELAY_S = 0.55
STEER_DISABLE_BLEND_DURATION_S = 0.5
SNG_INITIAL_PRESS_DELAY_FRAMES = 310
SNG_REPEAT_PRESS_DELAY_FRAMES = 110
SNG_MAX_RESUME_PRESSES = 2
CANCEL_SPAM_INTERVAL_FRAMES = 15
CANCEL_SPAM_PRESS_COUNT = 2
ACCEL_POSITIVE_SCALE = 16
ACCEL_NEGATIVE_SCALE = 23

try:
  from openpilot.common.features import Features
except (ImportError, ModuleNotFoundError):
  class Features:
    def has(self, _):
      return False


def apply_proton_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):
  # Proton-specific driver torque envelope.
  driver_offset = driver_torque * PROTON_DRIVER_TORQUE_FACTOR
  max_steer_allowed = float(np.clip(LIMITS.STEER_MAX + driver_offset, 0, LIMITS.STEER_MAX))
  min_steer_allowed = float(np.clip(-LIMITS.STEER_MAX + driver_offset, -LIMITS.STEER_MAX, 0))
  apply_torque = float(np.clip(apply_torque, min_steer_allowed, max_steer_allowed))

  # Delegate common ramp limiting to shared helper while keeping Proton-specific driver bounds.
  apply_torque = apply_dist_to_meas_limits(
    apply_torque,
    apply_torque_last,
    0.0,
    LIMITS.STEER_DELTA_UP,
    LIMITS.STEER_DELTA_DOWN,
    LIMITS.STEER_MAX,
    LIMITS.STEER_MAX,
  )

  return round(apply_torque)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.params = CarControllerParams(self.CP)

    self.last_steer = 0
    features = Features()
    self.always_lks_tactile = features.has("lks-tactile")
    self.openpilot_long = not features.has("stock-acc")

    self.prev_steer_enabled = False
    self.last_steer_disable = 0.0

    self.sng_next_press_frame = 0 # The frame where the next resume press is allowed
    self.resume_counter = 0       # Counter for tracking the progress of a resume press
    self.is_sng_check = False

    self.cancel_press_cnt = 0
    self.last_cancel_press = 0

  def _compute_steer(self, CC, CS):
    new_steer = round(CC.actuators.torque * self.params.STEER_MAX)
    apply_steer = apply_proton_steer_torque_limits(new_steer, self.last_steer, 0, self.params)

    steer_enabled = CC.latActive
    if not steer_enabled and self.prev_steer_enabled:
      self.last_steer_disable = monotonic()
    self.prev_steer_enabled = steer_enabled

    # Stock Lane Departure Prevention / Centering Control (LKS Auxiliary / Blue line)
    lat_active = steer_enabled
    if (
      not steer_enabled
      and CS.stock_ldp_cmd > 0
      and not ((CS.out.rightBlinker and CS.stock_ldp_right) or (CS.out.leftBlinker and CS.stock_ldp_left))
    ):
      # Prevents sudden pull from LDP/ICC/LKA Centering after steer disable.
      blend = float(np.clip((monotonic() - self.last_steer_disable - STEER_DISABLE_BLEND_DELAY_S) / STEER_DISABLE_BLEND_DURATION_S, 0.0, 1.0))
      apply_steer = round(CS.stock_ldp_cmd * (-1 if CS.stock_steer_dir else 1) * blend) & ~1 # Ensure cmd LSB 0 for 11-bit cmd
      lat_active = True

    return apply_steer, lat_active, steer_enabled

  def _update_sng(self, CC, CS, can_sends):
    if not (CS.cruise_standstill and CC.longActive):
      self.is_sng_check = False
      return

    if not self.is_sng_check:
      self.is_sng_check = True
      self.sng_next_press_frame = self.frame + SNG_INITIAL_PRESS_DELAY_FRAMES
      self.resume_counter = 0
      return

    if CS.out.gasPressed or CS.res_btn_pressed or self.resume_counter >= SNG_MAX_RESUME_PRESSES:
      self.sng_next_press_frame = max(self.sng_next_press_frame, self.frame + SNG_REPEAT_PRESS_DELAY_FRAMES)
      self.resume_counter = 0
      return

    # Brake check added for resume because S70 can still increase speed when standstill if brake pressed.
    if CC.actuators.accel > 0 and self.frame > self.sng_next_press_frame and not CS.out.brakePressed:
      can_sends.append(send_buttons(self.packer, False))
      self.resume_counter += 1

  def _update_cancel_spam(self, CS, pcm_cancel_cmd, can_sends):
    if not pcm_cancel_cmd:
      self.cancel_press_cnt = 0
      self.last_cancel_press = 0
      return

    if self.frame > self.last_cancel_press + CANCEL_SPAM_INTERVAL_FRAMES and not (CS.out.brakePressed and not CS.cruise_standstill):
      can_sends.append(send_buttons(self.packer, True))
      self.cancel_press_cnt += 1
      if self.cancel_press_cnt == CANCEL_SPAM_PRESS_COUNT:
        self.cancel_press_cnt = 0
        self.last_cancel_press = self.frame

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel
    accel_cmd = actuators.accel

    apply_steer, lat_active, steer_enabled = self._compute_steer(CC, CS)

    if (self.frame % 2) == 0:
      # stock lane departure settings
      ldw_steering = CS.stock_ldw_steering
      if self.always_lks_tactile:
        ldw_steering = ldw_steering or CS.has_audio_ldw
        lks_audio, lks_tactile = False, True
      else:
        lks_audio, lks_tactile = CS.lks_audio, CS.lks_tactile

      self._update_sng(CC, CS, can_sends)

      is_x90 = self.CP.carFingerprint == CAR.PROTON_X90

      # TODO: Remove line below and test on X90 since stock LKA last bit is always 0 for any Proton car.
      steer_cmd = (round(apply_steer) * 2) if (is_x90 and CC.latActive) else apply_steer

      can_sends.append(
        create_can_steer_command(
          self.packer,
          steer_cmd,
          lat_active,
          CS.hand_on_wheel_warning and CS.is_icc_on,
          CS.hand_on_wheel_warning_2 and CS.is_icc_on,
          CS.lks_aux,
          lks_audio,
          lks_tactile,
          CS.lks_assist_mode,
          CS.lka_enable,
          ldw_steering,
          steer_enabled,
          is_x90,
        )
      )

      if self.openpilot_long:
        accel_cmd = accel_cmd * ACCEL_POSITIVE_SCALE if accel_cmd >= 0 else accel_cmd * ACCEL_NEGATIVE_SCALE

        can_sends.append(
          create_acc_cmd(
            self.packer,
            accel_cmd,
            CC.longActive,
            CS.out.gasPressed and CS.out.cruiseState.enabled,
            CS.out.standstill,
            CS.stock_acc_cmd_values,
          )
        )

    self._update_cancel_spam(CS, pcm_cancel_cmd, can_sends)

    self.last_steer = apply_steer
    new_actuators = actuators.as_builder()
    new_actuators.accel = accel_cmd / ACCEL_POSITIVE_SCALE if accel_cmd >= 0 else accel_cmd / ACCEL_NEGATIVE_SCALE
    new_actuators.torque = apply_steer / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_steer

    self.frame += 1
    return new_actuators, can_sends

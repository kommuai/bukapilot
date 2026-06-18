from cereal import car
from opendbc.can.packer import CANPacker

from opendbc.car import DT_CTRL
from opendbc.car.chery import cherycan
from opendbc.car.chery.values import (
  AUTORESUME_BURST_FRAMES,
  AUTORESUME_CYCLE_S,
  CarControllerParams,
  DBC,
  EPS_SPOOF_STEP,
  EPS_TAP_FRAMES,
  EPS_TAP_PERIOD_FRAMES,
  EPS_TAP_TORQUE,
  HUD_STEP,
  LANE_KEEP_STEP,
  LKAS_INFO_STEP,
  lowpass_steer_cmd,
)
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

ButtonType = car.CarState.ButtonEvent.Type


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.last_apply_angle: float | None = None
    self.prev_brake_pressed = False

    # ACC arming + auto-resume from standstill state.
    self.acc_armed = False
    self.autoresume_burst_left = 0
    self.autoresume_last_burst_frame = -10_000
    self.autoresume_burst_idx = 0  # alternates RES (even) / SET (odd)
    self.hud_counter = 0
    self.eps_spoof_counter = 0
    self.eps_spoof_armed = False  # latched on first cam-spoof frame to sync with PT counter
    self.eps_tap_active_for = 0   # frames remaining on the current HOW-suppression tap

  def _compute_apply_angle(self, CS, actuators, steer_req):
    if not steer_req:
      return CS.out.steeringAngleDeg
    meas = CS.out.steeringAngleDeg
    prev = self.last_apply_angle if self.last_apply_angle is not None else meas
    filtered = lowpass_steer_cmd(actuators.steeringAngleDeg, self.last_apply_angle)
    self.last_apply_angle = apply_std_steer_angle_limits(
      filtered, prev, CS.out.vEgo, meas, True, CarControllerParams.ANGLE_LIMITS,
    )
    return self.last_apply_angle

  def _update_acc_armed(self, CS):
    """Arm when ACC is actively engaged while moving; disarm on explicit driver intent."""
    if CS.out.cruiseState.enabled and not CS.out.standstill:
      self.acc_armed = True

    brake_edge = CS.out.brakePressed and not self.prev_brake_pressed
    icc_press = any(be.pressed and be.type == ButtonType.altButton2 for be in CS.out.buttonEvents)
    if brake_edge or icc_press:
      self.acc_armed = False
    self.prev_brake_pressed = CS.out.brakePressed

  def _auto_resume(self, CS, can_sends):
    """Alternate RES / SET bursts while standing still and armed.

    burst_idx persists across transient standstill exits; only a hard disarm
    (brake/ICC, via acc_armed) rolls the index back to RES.
    """
    hard_disarm = self.CP.openpilotLongitudinalControl or not self.acc_armed
    if hard_disarm or not CS.out.standstill or CS.out.brakePressed:
      self.autoresume_burst_left = 0
      if hard_disarm:
        self.autoresume_burst_idx = 0
      return

    elapsed = (self.frame - self.autoresume_last_burst_frame) * DT_CTRL
    if self.autoresume_burst_left == 0 and elapsed >= AUTORESUME_CYCLE_S:
      self.autoresume_burst_left = AUTORESUME_BURST_FRAMES
      self.autoresume_last_burst_frame = self.frame

    if self.autoresume_burst_left > 0 and self.frame % 2 == 0:
      button = "CRUISE_BUTTON" if self.autoresume_burst_idx % 2 else "RES_BUTTON"
      ctr = (CS.pcm_button_counter + 1) % 16
      can_sends.append(cherycan.create_pcm_button(self.packer, ctr, 0, button))
      can_sends.append(cherycan.create_pcm_button(self.packer, ctr, 2, button))
      self.autoresume_burst_left -= 1
      if self.autoresume_burst_left == 0:
        self.autoresume_burst_idx += 1

  def _cam_torque_spoof_active(self, CS) -> bool:
    """Feed bus-2 torque spoofs while parked or while cruise is on.

    Pre-arms the camera path at standstill so you can engage ACC/LKAS in the
    driveway and see HOW / LKAS-fault on the meter without another drive.
    """
    return CS.out.standstill or CS.out.cruiseState.enabled

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []

    cam_spoof = self._cam_torque_spoof_active(CS)

    lat_active = CC.latActive and not CS.out.standstill
    driver_over = CS.out.steeringPressed or CS.steer_related_intervention
    steer_req = lat_active and not driver_over

    apply_angle = CS.out.steeringAngleDeg
    if self.frame % LANE_KEEP_STEP == 0:
      apply_angle = self._compute_apply_angle(CS, CC.actuators, steer_req)
      can_sends.append(cherycan.create_lane_keep_command(
        self.packer, apply_angle, steer_req, CS.out.steeringAngleDeg,
      ))

    if self.frame % LKAS_INFO_STEP == 0:
      # Bus-2 mirror only when cruise is engaged. At standstill (cruise off) we let
      # panda forward the native LKAS_INFO PT->cam — injecting a stale spoof there
      # makes the cam see MAIN_TORQUE>0 with LKAS inactive, which the meter flags.
      can_sends.extend(cherycan.create_lkas_info_torque_spoof(
        self.packer, steer_req, steer_req,
        steer_related=CS.lkas_info_steer_related,
        apply_spoof_offset=not driver_over,
        inject_on_cam=CS.out.cruiseState.enabled,
      ))

    if self.frame % HUD_STEP == 0:
      can_sends.append(cherycan.create_hud_override(self.packer, CS.cam_hud, self.hud_counter))
      self.hud_counter = (self.hud_counter + 1) % 16

    # While cam_spoof is active, panda blocks native EPS PT->cam (see chery_fwd_hook).
    # Re-emit EPS on bus 2 byte-identical to stock: real STEERING_ANGLE + real
    # DRIVER_TORQUE + self-incrementing counter (synced once to PT). To suppress the
    # hands-on-wheel warning we overlay a brief "tap" override every few seconds while
    # LKAS is actively asking — mimics the natural light hand touch (T~12) the camera
    # uses to reset its HOW timer.
    if cam_spoof:
      if not self.eps_spoof_armed:
        self.eps_spoof_counter = CS.eps_counter
        self.eps_spoof_armed = True
        self.eps_tap_active_for = 0
      if self.frame % EPS_SPOOF_STEP == 0:
        self.eps_spoof_counter = (self.eps_spoof_counter + 1) % 16

        driver_torque = CS.eps_driver_torque
        if steer_req and not driver_over:
          if self.eps_tap_active_for > 0:
            driver_torque = EPS_TAP_TORQUE
            self.eps_tap_active_for -= 1
          elif self.frame % EPS_TAP_PERIOD_FRAMES == 0:
            self.eps_tap_active_for = EPS_TAP_FRAMES - 1
            driver_torque = EPS_TAP_TORQUE
        else:
          self.eps_tap_active_for = 0

        can_sends.append(cherycan.create_eps_passthrough(
          self.packer,
          steering_angle_deg=CS.eps_steering_angle,
          driver_torque=driver_torque,
          counter=self.eps_spoof_counter,
        ))
    else:
      self.eps_spoof_armed = False
      self.eps_tap_active_for = 0

    self._update_acc_armed(CS)
    self._auto_resume(CS, can_sends)

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1
    return new_actuators, can_sends

from bisect import bisect_left
from enum import IntEnum

import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import DT_CTRL, make_can_msg, rate_limit, structs
from opendbc.car.dnga.dngacan import (
  BRAKE_DECEL_CMD_MAX,
  create_accel_command,
  create_brake_command,
  create_can_steer_command,
  create_hud,
  dnga_buttons,
)
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.dnga.values import BRAKE_SCALE, DBC, SNG_CAR, CarControllerParams
from opendbc.car.interfaces import CarControllerBase

BRAKE_THRESHOLD = 0.01
BRAKE_MAG = [BRAKE_THRESHOLD, 0.32, 0.46, 0.61, 0.76, 0.90, 1.06, 1.21, 1.35, 1.51, 4.0]
PUMP_VALS = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
PUMP_RESET_INTERVAL = 1.5
PUMP_RESET_DURATION = 0.1
REENGAGE_SET_MINUS_DELTA_KPH = 25.0
STOCK_ACC_ENABLE_MIN_SPEED_MS = 10.0
STOCK_ACC_DISABLE_SPEED_MS = 8.3
STOCK_ACC_ADJUST_DELTA_MS = 0.3
STANDSTILL_BRAKE_ADD = 0.2

class BrakingStatus(IntEnum):
  STANDSTILL_INIT = 0
  BRAKE_HOLD = 1
  PUMP_RESET = 2


def standstill_brake(min_accel, ts_last, ts_now, prev_status):
  status = prev_status
  brake = min_accel
  dt = ts_now - ts_last

  if prev_status == BrakingStatus.PUMP_RESET and dt > PUMP_RESET_DURATION:
    status = BrakingStatus.BRAKE_HOLD
    ts_last = ts_now
  elif prev_status in (BrakingStatus.BRAKE_HOLD, BrakingStatus.STANDSTILL_INIT) and dt > PUMP_RESET_INTERVAL:
    status = BrakingStatus.PUMP_RESET
    ts_last = ts_now

  if status == BrakingStatus.PUMP_RESET:
    brake = 0.0

  return brake, status, ts_last


def psd_brake(apply_brake, last_pump):
  pump = PUMP_VALS[bisect_left(BRAKE_MAG, apply_brake)]
  pump = rate_limit(pump, last_pump, -0.1, 0.1)
  brake_req = int(apply_brake >= BRAKE_THRESHOLD)
  return pump, brake_req, pump


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])

    self.last_steer = 0
    self.last_pump = 0.0
    self.brake_scale = BRAKE_SCALE[CP.carFingerprint]

    self.prev_ts = 0.0
    self.standstill_status = BrakingStatus.STANDSTILL_INIT
    self.min_standstill_accel = 0.0

    self.stock_ldw = False
    self.using_stock_acc = False
    self.frame = 0

  def _get_desired_speed(self, CS, acceleration):
    accel_cmd = acceleration
    t_lookup = 0.45 + 0.08 * CS.out.vEgo
    a_tau = (accel_cmd * t_lookup) if CS.out.vEgo > 1.2 else 0.0
    desired_speed = CS.out.vEgo + a_tau
    return accel_cmd, desired_speed

  def _get_apply_brake(self, CS, acceleration):
    if CS.out.gasPressed or acceleration >= 0.0:
      apply_brake = 0.0
    else:
      speed_scale = float(np.interp(CS.out.vEgo, [0.0, 140.0 * CV.KPH_TO_MS], [1.0, 1.0 / 3.0]))
      base_brake = abs(acceleration * self.brake_scale * speed_scale)
      apply_brake = float(np.clip(base_brake, 0.0, BRAKE_DECEL_CMD_MAX))
    return apply_brake

  def _send_reengage_button(self, CS, can_sends):
    stock_set_speed_kph = float(CS.stock_acc_set_speed)
    current_speed_kph = float(CS.out.vEgoCluster) * 3.6
    # Large stock set-speed deltas can fail to latch on RES_PLUS, use SET_MINUS to re-engage first.
    if (stock_set_speed_kph - current_speed_kph) > REENGAGE_SET_MINUS_DELTA_KPH:
      can_sends.append(dnga_buttons(self.packer, 1, 0, 0))
    else:
      can_sends.append(dnga_buttons(self.packer, 0, 1, 0))

  def _update_stock_acc_state(self, enabled, CS, can_sends):
    if enabled and CS.out.vEgo > STOCK_ACC_ENABLE_MIN_SPEED_MS:
      if CS.stock_acc_engaged:
        self.using_stock_acc = True
      else:
        self._send_reengage_button(CS, can_sends)
    elif enabled:
      self._send_reengage_button(CS, can_sends)

    if CS.out.vEgo < STOCK_ACC_DISABLE_SPEED_MS:
      self.using_stock_acc = False

    if enabled and self.using_stock_acc:
      stock_set_speed_ms = CS.stock_acc_set_speed // 3.6
      if CS.out.cruiseState.speedCluster - stock_set_speed_ms > STOCK_ACC_ADJUST_DELTA_MS:
        can_sends.append(dnga_buttons(self.packer, 0, 1, 0))
      if stock_set_speed_ms - CS.out.cruiseState.speedCluster > STOCK_ACC_ADJUST_DELTA_MS:
        can_sends.append(dnga_buttons(self.packer, 1, 0, 0))

  def _update_standstill_state(self, enabled, CS, apply_brake, ts):
    if enabled and apply_brake > 0.0 and CS.out.standstill and CS.CP.carFingerprint not in SNG_CAR:
      if self.standstill_status == BrakingStatus.STANDSTILL_INIT:
        self.min_standstill_accel = apply_brake + STANDSTILL_BRAKE_ADD
      return standstill_brake(self.min_standstill_accel, self.prev_ts, ts, self.standstill_status)

    self.standstill_status = BrakingStatus.STANDSTILL_INIT
    self.prev_ts = ts
    return apply_brake, self.standstill_status, self.prev_ts

  def update(self, CC, CS, now_nanos):
    del now_nanos
    self.stock_longitudinal_contributing = False
    can_sends = []
    ts = self.frame * DT_CTRL

    enabled = CC.enabled
    lat_active = CC.latActive
    actuators = CC.actuators
    hud_control = CC.hudControl

    new_steer = int(round(actuators.torque * self.params.STEER_MAX))
    apply_steer = apply_driver_steer_torque_limits(
      new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params
    )

    stock_brake_mag = float(getattr(CS, "stock_brake_mag", 0.0))
    stock_brake_mag = np.clip(stock_brake_mag, 0.0, 0.8)
    stock_brake_contributing = (CS.out.vEgo > 0.25) and (stock_brake_mag > 1e-6)
    self.stock_longitudinal_contributing = stock_brake_contributing and False
    acceleration = float(actuators.accel) if CS.out.vEgo > 0.25 else float(actuators.accel)

    acceleration, desired_speed = self._get_desired_speed(CS, acceleration)
    apply_brake = self._get_apply_brake(CS, acceleration)

    if self.frame <= 1000:
      can_sends.append(make_can_msg(2015, b"\x01\x04\x00\x00\x00\x00\x00\x00", 0))

    if (self.frame % 2) == 0:
      self.stock_ldw = bool(CS.laneDepartWarning)
      if self.stock_ldw and not enabled:
        apply_steer = -CS.ldpSteerV

      steer_req = lat_active or self.stock_ldw
      steer_counter = (self.frame // 2) % 16
      can_sends.append(create_can_steer_command(self.packer, apply_steer, steer_req, steer_counter))

    if (self.frame % 5) == 0:
      self._update_stock_acc_state(enabled, CS, can_sends)

      if CC.cruiseControl.cancel:
        can_sends.append(dnga_buttons(self.packer, 0, 0, 1))

      apply_brake, self.standstill_status, self.prev_ts = self._update_standstill_state(enabled, CS, apply_brake, ts)
      pump, brake_req, self.last_pump = psd_brake(apply_brake, self.last_pump)

      can_sends.append(
        create_accel_command(
          self.packer,
          CS.out.cruiseState.speedCluster,
          CS.out.cruiseState.available,
          enabled,
          hud_control.leadVisible,
          desired_speed,
          apply_brake,
          pump,
          CS.distance_val,
        )
      )

      aeb = bool((not enabled) and CS.aebV)
      can_sends.append(create_brake_command(self.packer, enabled, brake_req, pump, apply_brake, aeb))
      can_sends.append(
        create_hud(
          self.packer,
          CS.out.cruiseState.available and CS.lkas_latch,
          enabled,
          hud_control.leftLaneVisible,
          hud_control.rightLaneVisible,
          self.stock_ldw,
          CS.out.stockFcw,
          CS.out.stockAeb,
          CS.frontDepartWarning,
          CS.stock_lkc_off,
          CS.stock_fcw_off,
        )
      )

    self.last_steer = apply_steer
    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_steer / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_steer
    new_actuators.accel = acceleration

    self.frame += 1
    return new_actuators, can_sends

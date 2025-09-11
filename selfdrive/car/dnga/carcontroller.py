from opendbc.can.packer import CANPacker

from openpilot.selfdrive.car import make_can_msg
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.dnga.dngacan import create_can_steer_command, create_accel_command, dnga_buttons,\
                                       create_brake_command, create_hud
from openpilot.selfdrive.car.dnga.values import CAR, DBC, BRAKE_SCALE, SNG_CAR
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL

from bisect import bisect_left

BRAKE_THRESHOLD = 0.01
BRAKE_MAG = [BRAKE_THRESHOLD,.32,.46,.61,.76,.90,1.06,1.21,1.35,1.51,4.0]
PUMP_VALS = [0, .1, .2, .3, .4, .5, .6, .7, .8, .9, 1.0]
PUMP_RESET_INTERVAL = 1.5
PUMP_RESET_DURATION = 0.1

BRAKE_M = 1.4

class BrakingStatus():
  STANDSTILL_INIT = 0
  BRAKE_HOLD = 1
  PUMP_RESET = 2

def apply_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, blinkerOn, LIMITS):

  # limits due to driver torque and lane change
  reduced_torque_mult = 10 if blinkerOn else 1.5
  driver_max_torque = 255 + driver_torque * reduced_torque_mult
  driver_min_torque = -255 - driver_torque * reduced_torque_mult
  max_steer_allowed = max(min(255, driver_max_torque), 0)
  min_steer_allowed = min(max(-255, driver_min_torque), 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))

# reset pump every PUMP_RESET_INTERVAL seconds for. Reset to zero for PUMP_RESET_DURATION
def standstill_brake(min_accel, ts_last, ts_now, prev_status):
  brake = min_accel
  status = prev_status

  dt = ts_now - ts_last
  if prev_status == BrakingStatus.PUMP_RESET and dt > PUMP_RESET_DURATION:
    status = BrakingStatus.BRAKE_HOLD
    ts_last = ts_now

  if prev_status == BrakingStatus.BRAKE_HOLD and dt > PUMP_RESET_INTERVAL:
    status = BrakingStatus.PUMP_RESET
    ts_last = ts_now

  if prev_status == BrakingStatus.STANDSTILL_INIT and dt > PUMP_RESET_INTERVAL:
    status = BrakingStatus.PUMP_RESET
    ts_last = ts_now

  if status == BrakingStatus.PUMP_RESET:
    brake = 0

  return brake, status, ts_last

def psd_brake(apply_brake, last_pump):
  # reversed engineered from Ativa stock braking
  # this is necessary for noiseless pump braking
  pump = PUMP_VALS[bisect_left(BRAKE_MAG, apply_brake)]

  # make sure the pump value decrease and increases within 0.1
  # to prevent brake bleeding.
  # TODO does it really prevent brake bleed?
  if abs(pump - last_pump) > 0.1:
    pump = last_pump + clip(pump - last_pump, -0.1, 0.1)
  last_pump = pump

  if apply_brake >= BRAKE_THRESHOLD:
    brake_req = 1
  else:
    brake_req = 0

  return pump, brake_req, last_pump

# 100hz rate_limit
def rate_limit_positive_speed(x, last):
  if x > last:
    return min(x, last + 0.006)
  else:
    return x

class CarControllerParams():
  def __init__(self, CP):

    self.STEER_BP = CP.lateralParams.torqueBP
    self.STEER_LIM_TORQ = CP.lateralParams.torqueV
    self.STEER_MAX = CP.lateralParams.torqueV[0]
    # make sure Proton only has one max steer torque value
    assert(len(CP.lateralParams.torqueV) == 1)

    # for torque limit calculation
    self.STEER_DELTA_UP = 10
    self.STEER_DELTA_DOWN = 30

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):

    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False
    self.brake_pressed = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.brake = 0
    self.brake_scale = BRAKE_SCALE[CP.carFingerprint]
    self.last_pump = 0

    # standstill globals
    self.prev_ts = 0.
    self.standstill_status = BrakingStatus.STANDSTILL_INIT
    self.min_standstill_accel = 0

    self.stockLdw = False
    self.frame = 0

    self.using_stock_acc = False
    self.fingerprint = CP.carFingerprint

    self.last_des_speed = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    enabled = CC.enabled
    actuators = CC.actuators
    lead_visible = CC.hudControl.leadVisible
    rlane_visible = CC.hudControl.rightLaneVisible
    llane_visible = CC.hudControl.leftLaneVisible
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # steer
    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)
    new_steer = int(round(actuators.steer * steer_max_interp))

    isBlinkerOn = CS.out.leftBlinker != CS.out.rightBlinker
    apply_steer = apply_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, isBlinkerOn, self.params)

    ts = self.frame * DT_CTRL

    # speed and brake, speed using simple kinematics v = u + at
    # because dnga is speed controlled, the PID for positive accel is done by the car
    # so we change the equation to v = u + ka and assume k include the time horizon of 1s
    k = 0.3 + 0.06 * CS.out.vEgo
    des_speed = CS.out.vEgo + actuators.accel * k

    apply_brake = 0 if (CS.out.gasPressed or actuators.accel >= 0) else clip(abs(actuators.accel / BRAKE_M), 0., 1.25)
    if self.fingerprint in (CAR.ALZA):
      apply_brake = max(CS.stock_brake_mag * 0.2, apply_brake * 0.8)
    else:
      apply_brake = max(CS.stock_brake_mag * 0.6, apply_brake)

    if apply_brake > 0:
      self.last_des_speed = CS.out.vEgo
    else:
      self.last_des_speed = des_speed

    # positive des_speed rate limit
    if (CS.out.vEgo > 8.33):
      des_speed = rate_limit_positive_speed(des_speed, self.last_des_speed)

    # reduce max brake when below 10kmh to reduce jerk. TODO: more elegant way to do this?
    if CS.out.vEgo < 2.8:
      apply_brake = clip(apply_brake, 0., 0.8)

    # always clear dtc for dnga for the first 10s
    if self.frame <= 1000:
      can_sends.append(make_can_msg(2015, b'\x01\x04\x00\x00\x00\x00\x00\x00', 0))

    if (self.frame % 2) == 0:
      # allow stock LDP passthrough
      self.stockLdw = CS.laneDepartWarning
      if self.stockLdw and not enabled:
        apply_steer = -CS.ldpSteerV

      steer_req = (enabled or self.stockLdw) and CS.lkas_latch and not CS.lkaDisabled
      can_sends.append(create_can_steer_command(self.packer, apply_steer, steer_req, (self.frame / 2) % 16))

    # CAN controlled longitudinal
    if (self.frame % 5) == 0:

      # check if need to revert to stock acc
      if enabled and CS.out.vEgo > 10: # 36kmh
        if CS.stock_acc_engaged:
          self.using_stock_acc = True
      else:
        if enabled:
          # spam engage until stock ACC engages
          can_sends.append(dnga_buttons(self.packer, 0, 1, 0))

      # check if need to revert to bukapilot acc
      if CS.out.vEgo < 8.3: # 30kmh
        self.using_stock_acc = False

      # set stock acc follow speed
      if enabled and self.using_stock_acc:
        if CS.out.cruiseState.speedCluster - (CS.stock_acc_set_speed // 3.6) > 0.3:
          can_sends.append(dnga_buttons(self.packer, 0, 1, 0))
        if (CS.stock_acc_set_speed // 3.6) - CS.out.cruiseState.speedCluster > 0.3:
          can_sends.append(dnga_buttons(self.packer, 1, 0, 0))

      # spam cancel if op cancel
      if pcm_cancel_cmd:
        can_sends.append(dnga_buttons(self.packer, 0, 0, 1))

      # standstill logic
      if enabled and apply_brake > 0 and CS.out.standstill and CS.CP.carFingerprint not in SNG_CAR:
        if self.standstill_status == BrakingStatus.STANDSTILL_INIT:
          self.min_standstill_accel = apply_brake + 0.2
        apply_brake, self.standstill_status, self.prev_ts = standstill_brake(self.min_standstill_accel, self.prev_ts, ts, self.standstill_status)
      else:
        self.standstill_status = BrakingStatus.STANDSTILL_INIT
        self.prev_ts = ts

      # PSD brake logic
      pump, brake_req, self.last_pump = psd_brake(apply_brake, self.last_pump)

      can_sends.append(create_accel_command(self.packer, CS.out.cruiseState.speedCluster,
                       CS.out.cruiseState.available, enabled, lead_visible,
                       des_speed, apply_brake, pump, CS.distance_val))

      # Let stock AEB kick in only when system not engaged
      aeb = not enabled and CS.aebV
      can_sends.append(create_brake_command(self.packer, enabled, brake_req, pump, apply_brake, aeb))
      can_sends.append(create_hud(self.packer, CS.out.cruiseState.available and CS.lkas_latch, enabled, llane_visible, rlane_visible, self.stockLdw, CS.out.stockFcw, CS.out.stockAeb, CS.frontDepartWarning, CS.stock_lkc_off, CS.stock_fcw_off))

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    # sometimes we let stock brake, brake takes precedence over gas
    new_actuators.accel = actuators.accel - apply_brake if actuators.accel > 0 else actuators.accel

    self.frame += 1
    return new_actuators, can_sends

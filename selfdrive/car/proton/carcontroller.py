from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.proton.protoncan import create_hud
from selfdrive.car.proton.values import CAR, DBC, BRAKE_SCALE, GAS_SCALE
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
import cereal.messaging as messaging

from bisect import bisect_left

def apply_acttr_steer_torque_limits(apply_torque, apply_torque_last, LIMITS):
  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))

def compute_gb(accel):
  gb = float(accel) / 4.0
  return clip(gb, 0.0, 1.0), clip(-gb, 0.0, 1.0)

class CarControllerParams():
  def __init__(self, CP):

    self.STEER_BP = CP.lateralParams.torqueBP
    self.STEER_LIM_TORQ = CP.lateralParams.torqueV

    # for torque limit calculation
    self.STEER_DELTA_UP = 20                      # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = 30                    # torque decrease per refresh

    self.GAS_MAX = 2600                             # KommuActuator dac gas value
    self.GAS_STEP = 2                               # how often we update the longitudinal cmd
    self.ADAS_STEP = 5                              # 100/5 approx ASA frequency of 20 hz

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.last_pump_start_ts = 0.
    self.pump_saturated = False
    self.steer_rate_limited = False
    self.steering_direction = False
    self.brake_pressed = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.brake = 0
    self.brake_scale = BRAKE_SCALE[CP.carFingerprint]
    self.gas_scale = GAS_SCALE[CP.carFingerprint]

  def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, ldw):
    can_sends = []

    # steer
    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)
    new_steer = int(round(actuators.steer * steer_max_interp))
    apply_steer = apply_acttr_steer_torque_limits(new_steer, self.last_steer, self.params)
    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)

    # gas, brake
    apply_gas, apply_brake = compute_gb(actuators.accel)
    apply_brake *= self.brake_scale
    if CS.out.gasPressed:
      apply_brake = 0
    apply_gas *= self.gas_scale

    ts = frame * DT_CTRL

    # CAN controlled lateral
    if (frame % 2) == 0:

      # allow stock LDP passthrough
      stockLdw = CS.out.stockAdas.laneDepartureHUD
      if stockLdw:
        apply_steer = -CS.out.stockAdas.ldpSteerV

      steer_req = enabled or stockLdw
      #can_sends.append(create_can_steer_command(self.packer, apply_steer, steer_req, (frame/2) % 15))

    # CAN controlled longitudinal
    #if (frame % 5) == 0 and CS.CP.openpilotLongitudinalControl:
      # PSD brake logic
      #pump, self.last_pump_start_ts, brake_req, self.pump_saturated = psd_brake(apply_brake, self.last_pump_start_ts, ts)

      #mult = CS.out.vEgo * (apply_gas - apply_brake)
      #des_speed = max(0, CS.out.vEgo + mult)

      #can_sends.append(create_accel_command(self.packer, CS.out.cruiseState.speedCluster,
      #                                            CS.out.cruiseState.available, enabled, lead_visible,
      #                                            des_speed, apply_brake, pump, CS.out.cruiseState.setDistance))
      #can_sends.append(create_brake_command(self.packer, enabled, brake_req, pump, apply_brake, CS.out.stockAeb, (frame/5) % 8))
      #can_sends.append(create_hud(self.packer, CS.out.cruiseState.available, enabled, llane_visible, rlane_visible, ldw, CS.out.stockFcw, CS.out.stockAeb, CS.out.stockAdas.frontDepartureHUD))

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    if CS.out.gasPressed:
      new_actuators.accel = 0.5
    new_actuators.steer = apply_steer / steer_max_interp

    return new_actuators, can_sends

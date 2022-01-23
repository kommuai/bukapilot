from cereal import car
from selfdrive.car import make_can_msg, apply_std_steer_torque_limits
from selfdrive.car.perodua.peroduacan import create_steer_command, perodua_create_gas_command, \
                                             perodua_aeb_brake, create_can_steer_command
from selfdrive.car.perodua.values import CAR, DBC, NOT_CAN_CONTROLLED
from selfdrive.controls.lib.lateral_planner import LANE_CHANGE_SPEED_MIN
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip, interp
import cereal.messaging as messaging

class CarControllerParams():
  def __init__(self):

    self.STEER_MAX = 830                   # KommuActuator dac steering value
    self.STEER_DELTA_UP = 10               # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = 30             # torque decrease per refresh
    self.STEER_DRIVER_ALLOWANCE = 1        # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = 1       # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1           # from dbc
    self.STEER_REDUCE_FACTOR = 1000        # how much to divide the steer when reducing fighting torque
    self.GAS_MAX = 2600                    # KommuActuator dac gas value
    self.GAS_STEP = 2                      # how often we update the longitudinal cmd
    self.BRAKE_ALERT_PERCENT = 20          # percentage of brake to sound stock AEB alert
    self.ADAS_STEP = 5                     # 100/5 approx ASA frequency of 20 hz

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False
    self.brake_pressed = True
    self.params = CarControllerParams()
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel):
    can_sends = []

    # generate steering command
    steer_max_interp = self.params.STEER_MAX

    # myvi has a more reactive EPS
    if CS.CP.carFingerprint == CAR.MYVI:
      steer_max_interp = interp(CA.out.vEgo, [11, 22], [390, self.params.STEER_MAX - 220])
    if CS.CP.carFingerprint == CAR.BEZZA:
      steer_max_interp = interp(CS.out.vEgo, [11, 22], [400, self.params.STEER_MAX - 200])
    if CS.CP.carFingerprint == CAR.ARUZ:
      steer_max_interp = interp(CS.out.vEgo, [11, 22], [380, self.params.STEER_MAX - 230])
    if CS.CP.carFingerprint == CAR.AXIA:
      steer_max_interp = interp(CS.out.vEgo, [11, 26], [490, self.params.STEER_MAX])

    new_steer = int(round(actuators.steer * steer_max_interp))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)
    self.steer_rate_limited = ( new_steer != apply_steer ) and (apply_steer != 0)

    reduce_fighting_torque = ((CS.out.vEgo < LANE_CHANGE_SPEED_MIN) and (CS.out.leftBlinker != CS.out.rightBlinker))
    if reduce_fighting_torque:
      apply_steer = apply_steer / self.params.STEER_REDUCE_FACTOR

    self.steering_direction = True if (apply_steer >= 0) else False

    if CS.CP.carFingerprint in NOT_CAN_CONTROLLED:
      can_sends.append(create_steer_command(self.packer, apply_steer, self.steering_direction, enabled, frame))
    else:
      can_sends.append(create_can_steer_command(self.packer, apply_steer, enabled, frame))

    self.last_steer = apply_steer

    # gas
    if (frame % self.params.GAS_STEP) == 0:
      idx = frame // self.params.GAS_STEP
      apply_gas = clip(actuators.gas, 0., 1.)
      apply_gas = abs(apply_gas * self.params.GAS_MAX)

      # interceptor gas
      if CS.CP.enableGasInterceptor:
        can_sends.append(perodua_create_gas_command(self.packer, apply_gas, enabled, idx))

    # brakes
    if (frame % self.params.ADAS_STEP) == 0:
      apply_brake = clip(actuators.brake, 0., 1.)

      # AEB alert for non-braking cars
      if CS.CP.carFingerprint in NOT_CAN_CONTROLLED and apply_brake > (self.params.BRAKE_ALERT_PERCENT / 100):
        if not self.brake_pressed:
          can_sends.append(perodua_aeb_brake(self.packer, apply_brake))
          self.brake_pressed = True
        else:
          self.brake_pressed = False
          can_sends.append(perodua_aeb_brake(self.packer, apply_brake))

    return can_sends

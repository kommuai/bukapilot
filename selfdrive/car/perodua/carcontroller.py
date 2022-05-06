from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.perodua.peroduacan import create_steer_command, perodua_create_gas_command, \
                                             perodua_aeb_warning, create_can_steer_command, \
                                             perodua_create_accel_command, \
                                             perodua_create_brake_command, perodua_create_hud
from selfdrive.car.perodua.values import ACC_CAR, CAR, DBC, NOT_CAN_CONTROLLED
from selfdrive.controls.lib.lateral_planner import LANE_CHANGE_SPEED_MIN
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
import cereal.messaging as messaging

def apply_acttr_steer_torque_limits(apply_torque, apply_torque_last, LIMITS):
  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))


class CarControllerParams():
  def __init__(self, CP):

    self.STEER_BP = CP.lateralParams.torqueBP
    self.STEER_LIM_TORQ = CP.lateralParams.torqueV

    # for torque limit calculation
    if CP.carFingerprint in NOT_CAN_CONTROLLED:
      self.STEER_DELTA_UP = 20                      # torque increase per refresh, 0.8s to max
      self.STEER_DELTA_DOWN = 30                    # torque decrease per refresh
    else:
      self.STEER_DELTA_UP = 10
      self.STEER_DELTA_DOWN = 30

    self.STEER_REDUCE_FACTOR = 1000                 # how much to divide the steer when reducing fighting torque
    self.GAS_MAX = 2600                             # KommuActuator dac gas value
    self.GAS_STEP = 2                               # how often we update the longitudinal cmd
    self.BRAKE_ALERT_PERCENT = 30                   # percentage of brake to sound stock AEB alert
    self.ADAS_STEP = 5                              # 100/5 approx ASA frequency of 20 hz

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False
    self.brake_pressed = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, v_target, ldw):
    can_sends = []

    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)
    new_steer = int(round(actuators.steer * steer_max_interp))
    apply_steer = apply_acttr_steer_torque_limits(new_steer, self.last_steer, self.params)

    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)
    apply_gas = clip(actuators.gas, 0., 1.)

    '''
    Perodua vehicles supported by Kommu includes vehicles that does not have stock LKAS and ACC.
    These vehicles are controlled by KommuActuator and is labelled as NOT_CAN_CONTROLLED.
    KommuActuator uses 4 DACs to control the gas and steer of the vehicle. All values reflects the
    value of 12 bit DAC.

    Vehicles that comes with Perodua Smart Drive (PSD), includes stock LKAS and ACC. Note that the
    ACC command is done by giving a set speed so the stock internal controller will obtain the speed.
    '''

    if CS.CP.carFingerprint not in NOT_CAN_CONTROLLED:
      # CAN controlled lateral
      if (frame % 2) == 0:
        can_sends.append(create_can_steer_command(self.packer, apply_steer, enabled, (frame/2) % 15))

      # CAN controlled longitudinal
      if (frame % 5) == 0:

        apply_brake = actuators.brake

        can_sends.append(perodua_create_accel_command(self.packer, CS.out.vEgo, CS.out.cruiseState.speed,
                                                      CS.out.cruiseState.available, enabled, lead_visible,
                                                      v_target, apply_brake, apply_gas))
        can_sends.append(perodua_create_brake_command(self.packer, enabled, apply_brake, (frame/5) % 8))
        can_sends.append(perodua_create_hud(self.packer, CS.out.cruiseState.available, enabled, llane_visible, rlane_visible, ldw))

    # KommuActuator controls
    else:
      # steer
      reduce_fighting_torque = ((CS.out.vEgo < LANE_CHANGE_SPEED_MIN) and (CS.out.leftBlinker != CS.out.rightBlinker))
      if reduce_fighting_torque:
        apply_steer = apply_steer / self.params.STEER_REDUCE_FACTOR
      self.steering_direction = True if (apply_steer >= 0) else False

      can_sends.append(create_steer_command(self.packer, apply_steer, self.steering_direction, enabled, frame))

      # gas
      if (frame % self.params.GAS_STEP) == 0:
        idx = frame // self.params.GAS_STEP
        apply_gas = clip(actuators.gas, 0., 1.)
        apply_gas = abs(apply_gas * self.params.GAS_MAX)

        if CS.CP.enableGasInterceptor:
          can_sends.append(perodua_create_gas_command(self.packer, apply_gas, enabled, idx))

      # brakes, AEB alert for non-braking cars
      if (frame % self.params.ADAS_STEP) == 0:
        apply_brake = clip(actuators.brake, 0., 1.)

        if apply_brake > (self.params.BRAKE_ALERT_PERCENT / 100):
          if not self.brake_pressed and lead_visible:
            can_sends.append(perodua_aeb_warning(self.packer))
            self.brake_pressed = True
        else:
          self.brake_pressed = False

    self.last_steer = apply_steer
    return can_sends

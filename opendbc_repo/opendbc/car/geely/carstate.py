from cereal import car
from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.interfaces import CarStateBase
from opendbc.car.geely.values import (
  BRAKE_THRESHOLD,
  CANBUS,
  DBC,
  PARSER_MSGS,
  STEER_RELATED_INTERVENTION_RAW_MIN,
  geely_steering_deg_sign,
)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.prev_angle = 0.0
    self.steer_related_intervention = False

  def _apply_defaults(self, ret):
    ret.doorOpen = False
    ret.leftBlinker = False
    ret.rightBlinker = False
    ret.genericToggle = False
    ret.gasPressed = False
    ret.gearShifter = car.CarState.GearShifter.drive
    ret.seatbeltUnlatched = False
    ret.espDisabled = False
    ret.stockAeb = False
    ret.stockFcw = False
    ret.personality = -1

  def _parse_motion(self, ret, cp):
    self.parse_wheel_speeds(
      ret, cp.vl["WHEELSPEED_2"]["WHEEL_FL"], cp.vl["WHEELSPEED_2"]["WHEEL_FR"],
      cp.vl["WHEELSPEED_1"]["WHEEL_BL"], cp.vl["WHEELSPEED_1"]["WHEEL_BR"],
    )
    ret.vEgoCluster = ret.vEgo
    sign = geely_steering_deg_sign(self.CP)
    ret.steeringAngleDeg = sign * float(cp.vl["EPS"]["STEERING_ANGLE"])
    ret.steeringTorque = float(cp.vl["EPS"]["EPS_TORQUE_SIGNED"])
    steer_dir = 1 if ret.steeringAngleDeg >= self.prev_angle else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorqueEps = ret.steeringTorque * steer_dir
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 10, 5)
    ret.brake = float(cp.vl["PEDALS"]["BRAKE_PRESSURE"])
    ret.brakePressed = ret.brake > BRAKE_THRESHOLD
    ret.standstill = ret.vEgoRaw < 0.01

  def _parse_cruise(self, ret, cp):
    cruise_state = int(cp.vl["LKAS"]["CRUISE_STATE"])
    ret.cruiseState.available = True
    ret.cruiseState.enabled = cruise_state == 7
    ret.cruiseState.speed = 0.0
    ret.cruiseState.speedCluster = 0.0
    ret.cruiseState.standstill = ret.standstill and ret.cruiseState.enabled
    ret.cruiseState.nonAdaptive = False

  def _parse_adas(self, cp):
    sr = int(cp.vl["STEER_RELATED"]["STEERING_RELATED"])
    self.steer_related_intervention = abs(sr) >= STEER_RELATED_INTERVENTION_RAW_MIN

  def update(self, can_parsers):
    cp = can_parsers[Bus.pt]
    ret = car.CarState.new_message()
    self._apply_defaults(ret)
    self._parse_motion(ret, cp)
    self._parse_cruise(ret, cp)
    self._parse_adas(cp)
    return ret

  @staticmethod
  def get_can_parser(CP):
    return CANParser(DBC[CP.carFingerprint]["pt"], PARSER_MSGS, CANBUS.main_bus)

  @staticmethod
  def get_can_parsers(CP):
    return {Bus.pt: CarState.get_can_parser(CP)}

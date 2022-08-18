from cereal import car
from collections import deque
from math import ceil
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.proton.values import DBC, CAR, HUD_MULTIPLIER
from time import time

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']
    self.set_distance_values = can_define.dv['PCM_BUTTONS']['SET_DISTANCE']
    self.is_cruise_latch = False

  def update(self, cp):
    ret = car.CarState.new_message()

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_B'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_B'],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.rr, ret.wheelSpeeds.rl, ret.wheelSpeeds.fr, ret.wheelSpeeds.fl])

    # unfiltered speed from CAN sensors
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    # safety checks to engage
    can_gear = int(cp.vl["TRANSMISSION"]['GEAR'])

    ret.doorOpen = any([cp.vl["DOOR_LEFT_SIDE"]['BACK_LEFT_DOOR'],
                     cp.vl["DOOR_LEFT_SIDE"]['FRONT_LEFT_DOOR'],
                     cp.vl["DOOR_RIGHT_SIDE"]['BACK_RIGHT_DOOR'],
                     cp.vl["DOOR_RIGHT_SIDE"]['FRONT_RIGHT_DOOR']])

    ret.seatbeltUnlatched = cp.vl["SEATBELTS"]['RIGHT_SIDE_SEATBELT_ACTIVE_LOW'] == 1
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # Todo: need?
    disengage = ret.doorOpen or ret.seatbeltUnlatched
    if disengage:
      self.is_cruise_latch = False

    # gas pedal
    ret.gas = cp.vl["GAS_PEDAL"]['APPS_1']
    ret.gasPressed = ret.gas > 0.01

    # brake pedal
    ret.brake = cp.vl["BRAKE"]['BRAKE_PRESSURE']
    ret.brakePressed = ret.brake >  0.01

    # steer
    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]['STEER_ANGLE']
    steer_dir = 1 if (ret.steeringAngleDeg >= 0) else -1
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE'] * steer_dir
    ret.steeringTorqueEps = cp.vl["STEERING_TORQUE"]['DRIVER_TORQUE'] * steer_dir
    ret.steeringPressed = bool(abs(ret.steeringTorqueEps) > 3)
    ret.steerWarning = False
    ret.steerError = False

    # Todo: get the real value
    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER

    # Todo: get the real value
    ret.stockAeb = False
    ret.stockFcw = False

    ret.cruiseState.available = cp.vl["ACC_CMD"]["CRUISE_ENABLE"] != 0
    distance_val = int(cp.vl["PCM_BUTTONS"]['SET_DISTANCE'])
    ret.cruiseState.setDistance = self.parse_set_distance(self.set_distance_values.get(distance_val, None))

    self.is_cruise_latch = cp.vl["PCM_BUTTONS"]["ACC_SET"] != 0

    if ret.brakePressed:
      self.is_cruise_latch = False

    # set speed in range of 30 - 130kmh only
    self.cruise_speed = int(cp.vl["PCM_BUTTONS"]['ACC_SET_SPEED']) * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = self.cruise_speed
    ret.cruiseState.speed = ret.cruiseState.speedCluster / HUD_MULTIPLIER
    ret.cruiseState.standstill = ret.standstill
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.enabled = self.is_cruise_latch
    if not ret.cruiseState.available:
      self.is_cruise_latch = False

    # button presses
    ret.leftBlinker = bool(cp.vl["LEFT_STALK"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["LEFT_STALK"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["LEFT_STALK"]["GENERIC_TOGGLE"])

    ret.brakeHoldActive = bool(cp.vl["PARKING_BRAKE"]["CAR_ON_HOLD"])
    ret.espDisabled = bool(cp.vl["PARKING_BRAKE"]["ESC_ON"]) != 1

    # blindspot sensors
    if self.CP.enableBsm:
      # used for lane change so its okay for the chime to work on both side.
      # Todo: to find
      ret.leftBlindspot = False
      ret.rightBlindspot = False
    else:
      ret.leftBlindspot = False
      ret.rightBlindspot = False

    return ret


  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("WHEELSPEED_F", "WHEEL_SPEED", 0.),
      ("WHEELSPEED_B", "WHEEL_SPEED", 0.),
      ("SET_DISTANCE", "PCM_BUTTONS", 0.),
      ("CAR_ON_HOLD", "PARKING_BRAKE", 0.),
      ("ACC_SET", "PCM_BUTTONS", 0.),
      ("ACC_SET_SPEED", "PCM_BUTTONS", 0.),
      ("GEAR", "TRANSMISSION", 0),
      ("APPS_1", "GAS_PEDAL", 0.),
      ("BRAKE_PRESSURE", "BRAKE", 0.),
      ("MAIN_TORQUE", "STEERING_TORQUE", 0),
      ("DRIVER_TORQUE", "STEERING_TORQUE", 0),
      ("STEER_ANGLE", "STEERING_MODULE", 0),
      ("CRUISE_ENABLE", "ACC_CMD", 0),
      ("ESC_ON", "PARKING_BRAKE", 0),
      ("LEFT_SIGNAL", "LEFT_STALK", 0),
      ("RIGHT_SIGNAL", "LEFT_STALK", 0),
      ("GENERIC_TOGGLE", "LEFT_STALK", 0),
      ("RIGHT_SIDE_SEATBELT_ACTIVE_LOW", "SEATBELTS", 0),
      ("BACK_LEFT_DOOR", "DOOR_LEFT_SIDE", 1),
      ("FRONT_LEFT_DOOR", "DOOR_LEFT_SIDE", 1),
      ("BACK_RIGHT_DOOR", "DOOR_RIGHT_SIDE", 1),
      ("FRONT_RIGHT_DOOR", "DOOR_RIGHT_SIDE", 1)
    ]
    checks = []

    # todo: make it such that enforce_checks=True
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0, enforce_checks=False)

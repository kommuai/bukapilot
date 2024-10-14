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

from common.features import Features

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']
    self.set_distance_values = can_define.dv['PCM_BUTTONS']['SET_DISTANCE']
    self.is_cruise_latch = False
    self.acc_req = False
    self.hand_on_wheel_warning = False
    self.is_icc_on = False
    self.prev_angle = 0

    f = Features()
    self.mads = f.has("StockAcc")

    self.stock_lks_settings = 0
    self.stock_lks_settings2 = 0
    self.stock_ldp = 0
    self.stock_ldp_cmd = 0
    self.steer_dir = 0

  def update(self, cp):
    ret = car.CarState.new_message()

    self.stock_lks_settings = cp.vl["ADAS_LKAS"]["STOCK_LKS_SETTINGS"]
    self.stock_ldp_cmd = cp.vl["ADAS_LKAS"]["STEER_CMD"]
    self.stock_lks_settings2 = cp.vl["ADAS_LKAS"]["SET_ME_1_1"]
    self.steer_dir = cp.vl["ADAS_LKAS"]["STEER_DIR"]
    self.stock_ldp = bool(cp.vl["LKAS"]["LANE_DEPARTURE_WARNING_RIGHT"]) or bool(cp.vl["LKAS"]["LANE_DEPARTURE_WARNING_LEFT"])
    self.leadDistance = cp.vl["ADAS_LEAD_DETECT"]['LEAD_DISTANCE']
    # If cruise mode is ICC, make bukapilot control steering so it won't disengage.
    ret.lkaDisabled = not (bool(cp.vl["ADAS_LKAS"]["LKS_ENABLE"]) or self.is_icc_on)

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
    ret.brakeHoldActive = bool(cp.vl["PARKING_BRAKE"]["CAR_ON_HOLD"])

    disengage = ret.doorOpen or ret.seatbeltUnlatched or ret.brakeHoldActive
    if disengage:
      self.is_cruise_latch = False

    # gas pedal
    ret.gas = cp.vl["GAS_PEDAL"]['APPS_1']
    ret.gasPressed = ret.gas > 0.01

    # brake pedal
    ret.brake = cp.vl["BRAKE"]['BRAKE_PRESSURE']
    if self.mads:
      ret.brakePressed = False
    else:
      ret.brakePressed = bool(cp.vl["PARKING_BRAKE"]["BRAKE_PRESSED"])

    # steer
    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]['STEER_ANGLE']
    steer_dir = 1 if (ret.steeringAngleDeg - self.prev_angle >= 0) else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE'] * steer_dir
    ret.steeringTorqueEps = cp.vl["STEERING_MODULE"]['STEER_RATE'] * steer_dir
    ret.steeringPressed = bool(abs(ret.steeringTorqueEps) > 5)
    ret.steerWarning = False
    ret.steerError = False
    self.hand_on_wheel_warning = bool(cp.vl["ADAS_LKAS"]["HAND_ON_WHEEL_WARNING"])
    self.is_icc_on = bool(cp.vl["PCM_BUTTONS"]["ICC_ON"])
    self.steeringTorqueCounter = cp.vl["STEERING_TORQUE"]["COUNTER"]

    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER

    # Todo: get the real value
    ret.stockAeb = False
    ret.stockFcw = bool(cp.vl["FCW"]["STOCK_FCW_TRIGGERED"])

    self.acc_req = bool(cp.vl["ACC_CMD"]["ACC_REQ"])
    ret.cruiseState.available = any([cp.vl["PCM_BUTTONS"]["ACC_ON_OFF_BUTTON"], cp.vl["PCM_BUTTONS"]["GAS_OVERRIDE"]])

    distance_val = int(cp.vl["PCM_BUTTONS"]['SET_DISTANCE'])
    ret.cruiseState.setDistance = self.parse_set_distance(self.set_distance_values.get(distance_val, None))

    # engage and disengage logic
    if self.mads:
      self.is_cruise_latch = ret.cruiseState.available
    else:
       if cp.vl["PCM_BUTTONS"]["ACC_SET"] == 0 and ret.brakePressed:
         self.is_cruise_latch = False

    if cp.vl["PCM_BUTTONS"]["ACC_SET"] != 0 and not ret.brakePressed:
      self.is_cruise_latch = True

    # set speed in range of 30 - 130kmh only
    self.cruise_speed = int(cp.vl["PCM_BUTTONS"]['ACC_SET_SPEED']) * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = self.cruise_speed
    ret.cruiseState.speed = ret.cruiseState.speedCluster / HUD_MULTIPLIER
    ret.cruiseState.standstill = bool(cp.vl["ACC_CMD"]["STANDSTILL2"])
    ret.cruiseState.nonAdaptive = False

    if not ret.cruiseState.available:
      self.is_cruise_latch = False

    if not self.mads:
      if ret.brakePressed or (not self.acc_req and not ret.cruiseState.standstill):
        self.is_cruise_latch = False

    ret.cruiseState.enabled = self.is_cruise_latch

    # button presses
    ret.leftBlinker = bool(cp.vl["LEFT_STALK"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["LEFT_STALK"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["LEFT_STALK"]["GENERIC_TOGGLE"])

    ret.espDisabled = bool(cp.vl["PARKING_BRAKE"]["ESC_ON"]) != 1

    # blindspot sensors
    if self.CP.enableBsm:
      # used for lane change so its okay for the chime to work on both side.
      ret.leftBlindspot = bool(cp.vl["BSM_ADAS"]["LEFT_APPROACH"]) or bool(cp.vl["BSM_ADAS"]["LEFT_APPROACH_WARNING"])
      ret.rightBlindspot = bool(cp.vl["BSM_ADAS"]["RIGHT_APPROACH"]) or bool(cp.vl["BSM_ADAS"]["RIGHT_APPROACH_WARNING"])
    return ret


  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("LEAD_DISTANCE", "ADAS_LEAD_DETECT", 0.),
      ("WHEELSPEED_F", "WHEEL_SPEED", 0.),
      ("WHEELSPEED_B", "WHEEL_SPEED", 0.),
      ("SET_DISTANCE", "PCM_BUTTONS", 0.),
      ("BRAKE_PRESSED", "PARKING_BRAKE", 0.),
      ("CAR_ON_HOLD", "PARKING_BRAKE", 0.),
      ("ACC_SET", "PCM_BUTTONS", 0.),
      ("ACC_SET_SPEED", "PCM_BUTTONS", 0.),
      ("ACC_ON_OFF_BUTTON", "PCM_BUTTONS", 0.),
      ("GAS_OVERRIDE", "PCM_BUTTONS", 0.),
      ("ICC_ON", "PCM_BUTTONS", 0.),
      ("GEAR", "TRANSMISSION", 0),
      ("APPS_1", "GAS_PEDAL", 0.),
      ("BRAKE_PRESSURE", "BRAKE", 0.),
      ("MAIN_TORQUE", "STEERING_TORQUE", 0),
      ("DRIVER_TORQUE", "STEERING_TORQUE", 0),
      ("STEER_ANGLE", "STEERING_MODULE", 0),
      ("STEER_RATE", "STEERING_MODULE", 0),
      ("ESC_ON", "PARKING_BRAKE", 0),
      ("LEFT_SIGNAL", "LEFT_STALK", 0),
      ("RIGHT_SIGNAL", "LEFT_STALK", 0),
      ("GENERIC_TOGGLE", "LEFT_STALK", 0),
      ("RIGHT_APPROACH", "BSM_ADAS", 0),
      ("RIGHT_APPROACH_WARNING", "BSM_ADAS", 0),
      ("LEFT_APPROACH", "BSM_ADAS", 0),
      ("LEFT_APPROACH_WARNING", "BSM_ADAS", 0),
      ("RIGHT_SIDE_SEATBELT_ACTIVE_LOW", "SEATBELTS", 0),
      ("BACK_LEFT_DOOR", "DOOR_LEFT_SIDE", 1),
      ("FRONT_LEFT_DOOR", "DOOR_LEFT_SIDE", 1),
      ("BACK_RIGHT_DOOR", "DOOR_RIGHT_SIDE", 1),
      ("FRONT_RIGHT_DOOR", "DOOR_RIGHT_SIDE", 1),
      ("STANDSTILL2", "ACC_CMD", 1),
      ("CRUISE_ENABLE", "ACC_CMD", 1),
      ("ACC_REQ", "ACC_CMD", 1),
      ("HAND_ON_WHEEL_WARNING", "ADAS_LKAS", 1),
      ("STOCK_LKS_SETTINGS", "ADAS_LKAS", 1),
      ("STEER_DIR", "ADAS_LKAS", 1),
      ("SET_ME_1_1", "ADAS_LKAS", 1),
      ("STEER_CMD", "ADAS_LKAS", 1),
      ("LANE_DEPARTURE_WARNING_RIGHT", "LKAS", 1),
      ("LANE_DEPARTURE_WARNING_LEFT", "LKAS", 1),
      ("STOCK_FCW_TRIGGERED", "FCW", 1),
      ("LKS_ENABLE", "ADAS_LKAS", 1),
      ("COUNTER", "STEERING_TORQUE", 0),
    ]
    checks = []

    # todo: make it such that enforce_checks=True
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0, enforce_checks=False)

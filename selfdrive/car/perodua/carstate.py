from cereal import car
from collections import deque
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.perodua.values import DBC, CAR

# todo: clean this part up
pedal_counter = 0
pedal_press_state = 0
PEDAL_COUNTER_THRES = 25
PEDAL_UPPER_TRIG_THRES = 0.125
PEDAL_NON_ZERO_THRES = 0.001

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']
    self.is_cruise_latch = False
    self.cruise_speed = 0
    self.cruise_speed_counter = 0
    self.acttrGas = 0

  def update(self, cp):
    ret = car.CarState.new_message()

    # there is a backwheel speed, but it will overflow to 0 when reach 60kmh
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = ret.wheelSpeeds.rr
    ret.wheelSpeeds.fr = ret.wheelSpeeds.rr
    ret.wheelSpeeds.fl = ret.wheelSpeeds.rr
    ret.vEgoRaw = mean([ret.wheelSpeeds.rr, ret.wheelSpeeds.rl, ret.wheelSpeeds.fr, ret.wheelSpeeds.fl])
    # unfiltered speed from CAN sensors
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    # gas pedal
    ret.gas = cp.vl["GAS_PEDAL_1"]['APPS_1']
    # todo: let gas pressed legit
    ret.gasPressed = ret.gas > 0.6
    self.acttrGas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS']) /1800          # KommuActuator gas, read when stock pedal is being intercepted

    # brake pedal
    ret.brake = cp.vl["BRAKE_PEDAL"]['BRAKE_PRESSURE']

    # perodua bezza has a lower resolution brake pressure sensor
    if self.CP.carFingerprint == CAR.PERODUA_BEZZA:
      ret.brakePressed = ret.brake > 1.2
    else:
      ret.brakePressed = ret.brake > 1e-5

    # steering wheel
    ret.steeringAngleDeg = cp.vl["STEERING_ANGLE_SENSOR"]['STEER_ANGLE']
    steer_dir = 1 if (ret.steeringAngleDeg >= 0) else -1
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE'] * steer_dir
    ret.steeringTorqueEps = ret.steeringTorque/1000

    if self.CP.carFingerprint == CAR.PERODUA_AXIA:
      ret.steeringPressed = bool(abs(ret.steeringTorque) > 15)
    else:
      ret.steeringPressed = bool(abs(ret.steeringTorque) > 150)

    ret.steerWarning = False                                                # since Perodua has no LKAS, make it always no warning
    ret.steerError = False                                                  # since Perodua has no LKAS, make it always no warning

    # todo: find this out and add it in
#    ret.stockAeb = cp.vl["FWD_CAM1"]['AEB_BRAKE'] != 0                     # is stock AEB giving a braking signal?
    ret.stockFcw = cp.vl["FWD_CAM3"]['AEB_ALARM'] != 0
#    ret.espDisabled = cp.vl["ESC_CONTROL"]['STATUS'] != 0                  # electronic stability control status

    # cruise state
    ret.cruiseState.available = True
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.speed = self.cruise_speed

    # increase cruise_speed using pedal when engage
    if self.is_cruise_latch:
      self.cruise_speed_counter += 1
      if self.cruise_speed_counter % 120 == 0 and self.acttrGas > 0.3:
        self.cruise_speed += (5 * CV.KPH_TO_MS)
        self.cruise_speed_counter = 0

    # latching cruiseState logic
    if self.check_pedal_engage(ret.gas, pedal_press_state):
      if not self.is_cruise_latch:
        self.cruise_speed = ret.vEgo + (5 * CV.KPH_TO_MS)
      self.is_cruise_latch = True
    if ret.brakePressed:
      self.is_cruise_latch = False

    ret.cruiseState.enabled = self.is_cruise_latch
    ret.cruiseState.standstill = ret.standstill

    # safety checks to engage
    can_gear = int(cp.vl["TRANSMISSION"]['GEAR'])
    ret.doorOpen = any([cp.vl["METER_CLUSTER"]['MAIN_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_FRONT_DOOR'],
                     cp.vl["METER_CLUSTER"]['RIGHT_BACK_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_BACK_DOOR']])

    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEAT_BELT_WARNING'] == 1
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # button presses
    ret.leftBlinker = bool(cp.vl["METER_CLUSTER"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["METER_CLUSTER"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"])

    # blindspot sensors
    ret.leftBlindspot = False
    ret.rightBlindspot = False

    return ret

  @staticmethod
  def check_pedal_engage(gas,state):
    ''' Pedal engage logic '''
    global pedal_counter
    global pedal_press_state
    if (state == 0):
      if (gas > PEDAL_UPPER_TRIG_THRES):
        pedal_counter += 1
        if (pedal_counter == PEDAL_COUNTER_THRES):
          pedal_counter = 0
          return False
      if (pedal_counter > 2 and gas <= PEDAL_NON_ZERO_THRES):
        pedal_press_state = 1
        pedal_counter = 0
      return False
    if (state == 1):
      pedal_counter += 1
      if (pedal_counter == PEDAL_COUNTER_THRES):
        pedal_counter = 0
        pedal_press_state = 0
        return False
      if (gas > PEDAL_UPPER_TRIG_THRES):
        pedal_press_state = 2
        pedal_counter = 0
      return False
    if (state == 2):
      pedal_counter += 1
      if (pedal_counter == PEDAL_COUNTER_THRES):
        pedal_counter = 0
        pedal_press_state = 0
        return False
      if (gas <= PEDAL_NON_ZERO_THRES):
        pedal_counter = 0
        pedal_press_state = 0
        return True
    return False


  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("WHEELSPEED_F", "WHEEL_SPEED", 0.),
      ("WHEELSPEED_B", "WHEEL_SPEED", 0.),
      ("GEAR", "TRANSMISSION", 0),
      ("APPS_1", "GAS_PEDAL_1", 0.),
      ("BRAKE_PRESSURE", "BRAKE_PEDAL", 0.),
      ("STEER_ANGLE", "STEERING_ANGLE_SENSOR", 0.),
      ("INTERCEPTOR_GAS", "GAS_SENSOR", 0),
      ("MAIN_TORQUE", "STEERING_TORQUE", 0),
      ("GENERIC_TOGGLE", "RIGHT_STALK", 0),
      ("FOG_LIGHT", "RIGHT_STALK", 0),
      ("LEFT_SIGNAL", "METER_CLUSTER", 0),
      ("RIGHT_SIGNAL", "METER_CLUSTER", 0),
      ("SEAT_BELT_WARNING", "METER_CLUSTER", 0),
      ("AEB_ALARM", "FWD_CAM3", 0),
      ("MAIN_DOOR", "METER_CLUSTER", 1),
      ("LEFT_FRONT_DOOR", "METER_CLUSTER", 1),
      ("RIGHT_BACK_DOOR", "METER_CLUSTER", 1),
      ("LEFT_BACK_DOOR", "METER_CLUSTER", 1),
    ]
    checks = []
    # todo: make it such that enforce_checks=True
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0, enforce_checks=False)

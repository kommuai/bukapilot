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
PEDAL_COUNTER_THRES = 35
PEDAL_UPPER_TRIG_THRES = 0.125
PEDAL_NON_ZERO_THRES = 0.01

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

    # safety checks to engage
    can_gear = int(cp.vl["TRANSMISSION"]['GEAR'])

    ret.doorOpen = any([cp.vl["METER_CLUSTER"]['MAIN_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_FRONT_DOOR'],
                     cp.vl["METER_CLUSTER"]['RIGHT_BACK_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_BACK_DOOR']])

    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEAT_BELT_WARNING'] == 1
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    disengage = ret.doorOpen or ret.seatbeltUnlatched
    if disengage:
      self.is_cruise_latch = False

    # gas pedal
    ret.gas = cp.vl["GAS_PEDAL"]['APPS_1']
    # todo: let gas pressed legit
    ret.gasPressed = ret.gas > 1.0
    self.acttrGas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS']) # KommuActuator gas, read when stock pedal is being intercepted
    if self.acttrGas < 0:
      self.acttrGas = 0

    # brake pedal
    ret.brake = cp.vl["BRAKE"]['BRAKE_PRESSURE']

    # perodua bezza has a lower resolution brake pressure sensor
    if self.CP.carFingerprint == CAR.BEZZA:
      ret.brakePressed = ret.brake > 1.2
    else:
      ret.brakePressed = ret.brake > 1e-5

    # steering wheel
    if self.CP.carFingerprint == CAR.ATIVA:
      ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]['STEER_ANGLE']
      ret.steeringTorque = cp.vl["STEERING_MODULE"]['MAIN_TORQUE']
      ret.steeringTorqueEps = cp.vl["EPS_SHAFT_TORQUE"]['STEERING_TORQUE']
    else:
      ret.steeringAngleDeg = cp.vl["STEERING_ANGLE_SENSOR"]['STEER_ANGLE']
      steer_dir = 1 if (ret.steeringAngleDeg >= 0) else -1
      ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE'] * steer_dir
      ret.steeringTorqueEps = ret.steeringTorque/1000

    if self.CP.carFingerprint == CAR.AXIA:
      ret.steeringPressed = bool(abs(ret.steeringTorque) > 18)
    elif self.CP.carFingerprint == CAR.ATIVA:
      ret.steeringPressed = bool(abs(ret.steeringTorque) > 25)
    else:
      ret.steeringPressed = bool(abs(ret.steeringTorque) > 70)

    ret.steerWarning = False      # since Perodua has no LKAS, make it always no warning
    ret.steerError = False        # since Perodua has no LKAS, make it always no warning

    # todo: find this out and add it in
#    ret.stockAeb = cp.vl["FWD_CAM1"]['AEB_BRAKE'] != 0                     # is stock AEB giving a braking signal?
    if self.CP.carFingerprint != CAR.ATIVA:
      ret.stockFcw = cp.vl["FWD_CAM3"]['AEB_ALARM'] != 0
#    ret.espDisabled = cp.vl["ESC_CONTROL"]['STATUS'] != 0                  # electronic stability control status

      # cruise state
      ret.cruiseState.available = True
      ret.cruiseState.nonAdaptive = False
      ret.cruiseState.speed = self.cruise_speed

      if self.is_cruise_latch:
        # pedal disengage
        if self.check_pedal_engage(self.acttrGas, pedal_press_state):
          self.is_cruise_latch = False

        # increase cruise_speed using pedal when engage
        self.cruise_speed_counter += 1
        if self.cruise_speed_counter % 100 == 0 and self.acttrGas > 0.2:
          self.cruise_speed += (5 * CV.KPH_TO_MS)
          self.cruise_speed_counter = 0

      # latching cruiseState logic
      if not self.is_cruise_latch:
        if self.check_pedal_engage(ret.gas, pedal_press_state):
          self.cruise_speed = ret.vEgo + (5 * CV.KPH_TO_MS)
          self.is_cruise_latch = True
      if ret.brakePressed:
        self.is_cruise_latch = False

      ret.cruiseState.enabled = self.is_cruise_latch
    else:
      ret.cruiseState.available = cp.vl["PCM_BUTTONS"]["ACC_RDY"] != 0
      ret.cruiseState.nonAdaptive = False
      ret.cruiseState.speed = 7
      if bool(cp.vl["PCM_BUTTONS"]["SET_MINUS"]):
        self.is_cruise_latch = True
      if ret.brakePressed:
        self.is_cruise_latch = False

      ret.cruiseState.enabled = self.is_cruise_latch

    ret.cruiseState.standstill = ret.standstill

    # button presses
    ret.leftBlinker = bool(cp.vl["METER_CLUSTER"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["METER_CLUSTER"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"])

    # blindspot sensors
    if self.CP.enableBsm:
      ret.leftBlindspot = False
      ret.rightBlindspot = bool(cp.vl["BSM"]["R_BLINDSPOT"])
    else:
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
      ("GEAR", "TRANSMISSION", 0),
      ("APPS_1", "GAS_PEDAL", 0.),
      ("BRAKE_PRESSURE", "BRAKE", 0.),
      ("INTERCEPTOR_GAS", "GAS_SENSOR", 0),
      ("GENERIC_TOGGLE", "RIGHT_STALK", 0),
      ("FOG_LIGHT", "RIGHT_STALK", 0),
      ("LEFT_SIGNAL", "METER_CLUSTER", 0),
      ("RIGHT_SIGNAL", "METER_CLUSTER", 0),
      ("SEAT_BELT_WARNING", "METER_CLUSTER", 0),
      ("MAIN_DOOR", "METER_CLUSTER", 1),
    ]
    checks = []
    
    if CP.carFingerprint == CAR.ATIVA:
      signals.append(("R_BLINDSPOT","BSM", 0))
      signals.append(("STEER_ANGLE", "STEERING_MODULE", 0.))
      signals.append(("MAIN_TORQUE", "STEERING_MODULE", 0.))
      signals.append(("STEERING_TORQUE", "EPS_SHAFT_TORQUE", 0.))
      signals.append(("ACC_RDY", "PCM_BUTTONS", 0))
      signals.append(("SET_MINUS", "PCM_BUTTONS", 0))
    else:
      signals.append(("MAIN_TORQUE", "STEERING_TORQUE", 0))
      signals.append(("STEER_ANGLE", "STEERING_ANGLE_SENSOR", 0.))
      signals.append(("AEB_ALARM", "FWD_CAM3", 0))
      signals.append(("WHEELSPEED_B", "WHEEL_SPEED", 0.))
      signals.append(("LEFT_FRONT_DOOR", "METER_CLUSTER", 1))
      signals.append(("RIGHT_BACK_DOOR", "METER_CLUSTER", 1))
      signals.append(("LEFT_BACK_DOOR", "METER_CLUSTER", 1))
    

    # todo: make it such that enforce_checks=True
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0, enforce_checks=False)

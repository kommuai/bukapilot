from cereal import car
from collections import deque
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.perodua.values import DBC

pedal_counter = 0
pedal_press_state = 0
PEDAL_COUNTER_THRES = 25
PEDAL_NON_ZERO_THRES = 0.01

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']
    self.is_cruise_latch = False
    #self.steeringTorqueSamples = deque(TORQUE_SAMPLES*[0], TORQUE_SAMPLES)
    #self.current_steering_thres = MIN_STEER_THRESHOLD

  def update(self, cp):
    ret = car.CarState.new_message()
    # there is a backwheel speed, but it will overflow to 0 when reach 60kmh
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.rr, ret.wheelSpeeds.rl, ret.wheelSpeeds.fr, ret.wheelSpeeds.fl])
    # unfiltered speed from CAN sensors
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    # gas pedal
    ret.gas = cp.vl["GAS_PEDAL_1"]['APPS_1']                                              # gas pedal, 0.0-1.0
    ret.gasPressed = ret.gas > 0.60

    # brake pedal
    ret.brake = cp.vl["BRAKE_PEDAL"]['BRAKE_PRESSURE']                                    # Use for pedal
    ret.brakePressed = ret.brake > 1e-5                                                   # Use for pedal
    ret.brakeLights = ret.brakePressed

    # steering wheel
    ret.steeringAngle = cp.vl["STEERING_ANGLE_SENSOR"]['STEER_ANGLE']                     # deg
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE']                          # no units, as defined by steering interceptor, the sensor
#    ret.steeringTorqueEps = cp.vl["TORQUE_COMMAND"]['INTERCEPTOR_MAIN_TORQUE']           # no units, as defined by steering interceptor, the actuator
    ret.steeringPressed = bool(abs(ret.steeringTorque) > 34)
    ret.steerWarning = False                                                              # since Perodua has no LKAS, make it always no warning
    ret.steerError = False                                                                # since Perodua has no LKAS, make it always no warning
#    ret.stockAeb = cp.vl["FWD_CAM1"]['AEB_BRAKE'] != 0                                   # is stock AEB giving a braking signal?
#    ret.stockFcw = cp.vl["FWD_CAM1"]['AEB_WARNING'] != 0                                 # is stock AEB giving a frontal collision warning?
#    ret.espDisabled = cp.vl["ESC_CONTROL"]['STATUS'] != 0                                # electronic stability control status

    # cruise state, need to fake it for now, its used for driver monitoring, and controlsd see below
    ret.cruiseState.available = True

    # latching cruiseState logic
    if self.check_pedal_engage(ret.gas, pedal_press_state):
      self.is_cruise_latch = True
    if ret.brakePressed:
      self.is_cruise_latch = False
    ret.cruiseState.enabled = self.is_cruise_latch
 #   ret.cruiseState.enabled = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"]) or self.check_pedal_engage(ret.gas, pedal_press_state)
    ret.cruiseState.standstill = ret.standstill

    # gear
    can_gear = int(cp.vl["TRANSMISSION"]['GEAR'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # button presses
    ret.leftBlinker = bool(cp.vl["METER_CLUSTER"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["METER_CLUSTER"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"])                       # special toggle

    # blindspot sensors
    ret.leftBlindspot = False                                                              # Is there something blocking the left lane change
    ret.rightBlindspot = False                                                             # Is there something blocking the right lane change

    # lock info
    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEAT_BELT_WARNING'] == 1
    ret.doorOpen = any([cp.vl["METER_CLUSTER"]['MAIN_DOOR'],
                        cp.vl["METER_CLUSTER"]['LEFT_FRONT_DOOR'],
                        cp.vl["METER_CLUSTER"]['RIGHT_BACK_DOOR'],
                        cp.vl["METER_CLUSTER"]['LEFT_BACK_DOOR']])

    # NEED TO ADD BUTTON EVENTS FOR CRUISE

    return ret
  @staticmethod
  def check_pedal_engage(gas,state):
    ''' Pedal engage logic '''
    global pedal_counter
    global pedal_press_state
    if (state == 0):
      if (gas > PEDAL_NON_ZERO_THRES):
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
      if (gas > PEDAL_NON_ZERO_THRES):
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
      ("MAIN_TORQUE", "STEERING_TORQUE", 0),
 #     ("INTERCEPTOR_MAIN_TORQUE", "TORQUE_COMMAND", 0),
 #     ("STATUS", "ESC_CONTROL", 0),
      ("GENERIC_TOGGLE", "RIGHT_STALK", 0),
      ("FOG_LIGHT", "RIGHT_STALK", 0),
      ("LEFT_SIGNAL", "METER_CLUSTER", 0),
      ("RIGHT_SIGNAL", "METER_CLUSTER", 0),
      ("SEAT_BELT_WARNING", "METER_CLUSTER", 0),
      ("MAIN_DOOR", "METER_CLUSTER", 1),
      ("LEFT_FRONT_DOOR", "METER_CLUSTER", 1),
      ("RIGHT_BACK_DOOR", "METER_CLUSTER", 1),
      ("LEFT_BACK_DOOR", "METER_CLUSTER", 1),
    ]
    checks = []
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

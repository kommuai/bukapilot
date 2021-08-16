from cereal import car
from collections import deque
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.proton.values import DBC

# livetuner import
from kommu.livetuner.livetune_conf import livetune_conf

pedal_counter = 0
pedal_press_state = 0
PEDAL_COUNTER_THRES = 25
PEDAL_UPPER_TRIG_THRES = 0.125
PEDAL_NON_ZERO_THRES = 0.001 #was 0.01
STEER_DIFF_THRES = 0.2 # deg

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']
    self.is_cruise_latch = False
    self.prev_steer_angle = 0
    self.base_steer_thres = 0
    self.steer_boost = 0
    self.cruise_speed = 0
    self.cruise_speed_counter = 0
    self.acttrGas = 0

  def update(self, cp):
    livetune = livetune_conf()
    self.isFakeEngage = bool(round(float((livetune.conf['fakeEngage']))))

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
    self.acttrGas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS']) /1800

    # brake pedal
    ret.brake = cp.vl["BRAKE_PEDAL"]['BRAKE_PRESSURE']                                    # Use for pedal
    ret.brakePressed = ret.brake > 1e-5                                                   # Use for pedal
    ret.brakeLights = ret.brakePressed

    # steering wheel
    ret.steeringAngle = cp.vl["STEERING_ANGLE_SENSOR"]['STEER_ANGLE']                     # deg

    # perform steeringPressed value boost during actuation from baseline steering threshold
    if abs(ret.steeringAngle - self.prev_steer_angle) >= STEER_DIFF_THRES:
      self.steer_boost = 5
    else:
      self.steer_boost = 0

    self.prev_steer_angle = ret.steeringAngle
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE']                          # no units, as defined by steering interceptor, the sensor
#    ret.steeringTorqueEps = cp.vl["TORQUE_COMMAND"]['INTERCEPTOR_MAIN_TORQUE']           # no units, as defined by steering interceptor, the actuator
    # currently value got from the calibration jupyter notebook
    self.base_steer_thres = 0.00377103618*(ret.vEgo**2)+0.0568116542*ret.vEgo+19.3775297
    #ret.steeringPressed = bool(abs(ret.steeringTorque) >  + self.base_steer_thres + self.steer_boost)
    ret.steeringPressed = bool(abs(ret.steeringTorque) > 35)
    ret.steerWarning = False                                                              # since Perodua has no LKAS, make it always no warning
    ret.steerError = False                                                                # since Perodua has no LKAS, make it always no warning
#    ret.stockAeb = cp.vl["FWD_CAM1"]['AEB_BRAKE'] != 0                                   # is stock AEB giving a braking signal?
#    ret.stockFcw = cp.vl["FWD_CAM1"]['AEB_WARNING'] != 0                                 # is stock AEB giving a frontal collision warning?
#    ret.espDisabled = cp.vl["ESC_CONTROL"]['STATUS'] != 0                                # electronic stability control status

    # cruise state, need to fake it for now, its used for driver monitoring, and controlsd see below
    ret.cruiseState.available = True
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.speed = self.cruise_speed

    # Increase cruise_speed using pedal when engage
    if self.is_cruise_latch:
      self.cruise_speed_counter += 1
      if self.cruise_speed_counter % 120 == 0 and self.acttrGas > 0.4 and self.acttrGas <= 0.8:
        self.cruise_speed += (5 * CV.KPH_TO_MS)
        self.cruise_speed_counter = 0
      elif self.cruise_speed_counter % 120 == 0 and self.acttrGas > 0.8:
        self.cruise_speed -= (5 * CV.KPH_TO_MS)
        self.cruise_speed_counter = 0

    # latching cruiseState logic
    #if self.check_pedal_engage(ret.gas, pedal_press_state) or self.isFakeEngage:
    if self.check_pedal_engage(ret.gas, pedal_press_state):
      if not self.is_cruise_latch: 
        self.cruise_speed = ret.vEgo + (5 * CV.KPH_TO_MS)
      self.is_cruise_latch = True
    #if ret.brakePressed or not self.isFakeEngage:
    if ret.brakePressed:
      self.is_cruise_latch = False
    
    ret.cruiseState.enabled = self.is_cruise_latch
    ret.cruiseState.standstill = ret.standstill

    # gear
    can_gear = int(cp.vl["TRANSMISSION"]['GEAR'])
    if (self.isFakeEngage):
      ret.seatbeltUnlatched = False
      ret.doorOpen = False
      ret.gearShifter = 2
    else:
      ret.doorOpen = any([cp.vl["METER_CLUSTER"]['MAIN_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_FRONT_DOOR'],
                     cp.vl["METER_CLUSTER"]['RIGHT_BACK_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_BACK_DOOR']])

      ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEAT_BELT_WARNING'] == 1
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # button presses
    ret.leftBlinker = bool(cp.vl["METER_CLUSTER"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["METER_CLUSTER"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"])                       # special toggle

    # blindspot sensors
    ret.leftBlindspot = False                                                              # Is there something blocking the left lane change
    ret.rightBlindspot = False                                                             # Is there something blocking the right lane change

    # NEED TO ADD BUTTON EVENTS FOR CRUISE

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
      ("MAIN_DOOR", "METER_CLUSTER", 1),
      ("LEFT_FRONT_DOOR", "METER_CLUSTER", 1),
      ("RIGHT_BACK_DOOR", "METER_CLUSTER", 1),
      ("LEFT_BACK_DOOR", "METER_CLUSTER", 1),
    ]
    checks = []
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

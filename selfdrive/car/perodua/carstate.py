from cereal import car
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.perodua.values import DBC

STEER_THRESHOLD = 30

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']


  def update(self, cp):
    ret = car.CarState.new_message()
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEED"]['WHEELSPEED_B'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEED"]['WHEELSPEED_B'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.rr, ret.wheelSpeeds.rl, ret.wheelSpeeds.fr, ret.wheelSpeeds.fl])
    # unfiltered speed from CAN sensors
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    # gas pedal
    ret.gas = cp.vl["GAS_PEDAL_1"]['APPS_1']                                              # gas pedal, 0.0-1.0
    ret.gasPressed = ret.gas > 0.15

    # brake pedal
    ret.brake = cp.vl["BRAKE_PEDAL"]['BRAKE_PRESSURE']                                    # Use for pedal
    ret.brakePressed = ret.brake > 1e-5                                                   # Use for pedal
    ret.brakeLights = ret.brakePressed

    # steering wheel
    ret.steeringAngle = cp.vl["STEERING_ANGLE_SENSOR"]['STEER_ANGLE']                     # deg
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE']                          # no units, as defined by steering interceptor, the sensor
#    ret.steeringTorqueEps = cp.vl["TORQUE_COMMAND"]['INTERCEPTOR_MAIN_TORQUE']           # no units, as defined by steering interceptor, the actuator
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD                       # if the user is using the steering wheel
    ret.steerWarning = False                                                              # since Perodua has no LKAS, make it always no warning
    ret.steerError = False                                                                # since Perodua has no LKAS, make it always no warning
#    ret.stockAeb = cp.vl["FWD_CAM1"]['AEB_BRAKE'] != 0                                   # is stock AEB giving a braking signal?
#    ret.stockFcw = cp.vl["FWD_CAM1"]['AEB_WARNING'] != 0                                 # is stock AEB giving a frontal collision warning?
#    ret.espDisabled = cp.vl["ESC_CONTROL"]['STATUS'] != 0                                # electronic stability control status

    # cruise state, need to fake it for now, its used for driver monitoring, and controlsd see below
    ret.cruiseState.available = True
    ret.cruiseState.enabled = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"])
    ret.cruiseState.standstill = ret.standstill

    # gear
    can_gear = int(cp.vl["TRANSMISSION"]['GEAR'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # button presses
    ret.leftBlinker = bool(cp.vl["METER_CLUSTER"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["METER_CLUSTER"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"])                       # special function button toggle

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

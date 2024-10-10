from cereal import car
from common.params import Params
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from openpilot.common.numpy_fast import mean
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.proton.values import DBC, HUD_MULTIPLIER, CANBUS

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']
    self.set_distance_values = can_define.dv['PCM_BUTTONS']['SET_DISTANCE']

    # lks settings
    self.lks_audio = False
    self.lks_tactile = False
    self.lks_assist_mode = False
    self.lks_aux = False
    self.lka_enable = False
    self.is_icc_on = False
    self.has_audio_ldw = False

    # stock ldw ldp
    self.stock_ldw_steering = False
    self.stock_ldp_left = False
    self.stock_ldp_right = False
    self.stock_ldp_cmd = 0
    self.stock_steer_dir = 0

    self.hand_on_wheel_warning = False
    self.hand_on_wheel_chime = False

    self.is_cruise_latch = False
    self.acc_req = False
    self.prev_angle = 0

    self.p = Params()
    self.prev_distance_val = -1

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    """
    Lane Keep Assist (LKA)
    Warning Only:         LKS Assist True,  Auxiliary False
    Departure Prevention: LKS Assist True,  Auxiliary True
    Centering Control:    LKS Assist False, Auxiliary False
    """
    self.lks_audio = bool(cp_cam.vl["ADAS_LKAS"]["LKS_WARNING_AUDIO_TYPE"])
    self.lks_tactile = bool(cp_cam.vl["ADAS_LKAS"]["LKS_WARNING_TACTILE_TYPE"])
    self.lks_assist_mode = bool(cp_cam.vl["ADAS_LKAS"]["LKS_ASSIST_MODE"])
    self.lks_aux = bool(cp_cam.vl["ADAS_LKAS"]["STOCK_LKS_AUX"])
    self.lka_enable = bool(cp_cam.vl["ADAS_LKAS"]["LKA_ENABLE"])
    self.is_icc_on = bool(cp_cam.vl["PCM_BUTTONS"]["ICC_ON"])
    self.has_audio_ldw = bool(cp_cam.vl["LKAS"]["LANE_DEPARTURE_AUDIO_RIGHT"]) or \
                         bool(cp_cam.vl["LKAS"]["LANE_DEPARTURE_AUDIO_LEFT"])

    # stock ldw ldp settings update
    self.stock_ldw_steering = bool(cp_cam.vl["ADAS_LKAS"]["LDW_STEERING"])
    self.stock_ldp_left = bool(cp_cam.vl["LKAS"]["STEER_REQ_LEFT"])
    self.stock_ldp_right = bool(cp_cam.vl["LKAS"]["STEER_REQ_RIGHT"])
    self.stock_ldp_cmd = cp_cam.vl["ADAS_LKAS"]["STEER_CMD"]
    self.stock_steer_dir = bool(cp_cam.vl["ADAS_LKAS"]["STEER_DIR"])

    # miscs
    self.hand_on_wheel_warning = bool(cp_cam.vl["ADAS_LKAS"]["HAND_ON_WHEEL_WARNING"])
    self.hand_on_wheel_chime = bool(cp_cam.vl["ADAS_LKAS"]["WHEEL_WARNING_CHIME"])
    self.acc_req = bool(cp_cam.vl["ACC_CMD"]["ACC_REQ"]) or bool(cp_cam.vl["PCM_BUTTONS"]["GAS_OVERRIDE"])

    # kinematics
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
    ret.brakePressed = bool(cp.vl["PARKING_BRAKE"]["BRAKE_PRESSED"])

    # steer
    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]['STEER_ANGLE']
    steer_dir = 1 if (ret.steeringAngleDeg - self.prev_angle >= 0) else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE'] * steer_dir
    ret.steeringTorqueEps = cp.vl["STEERING_MODULE"]['STEER_RATE'] * steer_dir
    ret.steeringPressed = bool(abs(ret.steeringTorque) > 55)

    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER

    # Todo: get the real value
    ret.stockAeb = False
    ret.stockFcw = bool(cp_cam.vl["FCW"]["STOCK_FCW_TRIGGERED"])

    ret.cruiseState.available = True
    distance_val = int(cp_cam.vl["PCM_BUTTONS"]['SET_DISTANCE'])

    if distance_val != self.prev_distance_val:
      self.p.put("LongitudinalPersonality", str(distance_val - 1))
    self.prev_distance_val = distance_val

    # TODO: ret.cruiseState.setDistance = self.parse_set_distance(self.set_distance_values.get(distance_val, None))

    self.cruise_speed = int(cp_cam.vl["PCM_BUTTONS"]['ACC_SET_SPEED']) * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = self.cruise_speed
    ret.cruiseState.speed = ret.cruiseState.speedCluster / HUD_MULTIPLIER
    ret.cruiseState.standstill = bool(cp_cam.vl["ACC_CMD"]["STANDSTILL_REQ"])
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False

    if (bool(cp.vl["ACC_BUTTONS"]["SET_BUTTON"]) or bool(cp.vl["ACC_BUTTONS"]["RES_BUTTON"])) and bool(cp_cam.vl["PCM_BUTTONS"]["ACC_ON_OFF_BUTTON"]):
      self.is_cruise_latch = True
    if not ret.cruiseState.available or ret.brakePressed:
      self.is_cruise_latch = False 
    if bool(cp.vl["ACC_BUTTONS"]["CRUISE_BTN"]):
      self.is_cruise_latch = False
    # temporary safety
    if ret.standstill and ret.gasPressed:
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
      # TODO get the frequency
      # sig_address, frequency
      ("WHEEL_SPEED", 0),
      ("ACC_BUTTONS", 0),
      ("PARKING_BRAKE", 0),
      ("TRANSMISSION", 0),
      ("GAS_PEDAL", 0),
      ("BRAKE", 0),
      ("STEERING_TORQUE", 0),
      ("STEERING_MODULE", 0),
      ("LEFT_STALK", 0),
      ("BSM_ADAS", 0),
      ("SEATBELTS", 0),
      ("DOOR_LEFT_SIDE", 0),
      ("DOOR_RIGHT_SIDE", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # TODO get the frequency
      # sig_address, frequency
      ("ADAS_LEAD_DETECT", 0),
      ("ACC_CMD", 0),
      ("ADAS_LKAS", 0),
      ("LKAS", 0),
      ("FCW", 0),
      ("PCM_BUTTONS", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, 1)

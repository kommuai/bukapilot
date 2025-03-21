from cereal import car
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.proton.values import DBC, HUD_MULTIPLIER
from time import monotonic
from enum import Enum, auto
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from common.features import Features
from common.params import Params

BLINKER_MIN = 3.5 # Minimum turn signal length in seconds

class Dir(Enum):
  LEFT = auto()
  RIGHT = auto()

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']
    self.set_distance_values = can_define.dv['PCM_BUTTONS']['SET_DISTANCE']

    self.lks_audio = None
    self.lks_tactile = None
    self.is_icc_on = None
    self.lks_assist_mode = False
    self.lks_aux = False
    self.lka_enable = False
    self.stock_ldw_steering = False
    self.has_audio_ldw = False
    self.stock_ldp_left = False
    self.stock_ldp_right = False
    self.stock_ldp_cmd = 0
    self.steer_dir = 0

    self.cur_blinker = None
    self.blinker_on_alc_active = False
    self.blinker_start_time = 0

    self.is_cruise_latch = False
    self.acc_req = False
    self.hand_on_wheel_warning = False
    self.hand_on_wheel_warning_2 = False
    self.prev_angle = 0
    self.res_btn_pressed = False
    self.gas_override = False

    f = Features()
    self.mads = f.has("StockAcc")
    self.is_alc_enabled = Params().get_bool("IsAlcEnabled")

  def set_cur_blinker(self, alc_not_active, rightBlinker):
    """Reset time and set cur_blinker"""
    self.blinker_start_time = monotonic()
    self.cur_blinker = Dir.RIGHT if rightBlinker else Dir.LEFT
    self.blinker_on_alc_active = not alc_not_active # Check when blinker on / direction change, if ALC was active

  def update(self, cp):
    mads = self.mads
    ret = car.CarState.new_message()

    self.stock_ldp_cmd = cp.vl["ADAS_LKAS"]["STEER_CMD"]
    self.stock_ldw_steering = cp.vl["ADAS_LKAS"]["LDW_STEERING"]
    self.steer_dir = cp.vl["ADAS_LKAS"]["STEER_DIR"]
    self.stock_ldp_left = cp.vl["LKAS"]["STEER_REQ_LEFT"]
    self.stock_ldp_right = cp.vl["LKAS"]["STEER_REQ_RIGHT"]
    self.has_audio_ldw = cp.vl["LKAS"]["LANE_DEPARTURE_AUDIO_RIGHT"] or cp.vl["LKAS"]["LANE_DEPARTURE_AUDIO_LEFT"]

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_B'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_B'],
    )
    ret.vEgoRaw = vEgoRaw = mean([ret.wheelSpeeds.rr, ret.wheelSpeeds.rl, ret.wheelSpeeds.fr, ret.wheelSpeeds.fl])

    # unfiltered speed from CAN sensors
    ret.vEgo, ret.aEgo = self.update_speed_kf(vEgoRaw)
    ret.standstill = vEgoRaw < 0.01

    # safety checks to engage
    can_gear = int(cp.vl["TRANSMISSION"]['GEAR'])

    ret.doorOpen = doorOpen = any([cp.vl["DOOR_LEFT_SIDE"]['BACK_LEFT_DOOR'],
                              cp.vl["DOOR_LEFT_SIDE"]['FRONT_LEFT_DOOR'],
                              cp.vl["DOOR_RIGHT_SIDE"]['BACK_RIGHT_DOOR'],
                              cp.vl["DOOR_RIGHT_SIDE"]['FRONT_RIGHT_DOOR']])

    ret.seatbeltUnlatched = seatbeltUnlatched = bool(cp.vl["SEATBELTS"]['RIGHT_SIDE_SEATBELT_ACTIVE_LOW'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.brakeHoldActive = brakeHoldActive = bool(cp.vl["PARKING_BRAKE"]["CAR_ON_HOLD"])

    # disengage
    if brakeHoldActive or seatbeltUnlatched or doorOpen:
      self.is_cruise_latch = False

    # gas pedal
    ret.gas = gas = cp.vl["GAS_PEDAL"]['APPS_1']
    ret.gasPressed = gas > 0.01

    # brake pedal
    ret.brake = cp.vl["BRAKE"]['BRAKE_PRESSURE']
    ret.brakePressed = brakePressed = False if mads else bool(cp.vl["PARKING_BRAKE"]["BRAKE_PRESSED"])

    # steer
    ret.steeringAngleDeg = steeringAngleDeg = cp.vl["STEERING_MODULE"]['STEER_ANGLE']
    steer_dir = 1 if (steeringAngleDeg - self.prev_angle >= 0) else -1
    self.prev_angle = steeringAngleDeg
    ret.steeringTorque = steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE'] * steer_dir
    ret.steeringTorqueEps = cp.vl["STEERING_MODULE"]['STEER_RATE'] * steer_dir
    ret.steeringPressed = abs(steeringTorque) > 124
    ret.steerWarning = False
    ret.steerError = False
    self.hand_on_wheel_warning = cp.vl["ADAS_LKAS"]["HAND_ON_WHEEL_WARNING"]
    self.hand_on_wheel_warning_2 = cp.vl["ADAS_LKAS"]["WHEEL_WARNING_CHIME"]
    self.leadDistance = cp.vl["ADAS_LEAD_DETECT"]["LEAD_DISTANCE"]
    self.lka_enable = lka_enable = cp.vl["ADAS_LKAS"]["LKA_ENABLE"]

    ret.vEgoCluster = (vEgo := ret.vEgo) * HUD_MULTIPLIER

    # Todo: get the real value
    ret.stockAeb = False
    ret.stockFcw = bool(cp.vl["FCW"]["STOCK_FCW_TRIGGERED"])

    self.acc_req = acc_req = cp.vl["ACC_CMD"]["ACC_REQ"]
    self.gas_override = gas_override = cp.vl["PCM_BUTTONS"]["GAS_OVERRIDE"]
    ret.cruiseState.available = cruise_available = bool(gas_override or cp.vl["PCM_BUTTONS"]["ACC_ON_OFF_BUTTON"])
    ret.cruiseState.setDistance = self.parse_set_distance(self.set_distance_values.get(int(cp.vl["PCM_BUTTONS"]['SET_DISTANCE']), None))
    self.res_btn_pressed = cp.vl["ACC_BUTTONS"]["RES_BUTTON"]

    # engage and disengage logic
    if mads:
      self.is_cruise_latch = cruise_available or (not brakePressed and cp.vl["PCM_BUTTONS"]["ACC_SET"])
    else:
      self.is_cruise_latch = not brakePressed and cp.vl["PCM_BUTTONS"]["ACC_SET"]

    # set speed
    self.cruise_speed = int(cp.vl["PCM_BUTTONS"]['ACC_SET_SPEED']) * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = cruise_speedCluster =  self.cruise_speed
    ret.cruiseState.speed = cruise_speedCluster / HUD_MULTIPLIER
    ret.cruiseState.standstill = cruise_standstill = bool(cp.vl["ACC_CMD"]["STANDSTILL2"])
    ret.cruiseState.nonAdaptive = False

    if not cruise_available or (not mads and (brakePressed or (not cruise_standstill and not acc_req))):
      self.is_cruise_latch = False

    ret.cruiseState.enabled = bool(self.is_cruise_latch)

    # Turn signal with a required minimum time
    one_blinker = (leftBlinker := bool(cp.vl["LEFT_STALK"]["LEFT_SIGNAL"])) != (rightBlinker := bool(cp.vl["LEFT_STALK"]["RIGHT_SIGNAL"]))

    # Use minimum blinker time if ALC is not active
    alc_not_active = vEgo < LANE_CHANGE_SPEED_MIN or not self.is_alc_enabled

    if self.cur_blinker is None:
      self.blinker_on_alc_active = False
      if one_blinker: # Turn signal was off and is now on
        self.set_cur_blinker(alc_not_active, rightBlinker)
    else:
      # cur_blinker is left or right
      if not one_blinker and \
      (self.blinker_on_alc_active or (monotonic() - self.blinker_start_time) >= BLINKER_MIN):
        self.cur_blinker = None
      elif (leftBlinker and self.cur_blinker == Dir.RIGHT) or (rightBlinker and self.cur_blinker == Dir.LEFT):
        # Change in blinker direction
        self.set_cur_blinker(alc_not_active, rightBlinker)

    if alc_not_active:
      cur_blinker = self.cur_blinker
      ret.leftBlinker, ret.rightBlinker = cur_blinker == Dir.LEFT, cur_blinker == Dir.RIGHT
    else:
      ret.leftBlinker, ret.rightBlinker = leftBlinker, rightBlinker

    # button presses
    ret.genericToggle = bool(cp.vl["LEFT_STALK"]["GENERIC_TOGGLE"]) # High beam toggle
    ret.espDisabled = bool(not cp.vl["PARKING_BRAKE"]["ESC_ON"])

    # blindspot sensors
    if self.CP.enableBsm:
      # used for lane change so its okay for the chime to work on both side.
      ret.leftBlindspot = bool(cp.vl["BSM_ADAS"]["LEFT_APPROACH"] or cp.vl["BSM_ADAS"]["LEFT_APPROACH_WARNING"])
      ret.rightBlindspot = bool(cp.vl["BSM_ADAS"]["RIGHT_APPROACH"] or cp.vl["BSM_ADAS"]["RIGHT_APPROACH_WARNING"])

    """
    Lane Keep Assist (LKA)
    Warning Only:         LKS Assist True,  Auxiliary False
    Departure Prevention: LKS Assist True,  Auxiliary True
    Centering Control:    LKS Assist False, Auxiliary False
    """
    self.lks_assist_mode = cp.vl["ADAS_LKAS"]["LKS_ASSIST_MODE"]
    self.lks_aux = cp.vl["ADAS_LKAS"]["STOCK_LKS_AUX"]
    self.lks_audio = cp.vl["ADAS_LKAS"]["LKS_WARNING_AUDIO_TYPE"]
    self.lks_tactile = cp.vl["ADAS_LKAS"]["LKS_WARNING_TACTILE_TYPE"]
    self.is_icc_on = is_icc_on = cp.vl["PCM_BUTTONS"]["ICC_ON"]
    # If cruise mode is ICC, make bukapilot control steering so it won't disengage.
    ret.lkaDisabled = bool(not lka_enable and not is_icc_on)

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("RES_BUTTON", "ACC_BUTTONS", 0),
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
      ("WHEEL_WARNING_CHIME", "ADAS_LKAS", 1),
      ("STOCK_LKS_AUX", "ADAS_LKAS", 0),
      ("LKS_WARNING_AUDIO_TYPE", "ADAS_LKAS", 0),
      ("LKS_WARNING_TACTILE_TYPE", "ADAS_LKAS", 0),
      ("LKS_ASSIST_MODE", "ADAS_LKAS", 1),
      ("STEER_DIR", "ADAS_LKAS", 1),
      ("LDW_STEERING", "ADAS_LKAS", 0),
      ("STEER_CMD", "ADAS_LKAS", 1),
      ("LANE_DEPARTURE_WARNING_RIGHT", "LKAS", 1),
      ("LANE_DEPARTURE_WARNING_LEFT", "LKAS", 1),
      ("STOCK_FCW_TRIGGERED", "FCW", 1),
      ("LKA_ENABLE", "ADAS_LKAS", 1),
      ("STEER_REQ_RIGHT", "LKAS", 0),
      ("STEER_REQ_LEFT", "LKAS", 0),
      ("LANE_DEPARTURE_AUDIO_RIGHT", "LKAS", 0),
      ("LANE_DEPARTURE_AUDIO_LEFT", "LKAS", 0),
    ]
    checks = []

    # todo: make it such that enforce_checks=True
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0, enforce_checks=False)

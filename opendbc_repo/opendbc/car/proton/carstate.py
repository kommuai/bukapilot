from cereal import car
import math
from opendbc.can import CANDefine, CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import Bus, create_button_events
from opendbc.car.interfaces import CarStateBase
from opendbc.car.proton.values import DBC, HUD_MULTIPLIER, CANBUS, CAR
from time import monotonic
from enum import Enum, auto
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN

from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
PROTON_DISTANCE_TO_PERSONALITY = {
  1: 0,  # 1 bar
  2: 1,  # 2 bar
  3: 2,  # 3 bar
}
BLINKER_MIN = 2.25 # Minimum turn signal length in seconds

class Dir(Enum):
  LEFT = auto()
  RIGHT = auto()


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["TRANSMISSION"]["GEAR"]
    self.CP = CP

    self.lks_audio = False
    self.lks_tactile = False
    self.lks_assist_mode = False
    self.lks_aux = False
    self.lka_enable = False
    self.is_icc_on = False
    self.has_audio_ldw = False

    self.stock_ldw_steering = False
    self.stock_ldp_left = False
    self.stock_ldp_right = False
    self.stock_ldp_cmd = 0
    self.stock_steer_dir = 0

    self.hand_on_wheel_warning = False
    self.hand_on_wheel_warning_2 = False

    self.prev_angle = 0
    self.res_btn_pressed = False

    self.stock_acc_cmd = 0
    self.stock_acc_cmd_values = {}
    self.cruise_standstill = False

    self.is_alc_enabled = Params().get_bool("IsAlcEnabled")
    self.cur_blinker = None
    self.blinker_on_alc_speed = False
    self.blinker_start_time = 0
    self.distance_val = 1

  def set_cur_blinker(self, alc_below_min_speed, right_blinker):
    self.blinker_start_time = monotonic()
    self.cur_blinker = Dir.RIGHT if right_blinker else Dir.LEFT
    self.blinker_on_alc_speed = not alc_below_min_speed # Check when blinker on / direction change, if ALC speed was enough

  def _update_lks_state(self, cp_cam):
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
    self.has_audio_ldw = bool(cp_cam.vl["LKAS"]["LANE_DEPARTURE_AUDIO_RIGHT"]) or bool(
      cp_cam.vl["LKAS"]["LANE_DEPARTURE_AUDIO_LEFT"]
    )

    self.stock_ldw_steering = bool(cp_cam.vl["ADAS_LKAS"]["LDW_STEERING"])
    self.stock_ldp_left = bool(cp_cam.vl["LKAS"]["STEER_REQ_LEFT"])
    self.stock_ldp_right = bool(cp_cam.vl["LKAS"]["STEER_REQ_RIGHT"])
    self.stock_ldp_cmd = cp_cam.vl["ADAS_LKAS"]["STEER_CMD"]
    self.stock_steer_dir = bool(cp_cam.vl["ADAS_LKAS"]["STEER_DIR"])

    self.hand_on_wheel_warning = bool(cp_cam.vl["ADAS_LKAS"]["HAND_ON_WHEEL_WARNING"])
    self.hand_on_wheel_warning_2 = bool(cp_cam.vl["ADAS_LKAS"]["WHEEL_WARNING_CHIME"]) # The second warning before ICC disengage
    acc_cmd = cp_cam.vl["ACC_CMD"]
    self.stock_acc_cmd = acc_cmd["CMD"]
    self.stock_acc_cmd_values = dict(acc_cmd)

  def _apply_blinker_minimum_time(self, ret):
    left_blinker = ret.leftBlinker
    right_blinker = ret.rightBlinker
    one_blinker = left_blinker != right_blinker

    # Use minimum blinker time if ALC speed not enough
    alc_below_min_speed = ret.vEgo < LANE_CHANGE_SPEED_MIN or not self.is_alc_enabled

    if self.cur_blinker is None:
      self.blinker_on_alc_speed = False
      if one_blinker: # Turn signal was off and is now on
        self.set_cur_blinker(alc_below_min_speed, right_blinker)
    else:
      # cur_blinker is left or right
      elapsed = monotonic() - self.blinker_start_time
      if not one_blinker and (self.blinker_on_alc_speed or elapsed >= BLINKER_MIN):
        self.cur_blinker = None
      elif (left_blinker and self.cur_blinker == Dir.RIGHT) or (right_blinker and self.cur_blinker == Dir.LEFT):
        self.set_cur_blinker(alc_below_min_speed, right_blinker) # Change in blinker direction

    if alc_below_min_speed:
      ret.leftBlinker = self.cur_blinker == Dir.LEFT
      ret.rightBlinker = self.cur_blinker == Dir.RIGHT

  def update(self, can_parsers):
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = car.CarState.new_message()
    ret.buttonEvents = []
    ret.personality = -1

    self._update_lks_state(cp_cam)
    ret.stockAccelCmd = float(self.stock_acc_cmd)

    # If cruise mode is ICC, make bukapilot control steering so it won't disengage by itself.
    ret.lkaDisabled = not (self.lka_enable or self.is_icc_on)

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_B"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_B"],
    )
    ret.standstill = ret.vEgoRaw < 0.01

    can_gear = int(cp.vl["TRANSMISSION"]["GEAR"])

    ret.doorOpen = any([
      cp.vl["DOOR_LEFT_SIDE"]["BACK_LEFT_DOOR"],
      cp.vl["DOOR_LEFT_SIDE"]["FRONT_LEFT_DOOR"],
      cp.vl["DOOR_RIGHT_SIDE"]["BACK_RIGHT_DOOR"],
      cp.vl["DOOR_RIGHT_SIDE"]["FRONT_RIGHT_DOOR"],
    ])

    ret.seatbeltUnlatched = cp.vl["SEATBELTS"]["RIGHT_SIDE_SEATBELT_ACTIVE_LOW"] == 1

    if self.CP.carFingerprint == CAR.PROTON_X90:
      ret.gearShifter = 2 # hardcode to drive because stock X90 has non standard gear
    else:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.brakeHoldActive = bool(cp.vl["PARKING_BRAKE"]["CAR_ON_HOLD"])

    ret.gasPressed = cp.vl["GAS_PEDAL"]["APPS_1"] > 0.01

    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"]
    ret.brakePressed = bool(cp.vl["PARKING_BRAKE"]["BRAKE_PRESSED"])

    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]["STEER_ANGLE"]
    steer_dir = 1 if (ret.steeringAngleDeg - self.prev_angle >= 0) else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"] * steer_dir
    ret.steeringTorqueEps = cp.vl["STEERING_MODULE"]["STEER_RATE"] * steer_dir
    if self.CP.carFingerprint == CAR.PROTON_X50:
      ret.steeringPressed = abs(ret.steeringTorque) > 124
    else:
      ret.steeringPressed = abs(ret.steeringTorque) > 65

    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER

    ret.stockAeb = False
    ret.stockFcw = bool(cp_cam.vl["FCW"]["STOCK_FCW_TRIGGERED"])

    #TODO: If using car signal, S70 cannot engage, X50 gas press would make it False.
    ret.cruiseState.available = True

    self.res_btn_pressed = bool(cp.vl["ACC_BUTTONS"]["RES_BUTTON"])
    prev_distance_val = self.distance_val
    self.distance_val = int(cp_cam.vl["PCM_BUTTONS"]["SET_DISTANCE"])
    ret.personality = PROTON_DISTANCE_TO_PERSONALITY.get(self.distance_val, -1)
    if self.distance_val != prev_distance_val:
      ret.buttonEvents = create_button_events(1, 0, {1: ButtonType.gapAdjustCruise}) + \
                         create_button_events(0, 1, {1: ButtonType.gapAdjustCruise})

    self.cruise_speed = int(cp_cam.vl["PCM_BUTTONS"]["ACC_SET_SPEED"]) * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = self.cruise_speed
    ret.cruiseState.speed = ret.cruiseState.speedCluster / HUD_MULTIPLIER
    self.cruise_standstill = bool(cp_cam.vl["ACC_CMD"]["STANDSTILL_REQ"]) and not ret.gasPressed
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.enabled = (
      cp_cam.vl["ACC_CMD"]["ACC_REQ"] + cp_cam.vl["ACC_CMD"]["STANDSTILL_REQ"] + cp_cam.vl["ACC_CMD"]["NOT_MOVE_BLOCK"]
    ) > 1

    ret.leftBlinker = bool(cp.vl["LEFT_STALK"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["LEFT_STALK"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["LEFT_STALK"]["GENERIC_TOGGLE"])
    ret.espDisabled = bool(cp.vl["PARKING_BRAKE"]["ESC_ON"]) != 1

    if self.CP.enableBsm:
      ret.leftBlindspot = bool(cp.vl["BSM_ADAS"]["LEFT_APPROACH"]) or bool(cp.vl["BSM_ADAS"]["LEFT_APPROACH_WARNING"])
      ret.rightBlindspot = bool(cp.vl["BSM_ADAS"]["RIGHT_APPROACH"]) or bool(cp.vl["BSM_ADAS"]["RIGHT_APPROACH_WARNING"])

    self._apply_blinker_minimum_time(ret)

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CarState.get_can_parser(CP),
      Bus.cam: CarState.get_cam_can_parser(CP),
    }

  @staticmethod
  def get_can_parser(CP):
    # Some Proton signals may be missing for more than 3 seconds after startup, so math.nan avoids false timeout errors.
    signals = [
      ("WHEEL_SPEED", 50),
      ("ACC_BUTTONS", math.nan),
      ("PARKING_BRAKE", 50),
      ("TRANSMISSION", 50),
      ("GAS_PEDAL", 100),
      ("BRAKE", 50),
      ("STEERING_TORQUE", math.nan),
      ("STEERING_MODULE", 100),
      ("LEFT_STALK", math.nan),
      ("BSM_ADAS", math.nan),
      ("SEATBELTS", math.nan),
      ("DOOR_LEFT_SIDE", math.nan),
      ("DOOR_RIGHT_SIDE", math.nan),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    # FCW is event-driven on Proton and should not be treated as a periodic alive signal.
    # Some Proton signals may be missing after startup, so math.nan avoids false timeout errors.
    signals = [
      ("ADAS_LEAD_DETECT", math.nan),
      ("ACC_CMD", 50),
      ("ADAS_LKAS", 50),
      ("LKAS", math.nan),
      ("FCW", math.nan),
      ("PCM_BUTTONS", math.nan),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.cam_bus)

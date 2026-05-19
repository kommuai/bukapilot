from cereal import car
from opendbc.can import CANParser
from opendbc.car import Bus, create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.chery.values import (
  CAM_PARSER_MSGS,
  CANBUS,
  DBC,
  FOLLOW_RAW_TO_PERSONALITY,
  GEAR_MAP,
  HUD_MULTIPLIER,
  PT_PARSER_MSGS,
  STEER_RELATED_INTERVENTION_RAW_MIN,
  chery_steering_deg_sign,
)

ButtonType = car.CarState.ButtonEvent.Type


def _can_parser(CP, msgs, bus):
  return CANParser(DBC[CP.carFingerprint]["pt"], msgs, bus)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.prev_icc = self.prev_cruise = False
    self.distance_val = 1
    self.pcm_button_counter = 0
    self.prev_angle = 0.0
    self.lkas_info_steer_related = 0.0
    self.steer_related_intervention = False
    self.lkas_enable_lane = self.lkas_enable_info = False

  def _parse_motion(self, ret, cp):
    self.parse_wheel_speeds(
      ret, cp.vl["WHEELSPEED_2"]["WHEEL_FL"], cp.vl["WHEELSPEED_2"]["WHEEL_FR"],
      cp.vl["WHEELSPEED_1"]["WHEEL_BL"], cp.vl["WHEELSPEED_1"]["WHEEL_BR"],
    )
    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER
    sign = chery_steering_deg_sign(self.CP)
    ret.steeringAngleDeg = sign * float(cp.vl["EPS"]["STEERING_ANGLE"])
    ret.steeringTorque = cp.vl["EPS"]["DRIVER_TORQUE"]
    steer_dir = 1 if ret.steeringAngleDeg >= self.prev_angle else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorqueEps = ret.steeringTorque * steer_dir
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 10, 5)
    ret.gasPressed = cp.vl["GAS"]["GAS_PEDAL_PRESSURE"] > 0.01
    ret.brake = cp.vl["BRAKE_PEDAL"]["BRAKE_PRESSURE"]
    ret.brakePressed = ret.brake > 0.01
    ret.gearShifter = self.parse_gear_shifter(GEAR_MAP.get(int(cp.vl["TRANSMISSION"]["GEAR"])))
    ret.standstill = ret.vEgoRaw < 0.01

  def _parse_body(self, ret, cp, cam):
    left, right = self.update_blinker_from_stalk(3, cp.vl["STALK"]["LEFT_BLINKER"], cp.vl["STALK"]["RIGHT_BLINKER"])
    ret.leftBlinker, ret.rightBlinker = left, right
    ret.doorOpen = bool(int(cp.vl["STALK"]["PAYLOAD391_B3"]) & 1)
    ret.genericToggle = False
    ret.seatbeltUnlatched = bool(
      cp.vl["SEATBELT_287"]["DRIVER_UNBUCKLED"] or cp.vl["SEATBELT_430"]["DRIVER_UNBUCKLED"]
    )
    ret.espDisabled = False
    ret.stockAeb = bool(cam.vl["HUD"]["AEB"])
    ret.stockFcw = bool(cam.vl["HUD"]["PCW"])

  def _parse_cruise(self, ret, cam):
    hud = cam.vl["HUD"]
    cruise_state = int(hud["CRUISE_STATE"])
    set_kph = float(hud["SET_SPEED"])
    ret.cruiseState.available = True
    ret.cruiseState.enabled = cruise_state == 3
    speed = set_kph * CV.KPH_TO_MS if set_kph > 0 else 0.0
    ret.cruiseState.speedCluster = ret.cruiseState.speed = speed
    ret.cruiseState.standstill = ret.standstill and ret.cruiseState.enabled
    ret.cruiseState.nonAdaptive = False

    self.distance_val = int(hud["FOLLOW_DISTANCE"])
    ret.personality = FOLLOW_RAW_TO_PERSONALITY.get(self.distance_val, -1)
    if ret.personality != -1:
      ret.personality = max(0, min(ret.personality, 2))

  def _parse_buttons(self, ret, cp):
    pcm = cp.vl["PCM_BUTTONS"]
    self.pcm_button_counter = int(pcm["COUNTER"])
    icc, cruise_btn = bool(pcm["ICC_TOGGLE"]), bool(pcm["CRUISE_BUTTON"])
    ret.buttonEvents = (
      create_button_events(icc, self.prev_icc, {1: ButtonType.altButton2}) +
      create_button_events(cruise_btn, self.prev_cruise, {1: ButtonType.mainCruise})
    )
    self.prev_icc, self.prev_cruise = icc, cruise_btn

  def _parse_adas(self, cp, cam):
    self.lkas_enable_lane = bool(cam.vl["LANE_KEEP"]["LKAS_ENABLE"])
    lkas = cp.vl["LKAS_INFO"]
    self.lkas_enable_info = bool(lkas["LKAS_ENABLE"])
    self.lkas_info_steer_related = float(lkas["STEER_RELATED"])
    sr = int(cp.vl["STEER_RELATED"]["STEERING_ANGLE_NOT_CALIBRATED"])
    self.steer_related_intervention = sr >= STEER_RELATED_INTERVENTION_RAW_MIN

  def update(self, can_parsers):
    cp, cam = can_parsers[Bus.pt], can_parsers[Bus.cam]
    ret = car.CarState.new_message()
    ret.personality = -1
    self._parse_motion(ret, cp)
    self._parse_body(ret, cp, cam)
    self._parse_cruise(ret, cam)
    self._parse_buttons(ret, cp)
    self._parse_adas(cp, cam)
    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {Bus.pt: CarState.get_can_parser(CP), Bus.cam: CarState.get_cam_can_parser(CP)}

  @staticmethod
  def get_can_parser(CP):
    return _can_parser(CP, PT_PARSER_MSGS, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    return _can_parser(CP, CAM_PARSER_MSGS, CANBUS.cam_bus)

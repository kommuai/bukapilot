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
  PT_PARSER_MSGS,
  STEER_RELATED_INTERVENTION_RAW_MIN,
)

ButtonType = car.CarState.ButtonEvent.Type

# Camera HUD fields mirrored on bus 0 by CarController (HANDS_ON_WHEEL_STEER forced to 0).
_CAM_HUD_FIELDS = (
  "AEB", "CANCEL_CRUISE_UNCERTAIN", "GAS_RESUME_UNCERTAIN", "FOLLOW_DISTANCE",
  "NEW_SIGNAL_1", "PCW", "CRUISE_STATE", "GAS_OVERRIDE", "AEB_RELATED", "SET_SPEED",
  "HANDS_ON_WHEEL_STEER",
)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.prev_icc = False
    self.prev_cruise = False
    self.prev_angle = 0.0
    # Exposed to CarController:
    self.pcm_button_counter = 0
    self.lkas_info_steer_related = 0.0
    self.steer_related_intervention = False
    self.cam_hud = {f: 0 for f in _CAM_HUD_FIELDS}
    # Live EPS snapshot — used by CarController to rebuild EPS on bus 2 byte-identical
    # to stock (panda blocks native fwd while our spoof loop is active).
    self.eps_steering_angle = 0.0
    self.eps_driver_torque = 0
    self.eps_counter = 0

  def update(self, can_parsers):
    cp, cam = can_parsers[Bus.pt], can_parsers[Bus.cam]
    ret = car.CarState.new_message()
    ret.personality = -1

    # --- Wheels / pedals / gear ---
    self.parse_wheel_speeds(
      ret, cp.vl["WHEELSPEED_2"]["WHEEL_FL"], cp.vl["WHEELSPEED_2"]["WHEEL_FR"],
      cp.vl["WHEELSPEED_1"]["WHEEL_BL"], cp.vl["WHEELSPEED_1"]["WHEEL_BR"],
    )
    ret.vEgoCluster = ret.vEgo
    ret.standstill = ret.vEgoRaw < 0.01

    eps = cp.vl["EPS"]
    ret.steeringAngleDeg = float(eps["STEERING_ANGLE"])
    ret.steeringTorque = eps["DRIVER_TORQUE"]
    self.eps_steering_angle = float(eps["STEERING_ANGLE"])
    self.eps_driver_torque = int(eps["DRIVER_TORQUE"])
    self.eps_counter = int(eps["COUNTER"])
    # DRIVER_TORQUE is unsigned; infer sign from angle delta for steeringTorqueEps.
    steer_dir = 1 if ret.steeringAngleDeg >= self.prev_angle else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorqueEps = ret.steeringTorque * steer_dir
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 5, 5)

    ret.gasPressed = cp.vl["GAS"]["GAS_PEDAL_PRESSURE"] > 0.01
    ret.brake = cp.vl["BRAKE_PEDAL"]["BRAKE_PRESSURE"]
    ret.brakePressed = ret.brake > 0.01
    ret.gearShifter = self.parse_gear_shifter(GEAR_MAP.get(int(cp.vl["TRANSMISSION"]["GEAR"])))

    # --- Body / stalk ---
    stalk = cp.vl["STALK"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(
      3, stalk["LEFT_BLINKER"], stalk["RIGHT_BLINKER"],
    )
    ret.doorOpen = bool(int(stalk["PAYLOAD391_B3"]) & 1)
    ret.genericToggle = bool(stalk["GENERIC_TOGGLE"])
    ret.seatbeltUnlatched = bool(
      cp.vl["SEATBELT_287"]["DRIVER_UNBUCKLED"] or cp.vl["SEATBELT_430"]["DRIVER_UNBUCKLED"]
    )
    ret.espDisabled = False

    # --- Cruise / HUD (from camera bus) ---
    hud = cam.vl["HUD"]
    self.cam_hud = {f: hud[f] for f in _CAM_HUD_FIELDS}
    ret.stockAeb = bool(hud["AEB"])
    ret.stockFcw = bool(hud["PCW"])

    set_kph = float(hud["SET_SPEED"])
    ret.cruiseState.available = True
    ret.cruiseState.enabled = int(hud["CRUISE_STATE"]) == 3
    ret.cruiseState.speed = ret.cruiseState.speedCluster = set_kph * CV.KPH_TO_MS if set_kph > 0 else 0.0
    ret.cruiseState.standstill = ret.standstill and ret.cruiseState.enabled
    ret.cruiseState.nonAdaptive = False

    distance_raw = int(hud["FOLLOW_DISTANCE"])
    personality = FOLLOW_RAW_TO_PERSONALITY.get(distance_raw, -1)
    ret.personality = max(0, min(personality, 2)) if personality != -1 else -1

    # --- PCM buttons ---
    pcm = cp.vl["PCM_BUTTONS"]
    self.pcm_button_counter = int(pcm["COUNTER"])
    icc, cruise_btn = bool(pcm["ICC_TOGGLE"]), bool(pcm["CRUISE_BUTTON"])
    ret.buttonEvents = (
      create_button_events(icc, self.prev_icc, {1: ButtonType.altButton2}) +
      create_button_events(cruise_btn, self.prev_cruise, {1: ButtonType.mainCruise})
    )
    self.prev_icc, self.prev_cruise = icc, cruise_btn

    # --- ADAS / LKAS state used by CarController ---
    lkas = cp.vl["LKAS_INFO"]
    self.lkas_info_steer_related = float(lkas["STEER_RELATED"])
    sr_raw = int(cp.vl["STEER_RELATED"]["STEERING_ANGLE_NOT_CALIBRATED"])
    self.steer_related_intervention = sr_raw >= STEER_RELATED_INTERVENTION_RAW_MIN

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {Bus.pt: CarState.get_can_parser(CP), Bus.cam: CarState.get_cam_can_parser(CP)}

  @staticmethod
  def get_can_parser(CP):
    return CANParser(DBC[CP.carFingerprint]["pt"], PT_PARSER_MSGS, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    return CANParser(DBC[CP.carFingerprint]["pt"], CAM_PARSER_MSGS, CANBUS.cam_bus)

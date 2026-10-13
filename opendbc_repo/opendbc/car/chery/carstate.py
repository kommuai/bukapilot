from cereal import car
from opendbc.can import CANParser
from opendbc.car import Bus, create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.chery.values import (
  CAM_PARSER_MSGS,
  CANBUS,
  CAR,
  DBC,
  FOLLOW_RAW_TO_PERSONALITY,
  GEAR_MAP,
  ICAUR_BLINKER_LEFT,
  ICAUR_BLINKER_RIGHT,
  ICAUR_BRAKE_PRESSED,
  ICAUR_CAM_PARSER_MSGS,
  ICAUR_GAS_PRESSED,
  ICAUR_GEAR_MAP,
  ICAUR_PT_PARSER_MSGS,
  ICAUR_STEERING_PRESSED_MIN_COUNT,
  ICAUR_STEERING_PRESSED_TORQUE,
  OMODA_BRAKE_PRESSURE_RAW_MAX,
  OMODA_BRAKE_PRESSURE_RAW_MIN,
  OMODA_CAM_PARSER_MSGS,
  OMODA_GEAR_MAP,
  OMODA_PT_PARSER_MSGS,
  PT_PARSER_MSGS,
  STEER_RELATED_INTERVENTION_DEG_MIN,
  TIGGO22_CAM_PARSER_MSGS,
  TIGGO22_LK_PARSER_BYPASS,
  TIGGO22_LK_PARSER_MSGS,
  TIGGO22_PT_PARSER_MSGS,
  TIGGO_DISTANCE_BAR_TO_PERSONALITY,
)

ButtonType = car.CarState.ButtonEvent.Type

# Camera HUD fields used for cruise/ADAS state (and Jaecoo HUD override TX when enabled).
_CAM_HUD_FIELDS = (
  "AEB", "CANCEL_CRUISE_UNCERTAIN", "GAS_RESUME_UNCERTAIN", "FOLLOW_DISTANCE",
  "NEW_SIGNAL_1", "PCW", "CRUISE_STATE", "GAS_OVERRIDE", "AEB_RELATED", "SET_SPEED",
  "HANDS_ON_WHEEL_STEER",
)
# Tiggo 2022-24: STEER_STATUS (0x307) fields copied from cam onto the PT spoof.
_CAM_STEER_STATUS_FIELDS = ("LKAS_AVAILABLE", "LKAS_AVAILABLE_2", "UNKNOWN", "ADAS_STATE")
_CAM_STEER_STATUS_DEFAULT = {"LKAS_AVAILABLE": 1, "LKAS_AVAILABLE_2": 0, "UNKNOWN": 0, "ADAS_STATE": 1}


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
    self.cam_steer_status = dict(_CAM_STEER_STATUS_DEFAULT)
    # Live EPS snapshot — used by CarController to rebuild EPS on bus 2 byte-identical
    # to stock (panda blocks native fwd while our spoof loop is active).
    self.eps_steering_angle = 0.0
    self.eps_driver_torque = 0
    self.eps_counter = 0
    self.cruise_state = 1  # HUD CRUISE_STATE: 3=ENABLE 2=READY 1=IDLE
    # Tiggo 2022-24: TIGGO_8_ACC_ENABLE (0x3A5) flickers low at standstill while ACC still holds.
    self.tiggo22_cruise_latched = False

  def update(self, can_parsers):
    cp, cam = can_parsers[Bus.pt], can_parsers[Bus.cam]
    ret = car.CarState.new_message()
    ret.personality = -1
    omoda = self.CP.carFingerprint == CAR.CHERY_OMODA_5
    icaur = self.CP.carFingerprint == CAR.CHERY_ICAUR_03
    tiggo22 = self.CP.carFingerprint == CAR.CHERY_TIGGO_8_PRO_2022_2024
    steer_related_steer = icaur or tiggo22

    # --- Wheels / pedals / gear ---
    if icaur:
      ws_a = cp.vl["ICAUR_WHEELSPEED_A"]
      ws_b = cp.vl["ICAUR_WHEELSPEED_B"]
      # DBC signals are already m/s — do not apply KPH_TO_MS.
      self.parse_wheel_speeds(
        ret,
        ws_a["WHEEL_FL"], ws_a["WHEEL_FR"],
        ws_b["WHEEL_BL"], ws_b["WHEEL_BR"],
        unit=1.0,
      )
    else:
      self.parse_wheel_speeds(
        ret, cp.vl["WHEELSPEED_2"]["WHEEL_FL"], cp.vl["WHEELSPEED_2"]["WHEEL_FR"],
        cp.vl["WHEELSPEED_1"]["WHEEL_BL"], cp.vl["WHEELSPEED_1"]["WHEEL_BR"],
      )
    ret.vEgoCluster = ret.vEgo
    ret.standstill = ret.vEgoRaw < 0.01

    if steer_related_steer:
      steer = cp.vl["STEER_RELATED"]
      ret.steeringAngleDeg = max(-450.0, min(450.0, float(steer["STEERING_ANGLE"])))
      ret.steeringTorque = float(steer["DRIVER_TORQUE"])
      steer_dir = 1 if ret.steeringAngleDeg >= self.prev_angle else -1
      self.prev_angle = ret.steeringAngleDeg
      ret.steeringTorqueEps = ret.steeringTorque * steer_dir
      if icaur:
        ret.steeringPressed = self.update_steering_pressed(
          abs(ret.steeringTorque) >= ICAUR_STEERING_PRESSED_TORQUE,
          ICAUR_STEERING_PRESSED_MIN_COUNT,
        )
      else:
        ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 5, 5)
      self.eps_steering_angle = ret.steeringAngleDeg
      self.eps_driver_torque = int(steer["DRIVER_TORQUE"])
      self.eps_counter = int(steer["COUNTER"])

      if icaur:
        brake = max(0.0, min(float(cp.vl["ICAUR_BRAKE"]["BRAKE_PRESSURE"]), 1.0))
        ret.brakePressed = brake >= ICAUR_BRAKE_PRESSED
        ret.brake = brake if ret.brakePressed else 0.0
        ret.gasPressed = cp.vl["ICAUR_GAS"]["GAS_PEDAL_PRESSURE"] >= ICAUR_GAS_PRESSED
        ret.gearShifter = self.parse_gear_shifter(
          ICAUR_GEAR_MAP.get(int(cp.vl["ICAUR_TRANSMISSION"]["GEAR"]))
        )
      else:
        ret.gasPressed = cp.vl["GAS"]["GAS_PEDAL_PRESSURE"] > 0.01
    else:
      eps = cp.vl["EPS"]
      ret.steeringAngleDeg = float(eps["STEERING_ANGLE"])
      ret.steeringTorque = eps["DRIVER_TORQUE"]
      self.eps_steering_angle = float(eps["STEERING_ANGLE"])
      self.eps_driver_torque = int(eps["DRIVER_TORQUE"])
      self.eps_counter = int(eps["COUNTER"])
      steer_dir = 1 if ret.steeringAngleDeg >= self.prev_angle else -1
      self.prev_angle = ret.steeringAngleDeg
      ret.steeringTorqueEps = ret.steeringTorque * steer_dir
      ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 5, 5)
      ret.gasPressed = cp.vl["GAS"]["GAS_PEDAL_PRESSURE"] > 0.01
    if omoda or tiggo22:
      raw = int(cp.vl["OMODA_BRAKE"]["BRAKE_PRESSURE"] / 0.0188679)
      if raw <= OMODA_BRAKE_PRESSURE_RAW_MAX:
        ret.brake = raw / OMODA_BRAKE_PRESSURE_RAW_MAX
        ret.brakePressed = raw > OMODA_BRAKE_PRESSURE_RAW_MIN
      else:
        ret.brake = 0.0
        ret.brakePressed = False
      ret.gearShifter = self.parse_gear_shifter(OMODA_GEAR_MAP.get(int(cp.vl["OMODA_TRANSMISSION"]["GEAR"])))
    elif not icaur:
      ret.brake = cp.vl["BRAKE_PEDAL"]["BRAKE_PRESSURE"]
      ret.brakePressed = ret.brake > 0.01
      ret.gearShifter = self.parse_gear_shifter(GEAR_MAP.get(int(cp.vl["TRANSMISSION"]["GEAR"])))

    # --- Body / stalk ---
    if icaur:
      blink = int(cp.vl["ICAUR_STALK"]["BLINKER"])
      ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(
        3, blink in ICAUR_BLINKER_LEFT, blink in ICAUR_BLINKER_RIGHT,
      )
      # Door / seatbelt / generic toggle not reverse-engineered yet.
      ret.doorOpen = False
      ret.genericToggle = False
      ret.seatbeltUnlatched = False
    else:
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

    # --- Cruise / HUD ---
    if tiggo22:
      # Tiggo 2022-24 HUD (0x387) is a different layout; use the dedicated meter (0x305).
      meter = cp.vl["TIGGO8PRO_2022_METER"]
      set_kph = float(meter["SET_SPEED"])
      distance_bar = int(meter["DISTANCE_BAR"])
      ret.personality = TIGGO_DISTANCE_BAR_TO_PERSONALITY.get(distance_bar, -1)
      acc_enable_raw = bool(cp.vl["LKA_STATUS"]["TIGGO_8_ACC_ENABLE"])
      acc_off_meter = distance_bar == 3  # raw 3 = ACC off on cluster
      pcm = cp.vl["PCM_BUTTONS"]
      icc_cancel = bool(pcm["ICC_TOGGLE"]) and not self.prev_icc
      # Hold cruise enabled through standstill ACC-enable flicker; clear on real driver/PCM cancel.
      real_disengage = ret.brakePressed or acc_off_meter or set_kph <= 0 or icc_cancel
      if acc_enable_raw:
        self.tiggo22_cruise_latched = True
      elif real_disengage:
        self.tiggo22_cruise_latched = False
      elif ret.standstill and self.tiggo22_cruise_latched and set_kph > 0 and not acc_off_meter:
        pass  # keep latched — stock ACC still holding set speed at standstill
      else:
        self.tiggo22_cruise_latched = acc_enable_raw
      cruise_enabled = self.tiggo22_cruise_latched
      self.cruise_state = 3 if cruise_enabled else 1
      ret.stockAeb = False
      ret.stockFcw = False
      ss_state = can_parsers[Bus.alt].message_states.get(0x307)
      if ss_state is not None and ss_state.first_seen_nanos:
        ss = can_parsers[Bus.alt].vl["STEER_STATUS"]
        self.cam_steer_status = {f: int(ss[f]) for f in _CAM_STEER_STATUS_FIELDS}
    else:
      hud = cp.vl["HUD"] if omoda else cam.vl["HUD"]
      self.cam_hud = {f: hud[f] for f in _CAM_HUD_FIELDS}
      ret.stockAeb = bool(hud["AEB"])
      ret.stockFcw = bool(hud["PCW"])
      set_kph = float(hud["SET_SPEED"])
      self.cruise_state = int(hud["CRUISE_STATE"])
      distance_raw = int(hud["FOLLOW_DISTANCE"])
      personality = FOLLOW_RAW_TO_PERSONALITY.get(distance_raw, -1)
      ret.personality = max(0, min(personality, 2)) if personality != -1 else -1

    ret.cruiseState.available = True
    if tiggo22:
      ret.cruiseState.enabled = cruise_enabled
    else:
      ret.cruiseState.enabled = self.cruise_state == 3
    ret.cruiseState.speed = ret.cruiseState.speedCluster = set_kph * CV.KPH_TO_MS if set_kph > 0 else 0.0
    ret.cruiseState.standstill = ret.standstill and ret.cruiseState.enabled
    ret.cruiseState.nonAdaptive = False

    # --- PCM buttons ---
    if icaur:
      self.pcm_button_counter = 0
      ret.buttonEvents = []
    else:
      pcm = cp.vl["PCM_BUTTONS"]
      self.pcm_button_counter = int(pcm["COUNTER"])
      icc, cruise_btn = bool(pcm["ICC_TOGGLE"]), bool(pcm["CRUISE_BUTTON"])
      ret.buttonEvents = (
        create_button_events(icc, self.prev_icc, {1: ButtonType.altButton2}) +
        create_button_events(cruise_btn, self.prev_cruise, {1: ButtonType.mainCruise})
      )
      self.prev_icc, self.prev_cruise = icc, cruise_btn

    # --- ADAS / LKAS state used by CarController ---
    self.lkas_info_steer_related = float(cp.vl["LKAS_INFO"]["STEER_RELATED"])
    # Jaecoo: STEER_RELATED angle raw>=36000 is a status code (decoded ≈342.7°), not road angle.
    # iCaur/Tiggo22: same bits are real degrees — rely on DRIVER_TORQUE via steeringPressed instead.
    self.steer_related_intervention = (
      False if steer_related_steer else
      float(cp.vl["STEER_RELATED"]["STEERING_ANGLE"]) >= STEER_RELATED_INTERVENTION_DEG_MIN
    )

    return ret

  @staticmethod
  def get_can_parsers(CP):
    parsers = {Bus.pt: CarState.get_can_parser(CP), Bus.cam: CarState.get_cam_can_parser(CP)}
    if CP.carFingerprint == CAR.CHERY_TIGGO_8_PRO_2022_2024:
      parsers[Bus.alt] = CarState.get_tiggo22_lk_parser(CP)
    return parsers

  @staticmethod
  def get_can_parser(CP):
    if CP.carFingerprint == CAR.CHERY_OMODA_5:
      msgs = OMODA_PT_PARSER_MSGS
    elif CP.carFingerprint == CAR.CHERY_ICAUR_03:
      msgs = ICAUR_PT_PARSER_MSGS
    elif CP.carFingerprint == CAR.CHERY_TIGGO_8_PRO_2022_2024:
      msgs = TIGGO22_PT_PARSER_MSGS
    else:
      msgs = PT_PARSER_MSGS
    return CANParser(DBC[CP.carFingerprint]["pt"], msgs, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    if CP.carFingerprint == CAR.CHERY_OMODA_5:
      msgs = OMODA_CAM_PARSER_MSGS
    elif CP.carFingerprint == CAR.CHERY_ICAUR_03:
      msgs = ICAUR_CAM_PARSER_MSGS
    elif CP.carFingerprint == CAR.CHERY_TIGGO_8_PRO_2022_2024:
      return CANParser(DBC[CP.carFingerprint]["pt"], TIGGO22_CAM_PARSER_MSGS, CANBUS.tiggo22_cam_bus)
    else:
      msgs = CAM_PARSER_MSGS
    return CANParser(DBC[CP.carFingerprint]["pt"], msgs, CANBUS.cam_bus)

  @staticmethod
  def get_tiggo22_lk_parser(CP):
    parser = CANParser(DBC[CP.carFingerprint]["pt"], TIGGO22_LK_PARSER_MSGS, CANBUS.tiggo22_lk_bus)
    for addr in TIGGO22_LK_PARSER_BYPASS:
      if addr in parser.message_states:
        parser.message_states[addr].ignore_counter = True
        parser.message_states[addr].ignore_checksum = True
    return parser

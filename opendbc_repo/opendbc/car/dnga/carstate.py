from cereal import car
import math
from opendbc.can import CANDefine, CANParser
import numpy as np
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import DT_CTRL, Bus, create_button_events
from opendbc.car.interfaces import CarStateBase
from opendbc.car.dnga.values import DBC, CANBUS, HYBRID_CAR

HUD_MULTIPLIER = 1.04

ButtonType = car.CarState.ButtonEvent.Type
SEC_HOLD_TO_STEP_SPEED = 0.6
SHORT_PRESS_SEC = 1.0
CRUISE_MIN_KPH = 30
CRUISE_MAX_KPH = 140
DNGA_DISTANCE_TO_PERSONALITY = {
  2: 0,  # 1 bar
  1: 1,  # 2 bar
  0: 2,  # 3 bar
}

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["TRANSMISSION"]["GEAR"]

    self.is_cruise_latch = False
    self.cruise_speed = CRUISE_MIN_KPH * CV.KPH_TO_MS

    self.is_plus_btn_latch = False
    self.is_minus_btn_latch = False
    self.button_hold_timer = 0.0
    self.press_duration = 0.0
    self.step_period = SEC_HOLD_TO_STEP_SPEED

    self.stock_lkc_off = True
    self.stock_fcw_off = True
    self.lkas_rdy = True
    self.lkas_latch = True
    self.lkas_btn_rising_edge_seen = False
    self.stock_acc_engaged = False
    self.stock_acc_cmd = 0
    self.stock_brake_mag = 0
    self.stock_acc_set_speed = 0

    self.laneDepartWarning = 0
    self.frontDepartWarning = 0
    self.ldpSteerV = 0
    self.aebV = 0

    self.distance_button = 0
    self.distance_val = 2
    self.lkaDisabled = 0
    self.camera_bus_valid = False

  @staticmethod
  def _clip_cruise_speed(speed_ms):
    return float(np.clip(speed_ms, CRUISE_MIN_KPH * CV.KPH_TO_MS, CRUISE_MAX_KPH * CV.KPH_TO_MS))

  def _read_cruise_buttons(self, cp):
    if self.CP.carFingerprint in HYBRID_CAR:
      minus_button = bool(cp.vl["PCM_BUTTONS_HYBRID"]["SET_MINUS"])
      plus_button = bool(cp.vl["PCM_BUTTONS_HYBRID"]["RES_PLUS"])
      hybrid_cancel = bool(cp.vl["PCM_BUTTONS_HYBRID"]["CANCEL"])
    else:
      minus_button = bool(cp.vl["PCM_BUTTONS"]["SET_MINUS"])
      plus_button = bool(cp.vl["PCM_BUTTONS"]["RES_PLUS"])
      hybrid_cancel = False
    return minus_button, plus_button, hybrid_cancel

  def _handle_lkas_button_latch(self, cp):
    lkc_pressed = bool(cp.vl["BUTTONS"]["LKC_BTN"])
    if lkc_pressed and not self.lkas_btn_rising_edge_seen:
      self.lkas_btn_rising_edge_seen = True
    elif self.lkas_btn_rising_edge_seen and not lkc_pressed:
      self.lkas_latch = not self.lkas_latch
      self.lkas_btn_rising_edge_seen = False

  def _handle_cruise_speed_buttons(self, plus_button, minus_button):
    if not self.is_cruise_latch:
      return

    self.button_hold_timer += DT_CTRL

    # Plus button edge handling and hold stepping.
    if self.is_plus_btn_latch != plus_button:
      if plus_button:
        self.press_duration = 0.0
        self.button_hold_timer = 0.0
      elif self.press_duration < SHORT_PRESS_SEC:
        self.cruise_speed += CV.KPH_TO_MS
    elif plus_button:
      self.press_duration += DT_CTRL
      while self.button_hold_timer >= self.step_period:
        kph = self.cruise_speed * CV.MS_TO_KPH
        kph += 5 - (kph % 5)
        self.cruise_speed = kph * CV.KPH_TO_MS
        self.button_hold_timer -= self.step_period

    # Minus button edge handling and hold stepping.
    if self.is_minus_btn_latch != minus_button:
      if minus_button:
        self.press_duration = 0.0
        self.button_hold_timer = 0.0
      elif self.press_duration < SHORT_PRESS_SEC:
        self.cruise_speed -= CV.KPH_TO_MS
    elif minus_button:
      self.press_duration += DT_CTRL
      while self.button_hold_timer >= self.step_period:
        kph = self.cruise_speed * CV.MS_TO_KPH
        kph = max(CRUISE_MIN_KPH, ((kph // 5) - 1) * 5)
        self.cruise_speed = kph * CV.KPH_TO_MS
        self.button_hold_timer -= self.step_period

  def _handle_cruise_latch_activation(self, plus_button, minus_button, current_speed_cluster):
    if self.is_cruise_latch:
      return

    if self.is_plus_btn_latch and not plus_button:
      self.is_cruise_latch = True
    elif self.is_minus_btn_latch and not minus_button:
      self.cruise_speed = max(CRUISE_MIN_KPH * CV.KPH_TO_MS, current_speed_cluster)
      self.is_cruise_latch = True

  def update(self, can_parsers):
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    self.camera_bus_valid = cp_cam.can_valid
    ret = car.CarState.new_message()
    ret.personality = -1

    self.lkaDisabled = not self.lkas_latch

    # Rear wheel speed scales poorly on these vehicles, so use front signal.
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
    )
    ret.standstill = ret.vEgoRaw < 0.01

    can_gear = int(cp.vl["TRANSMISSION"]["GEAR"])

    ret.doorOpen = any(
      [
        cp.vl["METER_CLUSTER"]["MAIN_DOOR"],
        cp.vl["METER_CLUSTER"]["LEFT_FRONT_DOOR"],
        cp.vl["METER_CLUSTER"]["RIGHT_BACK_DOOR"],
        cp.vl["METER_CLUSTER"]["LEFT_BACK_DOOR"],
      ]
    )

    ret.seatbeltUnlatched = (
      cp.vl["METER_CLUSTER"]["SEAT_BELT_WARNING"] == 1 or cp.vl["METER_CLUSTER"]["SEAT_BELT_WARNING2"] == 1
    )
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    if ret.doorOpen or ret.seatbeltUnlatched:
      self.is_cruise_latch = False

    ret.gasPressed = not bool(cp.vl["GAS_PEDAL_2"]["GAS_PEDAL_STEP"])

    if self.CP.carFingerprint in HYBRID_CAR:
      ret.gasPressed |= not bool(cp.vl["PCM_BUTTONS_HYBRID"]["GAS_PRESSED"])

    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"]
    ret.brakePressed = bool(cp.vl["BRAKE"]["BRAKE_ENGAGED"])

    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]["STEER_ANGLE"]
    ret.steeringTorque = cp.vl["STEERING_MODULE"]["MAIN_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_SHAFT_TORQUE"]["STEERING_TORQUE"]
    ret.steeringPressed = bool(abs(ret.steeringTorqueEps) > 35) # noise floor average at 40

    ret.vEgoCluster = cp.vl["BUTTONS"]["UI_SPEED"] * CV.KPH_TO_MS * HUD_MULTIPLIER

    if cp_cam.can_valid:
      self.frontDepartWarning = bool(cp_cam.vl["LKAS_HUD"]["FRONT_DEPART"])
      self.laneDepartWarning = bool(cp_cam.vl["LKAS_HUD"]["LDA_ALERT"])
      self.ldpSteerV = cp_cam.vl["STEERING_LKAS"]["STEER_CMD"]
      self.aebV = cp_cam.vl["ACC_BRAKE"]['AEB_1019']

      ret.stockAeb = bool(cp_cam.vl["LKAS_HUD"]["AEB_BRAKE"])
      ret.stockFcw = bool(cp_cam.vl["LKAS_HUD"]["AEB_ALARM"])
      self.stock_lkc_off = bool(cp_cam.vl["LKAS_HUD"]["LDA_OFF"])
      self.lkas_rdy = bool(cp_cam.vl["LKAS_HUD"]["LKAS_SET"])
      self.stock_fcw_off = bool(cp_cam.vl["LKAS_HUD"]["FCW_DISABLE"])

      self.stock_acc_cmd = cp_cam.vl["ACC_CMD_HUD"]["ACC_CMD"]
      self.stock_acc_engaged = self.stock_acc_cmd > 0
      self.stock_acc_set_speed = cp_cam.vl["ACC_CMD_HUD"]["SET_SPEED"]
      self.stock_brake_mag = -cp_cam.vl["ACC_BRAKE"]["MAGNITUDE"]
    ret.stockAccelCmd = float(self.stock_brake_mag)

    self._handle_lkas_button_latch(cp)

    ret.lkaDisabled = not self.lkas_latch
    if cp_cam.can_valid:
      ret.cruiseState.available = bool(cp_cam.vl["ACC_CMD_HUD"]["SET_ME_1_2"])
      self.distance_val = int(cp_cam.vl["ACC_CMD_HUD"]["FOLLOW_DISTANCE"])
      ret.personality = DNGA_DISTANCE_TO_PERSONALITY.get(self.distance_val, -1)

    prev_distance_button = self.distance_button
    self.distance_button = cp.vl["BUTTONS"]["DISTANCE_BTN"]
    ret.buttonEvents = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    minus_button, plus_button, hybrid_cancel = self._read_cruise_buttons(cp)

    self._handle_cruise_speed_buttons(plus_button, minus_button)
    self._handle_cruise_latch_activation(plus_button, minus_button, ret.vEgoCluster)

    if bool(cp.vl["PCM_BUTTONS"]["CANCEL"]) or hybrid_cancel:
      self.is_cruise_latch = False

    if ret.brakePressed:
      self.is_cruise_latch = False

    self.cruise_speed = self._clip_cruise_speed(self.cruise_speed)
    ret.cruiseState.speedCluster = self.cruise_speed
    ret.cruiseState.speed = float(ret.cruiseState.speedCluster / np.interp(ret.vEgo, [0, 140], [1.0615, 1.0170]))

    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.enabled = self.is_cruise_latch
    if not ret.cruiseState.available:
      self.is_cruise_latch = False

    self.is_plus_btn_latch = plus_button
    self.is_minus_btn_latch = minus_button

    ret.leftBlinker = bool(cp.vl["METER_CLUSTER"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["METER_CLUSTER"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"])

    ret.leftBlindspot = bool(cp.vl["BSM"]["BSM_CHIME"])
    ret.rightBlindspot = bool(cp.vl["BSM"]["BSM_CHIME"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CarState.get_can_parser(CP),
      Bus.cam: CarState.get_cam_can_parser(CP),
    }

  @staticmethod
  def get_can_parser(CP):
    signals = [
      ("WHEEL_SPEED", 50),
      ("TRANSMISSION", 30),
      ("GAS_PEDAL", 60),
      ("BRAKE", 100),
      ("RIGHT_STALK", 33),
      ("METER_CLUSTER", 15),
      ("BSM", 15),
      ("STEERING_MODULE", 100),
      ("EPS_SHAFT_TORQUE", 40),
      ("PCM_BUTTONS", 30),
      ("GAS_PEDAL_2", 60),
      ("BUTTONS", 50),
    ]

    if CP.carFingerprint in HYBRID_CAR:
      signals.append(("PCM_BUTTONS_HYBRID", 30))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      ("LKAS_HUD", 20),
      ("ACC_CMD_HUD", 20),
      ("STEERING_LKAS", 40),
      ("ACC_BRAKE", 20),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.cam_bus)

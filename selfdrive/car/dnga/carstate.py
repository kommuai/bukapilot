from cereal import car
from common.params import Params
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from openpilot.common.numpy_fast import mean, interp
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import create_button_events
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.dnga.values import DBC, HUD_MULTIPLIER, CANBUS, HYBRID_CAR
from time import time # existing infra to reuse?

ButtonType = car.CarState.ButtonEvent.Type
SEC_HOLD_TO_STEP_SPEED = 0.6

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']
    self.is_cruise_latch = False
    self.cruise_speed = 30 * CV.KPH_TO_MS
    self.cruise_speed_counter = 0

    self.is_plus_btn_latch = False
    self.is_minus_btn_latch = False
    # shared by both + and - button, since release of another button will reset this
    self.rising_edge_since = 0
    self.last_frame = time() # todo: existing infra to reuse?
    self.dt = 0

    self.stock_lkc_off = True
    self.stock_fcw_off = True
    self.lkas_rdy = True
    self.lkas_latch = True # Set LKAS for Perodua to True by default
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
    self.lkaDisabled = 0

    self.p = Params()
    self.prev_distance_val = -1

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    self.lkaDisabled = not self.lkas_latch

    # there is a backwheel speed, but it will overflow to 0 when reach 60kmh
    # perodua vehicles doesn't have a good standard for their wheelspeed scaling
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.rr, ret.wheelSpeeds.rl, ret.wheelSpeeds.fr, ret.wheelSpeeds.fl])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    # safety checks to engage
    can_gear = int(cp.vl["TRANSMISSION"]['GEAR'])

    ret.doorOpen = any([cp.vl["METER_CLUSTER"]['MAIN_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_FRONT_DOOR'],
                     cp.vl["METER_CLUSTER"]['RIGHT_BACK_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_BACK_DOOR']])

    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEAT_BELT_WARNING'] == 1 or cp.vl["METER_CLUSTER"]['SEAT_BELT_WARNING2'] == 1
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    self.is_cruise_latch = False if (ret.doorOpen or ret.seatbeltUnlatched) else self.is_cruise_latch

    # gas pedal
    ret.gas = cp.vl["GAS_PEDAL"]['APPS_1']
    ret.gasPressed = not bool(cp.vl["GAS_PEDAL_2"]['GAS_PEDAL_STEP'])

    if self.CP.carFingerprint in HYBRID_CAR:
      ret.gasPressed |= not bool(cp.vl["PCM_BUTTONS_HYBRID"]['GAS_PRESSED'])

    # brake pedal
    ret.brake = cp.vl["BRAKE"]['BRAKE_PRESSURE']
    ret.brakePressed = bool(cp.vl["BRAKE"]['BRAKE_ENGAGED'])

    # steer
    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]['STEER_ANGLE']
    ret.steeringTorque = cp.vl["STEERING_MODULE"]['MAIN_TORQUE']
    ret.steeringTorqueEps = cp.vl["EPS_SHAFT_TORQUE"]['STEERING_TORQUE']
    ret.steeringPressed = bool(abs(ret.steeringTorque) > 20)

    ret.vEgoCluster = cp.vl["BUTTONS"]["UI_SPEED"] * CV.KPH_TO_MS * HUD_MULTIPLIER
    self.frontDepartWarning = bool(cp_cam.vl["LKAS_HUD"]["FRONT_DEPART"])
    self.laneDepartWarning = bool(cp_cam.vl["LKAS_HUD"]["LDA_ALERT"])
    self.ldpSteerV = cp_cam.vl["STEERING_LKAS"]['STEER_CMD']
    self.aebV = cp_cam.vl["ACC_BRAKE"]['AEB_1019']

    ret.stockAeb = bool(cp_cam.vl["LKAS_HUD"]['AEB_BRAKE'])
    ret.stockFcw = bool(cp_cam.vl["LKAS_HUD"]['AEB_ALARM'])
    self.stock_lkc_off = bool(cp_cam.vl["LKAS_HUD"]['LDA_OFF'])
    self.lkas_rdy = bool(cp_cam.vl["LKAS_HUD"]['LKAS_SET'])
    self.stock_fcw_off = bool(cp_cam.vl["LKAS_HUD"]['FCW_DISABLE'])

    self.stock_acc_cmd = cp_cam.vl["ACC_CMD_HUD"]["ACC_CMD"] # kph
    self.stock_acc_engaged = self.stock_acc_cmd > 0
    self.stock_acc_set_speed = cp_cam.vl["ACC_CMD_HUD"]["SET_SPEED"] #kph
    self.stock_brake_mag = -1 * cp_cam.vl["ACC_BRAKE"]["MAGNITUDE"]

    # logic to engage LKC
    if bool(cp.vl["BUTTONS"]['LKC_BTN']):
      if not self.lkas_btn_rising_edge_seen:
        self.lkas_btn_rising_edge_seen = True

    if self.lkas_btn_rising_edge_seen and not bool(cp.vl["BUTTONS"]['LKC_BTN']):
      self.lkas_latch = not self.lkas_latch
      self.lkas_btn_rising_edge_seen = False

    ret.cruiseState.available = bool(cp_cam.vl["ACC_CMD_HUD"]["SET_ME_1_2"])
    self.distance_val = int(cp_cam.vl["ACC_CMD_HUD"]['FOLLOW_DISTANCE'])
    self.prev_distance_val = self.distance_val
    prev_distance_button = self.distance_button
    self.distance_button = cp.vl["BUTTONS"]["DISTANCE_BTN"]
    ret.buttonEvents = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})
    if self.distance_button or self.prev_distance_val != self.distance_val:
      personality = str(1 if self.distance_val == 1 else (2 - self.distance_val))
      self.p.put("LongitudinalPersonality", personality)
    self.prev_distance_val = self.distance_val

    if self.CP.carFingerprint in HYBRID_CAR:
      minus_button = bool(cp.vl["PCM_BUTTONS_HYBRID"]["SET_MINUS"])
      plus_button = bool(cp.vl["PCM_BUTTONS_HYBRID"]["RES_PLUS"])
    else:
      minus_button = bool(cp.vl["PCM_BUTTONS"]["SET_MINUS"])
      plus_button = bool(cp.vl["PCM_BUTTONS"]["RES_PLUS"])

    if self.is_cruise_latch:
      cur_time = time()
      self.dt += cur_time - self.last_frame
      self.last_frame = cur_time

      # Handle + button (increase speed)
      if self.is_plus_btn_latch != plus_button:
        if not plus_button:  # Released
          if cur_time - self.rising_edge_since < 1:
            self.cruise_speed += CV.KPH_TO_MS
        else:  # Pressed
          self.rising_edge_since = cur_time
          self.dt = 0
      elif plus_button:
        while self.dt >= SEC_HOLD_TO_STEP_SPEED:
          kph = self.cruise_speed * CV.MS_TO_KPH
          kph += 5 - (kph % 5)  # Step up to next multiple of 5
          self.cruise_speed = kph * CV.KPH_TO_MS
          self.dt -= SEC_HOLD_TO_STEP_SPEED

      # Handle - button (decrease speed)
      if self.is_minus_btn_latch != minus_button:
        if not minus_button:  # Released
          if cur_time - self.rising_edge_since < 1:
            self.cruise_speed -= CV.KPH_TO_MS
        else:  # Pressed
          self.rising_edge_since = cur_time
          self.dt = 0
      elif minus_button:
        while self.dt >= SEC_HOLD_TO_STEP_SPEED:
          kph = self.cruise_speed * CV.MS_TO_KPH
          kph = max(30, ((kph // 5) - 1) * 5)  # Step down to next multiple of 5
          self.cruise_speed = kph * CV.KPH_TO_MS
          self.dt -= SEC_HOLD_TO_STEP_SPEED

    # Logic to activate cruise
    if not self.is_cruise_latch:
      if self.is_plus_btn_latch and not plus_button:
        self.is_cruise_latch = True
      elif self.is_minus_btn_latch and not minus_button:
        self.cruise_speed = max(30 * CV.KPH_TO_MS, ret.vEgoCluster)
        self.is_cruise_latch = True

    self.is_plus_btn_latch = plus_button
    self.is_minus_btn_latch = minus_button

    if bool(cp.vl["PCM_BUTTONS"]["CANCEL"]):
      self.is_cruise_latch = False

    if (self.CP.carFingerprint in HYBRID_CAR):
      if bool(cp.vl["PCM_BUTTONS_HYBRID"]["CANCEL"]) :
        self.is_cruise_latch = False

    if ret.brakePressed:
      self.is_cruise_latch = False

    # set speed in range of 30 - 140kmh only
    self.cruise_speed = max(min(self.cruise_speed, 140 * CV.KPH_TO_MS), 30 * CV.KPH_TO_MS)
    ret.cruiseState.speedCluster = self.cruise_speed
    ret.cruiseState.speed = ret.cruiseState.speedCluster / interp(ret.vEgo, [0,140], [1.0615,1.0170])

    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.enabled = self.is_cruise_latch
    if not ret.cruiseState.available:
      self.is_cruise_latch = False

    # button presses
    ret.leftBlinker = bool(cp.vl["METER_CLUSTER"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["METER_CLUSTER"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"])

    # blindspot sensors
    ret.leftBlindspot = bool(cp.vl["BSM"]["BSM_CHIME"])
    ret.rightBlindspot = bool(cp.vl["BSM"]["BSM_CHIME"])

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("WHEEL_SPEED", 50),
      ("TRANSMISSION", 30),
      ("GAS_PEDAL", 60),
      ("BRAKE", 100),
      ("RIGHT_STALK", 33),
      ("METER_CLUSTER", 0),
      ("BSM", 0),
      ("STEERING_MODULE", 100),
      ("EPS_SHAFT_TORQUE", 40),
      ("PCM_BUTTONS", 30),
      ("GAS_PEDAL_2", 0),
      ("BUTTONS", 50),
    ]

    if CP.carFingerprint in HYBRID_CAR:
      signals.append(("PCM_BUTTONS_HYBRID", 30))

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("LKAS_HUD", 20),
      ("ACC_CMD_HUD", 20),
      ("STEERING_LKAS", 40),
      ("ACC_BRAKE", 20)
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, 1)

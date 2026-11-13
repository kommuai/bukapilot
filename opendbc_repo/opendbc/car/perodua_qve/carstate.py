from cereal import car
from opendbc.can import CANParser
from opendbc.car import Bus, create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.perodua_qve.values import (
  ACC_CMD_RES_BYTE3,
  ACC_CMD_SET_BYTE4,
  BLINKER_AUX_LEFT,
  BLINKER_AUX_RIGHT,
  BODY_EVENT_DOOR_CLOSE,
  BODY_EVENT_DOOR_OPEN,
  BODY_EVENT_SEATBELT_LATCH,
  BODY_EVENT_SEATBELT_UNLATCH,
  BODY_SUB_DOOR,
  BODY_SUB_SEATBELT,
  BRAKE_B3_IDLE,
  BRAKE_B3_RANGE,
  BRAKE_B5_IDLE,
  BRAKE_B5_RANGE,
  BRAKE_THRESHOLD,
  CANBUS,
  CRUISE_STATE_ENABLED,
  CRUISE_STATE_READY,
  CAM_PARSER_MSGS,
  DBC,
  DOOR_CLOSED_BYTE2,
  DOOR_CLOSED_STATE,
  DOOR_OPEN_BYTE2,
  DOOR_OPEN_STATES,
  FOLLOW_BARS_STABLE_FRAMES,
  qve_gear_shifter,
  GAS_PEDAL_SCALE,
  GAS_THRESHOLD,
  PARSER_MSGS,
  PCM_BTN_SET_MASK,
  QVE_DISTANCE_TO_PERSONALITY,
  QVE_FOLLOW_HUD_B3_TO_BARS,
  qve_follow_distance_bars,
  EPS_TORQUE_BASELINE,
  STEER_DRIVER_TORQUE_THRESHOLD,
  SEATBELT_BUCKLING,
  SEATBELT_LATCHED,
  SEATBELT_MUX_LATCHED,
  SEATBELT_MUX_UNLATCHED,
  SEATBELT_PROFILE_UNLATCHED,
  SEATBELT_UNLATCHED,
  SEATBELT_UNLATCHED_ALT,
  TURN_SIGNAL_GENERIC,
  TURN_SIGNAL_LEFT,
  TURN_SIGNAL_RIGHT,
)


ButtonType = car.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.prev_angle = 0.0
    self.seatbelt_latched = True
    self.prev_set_btn = False
    self.prev_res_btn = False
    self.follow_bars = -1
    self.follow_bars_candidate = -1
    self.follow_bars_stable = 0
    self.cam_session_counter = 0  # cam bus b2; prefer 0x80, else 0xB0
    self.cam_session_counter_valid = False

  def _update_follow_bars(self, follow_byte: int, hud_byte: int | None) -> int:
    decoded = qve_follow_distance_bars(follow_byte, hud_byte)
    if decoded == self.follow_bars_candidate:
      self.follow_bars_stable += 1
    else:
      self.follow_bars_candidate = decoded
      self.follow_bars_stable = 1

    if self.follow_bars_stable >= FOLLOW_BARS_STABLE_FRAMES:
      self.follow_bars = decoded
    return self.follow_bars

  @staticmethod
  def _blinkers_from_stalk(turn_signal: int, blinker_aux: int) -> tuple[bool, bool]:
    left = turn_signal == TURN_SIGNAL_LEFT or blinker_aux == BLINKER_AUX_LEFT
    right = turn_signal == TURN_SIGNAL_RIGHT or blinker_aux == BLINKER_AUX_RIGHT
    return left, right

  @staticmethod
  def _door_open(door_state: int, door_state_2: int, body_sub: int, body_event: int) -> bool:
    if body_event == BODY_EVENT_DOOR_OPEN:
      return True
    if body_sub == BODY_SUB_DOOR and body_event == BODY_EVENT_DOOR_CLOSE:
      return False
    if door_state in DOOR_OPEN_STATES or door_state_2 == DOOR_OPEN_BYTE2:
      return True
    if door_state == DOOR_CLOSED_STATE or door_state_2 == DOOR_CLOSED_BYTE2:
      return False
    return False

  def _update_seatbelt_state(self, body_sub: int, body_event: int) -> tuple[bool, bool]:
    latch_event = body_sub == BODY_SUB_SEATBELT and body_event == BODY_EVENT_SEATBELT_LATCH
    unlatch_event = body_sub == BODY_SUB_SEATBELT and body_event == BODY_EVENT_SEATBELT_UNLATCH
    if latch_event:
      self.seatbelt_latched = True
    elif unlatch_event:
      self.seatbelt_latched = False
    return latch_event, unlatch_event

  @staticmethod
  def _seatbelt_unlatched(
    seatbelt_driver: int,
    seatbelt_mux: int,
    seatbelt_profile: int,
    latch_event: bool,
    unlatch_event: bool,
  ) -> bool:
    if latch_event:
      return False
    if unlatch_event:
      return True
    if seatbelt_profile == SEATBELT_PROFILE_UNLATCHED:
      return True
    if seatbelt_driver in (SEATBELT_UNLATCHED, SEATBELT_UNLATCHED_ALT):
      return True
    if seatbelt_driver == SEATBELT_BUCKLING:
      return False
    if seatbelt_driver == SEATBELT_LATCHED:
      if seatbelt_mux == SEATBELT_MUX_LATCHED:
        return False
      if seatbelt_mux == SEATBELT_MUX_UNLATCHED:
        return True
    return True

  @staticmethod
  def _gas_pedal_01(cp) -> float:
    gas_lo = float(cp.vl["GAS_PEDAL"]["GAS_PEDAL_1"])
    gas_mid = float(cp.vl["GAS_PEDAL"]["GAS_PEDAL_2"])
    gas_hi = int(cp.vl["GAS_PEDAL"]["GAS_PEDAL_3"]) & 0x7F
    pedal = max(gas_lo, gas_mid, gas_hi / GAS_PEDAL_SCALE)
    return float(min(1.0, max(0.0, pedal)))

  @staticmethod
  def _brake_pedal_01(cp) -> float:
    b3 = int(cp.vl["BRAKE"]["BRAKE_PRESSURE_1"])
    b5 = int(cp.vl["BRAKE"]["BRAKE_PRESSURE_2"])
    b3_pedal = (b3 - BRAKE_B3_IDLE) / BRAKE_B3_RANGE
    b5_pedal = (b5 - BRAKE_B5_IDLE) / BRAKE_B5_RANGE
    return float(min(1.0, max(0.0, max(b3_pedal, b5_pedal))))


  @staticmethod
  def _cam_counter_if_seen(cp_cam, msg_name: str, addr: int) -> int | None:
    """Return byte2 only if this cam message has been received at least once."""
    state = cp_cam.message_states.get(addr)
    if state is None or not state.timestamps:
      return None
    try:
      return int(cp_cam.vl[msg_name]["COUNTER_B2"]) & 0xFF
    except (KeyError, TypeError, ValueError):
      return None

  def _update_cam_session_counter(self, can_parsers) -> None:
    cp_cam = can_parsers.get(Bus.cam)
    if cp_cam is None:
      return
    # Prefer 0x80 session header; fallback stock 0xB0. Skip never-seen msgs (vl defaults to 0).
    b2 = self._cam_counter_if_seen(cp_cam, "CAM_SESSION_HDR", 0x80)
    if b2 is None:
      b2 = self._cam_counter_if_seen(cp_cam, "CAM_LANE_FD", 0xB0)
    if b2 is None:
      return
    self.cam_session_counter = b2
    self.cam_session_counter_valid = True

  def update(self, can_parsers):
    cp = can_parsers[Bus.pt]
    ret = car.CarState.new_message()

    fr = float(cp.vl["WHEEL_FR"]["WHEEL_SPEED"])
    fl = float(cp.vl["WHEEL_FL"]["WHEEL_SPEED"])
    rl = float(cp.vl["WHEEL_RL"]["WHEEL_SPEED"])
    rr = float(cp.vl["WHEEL_RR"]["WHEEL_SPEED"])
    self.parse_wheel_speeds(ret, fl, fr, rl, rr)
    ret.wheelSpeeds.fl = fl * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = fr * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = rl * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = rr * CV.KPH_TO_MS
    ret.vEgoCluster = ret.vEgo
    ret.standstill = ret.vEgoRaw < 0.01

    turn_signal = int(cp.vl["STALK"]["TURN_SIGNAL"])
    blinker_aux = int(cp.vl["STALK_2"]["BLINKER_AUX"])
    ret.leftBlinker, ret.rightBlinker = self._blinkers_from_stalk(turn_signal, blinker_aux)
    ret.genericToggle = turn_signal == TURN_SIGNAL_GENERIC

    gas = self._gas_pedal_01(cp)
    ret.gasPressed = gas > GAS_THRESHOLD

    ret.brake = self._brake_pedal_01(cp)
    ret.brakePressed = ret.brake > BRAKE_THRESHOLD

    ret.steeringAngleDeg = float(cp.vl["KINEMATICS"]["STEERING_ANGLE"])
    angle_delta = ret.steeringAngleDeg - self.prev_angle
    if angle_delta > 0.1:
      steer_dir = 1.0
    elif angle_delta < -0.1:
      steer_dir = -1.0
    else:
      steer_dir = 0.0

    driver_torque = float(cp.vl["STEERING_EPS"]["DRIVER_TORQUE"])
    eps_torque = max(0.0, float(cp.vl["STEERING_EPS"]["EPS_TORQUE"]) - EPS_TORQUE_BASELINE)
    if steer_dir != 0.0:
      ret.steeringTorque = driver_torque * steer_dir
      ret.steeringTorqueEps = eps_torque * steer_dir
    else:
      ret.steeringTorque = driver_torque
      ret.steeringTorqueEps = eps_torque
    ret.steeringPressed = self.update_steering_pressed(
      driver_torque >= STEER_DRIVER_TORQUE_THRESHOLD,
      5,
    )
    self.prev_angle = ret.steeringAngleDeg

    body_sub = int(cp.vl["BODY_STATUS"]["BODY_SUB"])
    body_event = int(cp.vl["BODY_STATUS"]["BODY_EVENT"])
    ret.doorOpen = self._door_open(
      int(cp.vl["DOORS"]["DOOR_STATE"]),
      int(cp.vl["DOORS"]["DOOR_STATE_2"]),
      body_sub,
      body_event,
    )

    latch_event, unlatch_event = self._update_seatbelt_state(body_sub, body_event)
    seatbelt_driver = int(cp.vl["SEATBELTS"]["SEATBELT_DRIVER"])
    seatbelt_mux = int(cp.vl["SEATBELTS"]["SEATBELT_MUX"])
    seatbelt_profile = int(cp.vl["SEATBELTS"]["SEATBELT_PROFILE"])
    ret.seatbeltUnlatched = not self._seatbelt_unlatched(
      seatbelt_driver,
      seatbelt_mux,
      seatbelt_profile,
      latch_event,
      unlatch_event,
    )
    if seatbelt_driver == SEATBELT_LATCHED and seatbelt_mux == SEATBELT_MUX_LATCHED:
      self.seatbelt_latched = True
    elif seatbelt_profile == SEATBELT_PROFILE_UNLATCHED or seatbelt_driver in (SEATBELT_UNLATCHED, SEATBELT_UNLATCHED_ALT):
      self.seatbelt_latched = False
    elif seatbelt_driver == SEATBELT_LATCHED and seatbelt_mux == SEATBELT_MUX_UNLATCHED:
      self.seatbelt_latched = False

    gear_b0 = int(cp.vl["GEAR_SHIFTER"]["GEAR_BYTE0"])
    gear_b1 = int(cp.vl["GEAR_SHIFTER"]["GEAR_BYTE1"])
    shift_b3 = int(cp.vl["SHIFT_BUTTON"]["SHIFT_BTN_B3"])
    ret.gearShifter = self.parse_gear_shifter(qve_gear_shifter(gear_b0, gear_b1, shift_b3))
    ret.espDisabled = False
    ret.stockAeb = False
    ret.stockFcw = False

    acc_enabled = int(cp.vl["ACC_HUD"]["CRUISE_STATE_RAW"]) == CRUISE_STATE_ENABLED
    follow_raw = int(cp.vl["ADAS_STATUS"]["FOLLOW_DISTANCE"])
    hud_val = int(cp.vl["ACC_HUD_2"]["FOLLOW_DISTANCE_HUD"])
    hud_byte = hud_val if hud_val in QVE_FOLLOW_HUD_B3_TO_BARS else None
    prev_follow_bars = self.follow_bars
    follow_bars = self._update_follow_bars(follow_raw, hud_byte)
    ret.cruiseState.available = int(cp.vl["ACC_HUD"]["CRUISE_STATE_RAW"]) in (CRUISE_STATE_READY, CRUISE_STATE_ENABLED)
    ret.cruiseState.enabled = acc_enabled
    ret.cruiseState.speed = 0.0
    ret.cruiseState.speedCluster = 0.0
    ret.cruiseState.standstill = ret.standstill
    ret.cruiseState.nonAdaptive = False
    ret.personality = QVE_DISTANCE_TO_PERSONALITY.get(follow_bars, -1)

    pcm_btn_state = int(cp.vl["PCM_BUTTONS"]["PCM_BTN_STATE"])
    acc_cmd_b3 = int(cp.vl["ACC_CMD"]["ACC_CMD_BYTE3"])
    acc_cmd_b4 = int(cp.vl["ACC_CMD"]["ACC_CMD_BYTE4"])
    set_btn = (pcm_btn_state & PCM_BTN_SET_MASK) == PCM_BTN_SET_MASK or acc_cmd_b4 == ACC_CMD_SET_BYTE4
    res_btn = acc_cmd_b3 == ACC_CMD_RES_BYTE3
    gap_events = []
    if follow_bars != prev_follow_bars and follow_bars != -1:
      gap_events = (
        create_button_events(1, 0, {1: ButtonType.gapAdjustCruise})
        + create_button_events(0, 1, {1: ButtonType.gapAdjustCruise})
      )
    ret.buttonEvents = (
      create_button_events(int(set_btn), int(self.prev_set_btn), {1: ButtonType.decelCruise})
      + create_button_events(int(res_btn), int(self.prev_res_btn), {1: ButtonType.accelCruise})
      + gap_events
    )
    self.prev_set_btn = set_btn
    self.prev_res_btn = res_btn

    self._update_cam_session_counter(can_parsers)

    return ret

  @staticmethod
  def get_can_parser(CP):
    # All PT sensor parsing on bus 1 (see CANBUS.pt).
    return CANParser(DBC[CP.carFingerprint]["pt"], PARSER_MSGS, CANBUS.pt)

  @staticmethod
  def get_cam_can_parser(CP):
    return CANParser(DBC[CP.carFingerprint]["pt"], CAM_PARSER_MSGS, CANBUS.cam)

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CarState.get_can_parser(CP),
      Bus.cam: CarState.get_cam_can_parser(CP),
    }

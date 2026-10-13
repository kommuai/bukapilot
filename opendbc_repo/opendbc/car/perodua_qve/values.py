# ruff: noqa: E501
import math
from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.docs_definitions import CarDocs, CarParts, CUSTOM_CAR_PARTS, CarFootnote, Column


@dataclass
class PeroduaQvePlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("perodua_qve_pt", None))


@dataclass
class PeroduaQveCarDocs(CarDocs):
  car_parts: CarParts = field(default_factory=CUSTOM_CAR_PARTS)


class CANBUS:
  # Radar bus 0 <-> cam bus 2 split at panda; main PT bus 1 is not forwarded.
  radar = 0
  pt = 1
  cam = 2


class PeroduaQveSafetyFlags(IntFlag):
  IGNORE_IGNITION_LINE = 1


BUS2_ALWAYS_FORWARD = 0x50
BUS2_FORWARD_RANGE: tuple[int, ...] = tuple(range(0x80, 0x89))  # 0x80-0x88

# Cam-bus lane spoof TX (0xB0/0xB1). Stock camera still emits these; dual-TX on cam bus.
ENABLE_CAM_LANE_SPOOF = True
# ~15 Hz stock; control loop 100 Hz → send every N frames when enabled.
CAM_LANE_SPOOF_PERIOD = 7

# Known cam-bus IDs on bus 2 (ascending), for tests/reference. Excludes BUS2_ALWAYS_FORWARD.
BUS2_POOL: tuple[int, ...] = (
  0x51, 0x70, 0x71,
  0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88,
  0xA0,
  0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB,
  0xBC, 0xBD, 0xBE, 0xBF,
  0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC,
  0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
  0x3A2, 0x3A3,
)


def qve_bus2_split_addrs() -> tuple[list[int], list[int]]:
  """Cam bus 2 -> radar bus 0: 0x50 + 0x80-0x88 forwarded; all else blocked."""
  forward_set = set(BUS2_FORWARD_RANGE)
  blocked = [a for a in BUS2_POOL if a not in forward_set]
  return blocked, list(BUS2_FORWARD_RANGE)


# Reverse-engineered from can_print_changes captures.
TURN_SIGNAL_LEFT = 1
TURN_SIGNAL_RIGHT = 2
TURN_SIGNAL_GENERIC = 8
BLINKER_AUX_LEFT = 0x2A
BLINKER_AUX_RIGHT = 0x34
DOOR_OPEN_STATES = (0xFC, 0xFD)
DOOR_CLOSED_STATE = 0xFB
DOOR_OPEN_BYTE2 = 0x41
DOOR_CLOSED_BYTE2 = 0x40
BODY_EVENT_DOOR_OPEN = 0x48
BODY_EVENT_DOOR_CLOSE = 0x08
BODY_SUB_SEATBELT = 1
BODY_SUB_DOOR = 3
BODY_EVENT_SEATBELT_LATCH = 0x38
BODY_EVENT_SEATBELT_UNLATCH = 0x08
SEATBELT_LATCHED = 0xE1
SEATBELT_UNLATCHED = 0xF3
SEATBELT_UNLATCHED_ALT = 0xE0
SEATBELT_BUCKLING = 0xF1
SEATBELT_MUX_LATCHED = 0x89
SEATBELT_MUX_UNLATCHED = 0x88
SEATBELT_PROFILE_UNLATCHED = 0xDB

# Pedal encoding from can_print_changes captures + QVE logs.
GAS_PEDAL_SCALE = 64.0
GAS_THRESHOLD = 0.02
BRAKE_B3_IDLE = 12
BRAKE_B3_RANGE = 30.0
BRAKE_B5_IDLE = 8
BRAKE_B5_RANGE = 56.0
BRAKE_THRESHOLD = 0.02
STEER_DRIVER_TORQUE_THRESHOLD = 3
EPS_TORQUE_BASELINE = 15.0

# PRNDL on msg 689 (0x2B1): byte0 bit7=1 -> D (R uses same pattern); byte0=0x3F byte1=0xD8=P 0xF8=N.
GEAR_SHIFTER_PARK_B0 = 0x3F
GEAR_SHIFTER_NEUTRAL_B0 = 0x3F
GEAR_SHIFTER_PARK_B1 = 0xD8
GEAR_SHIFTER_NEUTRAL_B1 = 0xF8
GEAR_SHIFTER_DRIVE_B0_MASK = 0x80


GEAR_SHIFT_TARGET_R = 0x4
GEAR_SHIFT_TARGET_D = 0x5


def qve_gear_shifter(byte0: int, byte1: int, shift_b3: int = 0) -> str:
  target = (shift_b3 >> 4) & 0xF
  if byte0 & GEAR_SHIFTER_DRIVE_B0_MASK:
    return "R" if target == GEAR_SHIFT_TARGET_R else "D"
  if byte0 == GEAR_SHIFTER_PARK_B0 and byte1 == GEAR_SHIFTER_PARK_B1:
    return "P"
  if byte0 == GEAR_SHIFTER_NEUTRAL_B0 and byte1 == GEAR_SHIFTER_NEUTRAL_B1:
    return "N"
  if target == GEAR_SHIFT_TARGET_R:
    return "R"
  return "D"

# ACC / follow distance (can_print_changes cruise + gap captures).
# 434 byte2: upper nibble family encodes gap bars; lower nibble is a rolling counter.
# Steady-state uses 0x1X=1, 0x5X/0x9X=2, 0x3X=3, 0x7X/0xBX=4. Transient presses may show 0x2X/0x4X.
QVE_FOLLOW_BASE_TO_BARS = {
  0x10: 1,
  0x20: 2,
  0x30: 3,
  0x40: 4,
  0x50: 2,
  0x70: 4,
  0x90: 2,
  0x80: 4,
  0xA0: 4,
  0xB0: 4,
}
# Optional HUD confirm on 965 byte4 (sparse); used when 434 lags by one bar.
QVE_FOLLOW_HUD_B3_TO_BARS = {
  0x47: 1,
  0x48: 1,
  0x49: 2,
  0x4A: 2,
  0x4B: 3,
  0x4C: 3,
  0x4D: 4,
}
QVE_DISTANCE_TO_PERSONALITY = {
  1: 0,  # 1 bar — aggressive
  2: 1,  # 2 bar — standard
  3: 2,  # 3 bar — chill
  4: 2,  # 4 bar — chill
}
FOLLOW_BARS_STABLE_FRAMES = 3


def qve_follow_distance_bars(follow_byte: int, hud_byte: int | None = None) -> int:
  bars = QVE_FOLLOW_BASE_TO_BARS.get(follow_byte & 0xF0)
  if bars is None:
    bars = follow_byte >> 4
  if bars not in QVE_DISTANCE_TO_PERSONALITY:
    bars = -1
  if hud_byte is not None:
    hud_bars = QVE_FOLLOW_HUD_B3_TO_BARS.get(hud_byte)
    # 965 byte4 can lead ADAS_STATUS byte2 by one bar during stalk changes.
    if hud_bars is not None and hud_bars == bars + 1:
      bars = hud_bars
  return bars

# Cruise HUD on msg 190 (0xBE): byte2 state from can_print_changes captures.
CRUISE_STATE_IDLE = 0x1B
CRUISE_STATE_ENABLED = 0x2B
CRUISE_STATE_READY = 0x4B

# Cruise stalk buttons (can_print_changes SET/RES captures).
PCM_BTN_SET_MASK = 0x05
ACC_CMD_SET_BYTE4 = 0xD5
ACC_CMD_RES_BYTE3 = 0xDF

CAM_PARSER_MSGS = [
  # ignore_alive: counter sync only; must not invalidate car while spoof is gated off
  ("CAM_SESSION_HDR", math.nan),  # 0x80 — preferred session counter (byte2)
  ("CAM_LANE_FD", math.nan),      # 0xB0 — fallback counter
]

PARSER_MSGS = [
  ("IMU", 100),
  ("WHEEL_FR", 50),
  ("WHEEL_FL", 50),
  ("WHEEL_RL", 50),
  ("WHEEL_RR", 50),
  ("STALK", 10),
  ("STALK_2", 10),
  ("GAS_PEDAL", 100),
  ("BRAKE", 50),
  ("KINEMATICS", 100),
  ("STEERING_EPS", 50),
  ("DOORS", 10),
  ("BODY_STATUS", 10),
  ("SEATBELTS", 10),
  ("GEAR_SHIFTER", 5),
  ("SHIFT_BUTTON", 50),
  ("ACC_HUD", 20),
  ("ACC_CMD", 20),
  ("ADAS_STATUS", 20),
  ("ACC_HUD_2", math.nan),  # ~1 Hz; optional HUD confirm for follow distance
  ("PCM_BUTTONS", 20),
]

QVE_SUPPORT_COMMON_FIELDS = {
  "acc_low_speed": True,
  "acc_speed_range": "0 - 165",
  "acc_stop_and_go": True,
  "lkc_torque": "TBD",
  "lkc_speed_range": "0 - 165",
  "max_steering_angle": "TBD",
}

QVE_LKC_ACC_NOTE = CarFootnote(
  "Support: Perodua Smart Drive Assist (ACC + Lane Keep Control). Platform P01A Magna Steyr modular BEV.",
  Column.LONGITUDINAL,
)


class Footnote(Enum):
  LKC_ACC = QVE_LKC_ACC_NOTE


class CAR(Platforms):
  PERODUA_QVE = PeroduaQvePlatformConfig(
    [PeroduaQveCarDocs(
      "Perodua QV-E 2025-26",
      "ALL",
      footnotes=[Footnote.LKC_ACC],
      variant="All",
      kommu_supported=True,
      **QVE_SUPPORT_COMMON_FIELDS,
    )],
    # P01A: 1600 kg kerb, 2680 mm wheelbase, FWD 150 kW BEV (Perodua / Magna Steyr)
    CarSpecs(mass=1600.0, wheelbase=2.680, steerRatio=15.5),
  )


DBC = CAR.create_dbc_map()

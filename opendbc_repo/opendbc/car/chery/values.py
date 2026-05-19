import math
from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.byd.angle_rate_limit import AngleRateLimit
from opendbc.car.docs_definitions import CarDocs, CarParts, CUSTOM_CAR_PARTS, CarFootnote, Column
from opendbc.car.lateral import AngleSteeringLimits


@dataclass
class CheryPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("chery_general_pt", None))


@dataclass
class CheryCarDocs(CarDocs):
  car_parts: CarParts = field(default_factory=CUSTOM_CAR_PARTS)


class CANBUS:
  main_bus = 0   # PT / EPS — LANE_KEEP + LKAS_INFO TX
  cam_bus = 2    # stock camera — HUD, LANE_KEEP RX


# --- CarController ---
STEER_LOWPASS_ALPHA = math.exp(-2.0 * math.pi * 2.0 * 0.02)  # 2 Hz @ 50 Hz LANE_KEEP
LANE_KEEP_STEP = 2
LKAS_INFO_STEP = 5
SPOOF_DURATION_FRAMES = 50
SPOOF_CYCLE_FRAMES = 150
RESUME_BUTTON_INTERVAL_S = 0.05

# --- CarState ---
HUD_MULTIPLIER = 1.0
STEER_RELATED_INTERVENTION_RAW_MIN = 36000
FOLLOW_RAW_TO_PERSONALITY = {1: 2, 2: 2, 3: 1, 4: 0, 5: 0}  # HUD raw 1..5 bars -> OP personality
GEAR_MAP = {1: "P", 2: "R", 3: "N", 4: "D"}

PT_PARSER_MSGS = [
  ("WHEELSPEED_1", 50), ("WHEELSPEED_2", 50), ("EPS", 100), ("GAS", 100),
  ("TRANSMISSION", 100), ("BRAKE_PEDAL", 50), ("STALK", 50), ("PCM_BUTTONS", 20),
  ("ADAS_RELATED", 100), ("SPEED_RELATED", 50), ("STEER_RELATED", 100),
  ("SEATBELT_287", 50), ("SEATBELT_430", 50), ("BCM_STAT_412", math.nan),
  ("BCM_STAT_465", math.nan), ("LKAS_INFO", 50),
]
CAM_PARSER_MSGS = [("HUD", 20), ("LANE_KEEP", 50), ("ACC_UNCERTAIN", 20)]

# --- cherycan ---
LANE_KEEP_PADDING = {
  "SET_ME_XFF": 255, "SET_ME_XFC": 252, "SET_ME_XF4": 244, "SET_ME_X63": 99, "SET_ME_XF": 15,
}
SPOOF_TORQUE_MIN = 5.0
SPOOF_TORQUE_VAR_MIN = 4.0
SPOOF_TORQUE_RAMP = 3.0
SPOOF_TORQUE_MAX = 10.0
SPOOF_NEG_PROB = 0.3
SPOOF_VAR_PROB = 0.2

CHERY_SUPPORT_COMMON_FIELDS = {
  "acc_low_speed": True,
  "acc_speed_range": "0 - 150",
  "acc_stop_and_go": True,
  "lkc_torque": "TBD",
  "lkc_speed_range": "0 - 150",
  "max_steering_angle": "TBD",
}

CHERY_LKC_ACC_NOTE = CarFootnote(
  "Support: under validation on chery_general_pt.dbc (Jaecoo J7 PHEV).",
  Column.LONGITUDINAL,
)


class Footnote(Enum):
  J7_NOTE = CHERY_LKC_ACC_NOTE


class CAR(Platforms):
  CHERY_JAECOO_J7_PHEV = CheryPlatformConfig(
    [CheryCarDocs(
      "Jaecoo J7 PHEV 2024-26",
      "ALL",
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      kommu_supported=True,
      **CHERY_SUPPORT_COMMON_FIELDS,
    )],
    CarSpecs(mass=1980.0, wheelbase=2.67, steerRatio=16.0),
  )


DBC = CAR.create_dbc_map()
ACCEL_MULT = defaultdict(lambda: 1, {CAR.CHERY_JAECOO_J7_PHEV: 1})


def chery_steering_deg_sign(cp) -> float:
  """+1: OP +deg = left; matches EPS / LANE_KEEP DBC on Jaecoo J7."""
  return 1.0


def lowpass_steer_cmd(x: float, y_prev: float | None) -> float:
  if y_prev is None:
    return x
  a = STEER_LOWPASS_ALPHA
  return a * y_prev + (1.0 - a) * x


class CarControllerParams:
  STEER_ANGLE_MAX = 120.0
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0.0, 5.0, 15.0], angle_v=[6.0, 4.0, 3.0])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0.0, 5.0, 15.0], angle_v=[8.0, 6.0, 4.0])
  ANGLE_LIMITS = AngleSteeringLimits(STEER_ANGLE_MAX, ANGLE_RATE_LIMIT_UP, ANGLE_RATE_LIMIT_DOWN)

  def __init__(self, CP):
    pass

# Back-compat alias for tests / external imports
CHERY_FOLLOW_RAW_TO_PERSONALITY = FOLLOW_RAW_TO_PERSONALITY

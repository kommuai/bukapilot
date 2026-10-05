import math
from dataclasses import dataclass, field
from enum import Enum

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.byd.angle_rate_limit import AngleRateLimit
from opendbc.car.docs_definitions import CarDocs, CarParts, CUSTOM_CAR_PARTS, CarFootnote, Column
from opendbc.car.lateral import AngleSteeringLimits


@dataclass
class GeelyPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("geely_ev_general_pt", None))


@dataclass
class GeelyCarDocs(CarDocs):
  car_parts: CarParts = field(default_factory=CUSTOM_CAR_PARTS)


class CANBUS:
  main_bus = 0


STEER_LOWPASS_ALPHA = math.exp(-2.0 * math.pi * 2.0 * 0.01)  # 2 Hz @ 100 Hz LKAS
LKAS_STEP = 1  # TX every control frame (DT_CTRL = 10 ms)

STEER_RELATED_INTERVENTION_RAW_MIN = 500
BRAKE_THRESHOLD = 0.01

PARSER_MSGS = [
  ("WHEELSPEED_1", 50),
  ("WHEELSPEED_2", 50),
  ("EPS", 100),
  ("PEDALS", 10),
  ("LKAS", 100),
  ("STEER_RELATED", 100),
]

GEELY_SUPPORT_COMMON_FIELDS = {
  "acc_low_speed": True,
  "acc_speed_range": "0 - 150",
  "acc_stop_and_go": True,
  "lkc_torque": "TBD",
  "lkc_speed_range": "0 - 150",
  "max_steering_angle": "TBD",
}

GEELY_LKC_NOTE = CarFootnote(
  "Support: under validation on geely_ev_general_pt.dbc (Galaxy E5 / Proton EMAS 7).",
  Column.LONGITUDINAL,
)


class Footnote(Enum):
  GEELY_NOTE = GEELY_LKC_NOTE


class CAR(Platforms):
  GEELY_GALAXY_E5 = GeelyPlatformConfig(
    [
      GeelyCarDocs(
        "Geely Galaxy E5 2025-26",
        "ALL",
        footnotes=[Footnote.GEELY_NOTE],
        variant="All",
        kommu_supported=True,
        **GEELY_SUPPORT_COMMON_FIELDS,
      ),
      GeelyCarDocs(
        "Proton EMAS 7 2025-26",
        "ALL",
        footnotes=[Footnote.GEELY_NOTE],
        variant="All",
        kommu_supported=True,
        **GEELY_SUPPORT_COMMON_FIELDS,
      ),
    ],
    CarSpecs(mass=1900.0, wheelbase=2.75, steerRatio=15.0),
  )


DBC = CAR.create_dbc_map()


def geely_steering_deg_sign(cp) -> float:
  """+1: OP +deg = left; matches EPS / LKAS DBC."""
  del cp
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
    del CP

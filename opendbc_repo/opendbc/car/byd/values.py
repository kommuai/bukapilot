from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.byd.angle_rate_limit import AngleRateLimit
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarDocs, CarParts, CUSTOM_CAR_PARTS, CarFootnote, Column


@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("byd_general_pt", "byd_radar_fd"))


@dataclass
class BYDCarDocs(CarDocs):
  car_parts: CarParts = field(default_factory=CUSTOM_CAR_PARTS)


class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2

BYD_SUPPORT_COMMON_FIELDS = {
  "acc_low_speed": True,
  "acc_speed_range": "0 - 150",
  "acc_stop_and_go": True,
  "lkc_torque": "Very High",
  "lkc_speed_range": "0 - 150",
  "max_steering_angle": "120°",
}

BYD_LKC_ACC_INTELLIGENT_NOTE = CarFootnote(
  "Support: Lane Keep Assist + Adaptive Cruise Control. (Intelligent Cruise: yes)",
  Column.LONGITUDINAL,
)
BYD_LKC_ACC_STOCK_NOTE = CarFootnote(
  "Support: Lane Keep Assist + Adaptive Cruise Control. (Longitudinal: stock ACC)",
  Column.LONGITUDINAL,
)


class Footnote(Enum):
  # Aliases: BYD vehicles share two longitudinal note strings.
  ATTO3_NOTE = BYD_LKC_ACC_INTELLIGENT_NOTE
  M6_NOTE = BYD_LKC_ACC_INTELLIGENT_NOTE
  SEAL_NOTE = BYD_LKC_ACC_STOCK_NOTE
  SEALION7_NOTE = BYD_LKC_ACC_STOCK_NOTE


class CAR(Platforms):
  BYD_ATTO3 = BYDPlatformConfig(
    [BYDCarDocs(
      "BYD Atto 3 2023-26",
      "ALL",
      footnotes=[Footnote.ATTO3_NOTE],
      variant="All",
      kommu_supported=True,
      **BYD_SUPPORT_COMMON_FIELDS,
    )],
    CarSpecs(mass=2090.0, wheelbase=2.72, steerRatio=16.0),
  )
  BYD_M6 = BYDPlatformConfig(
    [BYDCarDocs(
      "BYD M6 2024-26",
      "ALL",
      footnotes=[Footnote.M6_NOTE],
      variant="Extended",
      kommu_supported=True,
      **BYD_SUPPORT_COMMON_FIELDS,
    )],
    CarSpecs(mass=2374.0, wheelbase=2.80, steerRatio=16.0),
  )
  BYD_SEAL = BYDPlatformConfig(
    [
      BYDCarDocs(
        "BYD Seal 2024-26",
        "ALL",
        footnotes=[Footnote.SEAL_NOTE],
        variant="All",
        kommu_supported=True,
        **BYD_SUPPORT_COMMON_FIELDS,
      ),
        BYDCarDocs(
        "BYD Dolphin 2023-26",
        "ALL",
        footnotes=[Footnote.SEAL_NOTE],
        variant="All",
        kommu_supported=True,
        **BYD_SUPPORT_COMMON_FIELDS,
      )
    ],
    CarSpecs(mass=2180.0, wheelbase=2.92, steerRatio=16.0),
  )
  BYD_SEALION7 = BYDPlatformConfig(
    [BYDCarDocs(
      "BYD Sealion 7 2024-26",
      "ALL",
      footnotes=[Footnote.SEALION7_NOTE],
      variant="All",
      kommu_supported=True,
      **BYD_SUPPORT_COMMON_FIELDS,
    )],
    CarSpecs(mass=2340.0, wheelbase=2.93, steerRatio=16.0),
  )

DBC = CAR.create_dbc_map()
ACCEL_MULT = defaultdict(
  lambda: 1,
  {
    CAR.BYD_ATTO3: 26,
    CAR.BYD_M6: 26,
    CAR.BYD_SEAL: 1,
    CAR.BYD_SEALION7: 1,
  },
)
HUD_MULTIPLIER = 1.12

class CarControllerParams:
  STEER_ANGLE_MAX = 120.0  # deg
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[6., 3., 1.])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[8., 6., 4.])
  ANGLE_LIMITS = AngleSteeringLimits(STEER_ANGLE_MAX, ANGLE_RATE_LIMIT_UP, ANGLE_RATE_LIMIT_DOWN)

  def __init__(self, CP):
    pass

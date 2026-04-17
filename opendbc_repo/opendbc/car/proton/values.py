from dataclasses import dataclass, field
from enum import Enum

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.docs_definitions import CarDocs, CarParts, CUSTOM_CAR_PARTS, CarFootnote, Column

HUD_MULTIPLIER = 1.025


@dataclass
class ProtonPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("proton_general_pt", None))


@dataclass
class ProtonCarDocs(CarDocs):
  car_parts: CarParts = field(default_factory=CUSTOM_CAR_PARTS)


class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2

PROTON_SUPPORT_COMMON_FIELDS = {
  "acc_low_speed": True,
  "acc_speed_range": "0 - 150",
  "acc_stop_and_go": True,
  "lkc_torque": "Very High",
  "lkc_speed_range": "0 - 150",
  "max_steering_angle": "120°",
}

PROTON_LKC_ACC_NOTE = CarFootnote(
  "Support: Lane Keep Assist + Adaptive Cruise Control.",
  Column.LONGITUDINAL,
)


class Footnote(Enum):
  # Aliases: all Proton vehicles share the same longitudinal support note text.
  S70_NOTE = PROTON_LKC_ACC_NOTE
  X50_NOTE = PROTON_LKC_ACC_NOTE
  X50_FL_NOTE = PROTON_LKC_ACC_NOTE
  X70_FL_NOTE = PROTON_LKC_ACC_NOTE
  X90_NOTE = PROTON_LKC_ACC_NOTE


class CAR(Platforms):
  PROTON_S70 = ProtonPlatformConfig(
    [
      ProtonCarDocs(
        "Proton S70 2023-26",
        "All",
        footnotes=[Footnote.S70_NOTE],
        variant="Flagship, Flagship X",
        kommu_supported=True,
        **PROTON_SUPPORT_COMMON_FIELDS,
      ),
      ProtonCarDocs(
        "Proton X50 FL 2025-26",
        "All",
        footnotes=[Footnote.X50_FL_NOTE],
        variant="Premium, Flagship",
        kommu_supported=True,
        **PROTON_SUPPORT_COMMON_FIELDS,
      ),
    ],
    CarSpecs(mass=1300.0, wheelbase=2.627, steerRatio=15.0),
  )
  PROTON_X50 = ProtonPlatformConfig(
    [
      ProtonCarDocs(
        "Proton X50 2020-24",
        "All",
        footnotes=[Footnote.X50_NOTE],
        variant="Flagship",
        kommu_supported=True,
        **PROTON_SUPPORT_COMMON_FIELDS,
      )
    ],
    CarSpecs(mass=1370.0, wheelbase=2.6, steerRatio=15.0),
  )
  PROTON_X70 = ProtonPlatformConfig(
    [
      ProtonCarDocs(
        "Proton X70 FL 2024-26",
        "All",
        footnotes=[Footnote.X70_FL_NOTE],
        variant="Premium, Premium X",
        kommu_supported=True,
        **PROTON_SUPPORT_COMMON_FIELDS,
      )
    ],
    CarSpecs(mass=1610.0, wheelbase=2.67, steerRatio=15.0),
  )
  PROTON_X90 = ProtonPlatformConfig(
    [
      ProtonCarDocs(
        "Proton X90 2023-25",
        "All",
        footnotes=[Footnote.X90_NOTE],
        variant="Premium, Flagship",
        kommu_supported=True,
        **PROTON_SUPPORT_COMMON_FIELDS,
      )
    ],
    CarSpecs(mass=1705.0, wheelbase=2.805, steerRatio=15.0),
  )


DBC = CAR.create_dbc_map()

class CarControllerParams:
  STEER_STEP = 1

  def __init__(self, CP):
    self.STEER_MAX = CP.lateralParams.torqueV[0]
    assert len(CP.lateralParams.torqueV) == 1

    if CP.carFingerprint == CAR.PROTON_X90:
      self.STEER_DELTA_UP = 4
      self.STEER_DELTA_DOWN = 8
    else:
      self.STEER_DELTA_UP = 15
      self.STEER_DELTA_DOWN = 35

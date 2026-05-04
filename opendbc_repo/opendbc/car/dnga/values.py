from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.docs_definitions import CarDocs, CarParts, CUSTOM_CAR_PARTS, CarFootnote, Column

@dataclass
class DNGAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("dnga_general_pt", None))

@dataclass
class DNGACarDocs(CarDocs):
  car_parts: CarParts = field(default_factory=CUSTOM_CAR_PARTS)

class CANBUS:
  main_bus = 0
  cam_bus = 2

class DNGAFlags(IntFlag):
  HYBRID = 1
  SNG = 2


class Footnote(Enum):
  NO_RADAR_REDUNDANCY = CarFootnote(
    "Vehicle without radar will use both bukapilot and stock braking as redundancy.",
    Column.LONGITUDINAL,
  )
  BRAKE_PUMP_NOTE = CarFootnote(
    "Braking at 0km/h will produce a brake pump sound which is completely normal. Vehicle without radar will use both bukapilot and stock braking as redundancy.",
    Column.LONGITUDINAL,
  )


def dnga_car(name: str, footnote: Footnote | None = None, kommu_supported: bool | None = None):
  return DNGACarDocs(
    name,
    "All",
    footnotes=[footnote] if footnote is not None else [],
    kommu_supported=kommu_supported,
  )


class CAR(Platforms):
  PERODUA_ALZA = DNGAPlatformConfig(
    [
      dnga_car("Perodua Alza 2022-26", Footnote.NO_RADAR_REDUNDANCY, kommu_supported=True),
      dnga_car("Toyota Veloz 2022-26", Footnote.NO_RADAR_REDUNDANCY, kommu_supported=True),
    ],
    CarSpecs(mass=1170.0, wheelbase=2.750, steerRatio=17.0),
    flags=DNGAFlags.SNG,
  )
  PERODUA_ATIVA = DNGAPlatformConfig(
    [
      dnga_car("Perodua Ativa 2021-26", Footnote.BRAKE_PUMP_NOTE, kommu_supported=True),
      dnga_car("Perodua Ativa Hybrid 2022", Footnote.BRAKE_PUMP_NOTE, kommu_supported=True),
      dnga_car("Toyota Raize 2021-26"),
    ],
    CarSpecs(mass=1035.0, wheelbase=2.525, steerRatio=17.0),
  )
  PERODUA_MYVI = DNGAPlatformConfig(
    [dnga_car("Perodua Myvi 2022-26", Footnote.BRAKE_PUMP_NOTE, kommu_supported=True)],
    CarSpecs(mass=1025.0, wheelbase=2.500, steerRatio=17.0),
  )
  TOYOTA_VIOS = DNGAPlatformConfig(
    [dnga_car("Toyota Vios 2023-26", Footnote.NO_RADAR_REDUNDANCY, kommu_supported=True)],
    CarSpecs(mass=1035.0, wheelbase=2.620, steerRatio=17.0),
    flags=DNGAFlags.SNG,
  )


BRAKE_SCALE = defaultdict(
  lambda: 1.0,
  {
    CAR.PERODUA_ALZA: 0.65,
    CAR.PERODUA_ATIVA: 0.85,
    CAR.PERODUA_MYVI: 0.9,
    CAR.TOYOTA_VIOS: 0.68,
  },
)

SNG_CAR = CAR.with_flags(DNGAFlags.SNG)
HYBRID_CAR = CAR.with_flags(DNGAFlags.HYBRID)
DBC = CAR.create_dbc_map()

class CarControllerParams:
  STEER_STEP = 1
  ACCEL_MIN = -3.5
  ACCEL_MAX = 1.6  # not advisable to go higher because brake may fail
  LONG_CRUISE_ACCEL_DEADZONE = 0.08
  LONG_CRUISE_DEADZONE_MIN_V_EGO = 2.5
  STEER_DRIVER_ALLOWANCE = 0
  STEER_DRIVER_FACTOR = 1
  STEER_DRIVER_MULTIPLIER = 1.5

  def __init__(self, CP):
    self.STEER_MAX = CP.lateralParams.torqueV[0]
    assert len(CP.lateralParams.torqueV) == 1
    self.STEER_DELTA_UP = 10
    self.STEER_DELTA_DOWN = 30

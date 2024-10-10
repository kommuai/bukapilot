from dataclasses import dataclass, field
from collections import defaultdict
from enum import IntFlag

from cereal import car
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarInfo

HUD_MULTIPLIER = 1.04
Ecu = car.CarParams.Ecu

@dataclass
class DNGAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('dnga_general_pt', None))

class CANBUS:
  main_bus = 0
  cam_bus = 2

class DNGAFlags(IntFlag):
  HYBRID = 1
  SNG = 2

class CAR(Platforms):
  ALZA = DNGAPlatformConfig(
    'PERODUA ALZA',
    [
      CarInfo("Perodua Alza", "All"),
      CarInfo("Toyota Veloz", "All"),
    ],
    flags=DNGAFlags.SNG,
    specs=CarSpecs(mass=1170., wheelbase=2.750, steerRatio=17.0)
  )
  ATIVA = DNGAPlatformConfig(
    "PERODUA ATIVA",
    [
      CarInfo("Perodua Ativa", "All"),
      CarInfo("Perodua Ativa Hybrid", "All"),
      CarInfo("Toyota Raize", "All"),
    ],
    specs=CarSpecs(mass=1035., wheelbase=2.525, steerRatio=17.0)
  )
  MYVI = DNGAPlatformConfig(
    "PERODUA MYVI",
    CarInfo("Perodua Myvi", "All"),
    specs=CarSpecs(mass=1025., wheelbase=2.500, steerRatio=17.4)
  )
  VIOS = DNGAPlatformConfig(
    "TOYOTA VIOS",
    CarInfo("Toyota Vios", "All"),
    flags=DNGAFlags.SNG,
    specs=CarSpecs(mass=1035., wheelbase=2.620, steerRatio=17.0)
  )

BRAKE_SCALE = defaultdict(lambda: 1, {CAR.ATIVA: 3.3, CAR.MYVI: 3.3, CAR.ALZA: 2.6, CAR.VIOS: 3.2})

SNG_CAR = CAR.with_flags(DNGAFlags.SNG)
HYBRID_CAR = CAR.with_flags(DNGAFlags.HYBRID)
CAR_INFO = CAR.create_carinfo_map()
DBC = CAR.create_dbc_map()

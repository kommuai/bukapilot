from dataclasses import dataclass, field
from collections import defaultdict

from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarInfo

HUD_MULTIPLIER = 1.07

@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('byd_general_pt', 'byd_radar_fd'))

class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2

class CAR(Platforms):
  ATTO3 = BYDPlatformConfig(
    "BYD ATTO 3",
    CarInfo("BYD Atto 3", "ALL"),
    specs=CarSpecs(mass=2090., wheelbase=2.72, steerRatio=16.0)
  )
  M6 = BYDPlatformConfig(
    "BYD M6",
    CarInfo("BYD M6", "ALL"),
    specs=CarSpecs(mass=2374., wheelbase=2.80, steerRatio=16.0)
  )
  SEAL = BYDPlatformConfig(
    "BYD SEAL",
    CarInfo("BYD Seal", "ALL"),
    specs=CarSpecs(mass=2180., wheelbase=2.92, steerRatio=16.0)
  )
  SEALION7 = BYDPlatformConfig(
    "BYD SEALION 7",
    CarInfo("BYD Sealion 7", "ALL"),
    specs=CarSpecs(mass=2340., wheelbase=2.93, steerRatio=16.0)
  )


CAR_INFO = CAR.create_carinfo_map()
DBC = CAR.create_dbc_map()
ACCEL_MULT = defaultdict(lambda: 1, {CAR.ATTO3: 25, CAR.M6: 25, CAR.SEAL: 1, CAR.SEALION7: 1})

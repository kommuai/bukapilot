# flake8: noqa

from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

class CAR:
  PERODUA_AXIA = "PERODUA AXIA 2019"

FINGERPRINTS = {
  # PERODUA AXIA ADVANCED 2019
  CAR.PERODUA_AXIA: [{
    32: 4, 33: 8, 36: 4, 37: 8, 40: 3, 71: 8, 72: 5, 73: 6, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 122: 6, 128: 5, 186: 3, 384: 4, 592: 8, 593: 8, 679: 8, 680: 8, 681: 7, 800: 8, 802: 7, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1267: 8, 1552: 4, 1584: 6, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }]
}

## NOT SURE WHAT IS THIS
STEER_THRESHOLD = 25

# CAN_ID which identifies the ecu
ECU_FINGERPRINT = {
  Ecu.fwdCamera: [679, 680, 681, 1267],   # steer torque cmd
}

DBC = {
  CAR.PERODUA_AXIA: dbc_dict('perodua_axia_advanced_2019_pt', None)
}


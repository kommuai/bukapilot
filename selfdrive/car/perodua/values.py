# flake8: noqa

from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

class CAR:
  PERODUA_AXIA= "PERODUA AXIA 2019"
  PERODUA_MYVI_AV = "PERODUA MYVI 2020 AV"
  PERODUA_BEZZA = "PERODUA BEZZA 2019"


# Note to porter, put the shortest subset at the front of FINGERPRINTS
FINGERPRINTS = {
  CAR.PERODUA_BEZZA: [{
    33: 8, 40: 3, 71: 8, 72: 5, 73: 6, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 128: 5, 384: 4, 802: 7, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1552: 4, 1584: 6, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }],
  CAR.PERODUA_AXIA: [{
    32: 4, 33: 8, 36: 4, 37: 8, 40: 3, 71: 8, 72: 5, 73: 6, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 122: 6, 128: 5, 186: 3, 384: 4, 513: 6, 592: 8, 593: 8, 679: 8, 680: 8, 681: 7, 800: 8, 802: 7, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1267: 8, 1552: 4, 1584: 6, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }],
  CAR.PERODUA_MYVI_AV: [{
    32: 4, 33: 8, 36: 4, 37: 8, 40: 3, 69: 7, 71: 8, 72: 5, 73: 6, 74: 7, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 122: 6, 128: 5, 186: 3, 355: 7, 384: 4, 592: 8, 593: 8, 679: 8, 680: 8, 681: 7, 800: 8, 802: 7, 867: 4, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1267: 8, 1552: 4, 1584: 8, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }],
}

ECU_FINGERPRINT = {
  Ecu.fwdCamera: [679, 680, 681, 1267]

}

DBC = {
  CAR.PERODUA_AXIA: dbc_dict('perodua_general_pt', None),
  CAR.PERODUA_MYVI_AV : dbc_dict('perodua_general_pt', None),
  CAR.PERODUA_BEZZA: dbc_dict('perodua_general_pt', None),
}

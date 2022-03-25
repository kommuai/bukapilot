# flake8: noqa

from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

class CAR:
  ARUZ = "PERODUA ARUZ"
  ATIVA = "PERODUA ATIVA"
  AXIA = "PERODUA AXIA"
  BEZZA = "PERODUA BEZZA"
  MYVI = "PERODUA MYVI"
  MYVI_PSD = "PERODUA MYVI PSD"

FINGERPRINTS = {
  CAR.AXIA: [{
    32: 4, 33: 8, 36: 4, 37: 8, 40: 3, 71: 8, 72: 5, 73: 6, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 122: 6, 128: 5, 186: 3, 384: 4, 513: 6, 592: 8, 593: 8, 679: 8, 680: 8, 681: 7, 800: 8, 802: 7, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1267: 8, 1552: 4, 1584: 6, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }],
  CAR.ARUZ: [{
    32: 4, 33: 8, 36: 4, 37: 8, 40: 3, 69: 7, 71: 8, 72: 5, 73: 6, 74: 7, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 122: 6, 128: 5, 186: 3, 355: 7, 384: 4, 513: 8, 592: 8, 593: 8, 679: 8, 680: 8, 681: 7, 800: 8, 802: 7, 867: 4, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1267: 8, 1552: 4, 1584: 8, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }],
  CAR.ATIVA: [{
    160: 5, 161: 8, 164: 8, 165: 4, 308: 6, 398: 8, 399: 8, 400: 8, 409: 8, 410: 8, 416: 8, 417: 7, 427: 8, 429: 8, 448: 4, 449: 8, 464: 8, 496: 5, 516: 8, 520: 6, 524: 6, 608: 8, 609: 8, 624: 8, 625: 8, 627: 8, 628: 8, 636: 6, 752: 8, 834: 8, 848: 5, 856: 8, 857: 4, 900: 4, 912: 4, 913: 4, 976: 5, 980: 8, 1012: 7, 1032: 8, 1033: 8, 1034: 8, 1088: 8, 1090: 8, 1100: 8, 1152: 8, 1160: 4, 1162: 8, 1163: 8, 1164: 8, 1168: 8, 1176: 3, 1188: 8, 1200: 3, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1248: 8, 1267: 8, 1270: 8, 1312: 8, 1408: 8, 1409: 8, 1410: 8, 1416: 8, 1417: 8, 1418: 8, 1434: 8, 1435: 8
  }],
  CAR.BEZZA: [{
    32: 4, 33: 8, 36: 4, 37: 8, 40: 3, 69: 7, 71: 8, 72: 5, 73: 6, 74: 7, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 122: 6, 128: 5, 186: 3, 355: 7, 384: 4, 513: 8, 592: 8, 593: 8, 679: 8, 680: 8, 681: 7, 800: 8, 802: 7, 867: 4, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1267: 8, 1552: 4, 1584: 6, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }],
  CAR.MYVI: [{
    32: 4, 33: 8, 36: 4, 37: 8, 40: 3, 69: 7, 71: 8, 72: 5, 73: 6, 74: 7, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 122: 6, 128: 5, 186: 3, 355: 7, 384: 4, 513: 6, 592: 8, 593: 8, 679: 8, 680: 8, 681: 7, 800: 8, 802: 7, 867: 4, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1267: 8, 1552: 4, 1584: 8, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }],
  CAR.MYVI_PSD: [{
    160: 5, 161: 8, 164: 8, 165: 4, 308: 6, 398: 8, 399: 8, 400: 8, 405: 5, 409: 8, 410: 8, 416: 8, 417: 7, 427: 8, 429: 8, 448: 4, 449: 8, 464: 8, 496: 5, 516: 8, 520: 6, 524: 6, 608: 8, 609: 8, 624: 8, 625: 8, 627: 8, 628: 8, 752: 8, 834: 8, 848: 5, 856: 8, 857: 4, 900: 4, 976: 5, 980: 8, 1012: 7, 1032: 8, 1033: 8, 1034: 8, 1088: 8, 1090: 8, 1100: 8, 1152: 8, 1160: 4, 1162: 8, 1163: 8, 1164: 8, 1168: 8, 1176: 3, 1188: 8, 1200: 3, 1204: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1248: 8, 1267: 8, 1312: 8, 1329: 8, 1408: 8, 1409: 8, 1410: 8, 1416: 8, 1417: 8, 1418: 8, 1434: 8, 1435: 8
  }],
}

ECU_FINGERPRINT = {
  # ASA Camera CAN fingerprint
  Ecu.fwdCamera: [679, 680, 681, 1267]
}

DBC = {
  CAR.ARUZ: dbc_dict('perodua_general_pt', None),
  CAR.AXIA: dbc_dict('perodua_general_pt', None),
  CAR.BEZZA: dbc_dict('perodua_general_pt', None),
  CAR.MYVI: dbc_dict('perodua_general_pt', None),
  CAR.ATIVA: dbc_dict('perodua_ativa_pt', None),
  CAR.MYVI_PSD: dbc_dict('perodua_ativa_pt', None),
}

NOT_CAN_CONTROLLED = set([CAR.ARUZ, CAR.AXIA, CAR.BEZZA, CAR.MYVI])

ACC_CAR = set([CAR.ATIVA, CAR.MYVI_PSD])

# flake8: noqa

from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

class CAR:
  PROTON_SAGA = "PROTON SAGA 2019"

FINGERPRINTS = {
  CAR.PROTON_SAGA: [{
    257: 8, 277: 8, 512: 8, 513: 8, 528: 8, 529: 8, 625: 8, 626: 8, 627: 8, 657: 8, 776: 8, 784: 8, 786: 8, 864: 8, 961: 8, 1296: 8, 1776: 8, 1791: 3, 1827: 8, 1985: 8
  }],
}

DBC = {
  CAR.PROTON_SAGA: dbc_dict('proton_general_pt',None)
}

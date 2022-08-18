# flake8: noqa

from selfdrive.car import dbc_dict
from cereal import car

from collections import defaultdict
Ecu = car.CarParams.Ecu

# Todo
HUD_MULTIPLIER = 1.035

class CAR:
  X50 = "PROTON X50"

FINGERPRINTS = {
  CAR.X50: [{
    132: 8, 133: 8, 224: 8, 249: 8, 250: 8, 275: 8, 290: 8, 291: 8, 292: 8, 293: 8, 336: 8, 496: 8, 498: 8, 536: 8, 608: 8, 621: 8, 641: 8, 644: 8, 645: 8, 646: 8, 647: 8, 648: 8, 650: 8, 652: 8, 658: 8, 672: 8, 673: 8, 674: 8, 675: 8, 676: 8, 677: 8, 679: 8, 680: 8, 681: 8, 682: 8, 683: 8, 684: 8, 685: 8, 686: 8, 687: 8, 691: 8, 692: 8, 753: 8, 754: 8, 763: 8, 764: 8, 765: 8, 880: 8, 896: 8, 912: 8, 993: 8, 994: 8, 1005: 8, 1006: 8, 1008: 8, 1009: 8, 1014: 8, 1015: 8, 1026: 8, 1031: 8, 1038: 8, 1040: 8, 1045: 8, 2015: 8, 2024: 8
  }],
}

DBC = {
  CAR.X50: dbc_dict('proton_general_pt', None),
}

# Todo
BRAKE_SCALE = defaultdict(lambda: 1, {CAR.X50: 1})
# Todo
GAS_SCALE = defaultdict(lambda: 1, {CAR.X50: 1})


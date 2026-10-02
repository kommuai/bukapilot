import os
from typing import cast

from openpilot.system.hardware.base import HardwareBase
from openpilot.system.hardware.tici.hardware import Tici
from openpilot.system.hardware.pc.hardware import Pc
from openpilot.system.hardware.ka2.hardware import Ka2

TICI = os.path.isfile('/TICI')
KA2 = os.path.isfile('/KA2')
AGNOS = os.path.isfile('/AGNOS')
PC = not TICI and not KA2


if TICI:
  HARDWARE = cast(HardwareBase, Tici())
elif KA2:
  HARDWARE = cast(HardwareBase, Ka2())
else:
  HARDWARE = cast(HardwareBase, Pc())

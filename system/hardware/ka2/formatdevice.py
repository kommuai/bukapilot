#!/usr/bin/env python3
import cereal.messaging as messaging
from cereal import log
from openpilot.common.params import Params
from openpilot.system.hardware.ka2.hardware import Ka2
from openpilot.common.swaglog import cloudlog

_ka2 = Ka2()

def main():
  sm, p = messaging.SubMaster(["pandaStates"], poll="pandaStates"), Params()
  while True:
    sm.update()
    pandaStates = sm['pandaStates']
    if sm.updated['pandaStates'] and pandaStates:
      ignition = any(ps.ignitionLine or ps.ignitionCan for ps in pandaStates if ps.pandaType != log.PandaState.PandaType.unknown)
      if not ignition:
        if (st := _ka2.sd_status()) is None or ("not inserted" not in (st_l := st.lower()) and "formatting" not in st_l):
          cloudlog.info("Formatting SD card")
          _ka2.format_sd()
          p.put_bool_nonblocking("FormatSDCard", False)
        break

if __name__ == "__main__":
  main()

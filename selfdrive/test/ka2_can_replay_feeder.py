#!/usr/bin/env python3
"""KA2 bench: publish looped rlog CAN + panda/peripheral state on msgq (pandad blocked)."""

import os
import sys

from cereal import log
import cereal.messaging as messaging
from openpilot.common.realtime import DT_CTRL, Ratekeeper, config_realtime_process
from openpilot.selfdrive.pandad import can_capnp_to_list
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from openpilot.system.hardware import HARDWARE
from openpilot.tools.lib.logreader import LogReader

KA2_QC_RLOG_URL = "http://web.kommu.ai/depot/upload/publicbox/qc_rlog.zst"


def load_route_can_msgs(route_or_segment_name: str):
  print(f"Loading CAN from {route_or_segment_name!r}...")
  lr = LogReader(route_or_segment_name)
  cp = None
  cp_bytes = None
  mbytes = []
  for m in lr:
    if m.which() == "carParams" and cp_bytes is None:
      cp = m.carParams
      cp_bytes = m.as_builder().to_bytes()
    elif m.which() == "can":
      mbytes.append(m.as_builder().to_bytes())
  if cp is None or cp_bytes is None:
    raise ValueError(f"no carParams in {route_or_segment_name!r}")
  print(f"carFingerprint: '{cp.carFingerprint}'")
  can_msgs = [m[1] for m in can_capnp_to_list(mbytes)]
  if not can_msgs:
    raise ValueError(f"no CAN in {route_or_segment_name!r}")
  print(f"loaded {len(can_msgs)} CAN batches")
  return can_msgs, cp, cp_bytes


def default_route() -> str:
  return os.environ.get("KA2_CAN_REPLAY_ROUTE", KA2_QC_RLOG_URL).strip()


def main() -> None:
  route = sys.argv[1] if len(sys.argv) > 1 else default_route()
  os.environ.setdefault("FILEREADER_CACHE", "1")
  config_realtime_process(3, 55)

  can_msgs, cp, _ = load_route_can_msgs(route)
  safety = cp.safetyConfigs[-1] if len(cp.safetyConfigs) else None
  safety_model = safety.safetyModel if safety is not None else log.CarParams.SafetyModel.noOutput
  safety_param = safety.safetyParam if safety is not None else 0

  pm = messaging.PubMaster(["can", "pandaStates", "peripheralState"])
  rk = Ratekeeper(1 / DT_CTRL, print_delay_threshold=None)
  frame = 0
  while True:
    batch = [x for x in can_msgs[frame % len(can_msgs)] if x[-1] <= 2]
    if batch:
      pm.send("can", can_list_to_can_capnp(batch))
    if frame % 10 == 0:
      ps_msg = messaging.new_message("pandaStates", 1)
      ps_msg.valid = True
      ps = ps_msg.pandaStates[0]
      ps.ignitionLine = True
      ps.ignitionCan = True
      ps.controlsAllowed = True
      ps.harnessStatus = log.PandaState.HarnessStatus.normal
      ps.pandaType = log.PandaState.PandaType.redPanda
      ps.safetyModel = safety_model
      ps.safetyParam = safety_param
      ps.alternativeExperience = cp.alternativeExperience
      ps.faultStatus = log.PandaState.FaultStatus.none
      pm.send("pandaStates", ps_msg)
    if frame % 50 == 0:
      pe_msg = messaging.new_message("peripheralState")
      pe_msg.valid = True
      pe = pe_msg.peripheralState
      pe.pandaType = log.PandaState.PandaType.redPanda
      try:
        v, c = HARDWARE.get_voltage(), HARDWARE.get_current()
        pe.voltage = int(v) if v else 12000
        pe.current = int(c) if c else 500
      except Exception:
        pe.voltage = 12000
        pe.current = 500
      pm.send("peripheralState", pe_msg)
    frame += 1
    rk.keep_time()


if __name__ == "__main__":
  main()

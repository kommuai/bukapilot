#!/usr/bin/env python3
import os
import subprocess

import cereal.messaging as messaging
from cereal import log
from common.params import Params
from openpilot.common.swaglog import cloudlog

import subprocess
import os

def format_device(device_path="/dev/mmcblk1p1"):
  try:
      # Unmount the device (ignore errors if not mounted)
      subprocess.run(
          ["sudo", "umount", device_path],
          stderr=subprocess.DEVNULL,
          check=False,
      )

      # Format the device as ext4 (automatically confirms 'y')
      subprocess.run(
          f"echo y | sudo mkfs.ext4 {device_path}",
          shell=True,
          check=True,
      )
      cloudlog.info(f"Successfully formatted {device_path} as ext4.")
  except Exception as e:
      cloudlog.warning(f"Unexpected error: {e}")

def main():
  sm = messaging.SubMaster(["pandaStates"], poll="pandaStates")
  p = Params()

  while True:
    sm.update()
    pandaStates = sm['pandaStates']
    if sm.updated['pandaStates'] and len(pandaStates) > 0:
      ignition = any(ps.ignitionLine or ps.ignitionCan for ps in pandaStates if ps.pandaType != log.PandaState.PandaType.unknown)

      if not ignition:
        cloudlog.info("Formatting")
        format_device()
        p.put("FormatSDCard", b'0')

if __name__ == "__main__":
    main()

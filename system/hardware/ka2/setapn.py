#!/usr/bin/env python3
import os, time
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE

def apply_apn(apn: str | None):
  if not apn:
    cloudlog.info("Applying automatic GSM APN")
    try:
      mcc_mnc = (HARDWARE.get_sim_info().get('mcc_mnc', '') if HARDWARE.get_sim_info() else '')
    except Exception:
      mcc_mnc = ''
    os.system(f"bash /usr/kommu/lte/wwan0-setup.sh {mcc_mnc}")
  else:
    cloudlog.info(f"Applying GSM APN override: {apn}")
    os.system("sudo ip link set wwan0 up")
    os.system(f"sudo qmicli -d /dev/cdc-wdm0 --device-open-proxy "
              f"--wds-start-network=\"apn={apn},ip-type=4\" --client-no-release-cid")
    os.system("sudo udhcpc -i wwan0")

def main():
  params, prev_apn, modem_ready, startup_delay_done = Params(), None, False, False
  while True:
    if not modem_ready:
      try:
        if (sim_info := HARDWARE.get_sim_info()) and (sim_id := sim_info.get('sim_id', '')):
          modem_ready = True
          cloudlog.info("Modem detected, preparing APN setup")
          time.sleep(5)  # short delay to avoid startup conflict with thermald
          startup_delay_done = True
      except Exception:
        pass

    if modem_ready and startup_delay_done:
      raw = params.get("GsmApn")
      apn = None
      if raw is not None and (decoded := raw.decode().strip()):
        apn = decoded

      if apn != prev_apn:
        apply_apn(apn)
        prev_apn = apn

    time.sleep(2)

if __name__ == "__main__":
  main()

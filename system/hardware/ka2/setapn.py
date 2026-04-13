#!/usr/bin/env python3
import os
import subprocess
import time
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE

WWAN_SETUP = "/usr/kommu/lte/wwan0-setup.sh"

def _run(cmd: list[str], *, timeout: float) -> subprocess.CompletedProcess[str]:
  return subprocess.run(cmd, text=True, capture_output=True, timeout=timeout, check=False)

def apply_apn(apn: str | None):
  if not apn:
    cloudlog.info("Applying automatic GSM APN")
    try:
      mcc_mnc = (HARDWARE.get_sim_info().get('mcc_mnc', '') if HARDWARE.get_sim_info() else '')
    except Exception:
      mcc_mnc = ''
    try:
      r = _run(["bash", WWAN_SETUP, mcc_mnc], timeout=60)
      if r.returncode != 0:
        cloudlog.warning(f"wwan0-setup.sh failed rc={r.returncode} stderr_tail={(r.stderr or '')[-400:]}")
    except subprocess.TimeoutExpired:
      cloudlog.error("wwan0-setup.sh timed out after 60s")
  else:
    cloudlog.info(f"Applying GSM APN override: {apn}")
    try:
      _run(["sudo", "ip", "link", "set", "wwan0", "up"], timeout=10)
      r = _run([
        "sudo", "qmicli", "-d", "/dev/cdc-wdm0", "--device-open-proxy",
        f'--wds-start-network=apn={apn},ip-type=4',
        "--client-no-release-cid",
      ], timeout=60)
      if r.returncode != 0:
        cloudlog.warning(f"qmicli start-network failed rc={r.returncode} stderr_tail={(r.stderr or '')[-400:]}")

      # bounded DHCP; do not hang forever
      r = _run(["sudo", "udhcpc", "-q", "-f", "-n", "-t", "5", "-T", "3", "-i", "wwan0"], timeout=30)
      if r.returncode != 0:
        cloudlog.warning(f"udhcpc failed rc={r.returncode} stderr_tail={(r.stderr or '')[-400:]}")
    except subprocess.TimeoutExpired:
      cloudlog.error("APN override command timed out")

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
      apn = (apnParam := params.get("GsmApn")) and apnParam.strip()
      # Avoid racing `hardwared` modem bring-up on boot. `hardwared` already runs
      # `HARDWARE.configure_modem()` once the modem is detected.
      if prev_apn is None:
        prev_apn = apn
      elif apn != prev_apn:
        apply_apn(apn)
        prev_apn = apn

    time.sleep(2)

if __name__ == "__main__":
  main()

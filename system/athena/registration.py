#!/usr/bin/env python3
import time

from openpilot.common.params import Params
from openpilot.common.spinner import Spinner
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.system.athena import runescapej
from openpilot.system.hardware import HARDWARE, PC
from openpilot.common.swaglog import cloudlog

UNREGISTERED_DONGLE_ID = "UnregisteredDevice"

def is_registered_device() -> bool:
  dongle = Params().get("DongleId")
  if dongle is None:
    return False
  dongle = dongle.decode("utf8") if isinstance(dongle, bytes) else dongle
  return dongle != UNREGISTERED_DONGLE_ID

def register(show_spinner=False) -> str | None:
  params = Params()

  def _str(val):
    if val is None:
      return None
    return val.decode("utf8") if isinstance(val, bytes) else val

  dongle_id = _str(params.get("DongleId"))
  # Only DongleId is used for "already registered"; IMEI/HardwareSerial are written after first registration
  needs_registration = dongle_id is None or dongle_id == UNREGISTERED_DONGLE_ID

  if needs_registration:
    if show_spinner:
      spinner = Spinner()
      spinner.update("registering device")

    # Block until we get the imei
    serial = HARDWARE.get_serial()
    start_time = time.monotonic()
    imei1: str | None = None
    imei2: str | None = None
    while imei1 is None and imei2 is None:
      try:
        imei1, imei2 = HARDWARE.get_imei(0), HARDWARE.get_imei(1)
      except Exception:
        cloudlog.exception("Error getting imei, trying again...")
        time.sleep(1)

      if time.monotonic() - start_time > 60 and show_spinner:
        spinner.update(f"registering device - serial: {serial}, IMEI: ({imei1}, {imei2})")

    params.put("IMEI", imei1)
    params.put("HardwareSerial", serial)

    backoff = 0
    start_time = time.monotonic()
    while True:
      try:
        cloudlog.info("registering device with backend")
        resp = runescapej.register_user(HARDWARE.get_imei(1), HARDWARE.get_serial())
        if resp is None:
          cloudlog.info("Unable to register device, registration returned None")
          dongle_id = UNREGISTERED_DONGLE_ID
        else:
          dongle_id = resp
        break
      except Exception:
        cloudlog.exception("failed to authenticate")
        backoff = min(backoff + 1, 15)
        time.sleep(backoff)

      if time.monotonic() - start_time > 60 and show_spinner:
        spinner.update(f"registering device - serial: {serial}, IMEI: ({imei1}, {imei2})")

    if show_spinner:
      spinner.close()

  if dongle_id:
    params.put("DongleId", dongle_id)
    set_offroad_alert("Offroad_UnregisteredHardware", (dongle_id == UNREGISTERED_DONGLE_ID) and not PC)
  return dongle_id


if __name__ == "__main__":
  print(register())

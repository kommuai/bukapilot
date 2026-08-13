#!/usr/bin/env python3
import time
import threading
import cereal.messaging as messaging
from cereal import car, log
from opendbc.car.car_helpers import car_name_to_platform, interfaces
from openpilot.system.hardware.ka2.hardware import Ka2
from openpilot.common.params import Params

# Rate limit interval to avoid repeated D-Bus calls that can crash
QUERY_INTERVAL = 2.0

NetworkType = log.DeviceState.NetworkType
NETWORK_TYPES = {
  NetworkType.none:   "Offline",
  NetworkType.wifi:   "Wi-Fi",
  NetworkType.cell2G: "2G",
  NetworkType.cell3G: "3G",
  NetworkType.cell4G: "4G",
}

params = Params()

class HardwareHelper:
  # Provides a self-contained cache with refresh loop to ensure values stay current
  def __init__(self):
    self._ka2 = Ka2()
    self._lock = threading.Lock()
    self._cached_network_type = NetworkType.none
    self._cached_cellular_bundle = self._default_cellular_bundle()
    self._cached_sd_status = None
    self._cached_openpilot_long = False
    self._last_sd_format_time = 0
    self._refresh_car_support()
    threading.Thread(target=self._refresh_loop, daemon=True).start()

  @staticmethod
  def _effective_car_platform() -> str | None:
    if car_name := params.get("CarName"):
      return car_name_to_platform(car_name)
    if cp_bytes := params.get("CarParams", block=False):
      try:
        if (CP := messaging.log_from_bytes(cp_bytes, car.CarParams)).carFingerprint:
          return CP.carFingerprint
      except Exception:
        pass
    return None

  def _refresh_car_support(self) -> None:
    supported = bool(
      (platform := self._effective_car_platform()) and platform in interfaces
      and interfaces[platform].get_non_essential_params(platform).openpilotLongitudinalControl
    )
    with self._lock:
      self._cached_openpilot_long = supported

  def refresh_car_support(self) -> None:
    self._refresh_car_support()

  def car_has_openpilot_long(self) -> bool:
    with self._lock:
      return self._cached_openpilot_long

  def _default_cellular_bundle(self) -> dict:
    return {
      "sim_present": False,
      "data_session_up": False,
      "modem_state": "unknown",
      "technology": None,
      "operator": None,
      "wwan_tx_bytes": -1,
      "wwan_rx_bytes": -1,
      "summary": "Unknown",
    }

  def _refresh_loop(self) -> None:
    while True:
      try:
        nt = self._ka2.get_network_type()
      except Exception:
        nt = NetworkType.none
      try:
        cb = self._ka2.get_cellular_display_status()
      except Exception:
        cb = self._default_cellular_bundle()
      try:
        sd = self._ka2.sd_status()
      except Exception:
        sd = None
      self._refresh_car_support()
      with self._lock:
        self._cached_network_type = nt
        self._cached_cellular_bundle = cb
        self._cached_sd_status = sd
      time.sleep(QUERY_INTERVAL)

  def get_network_type(self) -> str:
    with self._lock:
      return NETWORK_TYPES.get(self._cached_network_type, "Unknown")

  def get_sim_status(self) -> str:
    with self._lock:
      return self._cached_cellular_bundle.get("summary", "Unknown")

  def get_sd_status(self) -> str | None:
    with self._lock:
      return self._cached_sd_status

  def format_sd(self) -> None:
    if (now := time.monotonic()) - self._last_sd_format_time < QUERY_INTERVAL:
      return
    self._last_sd_format_time = now
    self._ka2.format_sd()

  def update_gsm_apn(self, apn: str = "") -> None:
    def worker():
      params.put("GsmApn", apn) # Wait for parameter to be put.
      self._ka2.configure_modem()
    threading.Thread(target=worker, daemon=True).start()

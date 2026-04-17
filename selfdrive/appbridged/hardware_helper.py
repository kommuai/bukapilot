#!/usr/bin/env python3
import time
import threading
from cereal import log
from openpilot.system.hardware.ka2.hardware import Ka2

QUERY_INTERVAL = 2.0

NetworkType = log.DeviceState.NetworkType
NETWORK_TYPES = {
  NetworkType.none:   "Offline",
  NetworkType.wifi:   "Wi-Fi",
  NetworkType.cell2G: "2G",
  NetworkType.cell3G: "3G",
  NetworkType.cell4G: "4G",
}

class HardwareHelper:
  # Provides a self-contained cache with refresh loop to ensure values stay current
  def __init__(self, query_interval=QUERY_INTERVAL):
    self._ka2 = Ka2()
    self._lock = threading.Lock()
    self._cached_network_type = NetworkType.none
    self._cached_cellular_bundle = self._default_cellular_bundle()
    self._cached_sd_status = None
    self._query_interval = query_interval
    threading.Thread(target=self._refresh_loop, daemon=True).start()

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
      with self._lock:
        self._cached_network_type = nt
        self._cached_cellular_bundle = cb
        self._cached_sd_status = sd
      time.sleep(self._query_interval)

  def get_network_type(self) -> str:
    with self._lock:
      return NETWORK_TYPES.get(self._cached_network_type, "Unknown")

  def get_sim_status(self) -> str:
    with self._lock:
      return self._cached_cellular_bundle.get("summary", "Unknown")

  def get_sd_status(self) -> str | None:
    with self._lock:
      return self._cached_sd_status

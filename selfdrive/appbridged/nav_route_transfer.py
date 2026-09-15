from __future__ import annotations

import base64
import binascii
from time import monotonic
from typing import Any

RATE_WINDOW_SECONDS = 15.0
MIN_TRANSFER_RATE_BYTES_PER_SECOND = 2000
ROUTE_TRANSFER_IDLE_TIMEOUT_SECONDS = 45.0
ROUTE_MEMORY_HEADROOM = 32


def _available_memory_bytes() -> int | None:
  try:
    with open("/proc/meminfo") as meminfo:
      for line in meminfo:
        if line.startswith("MemAvailable:"):
          return int(line.split()[1]) * 1024
  except (OSError, ValueError, IndexError):
    pass
  return None


def _has_memory_for_route(total_bytes: int) -> bool:
  available = _available_memory_bytes()
  return available is not None and total_bytes * ROUTE_MEMORY_HEADROOM <= available


_ID_LIMITS = {
  "requestId": 256,
  "transferId": 256,
  "sessionId": 128,
  "destKey": 64,
}


class NavRouteTransfer:
  """Bounded assembler for one device-owned navigation route transfer."""

  def __init__(self):
    self._transfer: dict[str, Any] | None = None

  def clear(self, transfer_id: str | None = None) -> None:
    if transfer_id is None or self._transfer is None or self._transfer["transferId"] == transfer_id:
      self._transfer = None

  def start(self, message: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(message, dict):
      return {"ok": False, "reason": "invalid_start"}
    strings = {key: message.get(key) for key in _ID_LIMITS}
    if any(not isinstance(value, str) or not value or len(value) > _ID_LIMITS[key]
           for key, value in strings.items()):
      return {"ok": False, "reason": "invalid_identity"}
    generation = message.get("routeGeneration")
    total_bytes = message.get("totalBytes")
    part_count = message.get("partCount")
    if (isinstance(generation, bool) or not isinstance(generation, int) or generation < 0
        or isinstance(total_bytes, bool) or not isinstance(total_bytes, int)
        or total_bytes < 1
        or isinstance(part_count, bool) or not isinstance(part_count, int)
        or not 1 <= part_count <= total_bytes):
      return {"ok": False, "reason": "invalid_limits"}
    if not _has_memory_for_route(total_bytes):
      return {"ok": False, "reason": "insufficient_memory"}

    identity = {**strings, "routeGeneration": generation, "totalBytes": total_bytes, "partCount": part_count}
    current = self._transfer
    if current and current["transferId"] == strings["transferId"]:
      if all(current[key] == value for key, value in identity.items()):
        return self.progress(duplicate=True)
      self._transfer = None
      return {"ok": False, "reason": "conflicting_start"}

    now = monotonic()
    self._transfer = {
      **identity,
      "parts": {},
      "receivedParts": 0,
      "receivedBytes": 0,
      "startedAt": now,
      "updatedAt": now,
      "rateWindowStartedAt": now,
      "rateWindowBytes": 0,
    }
    return self.progress(started=True)

  def add_part(self, message: dict[str, Any]) -> dict[str, Any]:
    current = self._transfer
    if not current:
      return {"ok": False, "reason": "no_transfer"}
    if not isinstance(message, dict) or message.get("transferId") != current["transferId"]:
      return {"ok": False, "reason": "stale_transfer"}

    for key in (*_ID_LIMITS, "routeGeneration", "totalBytes", "partCount"):
      if type(message.get(key)) is not type(current[key]) or message.get(key) != current[key]:
        self._transfer = None
        return {"ok": False, "reason": "part_metadata_mismatch"}

    part_index = message.get("partIndex")
    if (isinstance(part_index, bool) or not isinstance(part_index, int)
        or not 0 <= part_index < current["partCount"]):
      self._transfer = None
      return {"ok": False, "reason": "invalid_part_index"}

    if "routePartBase64" in message:
      encoded = message.get("routePartBase64")
      max_encoded_bytes = 4 * ((current["totalBytes"] + 2) // 3)
      if not isinstance(encoded, str) or not encoded or len(encoded) > max_encoded_bytes:
        self._transfer = None
        return {"ok": False, "reason": "invalid_part_payload"}
      try:
        payload = base64.b64decode(encoded, validate=True)
      except (ValueError, binascii.Error):
        self._transfer = None
        return {"ok": False, "reason": "invalid_part_encoding"}
      if not payload or base64.b64encode(payload).decode("ascii") != encoded:
        self._transfer = None
        return {"ok": False, "reason": "invalid_part_encoding"}
    else:
      payload = message.get("routePart")
      if isinstance(payload, (bytes, bytearray)):
        payload = bytes(payload)
      elif (isinstance(payload, list) and payload
            and len(payload) <= current["totalBytes"]
            and all(type(value) is int and 0 <= value <= 255 for value in payload)):
        payload = bytes(payload)
      else:
        self._transfer = None
        return {"ok": False, "reason": "invalid_part_payload"}
    previous = current["parts"].get(part_index)
    if previous is not None:
      if previous == payload:
        return self.progress(duplicate=True)
      self._transfer = None
      return {"ok": False, "reason": "conflicting_duplicate"}
    if current["receivedBytes"] + len(payload) > current["totalBytes"]:
      self._transfer = None
      return {"ok": False, "reason": "transfer_overflow"}

    if not _has_memory_for_route(current["totalBytes"]):
      self._transfer = None
      return {"ok": False, "reason": "insufficient_memory"}

    now = monotonic()
    current["rateWindowBytes"] += len(payload)
    elapsed = now - current["rateWindowStartedAt"]
    if elapsed >= RATE_WINDOW_SECONDS:
      rate = current["rateWindowBytes"] / elapsed
      if rate < MIN_TRANSFER_RATE_BYTES_PER_SECOND:
        self._transfer = None
        return {"ok": False, "reason": "transfer_too_slow", "rateBytesPerSecond": int(rate)}
      current["rateWindowStartedAt"] = now
      current["rateWindowBytes"] = 0
    current["parts"][part_index] = payload
    current["receivedParts"] += 1
    current["receivedBytes"] += len(payload)
    current["updatedAt"] = now
    if current["receivedParts"] != current["partCount"]:
      return self.progress()

    route_bytes = b"".join(current["parts"][index] for index in range(current["partCount"]))
    expected_bytes = current["totalBytes"]
    received_parts = current["receivedParts"]
    self._transfer = None
    if len(route_bytes) != expected_bytes:
      return {"ok": False, "reason": "total_size_mismatch"}
    return {
      "ok": True,
      "complete": True,
      "routeBytes": route_bytes,
      "receivedParts": received_parts,
      "receivedBytes": len(route_bytes),
      "totalBytes": expected_bytes,
    }

  def progress(self, **extra: Any) -> dict[str, Any]:
    current = self._transfer
    if not current:
      return {"ok": False, "reason": "no_transfer"}
    return {
      "ok": True,
      "complete": False,
      "transferId": current["transferId"],
      "startedAt": current["startedAt"],
      "updatedAt": current["updatedAt"],
      "receivedParts": current["receivedParts"],
      "partCount": current["partCount"],
      "receivedBytes": current["receivedBytes"],
      "totalBytes": current["totalBytes"],
      **extra,
    }

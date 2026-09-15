from __future__ import annotations
import json
import fcntl
import math
import os
from contextlib import contextmanager
from typing import Any

NAV_DESTINATION_KEY = "NavDestination"
NAV_INSTRUCTION_STATE_KEY = "NavInstructionState"
NAV_ROUTE_DATA_KEY = "NavRouteData"
NAV_STOP_LIST_KEY = "NavStopList"
NAV_STOP_INDEX_KEY = "NavStopIndex"
NAV_STOP_VERSION_KEY = "NavStopListVersion"
NAV_SESSION_ID_KEY = "NavSessionId"
NAV_ROUTE_REQUEST_KEY = "NavRouteRequest"
NAV_STATUS_KEY = "NavStatus"
NAV_LAST_OPERATION_KEY = "NavLastOperationId"
NAV_OPERATION_RESULT_KEY = "NavOperationResult"
NAV_ROUTE_FAILURE_KEY = "NavRouteRequestFailure"
MAX_NAV_STOPS = 10


@contextmanager
def nav_stop_list_lock():
  """Serialize stop-list read/modify/write across appbridged and navigationd."""
  fd = os.open("/tmp/kommu-nav-stop-list.lock", os.O_CREAT | os.O_RDWR, 0o600)
  try:
    fcntl.flock(fd, fcntl.LOCK_EX)
    try:
      yield
    finally:
      fcntl.flock(fd, fcntl.LOCK_UN)
  finally:
    os.close(fd)



def _coerce_int(value: Any, default: int = 0) -> int:
  try:
    return int(value)
  except (TypeError, ValueError, OverflowError):
    return default


def _coerce_float(value: Any) -> float | None:
  if isinstance(value, bool):
    return None
  try:
    parsed = float(value)
  except (OverflowError, TypeError, ValueError):
    return None
  return parsed if math.isfinite(parsed) else None


def _json_value(raw_value: Any, default: Any) -> Any:
  if isinstance(raw_value, (list, dict)):
    return raw_value
  if isinstance(raw_value, bytes):
    raw_value = raw_value.decode("utf-8", errors="replace")
  if not raw_value:
    return default
  try:
    return json.loads(raw_value)
  except (TypeError, ValueError):
    return default


def normalize_destination_payload(payload: Any) -> dict[str, Any] | None:
  if not isinstance(payload, dict):
    return None
  name = str(payload.get("place_name") or payload.get("name") or "").strip()
  latitude, longitude = _coerce_float(payload.get("latitude")), _coerce_float(payload.get("longitude"))
  if not name or latitude is None or longitude is None or not (-90 <= latitude <= 90) or not (-180 <= longitude <= 180):
    return None
  return {"name": name, "place_name": name, "latitude": latitude, "longitude": longitude}


def parse_destination_json(raw_value: str | bytes | dict[str, Any] | None) -> dict[str, Any] | None:
  if not raw_value:
    return None
  return normalize_destination_payload(_json_value(raw_value, None))


def set_destination(params: Any, destination: dict[str, Any] | None) -> bool:
  if destination is None:
    params.remove(NAV_DESTINATION_KEY)
    return True
  if not (dest := normalize_destination_payload(destination)):
    return False
  params.put(NAV_DESTINATION_KEY, json.dumps(dest))
  return True


def normalize_stop_list(value: Any) -> list[dict[str, Any]]:
  if isinstance(value, bytes):
    value = value.decode("utf-8", errors="replace")
  if isinstance(value, str):
    value = _json_value(value, [])
  if not isinstance(value, list):
    return []
  out = []
  for item in value[:MAX_NAV_STOPS]:
    if (stop := normalize_destination_payload(item)):
      stop_id = str(item.get("id") or f"{stop['longitude']:.6f},{stop['latitude']:.6f}").strip()
      if not stop_id or len(stop_id) > 256:
        continue
      stop["id"] = stop_id
      out.append(stop)
  return out


def read_stop_list(params: Any) -> list[dict[str, Any]]:
  return normalize_stop_list(params.get(NAV_STOP_LIST_KEY))


def write_stop_list(params: Any, stops: Any, *, index: int = 0) -> list[dict[str, Any]]:
  normalized = normalize_stop_list(stops)
  params.put(NAV_STOP_LIST_KEY, json.dumps(normalized))
  params.put(NAV_STOP_INDEX_KEY, str(max(0, min(_coerce_int(index), len(normalized)))))
  params.put(NAV_STOP_VERSION_KEY, str(_coerce_int(params.get(NAV_STOP_VERSION_KEY)) + 1))
  if normalized:
    set_destination(params, normalized[0])
  else:
    set_destination(params, None)
  return normalized


def apply_stop_operation(stops: Any, command: str, payload: dict[str, Any]) -> tuple[list[dict[str, Any]], bool, str, bool]:
  """Validate one phone intent against the current device-owned list."""
  current = normalize_stop_list(stops)
  if command == "navAddStop":
    incoming = payload.get("stop")
    added = normalize_stop_list([incoming]) if isinstance(incoming, dict) else []
    if not added:
      return current, False, "invalid_stop", False
    stop = added[0]
    for existing in current:
      if existing["id"] == stop["id"] or (
        abs(existing["latitude"] - stop["latitude"]) < 1e-5
        and abs(existing["longitude"] - stop["longitude"]) < 1e-5
      ):
        return current, True, "already_present", False
    if len(current) >= MAX_NAV_STOPS:
      return current, False, "stop_limit", False
    return current + [stop], True, "added", True

  if command == "navRemoveStop":
    stop_id = payload.get("stopId")
    if not isinstance(stop_id, str) or not stop_id:
      return current, False, "invalid_stop_id", False
    updated = [stop for stop in current if stop["id"] != stop_id]
    if len(updated) == len(current):
      return current, False, "stop_not_found", False
    return updated, True, "removed", True

  if command == "navReorderStops":
    stop_ids = payload.get("stopIds")
    current_ids = [stop["id"] for stop in current]
    if not isinstance(stop_ids, list) or any(not isinstance(stop_id, str) or not stop_id for stop_id in stop_ids):
      return current, False, "invalid_stop_order", False
    if len(stop_ids) != len(current_ids) or len(set(stop_ids)) != len(stop_ids) or set(stop_ids) != set(current_ids):
      return current, False, "stops_changed", False
    by_id = {stop["id"]: stop for stop in current}
    updated = [by_id[stop_id] for stop_id in stop_ids]
    changed = stop_ids != current_ids
    return updated, True, "reordered" if changed else "unchanged", changed

  if command == "navSkipStop":
    expected_id = payload.get("expectedStopId")
    if not current:
      return current, False, "no_stops", False
    if not isinstance(expected_id, str) or current[0]["id"] != expected_id:
      return current, False, "active_stop_changed", False
    return current[1:], True, "skipped", True

  if command == "navClearStops":
    return [], True, "cleared" if current else "already_empty", bool(current)

  return current, False, "unsupported_operation", False

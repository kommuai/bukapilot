from __future__ import annotations
import json
import math
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
NAV_ROUTE_FAILURE_KEY = "NavRouteRequestFailure"
MAX_NAV_STOPS = 10



def _coerce_int(value: Any, default: int = 0) -> int:
  try:
    return int(value)
  except (TypeError, ValueError, OverflowError):
    return default


def _coerce_float(value: Any) -> float | None:
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
  if not name or latitude is None or longitude is None:
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
      stop["id"] = str(item.get("id") or f"{stop['longitude']:.6f},{stop['latitude']:.6f}")
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

from __future__ import annotations
import json
import math
from typing import Any

NAV_DESTINATION_KEY = "NavDestination"
NAV_INSTRUCTION_STATE_KEY = "NavInstructionState"
NAV_ROUTE_DATA_KEY = "NavRouteData"



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

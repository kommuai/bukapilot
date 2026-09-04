from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from openpilot.common.constants import CV

DIRECTIONS = ("left", "right", "straight")
MODIFIABLE_DIRECTIONS = ("left", "right")

EARTH_MEAN_RADIUS = 6371007.2
OFF_ROUTE_SPEED_BREAKPOINTS = [0.0, 5.0, 10.0, 20.0, 40.0]
OFF_ROUTE_DISTANCE_BREAKPOINTS = [40.0, 50.0, 60.0, 80.0, 100.0]
UTURN_MODIFIER = "uturn"

LANE_DIRECTIONS = frozenset({
  "none",
  "left",
  "right",
  "straight",
  "slightLeft",
  "slightRight",
})


@dataclass(frozen=True)
class Coordinate:
  latitude: float
  longitude: float

  def __sub__(self, other: "Coordinate") -> "Coordinate":
    return Coordinate(self.latitude - other.latitude, self.longitude - other.longitude)

  def __add__(self, other: "Coordinate") -> "Coordinate":
    return Coordinate(self.latitude + other.latitude, self.longitude + other.longitude)

  def __mul__(self, scale: float) -> "Coordinate":
    return Coordinate(self.latitude * scale, self.longitude * scale)

  def dot(self, other: "Coordinate") -> float:
    return self.latitude * other.latitude + self.longitude * other.longitude

  def distance_to(self, other: "Coordinate") -> float:
    dlat = math.radians(other.latitude - self.latitude)
    dlon = math.radians(other.longitude - self.longitude)

    haversine_dlat = math.sin(dlat / 2.0) ** 2
    haversine_dlon = math.sin(dlon / 2.0) ** 2

    a = haversine_dlat + math.cos(math.radians(self.latitude)) * math.cos(math.radians(other.latitude)) * haversine_dlon
    return 2.0 * math.asin(math.sqrt(a)) * EARTH_MEAN_RADIUS


@dataclass(frozen=True)
class RouteStep:
  banner_instructions: list[dict[str, Any]]
  distance: float
  duration: float
  maneuver: str
  location: Coordinate
  cumulative_distance: float
  modifier: str
  instruction: str


@dataclass(frozen=True)
class RouteProgress:
  closest_index: int
  closest_segment_index: int
  distance_from_route: float
  current_step: RouteStep
  next_step: RouteStep | None
  current_step_index: int
  distance_to_end_of_step: float
  distance_remaining: float
  time_remaining: float
  all_maneuvers: list[dict[str, Any]]


def bearing_between_two_points(point_one: Coordinate, point_two: Coordinate) -> float:
  lat_one = math.radians(point_one.latitude)
  lat_two = math.radians(point_two.latitude)
  dlon = math.radians(point_two.longitude - point_one.longitude)

  bearing_radians = math.atan2(
    math.sin(dlon) * math.cos(lat_two),
    math.cos(lat_one) * math.sin(lat_two) - math.sin(lat_one) * math.cos(lat_two) * math.cos(dlon),
  )
  return (math.degrees(bearing_radians) + 360.0) % 360.0


def minimum_distance(a: Coordinate, b: Coordinate, p: Coordinate) -> float:
  if a.distance_to(b) < 0.01:
    return a.distance_to(p)

  ap = p - a
  ab = b - a
  t = np.clip(ap.dot(ab) / ab.dot(ab), 0.0, 1.0)
  projection = a + ab * t
  return projection.distance_to(p)

def project_onto_segment(a: Coordinate, b: Coordinate, p: Coordinate) -> tuple[float, float]:
  lat_scale = EARTH_MEAN_RADIUS * math.pi / 180.0
  ref_lat = math.radians((a.latitude + b.latitude + p.latitude) / 3.0)
  lon_scale = lat_scale * math.cos(ref_lat)

  ab_x = (b.longitude - a.longitude) * lon_scale
  ab_y = (b.latitude - a.latitude) * lat_scale
  ap_x = (p.longitude - a.longitude) * lon_scale
  ap_y = (p.latitude - a.latitude) * lat_scale

  segment_length_sq = ab_x * ab_x + ab_y * ab_y
  if segment_length_sq <= 1e-6:
    return 0.0, math.hypot(ap_x, ap_y)

  t = float(np.clip((ap_x * ab_x + ap_y * ab_y) / segment_length_sq, 0.0, 1.0))
  proj_x = ab_x * t
  proj_y = ab_y * t
  return t, math.hypot(ap_x - proj_x, ap_y - proj_y)


def string_to_direction(direction: str) -> str:
  normalized = direction or ""
  for direction_name in DIRECTIONS:
    if direction_name not in normalized:
      continue
    if "slight" in normalized and direction_name in MODIFIABLE_DIRECTIONS:
      return f"slight{direction_name.capitalize()}"
    if "sharp" in normalized and direction_name in MODIFIABLE_DIRECTIONS:
      return f"sharp{direction_name.capitalize()}"
    return direction_name

  if "uturn" in normalized or "u-turn" in normalized:
    return UTURN_MODIFIER

  return "none"


def normalize_lane_direction(direction: str) -> str:
  normalized = string_to_direction(direction)
  return normalized if normalized in LANE_DIRECTIONS else "none"


def field_valid(dat: dict[str, Any], field: str) -> bool:
  return field in dat and dat[field] is not None


def parse_banner_instructions(banners: Any, distance_to_maneuver: float = 0.0) -> dict[str, Any] | None:
  if not banners:
    return None

  instruction: dict[str, Any] = {}
  current_banner = banners[0]
  for banner in banners:
    if distance_to_maneuver < banner["distanceAlongGeometry"]:
      current_banner = banner

  instruction["showFull"] = distance_to_maneuver < current_banner["distanceAlongGeometry"]

  primary = current_banner["primary"]
  if field_valid(primary, "text"):
    instruction["maneuverPrimaryText"] = primary["text"]
  if field_valid(primary, "type"):
    instruction["maneuverType"] = primary["type"]
  if field_valid(primary, "modifier"):
    instruction["maneuverModifier"] = string_to_direction(primary["modifier"])

  if field_valid(current_banner, "secondary"):
    instruction["maneuverSecondaryText"] = current_banner["secondary"]["text"]

  if field_valid(current_banner, "sub"):
    lanes = []
    for component in current_banner["sub"]["components"]:
      if component["type"] != "lane":
        continue

      lane = {
        "active": component["active"],
        "directions": [normalize_lane_direction(direction) for direction in component["directions"]],
      }
      if field_valid(component, "active_direction"):
        lane["activeDirection"] = normalize_lane_direction(component["active_direction"])
      lanes.append(lane)
    instruction["lanes"] = lanes

  return instruction


@dataclass
class NavigationRoute:
  geometry: list[Coordinate]
  geometry_cumulative_distances: list[float]
  bearings: list[float]
  steps: list[RouteStep]
  total_distance: float
  total_duration: float

  @classmethod
  def from_mapbox_route(cls, route_data: dict[str, Any]) -> "NavigationRoute" | None:
    geometry_data = route_data.get("geometry") or []
    steps_data = route_data.get("steps") or []
    if not geometry_data or not steps_data:
      return None

    geometry = [Coordinate(float(coord["latitude"]), float(coord["longitude"])) for coord in geometry_data]
    cumulative_distances = [0.0]
    for index in range(1, len(geometry)):
      cumulative_distances.append(cumulative_distances[-1] + geometry[index - 1].distance_to(geometry[index]))

    steps: list[RouteStep] = []
    for step in steps_data:
      location = Coordinate(float(step["location"]["latitude"]), float(step["location"]["longitude"]))
      closest_index = min(range(len(geometry)), key=lambda idx: location.distance_to(geometry[idx]))
      steps.append(RouteStep(
        banner_instructions=step.get("bannerInstructions", []),
        distance=float(step["distance"]),
        duration=float(step["duration"]),
        maneuver=str(step["maneuver"]),
        location=location,
        cumulative_distance=cumulative_distances[closest_index],
        modifier=string_to_direction(str(step.get("modifier", "none"))),
        instruction=str(step.get("instruction", "")),
      ))

    bearings = [bearing_between_two_points(geometry[index], geometry[index + 1]) for index in range(len(geometry) - 1)]
    return cls(
      geometry=geometry,
      geometry_cumulative_distances=cumulative_distances,
      bearings=bearings,
      steps=steps,
      total_distance=float(route_data.get("totalDistance", 0.0)),
      total_duration=float(route_data.get("totalDuration", 0.0)),
    )

  def route_bearing_misaligned(self, closest_segment_index: int, current_bearing: float | None, v_ego: float) -> bool:
    if current_bearing is None or v_ego < 2.5 or closest_segment_index < 0 or closest_segment_index >= len(self.bearings):
      return False

    route_bearing = self.bearings[closest_segment_index]
    normalized_bearing = (current_bearing + 360.0) % 360.0
    bearing_difference = abs(normalized_bearing - route_bearing)
    return min(bearing_difference, 360.0 - bearing_difference) > 75.0

  def get_progress(self, position: Coordinate) -> RouteProgress | None:
    if not self.geometry or not self.steps:
      return None
    if len(self.geometry) == 1:
      closest_index = 0
      closest_segment_index = 0
      min_distance = position.distance_to(self.geometry[0])
      closest_cumulative = 0.0
    else:
      best_segment_index = 0
      best_distance = float("inf")
      best_t = 0.0

      for index in range(len(self.geometry) - 1):
        t, segment_distance = project_onto_segment(self.geometry[index], self.geometry[index + 1], position)
        if segment_distance < best_distance:
          best_distance = segment_distance
          best_segment_index = index
          best_t = t

      closest_segment_index = best_segment_index
      segment_start = self.geometry_cumulative_distances[closest_segment_index]
      segment_end = self.geometry_cumulative_distances[closest_segment_index + 1]
      closest_cumulative = segment_start + (segment_end - segment_start) * best_t
      min_distance = best_distance
      closest_index = min(closest_segment_index + (1 if best_t >= 0.5 else 0), len(self.geometry) - 1)

    current_step_index = max(
      (idx for idx, step in enumerate(self.steps) if step.cumulative_distance <= (closest_cumulative + 1e-3)),
      default=-1,
    )
    current_step = self.steps[current_step_index if current_step_index >= 0 else 0]
    next_step_index = current_step_index + 1
    next_step = self.steps[next_step_index] if 0 <= next_step_index < len(self.steps) else None

    distance_to_end_of_step = max(0.0, current_step.distance - (closest_cumulative - current_step.cumulative_distance))
    distance_remaining = max(0.0, self.total_distance - closest_cumulative)

    current_step_distance = max(current_step.distance, 1.0)
    remaining_current_duration = current_step.duration * min(distance_to_end_of_step / current_step_distance, 1.0)
    later_duration = sum(step.duration for step in self.steps[next_step_index:])
    time_remaining = max(0.0, remaining_current_duration + later_duration)

    all_maneuvers: list[dict[str, Any]] = []
    start_index = max(current_step_index, 0)
    end_index = min(start_index + 3, len(self.steps))
    for index in range(start_index, end_index):
      step = self.steps[index]
      maneuver_distance = distance_to_end_of_step if index == start_index else max(0.0, step.cumulative_distance - closest_cumulative)
      all_maneuvers.append({
        "distance": maneuver_distance,
        "type": step.maneuver,
        "modifier": step.modifier,
      })

    return RouteProgress(
      closest_index=closest_index,
      closest_segment_index=closest_segment_index,
      distance_from_route=min_distance,
      current_step=current_step,
      next_step=next_step,
      current_step_index=max(current_step_index, 0),
      distance_to_end_of_step=distance_to_end_of_step,
      distance_remaining=distance_remaining,
      time_remaining=time_remaining,
      all_maneuvers=all_maneuvers,
    )

  def off_route_distance_exceeded(self, progress: RouteProgress, v_ego: float) -> bool:
    distance_threshold = float(np.interp(v_ego, OFF_ROUTE_SPEED_BREAKPOINTS, OFF_ROUTE_DISTANCE_BREAKPOINTS))
    return progress.distance_from_route > distance_threshold

  def arrived(self, progress: RouteProgress, v_ego: float) -> bool:
    # Loose enough for nearby car parks: crawl/slow + ~100 m, not only full stop at 40 m.
    if v_ego >= 6.0 or not progress.all_maneuvers:
      return False
    current = progress.all_maneuvers[0]
    destination_step = current["type"] == "arrive" or progress.current_step.maneuver == "arrive" or progress.current_step.instruction.startswith("Your destination")
    if not destination_step and progress.next_step is not None:
      destination_step = progress.next_step.maneuver == "arrive" and progress.distance_to_end_of_step <= max(25.0, v_ego * 10.0)
    if destination_step and progress.distance_remaining <= 100.0:
      return True
    # Parking / side lot: slow and close even if Mapbox has not flipped to "arrive" yet.
    return progress.distance_remaining <= 70.0 and v_ego < 4.0

  def build_instruction_payload(self, progress: RouteProgress) -> dict[str, Any]:
    parsed = parse_banner_instructions(progress.current_step.banner_instructions, progress.distance_to_end_of_step) or {}
    primary_text = parsed.get("maneuverPrimaryText") or progress.current_step.instruction
    secondary_text = parsed.get("maneuverSecondaryText") or ""
    maneuver_type = parsed.get("maneuverType") or progress.current_step.maneuver
    maneuver_modifier = parsed.get("maneuverModifier") or progress.current_step.modifier
    lanes = []
    for lane in parsed.get("lanes", []):
      directions = [direction for direction in lane.get("directions", []) if direction in LANE_DIRECTIONS]
      active_direction = lane.get("activeDirection", "none")
      lanes.append({
        "directions": directions or ["none"],
        "active": bool(lane.get("active", False)),
        "activeDirection": active_direction if active_direction in LANE_DIRECTIONS else "none",
      })

    return {
      "maneuverPrimaryText": primary_text,
      "maneuverSecondaryText": secondary_text,
      "maneuverDistance": progress.distance_to_end_of_step,
      "maneuverType": maneuver_type,
      "maneuverModifier": maneuver_modifier,
      "distanceRemaining": progress.distance_remaining,
      "timeRemaining": progress.time_remaining,
      "timeRemainingTypical": progress.time_remaining,
      "lanes": lanes,
      "showFull": bool(parsed.get("showFull", True)),
      "allManeuvers": progress.all_maneuvers,
    }



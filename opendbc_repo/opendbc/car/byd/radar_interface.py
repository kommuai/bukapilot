#!/usr/bin/env python3
# BYD front radar: same framing pattern as toyota/honda/gm (single update(), CANParser, trigger frame).
# Legacy BYD radar messages expose VLEAD as absolute speed; Seal 6 now provides decoded relative speed.
# WHEEL_SPEED is still read from the PT bus for ego-speed gating and legacy radar conversion.
import math
import time

from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.structs import RadarData
from opendbc.car.byd.values import CANBUS, DBC, CAR

RADAR_START_ADDR = 0x280
RADAR_MSG_COUNT = 10
SEAL6_RADAR_START_ADDR = 0x670
SEAL6_RADAR_MSG_COUNT = 16
SEAL6_RECORDS_PER_MESSAGE = 2
RADAR_FREQ_HZ = 20
SEAL6_RADAR_TIMEOUT_NS = 250_000_000  # 5 missed frames at the nominal 20 Hz rate
BUMPER_OFFSET_M = 4.0
# Seal 6 DREL is decoded in 1/16 m/count. Replay against vision on
# 2026-11-19--10-48-02 shows a small, stable positive radar-to-vision bias.
SEAL6_DREL_VISION_BIAS_M = 0.375
# Camera-fit adjustment from the tuner; keep the DBC's decoded sensor units and
# apply this once to outgoing points before downstream camera matching.
SEAL6_YREL_CAMERA_SCALE = 2.5
SEAL6_YREL_CAMERA_OFFSET_M = 0.5
SEAL6_INVALID_DREL_MIN = 0.5
# Sensor logs reach 249.625m with the 12-bit 1/16m range; farther tracks keep
# the stricter lateral gate and eight-frame validity requirement below.
SEAL6_DREL_MAX = 250.0
SEAL6_YREL_ABS_MAX = 1.2
# Stricter gates above this dRel to limit phantom far tracks while extending range.
SEAL6_FAR_DREL_M = 100.0
SEAL6_FAR_YREL_ABS_MAX = 0.85
SEAL6_FAR_VALID_CNT_ON = 8
# VREL and AREL are the decoded radial velocity and its derivative. The
# YVREL DBC candidate is retained for analysis but is not consumed here.
SEAL6_VREL_ABS_MAX = 16.0
SEAL6_AREL_ABS_MAX = 4.0
# Low-speed stop-and-go filtering. Stable range and near-zero relative speed are
# expected for co-moving leads and are not sufficient evidence of a stale return.
SEAL6_LOW_SPEED_V_EGO_MAX = 4.0   # match radard V_EGO_STATIONARY
SEAL6_LOW_SPEED_YREL_ABS_MAX = 1.0  # keep low-speed radar points within the lead corridor
SEAL6_LOW_SPEED_SNAP_M = 1.25       # m; single-frame range jump with no vRel
SEAL6_LOW_SPEED_CLOSE_DREL = 25.0
SEAL6_LOW_SPEED_CLOSE_VALID_CNT = 10  # 0.5s @ 20Hz before publishing close tracks
SEAL6_SIDE_RETURN_YREL_MIN = 0.45
SEAL6_CENTER_RETURN_YREL_MAX = 0.25
SEAL6_SIDE_RETURN_MIN_RANGE_ADVANTAGE = 1.5
# On a stable curve, compare returns with the estimated ego path instead of
# distance from the car's current heading. The margin covers target position
# within the lane and radar lateral noise. Interface filtering still uses raw
# yRel before the output calibration.
SEAL6_STEER_STABLE_WINDOW_NS = 200_000_000
SEAL6_STEER_STABLE_MIN_SPAN_NS = 100_000_000
SEAL6_STEER_STABLE_MAX_SPREAD_DEG = 2.0
SEAL6_CURVE_MIN_STEER_ANGLE_DEG = 3.0
SEAL6_CURVE_MIN_RANGE_M = 8.0
SEAL6_CURVE_MAX_RANGE_M = 45.0
SEAL6_CURVE_PATH_YREL_MAX = 1.8

# --- Temporal filtering / hysteresis ---
# Aggressive persistence to prevent track churn and Kalman filter resets
# Higher requirements for stability reduce false track switches
CONF_ON = 0.75         # Reasonable confidence threshold
CONF_OFF = 0.50
VALID_CNT_ON = 5       # Require 5 frames to be stable (increased from 3 to prevent track churn)
# Keep tracks alive for much longer to prevent Kalman filter resets
# When tracks briefly fail filtering, keep them published to maintain continuity
# Delete tracks after 15 missed frames; keeps tracks alive ~750ms (15*50ms) when filtering fails
MISS_MAX = 15

# --- Plausibility gates ---
DREL_MIN = 0.75        # near-field radar ghosts
DREL_MAX = 200.0
# Filter side-lane vehicles to prevent phantom braking
# Balanced at 1.5m - allows legitimate vehicles while filtering obvious side-lane vehicles
# Typical lane width ~3.5m, so 1.5m = center ~43% (reasonable for lane keeping)
YREL_ABS_MAX = 1.5     # Balanced: filter obvious side-lane vehicles (>1.5m) while allowing legitimate vehicles
VREL_ABS_MAX = 60.0
AREL_ABS_MAX = 12.0


def _create_radar_can_parser(CP):
  dbc = DBC[CP.carFingerprint]
  if Bus.radar not in dbc:
    return None

  if CP.carFingerprint == CAR.BYD_SEAL6:
    msg_count = SEAL6_RADAR_MSG_COUNT
  else:
    msg_count = RADAR_MSG_COUNT

  messages = [(f"RADAR_TRACK_{i:02d}", RADAR_FREQ_HZ) for i in range(msg_count)]
  return CANParser(dbc[Bus.radar], messages, CANBUS.radar_bus)


class RadarInterface(RadarInterfaceBase):
  """BYD radar parser with confidence and persistence filtering."""

  def __init__(self, CP):
    super().__init__(CP)

    self.seal6_radar = CP.carFingerprint == CAR.BYD_SEAL6
    self.radar_msg_count = SEAL6_RADAR_MSG_COUNT if self.seal6_radar else RADAR_MSG_COUNT
    self.radar_slot_count = self.radar_msg_count * (SEAL6_RECORDS_PER_MESSAGE if self.seal6_radar else 1)
    radar_start = SEAL6_RADAR_START_ADDR if self.seal6_radar else RADAR_START_ADDR

    self.updated_messages = set()
    self.trigger_msg = radar_start + self.radar_msg_count - 1

    self.track_id = 0
    self.slot_track_id = {}     # slot -> stable trackId

    self.valid_cnt = {i: 0 for i in range(self.radar_slot_count)}
    self.miss_cnt = {i: 0 for i in range(self.radar_slot_count)}

    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP)

    # Ego speed on PT bus for vRel = v_lead_radar - v_ego (same wheel scaling as CarState)
    self.speed_cp = None
    if not CP.radarUnavailable and Bus.pt in DBC[CP.carFingerprint]:
      pt_messages = [("WHEEL_SPEED", 50)]
      if self.seal6_radar:
        pt_messages.append(("STEER_MODULE_2", 100))
      self.speed_cp = CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.main_bus)

    # Track persistence: store last known position/velocity for disappeared tracks
    # This helps match reappearing tracks to their previous track IDs
    # Key: track_id, Value: (dRel, yRel, vRel, miss_frames)
    self.track_history: dict[int, tuple[float, float, float, int]] = {}
    self.MAX_HISTORY_FRAMES = 10  # Keep history for up to 10 frames after track disappears

    # Seal6: range history is used only to reject isolated low-speed snaps.
    self.seal6_prev_drel: dict[int, float] = {}
    self.seal6_last_trigger_monotonic_ns: int | None = None
    self.seal6_last_trigger_event_ns: int | None = None
    self.seal6_freshness_expired = False
    self.seal6_steering_samples: list[tuple[int, float]] = []

  def _ego_speed_accel(self) -> tuple[float, float]:
    if self.speed_cp is None:
      return 0.0, 0.0
    vl = self.speed_cp.vl["WHEEL_SPEED"]
    fl = float(vl["WHEELSPEED_FL"])
    fr = float(vl["WHEELSPEED_FR"])
    rl = float(vl["WHEELSPEED_BL"])
    rr = float(vl["WHEELSPEED_BR"])
    v_ego = (fl + fr + rl + rr) / 4.0 * CV.KPH_TO_MS * self.CP.wheelSpeedFactor
    return v_ego, 0.0

  def _seal6_update_steering_history(self):
    if not self.seal6_radar or self.speed_cp is None:
      return
    try:
      steering_timestamp_ns = int(self.speed_cp.ts_nanos["STEER_MODULE_2"]["STEER_ANGLE_2"])
      steering_angle_deg = float(self.speed_cp.vl["STEER_MODULE_2"]["STEER_ANGLE_2"])
    except (AttributeError, KeyError, TypeError, ValueError):
      return
    if steering_timestamp_ns <= 0 or not math.isfinite(steering_angle_deg):
      return

    samples = self.seal6_steering_samples
    if samples and steering_timestamp_ns < samples[-1][0]:  # replay seek
      samples.clear()
    elif samples and steering_timestamp_ns == samples[-1][0]:
      return
    samples.append((steering_timestamp_ns, steering_angle_deg))
    self.seal6_steering_samples = [s for s in samples if steering_timestamp_ns - s[0] <= SEAL6_STEER_STABLE_WINDOW_NS]

  def _seal6_curve_path_lateral_error(self, dRel: float, yRel: float, v_ego: float) -> float | None:
    """Return radar lateral error from the steering-predicted path, when trustworthy."""
    if (not self.seal6_radar or self.speed_cp is None or v_ego < SEAL6_LOW_SPEED_V_EGO_MAX or
        not SEAL6_CURVE_MIN_RANGE_M <= dRel <= SEAL6_CURVE_MAX_RANGE_M):
      return None

    try:
      now_ns = int(self.speed_cp._last_update_nanos)
      wheelbase = float(self.CP.wheelbase)
      steer_ratio = float(self.CP.steerRatio)
    except (AttributeError, TypeError, ValueError):
      return None
    if now_ns <= 0 or wheelbase <= 0.0 or steer_ratio <= 0.0:
      return None

    samples = [(ts, angle) for ts, angle in self.seal6_steering_samples if 0 <= now_ns - ts <= SEAL6_STEER_STABLE_WINDOW_NS]
    angles = [angle for _, angle in samples]
    if (len(samples) < 3 or samples[-1][0] - samples[0][0] < SEAL6_STEER_STABLE_MIN_SPAN_NS or
        max(angles) - min(angles) > SEAL6_STEER_STABLE_MAX_SPREAD_DEG):
      return None
    steering_angle_deg = sum(angles) / len(angles)
    if abs(steering_angle_deg) < SEAL6_CURVE_MIN_STEER_ANGLE_DEG:
      return None

    # Steering-angle sign is opposite the camera/model lateral convention on
    # this platform; radar yRel has the opposite sign to model lead.y as well.
    curvature = -math.tan(math.radians(steering_angle_deg / steer_ratio)) / wheelbase
    curve_x = curvature * dRel
    if abs(curve_x) >= 0.8:
      return None

    # Exact circle projection at the radar point's longitudinal distance.
    camera_path_y = (1.0 - math.sqrt(1.0 - curve_x * curve_x)) / curvature
    expected_radar_y = -camera_path_y
    return yRel - expected_radar_y

  def _seal6_drel_plausible(self, dRel: float) -> bool:
    return SEAL6_INVALID_DREL_MIN < dRel <= SEAL6_DREL_MAX

  def _seal6_low_speed_untrusted(self, slot: int, dRel: float, yRel: float, vRel: float, v_ego: float,
                                 prev_drel: float | None) -> bool:
    """Reject isolated low-speed range snaps; stable range and relative speed are not stale evidence."""
    return (v_ego < SEAL6_LOW_SPEED_V_EGO_MAX and abs(yRel) <= 0.5 and
            DREL_MIN <= dRel <= SEAL6_LOW_SPEED_CLOSE_DREL and prev_drel is not None and
            abs(dRel - prev_drel) > SEAL6_LOW_SPEED_SNAP_M and abs(vRel) < 1.0)

  def _read_track(self, msg, v_ego, a_ego, slot: int | None = None, record_index: int = 0):
    if self.seal6_radar:
      suffix = "_2" if record_index == 1 else ""
      valid = bool(msg.get(f"VALID{suffix}", 0))
      dRel = float(msg.get(f"DREL{suffix}", 0.0)) + SEAL6_DREL_VISION_BIAS_M
      yRel = float(msg.get(f"YREL{suffix}", 0.0))
      vRel = float(msg.get(f"VREL{suffix}", 0.0))
      aRel = float(msg.get(f"AREL{suffix}", 0.0))
      meas_ok = valid and self._seal6_drel_plausible(dRel)
      conf = 0.95 if meas_ok else 0.0
      if slot is not None and meas_ok:
        self.seal6_prev_drel[slot] = dRel
      return conf, meas_ok, dRel, yRel, vRel, aRel

    long_dist = float(msg.get("LONG_DIST", 255.0))
    meas_ok = long_dist < 255.0
    conf = float(msg.get("CONFIDENCE", 0.0))
    lat_dist = float(msg.get("LAT_DIST", 0.0))
    vlead = float(msg.get("VLEAD", 0.0))
    alead = float(msg.get("ALEAD", 0.0))

    dRel = long_dist - BUMPER_OFFSET_M
    yRel = lat_dist
    vRel = vlead - v_ego
    aRel = alead - a_ego
    return conf, meas_ok, dRel, yRel, vRel, aRel

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)

    now_ns = time.monotonic_ns()
    event_ns = self.rcp._last_update_nanos if can_strings else None
    trigger_received = self.trigger_msg in vls
    trigger_event_ns = None
    if trigger_received:
      timestamps = self.rcp.message_states[self.trigger_msg].timestamps
      trigger_event_ns = timestamps[-1] if timestamps else None

    freshness_expired = self._seal6_check_freshness(now_ns, event_ns, trigger_received, trigger_event_ns)
    self.updated_messages.update(vls)

    if self.speed_cp is not None:
      self.speed_cp.update(can_strings)
      self._seal6_update_steering_history()
    v_ego, a_ego = self._ego_speed_accel()

    if self.trigger_msg not in self.updated_messages:
      if freshness_expired:
        ret = RadarData()
        ret.errors.canError = True
        return ret
      return None

    previous_tracks = {slot: (pt.trackId, pt.dRel, pt.yRel, pt.vRel) for slot, pt in self.pts.items()} if self.seal6_radar else None

    rr = self._update(self.updated_messages, v_ego, a_ego)
    if previous_tracks is not None:
      self._seal6_reassociate_track_ids(previous_tracks)
      rr.points = self._seal6_points_for_output()
    self.updated_messages.clear()
    return rr

  def _seal6_points_for_output(self):
    """Apply the camera-fit YREL calibration to outgoing Seal 6 radar points."""
    points = []
    for source in self.pts.values():
      point = RadarData.RadarPoint()
      point.trackId = source.trackId
      point.dRel = source.dRel
      point.yRel = SEAL6_YREL_CAMERA_SCALE * source.yRel + SEAL6_YREL_CAMERA_OFFSET_M
      point.vRel = source.vRel
      point.aRel = source.aRel
      point.yvRel = source.yvRel
      point.measured = source.measured
      points.append(point)
    return points

  def _seal6_check_freshness(self, now_ns: int, event_ns: int | None, trigger_received: bool,
                             trigger_event_ns: int | None) -> bool:
    """Clear stale tracks after a radar-frame gap or event-clock rewind."""
    if not self.seal6_radar:
      return False

    last_event_ns = self.seal6_last_trigger_event_ns
    event_clock_valid = event_ns is not None and last_event_ns is not None
    expired_now = not self.seal6_freshness_expired and (
      (event_clock_valid and (event_ns < last_event_ns or event_ns - last_event_ns > SEAL6_RADAR_TIMEOUT_NS)) or
      (not event_clock_valid and self.seal6_last_trigger_monotonic_ns is not None and
       now_ns - self.seal6_last_trigger_monotonic_ns > SEAL6_RADAR_TIMEOUT_NS)
    )

    if expired_now:
      self._seal6_reset_tracks()
      self.updated_messages.clear()
      self.seal6_freshness_expired = True

    if trigger_received:
      self.seal6_last_trigger_monotonic_ns = now_ns
      if trigger_event_ns is not None:
        self.seal6_last_trigger_event_ns = trigger_event_ns
      self.seal6_freshness_expired = False

    return expired_now

  def _seal6_reset_tracks(self):
    for slot in range(self.radar_slot_count):
      self._kill_slot(slot)
    self.track_history.clear()
    self.slot_track_id.clear()
    self.seal6_prev_drel.clear()

  def _seal6_reassociate_track_ids(self, previous_tracks: dict[int, tuple[int, float, float, float]]):
    """Keep software IDs when a radar object migrates between output slots."""
    if not self.pts:
      return

    # Match old active points to new active points one-to-one. Predict range one
    # nominal radar period ahead and require lateral/velocity agreement too.
    candidates = []
    dt = 1.0 / RADAR_FREQ_HZ
    for old_slot, (track_id, old_d, old_y, old_v) in previous_tracks.items():
      d_gate = min(2.5, max(1.0, abs(old_v) * dt + 0.75))
      for new_slot, pt in self.pts.items():
        predicted_d = old_d + old_v * dt
        d_diff = abs(predicted_d - pt.dRel)
        y_diff = abs(old_y - pt.yRel)
        v_diff = abs(old_v - pt.vRel)
        if d_diff <= d_gate and y_diff <= 0.8 and v_diff <= 4.0:
          score = d_diff / d_gate + y_diff / 0.8 + v_diff / 4.0
          if old_slot == new_slot:
            score -= 0.05
          candidates.append((score, track_id, new_slot))

    matched_old, matched_new = set(), set()
    for _, track_id, new_slot in sorted(candidates):
      if track_id in matched_old or new_slot in matched_new:
        continue
      self.pts[new_slot].trackId = track_id
      self.slot_track_id[new_slot] = track_id
      self.track_history.pop(track_id, None)
      matched_old.add(track_id)
      matched_new.add(new_slot)

    old_ids = {state[0] for state in previous_tracks.values()}
    # Preserve unmatched old states briefly for a later return, including when
    # a sensor slot was immediately recycled for a different object.
    for track_id, old_d, old_y, old_v in previous_tracks.values():
      if track_id not in matched_old:
        self.track_history[track_id] = (old_d, old_y, old_v, 0)

    used_ids = set(matched_old)
    for slot, pt in sorted(self.pts.items()):
      if slot in matched_new:
        continue
      if pt.trackId in used_ids or pt.trackId in old_ids:
        pt.trackId = self.track_id
        self.track_id += 1
      used_ids.add(pt.trackId)
      self.slot_track_id[slot] = pt.trackId

  def _seal6_suppress_low_speed_side_returns(self, v_ego: float):
    """Drop a closer off-center return only when a confirmed centered radar lead exists."""
    if v_ego >= SEAL6_LOW_SPEED_V_EGO_MAX:
      return
    centered_tracks = [
      (slot, pt) for slot, pt in self.pts.items()
      if pt.measured and self.valid_cnt[slot] >= SEAL6_LOW_SPEED_CLOSE_VALID_CNT and
      DREL_MIN <= pt.dRel <= SEAL6_LOW_SPEED_CLOSE_DREL and abs(pt.yRel) <= SEAL6_CENTER_RETURN_YREL_MAX
    ]
    if not centered_tracks:
      return
    for slot, pt in list(self.pts.items()):
      side_return = (pt.measured and self.valid_cnt[slot] >= VALID_CNT_ON and
                     DREL_MIN <= pt.dRel < SEAL6_LOW_SPEED_CLOSE_DREL and
                     SEAL6_SIDE_RETURN_YREL_MIN <= abs(pt.yRel) <= SEAL6_LOW_SPEED_YREL_ABS_MAX)
      if side_return and any(center_slot != slot and center_pt.dRel >= pt.dRel + SEAL6_SIDE_RETURN_MIN_RANGE_ADVANTAGE
                             for center_slot, center_pt in centered_tracks):
        self._kill_slot(slot)

  def _find_matching_track_id(self, dRel, yRel, vRel):
    """
    Try to match a reappearing track to a previous track ID based on position/velocity similarity.
    This prevents track ID churn when tracks briefly disappear and reappear.
    """
    best_match_id = None
    best_distance = float('inf')

    # Predict range through the short dropout using measured radial velocity.
    # Tighter gates reduce accidental ID reuse when a radar slot is recycled.
    MAX_DIST_DIFF = 2.5   # meters
    MAX_Y_DIFF = 0.8      # meters
    MAX_V_DIFF = 4.0      # m/s

    for track_id, (hist_d, hist_y, hist_v, miss_frames) in self.track_history.items():
      # Only match tracks that disappeared recently (within last few frames)
      if miss_frames > 8:  # Increased from 5 to allow matching slightly older tracks
        continue

      predicted_d = hist_d + hist_v * miss_frames / RADAR_FREQ_HZ
      dist_diff = abs(predicted_d - dRel)
      y_diff = abs(hist_y - yRel)
      v_diff = abs(hist_v - vRel)

      # Check if within thresholds
      if dist_diff <= MAX_DIST_DIFF and y_diff <= MAX_Y_DIFF and v_diff <= MAX_V_DIFF:
        # Use weighted distance (distance most important, then lateral, then velocity)
        # Reduced weight on velocity since it can change more during brief dropouts
        distance = dist_diff * 2.0 + y_diff * 1.5 + v_diff * 0.3  # Reduced v_diff weight from 0.5
        if distance < best_distance:
          best_distance = distance
          best_match_id = track_id

    return best_match_id

  def _alloc_track_id(self, slot, dRel=None, yRel=None, vRel=None):
    """
    Allocate track ID for a slot. Try to match to previous track ID if this is a reappearing track.
    Priority: 1) Reuse existing slot mapping, 2) Match to history, 3) Allocate new ID
    """
    # If slot already has a track ID mapping, reuse it (most common case)
    # This prevents track ID churn when tracks temporarily disappear and reappear
    if slot in self.slot_track_id:
      track_id = self.slot_track_id[slot]
      # If this track ID is in history (was recently deleted), remove it from history
      if track_id in self.track_history:
        self.track_history.pop(track_id, None)
      return track_id

    # Slot doesn't have a mapping - try to match to history based on position
    if dRel is not None and yRel is not None and vRel is not None:
      matched_id = self._find_matching_track_id(dRel, yRel, vRel)
      if matched_id is not None:
        self.slot_track_id[slot] = matched_id
        # Remove from history since we matched it
        self.track_history.pop(matched_id, None)
        return matched_id

    # No match found - allocate new track ID
    self.slot_track_id[slot] = self.track_id
    self.track_id += 1
    return self.slot_track_id[slot]

  def _kill_slot(self, slot):
    # Before deleting, save track state to history for matching reappearing tracks
    if slot in self.pts:
      pt = self.pts[slot]
      track_id = pt.trackId
      # Save last known state to history
      self.track_history[track_id] = (pt.dRel, pt.yRel, pt.vRel, 0)
      del self.pts[slot]

    # A sensor slot can be reused by a different object. Keep its old ID only
    # in the short-lived motion history so a returning object must match again.
    self.slot_track_id.pop(slot, None)

    self.valid_cnt[slot] = 0
    self.miss_cnt[slot] = 0
    self.seal6_prev_drel.pop(slot, None)

  def _update(self, updated_messages, v_ego: float = 0.0, a_ego: float = 0.0):
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    for slot in range(self.radar_slot_count):
      if self.seal6_radar:
        message_slot = slot % self.radar_msg_count
        record_index = slot // self.radar_msg_count
      else:
        message_slot = slot
        record_index = 0
      address = (SEAL6_RADAR_START_ADDR + message_slot) if self.seal6_radar else (RADAR_START_ADDR + message_slot)
      msg = self.rcp.vl[f"RADAR_TRACK_{message_slot:02d}"]

      # Do not count a cached CANParser value as a new radar sample. In
      # particular, a missing frame must not keep a stale close lead alive.
      if self.seal6_radar and address not in updated_messages:
        self._kill_slot(slot)
        continue

      prev_drel = self.seal6_prev_drel.get(slot) if self.seal6_radar else None
      conf, meas_ok, dRel, yRel, vRel, aRel = self._read_track(msg, v_ego, a_ego, slot, record_index)

      low_speed_untrusted = False
      if self.seal6_radar and meas_ok:
        low_speed_untrusted = self._seal6_low_speed_untrusted(slot, dRel, yRel, vRel, v_ego, prev_drel)
        if low_speed_untrusted:
          meas_ok = False
          conf = 0.0
          self._kill_slot(slot)
          continue

      drel_max = SEAL6_DREL_MAX if self.seal6_radar else DREL_MAX
      yrel_base = SEAL6_YREL_ABS_MAX if self.seal6_radar else YREL_ABS_MAX
      if self.seal6_radar and dRel > SEAL6_FAR_DREL_M:
        yrel_base = min(yrel_base, SEAL6_FAR_YREL_ABS_MAX)
      if self.seal6_radar and v_ego < SEAL6_LOW_SPEED_V_EGO_MAX:
        yrel_base = min(yrel_base, SEAL6_LOW_SPEED_YREL_ABS_MAX)
      yrel_max = yrel_base if dRel <= 30.0 else yrel_base * 0.7
      lateral_error = yRel
      lateral_limit = yrel_max
      if self.seal6_radar:
        curve_lateral_error = self._seal6_curve_path_lateral_error(dRel, yRel, v_ego)
        if curve_lateral_error is not None:
          lateral_error = curve_lateral_error
          lateral_limit = SEAL6_CURVE_PATH_YREL_MAX

      valid_cnt_on = VALID_CNT_ON
      if self.seal6_radar and dRel > SEAL6_FAR_DREL_M:
        valid_cnt_on = SEAL6_FAR_VALID_CNT_ON
      elif self.seal6_radar and v_ego < SEAL6_LOW_SPEED_V_EGO_MAX and abs(yRel) <= 0.5 and dRel <= SEAL6_LOW_SPEED_CLOSE_DREL:
        valid_cnt_on = max(valid_cnt_on, SEAL6_LOW_SPEED_CLOSE_VALID_CNT)

      vrel_max = SEAL6_VREL_ABS_MAX if self.seal6_radar else VREL_ABS_MAX
      arel_max = SEAL6_AREL_ABS_MAX if self.seal6_radar else AREL_ABS_MAX
      plausible = (meas_ok and DREL_MIN <= dRel <= drel_max and abs(lateral_error) <= lateral_limit and
                   abs(vRel) <= vrel_max and abs(aRel) <= arel_max)

      # Require higher confidence for vehicles near the edge (even if within threshold)
      # This adds extra filtering for vehicles that are close to the lateral limit
      conf_required = CONF_ON
      if abs(lateral_error) > 1.0:  # More than 1.0m outside the current lead corridor
        conf_required = max(CONF_ON, 0.85)  # Require higher confidence (85%)

      good = plausible and (conf >= conf_required)

      # Seal 6 has no identified sensor track ID. Drop a record as soon as its
      # validity/physical gates fail, so radard cannot use a stale point as a
      # low-speed lead. A returning record may reclaim its software ID through
      # the recent-motion matcher below.
      if self.seal6_radar and not good:
        self._kill_slot(slot)
        continue

      if good:
        self.valid_cnt[slot] += 1
        self.miss_cnt[slot] = 0
      else:
        if (not plausible) or (conf <= CONF_OFF) or (not meas_ok):
          self.miss_cnt[slot] += 1
        else:
          self.valid_cnt[slot] = max(self.valid_cnt[slot] - 1, 0)

      # Check if track is moving away (should be deleted immediately, don't publish)
      track_moving_away = False
      if slot in self.pts:
        pt = self.pts[slot]
        # Track is moving laterally away beyond threshold - delete immediately
        if abs(lateral_error) > lateral_limit * 1.2:  # 20% beyond the lateral corridor
          track_moving_away = True
        # Track is moving forward away rapidly (dRel increasing, positive vRel)
        elif meas_ok and dRel > pt.dRel + 3.0 and vRel > 2.0:  # Moved >3m forward and moving away >2m/s
          track_moving_away = True
        # Track was previously good but now clearly side-lane (moved beyond threshold)
        elif abs(lateral_error) > lateral_limit and self.valid_cnt[slot] >= valid_cnt_on:
          # Was a valid track but now moved to side-lane - delete immediately
          track_moving_away = True

      # Don't publish if track is moving away
      publish = (self.valid_cnt[slot] >= valid_cnt_on) and not track_moving_away
      # Seal 6: never overwrite a validated track with an out-of-range ghost DREL.
      publish = publish and (good or not self.seal6_radar)

      # Keep tracks alive longer - publish them even when they briefly fail filtering
      # This prevents Kalman filter resets in radard.py
      # BUT: Don't keep tracks alive if they're clearly moving away
      keep_alive = False
      if slot in self.pts and not track_moving_away:
        # Track already exists - keep it alive if it hasn't exceeded MISS_MAX
        if self.miss_cnt[slot] <= MISS_MAX:
          keep_alive = True

      if publish:
        if slot not in self.pts:
          self.pts[slot] = RadarData.RadarPoint()
          # Try to match to previous track ID when allocating
          self.pts[slot].trackId = self._alloc_track_id(slot, dRel, yRel, vRel)
        else:
          # Track exists - remove from history if it was there
          track_id = self.pts[slot].trackId
          self.track_history.pop(track_id, None)

        pt = self.pts[slot]
        pt.measured = True
        pt.dRel = dRel
        pt.yRel = yRel
        pt.vRel = vRel
        pt.aRel = aRel
        pt.yvRel = float("nan")

      elif keep_alive:
        # Track exists but filtering failed temporarily - keep it alive
        # Continue updating with current radar values (even if noisy) to maintain Kalman filter continuity
        # The Kalman filter in radard.py can handle some noise, but needs continuous input
        pt = self.pts[slot]
        # Update with current values but mark as unmeasured to indicate lower reliability
        # This prevents track deletion while still providing input to the Kalman filter
        pt.measured = False
        # Still update with current radar values (slightly smoothed) for continuity
        if meas_ok and plausible:  # Use values if measurement is OK, even if confidence is low
          # Use exponential smoothing to reduce noise when confidence is low
          alpha = 0.7  # Smoothing factor (higher = more weight to new value)
          pt.dRel = alpha * dRel + (1 - alpha) * pt.dRel
          pt.yRel = alpha * yRel + (1 - alpha) * pt.yRel
          pt.vRel = alpha * vRel + (1 - alpha) * pt.vRel
          pt.aRel = alpha * aRel + (1 - alpha) * pt.aRel
        # If measurement is not OK, keep last known values (already set)

      else:
        # Track doesn't exist or has exceeded MISS_MAX, or is moving away
        if track_moving_away:
          # Track is moving away - delete immediately (don't wait for MISS_MAX)
          self._kill_slot(slot)
          # DON'T delete slot_track_id mapping here - keep it for potential rematching
          # The slot_track_id will be reused if the track reappears in the same slot
        elif self.miss_cnt[slot] > MISS_MAX:
          # Track exceeded miss count - delete it
          self._kill_slot(slot)
          # DON'T delete slot_track_id mapping here - keep it for potential rematching
        elif slot in self.pts:
          # Should not reach here if keep_alive logic is correct, but keep as fallback
          self.pts[slot].measured = False

    # Update history for all existing tracks (increment miss frames) - do this once per frame
    # Clean up old history entries
    tracks_to_remove = []
    for track_id, (hist_d, hist_y, hist_v, miss_frames) in list(self.track_history.items()):
      # Increment miss frames for tracks in history
      new_miss_frames = miss_frames + 1
      self.track_history[track_id] = (hist_d, hist_y, hist_v, new_miss_frames)
      # Remove if too old
      if new_miss_frames > self.MAX_HISTORY_FRAMES:
        tracks_to_remove.append(track_id)

    for track_id in tracks_to_remove:
      self.track_history.pop(track_id, None)

    if self.seal6_radar:
      self._seal6_suppress_low_speed_side_returns(v_ego)

    ret.points = list(self.pts.values())
    return ret

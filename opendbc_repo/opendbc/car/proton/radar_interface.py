#!/usr/bin/env python3
import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.proton.values import CANBUS, DBC

RADAR_A_START = 0x20
RADAR_B_START = 0x50
RADAR_SLOT_COUNT = 40
RADAR_B_LAST = RADAR_B_START + RADAR_SLOT_COUNT - 1
RADAR_FREQ_HZ = 18
RADAR_DT = 1.0 / RADAR_FREQ_HZ
RADAR_WAKE_CYCLES = 3

DREL_MIN = 2.0
DREL_MAX = 64.0

# Motion ghost filter: reject static dRel-only tracks at any range (not fixed meters).
STATIC_DREL_FRAMES = 6
STATIC_DREL_EPS = 0.15

# CAN vRel + dRel derivative fusion (highway + stop-go creep).
VREL_PUBLISH_MIN = 1.6
VREL_PUBLISH_MAX = 25.0
VREL_STABLE_FRAMES = 3
VREL_STABLE_MAX_DELTA = 0.8
VREL_AGREE_MAX = 2.5
VREL_CREEP_MOTION_MIN = 0.08
VREL_CREEP_DREL_SPAN_MIN = 0.10
VREL_CREEP_AGREE_MAX = 1.5
VREL_CREEP_CAN_FALLBACK = 0.35

VALID_CNT_ON = 3
MISS_MAX = 5
DREL_JUMP_MAX = 6.0
MAX_PUBLISH_POINTS = 8
YREL_UNKNOWN = 2.5
DREL_HIST_MAX = max(STATIC_DREL_FRAMES, VREL_STABLE_FRAMES)

INVALID_A_PREFIXES = (b"\xff\x00\x00\x00\x80", b"\x80\x10\x00\x00\x00\x80")
INVALID_B_PREFIX = b"\xff\x00\x00\x00\x80"


def _create_radar_can_parser(CP):
  dbc = DBC[CP.carFingerprint]
  if Bus.radar not in dbc:
    return None
  messages = (
    [(f"RADAR_TRACK_{a:02x}_A", RADAR_FREQ_HZ) for a in range(RADAR_A_START, RADAR_A_START + RADAR_SLOT_COUNT)]
    + [(f"RADAR_TRACK_{a:02x}_B", RADAR_FREQ_HZ) for a in range(RADAR_B_START, RADAR_B_START + RADAR_SLOT_COUNT)]
  )
  return CANParser(dbc[Bus.radar], messages, CANBUS.radar_bus)


def _looks_invalid_a(dat: bytes) -> bool:
  return any(dat.startswith(p) for p in INVALID_A_PREFIXES)


def _looks_invalid_b(dat: bytes) -> bool:
  return dat.startswith(INVALID_B_PREFIX)


def _has_live_radar_bus(raw_frames: dict[int, bytes]) -> bool:
  return any(RADAR_A_START <= a < RADAR_A_START + RADAR_SLOT_COUNT for a in raw_frames)


def _decode_vrel_can(b_msg) -> float:
  return -float(b_msg.get("REL_SPEED", 0.0))


def _can_vrel_stable(can_hist: list[float]) -> bool:
  return len(can_hist) >= VREL_STABLE_FRAMES and max(abs(can_hist[-1] - v) for v in can_hist[:-1]) <= VREL_STABLE_MAX_DELTA


def _derive_vrel_drel(drel_hist: list[float]) -> float:
  return (drel_hist[-1] - drel_hist[-2]) / RADAR_DT if len(drel_hist) >= 2 else float("nan")


def _drel_span(drel_hist: list[float]) -> float:
  return max(drel_hist) - min(drel_hist) if len(drel_hist) >= 2 else 0.0


def _fusion_vrel_highway(can_v: float, can_hist: list[float], drel_v: float) -> bool:
  return (
    _can_vrel_stable(can_hist)
    and VREL_PUBLISH_MIN <= abs(can_v) <= VREL_PUBLISH_MAX
    and not math.isnan(drel_v)
    and abs(can_v - drel_v) <= VREL_AGREE_MAX
  )


def _fusion_vrel_creep(can_v: float, can_hist: list[float], drel_v: float, drel_hist: list[float]) -> bool:
  if not _can_vrel_stable(can_hist) or abs(can_v) >= VREL_PUBLISH_MIN or math.isnan(drel_v):
    return False
  if (span := _drel_span(drel_hist)) <= VREL_CREEP_DREL_SPAN_MIN or abs(drel_v) < VREL_CREEP_MOTION_MIN:
    return False
  return abs(can_v) < VREL_CREEP_CAN_FALLBACK or abs(can_v - drel_v) <= VREL_CREEP_AGREE_MAX


def _creep_fusion_vrel(can_v: float, drel_v: float) -> float:
  if abs(can_v) >= VREL_CREEP_CAN_FALLBACK and not math.isnan(drel_v) and abs(can_v - drel_v) <= VREL_CREEP_AGREE_MAX:
    return can_v
  if not math.isnan(drel_v) and abs(drel_v) >= VREL_CREEP_MOTION_MIN:
    return drel_v
  return can_v


def _compute_fusion_vrel(can_v: float, can_hist: list[float], drel_v: float, drel_hist: list[float]) -> tuple[float, float]:
  if _fusion_vrel_highway(can_v, can_hist, drel_v):
    return can_v, abs(can_v - drel_v)
  if _fusion_vrel_creep(can_v, can_hist, drel_v, drel_hist):
    return _creep_fusion_vrel(can_v, drel_v), abs(can_v - drel_v)
  return float("nan"), float("inf")


def _is_frozen_drel_only_ghost(drel_hist: list[float], v_rel: float) -> bool:
  if not math.isnan(v_rel):
    return False
  return len(drel_hist) < STATIC_DREL_FRAMES or _drel_span(drel_hist) <= STATIC_DREL_EPS


class RadarInterface(RadarInterfaceBase):
  """Proton X50 Pre-FL front radar on bus 1 (A/B track banks)."""

  def __init__(self, CP):
    super().__init__(CP)
    self.radar_off_can = CP.radarUnavailable
    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP)
    self.trigger_msg = RADAR_B_LAST
    self.updated_messages: set[int] = set()
    self.valid_cnt = {s: 0 for s in range(RADAR_SLOT_COUNT)}
    self.miss_cnt = {s: 0 for s in range(RADAR_SLOT_COUNT)}
    self.prev_drel: dict[int, float] = {}
    self.drel_hist = {s: [] for s in range(RADAR_SLOT_COUNT)}
    self.vrel_hist = {s: [] for s in range(RADAR_SLOT_COUNT)}
    self.vrel_agreement: dict[int, float] = {}
    self.creep_vrel_slots: set[int] = set()
    self.raw_frames: dict[int, bytes] = {}
    self.wake_cycles = 0

  def _drop_slot(self, slot: int):
    self.vrel_agreement.pop(slot, None)
    self.creep_vrel_slots.discard(slot)
    self.pts.pop(slot, None)

  def update(self, can_packets):
    if self.radar_off_can or self.rcp is None:
      return super().update(None)

    if can_packets:
      packets = can_packets if isinstance(can_packets[0], (list, tuple)) else [can_packets]
      for _, frames in packets:
        for address, dat, src in frames:
          if src == CANBUS.radar_bus and (
            RADAR_A_START <= address < RADAR_A_START + RADAR_SLOT_COUNT
            or RADAR_B_START <= address <= RADAR_B_LAST
          ):
            self.raw_frames[address] = bytes(dat)

    self.updated_messages.update(self.rcp.update(can_packets))
    if self.trigger_msg not in self.updated_messages:
      return None

    self.wake_cycles = min(self.wake_cycles + 1, RADAR_WAKE_CYCLES) if _has_live_radar_bus(self.raw_frames) else 0
    if self.wake_cycles < RADAR_WAKE_CYCLES:
      self.updated_messages.clear()
      self.raw_frames.clear()
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()
    self.raw_frames.clear()
    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    for slot in range(RADAR_SLOT_COUNT):
      a_addr, b_addr = RADAR_A_START + slot, RADAR_B_START + slot
      a_raw = self.raw_frames.get(a_addr)
      b_raw = self.raw_frames.get(b_addr)
      a_msg = self.rcp.vl.get(a_addr)
      b_msg = self.rcp.vl.get(b_addr)

      if None in (a_raw, b_raw, a_msg, b_msg) or _looks_invalid_a(a_raw) or _looks_invalid_b(b_raw):
        self._mark_invalid(slot)
        continue

      if (d_rel := float(b_msg.get("LONG_DIST", 255.0))) < DREL_MIN or d_rel > DREL_MAX:
        self._mark_invalid(slot)
        continue

      if slot in self.prev_drel and abs(d_rel - self.prev_drel[slot]) > DREL_JUMP_MAX:
        self._mark_invalid(slot)
        self.prev_drel[slot] = d_rel
        self.drel_hist[slot] = []
        continue

      self.valid_cnt[slot] = min(self.valid_cnt[slot] + 1, VALID_CNT_ON + 2)
      self.miss_cnt[slot] = 0
      self.prev_drel[slot] = d_rel

      dh = self.drel_hist[slot]
      dh.append(d_rel)
      if len(dh) > DREL_HIST_MAX:
        dh.pop(0)

      if self.valid_cnt[slot] < VALID_CNT_ON:
        if slot in self.pts:
          self.pts[slot].vRel = float("nan")
        continue

      can_v = _decode_vrel_can(b_msg)
      can_hist = self.vrel_hist[slot]
      can_hist.append(can_v)
      if len(can_hist) > VREL_STABLE_FRAMES:
        can_hist.pop(0)

      v_rel, agree = _compute_fusion_vrel(can_v, can_hist, _derive_vrel_drel(dh), dh)
      if not math.isnan(v_rel):
        self.vrel_agreement[slot] = agree
        if abs(can_v) < VREL_PUBLISH_MIN:
          self.creep_vrel_slots.add(slot)
        else:
          self.creep_vrel_slots.discard(slot)
      else:
        self.vrel_agreement.pop(slot, None)
        self.creep_vrel_slots.discard(slot)

      if _is_frozen_drel_only_ghost(dh, v_rel):
        self._drop_slot(slot)
        continue

      if slot not in self.pts:
        self.pts[slot] = structs.RadarData.RadarPoint()
        self.pts[slot].trackId = slot + 1

      pt = self.pts[slot]
      pt.measured = True
      pt.dRel = d_rel
      pt.yRel = -float(a_msg.get("LAT_DIST", 0.0)) if a_msg.get("VALID", 0) else YREL_UNKNOWN
      pt.vRel = v_rel
      pt.aRel = pt.yvRel = float("nan")

    for slot, pt in list(self.pts.items()):
      if not math.isnan(pt.vRel) and abs(pt.vRel) < VREL_PUBLISH_MIN and slot not in self.creep_vrel_slots:
        pt.vRel = float("nan")
      dh = self.drel_hist.get(slot, [])
      if _is_frozen_drel_only_ghost(dh, pt.vRel):
        pt.measured = False

    for slot in list(self.pts):
      pt = self.pts[slot]
      if not pt.measured or _is_frozen_drel_only_ghost(self.drel_hist.get(slot, []), pt.vRel):
        self._drop_slot(slot)

    if self.pts:
      keep = {
        s for s, _ in sorted(
          self.pts.items(),
          key=lambda kv: (0 if not math.isnan(kv[1].vRel) else 1, self.vrel_agreement.get(kv[0], float("inf")), kv[1].dRel),
        )[:MAX_PUBLISH_POINTS]
      }
      for slot in list(self.pts):
        if slot not in keep:
          self._drop_slot(slot)

    ret.points = list(self.pts.values())
    return ret

  def _mark_invalid(self, slot: int):
    self.valid_cnt[slot] = max(self.valid_cnt[slot] - 1, 0)
    self.miss_cnt[slot] = min(self.miss_cnt[slot] + 1, MISS_MAX)
    self.vrel_hist[slot] = []
    self.drel_hist[slot] = []
    self.vrel_agreement.pop(slot, None)
    self.creep_vrel_slots.discard(slot)
    if self.miss_cnt[slot] >= MISS_MAX:
      self.prev_drel.pop(slot, None)
      self._drop_slot(slot)

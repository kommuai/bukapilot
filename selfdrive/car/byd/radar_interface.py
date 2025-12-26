#!/usr/bin/env python3
from opendbc.can.parser import CANParser
from cereal import car
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from openpilot.selfdrive.car.byd.values import DBC

RADAR_START_ADDR = 0x280
RADAR_MSG_COUNT = 10
RADAR_FREQ_HZ = 20

# --- Temporal filtering / hysteresis ---
CONF_ON = 0.70
CONF_OFF = 0.50
VALID_CNT_ON = 2       # frames required to be considered stable
MISS_MAX = 3           # frames allowed to miss before deleting

# --- Plausibility gates ---
DREL_MIN = 0.75        # near-field radar ghosts
DREL_MAX = 200.0
YREL_ABS_MAX = 6.0
VREL_ABS_MAX = 60.0
AREL_ABS_MAX = 12.0


def _create_radar_can_parser(car_fingerprint):
  if DBC[car_fingerprint]["radar"] is None:
    return None

  messages = [(f"RADAR_TRACK_{addr:02d}", RADAR_FREQ_HZ)
              for addr in range(RADAR_MSG_COUNT)]
  return CANParser(DBC[car_fingerprint]["radar"], messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.updated_messages = set()
    self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1

    self.track_id = 0
    self.slot_track_id = {}     # slot -> stable trackId

    self.valid_cnt = {i: 0 for i in range(RADAR_MSG_COUNT)}
    self.miss_cnt = {i: 0 for i in range(RADAR_MSG_COUNT)}

    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP.carFingerprint)

  def update(self, can_strings, v_ego, a_ego):
    if self.rcp is None:
      return super().update(None, v_ego, a_ego)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages, v_ego, a_ego)
    self.updated_messages.clear()
    return rr

  def _alloc_track_id(self, slot):
    if slot not in self.slot_track_id:
      self.slot_track_id[slot] = self.track_id
      self.track_id += 1
    return self.slot_track_id[slot]

  def _kill_slot(self, slot):
    if slot in self.pts:
      del self.pts[slot]
    self.valid_cnt[slot] = 0
    self.miss_cnt[slot] = 0
    # keep slot_track_id to avoid ID churn on brief dropouts

  def _update(self, updated_messages, v_ego, a_ego):
    ret = car.RadarData.new_message()

    # propagate CAN validity
    if not self.rcp.can_valid:
      ret.errors = ["canError"]

    for slot in range(RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{slot:02d}"]

      conf = float(msg.get("CONFIDENCE", 0.0))
      long_dist = float(msg.get("LONG_DIST", 255.0))
      lat_dist = float(msg.get("LAT_DIST", 0.0))
      vlead = float(msg.get("VLEAD", 0.0))
      alead = float(msg.get("ALEAD", 0.0))

      meas_ok = long_dist < 255.0

      # --- convert to openpilot frame (intentional + correct per your note) ---
      dRel = long_dist - 4.0
      yRel = lat_dist
      vRel = vlead - v_ego
      aRel = alead - a_ego

      plausible = (
        meas_ok and
        (DREL_MIN <= dRel <= DREL_MAX) and
        (abs(yRel) <= YREL_ABS_MAX) and
        (abs(vRel) <= VREL_ABS_MAX) and
        (abs(aRel) <= AREL_ABS_MAX)
      )

      good = plausible and (conf >= CONF_ON)

      if good:
        self.valid_cnt[slot] += 1
        self.miss_cnt[slot] = 0
      else:
        if (not plausible) or (conf <= CONF_OFF) or (not meas_ok):
          self.miss_cnt[slot] += 1
        else:
          self.valid_cnt[slot] = max(self.valid_cnt[slot] - 1, 0)

      publish = (self.valid_cnt[slot] >= VALID_CNT_ON)

      if publish:
        if slot not in self.pts:
          self.pts[slot] = car.RadarData.RadarPoint.new_message()
          self.pts[slot].trackId = self._alloc_track_id(slot)

        pt = self.pts[slot]
        pt.measured = True
        pt.dRel = dRel
        pt.yRel = yRel
        pt.vRel = vRel
        pt.aRel = aRel
        pt.yvRel = float("nan")

      else:
        if self.miss_cnt[slot] > MISS_MAX:
          self._kill_slot(slot)
        else:
          if slot in self.pts:
            # keep track alive but unmeasured
            self.pts[slot].measured = False

    ret.points = list(self.pts.values())
    return ret






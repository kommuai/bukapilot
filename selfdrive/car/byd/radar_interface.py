#!/usr/bin/env python3
from opendbc.can.parser import CANParser
from cereal import car
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
from openpilot.selfdrive.car.byd.values import DBC

RADAR_START_ADDR = 0x280
RADAR_MSG_COUNT = 10
RADAR_FREQ_HZ = 20

def _create_radar_can_parser(car_fingerprint):
  if DBC[car_fingerprint]['radar'] is None:
    return None

  messages = [(f"RADAR_TRACK_{addr:02d}", RADAR_FREQ_HZ) for addr in range(RADAR_MSG_COUNT)]
  return CANParser(DBC[car_fingerprint]['radar'], messages, 1)

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    self.track_id = 0
    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP.carFingerprint)

  def update(self, can_strings, v_ego, a_ego):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages, v_ego, a_ego)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages, v_ego, a_ego):
    ret = car.RadarData.new_message()
    if self.rcp is None:
      return ret

    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    errors = []

    for addr in range(RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:02d}"]

      if addr not in self.pts:
        self.pts[addr] = car.RadarData.RadarPoint.new_message()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['CONFIDENCE'] > 0.99
      if valid:
        if msg["LONG_DIST"] < 255:
          self.pts[addr].measured = True
          self.pts[addr].dRel = msg['LONG_DIST']
          self.pts[addr].yRel = msg['LAT_DIST'] # negative is to the right of the car
          self.pts[addr].vRel = v_ego - msg['VLEAD']
          self.pts[addr].aRel = a_ego - msg['ALEAD']
          self.pts[addr].yvRel = float('nan')
      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret

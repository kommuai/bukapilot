#!/usr/bin/env python3
import unittest
from unittest import mock

from cereal import car

from opendbc.car.perodua_qve.carcontroller import CarController
from opendbc.car.perodua_qve.peroduaqvecan import pack_cam_b0, pack_cam_b1, create_cam_lane_spoof
from opendbc.car.perodua_qve.qve_checksum_data import (
  CAM_B0_IDLE_LUT,
  CAM_B0_IDLE_PAYLOAD,
  CAM_B1_IDLE_LUT,
  CAM_B1_IDLE_PAYLOAD,
)
from opendbc.car.perodua_qve.values import CANBUS, ENABLE_CAM_LANE_SPOOF


class TestCamLanePacker(unittest.TestCase):
  def test_idle_b0_checksum_lut(self):
    for b2, expect in ((0x00, 0x17B7), (0x01, 0xB97A), (0x10, CAM_B0_IDLE_LUT[0x1000])):
      frame = pack_cam_b0(CAM_B0_IDLE_PAYLOAD, b2)
      self.assertEqual(len(frame), 64)
      self.assertEqual(frame[2], b2)
      self.assertEqual(frame[3], b2 & 0x0F)
      chk = (frame[0] << 8) | frame[1]
      self.assertEqual(chk, expect)

  def test_idle_b1_checksum_lut(self):
    for b2 in (0x00, 0x01, 0x2A):
      frame = pack_cam_b1(CAM_B1_IDLE_PAYLOAD, b2)
      self.assertEqual(len(frame), 16)
      cnt = (b2 << 8) | (b2 & 0x0F)
      expect = CAM_B1_IDLE_LUT[cnt]
      chk = (frame[0] << 8) | frame[1]
      self.assertEqual(chk, expect)
      self.assertEqual(frame[2], b2)
      self.assertEqual(frame[3], b2 & 0x0F)

  def test_b0_duplicates_enforced(self):
    pl = bytearray(CAM_B0_IDLE_PAYLOAD)
    pl[24] = 0xAB  # frame b28
    pl[35] = 0x11  # frame b39
    pl[36] = 0x22  # frame b40
    pl[39] = 0x33  # frame b43
    frame = pack_cam_b0(pl, 0x05)
    self.assertEqual(frame[38], frame[28])
    self.assertEqual(frame[41], frame[39])
    self.assertEqual(frame[42], frame[40])
    self.assertEqual(frame[44], frame[43])

  def test_create_cam_lane_spoof_addrs_bus(self):
    msgs = create_cam_lane_spoof(0x03, line_mux=0x80)
    self.assertEqual(len(msgs), 2)
    self.assertEqual(msgs[0].address, 0xB0)
    self.assertEqual(msgs[1].address, 0xB1)
    self.assertEqual(msgs[0].src, CANBUS.cam)
    self.assertEqual(msgs[1].src, CANBUS.cam)
    self.assertEqual(len(msgs[0].dat), 64)
    self.assertEqual(len(msgs[1].dat), 16)
    self.assertEqual(msgs[0].dat[4], 0x80)


class TestCamLaneControllerGate(unittest.TestCase):
  def test_enable_flag_on(self):
    self.assertTrue(ENABLE_CAM_LANE_SPOOF)

  @mock.patch('opendbc.car.perodua_qve.carcontroller.ENABLE_CAM_LANE_SPOOF', False)
  def test_controller_sends_nothing_when_disabled(self):
    CC = mock.Mock()
    CS = mock.Mock()
    CS.out.steeringAngleDeg = 0.0
    CS.cam_session_counter = 0x12
    ctrl = CarController({"pt": "perodua_qve_pt"}, mock.Mock())
    for _ in range(20):
      _, sends = ctrl.update(CC, CS, 0)
      self.assertEqual(sends, [])

  @mock.patch('opendbc.car.perodua_qve.carcontroller.CAM_LANE_SPOOF_PERIOD', 1)
  def test_controller_sends_when_enabled(self):
    CC = mock.Mock()
    CS = mock.Mock()
    CS.out.steeringAngleDeg = 0.0
    CS.cam_session_counter = 0x12
    ctrl = CarController({"pt": "perodua_qve_pt"}, mock.Mock())
    _, sends = ctrl.update(CC, CS, 0)
    self.assertEqual(len(sends), 2)
    self.assertEqual(sends[0].address, 0xB0)
    self.assertEqual(sends[1].address, 0xB1)
    self.assertEqual(sends[0].dat[2], 0x12)



class TestCamCounterSync(unittest.TestCase):
  def _cp(self):
    from opendbc.can import CANParser
    from opendbc.car.perodua_qve.values import CAM_PARSER_MSGS, CANBUS
    return CANParser("perodua_qve_pt", CAM_PARSER_MSGS, CANBUS.cam)

  def _feed(self, cp, addr, b2, dlc):
    from opendbc.car.perodua_qve.values import CANBUS
    dat = bytearray(dlc)
    dat[2] = b2
    cp.update([(1_000_000_000, [(addr, bytes(dat), CANBUS.cam)])])

  def test_get_can_parsers_includes_cam(self):
    from opendbc.car import Bus
    from opendbc.car.perodua_qve.carstate import CarState
    from opendbc.car.perodua_qve.values import CAR
    CP = mock.Mock()
    CP.carFingerprint = CAR.PERODUA_QVE
    parsers = CarState.get_can_parsers(CP)
    self.assertIn(Bus.pt, parsers)
    self.assertIn(Bus.cam, parsers)

  def test_prefer_0x80_over_b0(self):
    from opendbc.car import Bus
    from opendbc.car.perodua_qve.carstate import CarState
    cp = self._cp()
    self._feed(cp, 0xB0, 0x11, 64)
    self._feed(cp, 0x80, 0x22, 20)
    CS = CarState.__new__(CarState)
    CS.cam_session_counter = 0
    CS.cam_session_counter_valid = False
    CS._update_cam_session_counter({Bus.cam: cp})
    self.assertEqual(CS.cam_session_counter, 0x22)
    self.assertTrue(CS.cam_session_counter_valid)

  def test_fallback_b0_when_0x80_unseen(self):
    from opendbc.car import Bus
    from opendbc.car.perodua_qve.carstate import CarState
    cp = self._cp()
    self._feed(cp, 0xB0, 0x33, 64)
    CS = CarState.__new__(CarState)
    CS.cam_session_counter = 0
    CS.cam_session_counter_valid = False
    CS._update_cam_session_counter({Bus.cam: cp})
    self.assertEqual(CS.cam_session_counter, 0x33)
    self.assertTrue(CS.cam_session_counter_valid)

  def test_unseen_does_not_claim_valid_zero(self):
    from opendbc.car import Bus
    from opendbc.car.perodua_qve.carstate import CarState
    cp = self._cp()
    CS = CarState.__new__(CarState)
    CS.cam_session_counter = 7
    CS.cam_session_counter_valid = False
    CS._update_cam_session_counter({Bus.cam: cp})
    self.assertEqual(CS.cam_session_counter, 7)
    self.assertFalse(CS.cam_session_counter_valid)

  def test_spoof_only_b0_b1_addrs(self):
    msgs = create_cam_lane_spoof(0x05, line_mux=0x00)
    addrs = {m.address for m in msgs}
    self.assertEqual(addrs, {0xB0, 0xB1})
    forbidden = set(range(0x80, 0x89)) | set(range(0xB2, 0xB8))
    self.assertTrue(addrs.isdisjoint(forbidden))


if __name__ == "__main__":
  unittest.main()

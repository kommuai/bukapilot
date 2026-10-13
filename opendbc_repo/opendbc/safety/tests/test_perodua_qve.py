#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.perodua_qve.values import (
  BUS2_ALWAYS_FORWARD,
  BUS2_FORWARD_RANGE,
  PeroduaQveSafetyFlags,
  qve_bus2_split_addrs,
)
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety


class TestPeroduaQveSafety(unittest.TestCase):
  RADAR_BUS = 0
  MAIN_BUS = 1
  CAM_BUS = 2

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.packer = CANPackerSafety("perodua_qve_pt")
    self.safety.set_safety_hooks(CarParams.SafetyModel.peroduaQve, int(PeroduaQveSafetyFlags.IGNORE_IGNITION_LINE))
    self.blocked, self.forwarded = qve_bus2_split_addrs()

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _gas(self, b5=0, b6=0, b7=0):
    data = bytearray(8)
    data[5] = b5 & 0xFF
    data[6] = b6 & 0xFF
    data[7] = b7 & 0xFF
    return libsafety_py.make_CANPacket(188, self.MAIN_BUS, bytes(data))

  def _brake(self, b3=12, b5=8):
    data = bytearray(8)
    data[3] = b3 & 0xFF
    data[5] = b5 & 0xFF
    return libsafety_py.make_CANPacket(181, self.MAIN_BUS, bytes(data))

  def test_radar_cam_forwarding_default(self):
    self.assertEqual(self.CAM_BUS, self.safety.safety_fwd_hook(self.RADAR_BUS, 0x71))
    self.assertEqual(-1, self.safety.safety_fwd_hook(self.MAIN_BUS, 0xA0))
    self.assertEqual(self.RADAR_BUS, self.safety.safety_fwd_hook(self.CAM_BUS, BUS2_ALWAYS_FORWARD))
    for addr in BUS2_FORWARD_RANGE:
      self.assertEqual(self.RADAR_BUS, self.safety.safety_fwd_hook(self.CAM_BUS, addr))
    self.assertEqual(-1, self.safety.safety_fwd_hook(self.CAM_BUS, 0x51))
    self.assertEqual(-1, self.safety.safety_fwd_hook(self.CAM_BUS, 0x89))
    self.assertEqual(-1, self.safety.safety_fwd_hook(self.CAM_BUS, 0xB0))
    self.assertEqual(-1, self.safety.safety_fwd_hook(self.CAM_BUS, 0x3A2))
    for addr in self.blocked:
      self.assertEqual(-1, self.safety.safety_fwd_hook(self.CAM_BUS, addr))
    for addr in self.forwarded:
      self.assertEqual(self.RADAR_BUS, self.safety.safety_fwd_hook(self.CAM_BUS, addr))

  def test_rx_gas_and_brake(self):
    self.assertTrue(self._rx(self._gas()))
    self.assertTrue(self._rx(self._gas(b7=6)))
    self.assertTrue(self._rx(self._brake()))
    self.assertTrue(self._rx(self._brake(b3=18)))
    self.assertTrue(self._rx(self._brake(b5=10)))

  def test_rx_vehicle_moving(self):
    msg = self.packer.make_can_msg_safety("IMU", self.MAIN_BUS, {"VEHICLE_SPEED": 10.0})
    self._rx(msg)
    self.assertTrue(self.safety.get_vehicle_moving())


  def test_tx_cam_lane_spoof_allowed(self):
    # 0xB0 (64B) / 0xB1 (16B) on cam bus must be TX-whitelisted for gated spoof.
    b0 = libsafety_py.make_CANPacket(0xB0, self.CAM_BUS, bytes(64))
    b1 = libsafety_py.make_CANPacket(0xB1, self.CAM_BUS, bytes(16))
    self.assertTrue(self.safety.safety_tx_hook(b0))
    self.assertTrue(self.safety.safety_tx_hook(b1))
    # Still blocked on wrong bus / other cam IDs
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0xB0, self.MAIN_BUS, bytes(64))))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0xA5, self.MAIN_BUS, bytes(8))))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x80, self.CAM_BUS, bytes(20))))


if __name__ == "__main__":
  unittest.main()

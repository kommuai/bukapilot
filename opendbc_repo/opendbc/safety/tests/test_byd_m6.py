#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py


class TestBydM6Safety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 7)
    self.safety.init_tests()

  def test_stock_messages_forward_and_control_messages_stay_blocked(self):
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x1FC))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1FC))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x32E))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x32E))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1E2))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x316))

  def test_controller_frames_are_not_in_m6_tx_allowlist(self):
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x1FC, 2, bytes(8))))
    accel_cmd = libsafety_py.make_CANPacket(0x32E, 0, bytes([100, 0, 0, 0, 0, 0, 0, 0]))
    self.assertFalse(self.safety.safety_tx_hook(accel_cmd))

  def test_seal6_profile_keeps_existing_blocks(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 3)
    self.safety.init_tests()
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x1FC))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x32E))


if __name__ == "__main__":
  unittest.main()

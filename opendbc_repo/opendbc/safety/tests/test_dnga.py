#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety


class TestDngaSafety(unittest.TestCase):
  """DNGA safety: RX state + ACC_CMD_HUD / ACC_BRAKE TX validation."""

  TX_MSGS = [[464, 0], [628, 0], [625, 0], [627, 0], [519, 0], [520, 0], [2015, 0]]

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.dnga, 0)
    self.safety.init_tests()
    self.packer = CANPackerSafety("dnga_general_pt")

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _wheel_speed(self, speed: float):
    return self.packer.make_can_msg_safety("WHEEL_SPEED", 0, {"WHEELSPEED_F": speed})

  def _gas_pedal(self, pressed: bool):
    # gas_pressed = not GAS_PEDAL_STEP
    return self.packer.make_can_msg_safety("GAS_PEDAL_2", 0, {"GAS_PEDAL_STEP": 0 if pressed else 1})

  def _brake(self, engaged: bool):
    return self.packer.make_can_msg_safety("BRAKE", 0, {"BRAKE_ENGAGED": engaged})

  def _acc_cmd_hud(self, *, engaged: bool = False, acc_cmd: float = 0.0):
    values = {
      "ACC_CMD": acc_cmd,
      "SET_ME_1_2": engaged,
      "SET_1_WHEN_ENGAGE": engaged,
      "IS_ACCEL": engaged,
      "IS_DECEL": False,
    }
    return self.packer.make_can_msg_safety("ACC_CMD_HUD", 0, values)

  def _acc_brake_raw(self, *, engage: bool, brake_req: bool, magnitude: int, pump: int):
    # Build raw bytes directly; DBC layout does not match dnga_tx_hook byte checks.
    data = bytearray(8)
    if engage:
      data[1] |= 1 << 0  # SET_ME_1_WHEN_ENGAGE @ bit 8
    if brake_req:
      data[1] |= 1 << 5  # BRAKE_REQ @ bit 13
    data[5] = magnitude & 0xFF
    data[4] = pump & 0xFF
    data[3] = (0 - pump) & 0xFF
    return libsafety_py.make_CANPacket(625, 0, bytes(data))

  def test_rx_sets_controls_allowed(self):
    self.safety.set_controls_allowed(False)
    self._rx(self._wheel_speed(0.0))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_rx_vehicle_moving(self):
    self._rx(self._wheel_speed(0.0))
    self.assertFalse(self.safety.get_vehicle_moving())
    self._rx(self._wheel_speed(10.0))
    self.assertTrue(self.safety.get_vehicle_moving())

  def test_rx_gas_and_brake(self):
    self._rx(self._gas_pedal(pressed=True))
    self._rx(self._gas_pedal(pressed=False))
    self._rx(self._brake(engaged=True))
    self._rx(self._brake(engaged=False))

  def test_acc_cmd_hud_blocks_accel_when_not_engaged(self):
    self.assertFalse(self._tx(self._acc_cmd_hud(engaged=False, acc_cmd=1.0)))

  def test_acc_cmd_hud_allows_neutral_when_not_engaged(self):
    self.assertTrue(self._tx(self._acc_cmd_hud(engaged=False, acc_cmd=0.0)))

  def test_acc_cmd_hud_allows_accel_when_engaged(self):
    self.assertTrue(self._tx(self._acc_cmd_hud(engaged=True, acc_cmd=1.0)))

  def test_acc_brake_blocks_brake_req_when_not_engaged(self):
    self.assertFalse(self._tx(self._acc_brake_raw(engage=False, brake_req=True, magnitude=128, pump=0)))

  def test_acc_brake_allows_neutral_when_not_engaged(self):
    self.assertTrue(self._tx(self._acc_brake_raw(engage=False, brake_req=False, magnitude=128, pump=0)))

  def test_acc_brake_magnitude_envelope(self):
    self.assertFalse(self._tx(self._acc_brake_raw(engage=True, brake_req=False, magnitude=43, pump=0)))
    self.assertFalse(self._tx(self._acc_brake_raw(engage=True, brake_req=False, magnitude=201, pump=0)))
    self.assertTrue(self._tx(self._acc_brake_raw(engage=True, brake_req=False, magnitude=128, pump=0)))

  def test_acc_brake_pump_envelope_and_inverse(self):
    self.assertFalse(self._tx(self._acc_brake_raw(engage=True, brake_req=False, magnitude=128, pump=11)))
    self.assertTrue(self._tx(self._acc_brake_raw(engage=True, brake_req=False, magnitude=128, pump=3)))
    msg = self._acc_brake_raw(engage=True, brake_req=False, magnitude=128, pump=3)
    data = bytearray(msg[0].data)
    data[3] = 0  # break pump inverse pairing
    bad = libsafety_py.make_CANPacket(625, 0, bytes(data))
    self.assertFalse(self._tx(bad))
    self.assertTrue(self._tx(self._acc_brake_raw(engage=True, brake_req=False, magnitude=128, pump=5)))


if __name__ == "__main__":
  unittest.main()

#!/usr/bin/env python3
import unittest

from opendbc.car.proton.protoncan import proton_checksum
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety


def _proton_checksum_fix(msg):
  addr, dat, bus = msg
  data = bytearray(dat)
  data[-1] = proton_checksum(addr, None, data)
  return addr, bytes(data), bus


class TestProtonSafety(unittest.TestCase):
  TX_MSGS = [[432, 0], [417, 0], [643, 2]]

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.proton, 0)
    self.safety.init_tests()
    self.packer = CANPackerSafety("proton_general_pt")
    self._acc_counter = 0
    self._lkas_counter = 0

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _pack(self, name: str, bus: int, values: dict):
    if name in ("ACC_CMD", "ADAS_LKAS", "PCM_BUTTONS"):
      if "COUNTER" not in values:
        if name == "ACC_CMD":
          values = {**values, "COUNTER": self._acc_counter % 16}
          self._acc_counter += 1
        elif name == "ADAS_LKAS":
          values = {**values, "COUNTER": self._lkas_counter % 16}
          self._lkas_counter += 1
      return self.packer.make_can_msg_safety(name, bus, values, fix_checksum=_proton_checksum_fix)
    return self.packer.make_can_msg_safety(name, bus, values)

  def _acc_buttons(self, *, res=False, set_btn=False, cancel=False):
    return self._pack("ACC_BUTTONS", 0, {
      "RES_BUTTON": res,
      "SET_BUTTON": set_btn,
      "CRUISE_BTN": cancel,
    })

  def _acc_cmd(self, bus: int, *, acc_req=False, standstill=False, cruise_disabled=False):
    return self._pack("ACC_CMD", bus, {
      "ACC_REQ": acc_req,
      "STANDSTILL_REQ": standstill,
      "CRUISE_DISABLED": cruise_disabled,
      "CMD": 0,
      "CMD_OFFSET1": 0,
      "CMD_OFFSET2": 0,
      "SET_ME_1": 1,
      "SET_ME_X6A": 0x6A,
    })

  def _adas_lkas(self, *, steer_req: bool, steer_cmd: int = 0):
    return self._pack("ADAS_LKAS", 0, {
      "LKAS_ENGAGED1": steer_req,
      "LKAS_LINE_ACTIVE": steer_req,
      "STEER_CMD": steer_cmd,
      "LKA_ENABLE": 1,
      "SET_ME_1": 1,
    })

  def _gas_pedal(self, data2: int):
    msg = self.packer.make_can_msg_safety("GAS_PEDAL", 0, {"APPS_1": 0})
    data = bytearray(msg[0].data)
    data[2] = data2 & 0xFF
    return libsafety_py.make_CANPacket(132, 0, bytes(data))

  def test_rx_gas_pressed(self):
    self._rx(self._gas_pedal(0))
    self._rx(self._gas_pedal(10))

  def test_init_controls_not_allowed(self):
    self.assertFalse(self.safety.get_controls_allowed())

  def test_acc_buttons_enable_and_cancel(self):
    self._rx(self._acc_buttons(res=True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._acc_buttons(cancel=True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pcm_cruise_engagement(self):
    for _ in range(3):
      self._rx(self._acc_cmd(2, acc_req=True))
    self.assertTrue(self.safety.get_longitudinal_allowed())

  def test_pcm_cruise_debounce(self):
    for _ in range(3):
      self._rx(self._acc_cmd(2, acc_req=True))
    self.assertTrue(self.safety.get_longitudinal_allowed())
    self._rx(self._acc_cmd(2, acc_req=False))
    self.assertTrue(self.safety.get_longitudinal_allowed())
    for _ in range(6):
      self._rx(self._acc_cmd(2, acc_req=False))
    self.assertFalse(self.safety.get_longitudinal_allowed())

  def test_adas_lkas_steer_consistency(self):
    self.assertTrue(self._tx(self._adas_lkas(steer_req=True, steer_cmd=100)))
    bad = self._pack("ADAS_LKAS", 0, {
      "LKAS_ENGAGED1": True,
      "LKAS_LINE_ACTIVE": False,
      "STEER_CMD": 0,
      "LKA_ENABLE": 1,
      "SET_ME_1": 1,
    })
    self.assertFalse(self._tx(bad))

  def test_adas_lkas_zero_steer_when_inactive(self):
    self.assertFalse(self._tx(self._adas_lkas(steer_req=False, steer_cmd=50)))
    self.assertTrue(self._tx(self._adas_lkas(steer_req=False, steer_cmd=0)))

  def test_adas_lkas_steer_limit(self):
    self.assertFalse(self._tx(self._adas_lkas(steer_req=True, steer_cmd=600)))
    self.assertTrue(self._tx(self._adas_lkas(steer_req=True, steer_cmd=500)))

  def test_acc_cmd_conflicting_flags_blocked(self):
    self.assertFalse(self._tx(self._acc_cmd(0, acc_req=True, cruise_disabled=True)))

  def test_acc_cmd_allowed_when_consistent(self):
    self.assertTrue(self._tx(self._acc_cmd(0, acc_req=True, cruise_disabled=False)))

  def test_fwd_blocks_adas_lkas_on_camera_bus(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 432))

  def test_fwd_blocks_acc_cmd_after_device_tx(self):
    self.assertTrue(self._tx(self._acc_cmd(0, acc_req=True, cruise_disabled=False)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 417))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 417))


if __name__ == "__main__":
  unittest.main()

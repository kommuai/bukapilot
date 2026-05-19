#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety


class TestCherySafety(unittest.TestCase):
  """Chery safety: HUD cruise state on bus 2 gates controls_allowed via pcm_cruise_check."""

  # Must match opendbc/safety/modes/chery.h CHERY_TX_MSGS ([addr, bus]).
  TX_MSGS = [[837, 0], [916, 0], [864, 0]]  # LANE_KEEP, LKAS_INFO, PCM_BUTTONS

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chery, 1)
    self.safety.init_tests()
    self.packer = CANPackerSafety("chery_general_pt")

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _hud(self, cruise_state: int):
    return self.packer.make_can_msg_safety(
      "HUD",
      2,
      {
        "AEB": 0,
        "CANCEL_CRUISE_UNCERTAIN": 0,
        "GAS_RESUME_UNCERTAIN": 0,
        "FOLLOW_DISTANCE": 1,
        "NEW_SIGNAL_1": 0,
        "PCW": 0,
        "CRUISE_STATE": cruise_state,
        "GAS_OVERRIDE": 0,
        "AEB_RELATED": 0,
        "SET_SPEED": 80,
      },
    )

  def _lane_keep(self, steer_cmd_deg: float, lkas_enable: int):
    return self.packer.make_can_msg_safety(
      "LANE_KEEP",
      0,
      {
        "STEER_CMD_ANGLE": steer_cmd_deg,
        "LKAS_ENABLE": lkas_enable,
        "SET_ME_XFF": 255,
        "SET_ME_XFC": 252,
        "SET_ME_XF4": 244,
        "SET_ME_X63": 99,
        "SET_ME_XF": 15,
      },
    )

  def test_controls_allowed_follows_hud_cruise_state(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._hud(cruise_state=1))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._hud(cruise_state=3))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._hud(cruise_state=3))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._hud(cruise_state=1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_lane_keep_tx_allowed_when_whitelisted(self):
    """LANE_KEEP is on the TX whitelist; chery_tx_hook does not further gate by controls_allowed."""
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._lane_keep(0.0, 0)))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._lane_keep(1.0, 1)))

  def _lkas_info(self, main_torque: float, lkas_enable: int):
    return self.packer.make_can_msg_safety(
      "LKAS_INFO",
      0,
      {
        "MAIN_TORQUE": main_torque,
        "LKAS_ENABLE": lkas_enable,
        "STEER_RELATED": 0.0,
      },
    )

  def test_lkas_info_tx_allowed_when_whitelisted(self):
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._lkas_info(50.0, 1)))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._lkas_info(0.0, 0)))

  def test_pcm_buttons_tx_allowed_when_whitelisted(self):
    msg = self.packer.make_can_msg_safety("PCM_BUTTONS", 0, {"ICC_TOGGLE": 1, "CRUISE_BUTTON": 0})
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(msg))


if __name__ == "__main__":
  unittest.main()

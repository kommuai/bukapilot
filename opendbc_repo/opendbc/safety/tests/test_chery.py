#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety


class TestCherySafety(unittest.TestCase):
  """Chery safety: HUD cruise state on bus 2 gates controls_allowed via pcm_cruise_check."""

  # Must match opendbc/safety/modes/chery.h CHERY_TX_MSGS ([addr, bus]).
  # LANE_KEEP, TIGGO22_LK(0), LKAS_INFO(0), LKAS_INFO(2), HUD, STEER_STATUS(0), EPS(2), PCM(0), PCM(2), GAS(0), GAS(2).
  TX_MSGS = [[0x345, 0], [0x220, 0], [0x394, 0], [0x394, 2], [0x387, 0], [0x307, 0], [0x1D3, 2], [0x360, 0], [0x360, 2], [0xFA, 0], [0xFA, 2]]
  SAFETY_PARAM = 0

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chery, self.SAFETY_PARAM)
    self.safety.init_tests()
    self.packer = CANPackerSafety("chery_general_pt")

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _hud(self, cruise_state: int, bus: int = 2):
    return self.packer.make_can_msg_safety(
      "HUD",
      bus,
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
    for bus in (0, 2):
      msg = self.packer.make_can_msg_safety("PCM_BUTTONS", bus, {"RES_BUTTON": 1, "COUNTER": 0})
      self.safety.set_controls_allowed(False)
      self.assertTrue(self._tx(msg))

  def test_pt_bus_stock_adas_does_not_trigger_relay_malfunction(self):
    """LKAS_INFO and LANE_KEEP on bus 0 are expected on Jaecoo PT; must not trip relayMalfunction."""
    self.assertFalse(self.safety.get_relay_malfunction())
    self._rx(self._lkas_info(0.0, 0))
    self.assertFalse(self.safety.get_relay_malfunction())
    self._rx(self._lane_keep(0.0, 0))
    self.assertFalse(self.safety.get_relay_malfunction())
    self._tx(self._lane_keep(1.0, 1))
    self._rx(self._lane_keep(1.0, 1))
    self.assertFalse(self.safety.get_relay_malfunction())

  def test_hud_tx_allowed_when_whitelisted(self):
    msg = self.packer.make_can_msg_safety(
      "HUD", 0,
      {
        "AEB": 0,
        "HANDS_ON_WHEEL_STEER": 0,
        "CANCEL_CRUISE_UNCERTAIN": 0,
        "GAS_RESUME_UNCERTAIN": 0,
        "FOLLOW_DISTANCE": 1,
        "NEW_SIGNAL_1": 0,
        "PCW": 0,
        "CRUISE_STATE": 3,
        "GAS_OVERRIDE": 0,
        "AEB_RELATED": 0,
        "SET_SPEED": 80,
        "COUNTER": 0,
      },
    )
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(msg))

  def test_fwd_blocks_camera_lane_keep(self):
    # Camera bus -> PT bus blocks: LANE_KEEP, LKAS_INFO, HUD.
    for addr in (0x345, 0x394, 0x387):
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, addr),
                       msg=f"fwd not blocked for cam addr 0x{addr:x}")
    # LKA_STATUS (0x3a5) must be forwarded so the cluster gets LKA-engaged status.
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x3A5))
    # PT bus -> cam bus passthrough (destination bus 2) for an addr we don't TX-whitelist.
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x999))

  def _eps(self, steering_angle_deg: float, driver_torque: int):
    return self.packer.make_can_msg_safety(
      "EPS", 2,
      {
        "STEERING_ANGLE": steering_angle_deg,
        "DRIVER_TORQUE": driver_torque,
        "COUNTER": 0,
      },
    )

  def test_eps_spoof_tx_allowed_on_cam_bus(self):
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._eps(0.0, 5)))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._eps(15.0, 5)))

  def test_gas_spoof_tx_allowed_on_pt_and_cam(self):
    msg0 = self.packer.make_can_msg_safety(
      "GAS", 0, {"GAS_THROTTLE": 0.12, "GAS_PEDAL_PRESSURE": 0.12},
    )
    msg2 = self.packer.make_can_msg_safety(
      "GAS", 2, {"GAS_THROTTLE": 0.12, "GAS_PEDAL_PRESSURE": 0.12},
    )
    self.assertTrue(self._tx(msg0))
    self.assertTrue(self._tx(msg2))

  def _wheel_speed(self, fl_kph: float, fr_kph: float):
    return self.packer.make_can_msg_safety(
      "WHEELSPEED_2", 0,
      {"WHEEL_FL": fl_kph, "WHEEL_FR": fr_kph},
    )

  def test_fwd_blocks_pt_torque_when_engaged(self):
    """Per-message PT->cam blocking matches the spoof loop's gating exactly.

    EPS  (0x1D3): blocked whenever cruise engaged OR vehicle stopped (passthrough is
                  byte-identical to stock so blocking while stopped is safe).
    LKAS (0x394): blocked only while cruise is engaged.
    STEER_RELATED (0xC4): never blocked — cam watchdog cancels LKAS otherwise.
    """
    self._rx(self._wheel_speed(30.0, 30.0))
    self.safety.set_controls_allowed(False)
    for addr in (0x394, 0x1D3, 0x0C4):
      self.assertEqual(2, self.safety.safety_fwd_hook(0, addr),
                       msg=f"PT 0x{addr:x} should fwd to cam when moving and disengaged")

    self._rx(self._wheel_speed(0.0, 0.0))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x1D3),
                     msg="EPS should be blocked PT->cam when stopped (passthrough TX takes over)")
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x394),
                     msg="LKAS_INFO must fwd while stopped/disengaged")
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x0C4))

    self._rx(self._wheel_speed(30.0, 30.0))
    self.safety.set_controls_allowed(True)
    for addr in (0x394, 0x1D3):
      self.assertEqual(-1, self.safety.safety_fwd_hook(0, addr),
                       msg=f"PT 0x{addr:x} should be blocked PT->cam when engaged")
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x0C4),
                     msg="STEER_RELATED must always forward")


class TestCheryTiggoNoTorqueSpoofSafety(TestCherySafety):
  """Tiggo 8 Pro: no torque spoof + native HUD fwd (production safetyParam=10)."""

  SAFETY_PARAM = 2 | 8  # NO_TORQUE_SPOOF | NATIVE_HUD_FWD

  def test_fwd_blocks_camera_lane_keep(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x345))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x394))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x387),
                     msg="Tiggo must forward cam HUD to PT (no HUD override)")
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x3A5))

  def test_fwd_blocks_pt_torque_when_engaged(self):
    self._rx(self._wheel_speed(0.0, 0.0))
    self.safety.set_controls_allowed(False)
    for addr in (0x394, 0x1D3):
      self.assertEqual(2, self.safety.safety_fwd_hook(0, addr),
                       msg=f"PT 0x{addr:x} should fwd to cam when spoof disabled")

    self._rx(self._wheel_speed(30.0, 30.0))
    self.safety.set_controls_allowed(True)
    for addr in (0x394, 0x1D3):
      self.assertEqual(2, self.safety.safety_fwd_hook(0, addr),
                       msg=f"PT 0x{addr:x} should fwd to cam when spoof disabled")
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x0C4),
                     msg="STEER_RELATED must always forward")


class TestCheryTiggo22Safety(TestCheryTiggoNoTorqueSpoofSafety):
  """Tiggo 8 Pro 2022-24: HUD on bus 1, 0x220 LKAS on bus 2 (production safetyParam=26)."""

  SAFETY_PARAM = 2 | 8 | 16  # NO_TORQUE_SPOOF | NATIVE_HUD_FWD | TIGGO22

  def _tiggo22_lane_keep(self, steer_state: int, bus: int = 2):
    return self.packer.make_can_msg_safety(
      "TIGGO22_LANE_KEEP",
      bus,
      {
        "STEER_CMD": 0,
        "STEER_STATE": steer_state,
        "COUNTER": 0,
      },
    )

  def _lka_status(self, acc_enable: int, bus: int = 0):
    return self.packer.make_can_msg_safety(
      "LKA_STATUS",
      bus,
      {"TIGGO_8_ACC_ENABLE": acc_enable},
    )

  def _tiggo22_meter(self, set_speed: int = 60, distance_bar: int = 0):
    return self.packer.make_can_msg_safety(
      "TIGGO8PRO_2022_METER",
      0,
      {"SET_SPEED": set_speed, "DISTANCE_BAR": distance_bar, "DISTANCE_BAR_2": 0},
    )

  def test_controls_allowed_follows_hud_cruise_state(self):
    """Tiggo 2022-24 must not gate on HUD (layout differs; CarState ignores it)."""
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._hud(3, bus=1))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._hud(3, bus=2))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_controls_allowed_follows_lka_status_acc_enable(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._tiggo22_lane_keep(2))
    self.assertFalse(self.safety.get_controls_allowed(),
                     msg="cam 0x220 STEER_STATE=2 must not arm controls_allowed")
    self._rx(self._lka_status(0, bus=2))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._wheel_speed(30.0, 30.0))
    self._rx(self._lka_status(1))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._lka_status(0))
    self.assertFalse(self.safety.get_controls_allowed(),
                     msg="moving: acc drop clears controls_allowed immediately")

  def test_controls_allowed_holds_acc_enable_flicker_at_standstill(self):
    self._rx(self._wheel_speed(0.0, 0.0))
    self._rx(self._tiggo22_meter(set_speed=60, distance_bar=0))
    self._rx(self._lka_status(1))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._lka_status(0))
    self.assertTrue(self.safety.get_controls_allowed(),
                     msg="standstill flicker should keep controls_allowed")
    self._rx(self._tiggo22_meter(set_speed=60, distance_bar=3))
    self._rx(self._lka_status(0))
    self.assertFalse(self.safety.get_controls_allowed(),
                     msg="meter ACC-off should clear controls_allowed")

  def test_fwd_blocks_pt_gas_for_tiggo22(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0xFA),
                     msg="stock GAS must not forward PT->cam on Tiggo 2022-24")

  def test_fwd_blocks_camera_lane_keep(self):
    # 0x345 is the generic Chery LKAS command and is always replaced on PT.
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x345))

    self.safety.set_controls_allowed(False)
    for addr in (0x220, 0x307, 0x387, 0x394, 0x3A5, 0x999):
      self.assertEqual(0, self.safety.safety_fwd_hook(2, addr),
                       msg=f"cam 0x{addr:x} must passthrough when not engaged")
    self.safety.set_controls_allowed(True)
    for addr in (0x220, 0x307):
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, addr),
                       msg=f"cam 0x{addr:x} must be blocked when engaged")
    for addr in (0x387, 0x394, 0x3A5, 0x999):
      self.assertEqual(0, self.safety.safety_fwd_hook(2, addr),
                       msg=f"cam 0x{addr:x} must fwd to PT")


class TestCheryOmodaSafety(TestCherySafety):
  SAFETY_PARAM = 1

  def test_fwd_blocks_camera_lane_keep(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x345))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x394))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x387),
                     msg="Omoda must forward cam HUD to PT (no HUD override)")
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x3A5))

  def test_controls_allowed_follows_hud_cruise_state(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._hud(1, 0))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._hud(3, 0))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._hud(3, 2))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._hud(1, 0))
    self.assertFalse(self.safety.get_controls_allowed())


class TestCheryOmodaNoTorqueSpoofSafety(TestCheryOmodaSafety):
  SAFETY_PARAM = 3

  def test_fwd_blocks_camera_lane_keep(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x345))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x394))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x387),
                     msg="Omoda must forward cam HUD to PT (no HUD override)")
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x3A5))

  def test_fwd_blocks_pt_torque_when_engaged(self):
    self.test_fwd_allows_pt_torque_when_spoof_disabled()

  def test_fwd_allows_pt_torque_when_spoof_disabled(self):
    self._rx(self._wheel_speed(0.0, 0.0))
    self.safety.set_controls_allowed(False)
    for addr in (0x394, 0x1D3):
      self.assertEqual(2, self.safety.safety_fwd_hook(0, addr),
                       msg=f"PT 0x{addr:x} should fwd to cam when spoof disabled")

    self._rx(self._wheel_speed(30.0, 30.0))
    self.safety.set_controls_allowed(True)
    for addr in (0x394, 0x1D3):
      self.assertEqual(2, self.safety.safety_fwd_hook(0, addr),
                       msg=f"PT 0x{addr:x} should fwd to cam when spoof disabled")

  def test_fwd_allows_cam_lkas_to_pt_when_spoof_disabled(self):
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x394))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x345))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x387))


class TestCheryIcaurSafety(TestCherySafety):
  """iCaur 03: standstill from 0x222; HUD forwards like Omoda (no override)."""

  SAFETY_PARAM = 4

  def _icaur_wheel_speed(self, fl_raw: int, fr_raw: int):
    return self.packer.make_can_msg_safety(
      "ICAUR_WHEELSPEED_A", 0,
      {
        "WHEEL_FL": fl_raw,
        "WHEEL_FR": fr_raw,
      },
    )

  def test_fwd_blocks_camera_lane_keep(self):
    # LANE_KEEP still blocked/replaced; HUD + LKAS_INFO (with torque spoof on in this
    # test param) follow Jaecoo torque path for LKAS, but HUD must forward (Omoda-style).
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x345))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x394))  # param 4: spoof enabled
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x387),
                     msg="iCaur must forward cam HUD to PT (no HUD override)")
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x3A5))

  def test_fwd_blocks_pt_torque_when_engaged(self):
    # 13-bit raw: 960 ↔ former 8-bit byte0=30 (high 8 bits); must leave byte0 >= 15.
    self._rx(self._icaur_wheel_speed(960, 960))
    self.safety.set_controls_allowed(False)
    for addr in (0x394, 0x1D3, 0x0C4):
      self.assertEqual(2, self.safety.safety_fwd_hook(0, addr),
                       msg=f"PT 0x{addr:x} should fwd to cam when moving and disengaged")

    self._rx(self._icaur_wheel_speed(0, 0))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x1D3),
                     msg="EPS should be blocked PT->cam when stopped")
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x394))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x0C4))

    self._rx(self._icaur_wheel_speed(960, 960))
    self.safety.set_controls_allowed(True)
    for addr in (0x394, 0x1D3):
      self.assertEqual(-1, self.safety.safety_fwd_hook(0, addr),
                       msg=f"PT 0x{addr:x} should be blocked PT->cam when engaged")
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x0C4))


class TestCheryIcaurNoTorqueSpoofSafety(TestCheryIcaurSafety):
  """Matches production iCaur safetyParam (ICAUR | NO_TORQUE_SPOOF)."""

  SAFETY_PARAM = 4 | 2  # CHERY_ICAUR_SAFETY_PARAM | CHERY_OMODA_NO_TORQUE_SPOOF_PARAM

  def test_fwd_blocks_camera_lane_keep(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x345))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x394))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x387))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x3A5))

  def test_fwd_blocks_pt_torque_when_engaged(self):
    self._rx(self._icaur_wheel_speed(0, 0))
    self.safety.set_controls_allowed(True)
    for addr in (0x394, 0x1D3, 0x0C4):
      self.assertEqual(2, self.safety.safety_fwd_hook(0, addr),
                       msg=f"PT 0x{addr:x} should fwd when torque spoof disabled")


if __name__ == "__main__":
  unittest.main()

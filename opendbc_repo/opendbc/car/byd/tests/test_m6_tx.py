#!/usr/bin/env python3
import unittest
import sys
import types
from types import SimpleNamespace

import cereal
from cereal import car

try:
  import cereal.messaging  # noqa: F401
except ModuleNotFoundError as exc:
  if exc.name != "msgq.ipc_pyx":
    raise
  messaging = types.ModuleType("cereal.messaging")
  messaging.SubMaster = lambda *_args, **_kwargs: None
  cereal.messaging = messaging
  sys.modules["cereal.messaging"] = messaging
from opendbc.car.byd.cam_lka.carcontroller import CarController
from opendbc.car.byd.interface import CarInterface
from opendbc.car.byd.values import CAR, DBC


def run_controller(candidate):
  cp = CarInterface.get_params(candidate, [], [], False, False, False)
  controller = CarController(DBC[candidate], cp)
  cs = SimpleNamespace(
    out=SimpleNamespace(
      steeringAngleDeg=0.0,
      standstill=False,
      steeringTorque=0.0,
      vEgo=10.0,
      gasPressed=False,
      brakePressed=False,
    ),
    lkas_rdy_btn=False,
    res_btn=False,
    sng_pcm_idle=None,
    cruise_standstill=False,
    standstill_wait=False,
    lkas_healthy=True,
    lss_state=0,
    lss_alert=0,
    tsr=0,
    HMA=0,
    pt2=0,
    pt3=0,
    pt4=0,
    pt5=0,
    lkas_hud_status_passthrough=0,
    lka_on=0,
  )
  control = car.CarControl.new_message()
  control.enabled = True
  control.actuators.accel = 1.0
  _, sends = controller.update(control.as_reader(), cs, 0)
  return [(int(msg[0]), int(msg[2])) for msg in sends]


class TestBydM6ControllerTx(unittest.TestCase):
  def test_only_m6_gets_the_new_safety_profile(self):
    m6 = CarInterface.get_params(CAR.BYD_M6, [], [], False, False, False)
    seal6 = CarInterface.get_params(CAR.BYD_SEAL6, [], [], False, False, False)
    atto3 = CarInterface.get_params(CAR.BYD_ATTO3, [], [], False, False, False)
    self.assertEqual(7, m6.safetyConfigs[0].safetyParam)
    self.assertEqual(3, seal6.safetyConfigs[0].safetyParam)
    self.assertEqual(1, atto3.safetyConfigs[0].safetyParam)

  def test_m6_does_not_send_stock_torque_or_accel_frames(self):
    sent = run_controller(CAR.BYD_M6)
    sent_addrs = {addr for addr, _bus in sent}
    self.assertNotIn(0x1FC, sent_addrs)
    self.assertNotIn(0x32E, sent_addrs)

  def test_atto3_send_behavior_is_unchanged(self):
    sent = run_controller(CAR.BYD_ATTO3)
    self.assertIn((0x1FC, 2), sent)
    self.assertIn((0x32E, 0), sent)


if __name__ == "__main__":
  unittest.main()

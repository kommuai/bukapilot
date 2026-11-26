#!/usr/bin/env python3
import unittest
from collections import defaultdict
import sys
import types
from types import SimpleNamespace

import cereal
from cereal import car

# The no-output controller test never calls SngHelper.lead_ready(), but importing
# the controller imports cereal.messaging. Allow this focused unit test to run in
# environments where the native msgq extension has not been built.
try:
  import cereal.messaging  # noqa: F401
except ModuleNotFoundError as exc:
  if exc.name != "msgq.ipc_pyx":
    raise
  messaging = types.ModuleType("cereal.messaging")
  messaging.SubMaster = lambda *_args, **_kwargs: None
  cereal.messaging = messaging
  sys.modules["cereal.messaging"] = messaging

from opendbc.car import Bus
from opendbc.car.byd.cam_lka.carcontroller import CarController
from opendbc.car.byd.cam_lka.carstate import CarState
from opendbc.car.byd.interface import CarInterface
from opendbc.car.byd.values import CAR, CANBUS, DBC, BYD_SEALION8_DMI_SAFETY_PARAM


class TestSealion8Dmi(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.CP = CarInterface.get_params(CAR.BYD_SEALION8_DMI, [], [], False, False, False)

  def test_dmi_state_fields_decode(self):
    pt = SimpleNamespace(vl=defaultdict(lambda: defaultdict(float)))
    pt.ts_nanos = defaultdict(lambda: defaultdict(int))
    cam = SimpleNamespace(vl=defaultdict(lambda: defaultdict(float)))

    pt.vl["DRIVE_STATE"]["GEAR"] = 4
    pt.vl["DRIVE_STATE"]["RAW_THROTTLE"] = 7
    pt.vl["METER_CLUSTER"]["SEATBELT_DRIVER"] = 1
    pt.vl["PEDAL"]["POWER_MODE"] = 1
    pt.vl["PEDAL"]["DRIVE_MODE"] = 3
    pt.vl["PEDAL"]["STILL_GENERATOR"] = 1
    pt.vl["ECM"]["ENG_RPM"] = 875.0
    pt.vl["EPB_NG"]["EPB_ACTIVE"] = 2
    pt.vl["EPB_NG"]["EPB_STATUS"] = 3
    pt.vl["ESC"]["VEHICLE_SPEED"] = 47.27
    pt.vl["IPB"]["BRAKE_PRESSED"] = 1
    pt.ts_nanos["IPB"]["BRAKE_PRESSED"] = 1
    pt.vl["PEDAL_PRESSED"]["PEDAL_PRESSED_ACTIVE_LOW"] = 1
    pt.ts_nanos["PEDAL_PRESSED"]["PEDAL_PRESSED_ACTIVE_LOW"] = 1
    pt.vl["ACCEL_SENSOR"]["LATERAL"] = -0.1
    pt.vl["ACC_AEB"]["PAYLOAD"] = 123456
    pt.vl["ACCEL_SENSOR"]["LONGITUDE"] = 0.25
    pt.vl["ACC_HUD_ADAS"]["ERROR"] = 3
    pt.vl["ACC_HUD_ADAS"]["SET_SPEED"] = 59
    pt.vl["ACC_HUD_ADAS"]["AEB"] = 1
    pt.vl["ACC_HUD_ADAS"]["FCW"] = 1
    pt.vl["ACC_HUD_ADAS"]["SET_DISTANCE"] = 2
    pt.vl["ACC_HUD_ADAS"]["HAS_LEAD"] = 1
    pt.vl["ACC_HUD_ADAS"]["LEAD_DISTANCE"] = 4
    pt.vl["ACC_HUD_ADAS"]["TOO_CLOSE"] = 1
    pt.vl["ACC_HUD_ADAS"]["CRUISE_STATE"] = 8
    pt.vl["ACC_HUD_ADAS"]["ACC_ON1"] = 1
    pt.vl["ACC_HUD_ADAS"]["ACC_ON2"] = 1
    pt.vl["ACC_HUD_ADAS"]["ACC_CONTROLLABLE_AND_ON"] = 1
    pt.vl["ACC_HUD_ADAS"]["OVERRIDE"] = 1
    pt.vl["ACC_CMD"]["ACCEL_CMD"] = -3.0
    pt.vl["ACC_CMD"]["ACC_CONTROLLABLE_AND_ON"] = 1
    pt.vl["ACC_CMD"]["ACC_REQ_NOT_STANDSTILL"] = 1
    cam.vl["LKAS_HUD_ADAS"]["LKAS_ENABLED"] = 1
    cam.vl["LKAS_HUD_ADAS"]["STEER_ACTIVE_ACTIVE_LOW"] = 1
    cam.vl["LKAS_HUD_ADAS"]["HAND_ON_WHEEL_WARNING"] = 1
    cam.vl["LKAS_HUD_ADAS"]["LSS_STATE"] = 2
    cam.vl["STEERING_MODULE_ADAS"]["EPS_OK"] = 1
    cam.vl["STEERING_MODULE_ADAS"]["STEER_REQ"] = 1
    pt.vl["STALKS"]["EPB_ACTIVE"] = 1
    pt.vl["YAW_SENSOR"]["YAW_RATE"] = 0.05
    pt.vl["STEER_MODULE_2"]["DRIVER_EPS_TORQUE"] = 12

    cs = CarState(self.CP)
    out = cs.update({Bus.pt: pt, Bus.cam: cam})

    self.assertEqual(1, out.bydState.powerModeRaw)
    self.assertEqual(3, out.bydState.driveModeRaw)
    self.assertEqual(1, out.bydState.stillGeneratorRaw)
    self.assertEqual(875.0, out.bydState.engineRpm)
    self.assertEqual(123456, out.bydState.accAebPayloadRaw)
    self.assertAlmostEqual(-0.1, out.bydState.lateralAccel)
    self.assertEqual(3, out.bydState.accErrorRaw)
    self.assertEqual(2, out.bydState.epbActiveRaw)
    self.assertEqual(3, out.bydState.epbStatusRaw)
    self.assertAlmostEqual(47.27, out.bydState.escVehicleSpeedRaw, places=5)
    self.assertEqual(1, out.bydState.ipbBrakePressedRaw)
    self.assertEqual(1, out.bydState.pedalPressedActiveLowRaw)
    self.assertAlmostEqual(59.0, out.bydState.accSetSpeedRaw)
    self.assertEqual(2, out.bydState.accSetDistanceRaw)
    self.assertTrue(out.bydState.accHasLead)
    self.assertEqual(4, out.bydState.accLeadDistanceRaw)
    self.assertTrue(out.bydState.accTooClose)
    self.assertEqual(1, out.bydState.accCmdControllableRaw)
    self.assertEqual(1, out.bydState.accCmdReqNotStandstillRaw)
    self.assertEqual(12, out.bydState.driverEpsTorqueOrRateRaw)
    self.assertEqual(8, out.bydState.accCruiseStateRaw)
    self.assertEqual(1, out.bydState.accHudOn1Raw)
    self.assertEqual(1, out.bydState.accHudOn2Raw)
    self.assertEqual(1, out.bydState.accHudControllableRaw)
    self.assertEqual(1, out.bydState.accHudOverrideRaw)
    self.assertEqual(7, out.bydState.driveStateThrottleRaw)
    self.assertEqual(1, out.bydState.lkaHudSteerActiveActiveLowRaw)
    self.assertEqual(1, out.bydState.lkaHudHandOnWheelWarningRaw)
    self.assertEqual(2, out.bydState.lkaHudLssStateRaw)
    self.assertEqual(1, out.bydState.epsOkRaw)
    self.assertEqual(1, out.bydState.steerReqRaw)
    self.assertAlmostEqual(0.25, out.aEgo)
    self.assertTrue(out.parkingBrake)
    self.assertTrue(out.brakePressed)
    self.assertFalse(out.steeringPressed)
    self.assertTrue(out.stockLkas)
    self.assertAlmostEqual(0.05, out.yawRate)
    self.assertTrue(out.stockAeb)
    self.assertTrue(out.stockFcw)
    self.assertEqual(-3.0, out.stockAccelCmd)

    pt.vl["IPB"]["BRAKE_PRESSED"] = 0
    pt.vl["PEDAL_PRESSED"]["PEDAL_PRESSED_ACTIVE_LOW"] = 0
    self.assertTrue(CarState(self.CP).update({Bus.pt: pt, Bus.cam: cam}).brakePressed)

  def test_diagnostic_profile_cannot_send_control_frames(self):
    self.assertTrue(self.CP.dashcamOnly)
    self.assertFalse(self.CP.openpilotLongitudinalControl)
    self.assertEqual(car.CarParams.SafetyModel.byd, self.CP.safetyConfigs[0].safetyModel)
    self.assertEqual(BYD_SEALION8_DMI_SAFETY_PARAM, self.CP.safetyConfigs[0].safetyParam)

    controller = CarController(DBC[CAR.BYD_SEALION8_DMI], self.CP)
    self.assertIsNone(controller.sng)
    control = car.CarControl.new_message()
    control.enabled = True
    control.latActive = True
    control.longActive = True
    control.actuators.steeringAngleDeg = 5.0
    control.actuators.accel = 1.0

    _, sends = controller.update(control.as_reader(), None, 0)
    self.assertEqual([], sends)

  def test_sealion8_uses_captured_wheel_speed_and_status_buses(self):
    parsers = CarState.get_can_parsers(self.CP)
    self.assertIn("WHEEL_SPEED_ALT", [state.name for state in parsers[Bus.pt].message_states.values()])
    self.assertNotIn("WHEEL_SPEED", [state.name for state in parsers[Bus.pt].message_states.values()])
    self.assertEqual(CANBUS.main_bus, parsers[Bus.cam].bus)


if __name__ == "__main__":
  unittest.main()

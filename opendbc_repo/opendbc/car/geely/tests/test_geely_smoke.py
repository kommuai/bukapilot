import pytest
from cereal import car

from opendbc.can import CANParser
from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.can_definitions import CanData
from opendbc.car.geely import geelycan
from opendbc.car.geely.carcontroller import CarController
from opendbc.car.geely.carstate import CarState
from opendbc.car.geely.interface import CarInterface
from opendbc.car.geely.values import CAR, DBC, PARSER_MSGS, geely_steering_deg_sign


@pytest.fixture
def cp():
  return CarInterface.get_params(CAR.GEELY_GALAXY_E5, {0: {}}, [], False, False, False)


def test_get_params(cp):
  assert cp.brand == "geely"
  assert cp.steerControlType == car.CarParams.SteerControlType.angle
  assert not cp.openpilotLongitudinalControl
  assert cp.radarUnavailable
  assert cp.safetyConfigs[0].safetyModel == car.CarParams.SafetyModel.allOutput


def _synthetic_frames(cp):
  packer = CANPacker(DBC[cp.carFingerprint]["pt"])
  frames = []
  for name, _ in PARSER_MSGS:
    if name == "PEDALS":
      vals = {"BRAKE_PRESSURE": 0.0, "BRAKE_PRESSURE2": 0.0}
    elif name == "EPS":
      vals = {"STEERING_ANGLE": 5.0, "EPS_TORQUE_SIGNED": 0.0, "COUNTER": 0}
    elif name == "WHEELSPEED_1":
      vals = {"WHEEL_BL": 10.0, "WHEEL_BR": 10.0, "COUNTER": 0}
    elif name == "WHEELSPEED_2":
      vals = {"WHEEL_FL": 10.0, "WHEEL_FR": 10.0, "COUNTER": 0}
    elif name == "LKAS":
      vals = {"CRUISE_STATE": 7, "STEER_CMD": 0.0, "COUNTER": 0}
    elif name == "STEER_RELATED":
      vals = {"STEERING_RELATED": 0}
    else:
      vals = {}
    addr, dat, bus = packer.make_can_msg(name, 0, vals)
    frames.append(CanData(address=addr, dat=dat, src=bus))
  return frames


def test_carstate_update(cp):
  cs = CarState(cp)
  parser = CarState.get_can_parser(cp)
  frames = _synthetic_frames(cp)
  parser.update([(0, frames)])
  ret = cs.update({Bus.pt: parser})
  assert ret.steeringAngleDeg == pytest.approx(5.0)
  assert not ret.brakePressed
  assert not ret.gasPressed
  assert not ret.doorOpen
  assert ret.cruiseState.enabled


def test_carcontroller_lkas_tx(cp):
  cs = CarState(cp)
  cc = CarController(DBC[cp.carFingerprint], cp)
  parser = CarState.get_can_parser(cp)
  frames = _synthetic_frames(cp)
  parser.update([(0, frames)])
  cs.out = cs.update({Bus.pt: parser})

  CC = car.CarControl.new_message()
  CC.latActive = True
  CC.enabled = True
  CC.actuators = car.CarControl.Actuators.new_message()
  CC.actuators.steeringAngleDeg = 10.0

  all_sends = []
  for _ in range(4):
    actuators, can_sends = cc.update(CC, cs, 0)
    all_sends.extend(can_sends)
    cs.out.steeringAngleDeg = actuators.steeringAngleDeg
    assert not cs.out.standstill

  lkas = [m for m in all_sends if m[0] == 0x33]
  assert len(lkas) >= 1
  assert lkas[0][2] == 0


def test_lkas_checksum_rejects_other_addresses():
  with pytest.raises(ValueError, match="0x33"):
    geelycan.geely_checksum(0x40, None, bytearray(8))


def test_lkas_checksum(cp):
  packer = CANPacker(DBC[cp.carFingerprint]["pt"])
  sign = geely_steering_deg_sign(cp)
  _, dat, _ = geelycan.create_lkas_command(packer, 5.0, True, 0.0, sign, 3)
  d = bytearray(dat)
  assert geelycan.geely_lkas_0x33_checksum(d) == geelycan.geely_checksum(0x33, None, d)
  # packed frame must pass parser checksum validation
  parser = CANParser(DBC[cp.carFingerprint]["pt"], [("LKAS", 50)], 0)
  parser.update([(0, [CanData(address=0x33, dat=dat, src=0)])])
  assert parser.can_valid


def test_lkas_1bit_counters_toggle(cp):
  packer = CANPacker(DBC[cp.carFingerprint]["pt"])
  _, d0, _ = geelycan.create_lkas_command(packer, 0.0, True, 0.0, counter_1bit_1=0, counter_1bit_2=0)
  _, d1, _ = geelycan.create_lkas_command(packer, 0.0, True, 0.0, counter_1bit_1=1, counter_1bit_2=1)
  assert d0 != d1


def test_steer_cmd_monotonic(cp):
  packer = CANPacker(DBC[cp.carFingerprint]["pt"])
  sign = geely_steering_deg_sign(cp)
  a = geelycan.create_lkas_command(packer, 0.0, True, 0.0, sign, 0)
  b = geelycan.create_lkas_command(packer, 10.0, True, 0.0, sign, 1)
  assert a[0] == 51 and b[0] == 51
  assert a[1] != b[1]

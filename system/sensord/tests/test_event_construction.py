from types import SimpleNamespace

from cereal import log
from openpilot.system.sensord.sensors.i2c_sensor import Sensor
from openpilot.system.sensord.sensors.icm42670_temp import ICM42670_Temp
from openpilot.system.sensord.sensors.lis2mdl_magn import LIS2MDL_Magn
from openpilot.system.sensord.sensors.lsm6ds3_temp import LSM6DS3_Temp


def expected_event(timestamp, version, source, union_name, payload, *, sensor=None, sensor_type=None, status=None):
  event = log.SensorEventData.new_message()
  event.timestamp = timestamp
  event.version = version
  if sensor is not None:
    event.sensor = sensor
  if sensor_type is not None:
    event.type = sensor_type
  event.source = source
  if union_name == "temperature":
    event.temperature = payload
  else:
    measurement = event.init(union_name)
    measurement.v = payload
    if status is not None:
      measurement.status = status
  return event


def test_icm42670_temperature_event_keeps_unset_sensor_and_explicit_type():
  sensor = object.__new__(ICM42670_Temp)
  sensor.bus = SimpleNamespace(close=lambda: None)
  sensor.source = log.SensorEventData.SensorSource.icm42670
  sensor.read = lambda _address, _length: bytes((0x00, 0x80))
  timestamp = 123456789

  event = sensor.get_event(timestamp)

  expected = expected_event(timestamp, 1, sensor.source, "temperature", 26.0, sensor_type=4)
  assert event.which() == "temperature"
  assert event.sensor == 0  # scalar field default: the baseline left it unset
  assert event.type == 4
  assert event.to_dict() == expected.to_dict()
  assert event.to_bytes() == expected.to_bytes()


def test_lsm6ds3_temperature_event_keeps_ignored_timestamp_and_unset_ids(monkeypatch):
  source = log.SensorEventData.SensorSource.lsm6ds3trc
  sensor = object.__new__(LSM6DS3_Temp)
  sensor.bus = SimpleNamespace(close=lambda: None)
  sensor.source = source
  sensor.read = lambda _address, _length: bytes((0x00, 0x01))
  monotonic_value = 123.456789123
  monkeypatch.setattr("openpilot.system.sensord.sensors.lsm6ds3_temp.time.monotonic", lambda: monotonic_value)

  event = sensor.get_event(987654321)

  timestamp = int(monotonic_value * 1e9)
  expected = expected_event(timestamp, 1, source, "temperature", 26.0)
  assert event.which() == "temperature"
  assert event.timestamp == timestamp
  assert event.timestamp != 987654321
  assert event.sensor == expected.sensor == 0
  assert event.type == expected.type == 0
  assert event.to_dict() == expected.to_dict()
  assert event.to_bytes() == expected.to_bytes()


def test_lis2mdl_event_keeps_timestamp_metadata_axis_order_and_status():
  sensor = object.__new__(LIS2MDL_Magn)
  sensor.bus = SimpleNamespace(close=lambda: None)
  sensor.source = log.SensorEventData.SensorSource.lis2mdl
  # Existing wire mapping: y comes from word 0, x from -word 1, z from -word 2.
  data = pack_s16(-200) + pack_s16(100) + pack_s16(300)
  sensor.read = lambda _address, _length: data
  timestamp = 24681012

  event = sensor.get_event(timestamp)

  scale = 0.15
  values = [
    -Sensor.parse_16bit(data[3], data[2]) * scale,
    Sensor.parse_16bit(data[1], data[0]) * scale,
    -Sensor.parse_16bit(data[5], data[4]) * scale,
  ]
  expected = expected_event(timestamp, 1, sensor.source, "magneticUncalibrated", values,
                            sensor=3, sensor_type=14, status=1)
  assert event.which() == "magneticUncalibrated"
  assert list(event.magneticUncalibrated.v) == list(expected.magneticUncalibrated.v)
  assert event.magneticUncalibrated.status == 1
  assert event.timestamp == timestamp
  assert event.version == 1
  assert event.sensor == 3
  assert event.type == 14
  assert event.source == sensor.source
  assert event.to_dict() == expected.to_dict()
  assert event.to_bytes() == expected.to_bytes()


def pack_s16(value: int) -> bytes:
  encoded = value & 0xFFFF
  return bytes((encoded & 0xFF, encoded >> 8))

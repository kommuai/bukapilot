import math
from types import SimpleNamespace

from cereal import log
from openpilot.system.sensord.sensors.i2c_sensor import Sensor
from openpilot.system.sensord.sensors.icm42670_accel import ICM42670_Accel
from openpilot.system.sensord.sensors.icm42670_gyro import ICM42670_Gyro


def pack_s16(value: int) -> bytes:
  encoded = value & 0xFFFF
  return bytes((encoded & 0xFF, encoded >> 8))


def expected_event(timestamp, version, sensor, sensor_type, source, union_name, values, status):
  event = log.SensorEventData.new_message()
  event.timestamp = timestamp
  event.version = version
  event.sensor = sensor
  event.type = sensor_type
  event.source = source
  measurement = event.init(union_name)
  measurement.v = values
  measurement.status = status
  return event


def test_icm42670_accel_fixed_scale_and_rotation_match_previous_formula():
  raw_x, raw_y, raw_z = 1234, -2345, 32767
  data = pack_s16(raw_y) + pack_s16(raw_z) + pack_s16(raw_x)
  sensor = object.__new__(ICM42670_Accel)
  sensor.bus = SimpleNamespace(close=lambda: None)
  sensor.source = log.SensorEventData.SensorSource.icm42670
  sensor.read = lambda _address, _length: data
  timestamp = 123456

  event = sensor.get_event(timestamp)

  # This is the pre-refactor order of operations and axis mapping.
  scale = 9.80665 / 16384.0
  x_raw = Sensor.parse_16bit(data[5], data[4]) * scale
  y_raw = -Sensor.parse_16bit(data[1], data[0]) * scale
  z_raw = -Sensor.parse_16bit(data[3], data[2]) * scale
  cosine, sine = math.cos(-0.4082), math.sin(-0.4082)
  expected = expected_event(timestamp, 1, 1, 1, sensor.source, "acceleration",
                            [cosine * x_raw - sine * z_raw, y_raw, sine * x_raw + cosine * z_raw], 1)
  assert event.to_bytes() == expected.to_bytes()


def test_icm42670_gyro_fixed_conversion_and_rotation_match_previous_formula():
  raw_x, raw_y, raw_z = 1234, -2345, 32767
  timestamp = 7654321
  sensor = object.__new__(ICM42670_Gyro)
  sensor.bus = SimpleNamespace(close=lambda: None)
  sensor.source = log.SensorEventData.SensorSource.icm42670
  sensor.bias_counts = (1.5, -2.25, 3.0)
  sensor._read_raw_counts = lambda: (raw_x, raw_y, raw_z)

  event = sensor.get_event(timestamp)

  def baseline_counts_to_rad(counts):
    return counts * (math.pi / 180.0) / 131.0

  gx_raw = baseline_counts_to_rad(raw_x - sensor.bias_counts[0])
  gy_raw = baseline_counts_to_rad(raw_y - sensor.bias_counts[1])
  gz_raw = baseline_counts_to_rad(raw_z - sensor.bias_counts[2])
  cosine, sine = math.cos(-0.4082), math.sin(-0.4082)
  expected_values = [cosine * gx_raw - sine * gz_raw, gy_raw, sine * gx_raw + cosine * gz_raw]
  expected = expected_event(timestamp, 1, 5, 16, sensor.source, "gyroUncalibrated", expected_values, 1)
  assert event.to_bytes() == expected.to_bytes()

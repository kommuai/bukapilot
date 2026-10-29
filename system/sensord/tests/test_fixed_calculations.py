import math
from types import SimpleNamespace

from cereal import log
from openpilot.system.sensord.sensors.i2c_sensor import Sensor
from openpilot.system.sensord.sensors.icm42670_accel import ICM42670_Accel
from openpilot.system.sensord.sensors.icm42670_gyro import ICM42670_Gyro
from openpilot.system.sensord.sensors.lsm6ds3_accel import LSM6DS3_Accel
from openpilot.system.sensord.sensors.lsm6ds3_gyro import LSM6DS3_Gyro
from openpilot.system.sensord.sensors.mmc5603nj_magn import MMC5603NJ_Magn, RESET, SET


def pack_s16(value: int) -> bytes:
  encoded = value & 0xFFFF
  return bytes((encoded & 0xFF, encoded >> 8))


def pack_mmc_axes(values: tuple[int, int, int]) -> bytes:
  data = bytearray(9)
  indices = ((0, 1, 6), (2, 3, 7), (4, 5, 8))
  for value, (high, middle, low) in zip(values, indices, strict=True):
    encoded = (value << 4) & 0xFFFFFF
    data[high] = (encoded >> 16) & 0xFF
    data[middle] = (encoded >> 8) & 0xFF
    data[low] = encoded & 0xFF
  return bytes(data)


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


def test_lsm6ds3_accel_fixed_scale_matches_previous_formula():
  raw_x, raw_y, raw_z = 1234, -2345, 32767
  data = pack_s16(raw_x) + pack_s16(raw_y) + pack_s16(raw_z)
  sensor = object.__new__(LSM6DS3_Accel)
  sensor.bus = SimpleNamespace(close=lambda: None)
  sensor.source = log.SensorEventData.SensorSource.lsm6ds3trc
  sensor.read = lambda address, _length: bytes((1,)) if address == sensor.LSM6DS3_ACCEL_I2C_REG_STAT_REG else data
  timestamp = 456789

  event = sensor.get_event(timestamp)

  scale = 9.81 * 2.0 / (1 << 15)
  x = Sensor.parse_16bit(data[0], data[1]) * scale
  y = Sensor.parse_16bit(data[2], data[3]) * scale
  z = Sensor.parse_16bit(data[4], data[5]) * scale
  expected = expected_event(timestamp, 1, 1, 1, sensor.source, "acceleration", [y, -x, z], 1)
  assert event.to_bytes() == expected.to_bytes()


def test_lsm6ds3_gyro_fixed_scale_matches_previous_formula():
  raw_x, raw_y, raw_z = 1234, -2345, -32768
  data = pack_s16(raw_x) + pack_s16(raw_y) + pack_s16(raw_z)
  sensor = object.__new__(LSM6DS3_Gyro)
  sensor.bus = SimpleNamespace(close=lambda: None)
  sensor.source = log.SensorEventData.SensorSource.lsm6ds3
  sensor.read = lambda address, _length: bytes((2,)) if address == sensor.LSM6DS3_GYRO_I2C_REG_STAT_REG else data
  timestamp = 987654

  event = sensor.get_event(timestamp)

  scale = (8.75 / 1000.0) * (math.pi / 180.0)
  x = Sensor.parse_16bit(data[0], data[1])
  y = Sensor.parse_16bit(data[2], data[3])
  z = Sensor.parse_16bit(data[4], data[5])
  expected = expected_event(timestamp, 2, 5, 16, sensor.source, "gyroUncalibrated", [y * scale, -x * scale, z * scale], 1)
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


def test_mmc5603_fixed_scale_and_set_reset_order_match_previous_formula(monkeypatch):
  set_data = (16384, 12000, 7777)
  reset_data = (0, 20000, 32768)
  packets = {SET: pack_mmc_axes(set_data), RESET: pack_mmc_axes(reset_data)}
  cycles = []
  sensor = object.__new__(MMC5603NJ_Magn)
  sensor.bus = SimpleNamespace(close=lambda: None)
  sensor.write = lambda _address, cycle: cycles.append(cycle)
  sensor.wait = lambda: None
  sensor.read = lambda _address, _length: packets[cycles[-1]]
  timestamp = 112233
  monkeypatch.setattr("openpilot.system.sensord.sensors.mmc5603nj_magn.time.monotonic_ns", lambda: timestamp)

  event = sensor.get_event(999)

  scale = 1.0 / 16384.0
  # Preserve the old call-site byte order and SET/RESET concatenation explicitly.
  expected_values = []
  for source_values in (set_data, reset_data):
    expected_values.extend((value * scale) - 32.0 for value in source_values)
  expected_status = int(all(int(value) != -32 for value in expected_values))
  expected = expected_event(timestamp, 1, 3, 14, log.SensorEventData.SensorSource.mmc5603nj,
                            "magneticUncalibrated", expected_values, expected_status)
  assert cycles == [SET, RESET]
  assert event.to_bytes() == expected.to_bytes()

import math
import time

from cereal import log
from openpilot.system.sensord.sensors.i2c_sensor import Sensor

# I2C address and chip ID
ICM42670_I2C_ADDRESS = 0x68
ICM42670_WHO_AM_I = 0x75
ICM42670_CHIP_ID = 0x67

# Data registers
ICM42670_REG_ACCEL_DATA_X1 = 0x0B
ICM42670_REG_ACCEL_DATA_X0 = 0x0C
ICM42670_REG_ACCEL_DATA_Y1 = 0x0D
ICM42670_REG_ACCEL_DATA_Y0 = 0x0E
ICM42670_REG_ACCEL_DATA_Z1 = 0x0F
ICM42670_REG_ACCEL_DATA_Z0 = 0x10

# Config and power
ICM42670_REG_PWR_MGMT0 = 0x1F
ICM42670_PWR_MGMT0_NORMAL = 0x0F
ICM42670_PWR_MGMT0_SLEEP = 0x00
ICM42670_REG_ACCEL_CONFIG0 = 0x21
ICM42670_REG_ACCEL_CONFIG1 = 0x24
ICM42670_CONFIG_ACCEL_2_G = 0b01100000
ICM42670_CONFIG_RATE_200_Hz = 0b00001000
ICM42670_ACCEL_UI_FILT_BW_16HZ = 0x07

ROT_ANGLE_RAD = 0.4082


class ICM42670_Accel(Sensor):
  @property
  def device_address(self) -> int:
    return ICM42670_I2C_ADDRESS

  def init(self) -> None:
    self.verify_chip_id(ICM42670_WHO_AM_I, [ICM42670_CHIP_ID])
    self.source = log.SensorEventData.SensorSource.icm42670
    self.write(ICM42670_REG_PWR_MGMT0, ICM42670_PWR_MGMT0_NORMAL)
    self.write(ICM42670_REG_ACCEL_CONFIG0, ICM42670_CONFIG_ACCEL_2_G | ICM42670_CONFIG_RATE_200_Hz)
    self.write(ICM42670_REG_ACCEL_CONFIG1, ICM42670_ACCEL_UI_FILT_BW_16HZ)

  def get_event(self, ts: int | None = None) -> log.SensorEventData:
    ts = ts if ts is not None else time.monotonic_ns()
    buf = self.read(ICM42670_REG_ACCEL_DATA_X1, 6)
    # 16-bit raw, ±2g → m/s² (scale 9.80665/16384)
    accel_scale = 9.80665 / 16384.0
    x_raw = self.parse_16bit(buf[5], buf[4]) * accel_scale
    y_raw = -self.parse_16bit(buf[1], buf[0]) * accel_scale
    z_raw = -self.parse_16bit(buf[3], buf[2]) * accel_scale
    c, s = math.cos(-ROT_ANGLE_RAD), math.sin(-ROT_ANGLE_RAD)
    xr = c * x_raw - s * z_raw
    zr = s * x_raw + c * z_raw
    event = log.SensorEventData.new_message()
    event.timestamp = ts
    event.version = 1
    event.sensor = 1
    event.type = 1
    event.source = self.source
    a = event.init("acceleration")
    a.v = [xr, y_raw, zr]
    a.status = 1
    return event

  def shutdown(self) -> None:
    try:
      self.write(ICM42670_REG_PWR_MGMT0, ICM42670_PWR_MGMT0_SLEEP)
    except Exception:
      pass

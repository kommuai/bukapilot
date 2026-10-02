import math
import time

from cereal import log
from openpilot.system.sensord.sensors.i2c_sensor import Sensor

ICM42670_I2C_ADDRESS = 0x68
ICM42670_WHO_AM_I = 0x75
ICM42670_CHIP_ID = 0x67

ICM42670_REG_GYRO_DATA_X1 = 0x11
ICM42670_REG_GYRO_DATA_X0 = 0x12
ICM42670_REG_GYRO_DATA_Y1 = 0x13
ICM42670_REG_GYRO_DATA_Y0 = 0x14
ICM42670_REG_GYRO_DATA_Z1 = 0x15
ICM42670_REG_GYRO_DATA_Z0 = 0x16

ICM42670_REG_PWR_MGMT0 = 0x1F
ICM42670_PWR_MGMT0_NORMAL = 0x0F
ICM42670_PWR_MGMT0_SLEEP = 0x00
ICM42670_REG_GYRO_CONFIG0 = 0x20
ICM42670_REG_GYRO_CONFIG1 = 0x23
ICM42670_CONFIG_GYRO_250_DPS = 0b01100000
ICM42670_CONFIG_RATE_200_Hz = 0b00001000
ICM42670_GYRO_UI_FILT_BW_16HZ = 0x07

ROT_ANGLE_RAD = 0.4082
GYRO_SCALE_LSB_DPS = 131.0


class ICM42670_Gyro(Sensor):
  @property
  def device_address(self) -> int:
    return ICM42670_I2C_ADDRESS

  def init(self) -> None:
    self.verify_chip_id(ICM42670_WHO_AM_I, [ICM42670_CHIP_ID])
    self.source = log.SensorEventData.SensorSource.icm42670
    self.write(ICM42670_REG_PWR_MGMT0, ICM42670_PWR_MGMT0_NORMAL)
    self.wait()
    time.sleep(0.05)
    self.write(ICM42670_REG_GYRO_CONFIG0, ICM42670_CONFIG_GYRO_250_DPS | ICM42670_CONFIG_RATE_200_Hz)
    time.sleep(0.02)
    self.write(ICM42670_REG_GYRO_CONFIG1, ICM42670_GYRO_UI_FILT_BW_16HZ)

  def get_event(self, ts: int | None = None) -> log.SensorEventData:
    ts = ts if ts is not None else time.monotonic_ns()
    buf = self.read(ICM42670_REG_GYRO_DATA_X1, 6)
    scale_rad = (math.pi / 180.0) / GYRO_SCALE_LSB_DPS
    gx_raw = self.parse_16bit(buf[5], buf[4]) * scale_rad
    gy_raw = -self.parse_16bit(buf[1], buf[0]) * scale_rad
    gz_raw = -self.parse_16bit(buf[3], buf[2]) * scale_rad
    c, s = math.cos(-ROT_ANGLE_RAD), math.sin(-ROT_ANGLE_RAD)
    gx = c * gx_raw - s * gz_raw
    gz = s * gx_raw + c * gz_raw
    event = log.SensorEventData.new_message()
    event.timestamp = ts
    event.version = 1
    event.sensor = 5
    event.type = 16
    event.source = self.source
    g = event.init("gyroUncalibrated")
    g.v = [gx, gy_raw, gz]
    g.status = 1
    return event

  def shutdown(self) -> None:
    try:
      self.write(ICM42670_REG_PWR_MGMT0, ICM42670_PWR_MGMT0_SLEEP)
    except Exception:
      pass

import time

from cereal import log
from openpilot.system.sensord.sensors.i2c_sensor import Sensor

ICM42670_I2C_ADDRESS = 0x68
ICM42670_WHO_AM_I = 0x75
ICM42670_CHIP_ID = 0x67
ICM42670_REG_PWR_MGMT0 = 0x1F
ICM42670_PWR_MGMT0_SLEEP = 0x00
ICM42670_REG_TEMP_DATA_X1 = 0x09
ICM42670_REG_TEMP_DATA_X0 = 0x0A


class ICM42670_Temp(Sensor):
  @property
  def device_address(self) -> int:
    return ICM42670_I2C_ADDRESS

  def init(self) -> None:
    self.verify_chip_id(ICM42670_WHO_AM_I, [ICM42670_CHIP_ID])
    self.source = log.SensorEventData.SensorSource.icm42670

  def get_event(self, ts: int | None = None) -> log.SensorEventData:
    ts = ts if ts is not None else time.monotonic_ns()
    buf = self.read(ICM42670_REG_TEMP_DATA_X1, 2)
    temp = self.parse_16bit(buf[1], buf[0]) / 128.0 + 25.0
    event = log.SensorEventData.new_message()
    event.timestamp = ts
    event.version = 1
    event.type = 4
    event.source = self.source
    event.temperature = temp
    return event

  def shutdown(self) -> None:
    try:
      self.write(ICM42670_REG_PWR_MGMT0, ICM42670_PWR_MGMT0_SLEEP)
    except Exception:
      pass

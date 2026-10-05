import time

from cereal import log
from openpilot.system.sensord.sensors.i2c_sensor import Sensor

LIS2MDL_I2C_ADDRESS = 0x1E
LIS2MDL_WHO_AM_I = 0x4F
LIS2MDL_CHIP_ID = 0x40
LIS2MDL_REG_MAGN_DATA = 0x68
LIS2MDL_REG_CFG_REG_A = 0x60
LIS2MDL_REG_CFG_REG_B = 0x61
LIS2MDL_TEMP_COMP_MODE = 1 << 7
LIS2MDL_LOW_POWER_MODE = 1 << 4
LIS2MDL_ODR_100HZ = 3 << 2
LIS2MDL_MODE_CONT = 0
LIS2MDL_LOW_PASS_ON = 1
SCALE_UT_PER_LSB = 0.15


class LIS2MDL_Magn(Sensor):
  @property
  def device_address(self) -> int:
    return LIS2MDL_I2C_ADDRESS

  def init(self) -> None:
    self.verify_chip_id(LIS2MDL_WHO_AM_I, [LIS2MDL_CHIP_ID])
    self.source = log.SensorEventData.SensorSource.lis2mdl
    self.write(LIS2MDL_REG_CFG_REG_A, LIS2MDL_TEMP_COMP_MODE | LIS2MDL_ODR_100HZ | LIS2MDL_MODE_CONT)
    self.write(LIS2MDL_REG_CFG_REG_B, LIS2MDL_LOW_PASS_ON)
    time.sleep(0.01)

  def get_event(self, ts: int | None = None) -> log.SensorEventData:
    ts = ts if ts is not None else time.monotonic_ns()
    buf = self.read(LIS2MDL_REG_MAGN_DATA, 6)
    # NED: x=-word2, y=word0, z=-word1 (C++ read_16_bit(high, low) -> parse_16bit(low, high))
    x = -self.parse_16bit(buf[3], buf[2]) * SCALE_UT_PER_LSB
    y = self.parse_16bit(buf[1], buf[0]) * SCALE_UT_PER_LSB
    z = -self.parse_16bit(buf[5], buf[4]) * SCALE_UT_PER_LSB
    event = log.SensorEventData.new_message()
    event.timestamp = ts
    event.version = 1
    event.sensor = 3
    event.type = 14
    event.source = self.source
    m = event.init("magneticUncalibrated")
    m.v = [x, y, z]
    m.status = 1
    return event

  def shutdown(self) -> None:
    try:
      self.write(LIS2MDL_REG_CFG_REG_A, LIS2MDL_LOW_POWER_MODE)
    except Exception:
      pass

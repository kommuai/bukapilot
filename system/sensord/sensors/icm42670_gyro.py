import math
import time
from statistics import fmean, pstdev

from cereal import log
from openpilot.common.swaglog import cloudlog
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

GYRO_CAL_SAMPLE_COUNT = 300
GYRO_CAL_SAMPLE_DT = 0.005
GYRO_CAL_MAX_STD_RAD_S = 0.08
GYRO_CAL_MAX_MEAN_RAD_S = 0.35


class ICM42670_Gyro(Sensor):
  @property
  def device_address(self) -> int:
    return ICM42670_I2C_ADDRESS

  def init(self) -> None:
    self.verify_chip_id(ICM42670_WHO_AM_I, [ICM42670_CHIP_ID])
    self.source = log.SensorEventData.SensorSource.icm42670
    self.bias_counts = (0.0, 0.0, 0.0)

    self.write(ICM42670_REG_PWR_MGMT0, ICM42670_PWR_MGMT0_NORMAL)
    self.wait()
    time.sleep(0.05)
    self.write(ICM42670_REG_GYRO_CONFIG0, ICM42670_CONFIG_GYRO_250_DPS | ICM42670_CONFIG_RATE_200_Hz)
    time.sleep(0.02)
    self.write(ICM42670_REG_GYRO_CONFIG1, ICM42670_GYRO_UI_FILT_BW_16HZ)

    # Estimate zero-rate offset from fresh stationary samples instead of
    # hardcoding a route-derived correction. This keeps the fix tied to the
    # physical sensor's current bias at boot.
    self._calibrate_zero_rate_bias()

  def _read_raw_counts(self) -> tuple[int, int, int]:
    buf = self.read(ICM42670_REG_GYRO_DATA_X1, 6)
    gx = self.parse_16bit(buf[5], buf[4])
    gy = -self.parse_16bit(buf[1], buf[0])
    gz = -self.parse_16bit(buf[3], buf[2])
    return gx, gy, gz

  def _counts_to_rad(self, counts: float) -> float:
    return counts * (math.pi / 180.0) / GYRO_SCALE_LSB_DPS

  def _calibrate_zero_rate_bias(self) -> None:
    samples: list[tuple[int, int, int]] = []
    time.sleep(0.05)
    for _ in range(GYRO_CAL_SAMPLE_COUNT):
      samples.append(self._read_raw_counts())
      time.sleep(GYRO_CAL_SAMPLE_DT)

    means = tuple(fmean(axis) for axis in zip(*samples, strict=True))
    stds_counts = tuple(pstdev(axis) for axis in zip(*samples, strict=True))
    mean_rad = [self._counts_to_rad(v) for v in means]
    std_rad = [self._counts_to_rad(v) for v in stds_counts]

    mean_norm = math.sqrt(sum(v * v for v in mean_rad))
    max_std = max(std_rad)
    stationary = mean_norm <= GYRO_CAL_MAX_MEAN_RAD_S and max_std <= GYRO_CAL_MAX_STD_RAD_S

    if stationary:
      self.bias_counts = means
    else:
      self.bias_counts = (0.0, 0.0, 0.0)
      cloudlog.warning(f"ICM42670 gyro bias calibration skipped: mean_rad={mean_rad} std_rad={std_rad}")

  def get_event(self, ts: int | None = None) -> log.SensorEventData:
    ts = ts if ts is not None else time.monotonic_ns()

    gx_counts, gy_counts, gz_counts = self._read_raw_counts()
    gx_raw = self._counts_to_rad(gx_counts - self.bias_counts[0])
    gy_raw = self._counts_to_rad(gy_counts - self.bias_counts[1])
    gz_raw = self._counts_to_rad(gz_counts - self.bias_counts[2])

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

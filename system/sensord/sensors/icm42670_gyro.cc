#include "system/sensord/sensors/icm42670_gyro.h"

#include <cassert>
#include <cmath>

#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"

#define ROT_ANGLE_RAD 0.4082f
#define DEG2RAD(x) ((x) * M_PI / 180.0)

#define TRY_OR_FAIL(x) \
  do { \
    int __ret = (x); \
    if (__ret < 0) { \
      LOGE("ICM42670 gyro init failed at %s:%d", __FILE__, __LINE__); \
      return __ret; \
    } \
  } while (0)


ICM42670_Gyro::ICM42670_Gyro(I2CBus *bus) : I2CSensor(bus) {}

int ICM42670_Gyro::init() {
  LOGD("Initializing ICM42670 gyroscope");
  TRY_OR_FAIL(verify_chip_id(ICM42670_WHO_AM_I, {ICM42670_CHIP_ID}));

  TRY_OR_FAIL(set_register(ICM42670_REG_PWR_MGMT0,
                           ICM42670_PWR_MGMT0_NORMAL));

  util::sleep_for(50);

  TRY_OR_FAIL(set_register(ICM42670_REG_GYRO_CONFIG0,
                           ICM42670_CONFIG_GYRO_250_DPS |
                           ICM42670_CONFIG_RATE_200_Hz));

  util::sleep_for(20);

  TRY_OR_FAIL(set_register(ICM42670_REG_GYRO_CONFIG1,
                           ICM42670_GYRO_UI_FILT_BW_34HZ));

  LOGD("ICM42670 gyroscope initialized OK");
  return 0;
}

int ICM42670_Gyro::shutdown() {
  int ret = set_register(ICM42670_REG_PWR_MGMT0,
                         ICM42670_PWR_MGMT0_SLEEP);
  if (ret < 0) {
    LOGE("Could not put ICM42670 gyro into sleep mode");
  }
  return ret;
}

bool ICM42670_Gyro::get_event(MessageBuilder &msg, uint64_t ts) {
  const uint64_t start_time = nanos_since_boot();

  uint8_t buffer[6];
  int len = read_register(ICM42670_REG_GYRO_DATA_X1,
                          buffer, sizeof(buffer));
  assert(len == 6);

  constexpr float scale = 131.0f;   // LSB/°/s
  float gx_raw =  DEG2RAD(read_16_bit(buffer[5], buffer[4]) / scale);
  float gy_raw = -DEG2RAD(read_16_bit(buffer[1], buffer[0]) / scale);
  float gz_raw = -DEG2RAD(read_16_bit(buffer[3], buffer[2]) / scale);

  // ---------------------------------------------------
  // Apply SAME ROTATION as accelerometer
  // This ensures accel + gyro are in same device frame
  // ---------------------------------------------------
  const double c = cos(-ROT_ANGLE_RAD);
  const double s = sin(-ROT_ANGLE_RAD);

  float gx = (float)(c * gx_raw - s * gz_raw);
  float gz = (float)(s * gx_raw + c * gz_raw);

  // gy axis is unchanged by this X–Z rotation
  float gy = gy_raw;

  float xyz[3] = {gx, gy, gz};

  auto event = msg.initEvent().initGyroscope();
  event.setSource(cereal::SensorEventData::SensorSource::ICM42670);
  event.setVersion(1);
  event.setSensor(SENSOR_GYRO_UNCALIBRATED);
  event.setType(SENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
  event.setTimestamp(start_time);

  auto svec = event.initGyroUncalibrated();
  svec.setV(xyz);
  svec.setStatus(true);

  return true;
}

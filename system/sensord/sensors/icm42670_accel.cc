#include "system/sensord/sensors/icm42670_accel.h"

#include <cassert>
#include <math.h>

#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"

#define ROT_ANGLE_RAD 0.4082f
#define TRY_OR_FAIL(x) \
  do { \
    int __ret = (x); \
    if (__ret < 0) { \
      LOGE("ICM42670 accel init failed at %s:%d", __FILE__, __LINE__); \
      return __ret; \
    } \
  } while (0)


ICM42670_Accel::ICM42670_Accel(I2CBus *bus) : I2CSensor(bus) {}

int ICM42670_Accel::init() {
  LOGD("Initializing ICM42670 accelerometer");

  TRY_OR_FAIL(verify_chip_id(ICM42670_WHO_AM_I, {ICM42670_CHIP_ID}));

  TRY_OR_FAIL(set_register(ICM42670_REG_PWR_MGMT0,
                           ICM42670_PWR_MGMT0_NORMAL));

  TRY_OR_FAIL(set_register(ICM42670_REG_ACCEL_CONFIG0,
                           ICM42670_CONFIG_ACCEL_2_G |
                           ICM42670_CONFIG_RATE_200_Hz));

  // LPF bandwidth 34hz
  TRY_OR_FAIL(set_register(ICM42670_REG_ACCEL_CONFIG1,
                           ICM42670_ACCEL_UI_FILT_BW_34HZ));

  LOGD("ICM42670 accelerometer initialized OK");
  return 0;
}

int ICM42670_Accel::shutdown() {
  int ret = set_register(ICM42670_REG_PWR_MGMT0,
                         ICM42670_PWR_MGMT0_SLEEP);

  if (ret < 0) {
    LOGE("Could not make ICM42670 enter sleep mode");
  }
  return ret;
}

bool ICM42670_Accel::get_event(MessageBuilder &msg, uint64_t ts) {
  const uint64_t start_time = nanos_since_boot();

  uint8_t buffer[6];
  int len = read_register(ICM42670_REG_ACCEL_DATA_X1,
                          buffer, sizeof(buffer));
  assert(len == 6);

  // 16-bit raw values → ±2g → m/s²
  constexpr double accel_scale = 9.80665 / 16384.0;  // datasheet

  // bias correction on X (0.25 m/s²)
  float x_raw = read_16_bit(buffer[5], buffer[4]) * accel_scale - 0.25f;
  float y_raw = -read_16_bit(buffer[1], buffer[0]) * accel_scale;
  float z_raw = -read_16_bit(buffer[3], buffer[2]) * accel_scale;

  const double c = cos(-ROT_ANGLE_RAD);
  const double s = sin(-ROT_ANGLE_RAD);
  float xr = (float)(c * x_raw - s * z_raw);
  float zr = (float)(s * x_raw + c * z_raw);

  float xyz[3] = {xr, y_raw, zr};

  auto event = msg.initEvent().initAccelerometer();
  event.setSource(cereal::SensorEventData::SensorSource::ICM42670);
  event.setVersion(1);
  event.setSensor(SENSOR_ACCELEROMETER);
  event.setType(SENSOR_TYPE_ACCELEROMETER);
  event.setTimestamp(start_time);

  auto svec = event.initAcceleration();
  svec.setV(xyz);
  svec.setStatus(true);

  return true;
}


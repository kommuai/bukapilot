#include "system/sensord/sensors/icm42670_accel.h"

#include <cassert>
#include <math.h>

#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"

#define ROT_ANGLE_RAD 0.4082f

ICM42670_Accel::ICM42670_Accel(I2CBus *bus) : I2CSensor(bus) {}

int ICM42670_Accel::init() {
  int ret = verify_chip_id(ICM42670_WHO_AM_I, {ICM42670_CHIP_ID});
  if (ret == -1) return -1;

  ret = set_register(ICM42670_REG_PWR_MGMT0, ICM42670_PWR_MGMT0_NORMAL);
  if (ret < 0) {
    goto fail;
  }

  ret = set_register(ICM42670_REG_ACCEL_CONFIG0, ICM42670_CONFIG_ACCEL_2_G | ICM42670_CONFIG_RATE_200_Hz);
  if (ret < 0) {
    goto fail;
  }


fail:
  return ret;
}


int ICM42670_Accel::shutdown()  {
  // enter deep suspend mode (lowest power mode)
  int ret = set_register(ICM42670_REG_PWR_MGMT0, ICM42670_PWR_MGMT0_SLEEP);
  if (ret < 0) {
    LOGE("Could not make ICM42670 into sleep mode!");
  }

  return ret;
}


bool ICM42670_Accel::get_event(MessageBuilder &msg, uint64_t ts) {
  uint64_t start_time = nanos_since_boot();

  uint8_t buffer[6];
  int len = read_register(ICM42670_REG_ACCEL_DATA_X1, buffer, sizeof(buffer));
  assert(len == 6);

  double accel_scale = 9.81 / 16384; // sensitivity scale factor from datasheet
  float x = -read_16_bit(buffer[5], buffer[4]) * accel_scale;
  float y = read_16_bit(buffer[1], buffer[0]) * accel_scale;
  float z = -read_16_bit(buffer[3], buffer[2]) * accel_scale;

  // rotate the frame along the y axis
  double cos_theta = cos(-ROT_ANGLE_RAD);
  double sin_theta = sin(-ROT_ANGLE_RAD);
  x = (cos_theta * x - sin_theta * z) * 1.017f; // TODO: find out why offset needed
  z = (sin_theta * x + cos_theta * z) * 1.177f;

  auto event = msg.initEvent().initAccelerometer();
  event.setSource(cereal::SensorEventData::SensorSource::ICM42670);
  event.setVersion(1);
  event.setSensor(SENSOR_ACCELEROMETER);
  event.setType(SENSOR_TYPE_ACCELEROMETER);
  event.setTimestamp(start_time);

  float xyz[] = {x, y, z};
  auto svec = event.initAcceleration();
  svec.setV(xyz);
  svec.setStatus(true);

  return true;
}


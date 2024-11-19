#include "system/sensord/sensors/icm42670_gyro.h"

#include <cassert>
#include <cmath>

#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"

#define DEG2RAD(x) ((x) * M_PI / 180.0)

ICM42670_Gyro::ICM42670_Gyro(I2CBus *bus) : I2CSensor(bus) {}

int ICM42670_Gyro::init() {
  int ret = verify_chip_id(ICM42670_WHO_AM_I, {ICM42670_CHIP_ID});
  if (ret == -1) return -1;

  ret = set_register(ICM42670_REG_PWR_MGMT0, ICM42670_PWR_MGMT0_NORMAL);
  if (ret < 0) {
    goto fail;
  }
  // icm42670 gyro has a 45ms wakeup time from deep suspend mode
  util::sleep_for(50);

  // start gyro
  ret = set_register(ICM42670_REG_GYRO_CONFIG0, ICM42670_CONFIG_GYRO_250_DPS | ICM42670_CONFIG_RATE_200_Hz);
  if (ret < 0) {
    goto fail;
  }

  // gyro reconfig time ~20ms
  util::sleep_for(20);

fail:
  return ret;
}


int ICM42670_Gyro::shutdown()  {
  // enter deep suspend mode (lowest power mode)
  int ret = set_register(ICM42670_REG_PWR_MGMT0, ICM42670_PWR_MGMT0_SLEEP);
  if (ret < 0) {
    LOGE("Could not make ICM42670 into sleep mode!");
  }

  return ret;
}


bool ICM42670_Gyro::get_event(MessageBuilder &msg, uint64_t ts) {
  uint64_t start_time = nanos_since_boot();

  uint8_t buffer[6];
  int len = read_register(ICM42670_REG_GYRO_DATA_X1, buffer, sizeof(buffer));
  assert(len == 6);

  float scale = 131; // sensitivity scale factor from datasheet
  float x = -DEG2RAD(read_16_bit(buffer[5], buffer[4]) / scale) - 0.0945f;
  float y = DEG2RAD(read_16_bit(buffer[1], buffer[0]) / scale) + 0.1855f;
  float z = -DEG2RAD(read_16_bit(buffer[3], buffer[2]) / scale) - 0.1183f;

  auto event = msg.initEvent().initGyroscope();
  event.setSource(cereal::SensorEventData::SensorSource::ICM42670);
  event.setVersion(1);
  event.setSensor(SENSOR_GYRO_UNCALIBRATED);
  event.setType(SENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
  event.setTimestamp(start_time);

  float xyz[] = {x, y, z};
  auto svec = event.initGyroUncalibrated();
  svec.setV(xyz);
  svec.setStatus(true);

  return true;
}


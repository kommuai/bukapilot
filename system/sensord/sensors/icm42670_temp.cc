#include "system/sensord/sensors/icm42670_temp.h"

#include <cassert>

#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"

ICM42670_Temp::ICM42670_Temp(I2CBus *bus) : I2CSensor(bus) {}

int ICM42670_Temp::init() {
  int ret = verify_chip_id(ICM42670_WHO_AM_I, {ICM42670_CHIP_ID});
  return ret;
}


int ICM42670_Temp::shutdown()  {
  // enter deep suspend mode (lowest power mode)
  int ret = set_register(ICM42670_REG_PWR_MGMT0, ICM42670_PWR_MGMT0_SLEEP);
  if (ret < 0) {
    LOGE("Could not make ICM42670 into sleep mode!");
  }

  return ret;
}


bool ICM42670_Temp::get_event(MessageBuilder &msg, uint64_t ts) {
  uint64_t start_time = nanos_since_boot();

  uint8_t buffer[2];
  int len = read_register(ICM42670_REG_TEMP_DATA_X1, buffer, sizeof(buffer));
  assert(len == 2);

  float temp = read_16_bit(buffer[1], buffer[0]) / 128.0f + 25.0f;

  auto event = msg.initEvent().initTemperatureSensor();
  event.setSource(cereal::SensorEventData::SensorSource::ICM42670);
  event.setVersion(1);
  event.setType(SENSOR_TYPE_AMBIENT_TEMPERATURE);
  event.setTimestamp(start_time);
  event.setTemperature(temp);

  return true;
}


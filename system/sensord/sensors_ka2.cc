#include <sys/resource.h>

#include <chrono>
#include <thread>
#include <vector>
#include <map>
#include <poll.h>
#include <linux/gpio.h>

#include "cereal/services.h"
#include "cereal/messaging/messaging.h"
#include "common/i2c.h"
#include "common/ratekeeper.h"
#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"
#include "system/sensord/sensors/constants.h"
#include "system/sensord/sensors/icm42670_accel.h"
#include "system/sensord/sensors/icm42670_gyro.h"
#include "system/sensord/sensors/icm42670_temp.h"
#include "system/sensord/sensors/lis2mdl_magn.h"
#include "system/sensord/sensors/lis2mdl_temp.h"

#define I2C_BUS_IMU 3
ExitHandler do_exit;

void polling_loop(Sensor *sensor, std::string msg_name) {
  PubMaster pm({msg_name.c_str()});
  RateKeeper rk(msg_name, services.at(msg_name).frequency);
  while (!do_exit) {
    MessageBuilder msg;
    if (sensor->get_event(msg) && sensor->is_data_valid(nanos_since_boot())) {
      pm.send(msg_name.c_str(), msg);
    }
    rk.keepTime();
  }
}

int sensor_loop(I2CBus *i2c_bus_imu) {
  // Sensor init
  std::vector<std::tuple<Sensor *, std::string>> sensors_init = {
    {new ICM42670_Accel(i2c_bus_imu), "accelerometer"},
    {new ICM42670_Gyro(i2c_bus_imu), "gyroscope"},
    {new ICM42670_Temp(i2c_bus_imu), "temperatureSensor"},
    
    {new LIS2MDL_Magn(i2c_bus_imu), "magnetometer"},
    {new LIS2MDL_Temp(i2c_bus_imu), "temperatureSensor2"},
  };

  // Initialize sensors
  std::vector<std::thread> threads;
  for (auto &[sensor, msg_name] : sensors_init) {
    int err = sensor->init();
    if (err < 0) {
      continue;
    }

    if (!sensor->has_interrupt_enabled()) {
      threads.emplace_back(polling_loop, sensor, msg_name);
    }
  }

  // wait for all threads to finish
  for (auto &t : threads) {
    t.join();
  }

  for (auto &[sensor, msg_name] : sensors_init) {
    sensor->shutdown();
    delete sensor;
  }
  
  return 0;
}


int main(int argc, char *argv[]) {
  try {
    auto i2c_bus_imu = std::make_unique<I2CBus>(I2C_BUS_IMU);
    return sensor_loop(i2c_bus_imu.get());
  } catch (std::exception &e) {
    LOGE("I2CBus init failed");
    return -1;
  }
}

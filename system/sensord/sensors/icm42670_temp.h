#pragma once

#include "system/sensord/sensors/i2c_sensor.h"

// I2C Address
#define ICM42670_I2C_ADDRESS         0x68

// Chip ID
#define ICM42670_WHO_AM_I            0x75
#define ICM42670_CHIP_ID             0x67

// Configs
#define ICM42670_REG_TEMP_CONFIG0    0x22

// Power Modes
#define ICM42670_REG_PWR_MGMT0       0x1F
#define ICM42670_PWR_MGMT0_NORMAL    0x0F
#define ICM42670_PWR_MGMT0_SLEEP     0x00

// Data
# define ICM42670_REG_TEMP_DATA_X0   0x0A
# define ICM42670_REG_TEMP_DATA_X1   0x09

class ICM42670_Temp : public I2CSensor {
  uint8_t get_device_address() {return ICM42670_I2C_ADDRESS;}
public:
  ICM42670_Temp(I2CBus *bus);
  int init();
  bool get_event(MessageBuilder &msg, uint64_t ts = 0);
  int shutdown();
};

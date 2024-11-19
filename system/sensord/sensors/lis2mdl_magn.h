#pragma once

#include "system/sensord/sensors/i2c_sensor.h"

// I2C Address
#define LIS2MDL_I2C_ADDRESS         0x1E

// Chip ID
#define LIS2MDL_WHO_AM_I            0x4F
#define LIS2MDL_CHIP_ID             0x40

// Data Registers
#define LIS2MDL_REG_MAGN_DATA       0x68

// Configs
#define LIS2MDL_REG_CFG_REG_A       0x60
#define LIS2MDL_REG_CFG_REG_B       0x61
#define LIS2MDL_REG_CFG_REG_C       0x62

// REG_A
#define LIS2MDL_TEMP_COMP_MODE      1 << 7
#define LIS2MDL_LOW_POWER_MODE      1 << 4
#define LIS2MDL_ODR_10HZ            0 << 2
#define LIS2MDL_ODR_20HZ            1 << 2
#define LIS2MDL_ODR_50HZ            2 << 2
#define LIS2MDL_ODR_100HZ           3 << 2
#define LIS2MDL_MODE_CONT           0
#define LIS2MDL_MODE_SINGLE         1
#define LIS2MDL_MODE_IDLE           2
#define LIS2MDL_MODE_IDLE_DEFAULT   3

// REG B
#define LIS2MDL_LOW_PASS_ON         1

// REG C
#define LIS2MDL_MAGN_INT_ON         1 << 7

class LIS2MDL_Magn : public I2CSensor {
  uint8_t get_device_address() {return LIS2MDL_I2C_ADDRESS;}
public:
  LIS2MDL_Magn(I2CBus *bus);
  int init();
  bool get_event(MessageBuilder &msg, uint64_t ts = 0);
  int shutdown();
};

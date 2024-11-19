#pragma once

#include "system/sensord/sensors/i2c_sensor.h"

// I2C Address
#define ICM42670_I2C_ADDRESS         0x68

// Chip ID
#define ICM42670_WHO_AM_I            0x75
#define ICM42670_CHIP_ID             0x67

// Data Register
#define ICM42670_REG_GYRO_DATA_X1    0x11
#define ICM42670_REG_GYRO_DATA_X0    0x12
#define ICM42670_REG_GYRO_DATA_Y1    0x13
#define ICM42670_REG_GYRO_DATA_Y0    0x14
#define ICM42670_REG_GYRO_DATA_Z1    0x15
#define ICM42670_REG_GYRO_DATA_Z0    0x16

// Configs
#define ICM42670_REG_GYRO_CONFIG0    0x20
#define ICM42670_REG_GYRO_CONFIG1    0x23
#define ICM42670_REG_TEMP_CONFIG0    0x22

// Power Modes
#define ICM42670_REG_PWR_MGMT0       0x1F
#define ICM42670_PWR_MGMT0_NORMAL    0x0F
#define ICM42670_PWR_MGMT0_SLEEP     0x00

// Calibration
#define ICM42670_CONFIG_GYRO_2k_DPS     0b00000000
#define ICM42670_CONFIG_GYRO_1k_DPS     0b00100000
#define ICM42670_CONFIG_GYRO_500_DPS    0b01000000
#define ICM42670_CONFIG_GYRO_250_DPS    0b01100000
#define ICM42670_CONFIG_RATE_1p6_kHz    0b00000101
#define ICM42670_CONFIG_RATE_800_Hz     0b00000110
#define ICM42670_CONFIG_RATE_400_Hz     0b00000111
#define ICM42670_CONFIG_RATE_200_Hz     0b00001000
#define ICM42670_CONFIG_RATE_100_Hz     0b00001001
#define ICM42670_CONFIG_RATE_50_Hz      0b00001010
#define ICM42670_CONFIG_RATE_25_Hz      0b00001011
#define ICM42670_CONFIG_RATE_12p5_Hz    0b00001100

class ICM42670_Gyro : public I2CSensor {
  uint8_t get_device_address() {return ICM42670_I2C_ADDRESS;}
public:
  ICM42670_Gyro(I2CBus *bus);
  int init();
  bool get_event(MessageBuilder &msg, uint64_t ts = 0);
  int shutdown();
};

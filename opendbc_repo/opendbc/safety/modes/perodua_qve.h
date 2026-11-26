#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/defaults.h"

#define PERODUA_QVE_SAFETY_PARAM_IGNORE_IGNITION_LINE 1U
#define PERODUA_QVE_RADAR_BUS               0U
#define PERODUA_QVE_MAIN_BUS                1U
#define PERODUA_QVE_CAM_BUS                 2U

#define PERODUA_QVE_CAM_TO_RADAR_ALLOW      0x50U
#define PERODUA_QVE_CAM_TO_RADAR_RANGE_LO   0x80U
#define PERODUA_QVE_CAM_TO_RADAR_RANGE_HI   0x88U

#define PERODUA_QVE_GAS_PEDAL         0xBCU
#define PERODUA_QVE_BRAKE             0xB5U
#define PERODUA_QVE_IMU               0xA0U
#define PERODUA_QVE_ADAS_STATUS       0x1B2U
#define PERODUA_QVE_CAM_HEARTBEAT     0x1BDU
#define PERODUA_QVE_ACC_HUD           0xBEU
#define PERODUA_QVE_ADAS_ACC          0x1AEU

#define PERODUA_QVE_BRAKE_B3_IDLE     12U
#define PERODUA_QVE_BRAKE_B5_IDLE      8U
#define PERODUA_QVE_GAS_THRESHOLD      2U
#define PERODUA_QVE_BRAKE_B3_THRESHOLD 1U
#define PERODUA_QVE_BRAKE_B5_THRESHOLD 2U
#define PERODUA_QVE_SPEED_RAW_MOVING  143U

extern bool ignition_can;
extern uint32_t ignition_can_cnt;

static uint8_t perodua_qve_gas_raw(const CANPacket_t *msg) {
  uint8_t gas_raw = msg->data[5];
  if (msg->data[6] > gas_raw) {
    gas_raw = msg->data[6];
  }
  const uint8_t gas_hi = msg->data[7] & 0x7FU;
  if (gas_hi > gas_raw) {
    gas_raw = gas_hi;
  }
  return gas_raw;
}

static uint16_t perodua_qve_imu_speed_raw(const CANPacket_t *msg) {
  uint16_t ret = 0U;
  int i = 2;
  int bits = 16;
  const int msb = 16;
  const int lsb = 33;

  while ((i < 8) && (bits > 0)) {
    const int byte_lsb = ((lsb / 8) == i) ? lsb : (i * 8);
    const int byte_msb = ((msb / 8) == i) ? msb : ((i + 1) * 8 - 1);
    const int size = byte_msb - byte_lsb + 1;
    const uint16_t d = (msg->data[i] >> (byte_lsb - (i * 8))) & ((1U << size) - 1U);
    ret |= d << (bits - size);
    bits -= size;
    i++;
  }
  return ret;
}

static void perodua_qve_rx_hook(const CANPacket_t *msg) {
  if ((msg->bus == PERODUA_QVE_MAIN_BUS) && (msg->addr == PERODUA_QVE_GAS_PEDAL)) {
    gas_pressed = (perodua_qve_gas_raw(msg) > PERODUA_QVE_GAS_THRESHOLD);
  }

  if ((msg->bus == PERODUA_QVE_MAIN_BUS) && (msg->addr == PERODUA_QVE_BRAKE)) {
    const bool b3_pressed = (msg->data[3] > (PERODUA_QVE_BRAKE_B3_IDLE + PERODUA_QVE_BRAKE_B3_THRESHOLD));
    const bool b5_pressed = (msg->data[5] > (PERODUA_QVE_BRAKE_B5_IDLE + PERODUA_QVE_BRAKE_B5_THRESHOLD));
    brake_pressed = b3_pressed || b5_pressed;
  }

  if ((msg->bus == PERODUA_QVE_MAIN_BUS) && (msg->addr == PERODUA_QVE_IMU)) {
    ignition_can = true;
    ignition_can_cnt = 0U;
    if (perodua_qve_imu_speed_raw(msg) > PERODUA_QVE_SPEED_RAW_MOVING) {
      vehicle_moving = true;
    }
  }

  if ((msg->bus == PERODUA_QVE_MAIN_BUS) && (msg->addr == PERODUA_QVE_ACC_HUD)) {
    pcm_cruise_check(msg->data[2] == 0x2BU);
  }
}

static bool perodua_qve_tx_hook(const CANPacket_t *msg) {
  // Allow camera lane spoof TX on cam bus. Stock camera still owns 0x80-0x88.
  // check_relay=false on TX_MSGS so stock 0xB0/0xB1 RX does not trip relay fault.
  if ((msg->bus == PERODUA_QVE_CAM_BUS) &&
      ((msg->addr == 0xB0U) || (msg->addr == 0xB1U))) {
    return true;
  }
  return false;
}

static bool perodua_qve_fwd_hook(int bus_num, int addr) {
  // true = block. Cam bus 2 -> radar bus 0: allow 0x50 and 0x80-0x88 only.
  if (bus_num != PERODUA_QVE_CAM_BUS) {
    return false;
  }
  if (addr == (int)PERODUA_QVE_CAM_TO_RADAR_ALLOW) {
    return false;
  }
  if ((addr >= (int)PERODUA_QVE_CAM_TO_RADAR_RANGE_LO) &&
      (addr <= (int)PERODUA_QVE_CAM_TO_RADAR_RANGE_HI)) {
    return false;
  }
  return true;
}

static safety_config perodua_qve_init(uint16_t param) {
  static RxCheck perodua_qve_rx_checks[] = {
    {.msg = {{PERODUA_QVE_IMU, PERODUA_QVE_MAIN_BUS, 8U, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PERODUA_QVE_GAS_PEDAL, PERODUA_QVE_MAIN_BUS, 8U, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PERODUA_QVE_BRAKE, PERODUA_QVE_MAIN_BUS, 8U, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PERODUA_QVE_ADAS_STATUS, PERODUA_QVE_MAIN_BUS, 8U, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PERODUA_QVE_ADAS_ACC, PERODUA_QVE_MAIN_BUS, 8U, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PERODUA_QVE_CAM_HEARTBEAT, PERODUA_QVE_MAIN_BUS, 8U, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
  };

  // Cam-bus lane FD frames for spoof TX. check_relay=false: stock camera still emits these.
  static const CanMsg PERODUA_QVE_TX_MSGS[] = {
    {0xB0, PERODUA_QVE_CAM_BUS, 64, .check_relay = false},
    {0xB1, PERODUA_QVE_CAM_BUS, 16, .check_relay = false},
  };

  controls_allowed = false;
  ignore_ignition_line = ((param & PERODUA_QVE_SAFETY_PARAM_IGNORE_IGNITION_LINE) != 0U);
  ignore_ignition_line_sticky = ignore_ignition_line;
  SAFETY_UNUSED(param);

  safety_config ret = BUILD_SAFETY_CFG(perodua_qve_rx_checks, PERODUA_QVE_TX_MSGS);
  return ret;
}

const safety_hooks perodua_qve_hooks = {
  .init = perodua_qve_init,
  .rx = perodua_qve_rx_hook,
  .tx = perodua_qve_tx_hook,
  .fwd = perodua_qve_fwd_hook,
};

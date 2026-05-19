#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/defaults.h"

// Chery (Jaecoo J7 PHEV, etc.) — TX whitelist for LANE_KEEP; cruise engagement gates controls_allowed
// via HUD on camera bus (matches chery_general_pt.dbc + CarState cruise parsing).
#define CHERY_LANE_KEEP   0x345U
#define CHERY_LKAS_INFO   0x394U
#define CHERY_HUD         0x387U
#define CHERY_PCM_BUTTONS 0x360U

static void chery_rx_hook(const CANPacket_t *msg) {
  // HUD CRUISE_STATE on camera leg (panda bus 2): DBC CM "3 is ENABLE" — same as CarState.cruiseState.enabled.
  if ((msg->addr == CHERY_HUD) && (msg->bus == 2U) && (GET_LEN(msg) >= 5U)) {
    const uint8_t cruise_state = (uint8_t)((msg->data[4] >> 2) & 0x3U);
    const bool cruise_engaged = (cruise_state == 3U);
    pcm_cruise_check(cruise_engaged);
  }
}

static safety_config chery_init(uint16_t param) {
  static RxCheck chery_rx_checks[] = {
    {.msg = {{CHERY_HUD, 2U, 8U, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
  };
  static const CanMsg CHERY_TX_MSGS[] = {
    {CHERY_LANE_KEEP, 0, 8, .check_relay = true},  // bus 0 = vehicle/ECU side; stock cam TXes on bus 2.
    {CHERY_LKAS_INFO, 0, 8, .check_relay = true},  // torque spoof on PT bus (CarState LKAS_INFO).
    {CHERY_PCM_BUTTONS, 0, 6, .check_relay = false},  // stock ACC resume (ICC_TOGGLE press).
  };
  SAFETY_UNUSED(param);
  controls_allowed = false;
  return BUILD_SAFETY_CFG(chery_rx_checks, CHERY_TX_MSGS);
}

static bool chery_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return true;
}

static bool chery_fwd_hook(int bus_num, int addr) {
  const bool chery_torque_spoof = true;
  bool block = false;

  if (bus_num == 2) {
    if (addr == (int)CHERY_LANE_KEEP) {
      block = true;
    } else if (chery_torque_spoof && (addr == (int)CHERY_LKAS_INFO)) {
      block = true;
    } else {
      /* no action */
    }
  } else {
    /* no action */
  }
  return block;
}

const safety_hooks chery_hooks = {
  .init = chery_init,
  .rx = chery_rx_hook,
  .tx = chery_tx_hook,
  .fwd = chery_fwd_hook,
};

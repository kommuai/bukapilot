#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/defaults.h"

// Chery (Jaecoo J7 PHEV, etc.) — TX whitelist for LANE_KEEP; cruise engagement gates controls_allowed
// via HUD on camera bus (matches chery_general_pt.dbc + CarState cruise parsing).
#define CHERY_LANE_KEEP    0x345U
#define CHERY_LKAS_INFO    0x394U
#define CHERY_HUD          0x387U
#define CHERY_PCM_BUTTONS  0x360U
// PT-side messages that carry driver-torque / steering-input info. We block them
// from forwarding to the camera bus *while controls_allowed* (cruise engaged) and
// feed the camera our own spoofed copies instead, so the hands-on-wheel detector
// sees pinned "driver-on-wheel" torque values.
#define CHERY_EPS            0x1D3U  // DRIVER_TORQUE, STEERING_ANGLE — spoofed on bus 2
#define CHERY_WHEELSPEED_2   0x313U  // 787 — WHEEL_FL / WHEEL_FR for standstill pre-arm

// True when FL/FR wheel speeds are near zero; used to pre-arm PT->cam torque blocks
// while parked (matches CarController cam_spoof at standstill).
static bool chery_vehicle_stopped = true;

static void chery_rx_hook(const CANPacket_t *msg) {
  // HUD CRUISE_STATE on camera leg (panda bus 2): DBC CM "3 is ENABLE" — same as CarState.cruiseState.enabled.
  if ((msg->addr == CHERY_HUD) && (msg->bus == 2U) && (GET_LEN(msg) >= 5U)) {
    const uint8_t cruise_state = (uint8_t)((msg->data[4] >> 2) & 0x3U);
    const bool cruise_engaged = (cruise_state == 3U);
    pcm_cruise_check(cruise_engaged);
  }

  // WHEEL_FL 7|16@0+ and WHEEL_FR 23|16@0+, scale 0.01 kph — same threshold as CarState standstill.
  if ((msg->addr == CHERY_WHEELSPEED_2) && (msg->bus == 0U) && (GET_LEN(msg) >= 4U)) {
    const uint16_t fl = (uint16_t)(((uint16_t)msg->data[0] << 8U) | msg->data[1]);
    const uint16_t fr = (uint16_t)(((uint16_t)msg->data[2] << 8U) | msg->data[3]);
    chery_vehicle_stopped = (fl < 100U) && (fr < 100U);  // < 1 kph
  }
}

static bool chery_cam_torque_spoof_active(void) {
  return controls_allowed || chery_vehicle_stopped;
}

static safety_config chery_init(uint16_t param) {
  static RxCheck chery_rx_checks[] = {
    {.msg = {{CHERY_HUD, 2U, 8U, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{CHERY_WHEELSPEED_2, 0U, 8U, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
  };
  static const CanMsg CHERY_TX_MSGS[] = {
    {CHERY_LANE_KEEP, 0, 8, .check_relay = false},
    {CHERY_LKAS_INFO, 0, 8, .check_relay = false},
    {CHERY_LKAS_INFO, 2, 8, .check_relay = false},  // mirror our spoof to cam while we block PT->cam fwd
    {CHERY_HUD, 0, 8, .check_relay = false},
    {CHERY_EPS, 2, 8, .check_relay = false},  // EPS spoof on cam bus (DRIVER_TORQUE forced high)
    {CHERY_PCM_BUTTONS, 0, 6, .check_relay = false},
    {CHERY_PCM_BUTTONS, 2, 6, .check_relay = false},  // camera leg (panda doesn't forward our TX 0->2)
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
  // cam -> PT: block frames we re-emit on PT ourselves. LKA_STATUS (0x3A5) is
  // left to forward — the cluster uses it for the LKA-engaged indicator and
  // blocking the whole frame caused a meter error in testing.
  if (bus_num == 2) {
    return (addr == (int)CHERY_LANE_KEEP) ||
           (addr == (int)CHERY_LKAS_INFO) ||
           (addr == (int)CHERY_HUD);
  }
  // PT -> cam blocking gates our bus-2 torque spoof so the cam sees our copy:
  //   EPS (0x1D3):      blocked whenever spoof loop is active (cruise engaged OR
  //                     vehicle stopped). Passthrough is byte-identical to stock
  //                     when no tap is active, so blocking while stopped is safe.
  //   LKAS_INFO (0x394): blocked only while cruise is engaged — at standstill the
  //                     cam still needs the native frame.
  //   STEER_RELATED (0xC4): never blocked — cam's calibration watchdog cancels
  //                     LKAS without it.
  if (bus_num == 0) {
    if ((addr == (int)CHERY_EPS) && chery_cam_torque_spoof_active()) {
      return true;
    }
    if ((addr == (int)CHERY_LKAS_INFO) && controls_allowed) {
      return true;
    }
  }
  return false;
}

const safety_hooks chery_hooks = {
  .init = chery_init,
  .rx = chery_rx_hook,
  .tx = chery_tx_hook,
  .fwd = chery_fwd_hook,
};

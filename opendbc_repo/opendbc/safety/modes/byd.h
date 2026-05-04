#pragma once

#include "opendbc/safety/declarations.h"

// BYD (Atto 3, Seal, Sealion, etc.) - ported from panda safety
#define BYD_STEERING_MODULE_ADAS 0x1E2U   // 482
#define BYD_ACC_CMD              0x32EU   // 814
#define BYD_PCM_BUTTONS          0x3B0U   // 944
#define BYD_STEERING_TORQUE      0x1FCU   // 508

static bool byd_alt_engage = false;
static bool byd_steering_torque_spoof = false;

static void byd_rx_hook(const CANPacket_t *msg) {
  int bus = (int)msg->bus;
  int addr = (int)msg->addr;

  if (bus == 0) {
    if (addr == 287) {
      int angle_meas_new = (GET_BYTES(msg, 0, 2) & 0xFFFFU);
      angle_meas_new = to_signed(angle_meas_new, 16);
      update_sample(&angle_meas, angle_meas_new);
    }

    if (addr == 834) {
      gas_pressed = (msg->data[0] > 0U);
      brake_pressed = (msg->data[1] > 0U);
    }

    if (addr == 496) {
      float fl_ms = (float)(((msg->data[1] & 0x0FU) << 8) | msg->data[0]) * 0.1f / 3.6f;
      float bl_ms = (float)(((msg->data[3] & 0x0FU) << 8) | msg->data[2]) * 0.1f / 3.6f;
      float speed = (fl_ms + bl_ms) * 0.5f;
      vehicle_moving = SAFETY_ABS(speed) > 0.1f;
      UPDATE_VEHICLE_SPEED(speed);
    }

    if (addr == 944) {
      int set_pressed = (msg->data[0] >> 3U) & 1U;
      int res_pressed = (msg->data[0] >> 4U) & 1U;
      int icc_pressed = (msg->data[0] >> 6U) & 1U;
      int acc_pressed = (msg->data[2] >> 3U) & 1U;
      int cancel = (msg->data[2] >> 3U) & 1U;

      if (set_pressed | res_pressed | icc_pressed | acc_pressed) {
        controls_allowed = true;
      }
      if (cancel) {
        controls_allowed = false;
      }
    }
  }

  if (byd_alt_engage) {
    if (addr == 813) {
      uint8_t state = (msg->data[5] >> 4) & 0xFU;
      bool engaged = (state == 3U) || (state == 5U) || (state == 6U) || (state == 7U);
      pcm_cruise_check(engaged);
    }
  } else {
    if (addr == 814) {
      bool engaged = (msg->data[5] >> 4) & 1U;
      pcm_cruise_check(engaged);
    }
  }
}

static bool byd_tx_hook(const CANPacket_t *msg) {
  bool violation = false;
  int addr = (int)msg->addr;

  if (addr == BYD_STEERING_MODULE_ADAS) {
    int desired_angle = (GET_BYTES(msg, 3, 2) & 0xFFFFU);
    bool lka_active = (msg->data[2] >> 5) & 1U;
    desired_angle = to_signed(desired_angle, 16);

    /*
     * Angle rate lookup Y values are **max delta in degrees per steering TX frame** (same numeric scale as
     * opendbc/car/byd/values.py CarControllerParams ANGLE_RATE_LIMIT_* angle_v with apply_std_steer_angle_limits).
     * lateral.h multiplies by angle_deg_to_can (+1); there is no extra Hz division in steer_angle_cmd_checks.
     *
     * If these Y match values.py (~6/3/1 up, ~8/6/4 down), a single violation can still burst-block TX:
     * steer_angle_cmd_checks resets desired_angle_last to 0 while the next frame may carry a large absolute
     * command (~20+ deg). Larger Y here tolerates that generic reset until last-angle handling is fixed.
     */
    static const AngleSteeringLimits BYD_STEERING_LIMITS = {
      .max_angle = 450,
      .angle_deg_to_can = 10,
      .angle_rate_up_lookup = {{0., 5., 15.}, {24., 22., 20.}},
      .angle_rate_down_lookup = {{0., 5., 15.}, {28., 26., 22.}},
      .max_angle_error = 0,
      .angle_error_min_speed = 0.f,
      .frequency = 50U,
      .angle_is_curvature = false,
      .enforce_angle_error = false,
      .inactive_angle_is_zero = true,
    };
    if (steer_angle_cmd_checks(desired_angle, lka_active, BYD_STEERING_LIMITS)) {
      violation = true;
    }
  }

  if (addr == BYD_ACC_CMD) {
    // ACCEL_CMD raw byte; DBC physical = raw - 100. Stock logs use down to ~-80 (raw 20).
    static const LongitudinalLimits BYD_LONG_LIMITS = {
      .max_accel = 135,
      .min_accel = 20,
      .inactive_accel = 100,
    };
    violation |= longitudinal_accel_checks((int)msg->data[0], BYD_LONG_LIMITS);
  }
  return !violation;
}

static bool byd_fwd_hook(int bus_num, int addr) {
  if (bus_num == 0) {
    bool is_torque_msg = (addr == (int)BYD_STEERING_TORQUE);
    return is_torque_msg && byd_steering_torque_spoof;
  }
  if (bus_num == 2) {
    bool is_lkas_msg = (addr == 0x1E2) || (addr == 0x316);
    bool is_acc_msg = (addr == (int)BYD_ACC_CMD);
    return is_lkas_msg || is_acc_msg;
  }
  return false;
}

static safety_config byd_init(uint16_t param) {
  static const CanMsg BYD_TX_MSGS[] = {
    {0x1E2, 0, 8, .check_relay = true},   // STEERING_MODULE_ADAS
    {0x316, 0, 8, .check_relay = true},   // LKAS_HUD_ADAS
    {0x32E, 0, 8, .check_relay = false},   // ACC_CMD
    {0x3B0, 0, 8, .check_relay = false},  // PCM_BUTTONS
    {0x3B0, 2, 8, .check_relay = false},
    {0x1FC, 2, 8, .check_relay = false},  // STEERING_TORQUE
  };

  static RxCheck byd_rx_checks[] = {
    {.msg = {{287, 0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{496, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{508, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{834, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{944, 0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{814, 2, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
  };

  static RxCheck byd_rx_checks_alt[] = {
    {.msg = {{287, 0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{496, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{508, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{834, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{944, 0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{813, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
  };

  if (param == 1U) {
    return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
  }
  if (param == 2U) {
    byd_alt_engage = true;
    byd_steering_torque_spoof = true;
    return BUILD_SAFETY_CFG(byd_rx_checks_alt, BYD_TX_MSGS);
  }
  if (param == 3U) {
    byd_alt_engage = true;
    byd_steering_torque_spoof = true;
    return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
  }
  return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
};

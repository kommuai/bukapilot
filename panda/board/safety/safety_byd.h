const SteeringLimits BYD_STEERING_LIMITS = {
  .angle_deg_to_can = 10,
  .angle_rate_up_lookup = {
    {0., 5., 15.},
    {6., 4., 3.}
  },
  .angle_rate_down_lookup = {
    {0., 5., 15.},
    {8., 6., 4.}
  },
};

const LongitudinalLimits BYD_LONG_LIMITS = {
  .max_accel = 130,       // 2.83 m/s^2 TODO: verify it again
  .min_accel = 50,        // -3.2 m/s^2   TODO: verify it again
  .inactive_accel = 100,  // 0. m/s^2
};

const CanMsg BYD_TX_MSGS[] = {
  {482, 0, 8}, // STEERING_MODULE_ADAS
  {790, 0, 8}, // LKAS_HUD_ADAS
  {814, 0, 8}, // ACC_CMD
  {944, 0, 8}, // PCM_BUTTONS
  {944, 2, 8}, // PCM_BUTTONS
  {508, 2, 8}  // STEERING_TORQUE
};

bool byd_alt_engage = false;
bool byd_steering_torque_spoof = false;

RxCheck byd_rx_checks[] = {
  {.msg = {{287, 0, 5, .check_checksum = false, .frequency = 100U}, { 0 }, { 0 }}}, // STEER_MODULE_2
  {.msg = {{496, 0, 8, .check_checksum = false, .frequency = 50U}, { 0 }, { 0 }}},  // WHEEL_SPEED
  {.msg = {{508, 0, 8, .check_checksum = false, .frequency = 50U}, { 0 }, { 0 }}},  // STEERING_TORQUE
  {.msg = {{834, 0, 8, .check_checksum = false, .frequency = 50U}, { 0 }, { 0 }}},  // PEDAL
  {.msg = {{944, 0, 8, .check_checksum = false, .frequency = 20U}, { 0 }, { 0 }}},  // PCM_BUTTONS
  {.msg = {{814, 2, 8, .check_checksum = false, .frequency = 50U}, { 0 }, { 0 }}},  // ACC_CMD
};

RxCheck byd_rx_checks_alt[] = {
  {.msg = {{287, 0, 5, .check_checksum = false, .frequency = 100U}, { 0 }, { 0 }}}, // STEER_MODULE_2
  {.msg = {{496, 0, 8, .check_checksum = false, .frequency = 50U}, { 0 }, { 0 }}},  // WHEEL_SPEED
  {.msg = {{508, 0, 8, .check_checksum = false, .frequency = 50U}, { 0 }, { 0 }}},  // STEERING_TORQUE
  {.msg = {{834, 0, 8, .check_checksum = false, .frequency = 50U}, { 0 }, { 0 }}},  // PEDAL
  {.msg = {{944, 0, 8, .check_checksum = false, .frequency = 20U}, { 0 }, { 0 }}},  // PCM_BUTTONS
  {.msg = {{813, 0, 8, .check_checksum = false, .frequency = 50U}, { 0 }, { 0 }}},  // ACC_HUD_ADAS
};

static void byd_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if(bus == 0) {
    // current steering angle, factor -0.1 and little endian
    if (addr == 287) {
      int angle_meas_new = (GET_BYTES(to_push, 0, 2) & 0xFFFFU);
      // let it be CAN unit degree
      angle_meas_new = to_signed(angle_meas_new, 16);

      update_sample(&angle_meas, angle_meas_new);
    }

    // gas and brakes
    if (addr == 834) {
      gas_pressed = (GET_BYTE(to_push, 0) > 0U);
      brake_pressed = (GET_BYTE(to_push, 1) > 0U);
    }

    // vehicle speed
    if (addr == 496) {
      // average of FL and BL
      float fl_ms = (float)(((GET_BYTE(to_push, 1) & 0x0FU) << 8) | (GET_BYTE(to_push, 0))) * 0.1f / 3.6f;
      float bl_ms = (float)(((GET_BYTE(to_push, 3) & 0x0FU) << 8) | (GET_BYTE(to_push, 2))) * 0.1f / 3.6f;
      float speed = (fl_ms + bl_ms) * 0.5f;
      vehicle_moving = ABS(speed) > 0.1;
      UPDATE_VEHICLE_SPEED(speed);
    }

    // engage logic with buttons
    if (addr == 944) {
      int set_pressed = (GET_BYTE(to_push, 0) >> 3U) & 1U;
      int res_pressed = (GET_BYTE(to_push, 0) >> 4U) & 1U;
      int icc_pressed = (GET_BYTE(to_push, 0) >> 6U) & 1U;
      int acc_pressed = (GET_BYTE(to_push, 2) >> 3U) & 1U;
      int cancel = (GET_BYTE(to_push, 2) >> 3U) & 1U;

      if (set_pressed | res_pressed | icc_pressed | acc_pressed) {
        controls_allowed = true;
      }

      if (cancel) {
        controls_allowed = false;
      }
    }
  }

  // cruise enabled
  if (byd_alt_engage) {
    // seal/sealion7/m6
    if (addr == 813) {
      uint8_t state = (GET_BYTE(to_push, 5) >> 4) & 0xFU;
      bool engaged = (state == 3U) ||
                     (state == 5U) ||
                     (state == 6U) ||
                     (state == 7U);

      pcm_cruise_check(engaged);
    }
  }
  else {
    // atto3
    if (addr == 814) {
      bool engaged = (GET_BYTE(to_push, 5) >> 4) & 1U;
      pcm_cruise_check(engaged);
    }
  }
  controls_allowed = true;
  generic_rx_checks((addr == 482) && (bus == 0));
}

static bool byd_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  bool violation = false;
  int addr = GET_ADDR(to_send);

  // steer violation checks
  if (addr == 482) {

    int desired_angle = (GET_BYTES(to_send, 3, 2) & 0xFFFFU);
    bool lka_active = (GET_BYTE(to_send, 2) >> 5) & 1U;

    desired_angle = to_signed(desired_angle, 16);

    if (steer_angle_cmd_checks(desired_angle, lka_active, BYD_STEERING_LIMITS)) {
      violation = true;
    }
  }

  // acc violation checks
  if (addr == 814) {
    int desired_accel = GET_BYTE(to_send, 0);
    violation |= longitudinal_accel_checks(desired_accel, BYD_LONG_LIMITS);
  }

  if (violation) {
    tx = false;
  }
  return tx;
}

static int byd_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  if (bus_num == 0) {
    bool is_torque_msg = (addr == 0x1FC);
    bool block_msg = is_torque_msg && byd_steering_torque_spoof;
    if (!block_msg) {
      bus_fwd = 2;
    }
  }

  if (bus_num == 2) {
    bool is_lkas_msg = ((addr == 0x1E2) || (addr == 0x316));
    bool is_acc_msg = (addr == 0x32E);
    bool block_msg = is_lkas_msg || is_acc_msg;
    if (!block_msg) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static safety_config byd_init(uint16_t param) {
  if (param == 1) {
    return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
  }
  else if (param == 2) {
    byd_alt_engage = true;
    byd_steering_torque_spoof = true;
    return BUILD_SAFETY_CFG(byd_rx_checks_alt, BYD_TX_MSGS);
  }
  else if (param == 3) {
    byd_alt_engage = true;
    byd_steering_torque_spoof = true;
    return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
  }
  else {
    return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
  }
}


const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
};

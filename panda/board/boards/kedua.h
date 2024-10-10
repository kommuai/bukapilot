// ///////////////////////////// //
// Kedua (STM32H7) on-board MCU  //
// ///////////////////////////// //

void kedua_enable_can_transceiver(uint8_t transceiver, bool enabled) {
  switch (transceiver) {
    case 1U:
      set_gpio_output(GPIOG, 11, !enabled);
      break;
    case 2U:
      set_gpio_output(GPIOB, 11, !enabled);
      break;
    case 3U:
      set_gpio_output(GPIOD, 7, !enabled);
      break;
    case 4U:
      set_gpio_output(GPIOB, 10, !enabled);
      break;
    default:
      break;
  }
}

void kedua_enable_can_transceivers(bool enabled) {
  uint8_t main_bus = (harness.status == HARNESS_STATUS_FLIPPED) ? 3U : 1U;
  for (uint8_t i=1U; i<=4U; i++) {
    // Leave main CAN always on for CAN-based ignition detection
    if (i == main_bus) {
      kedua_enable_can_transceiver(i, true);
    } else {
      kedua_enable_can_transceiver(i, enabled);
    }
  }
}

void kedua_set_led(uint8_t color, bool enabled) {
  switch (color) {
    case LED_RED:
      set_gpio_output(GPIOE, 2, !enabled);
      break;
     case LED_GREEN:
      set_gpio_output(GPIOE, 3, !enabled);
      break;
    case LED_BLUE:
      set_gpio_output(GPIOE, 4, !enabled);
      break;
    default:
      break;
  }
}

void kedua_set_ir_power(uint8_t percentage){
  pwm_set(TIM3, 4, percentage);
}

void kedua_set_can_mode(uint8_t mode) {
  kedua_enable_can_transceiver(2U, false);
  kedua_enable_can_transceiver(4U, false);
  switch (mode) {
    case CAN_MODE_NORMAL:
    case CAN_MODE_OBD_CAN2:
      if ((bool)(mode == CAN_MODE_NORMAL) != (bool)(harness.status == HARNESS_STATUS_FLIPPED)) {
        // B12,B13: disable normal mode
        set_gpio_pullup(GPIOB, 12, PULL_NONE);
        set_gpio_mode(GPIOB, 12, MODE_ANALOG);

        set_gpio_pullup(GPIOB, 13, PULL_NONE);
        set_gpio_mode(GPIOB, 13, MODE_ANALOG);

        // B5,B6: FDCAN2 mode
        set_gpio_pullup(GPIOB, 5, PULL_NONE);
        set_gpio_alternate(GPIOB, 5, GPIO_AF9_FDCAN2);

        set_gpio_pullup(GPIOB, 6, PULL_NONE);
        set_gpio_alternate(GPIOB, 6, GPIO_AF9_FDCAN2);
        kedua_enable_can_transceiver(2U, true);
      } else {
        // B5,B6: disable normal mode
        set_gpio_pullup(GPIOB, 5, PULL_NONE);
        set_gpio_mode(GPIOB, 5, MODE_ANALOG);

        set_gpio_pullup(GPIOB, 6, PULL_NONE);
        set_gpio_mode(GPIOB, 6, MODE_ANALOG);
        // B12,B13: FDCAN2 mode
        set_gpio_pullup(GPIOB, 12, PULL_NONE);
        set_gpio_alternate(GPIOB, 12, GPIO_AF9_FDCAN2);

        set_gpio_pullup(GPIOB, 13, PULL_NONE);
        set_gpio_alternate(GPIOB, 13, GPIO_AF9_FDCAN2);
        kedua_enable_can_transceiver(4U, true);
      }
      break;
    default:
      break;
  }
}

bool kedua_check_ignition(void) {
  // ignition is checked through harness
  return harness_check_ignition();
}

uint32_t kedua_read_voltage_mV(void){
  return adc_get_mV(2) * 11U;
}

uint32_t kedua_read_current_mA(void){
  return adc_get_mV(3) * 2U;
}

void kedua_init(void) {
  common_init_gpio();

  //PA1,PC4 : OBD_SBU1_RELAY, OBD_SBU2_RELAY
  set_gpio_output_type(GPIOC, 4, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOC, 4, PULL_NONE);
  set_gpio_mode(GPIOC, 4, MODE_OUTPUT);
  set_gpio_output(GPIOC, 4, 0);

  set_gpio_output_type(GPIOA, 1, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOA, 1, PULL_NONE);
  set_gpio_mode(GPIOA, 1, MODE_OUTPUT);
  set_gpio_output(GPIOA, 1, 0);

  // G11,B11,D7,B10: transceiver enable
  set_gpio_pullup(GPIOG, 11, PULL_NONE);
  set_gpio_mode(GPIOG, 11, MODE_OUTPUT);

  set_gpio_pullup(GPIOB, 11, PULL_NONE);
  set_gpio_mode(GPIOB, 11, MODE_OUTPUT);

  set_gpio_pullup(GPIOD, 7, PULL_NONE);
  set_gpio_mode(GPIOD, 7, MODE_OUTPUT);

  set_gpio_pullup(GPIOB, 10, PULL_NONE);
  set_gpio_mode(GPIOB, 10, MODE_OUTPUT);

  //B1: 5VOUT_S
  set_gpio_pullup(GPIOB, 1, PULL_NONE);
  set_gpio_mode(GPIOB, 1, MODE_ANALOG);

  // Initialize harness
  harness_init();

  // Initialize RTC
  rtc_init();

  // Enable CAN transceivers
  kedua_enable_can_transceivers(true);

  // Disable LEDs
  kedua_set_led(LED_RED, false);
  kedua_set_led(LED_GREEN, false);
  kedua_set_led(LED_BLUE, false);

  // Initialize IR PWM and set to 0%
  set_gpio_alternate(GPIOC, 9, GPIO_AF2_TIM3);
  pwm_init(TIM3, 4);
  kedua_set_ir_power(0U);

  // Set normal CAN mode
  kedua_set_can_mode(CAN_MODE_NORMAL);

  // change CAN mapping when flipped
  if (harness.status == HARNESS_STATUS_FLIPPED) {
    can_flip_buses(0, 2);
  }
}

const harness_configuration kedua_harness_config = {
  .has_harness = true,
  .GPIO_SBU1 = GPIOA,
  .GPIO_SBU2 = GPIOC,
  .GPIO_relay_SBU1 = GPIOA,
  .GPIO_relay_SBU2 = GPIOC,
  .pin_SBU1 = 1,
  .pin_SBU2 = 4,
  .pin_relay_SBU1 = 1,
  .pin_relay_SBU2 = 4,
  .adc_channel_SBU1 = 17, //ADC12_INP4
  .adc_channel_SBU2 = 4 //ADC1_INP17
};

const board board_kedua = {
  .set_bootkick = unused_set_bootkick, //TODO
  .harness_config = &kedua_harness_config,
  .has_obd = true,
  .has_spi = true,
  .has_canfd = true,
  .has_rtc_battery = false, //TODO
  .fan_max_rpm = 0U,
  .avdd_mV = 3300U,
  .fan_stall_recovery = false,
  .fan_enable_cooldown_time = 0U,
  .init = kedua_init,
  .init_bootloader = unused_init_bootloader,
  .enable_can_transceiver = kedua_enable_can_transceiver,
  .enable_can_transceivers = kedua_enable_can_transceivers,
  .set_led = kedua_set_led,
  .set_can_mode = kedua_set_can_mode,
  .check_ignition = kedua_check_ignition,
  .read_voltage_mV = kedua_read_voltage_mV,
  .read_current_mA = kedua_read_current_mA, //TODO
  .set_fan_enabled = unused_set_fan_enabled,
  .set_ir_power = kedua_set_ir_power,
  .set_siren = unused_set_siren,
  .read_som_gpio = unused_read_som_gpio //TODO
};

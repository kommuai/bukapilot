#pragma once

#include "board_declarations.h"

// ///////////////////////////// //
// Kedua (STM32H7) on-board MCU  //
// ///////////////////////////// //

// Kedua's MAIN and OBD bus isn't affected by orientation, only RADAR and ADAS needs to flip.
static void kedua_enable_can_transceiver(uint8_t transceiver, bool enabled) {
  UNUSED(enabled);
  switch (transceiver) {
    case 1U:
      set_gpio_output(GPIOG, 11, false);  // CAN3, FDCAN1, MAIN
      break;
    case 2U:
      set_gpio_output(GPIOB, 11, false);  // CAN1, FDCAN2, OBD
      break;
    case 3U:
      set_gpio_output(GPIOD, 7, false);   // CAN4, FDCAN3, CAMERA
      break;
    case 4U:
      set_gpio_output(GPIOB, 10, false);  // CAN2, FDCAN2, RADAR
      break;
    default:
      break;
  }
}

static void kedua_set_bootkick(BootState state) {
  set_gpio_output(GPIOC, 2, state == BOOT_BOOTKICK);
}

static void kedua_set_ir_power(uint8_t percentage) {
  pwm_set(TIM3, 4, percentage);
}

/*
  MODE NORMAL |   FLIPPED
  ------------------------------------------------
  TRUE        !=  TRUE     = FALSE;   radar (3U) & camera (4U) & MAIN (1U) & B5,B6
  TRUE        !=  FALSE    = TRUE;    radar (4U) & camera (3U) & MAIN (1U) & B5,B6
  FALSE       !=  TRUE     = TRUE;    obd (2U)   & camera (4U) & MAIN (1U) & B12,B13
  FALSE       !=  FALSE    = FALSE;   obd (2U)   & camera (3U) & MAIN (1U) & B12,B13
*/

static void kedua_set_can_mode(uint8_t mode) {
  kedua_enable_can_transceiver(2U, false);
  kedua_enable_can_transceiver(3U, false);
  kedua_enable_can_transceiver(4U, false);

  switch (mode) {
    case CAN_MODE_NORMAL:
    case CAN_MODE_OBD_CAN2:
      if ((bool)(mode == CAN_MODE_NORMAL)) {
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

        kedua_enable_can_transceiver(3U, true);
        kedua_enable_can_transceiver(4U, true);
      }
      break;
    default:
      break;
  }
}

static uint32_t kedua_read_voltage_mV(void) {
  return adc_get_mV(&(const adc_signal_t) ADC_CHANNEL_DEFAULT(ADC1, 2)) * 11U;
}

static uint32_t kedua_read_current_mA(void) {
  return adc_get_mV(&(const adc_signal_t) ADC_CHANNEL_DEFAULT(ADC1, 3)) * 2U;
}

static void kedua_init(void) {
  common_init_gpio();

  // PC5, PC4 : OBD_SBU1_RELAY, OBD_SBU2_RELAY
  set_gpio_output_type(GPIOC, 4, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOC, 4, PULL_NONE);
  set_gpio_mode(GPIOC, 4, MODE_OUTPUT);
  set_gpio_output(GPIOC, 4, 0);

  set_gpio_output_type(GPIOC, 5, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOC, 5, PULL_NONE);
  set_gpio_mode(GPIOC, 5, MODE_OUTPUT);
  set_gpio_output(GPIOC, 5, 0);

  // G11, B11, D7, B10: transceiver enable
  set_gpio_output_type(GPIOG, 11, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOG, 11, PULL_NONE);
  set_gpio_mode(GPIOG, 11, MODE_OUTPUT);

  set_gpio_output_type(GPIOB, 11, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOB, 11, PULL_NONE);
  set_gpio_mode(GPIOB, 11, MODE_OUTPUT);

  set_gpio_output_type(GPIOD, 7, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOD, 7, PULL_NONE);
  set_gpio_mode(GPIOD, 7, MODE_OUTPUT);

  set_gpio_output_type(GPIOB, 10, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOB, 10, PULL_NONE);
  set_gpio_mode(GPIOB, 10, MODE_OUTPUT);

  // PC0 Hardware reset, can be used for bootkicking
  set_gpio_output_type(GPIOC, 0, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOC, 0, PULL_NONE);
  set_gpio_mode(GPIOC, 0, MODE_OUTPUT);

  // PC2 Boot kick
  set_gpio_output_type(GPIOC, 2, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_pullup(GPIOC, 2, PULL_NONE);
  set_gpio_mode(GPIOC, 2, MODE_OUTPUT);

  // B1: 5VOUT_S
  set_gpio_pullup(GPIOB, 1, PULL_NONE);
  set_gpio_mode(GPIOB, 1, MODE_ANALOG);

  // Initialize harness
  harness_init();

  // Set IR power to 100% init only
  kedua_set_ir_power(100U);

  // Enable CAN transceivers (same pattern as power_saving.h enable_can_transceivers)
  for (uint8_t i = 1U; i <= 4U; i++) {
    kedua_enable_can_transceiver(i, true);
  }

  // Disable LEDs (PE2=RED, PE3=GREEN, PE4=BLUE, active low)
  set_gpio_pullup(GPIOE, 2, PULL_NONE);
  set_gpio_mode(GPIOE, 2, MODE_OUTPUT);
  set_gpio_output(GPIOE, 2, 1);
  set_gpio_pullup(GPIOE, 3, PULL_NONE);
  set_gpio_mode(GPIOE, 3, MODE_OUTPUT);
  set_gpio_output(GPIOE, 3, 1);
  set_gpio_pullup(GPIOE, 4, PULL_NONE);
  set_gpio_mode(GPIOE, 4, MODE_OUTPUT);
  set_gpio_output(GPIOE, 4, 1);

  // SPI init
  gpio_spi_init();

  // Initialize IR PWM and set to 0%
  set_gpio_alternate(GPIOC, 9, GPIO_AF2_TIM3);
  pwm_init(TIM3, 4);

  // Set normal CAN mode
  kedua_set_can_mode(CAN_MODE_NORMAL);
}

static harness_configuration kedua_harness_config = {
  .GPIO_SBU1 = GPIOC,
  .GPIO_SBU2 = GPIOC,
  .GPIO_relay_SBU1 = GPIOC,
  .GPIO_relay_SBU2 = GPIOC,
  .pin_SBU1 = 5,
  .pin_SBU2 = 4,
  .pin_relay_SBU1 = 5,
  .pin_relay_SBU2 = 4,
  .adc_signal_SBU1 = ADC_CHANNEL_DEFAULT(ADC1, 8),  // ADC12_INP8
  .adc_signal_SBU2 = ADC_CHANNEL_DEFAULT(ADC1, 4)   // ADC12_INP4
};

board board_kedua = {
  .harness_config = &kedua_harness_config,
  .led_GPIO = {GPIOE, GPIOE, GPIOE},
  .led_pin = {2, 3, 4},
  .led_pwm_channels = {0, 0, 0},
  .has_spi = true,
  .has_fan = false,
  .avdd_mV = 3300U,
  .fan_enable_cooldown_time = 0U,
  .init = kedua_init,
  .init_bootloader = unused_init_bootloader,
  .enable_can_transceiver = kedua_enable_can_transceiver,
  .set_can_mode = kedua_set_can_mode,
  .read_voltage_mV = kedua_read_voltage_mV,
  .read_current_mA = kedua_read_current_mA,
  .set_ir_power = kedua_set_ir_power,
  .set_fan_enabled = unused_set_fan_enabled,
  .set_siren = unused_set_siren,
  .set_bootkick = kedua_set_bootkick,
  .read_som_gpio = unused_read_som_gpio,
  .set_amp_enabled = unused_set_amp_enabled,
};

#pragma once

#include "opendbc/safety/declarations.h"

/* ACC_BRAKE encoding bounds for create_brake_command() clip logic:
 *   decel_cmd in [0.0, 1.56] -> MAGNITUDE raw in [44, 200] using (raw * 0.01 - 2)
 *   pump in [0.0, 1.0] -> PUMP_REACTION1 raw in [0, 10] using (raw * 0.1)
 */
static const uint8_t DNGA_ACC_BRAKE_MAGNITUDE_RAW_MIN = 44U;
static const uint8_t DNGA_ACC_BRAKE_MAGNITUDE_RAW_MAX = 200U;
static const uint8_t DNGA_ACC_BRAKE_PUMP_RAW_MAX = 10U;

static const CanMsg DNGA_TX_MSGS[] = {
  {464, 0, 8, .check_relay = true},   /* STEERING_LKAS */
  {628, 0, 8, .check_relay = true},   /* LKAS_HUD */
  {625, 0, 8, .check_relay = true},   /* ACC_BRAKE */
  {627, 0, 8, .check_relay = true},   /* ACC_CMD_HUD */
  {519, 0, 6, .check_relay = false},  /* PCM_BUTTONS_HYBRID */
  {520, 0, 6, .check_relay = false},  /* PCM_BUTTONS */
  {2015, 0, 8, .check_relay = false}, /* DTC clear */
};

static RxCheck dnga_rx_checks[] = {
  /* Wheel speed (used for vehicle_moving and for lag bookkeeping). */
  {.msg = {{416, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true}, {0}, {0}}},
  /* Gas pressed signal (carstate.py: gasPressed = not bool(GAS_PEDAL_2.GAS_PEDAL_STEP)). */
  {.msg = {{399, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true}, {0}, {0}}},
  /* Brake pressed. */
  {.msg = {{161, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true}, {0}, {0}}},
  /* Cruise engagement. */
  {.msg = {{627, 2, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true}, {0}, {0}}},
  {.msg = {{625, 2, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true}, {0}, {0}}},
};

static uint32_t dnga_get_wheelspeed_raw(const CANPacket_t *msg)
{
  uint32_t wheelspeed_raw;

  /* WHEEL_SPEED.WHEELSPEED_F: 7|24@0+ (little-endian) */
  wheelspeed_raw =
    ((uint32_t)((msg->data[0] >> 7U) & 0x1U)) |
    ((uint32_t)(msg->data[1]) << 1U) |
    ((uint32_t)(msg->data[2]) << 9U) |
    ((uint32_t)(msg->data[3] & 0x7FU) << 17U);

  return wheelspeed_raw;
}

static uint32_t dnga_get_acc_cmd_raw(const CANPacket_t *msg)
{
  uint32_t acc_cmd_raw;

  acc_cmd_raw =
    ((uint32_t)((msg->data[2] >> 7U) & 0x1U)) |
    ((uint32_t)(msg->data[3]) << 1U) |
    ((uint32_t)(msg->data[4] & 0x7FU) << 9U);

  return acc_cmd_raw;
}

static bool dnga_get_bit_bool(const CANPacket_t *msg, uint32_t bit_index)
{
  bool bit_is_set;

  bit_is_set = (GET_BIT(msg, bit_index) != 0U);

  return bit_is_set;
}

static void dnga_rx_hook(const CANPacket_t *msg)
{
  /* TODO / SUGGESTED SAFETY CHECK:
   *   controls_allowed is currently forced true on every RX packet.
   *   In a more typical openpilot safety model, controls_allowed should be
   *   gated by real engagement state and cleared on disengage / faults /
   *   driver override. Preserved exactly here to avoid logic changes.
   */
  controls_allowed = true;

  if (msg->addr == 416U) {
    uint32_t wheelspeed_raw;

    wheelspeed_raw = dnga_get_wheelspeed_raw(msg);
    vehicle_moving = (wheelspeed_raw != 0U);

  } else {
    /* no action */
  }

  if (msg->addr == 399U) {
    /* carstate.py: gasPressed = not bool(GAS_PEDAL_2.GAS_PEDAL_STEP) */
    gas_pressed = (!dnga_get_bit_bool(msg, 1U));

    /* TODO / SUGGESTED SAFETY CHECK:
     *   Consider using gas_pressed in TX hook to suppress or constrain
     *   outgoing accel/brake requests during driver override.
     *   Common policy:
     *   - no positive accel when gas is pressed
     *   - maybe allow only neutral/cancel behavior under override
     */
  } else {
    /* no action */
  }

  if (msg->addr == 161U) {
    /* BRAKE.BRAKE_ENGAGED: bit 5|1@0+. */
    brake_pressed = dnga_get_bit_bool(msg, 5U);

    /* TODO / SUGGESTED SAFETY CHECK:
     *   Consider using brake_pressed in TX hook to reject commanded
     *   actuation when the driver is pressing the brake.
     *   This is a common immediate override/disengage condition.
     */
  } else {
    /* no action */
  }

  /* TODO / SUGGESTED SAFETY CHECK:
   *   If cruise-state RX frames contain a trustworthy engaged/disengaged
   *   state, use them here to:
   *   - allow controls only when stock ACC is truly engaged
   *   - clear controls_allowed immediately on disengage/fault
   *   - require valid recent engagement before first active TX
   *
   *   Current code only receives these frames through rx_checks and does
   *   not semantically use them in rx_hook.
   */

  /* TODO / SUGGESTED SAFETY CHECK:
   *   If the source frames have checksum/counter integrity fields, consider
   *   enforcing them instead of ignoring them in rx_checks.
   *   That would strengthen trust in wheel speed / brake / gas / cruise state.
   */
}

static bool dnga_tx_hook(const CANPacket_t *msg)
{
  bool tx;

  tx = true;

  /* TODO / SUGGESTED SAFETY CHECK:
   *   Consider explicitly using controls_allowed as a hard TX gate for
   *   actuation messages, rather than relying only on payload validation.
   *   Preserved unchanged here because original logic does not do that.
   */

  /* TODO / SUGGESTED SAFETY CHECK:
   *   Consider suppressing or constraining TX based on driver override:
   *   - brake_pressed
   *   - gas_pressed
   *   Typical policies differ by platform, but these states are often used
   *   to reject actuation or allow only cancel / neutral commands.
   */

  /* ACC_CMD_HUD: if engagement isn’t requested, ACC_CMD should be 0. */
  if (msg->addr == 627U) {
    bool set_me_1_2;
    bool set_1_when_engage;
    bool bit_37_set;
    bool bit_38_set;
    bool engage_requested;
    uint32_t acc_cmd_raw;

    set_me_1_2 = dnga_get_bit_bool(msg, 9U);
    set_1_when_engage = dnga_get_bit_bool(msg, 13U);
    bit_37_set = dnga_get_bit_bool(msg, 37U);
    bit_38_set = dnga_get_bit_bool(msg, 38U);

    engage_requested = set_me_1_2 || set_1_when_engage || bit_37_set || bit_38_set;

    if (!engage_requested) {
      acc_cmd_raw = dnga_get_acc_cmd_raw(msg);
      if (acc_cmd_raw != 0U) {
        tx = false;
      } else {
        /* no action */
      }

      /* TODO / SUGGESTED SAFETY CHECK:
       *   When engagement is not requested, consider requiring all other
       *   related ACC fields in this payload to also be neutral/default,
       *   not only ACC_CMD == 0.
       *   This catches inactive-looking messages that still carry
       *   nontrivial side data.
       */
    } else {
      /* no action */
    }

    /* TODO / SUGGESTED SAFETY CHECK:
     *   Consider rate limiting / step limiting ACC_CMD-related fields.
     *   Current code enforces only absolute validity, not how fast the
     *   command can change frame-to-frame.
     *
     *   Useful examples:
     *   - max upward step
     *   - max downward step
     *   - tighter limits on first engage frame
     */

    /* TODO / SUGGESTED SAFETY CHECK:
     *   Consider special engage-edge rules:
     *   - require near-neutral first active command
     *   - require recent valid cruise engagement observation
     *   - reject stale or sudden large first command at engagement
     */
  } else {
    /* no action */
  }

  /* ACC_BRAKE: when engagement isn’t requested, BRAKE_REQ should be 0. */
  if (msg->addr == 625U) {
    bool set_me_1_when_engage;
    bool brake_req;
    uint8_t magnitude_raw;
    uint8_t pump_raw;
    uint8_t pump_inverse_raw;

    set_me_1_when_engage = dnga_get_bit_bool(msg, 8U);

    if (!set_me_1_when_engage) {
      brake_req = dnga_get_bit_bool(msg, 13U);
      if (brake_req) {
        tx = false;
      } else {
        /* no action */
      }

      /* TODO / SUGGESTED SAFETY CHECK:
       *   When engagement is not requested, consider requiring a fully
       *   neutral brake payload, not only BRAKE_REQ == 0.
       *   Example ideas:
       *   - pump_raw == 0
       *   - inverse pump byte == 0
       *   - magnitude == neutral/default encoding if protocol defines one
       */
    } else {
      /* no action */
    }

    /* Enforce command clipping envelope for ACC_BRAKE payload. */
    magnitude_raw = msg->data[5];
    if ((magnitude_raw < DNGA_ACC_BRAKE_MAGNITUDE_RAW_MIN) ||
        (magnitude_raw > DNGA_ACC_BRAKE_MAGNITUDE_RAW_MAX)) {
      tx = false;
    } else {
      /* no action */
    }

    pump_raw = msg->data[4];
    if (pump_raw > DNGA_ACC_BRAKE_PUMP_RAW_MAX) {
      tx = false;
    } else {
      /* no action */
    }

    /* PUMP_REACTION2 should be the signed inverse of PUMP_REACTION1
     * (0, -1, ..., -10 in raw form).
     */
    pump_inverse_raw = (uint8_t)(0U - (uint32_t)pump_raw);
    if (msg->data[3] != pump_inverse_raw) {
      tx = false;
    } else {
      /* no action */
    }

    /* TODO / SUGGESTED SAFETY CHECK:
     *   Consider semantic consistency checks inside ACC_BRAKE, for example:
     *   - if BRAKE_REQ == 0, require neutral pump/magnitude
     *   - if pump_raw > 0, require BRAKE_REQ == 1
     *   - reject impossible field combinations that are individually in-range
     *   - validate any reserved/constant bits if protocol defines them
     */

    /* TODO / SUGGESTED SAFETY CHECK:
     *   Consider rate-of-change / step-limit checks for brake-related fields.
     *   Current code allows any instantaneous jump between two legal values.
     *   More typical safety logic would constrain:
     *   - magnitude increase per frame
     *   - magnitude decrease per frame
     *   - pump step changes
     *   - possibly separate limits for engage vs steady state
     */

  } else {
    /* no action */
  }

  SAFETY_UNUSED(msg);
  return tx;
}

static safety_config dnga_init(uint16_t param)
{
  SAFETY_UNUSED(param);

  /* TODO / SUGGESTED SAFETY CHECK:
   *   Consider initializing controls_allowed from a known-safe default
   *   that requires valid engagement before actuation.
   *   Preserved exactly here to avoid logic changes.
   */
  controls_allowed = true;

  return BUILD_SAFETY_CFG(dnga_rx_checks, DNGA_TX_MSGS);
}

const safety_hooks dnga_hooks = {
  .init = dnga_init,
  .rx = dnga_rx_hook,
  .tx = dnga_tx_hook,
};

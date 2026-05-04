from opendbc.car.crc import CRC8H2F


def proton_checksum(address: int, sig, d: bytearray) -> int:
  """CRC8 8H2F/AUTOSAR. Checksum in last byte; CRC over bytes 0..size-2, final XOR 0xFF."""
  del address, sig
  crc = 0xFF
  for i in range(len(d) - 1):
    crc ^= d[i]
    crc = CRC8H2F[crc]
  return crc ^ 0xFF


def create_can_steer_command(packer, steer, steer_req, wheel_touch_warning, wheel_touch_warning_2,
                             lks_aux, lks_audio, lks_tactile, lks_assist_mode, lka_enable, stock_ldw_ste,
                             steer_enabled, new_lka):

  # Disable steering vibration for LDW if steer not enabled and LKS set to Warn Only mode and Tactile warning type
  ldw_steering = 0 if (
    not steer_enabled and not lks_aux and lks_assist_mode and lks_tactile and not lks_audio
  ) else stock_ldw_ste

  values = {
    "LKA_ENABLE": lka_enable,
    "LKAS_ENGAGED1": steer_req,
    "LKAS_LINE_ACTIVE": steer_req,
    "STEER_CMD": abs(steer) if steer_req else 0,
    "STEER_DIR": steer <= 0,
    "LDW_READY": 1,
    "LDW_STEERING": ldw_steering,
    "SET_ME_1": 1,
    "SET_ME_1_2": new_lka, # Currently only for X90, pre-FL X50 needs to be False or LKS cannot be changed
    "LKS_STATUS": 1,
    "STOCK_LKS_AUX": lks_aux,
    "LKS_WARNING_AUDIO_TYPE": lks_audio,
    "LKS_WARNING_TACTILE_TYPE": lks_tactile,
    "LKS_ASSIST_MODE": lks_assist_mode,
    "HAND_ON_WHEEL_WARNING": wheel_touch_warning,
    "WHEEL_WARNING_CHIME": wheel_touch_warning_2,
  }

  return packer.make_can_msg("ADAS_LKAS", 0, values)


def create_acc_cmd(packer, accel_cmd, long_active, gas_override, car_standstill, stock_values):

  active = gas_override or long_active # long_active is False when gas override, so gas override needs to be checked first.
  car_standstill = car_standstill and long_active and not gas_override # Pass stock values to make resume work.
  cmd = stock_values["CMD"] if gas_override or car_standstill else 0 if not long_active else accel_cmd

  values = {
    "ACC_REQ": stock_values["ACC_REQ"] if car_standstill else active,
    "CRUISE_DISABLED": not active,
    "CMD": cmd,
    "CMD_OFFSET1": cmd,
    "CMD_OFFSET2": cmd,
    "SET_ME_1": 1,

    # Affects X50 engine rpm when standstill; if value too low, S70 ACC does not allow car to move.
    # The value is a lot different on X50 and S70, so the stock value should be passed.
    "SET_ME_X6A": stock_values["SET_ME_X6A"],

    # Not sure
    "STANDSTILL_REQ": stock_values["STANDSTILL_REQ"],
    "STATIONARY": car_standstill and stock_values["STATIONARY"],
    "UNKNOWN1": car_standstill and stock_values["UNKNOWN1"],

    # Affects resume, not sure about if it affects acceleration.
    "MOTION_CONTROL": (
      stock_values["MOTION_CONTROL"] if gas_override or car_standstill else
      4 if (not long_active or cmd < 0) else 6 if cmd > 0 else 1
    ),

    "NOT_MOVE_BLOCK": stock_values["NOT_MOVE_BLOCK"], # If forced to 0, the car stays at standstill and the set speed changes.
    "BRAKE_ENGAGED": stock_values["BRAKE_ENGAGED"], # Not sure, so pass stock value.
    "RISING_ENGAGE": stock_values["RISING_ENGAGE"], # Becomes True then False after resume button presses.
  }

  return packer.make_can_msg("ACC_CMD", 0, values)


def send_buttons(packer, send_cruise=True):
  if send_cruise:
    values = {"CRUISE_BTN": 1}
  else:
    values = {
      "SET_BUTTON": 0,
      "RES_BUTTON": 1,
      "NEW_SIGNAL_1": 1,
      "NEW_SIGNAL_2": 1,
      "SET_ME_BUTTON_PRESSED": 1,
    }

  return packer.make_can_msg("ACC_BUTTONS", 2, values)

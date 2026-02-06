from openpilot.common.numpy_fast import clip

def create_can_steer_command(packer, steer, steer_req, wheel_touch_warning, wheel_touch_warning_2,
    lks_aux, lks_audio, lks_tactile, lks_assist_mode, lka_enable, stock_ldw_ste, steer_enabled, new_lka):

  values = {
    "LKA_ENABLE": lka_enable,
    "LKAS_ENGAGED1": steer_req,
    "LKAS_LINE_ACTIVE": steer_req,
    "STEER_CMD": abs(steer) if steer_req else 0,
    "STEER_DIR": steer <= 0,
    "LDW_READY": 1,
    # Disable steering vibration for LDW if steer not enabled and LKS set to Warn Only mode and Tactile warning type
    "LDW_STEERING": 0 if not steer_enabled and not lks_aux and lks_assist_mode and lks_tactile and not lks_audio else stock_ldw_ste,
    "SET_ME_1": 1,
    "SET_ME_1_2": new_lka, # Currently only for X90, pre FL X50 needs to be False or LKS cannot be changed
    "LKS_STATUS": 1,
    "STOCK_LKS_AUX": lks_aux,
    "LKS_WARNING_AUDIO_TYPE": lks_audio,
    "LKS_WARNING_TACTILE_TYPE": lks_tactile,
    "LKS_ASSIST_MODE": lks_assist_mode,
    "HAND_ON_WHEEL_WARNING": wheel_touch_warning,
    "WHEEL_WARNING_CHIME": wheel_touch_warning_2,
  }

  return packer.make_can_msg("ADAS_LKAS", 0, values)

def create_acc_cmd(packer, accel_cmd, enabled, gas_override, standstill, resume):
  if accel_cmd > 0:
    standstill = False

  values = {
    "CMD": accel_cmd,
    "CMD_OFFSET1": accel_cmd,
    "CMD_OFFSET2": accel_cmd,
    "ACC_REQ": enabled and not resume,
    "CRUISE_DISABLED": not enabled,
    "SET_ME_1": 1,
    "NOT_GAS_OVERRIDE": enabled and not gas_override,
    "RISING_ENGAGE": resume,

    # not sure
    "SET_ME_X6A": 0xFD if resume else 0x6A if standstill else 0xFA,
    "STATIONARY": 0,
    "STANDSTILL_REQ": 0,
    # 5 = Standstill, 6 = Accelerate, 4 = Brake, 1 = Maintain speed
    "MOTION_CONTROL": 9 if resume else 5 if standstill else 6 if accel_cmd > 0 else 4 if accel_cmd < 0 else 1
  }

  return packer.make_can_msg("ACC_CMD", 0, values)

def send_buttons(packer, send_cruise=True):

  if send_cruise:
    values = {
      "CRUISE_BTN": 1,
    }
  else:
    values = {
      "SET_BUTTON": 0,
      "RES_BUTTON": 1,
      "NEW_SIGNAL_1": 1,
      "NEW_SIGNAL_2": 1,
      "SET_ME_BUTTON_PRESSED": 1,
    }

  return packer.make_can_msg("ACC_BUTTONS", 2, values)

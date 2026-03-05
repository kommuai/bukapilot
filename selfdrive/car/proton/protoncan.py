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

def create_acc_cmd(packer, accel_cmd, enabled, gas_override, standstill, resume, brake_pressed):

  values = {
    "CMD": accel_cmd,
    "CMD_OFFSET1": accel_cmd,
    "CMD_OFFSET2": accel_cmd,
    "ACC_REQ": enabled and not resume, # Resume check required for SNG to work but frequent switching can make X50 rpm drop.
    "CRUISE_DISABLED": not enabled,
    "SET_ME_1": 1,
    "NOT_GAS_OVERRIDE": enabled and not gas_override,
    "RISING_ENGAGE": resume,
    "BRAKE_ENGAGED": brake_pressed,
    "UNKNOWN1": 0, # Needs to be 0, if 1 when braking, braking is too hard on S70. This signal might mean hard braking needed.

    # The distance, if value too low, car cannot move when SNG resume.
    "SET_ME_X6A": 0x6A if not enabled else 0xFA if resume else 0x6A if (standstill and accel_cmd <= 0) else 0xFA,

    # 5 = Standstill, 6 = Accelerate, 4 = Brake, 1 = Maintain speed
    "MOTION_CONTROL": 4 if not enabled else 9 if resume else 5 if standstill else 4 if accel_cmd < 0 else 6 if accel_cmd > 0 else 1,

    # Hardcoded the 2 signals to 0 because when SNG resumes EPB is pulled by the car.
    # Hardcoding to 0 makes X50 and X50 FL engine run higher rpm when stopped.
    "STATIONARY": 0,
    "STANDSTILL_REQ": 0,
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

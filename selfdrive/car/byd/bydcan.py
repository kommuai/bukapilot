def create_can_steer_command(packer, steer_angle, steer_req, is_standstill, ecu_fault, recovery_btn):

  set_me_xe = 0xE if is_standstill else 0xB
  eps_ok = not steer_req
  if recovery_btn:
    eps_ok = ecu_fault
  values = {
    "STEER_REQ": steer_req,
    # to recover from ecu fault, it must be momentarily pulled low.
    "EPS_OK": eps_ok,
    "STEER_ANGLE": steer_angle,
    # must be 0x1 to steer
    "SET_ME_X01": 0x1 if steer_req else 0,
    # 0xB fault lesser, maybe higher value fault lesser, 0xB also seem to have the highest angle limit at high speed.
    "SET_ME_XE": set_me_xe if steer_req else 0,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    "UNKNOWN": 2773 if steer_req else 0,
    }

  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)

def create_accel_command(packer, accel, enabled, accel_mult, brake_hold):
  accel = max(min(accel * accel_mult, 30), -50)
  accel_factor = 12 if accel >= 2 else 5 if accel < 0 else 11
  enabled &= not brake_hold

  if brake_hold:
    accel = 0

  values = {
    "ACCEL_CMD": accel,
    # always 25
    "SET_ME_25_1": 25,
    "SET_ME_25_2": 25,
    "ACC_ON_1": enabled,
    "ACC_ON_2": enabled,
    #"ENGAGE_BIT": enabled,  # Keep ENGAGE_BIT based on openpilot's enabled state
    # some unknown state, 12 when accel, below 11 when braking, 11 when cruising
    "ACCEL_FACTOR": accel_factor if enabled else 0,
    # some unknown state, 0 when not engaged, 3/4 when accel, 8/9 when accel uphill, 1 when braking (all speculation)
    "DECEL_FACTOR": 8 if enabled else 0,
    "SET_ME_X8": 8,
    "SET_ME_1": 1,
    "SET_ME_XF": 0xF,
    "CMD_REQ_ACTIVE_LOW": 0 if enabled else 1,
    "ACC_REQ_NOT_STANDSTILL": enabled,
    "ACC_CONTROLLABLE_AND_ON": enabled,
    "ACC_OVERRIDE_OR_STANDSTILL": brake_hold,
    "STANDSTILL_STATE": brake_hold,
    "STANDSTILL_RESUME": 0, # TODO integrate buttons
  }

  return packer.make_can_msg("ACC_CMD", 0, values)

# 50hz
def create_lkas_hud(packer, lat_active, lss_state, lss_alert, tsr, ahb, passthrough,\
    hma, pt2, pt3, pt4, pt5, lka_on):

  values = {
    "STEER_ACTIVE_ACTIVE_LOW": lka_on,
    "LEFT_LANE_VISIBLE": lat_active,
    "LKAS_ENABLED": lat_active,
    "RIGHT_LANE_VISIBLE": lat_active,
    "LSS_STATE": lss_state,
    "SET_ME_1_2": 1,
    "SETTINGS": lss_alert,
    "TSR_STATUS": passthrough,
    "SET_ME_XFF": ahb,
    # TODO integrate warning signs when steer limited
    "HAND_ON_WHEEL_WARNING": 0,
    "TSR": tsr,
    "HMA": hma,
    "PT2": pt2,
    "PT3": pt3,
    "PT4": pt4,
    "PT5": pt5,
  }

  return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)

def send_buttons(packer, state, cancel):
  values = {
      "SET_BTN": state,
      "RES_BTN": state,
      "SET_ME_1_1": 1,
      "SET_ME_1_2": 1,
      "ACC_ON_BTN": cancel,
      "LKAS_ON_BTN": 0,
  }
  return packer.make_can_msg("PCM_BUTTONS", 2, values)

def create_steering_torque_spoof_main(packer, driver_torque=10, steer_angle=-7.7, counter=0):
  """
  Spoof steering torque on camera bus (2) to simulate hands on wheel
  Sends STEER_MODULE_2 message
  Address: 0x11F (287 decimal), Bus: 2 (camera)
  Message length: 5 bytes
  Frequency: ~20 Hz (every 2-3 frames at 50 Hz base rate)

  Based on log analysis:
  - Bytes 0-1: STEER_ANGLE_2 = -77 (-7.7 degrees) - constant in logs
  - Byte 2: Constant 0x00
  - Byte 3: Constant 0xFF
  - Byte 4: Counter increments by 0x11 (17 decimal): 0x08, 0x19, 0x2A, 0x3B, 0x4C, 0x5D, 0x6E, 0x7F, 0x80, 0x91, 0xA2, 0xB3, 0xC4, 0xD5, 0xE6, 0xF7
  """
  from openpilot.common.numpy_fast import clip
  # Counter pattern from logs: increments by 0x11, wraps through 16 values
  counter_values = [0x08, 0x19, 0x2A, 0x3B, 0x4C, 0x5D, 0x6E, 0x7F, 0x80, 0x91, 0xA2, 0xB3, 0xC4, 0xD5, 0xE6, 0xF7]
  counter_val = counter_values[counter % len(counter_values)]

  values = {
    "STEER_ANGLE_2": clip(steer_angle, -3276.8, 3276.7),  # Physical value in degrees, packer applies factor 0.1
    "DRIVER_EPS_TORQUE": int(clip(driver_torque, 0, 255)),
    "PAYLOAD": 0xFF,  # Constant 0xFF based on log analysis
    "MSG_COUNTER": counter_val,  # Counter pattern from logs
  }
  return packer.make_can_msg("STEER_MODULE_2", 2, values)

import random

def create_steering_torque_spoof_camera(packer, lat_active, main_torque=-15):
  """
  Spoof steering torque on camera bus (2) to simulate hands on wheel
  Sends STEERING_TORQUE message
  Address: 0x1FC (508 decimal), Bus: 2 (camera)
  """

  curvature = random.uniform(-5.0, 5.0) if lat_active else 0.0
  unknown_torque = random.uniform(-15.0, 15.0) if lat_active else main_torque

  values = {
    "MAIN_TORQUE": unknown_torque / 10,            # Physical value in Nm
    "STATE": 10 if lat_active else 9,
    "CURVATURE": curvature,
    "UNKNOWN_TORQUE": unknown_torque,
    "STEER_ACTIVE": lat_active,
    "STEER_STATE": 0 if lat_active else 6, # 6 on start, 0 OK, 4 fault
  }

  return packer.make_can_msg("STEERING_TORQUE", 2, values)

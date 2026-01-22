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

def send_buttons(packer, state, cancel, bus):
  values = {
      "SET_BTN": state,
      "RES_BTN": state,
      "SET_ME_1_1": 1,
      "SET_ME_1_2": 1,
      "ACC_ON_BTN": cancel,
  }
  return packer.make_can_msg("PCM_BUTTONS", bus, values)

import random

# Module-level state for realistic torque ramp-up simulation
_torque_spoof_state = {
  'current_torque_offset': 0.0,  # Current accumulated torque offset
  'target_torque': 0.0,          # Target torque when spoof is active
  'ramp_rate': 3.0,              # Nm per call ramp rate (realistic steering nudge at ~10 Hz)
  'max_torque': 10.0,            # Maximum torque offset (realistic hand touch, Nm)
}

def create_steering_torque_spoof_camera(packer, lat_active, main_torque, spoof):
  """
  Spoof steering torque on camera bus (2) to simulate hands on wheel
  Sends STEERING_TORQUE message
  Address: 0x1FC (508 decimal), Bus: 2 (camera)

  When spoof is True, simulates realistic hand touch by gradually ramping up torque
  like a natural steering nudge, rather than random jumps.
  """

  if spoof:
    # Realistic hand touch simulation: gradually ramp up torque like a steering nudge
    # Set target torque (with slight variation for realism)
    if abs(_torque_spoof_state['target_torque']) < 0.1:
      # Start new nudge - choose target torque (slightly randomized for natural feel)
      _torque_spoof_state['target_torque'] = random.uniform(5.0, _torque_spoof_state['max_torque'])
      # Occasionally apply negative torque for bidirectional realism
      if random.random() < 0.3:
        _torque_spoof_state['target_torque'] = -_torque_spoof_state['target_torque']

    # Ramp towards target torque (realistic steering nudge behavior)
    target = _torque_spoof_state['target_torque']
    current = _torque_spoof_state['current_torque_offset']
    ramp = _torque_spoof_state['ramp_rate']

    if abs(target - current) > 0.1:
      # Still ramping - move towards target
      if current < target:
        _torque_spoof_state['current_torque_offset'] = min(current + ramp, target)
      else:
        _torque_spoof_state['current_torque_offset'] = max(current - ramp, target)
    else:
      # Reached target - occasionally vary slightly for natural hand movement
      if random.random() < 0.2:
        _torque_spoof_state['target_torque'] = random.uniform(4.0, _torque_spoof_state['max_torque'])
        if random.random() < 0.5:
          _torque_spoof_state['target_torque'] = -_torque_spoof_state['target_torque']
  else:
    # spoof is False - gradually ramp down torque back to zero
    current = _torque_spoof_state['current_torque_offset']
    if abs(current) > 0.1:
      # Ramp down towards zero
      if current > 0:
        _torque_spoof_state['current_torque_offset'] = max(current - _torque_spoof_state['ramp_rate'], 0.0)
      else:
        _torque_spoof_state['current_torque_offset'] = min(current + _torque_spoof_state['ramp_rate'], 0.0)
    else:
      # Reset state when torque returns to zero
      _torque_spoof_state['current_torque_offset'] = 0.0
      _torque_spoof_state['target_torque'] = 0.0

  # Add the realistic ramp-up torque offset to main_torque
  main_torque += _torque_spoof_state['current_torque_offset']

  values = {
    "MAIN_TORQUE": main_torque,
    "STATE": 10 if lat_active else 9,
    "CURVATURE": 0,
    "UNKNOWN_TORQUE": 0,
    "STEER_ACTIVE": lat_active,
    "STEER_STATE": 0 if lat_active else 6, # 6 on start, 0 OK
  }

  return packer.make_can_msg("STEERING_TORQUE", 2, values)

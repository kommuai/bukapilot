from common.numpy_fast import clip, interp
from cereal import car
from selfdrive.config import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert

def perodua_checksum(addr,dat):
  return ( addr + len(dat) + 1 + 1 + sum(dat)) & 0xFF

def perodua_acc_checksum(addr,dat):
  return ( addr + len(dat) + 1 + 2 + sum(dat)) & 0xFF

def create_can_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Perodua LKA Steer Command."""

  values = {
    "STEER_REQ": steer_req,
    "STEER_CMD": -steer if steer_req else 0,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
    "SET_ME_1_2": 1,
  }

  dat = packer.make_can_msg("STEERING_LKAS", 0, values)[2]
  crc = perodua_checksum(0x1d0, dat[:-1])
  values["CHECKSUM"] = crc


  return packer.make_can_msg("STEERING_LKAS", 0, values)

def crc8_interceptor(data):
  crc = 0xFF                                                         # standard init value
  poly = 0xD5                                                        # standard crc8: x8+x7+x6+x4+x2+1
  size = len(data)
  for i in range(size - 1, -1, -1):
    crc ^= data[i]
    for _ in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc

def create_steer_command(packer, command, direction, enable, idx):
  """Creates a CAN message for the steering command."""

  values = {
    "INTERCEPTOR_MAIN_TORQUE": abs(command),
    "INTERCEPTOR_SUB_TORQUE": abs(command),
    "DIRECTION": 0 if direction else 1,
    "ENABLE": 1 if enable else 0,
    "COUNTER_STEERING": idx & 0xF,
  }

  dat = packer.make_can_msg("TORQUE_COMMAND", 0, values)[2]

  crc = crc8_interceptor(dat[:-1])
  values["CHECKSUM_STEERING"] = crc

  return packer.make_can_msg("TORQUE_COMMAND", 0, values)

def perodua_create_gas_command(packer, gas_amount, enable, idx):

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    # the value 3000 is a limiting constant, allow an addition of
    # 2500/4095 * 3.3 = 2.01V.
    values["GAS_COMMAND"] = gas_amount
    values["GAS_COMMAND2"] = gas_amount

  dat = packer.make_can_msg("GAS_COMMAND", 0, values)[2]
  checksum = crc8_interceptor(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", 0, values)

def perodua_aeb_brake(packer, brake_amount):

  values = {
    "AEB_ALARM": 1 if (brake_amount > 0.5) else 0,
    "SET_ME_XB2": 0xB2,
  }

  return packer.make_can_msg("FWD_CAM3", 0, values)

def perodua_create_brake_command(packer, enabled, decel_cmd, idx):
  decel_req = decel_cmd > 0.0
  pump_speed = interp(decel_cmd, [0., 0.8], [0.4, 1.0])

  values = {
    "COUNTER": idx,
    "PUMP_REACTION1": pump_speed if (decel_req and enabled) else 0,
    "BRAKE_REQ": decel_req,
    "MAGNITUDE": (-1* decel_cmd) if (enabled and decel_req) else 0,
    "SET_ME_1_WHEN_ENGAGE": 1 if enabled else 0,
    "PUMP_REACTION2": (-1* pump_speed) if (enabled and decel_req) else 0,
  }

  dat = packer.make_can_msg("ACC_BRAKE", 0, values)[2]
  crc = (perodua_acc_checksum(0x271, dat[:-1]))
  values["CHECKSUM"] = crc

  return packer.make_can_msg("ACC_BRAKE", 0, values)

def perodua_create_accel_command(packer, v_ego, set_speed, acc_rdy, enabled, is_lead, des_speed, brake_amt):

  # accel limiter
  if(v_ego >= set_speed):
    acc_des_speed = (set_speed * 0.036)
  elif(set_speed - v_ego > 0.01):
    acc_des_speed = (v_ego * 0.036 + 0.01)
  else:
    acc_des_speed = (v_ego * 0.036 + (set_speed - v_ego))
  # dont release gas at low speed to prevent rocking
  if(v_ego > 8):
    acc_des_speed = acc_des_speed * (1 - brake_amt * 0.25)
  is_braking = brake_amt > 0.0
  
  values = {
    "SET_SPEED": set_speed * 3.6,
    "FOLLOW_DISTANCE": 0,
    "IS_LEAD": is_lead,
    "IS_ACCEL": (not is_braking) and enabled,
    "IS_DECEL": is_braking and enabled,
    "SET_ME_1_2": acc_rdy, #rdy buton
    "SET_ME_1": 1,
    "SET_0_WHEN_ENGAGE": not enabled,
    "SET_1_WHEN_ENGAGE": enabled,
    "ACC_CMD": des_speed * 0.036 if enabled else 0,
  }

  dat = packer.make_can_msg("ACC_CMD_HUD", 0, values)[2]
  crc = (perodua_acc_checksum(0x273, dat[:-1]))
  values["CHECKSUM"] = crc

  return packer.make_can_msg("ACC_CMD_HUD", 0, values)

def perodua_create_hud(packer, lkas_rdy, enabled, llane_visible, rlane_visible):

  values = {
    "LKAS_SET": lkas_rdy,
    "LKAS_ENGAGED": enabled,
    "LDA_ALERT": 0,
    "LANE_RIGHT_DETECT": rlane_visible,
    "LANE_LEFT_DETECT": llane_visible,
    "SET_ME_X02": 0x2,
  }

  dat = packer.make_can_msg("LKAS_HUD", 0, values)[2]
  crc = (perodua_acc_checksum(0x274, dat[:-1]))
  values["CHECKSUM"] = crc

  return packer.make_can_msg("LKAS_HUD", 0, values)



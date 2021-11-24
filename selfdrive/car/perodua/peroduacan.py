from common.numpy_fast import clip
from cereal import car

VisualAlert = car.CarControl.HUDControl.VisualAlert

def create_can_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Perodua LKA Steer Command."""

  values = {
    "STEER_REQ": steer_req,
    "STEER_CMD": steer,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
  }

  dat = packer.make_can_msg("STEERING_LKAS", 0, values)[2]
  crc = crc8_interceptor(dat[:-1])
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


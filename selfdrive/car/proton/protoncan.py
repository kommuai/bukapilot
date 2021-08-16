from common.numpy_fast import clip
from cereal import car

VisualAlert = car.CarControl.HUDControl.VisualAlert

#https://github.com/commaai/openpilot/blob/825821f010db63c3498d3730069f3eac08ace789/selfdrive/car/__init__.py#L87
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

def proton_create_gas_command(packer, gas_amount, enable, idx):

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

# Not used
def proton_aeb_brake(packer, brake_amount):

  values = {
    "AEB_ALARM": 1 if (brake_amount > 0.5) else 0,
    #"COMPUTER_BRAKE": apply_brake,
    #"BRAKE_PUMP_REQUEST": pump_on,
    #"CRUISE_OVERRIDE": pcm_override,
    #"CRUISE_FAULT_CMD": pcm_fault_cmd,
    #"CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    #"COMPUTER_BRAKE_REQUEST": brake_rq,
    #"SET_ME_1": 1,
    #"BRAKE_LIGHTS": brakelights,
    #"CHIME": stock_brake["CHIME"] if fcw else 0,  # send the chime for stock fcw
    #"FCW": fcw << 1,  # TODO: Why are there two bits for fcw?
    #"AEB_REQ_1": 0,
    #"AEB_REQ_2": 0,
    #"AEB_STATUS": 0,
  }

  return packer.make_can_msg("FWD_CAM3", 0, values)

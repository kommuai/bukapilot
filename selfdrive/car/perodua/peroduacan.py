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
    "DIRECTION": 1 if direction else 0,
    "ENABLE": 1 if enable else 0,
    "COUNTER_STEERING": idx & 0xF,
  }

  dat = packer.make_can_msg("TORQUE_COMMAND", 0, values)[2]

  crc = crc8_interceptor(dat[:-1])
  values["CHECKSUM_STEERING"] = crc

  return packer.make_can_msg("TORQUE_COMMAND", 0, values)


from common.numpy_fast import clip, interp
from cereal import car

SetDistance = car.CarState.CruiseState.SetDistance

def compute_set_distance(state):
  if state == SetDistance.aggresive:
    return 2
  elif state == SetDistance.normal:
    return 1
  else:
    return 0

# not an actual known crc function, reverse engineered
def byte_crc4_linear_inverse(byte_list):
  return (-1 * sum(byte_list) + 0x9) & 0xf

def byd_checksum(byte_key, dat):

  second_bytes = [byte & 0xf for byte in dat]
  remainder = sum(second_bytes) >> 4
  second_bytes.append(byte_key >> 4)


  first_bytes = [byte >> 4 for byte in dat]
  first_bytes.append(byte_key & 0xf)

  return (((byte_crc4_linear_inverse(first_bytes) + (-1*remainder + 5)) << 4) + byte_crc4_linear_inverse(second_bytes)) & 0xff

def create_can_steer_command(packer, steer_angle, steer_req, raw_cnt):

  values = {
    "TORQUE": 0,       # not actual torque, useless params, leaving the name as it is first
    "STEER_REQ": steer_req,
    "STEER_REQ_ACTIVE_LOW": not steer_req,
    "STEER_ANGLE": steer_angle * 0.86,            # desired steer angle
    #"STEER_ANGLE": steer_angle * 0.958,            # desired steer angle
    "SET_ME_X01": 0x1 if steer_req else 0, # must be 0x1 to steer
    "SET_ME_XEB": 0xE if steer_req else 0, # 0xB fault lesser, seems like steer torque
    "COUNTER": raw_cnt,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    }

  dat = packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)[2]
  crc = byd_checksum(0xaf, dat[:-1])
  values["CHECKSUM"] = crc
  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)

def create_accel_command(packer, accel, enabled, is_standstill,  raw_cnt):

  values = {
    "ACCEL_CMD": accel,
    "SET_ME_25_1": 25,                     # always 25
    "SET_ME_25_2": 25,                     # always 25
    "COUNTER": raw_cnt,
    "ACC_ON_1": enabled,
    "ACC_ON_2": enabled,
    "UNKNOWN1": 2 if enabled else 0,       # 2 is needed to brake, 1 is to cruise, 3 is to accel, 4-9 more power?
    "UNKNOWN2": 12 if enabled else 0,      # prioritise test 12-14, was 12
    "SET_ME_X8": 8,
    "SET_ME_1": 1,
    "SET_ME_XF": 0xF,
    "CMD_REQ_ACTIVE_LOW": 0 if enabled else 1,
    "ACC_REQ_NOT_STANDSTILL": enabled,
    "ACC_CONTROLLABLE_AND_ON": enabled,
    "ACC_OVERRIDE_OR_STANDSTILL": is_standstill,
    "STANDSTILL_STATE": 0,                 # TODO integrate vEgo check
    "STANDSTILL_RESUME": 0,                # TODO integrate buttons
  }

  dat = packer.make_can_msg("ACC_CMD", 0, values)[2]
  crc = byd_checksum(0xaf, dat[:-1])
  values["CHECKSUM"] = crc
  return packer.make_can_msg("ACC_CMD", 0, values)

# 50hz
def create_lkas_hud(packer, enabled, is_standstill, raw_cnt):

  values = {
    "STEER_ACTIVE_ACTIVE_LOW": not enabled,
    "STEER_ACTIVE_1_1": enabled,
    "STEER_ACTIVE_1_2": enabled,
    "STEER_ACTIVE_1_3": enabled,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    "SET_ME_XE": 0xE,
    "SET_ME_X5F": 0x5F,
    "SET_ME_XFF": 0xFF,
    "HAND_ON_WHEEL_WARNING": 0,           # TODO integrate warning signs when steer limited
    "COUNTER": raw_cnt,
  }

  dat = packer.make_can_msg("LKAS_HUD_ADAS", 0, values)[2]
  crc = byd_checksum(0xaf, dat[:-1])
  values["CHECKSUM"] = crc
  return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)

def send_buttons(packer, count):
  """Spoof ACC Button Command."""
  values = {
      "SET_BUTTON": 1,
      "RES_BUTTON": 1,
      "SET_ME_1_1": 1,
      "SET_ME_1_2": 1,
      "COUNTER": count,
  }
  dat = packer.make_can_msg("PCM_BUTTONS", 0, values)[2]
  crc = byd_checksum(0xaf, dat[:-1])
  values["CHECKSUM"] = crc
  return packer.make_can_msg("PCM_BUTTONS", 0, values)


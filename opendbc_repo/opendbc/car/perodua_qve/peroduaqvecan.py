from opendbc.car import make_can_msg
from opendbc.car.can_definitions import CanData
from opendbc.car.perodua_qve.qve_checksum_data import (
  CAM_B0_DUPLICATES,
  CAM_B0_IDLE_LUT,
  CAM_B0_IDLE_PAYLOAD,
  CAM_B0_PAYLOAD_BIT_MASKS,
  CAM_B1_IDLE_LUT,
  CAM_B1_IDLE_PAYLOAD,
  CAM_B1_PAYLOAD_BIT_MASKS,
  LUT_ACC,
  LUT_LKAS,
  P_ACC,
  P_LKAS,
)
from opendbc.car.perodua_qve.values import CANBUS

LKAS_ADDR = 0xA5
ACC_ADDR = 0x1AE
LKAS_STEER_NEUTRAL_DEG = 0.0

CAM_B0_ADDR = 0xB0
CAM_B1_ADDR = 0xB1


def qve_checksum(address, sig, d: bytearray) -> int:
  del sig
  if address == LKAS_ADDR:
    x = LUT_LKAS.get(d[1], 0)
    p = P_LKAS
  elif address == ACC_ADDR:
    c = d[1]
    x = LUT_ACC.get(c, LUT_ACC.get(c % 15, 0))
    p = P_ACC
  else:
    return 0
  for i in range(2, 8):
    x ^= p[d[i]]
  return x


def _lkas_byte1_values(counter_byte: int) -> dict[str, int]:
  return {
    "SET_ME_7": 7,
    "COUNTER": counter_byte & 0x0F,
  }


def create_lkas_command(packer, angle_deg: float, active: bool, counter_byte: int):
  cmd = angle_deg if active else LKAS_STEER_NEUTRAL_DEG
  values = {
    **_lkas_byte1_values(counter_byte),
    "STEER_REQ": active,
    "LKAS_B2": 0xFF,
    "SET_ME_1": 1,
    "STEER_CMD": cmd,
    "LKAS_B5": 0x92,
    "LKAS_B6": 0xDB,
    "LKAS_B7": 0x6A,
  }
  return packer.make_can_msg("LKAS", CANBUS.pt, values)


def create_acc_command(packer, acc_req: bool, counter: int):
  return packer.make_can_msg("ADAS_ACC", CANBUS.pt, {
    "COUNTER": counter,
    "ACC_BYTE2": 0xFF if acc_req else 0x00,
    "ACC_REQ": acc_req,
    "ACC_CMD": 0,
    "ACC_BYTE6": 0x60,
  })


def _cam_cnt_word(b2: int) -> int:
  b2 &= 0xFF
  return (b2 << 8) | (b2 & 0x0F)


def _cam_payload_g(payload: bytes, idle: bytes, masks: list[int]) -> int:
  x = 0
  for bi, (pb, ib) in enumerate(zip(payload, idle, strict=True)):
    diff = pb ^ ib
    if not diff:
      continue
    base = bi * 8
    for bit in range(8):
      if diff & (1 << bit):
        x ^= masks[base + bit]
  return x


def _cam_chk16(idle_lut: dict[int, int], payload: bytes, idle: bytes, masks: list[int], b2: int) -> int:
  return idle_lut[_cam_cnt_word(b2)] ^ _cam_payload_g(payload, idle, masks)


def _enforce_b0_duplicates(payload60: bytearray) -> None:
  # duplicates are frame-byte indices; payload starts at frame byte 4
  for dst, src in CAM_B0_DUPLICATES.items():
    payload60[dst - 4] = payload60[src - 4]


def pack_cam_b0(payload60: bytes | bytearray, b2: int) -> bytes:
  """Pack 64B cam lane FD frame 0xB0 with GF2 checksum + required duplicates."""
  if len(payload60) != 60:
    raise ValueError(f"B0 payload must be 60 bytes, got {len(payload60)}")
  pl = bytearray(payload60)
  _enforce_b0_duplicates(pl)
  chk = _cam_chk16(CAM_B0_IDLE_LUT, pl, CAM_B0_IDLE_PAYLOAD, CAM_B0_PAYLOAD_BIT_MASKS, b2)
  frame = bytearray(64)
  frame[0] = (chk >> 8) & 0xFF
  frame[1] = chk & 0xFF
  frame[2] = b2 & 0xFF
  frame[3] = b2 & 0x0F
  frame[4:] = pl
  return bytes(frame)


def pack_cam_b1(payload12: bytes | bytearray, b2: int) -> bytes:
  """Pack 16B cam lane FD frame 0xB1 with GF2 checksum."""
  if len(payload12) != 12:
    raise ValueError(f"B1 payload must be 12 bytes, got {len(payload12)}")
  pl = bytes(payload12)
  chk = _cam_chk16(CAM_B1_IDLE_LUT, pl, CAM_B1_IDLE_PAYLOAD, CAM_B1_PAYLOAD_BIT_MASKS, b2)
  frame = bytearray(16)
  frame[0] = (chk >> 8) & 0xFF
  frame[1] = chk & 0xFF
  frame[2] = b2 & 0xFF
  frame[3] = b2 & 0x0F
  frame[4:] = pl
  return bytes(frame)


def create_cam_lane_spoof(b2: int, line_mux: int = 0) -> list[CanData]:
  """Idle/placeholder B0+B1 on cam bus. Does not TX 0x80–0x88 or B2–B7."""
  pl_b0 = bytearray(CAM_B0_IDLE_PAYLOAD)
  pl_b0[0] = line_mux & 0xFF  # frame byte4 LINE_MUX
  b0 = pack_cam_b0(pl_b0, b2)
  b1 = pack_cam_b1(CAM_B1_IDLE_PAYLOAD, b2)
  return [
    make_can_msg(CAM_B0_ADDR, b0, CANBUS.cam),
    make_can_msg(CAM_B1_ADDR, b1, CANBUS.cam),
  ]

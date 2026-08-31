"""Chery / Jaecoo ADAS CAN TX (chery_general_pt.dbc)."""

import random
from dataclasses import dataclass, field

from opendbc.car import rate_limit
from opendbc.car.chrysler.chryslercan import chrysler_checksum
from opendbc.car.crc import CRC8J1850
from opendbc.car.chery.values import (
  CANBUS,
  GAS_ADDR,
  GAS_SPOOF_RAW,
  LANE_KEEP_PADDING,
  SPOOF_NEG_PROB,
  SPOOF_TORQUE_MAX,
  SPOOF_TORQUE_MIN,
  SPOOF_TORQUE_RAMP,
  SPOOF_TORQUE_VAR_MIN,
  SPOOF_VAR_PROB,
  Tiggo22SteerLimits,
)

TIGGO22_LK_ADDR = 0x220

# --- Re-exported / referenced by opendbc.can.dbc; do not rename. ---

def chery_checksum(address: int, sig, d: bytearray) -> int:
  del address, sig
  crc = 0
  for i in range(len(d) - 1):
    crc ^= d[i]
    crc = CRC8J1850[crc]
  return crc ^ 0x0A


def chery_pcm_buttons_checksum(address: int, sig, d: bytearray) -> int:
  del address, sig
  return chrysler_checksum(0, None, bytearray(list(d[1:6]) + [0]))


# --- TX builders ---

def create_lane_keep_command(packer, steer_angle_deg, steer_req, meas_angle_deg):
  cmd = steer_angle_deg if steer_req else meas_angle_deg
  return packer.make_can_msg("LANE_KEEP", CANBUS.main_bus, {
    "STEER_CMD_ANGLE": float(cmd),
    "LKAS_ENABLE": int(steer_req),
    **LANE_KEEP_PADDING,
  })


def _finalize_tiggo22_lane_keep(data: bytearray, steer_req: bool, counter: int) -> None:
  """Tiggo 2022-24 0x220 byte1: STEER_STATE bits 4..2 (4=LKA, 2=ACC_ONLY) + 3-bit counter."""
  steer_state = 4 if steer_req else 2  # stock ACC-on idle is 2, not 0
  data[1] = ((steer_state & 0x7) << 2) | (((int(counter) % 6) << 5) & 0xE0)
  data[7] = chery_checksum(TIGGO22_LK_ADDR, None, data)


def create_tiggo22_lane_keep_command(packer, torque, steer_req, counter, bus):
  """Tiggo 2022-24 0x220 EPS torque. Neutral STEER_CMD=0; clip to tested ±STEER_MAX."""
  torque = max(-Tiggo22SteerLimits.STEER_MAX, min(Tiggo22SteerLimits.STEER_MAX, int(round(float(torque)))))
  addr, payload, bus = packer.make_can_msg("TIGGO22_LANE_KEEP", bus, {
    "STEER_CMD": torque,
    "COUNTER": int(counter) % 16,
  })
  data = bytearray(payload)
  _finalize_tiggo22_lane_keep(data, steer_req, counter)
  return addr, bytes(data), bus


def create_steer_status_spoof(packer, counter, cam_status, bus=None):
  """Re-emit STEER_STATUS (0x307) on PT while engaged. LKAS_FAULT forced off.

  Availability/ADAS copied from cam. Tiggo 2022-24: panda forwards cam 0x307 when
  not engaged; this TX runs only while CC.enabled.
  """
  if bus is None:
    bus = CANBUS.main_bus
  return packer.make_can_msg("STEER_STATUS", bus, {
    "LKAS_AVAILABLE": int(cam_status["LKAS_AVAILABLE"]),
    "LKAS_FAULT": 0,
    "LKAS_AVAILABLE_2": int(cam_status["LKAS_AVAILABLE_2"]),
    "UNKNOWN": int(cam_status.get("UNKNOWN", 0)),
    "ADAS_STATE": int(cam_status["ADAS_STATE"]),
    "COUNTER": int(counter) % 16,
  })


_PCM_BUTTON_FIELDS = ("ICC_TOGGLE", "CRUISE_BUTTON", "RES_BUTTON")


def create_hud_override(packer, cam_hud: dict, counter: int):
  """Re-emit camera HUD on PT bus with the cancel/uncertain indicator forced off.

  HANDS_ON_WHEEL_STEER (HUD 0x387 byte0 bit4) is actually the "ACC cancelled / hands-on
  uncertain" indicator the cluster latches AFTER cruise drops — not the active hands-on-wheel
  warning during LKAS (that lives on STEER_STATUS 0x307 byte5 bit6). We still zero it here so
  spurious cancel-glyphs from the cam don't reach the cluster. Panda blocks camera HUD fwd
  (chery_fwd_hook); this frame is the only HUD on bus 0.
  """
  signals = {k: cam_hud[k] for k in cam_hud if k != "HANDS_ON_WHEEL_STEER"}
  signals["HANDS_ON_WHEEL_STEER"] = 0
  signals["COUNTER"] = int(counter) % 16
  return packer.make_can_msg("HUD", CANBUS.main_bus, signals)


def create_pcm_button(packer, counter: int, bus: int, button: str):
  """Build a PCM_BUTTONS frame asserting exactly one of ICC_TOGGLE / CRUISE_BUTTON (SET) / RES_BUTTON.

  Stock layout (from a real bus capture, 2026-05-26):
    byte0 = chrysler_checksum(byte1..5 + 0)
    byte1 high nibble = COUNTER (stock only uses 0,2,4,...,14 → effectively 3-bit at bits 15..13)
    byte3 bit 0 = ICC_TOGGLE
    byte3 bit 6 = RES_BUTTON     (mask 0x40)
    byte4 bit 0 = CRUISE_BUTTON  (SET; mask 0x01)
  TX on bus 0 (PT) and bus 2 (camera): panda does not forward our own TX between buses.
  """
  assert button in _PCM_BUTTON_FIELDS, button
  signals = {b: int(b == button) for b in _PCM_BUTTON_FIELDS}
  signals["COUNTER"] = int(counter) % 16
  return packer.make_can_msg("PCM_BUTTONS", bus, signals)


def _gas_checksum(data: bytearray) -> int:
  return (~sum(data[:7])) & 0xFF


def gas_checksum_ok(dat: bytes) -> bool:
  return len(dat) >= 8 and dat[7] == _gas_checksum(bytearray(dat[:8]))


def gas_looks_idle(dat: bytes) -> bool:
  """Stock idle pedal frame — safe slot to overlay a tap without fighting ACC throttle."""
  return len(dat) >= 8 and dat[4] == 0x07 and dat[3] == 0


def gas_looks_pressed(dat: bytes) -> bool:
  """Stock/spoof press: byte4==0x08 and throttle/pedal nibble non-zero."""
  return len(dat) >= 8 and dat[4] == 0x08 and dat[3] != 0


def create_gas_pedal_spoof(stock: bytes, bus: int, pressed: bool = True):
  """Tiggo 2022-24 GAS (0xFA) on PT (bus 0). Stock ACC resumes from standstill on a pedal tap.

  Overlay a press on a live stock frame. Keep stock's decrementing byte2 counter and
  unused bytes; recompute byte7 = (~sum(bytes0..6)) & 0xFF.
  """
  if stock and len(stock) >= 8:
    data = bytearray(stock[:8])
  else:
    data = bytearray(b"\x00\x00\x00\x00\x07\xd0\x00\x00")
  raw = int(GAS_SPOOF_RAW) & 0x3F
  if pressed:
    data[3] = raw
    data[4] = 0x08
    data[6] = (data[6] & 0xC0) | raw
  else:
    data[3] = 0
    data[4] = 0x07
    data[6] = data[6] & 0xC0
  data[7] = _gas_checksum(data)
  return int(GAS_ADDR), bytes(data), bus


# --- LKAS torque spoof: keeps stock "hands on wheel" detector quiet during LKAS. ---

@dataclass
class _TorqueSpoof:
  offset: float = 0.0
  target: float = 0.0
  ramp: float = field(default=SPOOF_TORQUE_RAMP)
  max_torque: float = field(default=SPOOF_TORQUE_MAX)

  def _pick_target(self) -> None:
    self.target = random.uniform(SPOOF_TORQUE_MIN, self.max_torque)
    if random.random() < SPOOF_NEG_PROB:
      self.target = -self.target

  def _maybe_variate(self) -> None:
    if random.random() < SPOOF_VAR_PROB:
      self.target = random.uniform(SPOOF_TORQUE_VAR_MIN, self.max_torque)
      if random.random() < 0.5:
        self.target = -self.target

  def step(self, active: bool, apply_offset: bool) -> float:
    if active and apply_offset:
      if abs(self.target) < 0.1:
        self._pick_target()
      if abs(self.target - self.offset) > 0.1:
        self.offset = rate_limit(self.target, self.offset, -self.ramp, self.ramp)
      else:
        self._maybe_variate()
    elif abs(self.offset) > 0.1:
      self.offset = rate_limit(0.0, self.offset, -self.ramp, self.ramp)
    else:
      self.offset = self.target = 0.0
    return self.offset


_SPOOF = _TorqueSpoof()


def create_eps_passthrough(packer, steering_angle_deg: float, driver_torque: int, counter: int):
  """Re-emit EPS (0x1D3) on the camera bus with all fields mirrored from PT.

  Panda blocks native EPS PT->cam while our spoof loop is active (cruise on or
  standstill). This rebuild is byte-identical to stock: bytes 3..5 = 0 and byte6 high
  nibble = 0 per the DBC, so CANPacker yields the same frame the camera would see if
  it were simply forwarded. Use this when we want zero behavior change from stock.
  """
  return packer.make_can_msg("EPS", CANBUS.cam_bus, {
    "STEERING_ANGLE": float(steering_angle_deg),
    "DRIVER_TORQUE": int(max(0, min(driver_torque, 127))),
    "COUNTER": int(counter) % 16,
  })


def create_lkas_info_torque_spoof(packer, lkas_enable, spoof_active,
                                  steer_related=0.0, apply_spoof_offset=True,
                                  inject_on_cam=False):
  """MAIN_TORQUE is spoof-only (not EPS driver torque); stock p50 ~63 when LKAS active.

  Returns a list of frames. Always emits on PT (bus 0). When `inject_on_cam` is True
  (i.e. cruise engaged, panda is blocking native LKAS_INFO PT->cam), also emits on bus 2
  so the camera ECU still receives a torque heartbeat. When False, only PT — avoids
  duplicate-frame flicker on the cluster while disengaged (panda still forwards native).
  """
  torque = _SPOOF.step(spoof_active, apply_spoof_offset)
  signals = {
    "MAIN_TORQUE": max(0.0, min(abs(torque), 1023.0)),
    "LKAS_ENABLE": int(lkas_enable),
    "STEER_RELATED": float(steer_related),
  }
  msgs = [packer.make_can_msg("LKAS_INFO", CANBUS.main_bus, signals)]
  if inject_on_cam:
    msgs.append(packer.make_can_msg("LKAS_INFO", CANBUS.cam_bus, signals))
  return msgs

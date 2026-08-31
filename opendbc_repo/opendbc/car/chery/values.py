import math
from enum import Enum

from opendbc.car import CarSpecs, PlatformConfig, Platforms, dbc_dict
from opendbc.car.byd.angle_rate_limit import AngleRateLimit
from opendbc.car.docs_definitions import CarDocs, CUSTOM_CAR_PARTS, CarFootnote, Column
from opendbc.car.lateral import AngleSteeringLimits


# --- CAN buses ---
class CANBUS:
  main_bus = 0   # PT / EPS  — LANE_KEEP + LKAS_INFO TX
  cam_bus = 2    # stock cam — HUD, LANE_KEEP RX
  tiggo22_cam_bus = 1  # Tiggo 8 Pro 2022-24: HUD on bus 1
  tiggo22_lk_bus = 2   # Tiggo 2022-24 stock 0x220 / 0x307 receive bus


# --- CarController timing ---
LANE_KEEP_STEP = 2   # 50 Hz @ DT_CTRL=0.01
LKAS_INFO_STEP = 2   # 50 Hz — match stock LKAS_INFO on bus 2
HUD_STEP = 5         # 20 Hz — match stock HUD parser rate
EPS_SPOOF_STEP = 1   # 100 Hz — match stock EPS rate

# EPS DRIVER_TORQUE pass-through + periodic "tap" injection.
# Stock distribution measured on this car (route 2026-05-26--04-52-57):
#   standstill, hands off, cruise off:  T=0 in 99% of frames
#   moving, hands off, cruise off:      T=0 in 61% of frames, 1..5 in 32%
#   moving, hands on (light grip):      T=10..15 typical, p99=15
#   moving, hands on (override):        T>=80 spikes briefly
# Any constant non-zero value while the wheel isn't moving reads as "fault" to the cam.
# Strategy: mirror real DRIVER_TORQUE exactly, and only when LKAS is asking for steering
# inject a brief light-touch tap every few seconds to keep the HOW timer reset.
EPS_TAP_TORQUE = 12            # mid of stock "light grip" band (10..15)
EPS_TAP_FRAMES = 8             # ~80 ms tap at 100 Hz
EPS_TAP_PERIOD_FRAMES = 400    # ~4 s between taps

# 10 Hz lowpass on the OP angle command (barely filters at 50 Hz LANE_KEEP).
STEER_LOWPASS_ALPHA = math.exp(-2.0 * math.pi * 10.0 * 0.02)

# Tiggo 2022-24 stop-and-go: GAS (0xFA) tap when CC.cruiseControl.resume (planner go).
# Pedal/throttle raw ~20 (stock resume taps were 12–25); byte7 = (~sum(b0..b6))&0xFF.
# Each burst frame overlays press on a fresh stock 0xFA (same counter) on bus 0 only.
GAS_ADDR = 0xFA
GAS_SPOOF_RAW = 20
GAS_SPOOF_BURST_FRAMES = 3  # three stock ticks (~30 ms)
GAS_SPOOF_RETRY_CYCLE_S = 1.0  # retry if still standstill while resume held
GAS_SPOOF_MAX_RETRIES = 5  # up to 5 retries after first attempt (6 bursts total)

# Auto-resume from standstill alternates one RES burst then one SET burst:
# stock RES bumps set-speed by +1 km/h, so SET cancels that drift.
AUTORESUME_CYCLE_S = 1.2
AUTORESUME_BURST_FRAMES = 4
OMODA_PCM_DISABLE_RES_CYCLE_S = 3.5


# --- CarState ---
GEAR_MAP = {1: "P", 2: "R", 3: "N", 4: "D"}
OMODA_GEAR_MAP = {0xB: "D", 0xC: "N", 0xD: "R", 0xE: "P"}
# iCaur 0x315 gear nibble: P=11 D=1 N=10 R=9
ICAUR_GEAR_MAP = {11: "P", 1: "D", 10: "N", 9: "R"}
# iCaur 0x245 blinker nibble
ICAUR_BLINKER_LEFT = (4, 6)
ICAUR_BLINKER_RIGHT = (8, 9)
# HUD FOLLOW_DISTANCE: raw 1 = 1-bar (closest) … raw 5 = 5-bar (farthest); 0/6/7 unknown.
FOLLOW_RAW_TO_PERSONALITY = {1: 0, 2: 0, 3: 1, 4: 2, 5: 2}
# Tiggo 8 Pro 2022-24 meter DISTANCE_BAR → LongitudinalPersonality.
# Cluster vs raw (verified): 1 bar=raw 2, 2 bar=raw 0, 3 bar=raw 1, ACC off=raw 3.
TIGGO_DISTANCE_BAR_TO_PERSONALITY = {2: 0, 0: 1, 1: 2}
STEER_RELATED_INTERVENTION_RAW_MIN = 36000
# Jaecoo only: STEER_RELATED status when raw>=36000 (decoded with STEERING_ANGLE factor/offset).
# iCaur must not use this — 0xC4 STEERING_ANGLE is real road angle there.
STEER_RELATED_INTERVENTION_DEG_MIN = 36000 * 0.06 - 1966

PT_PARSER_MSGS = [
  ("WHEELSPEED_1", 50), ("WHEELSPEED_2", 50), ("EPS", 100), ("GAS", 100),
  ("TRANSMISSION", 100), ("BRAKE_PEDAL", 50), ("STALK", 50), ("PCM_BUTTONS", 20),
  ("ADAS_RELATED", 100), ("SPEED_RELATED", 50), ("STEER_RELATED", 100),
  ("SEATBELT_287", 50), ("SEATBELT_430", 50), ("BCM_STAT_412", math.nan),
  ("BCM_STAT_465", math.nan), ("LKAS_INFO", 50),
]
CAM_PARSER_MSGS = [("HUD", 20), ("LANE_KEEP", 50), ("ACC_UNCERTAIN", 20)]
# iCaur PT: only bus-0 signals CarState reads. Do NOT require HUD/LANE_KEEP on PT —
# HUD now forwards natively (Omoda-style); LANE_KEEP is still blocked cam->PT and
# re-emitted by CarController. Cam ADAS (HUD/LANE_KEEP) stay on ICAUR_CAM for parse.
ICAUR_PT_PARSER_MSGS = [
  ("ICAUR_WHEELSPEED_A", 50), ("ICAUR_WHEELSPEED_B", 50),
  ("ICAUR_BRAKE", 50), ("ICAUR_GAS", 50),
  ("STEER_RELATED", 100),
  ("ICAUR_STALK", 50), ("ICAUR_TRANSMISSION", 100),
]
ICAUR_CAM_PARSER_MSGS = [("HUD", 20), ("LANE_KEEP", 50), ("ACC_UNCERTAIN", 20)]
# iCaur03 pedal decode:
# - BRAKE: 0x132 start bit 31 (8-bit); DBC scale 0.01 → raw 100 = 1.0
# - GAS:   0x14A start bit 23 (12-bit); DBC (0.001,-2) → (raw-2000)/1000; idle 2000→0, 3000→1
ICAUR_BRAKE_PRESSED = 0.05  # scaled 0..1
ICAUR_GAS_PRESSED = 0.05
OMODA_PT_PARSER_MSGS = [
  ("WHEELSPEED_1", 50), ("WHEELSPEED_2", 50), ("EPS", 100), ("GAS", 100),
  ("STALK", 50), ("PCM_BUTTONS", 20),
  ("ADAS_RELATED", 100), ("SPEED_RELATED", 50), ("STEER_RELATED", 100),
  ("SEATBELT_287", 50), ("SEATBELT_430", 50), ("BCM_STAT_412", math.nan),
  ("BCM_STAT_465", math.nan), ("LKAS_INFO", 50),
  ("OMODA_TRANSMISSION", 100), ("OMODA_BRAKE", 50), ("HUD", 20),
]
# Tiggo 8 Pro 2022-24: STEER_RELATED steer (not EPS), Omoda brake/trans, LKAS_INFO on PT.
# HUD is on bus 1; 0x220 TIGGO22_LANE_KEEP is on bus 2. ACC_UNCERTAIN is intentionally excluded.
TIGGO22_PT_PARSER_MSGS = [
  ("WHEELSPEED_1", 50), ("WHEELSPEED_2", 50),
  ("STEER_RELATED", 100), ("GAS", 100),
  ("OMODA_BRAKE", 50), ("OMODA_TRANSMISSION", 100),
  ("STALK", 50), ("PCM_BUTTONS", 20),
  ("SPEED_RELATED", 50),
  ("SEATBELT_287", 50), ("SEATBELT_430", 50),
  ("BCM_STAT_412", math.nan), ("LKAS_INFO", 50), ("LKA_STATUS", 20), ("TIGGO8PRO_2022_METER", 20),
]
# Tiggo 8 Pro 2022-24: do not decode HUD (Jaecoo layout does not match bus-1 0x387).
TIGGO22_CAM_PARSER_MSGS = []
TIGGO22_LK_PARSER_MSGS = [("TIGGO22_LANE_KEEP", 50), ("STEER_STATUS", 20)]
TIGGO22_LK_PARSER_BYPASS = (0x220, 0x307)  # ignore counter + checksum on stock 0x220 / 0x307

# Omoda 5: 0x2E9 byte 2 is multiplexed — raw 1..5 are message variants, not pedal.
OMODA_BRAKE_PRESSURE_RAW_MAX = 0x35
OMODA_BRAKE_PRESSURE_RAW_MIN = 5

# Omoda 5: torque spoof + HUD override temporarily off (meter errors when enabled).
OMODA_DISABLE_TORQUE_SPOOF = True
OMODA_DISABLE_HUD_OVERRIDE = True
CHERY_OMODA_SAFETY_PARAM = 1
CHERY_OMODA_NO_TORQUE_SPOOF_PARAM = 2
# Tiggo 8 Pro: torque spoof + HUD override off (meter errors when enabled).
# Native cam HUD forwards to PT (safetyParam NATIVE_HUD_FWD); keeps Jaecoo RX/wheels.
TIGGO_DISABLE_TORQUE_SPOOF = True
TIGGO_DISABLE_HUD_OVERRIDE = True
# Temporarily disable Tiggo 8 stop-and-go (no RES/SET auto-resume from standstill).
TIGGO_DISABLE_STOP_AND_GO = True
CHERY_NATIVE_HUD_FWD_PARAM = 8
CHERY_TIGGO22_SAFETY_PARAM = 16
# Tiggo 8 Pro 2022-24: 0x220 STEER_CMD is EPS torque. High from stock cam max (11-bit).
class Tiggo22SteerLimits:
  STEER_MAX = 134
  STEER_DELTA_UP = 8
  STEER_DELTA_DOWN = 5
  STEER_DRIVER_ALLOWANCE = 15
  STEER_DRIVER_MULTIPLIER = 1
  STEER_DRIVER_FACTOR = 1

# iCaur 03: standstill on 0x222; torque spoof off (same bit as Omoda).
# HUD: Omoda-style — native cam HUD forwards to PT; no HUD override TX.
CHERY_ICAUR_SAFETY_PARAM = 4
ICAUR_DISABLE_TORQUE_SPOOF = True
ICAUR_DISABLE_HUD_OVERRIDE = True
# iCaur steer override: latch (drop LKAS) on steeringPressed, firm yank, OR opposing torque.
# OPPOSING_TORQUE kept at 10 (curve-safe). PRESSED_TORQUE=7 catches light first nudges that
# peaked 5–8 on 2026-08-13--01-23-35 while |T|>=10 often needed a second yank; jul20-21 sim
# sharp_OFF|T|<10 18.2%→24.3% (+6pp) vs press=10 — acceptable for trial. Exit: |T|<=3 for
# 20 frames (~200 ms). PRESSED_MIN_COUNT=0 ⇒ first |T|>=PRESSED frame. INSTANT=20 firm yank.
ICAUR_DRIVER_OVERRIDE_ENABLED = True
ICAUR_OVERRIDE_OPPOSING_TORQUE = 10
ICAUR_STEERING_PRESSED_TORQUE = 7
ICAUR_STEERING_PRESSED_MIN_COUNT = 0
ICAUR_OVERRIDE_INSTANT_TORQUE = 20
ICAUR_LKAS_CMD_DEADBAND_DEG = 0.5
ICAUR_OVERRIDE_EXIT = 3
ICAUR_OVERRIDE_EXIT_FRAMES = 20
OMODA_CAM_PARSER_MSGS = [("STEER_STATUS", 20), ("LANE_KEEP", 50), ("ACC_UNCERTAIN", 20)]


# --- cherycan TX constants ---
LANE_KEEP_PADDING = {
  "SET_ME_XFF": 255, "SET_ME_XFC": 252, "SET_ME_XF4": 244, "SET_ME_X63": 99, "SET_ME_XF": 15,
}

# LKAS torque spoof params (tuned to stock MAIN_TORQUE distribution: p10=7, p50=63, p90=149).
SPOOF_TORQUE_MIN = 30.0
SPOOF_TORQUE_VAR_MIN = 25.0
SPOOF_TORQUE_RAMP = 8.0
SPOOF_TORQUE_MAX = 110.0
SPOOF_NEG_PROB = 0.3
SPOOF_VAR_PROB = 0.2


def lowpass_steer_cmd(x: float, y_prev: float | None) -> float:
  if y_prev is None:
    return x
  return STEER_LOWPASS_ALPHA * y_prev + (1.0 - STEER_LOWPASS_ALPHA) * x


class CarControllerParams:
  STEER_ANGLE_MAX = 120.0
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0.0, 5.0, 15.0], angle_v=[50.0, 40.0, 25.0])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0.0, 5.0, 15.0], angle_v=[60.0, 50.0, 30.0])
  ANGLE_LIMITS = AngleSteeringLimits(STEER_ANGLE_MAX, ANGLE_RATE_LIMIT_UP, ANGLE_RATE_LIMIT_DOWN)


# --- Docs / platform ---
class Footnote(Enum):
  J7_NOTE = CarFootnote(
    "Support: under validation on chery_general_pt.dbc (Jaecoo J7 PHEV).",
    Column.LONGITUDINAL,
  )


class CAR(Platforms):
  CHERY_JAECOO_J7_PHEV = PlatformConfig(
    [CarDocs(
      "Jaecoo J7 PHEV 2024-26", "ALL",
      car_parts=CUSTOM_CAR_PARTS(),
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      kommu_supported=True,
      acc_low_speed=True,
      acc_speed_range="0 - 150",
      acc_stop_and_go=True,
      lkc_torque="TBD",
      lkc_speed_range="0 - 150",
      max_steering_angle="TBD",
    )],
    CarSpecs(mass=1980.0, wheelbase=2.67, steerRatio=16.0),
    dbc_dict("chery_general_pt", None),
  )
  CHERY_TIGGO_8_PRO_2025 = PlatformConfig(
    [CarDocs(
      "Chery Tiggo 8 Pro 2025-26", "ALL",
      car_parts=CUSTOM_CAR_PARTS(),
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      kommu_supported=True,
      acc_low_speed=True,
      acc_speed_range="0 - 150",
      acc_stop_and_go=False,
      lkc_torque="TBD",
      lkc_speed_range="0 - 150",
      max_steering_angle="TBD",
    )],
    CarSpecs(mass=1980.0, wheelbase=2.67, steerRatio=16.0),
    dbc_dict("chery_general_pt", None),
  )
  CHERY_TIGGO_8_PRO_2022_2024 = PlatformConfig(
    [CarDocs(
      "Chery Tiggo 8 Pro 2022-24", "ALL",
      car_parts=CUSTOM_CAR_PARTS(),
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      kommu_supported=True,
      acc_low_speed=True,
      acc_speed_range="0 - 150",
      acc_stop_and_go=True,
      lkc_torque="TBD",
      lkc_speed_range="0 - 150",
      max_steering_angle="TBD",
    )],
    CarSpecs(mass=1980.0, wheelbase=2.67, steerRatio=16.0),
    dbc_dict("chery_general_pt", None),
  )
  CHERY_OMODA_5 = PlatformConfig(
    [CarDocs(
      "Chery Omoda 5 2022-26", "ALL",
      car_parts=CUSTOM_CAR_PARTS(),
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      kommu_supported=True,
      acc_low_speed=True,
      acc_speed_range="0 - 150",
      acc_stop_and_go=True,
      lkc_torque="TBD",
      lkc_speed_range="0 - 150",
      max_steering_angle="TBD",
    )],
    CarSpecs(mass=1420.0, wheelbase=2.63, steerRatio=16.0),
    dbc_dict("chery_general_pt", None),
  )
  CHERY_ICAUR_03 = PlatformConfig(
    [CarDocs(
      "iCaur 03 2024-26", "ALL",
      car_parts=CUSTOM_CAR_PARTS(),
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      kommu_supported=True,
      acc_low_speed=True,
      acc_speed_range="0 - 150",
      acc_stop_and_go=True,
      lkc_torque="TBD",
      lkc_speed_range="0 - 150",
      max_steering_angle="TBD",
    )],
    CarSpecs(mass=1760.0, wheelbase=2.71, steerRatio=16.0),  # steerRatio matches Jaecoo J7  # steerRatio matches Jaecoo J7
    dbc_dict("chery_general_pt", None),
  )


DBC = CAR.create_dbc_map()

import numpy as np

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.dnga.values import CANBUS

BRAKE_DECEL_CMD_MAX = 1.56
BRAKE_PUMP_MAX = 1.0


def create_can_steer_command(packer, steer, steer_req, cnt):
  values = {
    "STEER_REQ": steer_req,
    "STEERING_COUNTER": cnt,
    "STEER_CMD": -steer if steer_req else 0,
    "SET_ME_1": 1,
    "SET_ME_1_2": 1,
  }
  return packer.make_can_msg("STEERING_LKAS", CANBUS.main_bus, values)


def create_brake_command(packer, enabled, decel_req, pump, decel_cmd, aeb):
  # Keep command values in observed stock-safe ranges.
  decel_cmd = float(np.clip(float(decel_cmd), 0.0, BRAKE_DECEL_CMD_MAX))
  pump = float(np.clip(float(pump), 0.0, BRAKE_PUMP_MAX))
  braking_active = bool(enabled and decel_req)

  values = {
    "PUMP_REACTION1": pump if enabled else 0,
    "BRAKE_REQ": braking_active,
    "MAGNITUDE": -decel_cmd if braking_active else 0,
    "SET_ME_1_WHEN_ENGAGE": 1 if enabled else 0,
    "PUMP_REACTION2": -pump if enabled else 0,
    "AEB_REQ1": 1 if aeb else 0,
    "AEB_REQ2": 1 if aeb else 0,
    "AEB_REQ3": 1 if aeb else 0,
    "AEB_1019": aeb,
  }
  return packer.make_can_msg("ACC_BRAKE", CANBUS.main_bus, values)


def create_accel_command(packer, set_speed, acc_rdy, enabled, is_lead, des_speed, brake_amt, brake_pump, distance_val):
  is_braking = brake_amt > 0.0 or brake_pump > 0.0

  values = {
    "SET_SPEED": set_speed * CV.MS_TO_KPH,
    "FOLLOW_DISTANCE": distance_val,
    "IS_LEAD": is_lead,
    "IS_ACCEL": (not is_braking) and enabled,
    "IS_DECEL": is_braking and enabled,
    "SET_ME_1_2": acc_rdy,  # ready button state
    "SET_ME_1": 1,
    "SET_0_WHEN_ENGAGE": not enabled,
    "SET_1_WHEN_ENGAGE": enabled,
    "ACC_CMD": des_speed * CV.MS_TO_KPH if enabled else 0,
  }
  return packer.make_can_msg("ACC_CMD_HUD", CANBUS.main_bus, values)


def create_hud(packer, lkas_rdy, enabled, llane_visible, rlane_visible, ldw, fcw, aeb, front_depart, ldp_off, fcw_off):
  values = {
    "LKAS_SET": lkas_rdy,
    "LKAS_ENGAGED": enabled,
    "LDA_ALERT": ldw,
    "LDA_OFF": ldp_off,
    "LANE_RIGHT_DETECT": rlane_visible,
    "LANE_LEFT_DETECT": llane_visible,
    "SET_ME_X02": 0x2,
    "AEB_ALARM": fcw,
    "AEB_BRAKE": aeb,
    "FRONT_DEPART": front_depart,
    "FCW_DISABLE": fcw_off,
  }
  return packer.make_can_msg("LKAS_HUD", CANBUS.main_bus, values)


def dnga_buttons(packer, set_button, res_button, cancel_button):
  values = {
    "SET_MINUS": set_button,
    "RES_PLUS": res_button,
    "ACC_RDY": 1,
    "PEDAL_DEPRESSED": 1,
    "NEW_SIGNAL_1": 1,
    "NEW_SIGNAL_2": 1,
    "CANCEL": cancel_button,
  }
  return packer.make_can_msg("PCM_BUTTONS", CANBUS.main_bus, values)

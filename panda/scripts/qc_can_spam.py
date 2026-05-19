#!/usr/bin/env python3
import argparse
import sys
import time

import usb1

from opendbc.can.packer import CANPacker
from opendbc.car.dnga.fingerprints import FINGERPRINTS
from opendbc.car.dnga.values import CAR
from opendbc.car.structs import CarParams
from panda import Panda


PT_BUS = 0
MID_BUS = 1
CAM_BUS = 2
LOOP_DT = 0.001
PHASE_SECONDS = 3.0
USB_RETRY_S = 3.0
SPOOF_SPEED_KPH = 50.0
IGNITION_ADDR = 0x380
ONROAD_CYCLE_ON_SECONDS = 60.0
ONROAD_CYCLE_OFF_SECONDS = 10.0


def connect_panda() -> Panda:
  p = Panda()
  p.set_power_save(0)
  p.set_safety_mode(CarParams.SafetyModel.allOutput)
  p.send_heartbeat(False)
  return p


def open_panda_resilient() -> Panda:
  while True:
    try:
      return connect_panda()
    except usb1.USBErrorTimeout:
      print(f"USB timeout opening Panda; retrying in {USB_RETRY_S}s...", file=sys.stderr)
      time.sleep(USB_RETRY_S)


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Spam DNGA Ativa CAN frames for QC.")
  parser.add_argument(
    "--onroad-cycle",
    action="store_true",
    help="Cycle CAN ignition spoof: OFF 10s, ON 60s, repeat.",
  )
  return parser.parse_args()


def main() -> None:
  args = parse_args()
  _fingerprint = dict(FINGERPRINTS[CAR.PERODUA_ATIVA][0])
  packer = CANPacker("dnga_general_pt")

  p = open_panda_resilient()

  phase_enabled = True
  phase_started = time.monotonic()
  button_pulse_until = phase_started
  button_pulse_type = "res"
  ignition_enabled = True
  ignition_cycle_started = phase_started

  # Match DNGA parser expectations and keep timing stable.
  periods = {
    # PT bus expected by dnga CarState parser
    "buttons": 0.02,            # 50 Hz
    "right_stalk": 1.0 / 33.0,  # 33 Hz
    "brake": 0.01,              # 100 Hz
    "steering_module": 0.01,    # 100 Hz
    "gas_pedal": 1.0 / 60.0,    # 60 Hz
    "gas_pedal_2": 1.0 / 60.0,  # 60 Hz
    "transmission": 1.0 / 30.0, # 30 Hz
    "pcm_buttons": 1.0 / 30.0,  # 30 Hz
    "wheel_speed": 0.02,        # 50 Hz
    "eps_shaft_torque": 0.025,  # 40 Hz
    "meter_cluster": 1.0 / 15.0,# 15 Hz
    "bsm": 1.0 / 15.0,          # 15 Hz
    # CAM bus expected by dnga camera parser
    "acc_brake": 0.05,          # 20 Hz
    "steering_lkas": 0.025,     # 40 Hz
    "acc_cmd_hud": 0.05,        # 20 Hz
    "lkas_hud": 0.05,           # 20 Hz
    # Bus 1 filler traffic
    "bus1_msg_210": 0.1,        # 10 Hz
    "bus1_msg_250": 0.1,        # 10 Hz
    "bus1_msg_2a0": 0.1,        # 10 Hz
    # Ignition spoof frame for panda ignition_can hook
    "ignition_spoof": 0.1,      # 10 Hz
    "panda_heartbeat": 0.2,     # 5 Hz keeps recent_heartbeat true -> prevents bootkick
    "panda_health": 1.0,        # 1 Hz heartbeat to keep host<->panda link active
  }
  next_send = {k: phase_started for k in periods}

  counters = {
    "pcm_buttons": 0,   # 4-bit
    "steering_lkas": 0, # 4-bit
    "acc_brake": 0,     # 3-bit
    "bus1_raw": 0,      # 8-bit
  }

  # Raw values for dnga_general_pt scaling.
  ui_speed_raw = int(SPOOF_SPEED_KPH / 0.01)
  speed_ms = SPOOF_SPEED_KPH / 3.6
  wheel_f_raw = int(speed_ms / 0.00001)
  wheel_b_raw = int(speed_ms / 0.00000250)

  print("Spamming DNGA Ativa CAN set...")
  print("ACC spoof phase: ENABLED (3s)")
  print("Spoof speed: 50 km/h")
  print("Panda power save: disabled")
  print("Panda heartbeat: enabled (bootkick guard)")
  if args.onroad_cycle:
    print("CAN ignition cycle: ONROAD mode (OFF 10s, ON 60s, repeat)")
  else:
    print("CAN ignition cycle: always ON")
  print("Press Ctrl+C to stop.")

  def tick() -> None:
    nonlocal p, phase_enabled, phase_started, button_pulse_until, button_pulse_type
    nonlocal ignition_enabled, ignition_cycle_started
    now = time.monotonic()

    if (now - phase_started) >= PHASE_SECONDS:
      phase_enabled = not phase_enabled
      phase_started = now
      button_pulse_until = now + 0.25
      button_pulse_type = "res" if phase_enabled else "cancel"
      print(f"ACC spoof phase: {'ENABLED' if phase_enabled else 'DISABLED'} (3s)")

    if args.onroad_cycle:
      ignition_phase_seconds = ONROAD_CYCLE_ON_SECONDS if ignition_enabled else ONROAD_CYCLE_OFF_SECONDS
      if (now - ignition_cycle_started) >= ignition_phase_seconds:
        ignition_enabled = not ignition_enabled
        ignition_cycle_started = now
        next_duration = ONROAD_CYCLE_ON_SECONDS if ignition_enabled else ONROAD_CYCLE_OFF_SECONDS
        print(f"CAN ignition: {'ON' if ignition_enabled else 'OFF'} ({int(next_duration)}s)")

    if now >= next_send["buttons"]:
      p.can_send(*packer.make_can_msg("BUTTONS", PT_BUS, {
        "UI_SPEED": ui_speed_raw,
        "DISTANCE_BTN": 0,
        "LKC_BTN": 0,
        "FCW_BTN": 0,
        "AUTOIDLE_BTN": 0,
        "HIGHBEAM_BTN": 0,
        "PWR_BTN": 0,
      }))
      next_send["buttons"] += periods["buttons"]

    if now >= next_send["right_stalk"]:
      p.can_send(*packer.make_can_msg("RIGHT_STALK", PT_BUS, {
        "LEFT_SIGNAL": 0,
        "RIGHT_SIGNAL": 0,
        "GENERIC_TOGGLE": 0,
        "HIGH_BEAM": 0,
      }))
      next_send["right_stalk"] += periods["right_stalk"]

    if now >= next_send["brake"]:
      p.can_send(*packer.make_can_msg("BRAKE", PT_BUS, {
        "BRAKE_ENGAGED": 0,
        "BRAKE_PRESSURE": 0,
        "SPEED": ui_speed_raw,
      }))
      next_send["brake"] += periods["brake"]

    if now >= next_send["steering_module"]:
      p.can_send(*packer.make_can_msg("STEERING_MODULE", PT_BUS, {
        "MAIN_TORQUE": 0,
        "STEER_ANGLE": 0,
      }))
      next_send["steering_module"] += periods["steering_module"]

    if now >= next_send["gas_pedal"]:
      p.can_send(*packer.make_can_msg("GAS_PEDAL", PT_BUS, {
        "PULSE_WHEN_PEDAL_ZERO": 1,
        "APPS_1": 0,
        "APPS_2": 0,
        "APPS_3": 0,
      }))
      next_send["gas_pedal"] += periods["gas_pedal"]

    if now >= next_send["gas_pedal_2"]:
      p.can_send(*packer.make_can_msg("GAS_PEDAL_2", PT_BUS, {
        # CarState expects this frame at 60Hz. 1 means not pressed.
        "GAS_PEDAL_STEP": 1,
      }))
      next_send["gas_pedal_2"] += periods["gas_pedal_2"]

    if now >= next_send["transmission"]:
      p.can_send(*packer.make_can_msg("TRANSMISSION", PT_BUS, {"GEAR": 2}))
      next_send["transmission"] += periods["transmission"]

    if now >= next_send["pcm_buttons"]:
      press_res = int(now < button_pulse_until and button_pulse_type == "res")
      press_cancel = int(now < button_pulse_until and button_pulse_type == "cancel")
      p.can_send(*packer.make_can_msg("PCM_BUTTONS", PT_BUS, {
        "ACC_RDY": 1,
        "RES_PLUS": press_res,
        "SET_MINUS": 0,
        "CANCEL": press_cancel,
        "PEDAL_DEPRESSED": 1,
        "NEW_SIGNAL_1": 1,
        "NEW_SIGNAL_2": 1,
        "COUNTER": counters["pcm_buttons"],
      }))
      counters["pcm_buttons"] = (counters["pcm_buttons"] + 1) & 0xF
      next_send["pcm_buttons"] += periods["pcm_buttons"]

    if now >= next_send["wheel_speed"]:
      p.can_send(*packer.make_can_msg("WHEEL_SPEED", PT_BUS, {
        "WHEELSPEED_F": wheel_f_raw,
        "WHEELSPEED_B": wheel_b_raw,
      }))
      next_send["wheel_speed"] += periods["wheel_speed"]

    if now >= next_send["eps_shaft_torque"]:
      p.can_send(*packer.make_can_msg("EPS_SHAFT_TORQUE", PT_BUS, {"STEERING_TORQUE": 0}))
      next_send["eps_shaft_torque"] += periods["eps_shaft_torque"]

    if now >= next_send["acc_brake"]:
      p.can_send(*packer.make_can_msg("ACC_BRAKE", CAM_BUS, {
        "COUNTER": counters["acc_brake"] & 0x7,
        "SET_ME_1_WHEN_ENGAGE": int(phase_enabled),
        "MAGNITUDE": 0,
        "BRAKE_REQ": 0,
        "AEB_REQ1": 0,
        "AEB_REQ2": 0,
        "AEB_REQ3": 0,
        "PUMP_REACTION2": 0,
        "PUMP_REACTION1": 0,
        "AEB_1019": 0,
        "CRUISE_STANDSTILL": 0,
      }))
      counters["acc_brake"] = (counters["acc_brake"] + 1) & 0x7
      next_send["acc_brake"] += periods["acc_brake"]

    if now >= next_send["steering_lkas"]:
      p.can_send(*packer.make_can_msg("STEERING_LKAS", CAM_BUS, {
        "STEERING_COUNTER": counters["steering_lkas"],
        "STEER_REQ": int(phase_enabled),
        "STEER_CMD": 0,
        "SET_ME_1": 1,
        "SET_ME_1_2": 1,
        "SET_ME_0": 0,
      }))
      counters["steering_lkas"] = (counters["steering_lkas"] + 1) & 0xF
      next_send["steering_lkas"] += periods["steering_lkas"]

    if now >= next_send["acc_cmd_hud"]:
      p.can_send(*packer.make_can_msg("ACC_CMD_HUD", CAM_BUS, {
        "SET_SPEED": int(SPOOF_SPEED_KPH),
        "FOLLOW_DISTANCE": 2,
        "IS_LEAD": 1,
        "IS_ACCEL": int(phase_enabled),
        "IS_DECEL": 0,
        "SET_ME_1_2": 1,
        "SET_ME_1": 1,
        "SET_0_WHEN_ENGAGE": int(not phase_enabled),
        "SET_1_WHEN_ENGAGE": int(phase_enabled),
        "ACC_CMD": int(100 if phase_enabled else 0),
        "UNKNOWN1": 0,
        "UNKNOWN2": 0,
      }))
      next_send["acc_cmd_hud"] += periods["acc_cmd_hud"]

    if now >= next_send["lkas_hud"]:
      p.can_send(*packer.make_can_msg("LKAS_HUD", CAM_BUS, {
        "LKAS_SET": 1,
        "LKAS_ENGAGED": int(phase_enabled),
        "LANE_RIGHT_DETECT": 1,
        "LANE_LEFT_DETECT": 1,
        "SET_ME_X02": 0x2,
        "HOLD_WARNING": 0,
        "LDA_RELATED1": 0,
        "LDA_ALERT": 0,
        "LDA_OFF": 0,
        "AEB_ALARM": 0,
        "AEB_BRAKE": 0,
        "FRONT_DEPART": 0,
        "FCW_DISABLE": 0,
      }))
      next_send["lkas_hud"] += periods["lkas_hud"]

    if now >= next_send["bus1_msg_210"]:
      p.can_send(0x210, bytes((counters["bus1_raw"], 0, 0, 0, 0, 0, 0, 0)), MID_BUS)
      next_send["bus1_msg_210"] += periods["bus1_msg_210"]

    if now >= next_send["bus1_msg_250"]:
      p.can_send(0x250, bytes((0xAA, counters["bus1_raw"], 0x55, 0, 0, 0, 0, 0)), MID_BUS)
      next_send["bus1_msg_250"] += periods["bus1_msg_250"]

    if now >= next_send["bus1_msg_2a0"]:
      p.can_send(0x2A0, bytes((0x12, 0x34, counters["bus1_raw"], int(phase_enabled), 0, 0, 0, 0)), MID_BUS)
      counters["bus1_raw"] = (counters["bus1_raw"] + 1) & 0xFF
      next_send["bus1_msg_2a0"] += periods["bus1_msg_2a0"]

    if now >= next_send["panda_heartbeat"]:
      # bootkick.h disables bootkick whenever recent_heartbeat is true.
      p.send_heartbeat(False)
      next_send["panda_heartbeat"] += periods["panda_heartbeat"]

    if now >= next_send["ignition_spoof"]:
      if ignition_enabled or not args.onroad_cycle:
        # Simple, robust ignition spoof frame recognized by panda hook.
        p.can_send(IGNITION_ADDR, b"\x01\x00\x00\x00\x00\x00\x00\x00", PT_BUS)
      next_send["ignition_spoof"] += periods["ignition_spoof"]

    if now >= next_send["panda_health"]:
      # Periodic health poll acts as a host-side heartbeat.
      health = p.health()
      if health.get("power_save_enabled", 0):
        # If another process toggles power save, immediately clear it.
        p.set_power_save(0)
      next_send["panda_health"] += periods["panda_health"]

    if now >= next_send["meter_cluster"]:
      p.can_send(*packer.make_can_msg("METER_CLUSTER", PT_BUS, {
        "SEAT_BELT_WARNING": 0,
        "SEAT_BELT_WARNING2": 0,
        "RIGHT_SIGNAL": 0,
        "LEFT_SIGNAL": 0,
        "MAIN_DOOR": 0,
        "LEFT_BACK_DOOR": 0,
        "LEFT_FRONT_DOOR": 0,
        "RIGHT_BACK_DOOR": 0,
      }))
      next_send["meter_cluster"] += periods["meter_cluster"]

    if now >= next_send["bsm"]:
      p.can_send(*packer.make_can_msg("BSM", PT_BUS, {"BSM_CHIME": 0}))
      next_send["bsm"] += periods["bsm"]

    time.sleep(LOOP_DT)
  try:
    while True:
      try:
        tick()
      except usb1.USBErrorTimeout:
        print(f"USB timeout on Panda USB; retrying in {USB_RETRY_S}s...", file=sys.stderr)
        time.sleep(USB_RETRY_S)
        p = open_panda_resilient()
        t0 = time.monotonic()
        for k in periods:
          next_send[k] = t0
        phase_started = t0
        ignition_enabled = True
        ignition_cycle_started = t0
  except KeyboardInterrupt:
    print("\nStopped DNGA Ativa CAN spam.")


if __name__ == "__main__":
  main()

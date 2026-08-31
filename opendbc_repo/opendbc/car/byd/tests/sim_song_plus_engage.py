#!/usr/bin/env python3
"""Replay Song Plus rlog CAN through bumpbump parsers + safety and report engage readiness."""
import glob
import statistics as stats
import sys
from collections import Counter
from dataclasses import dataclass

import capnp

capnp.remove_import_hook()

BUKAPILOT_ROOT = '/home/kommu/20-services/bukapilot-bumpbump'
OPENDBC_ROOT = f'{BUKAPILOT_ROOT}/opendbc_repo'
sys.path.insert(0, BUKAPILOT_ROOT)
sys.path.insert(0, OPENDBC_ROOT)

from cereal import car as cereal_car
from opendbc.car import Bus
from opendbc.car.byd.interface import CarInterface
from opendbc.car.byd.mpc_lka.carstate import CarState as MpcCarState
from opendbc.car.byd.values import CAR, BydFlags, CANBUS
from opendbc.car.common.conversions import Conversions as CV
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.safety_replay.helpers import package_can_msg
from openpilot.tools.lib.logreader import LogReader

DONGLE = 'd6d20a4a15f30ed4'
ROUTE = '2026-06-09--21-16-30'
UPLOAD = f'/home/kommu/data2/upload/{DONGLE}'
WARMUP_NS = int(2e9)
FLAGS_NEW = int(BydFlags.MPC_LKA | BydFlags.ACC_ON_ESC)


@dataclass
class SegmentResult:
  seg: str
  flags: int
  synthetic_acc: bool
  samples: int
  can_valid_pct: float
  cruise_avail_pct: float
  cruise_enabled_pct: float
  v_ego_raw_ratio: float | None
  safety_tick_invalid: int
  safety_controls_allowed_pct: float


def make_cp(flags: int):
  CP = CarInterface.get_params(CAR.BYD_SONG_PLUS_DMI_21, [], [], False, False, False)
  CP.flags = flags
  return CP


def patch_acc_engaged(dat: bytes) -> bytes:
  """Force stock ACC engaged signals on ACC_HUD_ADAS (813)."""
  b = bytearray(dat)
  # AccState 19|3@1+ -> set to 3 (ACC_ACTIVE): byte 2 bits 3-5
  b[2] = (b[2] & ~0x38) | (3 << 3)
  # AccOn1 22|1@0+ -> bit 22 motorola = byte 2 bit 6
  b[2] |= 0x40
  return bytes(b)


def patch_acc_toggle(dat: bytes) -> bytes:
  b = bytearray(dat)
  # BTN_TOGGLE_ACC_OnOff 8|1@0+ -> byte 1 bit 0
  b[1] |= 0x01
  return bytes(b)


def simulate_segment(path: str, flags: int, synthetic_acc: bool = False) -> SegmentResult:
  CP = make_cp(flags)
  cs = MpcCarState(CP)
  pt = MpcCarState.get_can_parser(CP)
  cam = MpcCarState.get_cam_can_parser(CP)
  parsers = {Bus.pt: pt, Bus.cam: cam}

  msgs = list(LogReader(path))
  seg = path.split('--')[-2]
  start_t = next(m.logMonoTime for m in msgs if m.which() == 'can')

  samples = valid = cruise_avail = cruise_enabled = 0
  gps_raw_pairs = []
  ret = None

  safety = libsafety_py.libsafety
  safety.set_safety_hooks(int(cereal_car.CarParams.SafetyModel.byd), 4)
  rx_allowed = rx_total = tick_invalid = 0

  for msg in msgs:
    if msg.logMonoTime - start_t < WARMUP_NS:
      continue

    if msg.which() == 'can':
      ts = msg.logMonoTime * 1e-9
      pt_pkts = []
      cam_pkts = []
      for c in msg.can:
        if c.src >= 128:
          continue
        b = bytes(c.dat)
        if synthetic_acc:
          if c.src == 0 and c.address == 813:
            b = patch_acc_engaged(b)
          if c.src == 0 and c.address == 944:
            b = patch_acc_toggle(b)
        if c.src == CANBUS.main_bus:
          pt_pkts.append((c.address, b, ts))
        elif c.src == CANBUS.cam_bus:
          cam_pkts.append((c.address, b, ts))

      if pt_pkts:
        pt.update((ts, [(a, d, CANBUS.main_bus) for a, d, _ in pt_pkts]))
      if cam_pkts:
        cam.update((ts, [(a, d, CANBUS.cam_bus) for a, d, _ in cam_pkts]))

      safety.set_timer((msg.logMonoTime // 1000) % 0xFFFFFFFF)
      safety.safety_tick_current_safety_config()
      if not safety.safety_config_valid():
        tick_invalid += 1
      for c in msg.can:
        if c.src >= 128:
          continue
        b = bytes(c.dat)
        if synthetic_acc:
          if c.src == 0 and c.address == 813:
            b = patch_acc_engaged(b)
          if c.src == 0 and c.address == 944:
            b = patch_acc_toggle(b)
        safety.safety_fwd_hook(c.src, c.address)
        pkt = package_can_msg(type('M', (), {'address': c.address, 'src': c.src, 'dat': b})())
        safety.safety_rx_hook(pkt)
        rx_total += 1
        if safety.get_controls_allowed():
          rx_allowed += 1

      if pt_pkts or cam_pkts:
        ret = cs.update(parsers)
        samples += 1
        if pt.can_valid and cam.can_valid:
          valid += 1
        if ret.cruiseState.available:
          cruise_avail += 1
        if ret.cruiseState.enabled:
          cruise_enabled += 1

    elif msg.which() == 'gpsLocation' and ret is not None and msg.gpsLocation.speed > 2.0 and samples:
      if ret.vEgoRaw > 0.5:
        gps_raw_pairs.append((msg.gpsLocation.speed, ret.vEgoRaw))

  ratio = None
  if gps_raw_pairs:
    ratio = stats.mean([raw / gps for gps, raw in gps_raw_pairs])

  return SegmentResult(
    seg=seg,
    flags=flags,
    synthetic_acc=synthetic_acc,
    samples=samples,
    can_valid_pct=100.0 * valid / max(samples, 1),
    cruise_avail_pct=100.0 * cruise_avail / max(samples, 1),
    cruise_enabled_pct=100.0 * cruise_enabled / max(samples, 1),
    v_ego_raw_ratio=ratio,
    safety_tick_invalid=tick_invalid,
    safety_controls_allowed_pct=100.0 * rx_allowed / max(rx_total, 1),
  )


def print_result(label: str, r: SegmentResult):
  ratio = f"{r.v_ego_raw_ratio:.3f}" if r.v_ego_raw_ratio is not None else "n/a"
  print(
    f"{label} seg {r.seg}: canValid={r.can_valid_pct:.1f}% cruiseAvail={r.cruise_avail_pct:.1f}% "
    f"cruiseOn={r.cruise_enabled_pct:.1f}% vEgoRaw/GPS={ratio} "
    f"safetyTickBad={r.safety_tick_invalid} ctrlAllow={r.safety_controls_allowed_pct:.1f}%"
  )


def main():
  paths = sorted(glob.glob(f'{UPLOAD}/{DONGLE}---{ROUTE}--*-rlog.zst'))
  drive_segs = {str(i) for i in range(10, 20)}
  paths = [p for p in paths if p.split('--')[-2] in drive_segs]
  p10 = [p for p in paths if '--10---' in p][0]

  CP = make_cp(FLAGS_NEW)
  print('=== Song Plus engage simulation ===')
  print(f'Route: {ROUTE}, segments: {len(paths)}')
  print(f'carParams: flags={CP.flags} safetyParam={CP.safetyConfigs[0].safetyParam} wheelSpeedFactor={CP.wheelSpeedFactor}')

  print('\n--- A) Real CAN (this drive: stock ACC was OFF on wire) ---')
  old = simulate_segment(p10, flags=0)
  new = simulate_segment(p10, flags=FLAGS_NEW)
  print_result('OLD flags=0', old)
  print_result('NEW flags=3', new)

  print('\n--- B) Synthetic stock ACC ON (same rlog, patched 813/944) ---')
  syn = simulate_segment(p10, flags=FLAGS_NEW, synthetic_acc=True)
  print_result('NEW+ACC_ON', syn)

  print('\n--- C) NEW config all driving segments (real CAN) ---')
  results = [simulate_segment(p, FLAGS_NEW) for p in paths]
  for r in results:
    print_result('NEW', r)

  can_ok = all(r.can_valid_pct > 95 for r in results)
  speed_ok = all(r.v_ego_raw_ratio and 0.9 < r.v_ego_raw_ratio < 1.15 for r in results if r.v_ego_raw_ratio)
  safety_ok = all(r.safety_tick_invalid <= 5 for r in results)
  acc_real = any(r.cruise_avail_pct > 1 for r in results)
  acc_syn = syn.cruise_avail_pct > 90 and syn.cruise_enabled_pct > 50
  ctrl_syn = syn.safety_controls_allowed_pct > 40

  print('\n=== ENGAGE READINESS ===')
  checks = [
    ('canValid >95% (flags=3)', can_ok),
    ('vEgoRaw within 15% GPS (flags=3)', speed_ok),
    ('safety param4 tick OK', safety_ok),
    ('stock ACC seen on THIS route (real CAN)', acc_real),
    ('cruise available when ACC patched ON', acc_syn),
    ('panda controls_allowed when ACC patched ON', ctrl_syn),
  ]
  for name, ok in checks:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")

  engage_when_acc_on = can_ok and speed_ok and safety_ok and acc_syn and ctrl_syn
  print(f"\nVerdict: {'ENGAGE READY when stock ACC is ON' if engage_when_acc_on else 'BLOCKERS REMAIN'}")
  if not acc_real:
    print('Note: route 2026-06-09--21-16-30 had stock ACC OFF entire drive (AccState=0, AccOn1=0).')
    print('User must enable stock ACC before OP can engage (openpilotLongitudinalControl=False).')
  return 0 if engage_when_acc_on else 1


if __name__ == '__main__':
  raise SystemExit(main())

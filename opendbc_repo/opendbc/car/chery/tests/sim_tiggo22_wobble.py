#!/usr/bin/env python3
"""Before/after Tiggo 8 Pro 2022-24 lateral PID wobble sim on recorded qlogs.

Replays planner curvature + carState through LatControlPID (stock vs tuned #1/#5).
"""
from __future__ import annotations

import math
import statistics as stats
import sys
from dataclasses import dataclass, field
from types import SimpleNamespace

import capnp

capnp.remove_import_hook()

BUKAPILOT_ROOT = '/home/kommu/20-services/bukapilot-bumpbump'
OPENDBC_ROOT = f'{BUKAPILOT_ROOT}/opendbc_repo'
sys.path.insert(0, BUKAPILOT_ROOT)
sys.path.insert(0, OPENDBC_ROOT)

from opendbc.car import DT_CTRL
from opendbc.car.chery.interface import CarInterface
from opendbc.car.chery.values import (
  CAR,
  TIGGO22_PID_KF,
  TIGGO22_PID_KP_BP,
  TIGGO22_PID_KP_V,
  Tiggo22SteerLimits,
)
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.pid import PIDController
from openpilot.tools.lib.logreader import LogReader

DONGLE = 'cc6363fa3238335a'
ROUTE = '2026-09-01--15-35-14'
UPLOAD = f'/home/kommu/data2/upload/{DONGLE}'
SEGMENTS = {
  'highway': 110,  # ~133 kph cruise wobble
  'lowspeed': 57,  # <15 kph hunting
}

STOCK_KP_BP = [[0.], [0.22]]
STOCK_KI_BP = [[0.], [0.02]]
STOCK_KF = 0.00010
STEER_MAX = Tiggo22SteerLimits.STEER_MAX


@dataclass
class Sample:
  v_ego: float
  error: float
  torque: float
  p: float
  i: float
  f: float
  steer_req: bool
  apply_torque: int
  eps_mismatch: bool


@dataclass
class SimMetrics:
  label: str
  seg: int
  n: int = 0
  mean_abs_error: float = 0.0
  oppose_pct: float = 0.0
  mean_abs_p: float = 0.0
  mean_abs_i: float = 0.0
  mean_abs_f: float = 0.0
  mean_abs_torque: float = 0.0
  hunt_pct: float = 0.0
  error_flips_per_100: float = 0.0
  torque_delta_sigma: float = 0.0
  eps_mismatch_pct: float = 0.0
  torque_deltas: list[float] = field(default_factory=list, repr=False)


def make_cp():
  return CarInterface.get_params(CAR.CHERY_TIGGO_8_PRO_2022_2024, [], [], False, False, False)


def stock_pid_step(lat_pid, ff_factor, get_ff, car, VM, params, desired_curvature):
  """Pre-tuning LatControlPID behavior (kp=0.22, kf=0.00010, no FF attenuation or des smoothing)."""
  angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, car.vEgo, params.roll))
  angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
  error = angle_steers_des - car.steeringAngleDeg
  ff = ff_factor * get_ff(angle_steers_des_no_offset, car.vEgo)
  freeze = car.steeringPressed or car.vEgo < 5
  out = lat_pid.update(error, feedforward=ff, speed=car.vEgo, freeze_integrator=freeze)
  return out, error, float(lat_pid.p), float(lat_pid.i), float(lat_pid.f)


def tuned_pid_step(lat, car, VM, params, desired_curvature):
  out, _, pid_log = lat.update(True, car, VM, params, False, desired_curvature, False, 0.0)
  return out, float(pid_log.angleError), float(pid_log.p), float(pid_log.i), float(pid_log.f)


def qlog_path(seg: int) -> str:
  return f'{UPLOAD}/{DONGLE}---{ROUTE}--{seg}---qlog.zst'


def _cs_from_carstate(msg) -> SimpleNamespace:
  cs = msg.carState
  return SimpleNamespace(
    vEgo=float(cs.vEgo),
    steeringAngleDeg=float(cs.steeringAngleDeg),
    steeringRateDeg=float(cs.steeringRateDeg),
    steeringPressed=bool(cs.steeringPressed),
  )


def _params_from_live(msg) -> SimpleNamespace:
  lp = msg.liveParameters
  return SimpleNamespace(
    angleOffsetDeg=float(lp.angleOffsetAverageDeg),
    roll=float(lp.roll),
  )


def simulate_segment(seg: int, stock: bool) -> SimMetrics:
  path = qlog_path(seg)
  CP = make_cp()
  CI = CarInterface(CP)
  VM = VehicleModel(CP)
  get_ff = CI.get_steer_feedforward_function()
  stock_pid = PIDController(([0.], [0.22]), ([0.], [0.02]), pos_limit=1.0, neg_limit=-1.0)
  tuned_lat = None
  if not stock:
    from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
    tuned_lat = LatControlPID(CP, CI, DT_CTRL)
  label = 'stock' if stock else 'tuned'

  car = None
  params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  enabled = False
  lat_active = False
  desired_curvature = 0.0
  steer_req = False
  apply_torque_last = 0
  prev_error_sign = 0
  error_flips = 0
  samples: list[Sample] = []

  for msg in LogReader(path):
    which = msg.which()
    if which == 'carState':
      car = _cs_from_carstate(msg)
    elif which == 'liveParameters':
      params = _params_from_live(msg)
    elif which == 'selfdriveState':
      enabled = bool(msg.selfdriveState.enabled)
      lat_active = bool(msg.selfdriveState.active)
    elif which == 'controlsState':
      desired_curvature = float(msg.controlsState.desiredCurvature)

    if car is None or which != 'controlsState':
      continue

    pid_logged = msg.controlsState.lateralControlState.pidState
    if not (enabled and pid_logged.active):
      steer_req = False
      continue

    steer_req = lat_active and not car.steeringPressed
    if stock:
      out_torque, err, p, i, f = stock_pid_step(
        stock_pid, STOCK_KF, get_ff, car, VM, params, desired_curvature,
      )
    else:
      out_torque, err, p, i, f = tuned_pid_step(tuned_lat, car, VM, params, desired_curvature)

    requested = int(round(out_torque * STEER_MAX)) if steer_req else 0
    apply_torque_last = apply_driver_steer_torque_limits(
      requested, apply_torque_last, 0, Tiggo22SteerLimits,
    )
    eps_mismatch = (not steer_req) and abs(apply_torque_last) > 3

    sign = 1 if err > 0 else (-1 if err < 0 else 0)
    if sign and prev_error_sign and sign != prev_error_sign:
      error_flips += 1
    if sign:
      prev_error_sign = sign

    samples.append(Sample(
      v_ego=car.vEgo,
      error=err,
      torque=float(out_torque),
      p=p,
      i=i,
      f=f,
      steer_req=steer_req,
      apply_torque=apply_torque_last,
      eps_mismatch=eps_mismatch,
    ))

  m = SimMetrics(label=label, seg=seg)
  if not samples:
    return m

  m.n = len(samples)
  abs_err = [abs(s.error) for s in samples]
  m.mean_abs_error = stats.mean(abs_err)
  oppose = sum(1 for s in samples if s.error * s.torque < 0)
  m.oppose_pct = 100.0 * oppose / m.n
  m.mean_abs_p = stats.mean(abs(s.p) for s in samples)
  m.mean_abs_i = stats.mean(abs(s.i) for s in samples)
  m.mean_abs_f = stats.mean(abs(s.f) for s in samples)
  m.mean_abs_torque = stats.mean(abs(s.torque) for s in samples)
  hunt = sum(1 for s in samples if abs(s.error) < 0.5 and abs(s.torque) > 0.15)
  m.hunt_pct = 100.0 * hunt / m.n
  m.error_flips_per_100 = 100.0 * error_flips / max(m.n - 1, 1)
  deltas = [abs(samples[i].torque - samples[i - 1].torque) for i in range(1, m.n)]
  m.torque_delta_sigma = stats.pstdev(deltas) if len(deltas) > 1 else 0.0
  mismatch = sum(1 for s in samples if s.eps_mismatch)
  m.eps_mismatch_pct = 100.0 * mismatch / m.n
  return m


def logged_stock_metrics(seg: int) -> SimMetrics:
  """Ground-truth metrics from the recorded PID state (pre-tuning drive)."""
  path = qlog_path(seg)
  samples = []
  prev_error_sign = 0
  error_flips = 0
  enabled = False

  for msg in LogReader(path):
    if msg.which() == 'selfdriveState':
      enabled = bool(msg.selfdriveState.enabled)
    if msg.which() != 'controlsState' or not enabled:
      continue
    pid = msg.controlsState.lateralControlState.pidState
    if not pid.active:
      continue
    err = float(pid.angleError)
    tq = float(pid.output)
    sign = 1 if err > 0 else (-1 if err < 0 else 0)
    if sign and prev_error_sign and sign != prev_error_sign:
      error_flips += 1
    if sign:
      prev_error_sign = sign
    samples.append((err, tq, float(pid.p), float(pid.i), float(pid.f)))

  m = SimMetrics(label='logged', seg=seg)
  if not samples:
    return m
  m.n = len(samples)
  m.mean_abs_error = stats.mean(abs(e) for e, _, _, _, _ in samples)
  m.oppose_pct = 100.0 * sum(1 for e, t, _, _, _ in samples if e * t < 0) / m.n
  m.mean_abs_p = stats.mean(abs(p) for _, _, p, _, _ in samples)
  m.mean_abs_i = stats.mean(abs(i) for _, _, _, i, _ in samples)
  m.mean_abs_f = stats.mean(abs(f) for _, _, _, _, f in samples)
  m.mean_abs_torque = stats.mean(abs(t) for _, t, _, _, _ in samples)
  m.hunt_pct = 100.0 * sum(1 for e, t, _, _, _ in samples if abs(e) < 0.5 and abs(t) > 0.15) / m.n
  m.error_flips_per_100 = 100.0 * error_flips / max(m.n - 1, 1)
  deltas = [abs(samples[i][1] - samples[i - 1][1]) for i in range(1, m.n)]
  m.torque_delta_sigma = stats.pstdev(deltas) if len(deltas) > 1 else 0.0
  return m


def print_row(name: str, m: SimMetrics) -> None:
  print(
    f'  {name:8s} n={m.n:5d} |err|={m.mean_abs_error:.3f}° oppose={m.oppose_pct:4.1f}% '
    f'|P|={m.mean_abs_p:.3f} |I|={m.mean_abs_i:.3f} |F|={m.mean_abs_f:.3f} |T|={m.mean_abs_torque:.3f} '
    f'hunt={m.hunt_pct:4.1f}% flips/100={m.error_flips_per_100:4.1f} ΔTσ={m.torque_delta_sigma:.4f} '
    f'eps_mis={m.eps_mismatch_pct:4.2f}%'
  )


def main() -> None:
  print('Tiggo 8 Pro 2022-24 wobble sim — tuning #1 (FF), #5 (low-speed Kp/smooth)')
  print(f'Route: {DONGLE}---{ROUTE}')
  print(f'Tuned kf={TIGGO22_PID_KF} kpBP/kpV={TIGGO22_PID_KP_BP}/{TIGGO22_PID_KP_V}')
  print()

  for tag, seg in SEGMENTS.items():
    logged = logged_stock_metrics(seg)
    stock = simulate_segment(seg, stock=True)
    tuned = simulate_segment(seg, stock=False)
    print(f'=== Segment {seg} ({tag}) ===')
    print_row('logged', logged)
    print_row('stock', stock)
    print_row('tuned', tuned)
  print()
  print('On hold: #2 deadband, #3 rate limits. Rejected: #6 Kd.')


if __name__ == '__main__':
  main()

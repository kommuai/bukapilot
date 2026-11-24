#!/usr/bin/env python3
"""Compare the prior three-array calibration candidate with the flat array."""
from __future__ import annotations

import hashlib
import importlib
import json
import math
import os
import platform
import random
import resource
import statistics
import subprocess
import sys
import time
import types
from array import array
from pathlib import Path

SOURCE = Path("system/sensord/sensors/icm42670_gyro.py")
SAMPLE_COUNT = 300
PILOT_PAIRS = 7
FINAL_PAIRS = 40
WARMUP_PAIRS = 5
TOTAL_LIMIT_SECONDS = 180.0
MEMORY_LIMIT_MIB = 512.0
LOAD_LIMIT = 10.0
SEED = 20261029

FLAT_BLOCK = '''    samples = array("h")
    if samples.itemsize != 2:
      raise RuntimeError("ICM42670 gyro calibration requires two-byte signed short values")

    time.sleep(0.05)
    for _ in range(GYRO_CAL_SAMPLE_COUNT):
      samples.extend(self._read_raw_counts())
      time.sleep(GYRO_CAL_SAMPLE_DT)

    means = tuple(fmean(samples[axis::3]) for axis in range(3))
    stds_counts = tuple(pstdev(samples[axis::3]) for axis in range(3))
'''

PREVIOUS_BLOCK = '''    samples = [array("h") for _ in range(3)]
    if any(axis.itemsize != 2 for axis in samples):
      raise RuntimeError("ICM42670 gyro calibration requires two-byte signed short values")

    time.sleep(0.05)
    for _ in range(GYRO_CAL_SAMPLE_COUNT):
      for axis, count in zip(samples, self._read_raw_counts(), strict=True):
        axis.append(count)
      time.sleep(GYRO_CAL_SAMPLE_DT)

    means = tuple(fmean(axis) for axis in samples)
    stds_counts = tuple(pstdev(axis) for axis in samples)
'''


def digest(data: bytes) -> str:
  return hashlib.sha256(data).hexdigest()


def environment() -> dict:
  def read(path: str) -> str:
    try:
      return Path(path).read_text().strip()
    except OSError:
      return "unavailable"

  try:
    load = list(os.getloadavg())
  except OSError:
    load = None
  return {
    "python": sys.version.replace("\n", " "),
    "platform": platform.platform(),
    "machine": platform.machine(),
    "cpu_count": os.cpu_count(),
    "affinity": sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None,
    "load_average_1_5_15": load,
    "governor_cpu0": read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
    "frequency_cpu0_khz": read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"),
    "git_head": subprocess.run(["git", "rev-parse", "HEAD"], check=True, capture_output=True, text=True).stdout.strip(),
  }


def check_limits(started: float) -> None:
  if time.perf_counter() - started > TOTAL_LIMIT_SECONDS:
    raise RuntimeError("duration limit exceeded")
  if resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0 > MEMORY_LIMIT_MIB:
    raise RuntimeError("RSS limit exceeded")
  try:
    load = os.getloadavg()[0]
  except OSError:
    load = 0.0
  if load > LOAD_LIMIT:
    raise RuntimeError(f"load limit exceeded: {load:.2f}")


def make_previous_module(source: str) -> types.ModuleType:
  if source.count(FLAT_BLOCK) != 1:
    raise RuntimeError("could not identify exactly one current flat-array block")
  previous_source = source.replace(FLAT_BLOCK, PREVIOUS_BLOCK, 1)
  module = types.ModuleType("sensord_previous_three_array_candidate")
  module.__file__ = "in-memory previous three-array candidate"
  module.__package__ = "openpilot.system.sensord.sensors"
  module.__source__ = previous_source
  exec(compile(previous_source, module.__file__, "exec"), module.__dict__)
  return module


def calibration_sample(module) -> tuple[int, int, tuple[float, float, float]]:
  sensor = object.__new__(module.ICM42670_Gyro)
  sensor.bus = types.SimpleNamespace(close=lambda: None)
  sensor.bias_counts = (0.0, 0.0, 0.0)
  index = 0

  def read_counts():
    nonlocal index
    i = index
    index += 1
    return (10 + (i % 3), -20 + (i % 5), 30 + (i % 2))

  sensor._read_raw_counts = read_counts
  wall_start = time.perf_counter_ns()
  cpu_start = time.process_time_ns()
  sensor._calibrate_zero_rate_bias()
  cpu_ns = time.process_time_ns() - cpu_start
  wall_ns = time.perf_counter_ns() - wall_start
  if index != SAMPLE_COUNT:
    raise RuntimeError(f"calibration consumed {index} samples")
  return wall_ns, cpu_ns, sensor.bias_counts


def order(pair_index: int) -> tuple[str, str]:
  return ("previous", "flat") if pair_index % 2 == 0 else ("flat", "previous")


def percentile(values: list[float], percent: float) -> float:
  ordered = sorted(values)
  return ordered[max(0, min(len(ordered) - 1, math.ceil(percent / 100 * len(ordered)) - 1))]


def cv(values: list[float]) -> float:
  return statistics.pstdev(values) / statistics.mean(values) * 100.0


def bootstrap_ci(values: list[float], seed: int) -> list[float]:
  rng = random.Random(seed)
  n = len(values)
  medians = [statistics.median(values[rng.randrange(n)] for _ in range(n)) for _ in range(5000)]
  return [percentile(medians, 2.5), percentile(medians, 97.5)]


def summarize(previous: list[float], flat: list[float], seed: int) -> dict:
  changes = [(old - new) / old * 100.0 for old, new in zip(previous, flat, strict=True)]
  return {
    "samples_per_arm": len(previous),
    "previous": {
      "median": statistics.median(previous), "p95": percentile(previous, 95), "p99": percentile(previous, 99),
      "cv_percent": cv(previous), "raw": previous,
    },
    "flat": {
      "median": statistics.median(flat), "p95": percentile(flat, 95), "p99": percentile(flat, 99),
      "cv_percent": cv(flat), "raw": flat,
    },
    "median_time_reduction_percent": statistics.median(changes),
    "paired_median_reduction_95ci_percent": bootstrap_ci(changes, seed),
    "paired_reductions_percent": changes,
  }


def run_pairs(modules: dict, pair_count: int, started: float, discard: bool = False) -> dict:
  values = {arm: {"wall_ns": [], "cpu_ns": []} for arm in modules}
  for pair_index in range(pair_count):
    outputs = {}
    for arm in order(pair_index):
      wall_ns, cpu_ns, bias = calibration_sample(modules[arm])
      outputs[arm] = bias
      if not discard:
        values[arm]["wall_ns"].append(float(wall_ns))
        values[arm]["cpu_ns"].append(float(cpu_ns))
    if outputs["previous"] != outputs["flat"]:
      raise RuntimeError("calibration bias parity failed")
    check_limits(started)
  return values


def sample_storage_sizes() -> dict:
  samples = [(10 + i % 3, -20 + i % 5, 30 + i % 2) for i in range(SAMPLE_COUNT)]
  per_axis = [array("h") for _ in range(3)]
  flat = array("h")
  for xyz in samples:
    for axis, count in zip(per_axis, xyz, strict=True):
      axis.append(count)
    flat.extend(xyz)
  if any(axis.itemsize != 2 for axis in per_axis) or flat.itemsize != 2:
    raise RuntimeError("array('h') is not two bytes on this host")
  return {
    "previous_three_arrays_bytes": sum(sys.getsizeof(axis) for axis in per_axis),
    "flat_array_bytes": sys.getsizeof(flat),
    "payload_bytes_each": SAMPLE_COUNT * 3 * 2,
  }


def main() -> None:
  started = time.perf_counter()
  source = SOURCE.read_text()
  source_hash_before = digest(source.encode())
  previous = make_previous_module(source)
  flat = importlib.import_module("openpilot.system.sensord.sensors.icm42670_gyro")
  modules = {"previous": previous, "flat": flat}
  before = environment()
  original_sleep = time.sleep
  time.sleep = lambda _duration: None
  try:
    for _ in range(WARMUP_PAIRS):
      run_pairs(modules, 1, started, discard=True)
    pilot = run_pairs(modules, PILOT_PAIRS, started)
    pilot_cvs = [cv(pilot[arm][metric]) for arm in modules for metric in ("wall_ns", "cpu_ns")]
    final_count = 80 if max(pilot_cvs) > 20 else FINAL_PAIRS
    final = run_pairs(modules, final_count, started)
  finally:
    time.sleep = original_sleep
  after = environment()
  source_hash_after = digest(SOURCE.read_bytes())
  if source_hash_before != source_hash_after:
    raise RuntimeError("candidate source changed during comparison")
  if before["git_head"] != after["git_head"]:
    raise RuntimeError("Git HEAD changed during comparison")
  check_limits(started)
  result = {
    "method": "previous three-array candidate reconstructed in-memory from current source; same deterministic 300 XYZ samples; time.sleep mocked; interleaved arms",
    "source": str(SOURCE),
    "previous_source_sha256": digest(previous.__source__.encode()),
    "flat_source_sha256": source_hash_before,
    "warmup_pairs_discarded": WARMUP_PAIRS,
    "pilot_pairs": PILOT_PAIRS,
    "pilot_cv_percent": {arm: {metric: cv(pilot[arm][metric]) for metric in ("wall_ns", "cpu_ns")} for arm in modules},
    "final_pairs": final_count,
    "wall_time": summarize(final["previous"]["wall_ns"], final["flat"]["wall_ns"], SEED),
    "process_cpu_time": summarize(final["previous"]["cpu_ns"], final["flat"]["cpu_ns"], SEED + 1),
    "sample_storage_size_estimate": sample_storage_sizes(),
    "environment_before": before,
    "environment_after": after,
    "elapsed_seconds": time.perf_counter() - started,
    "peak_rss_mib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
    "hardware_access": "none",
    "sleep_restored": time.sleep is original_sleep,
  }
  print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
  main()

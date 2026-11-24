#!/usr/bin/env python3
"""Bounded local microbenchmark for sensord hot-path changes; never opens devices."""
from __future__ import annotations

import hashlib
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
from pathlib import Path
from types import SimpleNamespace

BASELINE = "ad334254e36a6694df5e651ef5aecb8e0e7a3085"
ROOT = Path.cwd()
PILOT_PAIRS = 7
FINAL_PAIRS = int(os.environ.get("SENSORD_BENCH_SAMPLES", "40"))
WARMUP_PAIRS = 5
EVENTS_PER_SAMPLE = 2_000
POLL_SECONDS = 0.200
TOTAL_LIMIT_SECONDS = 180.0
MEMORY_LIMIT_MIB = 512.0
LOAD_LIMIT = 10.0
SEED = 20261029
FIXED_TS = 123456789

SOURCE_PATHS = [
  "system/sensord/sensord.py",
  "system/sensord/sensors/icm42670_accel.py",
  "system/sensord/sensors/icm42670_gyro.py",
  "system/sensord/sensors/icm42670_temp.py",
  "system/sensord/sensors/lis2mdl_magn.py",
  "system/sensord/sensors/lsm6ds3_accel.py",
  "system/sensord/sensors/lsm6ds3_gyro.py",
  "system/sensord/sensors/lsm6ds3_temp.py",
  "system/sensord/sensors/mmc5603nj_magn.py",
]


def check_limits(started: float) -> None:
  if time.perf_counter() - started > TOTAL_LIMIT_SECONDS:
    raise RuntimeError("benchmark duration ceiling exceeded")
  if resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0 > MEMORY_LIMIT_MIB:
    raise RuntimeError("benchmark RSS ceiling exceeded")
  try:
    load = os.getloadavg()[0]
  except OSError:
    load = 0.0
  if load > LOAD_LIMIT:
    raise RuntimeError(f"host load ceiling exceeded: {load:.2f}")


def sha256(path: str) -> str:
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_source_module(path: str, short_name: str, source: str) -> types.ModuleType:
  # Load both arms into identically shaped isolated modules so importer metadata
  # and package loading cannot bias the per-method comparison.
  module = types.ModuleType(f"sensord_{short_name}")
  module.__file__ = f"benchmark://{short_name}"
  module.__package__ = "openpilot.system.sensord.sensors"
  exec(compile(source, module.__file__, "exec"), module.__dict__)
  return module


def load_baseline_module(path: str, short_name: str) -> types.ModuleType:
  source = subprocess.run(
    ["git", "show", f"{BASELINE}:{path}"], check=True, capture_output=True, text=True,
  ).stdout
  return load_source_module(path, short_name, source)


def load_candidate_module(path: str, short_name: str) -> types.ModuleType:
  return load_source_module(path, short_name, Path(path).read_text())

def get_modules():
  paths = {
    "sensord": "system/sensord/sensord.py",
    "icm_accel": "system/sensord/sensors/icm42670_accel.py",
    "icm_gyro": "system/sensord/sensors/icm42670_gyro.py",
    "icm_temp": "system/sensord/sensors/icm42670_temp.py",
    "lis_magn": "system/sensord/sensors/lis2mdl_magn.py",
    "lsm_accel": "system/sensord/sensors/lsm6ds3_accel.py",
    "lsm_gyro": "system/sensord/sensors/lsm6ds3_gyro.py",
    "lsm_temp": "system/sensord/sensors/lsm6ds3_temp.py",
    "mmc_magn": "system/sensord/sensors/mmc5603nj_magn.py",
  }
  baseline = {key: load_baseline_module(path, key) for key, path in paths.items()}
  candidate = {key: load_candidate_module(path, key) for key, path in paths.items()}
  return baseline, candidate


def pack_s16(value: int) -> bytes:
  value &= 0xFFFF
  return bytes((value & 0xFF, value >> 8))


def pack_mmc_axes(values: tuple[int, int, int]) -> bytes:
  data = bytearray(9)
  indexes = ((0, 1, 6), (2, 3, 7), (4, 5, 8))
  for value, (high, middle, low) in zip(values, indexes, strict=True):
    encoded = (value << 4) & 0xFFFFFF
    data[high] = (encoded >> 16) & 0xFF
    data[middle] = (encoded >> 8) & 0xFF
    data[low] = encoded & 0xFF
  return bytes(data)


class DummyBus:
  def close(self) -> None:
    pass


def make_case(module: types.ModuleType, key: str):
  from cereal import log
  from openpilot.system.sensord.sensors.i2c_sensor import Sensor

  if key == "icm42670_accel":
    sensor = object.__new__(module.ICM42670_Accel)
    sensor.source = log.SensorEventData.SensorSource.icm42670
    data = pack_s16(-2345) + pack_s16(32767) + pack_s16(1234)
    sensor.read = lambda _address, _length: data
  elif key == "lsm6ds3_accel":
    sensor = object.__new__(module.LSM6DS3_Accel)
    sensor.source = log.SensorEventData.SensorSource.lsm6ds3trc
    data = pack_s16(1234) + pack_s16(-2345) + pack_s16(32767)
    sensor.read = lambda address, _length: bytes((1,)) if address == sensor.LSM6DS3_ACCEL_I2C_REG_STAT_REG else data
  elif key == "lsm6ds3_gyro":
    sensor = object.__new__(module.LSM6DS3_Gyro)
    sensor.source = log.SensorEventData.SensorSource.lsm6ds3
    data = pack_s16(1234) + pack_s16(-2345) + pack_s16(-32768)
    sensor.read = lambda address, _length: bytes((2,)) if address == sensor.LSM6DS3_GYRO_I2C_REG_STAT_REG else data
  elif key == "icm42670_gyro":
    sensor = object.__new__(module.ICM42670_Gyro)
    sensor.source = log.SensorEventData.SensorSource.icm42670
    sensor.bias_counts = (1.5, -2.25, 3.0)
    sensor._read_raw_counts = lambda: (1234, -2345, 32767)
  elif key == "mmc5603nj_magn":
    sensor = object.__new__(module.MMC5603NJ_Magn)
    packets = {
      module.SET: pack_mmc_axes((16384, 12000, 7777)),
      module.RESET: pack_mmc_axes((0, 20000, 32768)),
    }
    sensor._cycle = module.SET
    sensor.write = lambda _address, cycle: setattr(sensor, "_cycle", cycle)
    sensor.wait = lambda: None
    sensor.read = lambda _address, _length: packets[sensor._cycle]
  elif key == "icm42670_temp":
    sensor = object.__new__(module.ICM42670_Temp)
    sensor.source = log.SensorEventData.SensorSource.icm42670
    sensor.read = lambda _address, _length: bytes((0x00, 0x80))
  elif key == "lsm6ds3_temp":
    sensor = object.__new__(module.LSM6DS3_Temp)
    sensor.source = log.SensorEventData.SensorSource.lsm6ds3trc
    sensor.read = lambda _address, _length: bytes((0x00, 0x01))
  elif key == "lis2mdl_magn":
    sensor = object.__new__(module.LIS2MDL_Magn)
    sensor.source = log.SensorEventData.SensorSource.lis2mdl
    sensor.read = lambda _address, _length: pack_s16(-200) + pack_s16(100) + pack_s16(300)
  else:
    raise ValueError(key)

  sensor.bus = DummyBus()
  return sensor


def event_dict(sensor, key):
  if key in ("icm42670_accel", "lsm6ds3_accel", "lsm6ds3_gyro", "icm42670_gyro", "lis2mdl_magn"):
    event = sensor.get_event(FIXED_TS)
  elif key == "mmc5603nj_magn":
    event = sensor.get_event(999)
  else:
    event = sensor.get_event(999)
  value = event.to_dict()
  value.pop("timestamp", None)  # local clock source differs by a few ns between arms
  return value


def make_cases(baseline, candidate):
  keys = (
    "icm42670_accel", "lsm6ds3_accel", "lsm6ds3_gyro", "icm42670_gyro",
    "mmc5603nj_magn", "icm42670_temp", "lsm6ds3_temp", "lis2mdl_magn",
  )
  module_keys = {
    "icm42670_accel": "icm_accel", "lsm6ds3_accel": "lsm_accel",
    "lsm6ds3_gyro": "lsm_gyro", "icm42670_gyro": "icm_gyro",
    "mmc5603nj_magn": "mmc_magn", "icm42670_temp": "icm_temp",
    "lsm6ds3_temp": "lsm_temp", "lis2mdl_magn": "lis_magn",
  }
  pairs = {}
  for key in keys:
    base = make_case(baseline[module_keys[key]], key)
    cand = make_case(candidate[module_keys[key]], key)
    if event_dict(base, key) != event_dict(cand, key):
      raise RuntimeError(f"baseline/candidate output parity failed for {key}")
    pairs[key] = (base, cand)
  return pairs


def timed_events(sensor, key: str, count: int) -> float:
  start = time.perf_counter_ns()
  event = None
  for _ in range(count):
    event = sensor.get_event(FIXED_TS)
  elapsed = time.perf_counter_ns() - start
  if event is None:
    raise RuntimeError(f"no event produced for {key}")
  return elapsed / count


def percentile(values: list[float], percent: float) -> float:
  ordered = sorted(values)
  if not ordered:
    return float("nan")
  index = max(0, min(len(ordered) - 1, math.ceil(percent / 100.0 * len(ordered)) - 1))
  return ordered[index]


def bootstrap_median_ci(values: list[float], seed: int) -> tuple[float, float]:
  rng = random.Random(seed)
  n = len(values)
  medians = []
  for _ in range(5_000):
    sample = [values[rng.randrange(n)] for _ in range(n)]
    medians.append(statistics.median(sample))
  return percentile(medians, 2.5), percentile(medians, 97.5)


def summarize_pair(base: list[float], cand: list[float], seed: int) -> dict:
  if len(base) != len(cand) or not base:
    raise ValueError("paired sample arrays differ or are empty")
  base_median = statistics.median(base)
  cand_median = statistics.median(cand)
  improvements = [(b - c) / b * 100.0 for b, c in zip(base, cand, strict=True)]
  ci = bootstrap_median_ci(improvements, seed)
  return {
    "samples_per_arm": len(base),
    "baseline": {
      "median": base_median, "p95": percentile(base, 95), "p99": percentile(base, 99),
      "mean": statistics.mean(base),
      "cv_percent": statistics.pstdev(base) / statistics.mean(base) * 100.0,
    },
    "candidate": {
      "median": cand_median, "p95": percentile(cand, 95), "p99": percentile(cand, 99),
      "mean": statistics.mean(cand),
      "cv_percent": statistics.pstdev(cand) / statistics.mean(cand) * 100.0,
    },
    "median_improvement_percent": (base_median - cand_median) / base_median * 100.0,
    "paired_median_improvement_95ci_percent": list(ci),
    "paired_improvements_percent": improvements,
    "baseline_samples": base,
    "candidate_samples": cand,
  }


def trial_order(pair_index: int):
  return ("baseline", "candidate") if pair_index % 2 == 0 else ("candidate", "baseline")


def run_event_workloads(pairs, count: int, samples: int, warmups: int, started: float) -> dict:
  output = {}
  for work_index, (key, (base_sensor, cand_sensor)) in enumerate(pairs.items()):
    # The fixed timestamp prevents timestamp-source noise from dominating this CPU comparison.
    for pair_index in range(warmups):
      for arm in trial_order(pair_index):
        sensor = base_sensor if arm == "baseline" else cand_sensor
        timed_events(sensor, key, count)
      check_limits(started)

    baseline_values, candidate_values = [], []
    for pair_index in range(samples):
      for arm in trial_order(pair_index):
        sensor = base_sensor if arm == "baseline" else cand_sensor
        value = timed_events(sensor, key, count)
        (baseline_values if arm == "baseline" else candidate_values).append(value)
      check_limits(started)
    output[key] = {
      "unit": "ns/event",
      "events_per_sample": count,
      **summarize_pair(baseline_values, candidate_values, SEED + work_index),
    }
  return output


def polling_sample(module, duration: float) -> dict[str, float]:
  from cereal import messaging
  from openpilot.common.realtime import Ratekeeper

  class InvalidSensor:
    def __init__(self):
      self.calls = 0

    def get_event(self):
      self.calls += 1
      return None

    def is_data_valid(self):
      return False

  class DurationEvent:
    def __init__(self, seconds):
      self.deadline = time.monotonic() + seconds

    def is_set(self):
      return time.monotonic() >= self.deadline

  class FakePublisher:
    def __init__(self, _services):
      pass

    def send(self, *_args, **_kwargs):
      raise RuntimeError("invalid-data benchmark unexpectedly published")

  old_pub = messaging.PubMaster
  old_new_message = messaging.new_message
  old_exception = module.cloudlog.exception
  messaging.PubMaster = FakePublisher
  messaging.new_message = lambda *_args, **_kwargs: None
  module.cloudlog.exception = lambda *_args, **_kwargs: None
  module.Ratekeeper = Ratekeeper
  sensor = InvalidSensor()
  stop = DurationEvent(duration)
  wall_start = time.perf_counter_ns()
  cpu_start = time.process_time_ns()
  try:
    module.polling_loop(sensor, "accelerometer", stop)
  finally:
    cpu_ns = time.process_time_ns() - cpu_start
    wall_ns = time.perf_counter_ns() - wall_start
    module.cloudlog.exception = old_exception
    messaging.PubMaster = old_pub
    messaging.new_message = old_new_message
  return {
    "wall_ms": wall_ns / 1e6,
    "cpu_ms": cpu_ns / 1e6,
    "cpu_fraction": cpu_ns / wall_ns if wall_ns else 0.0,
    "iterations": float(sensor.calls),
    "iterations_per_second": sensor.calls / (wall_ns / 1e9),
  }


def run_polling_workload(baseline, candidate, samples: int, warmups: int, started: float) -> dict:
  output = {"baseline": [], "candidate": []}
  for pair_index in range(warmups):
    for arm in trial_order(pair_index):
      module = baseline["sensord"] if arm == "baseline" else candidate["sensord"]
      polling_sample(module, POLL_SECONDS)
    check_limits(started)
  for pair_index in range(samples):
    for arm in trial_order(pair_index):
      module = baseline["sensord"] if arm == "baseline" else candidate["sensord"]
      output[arm].append(polling_sample(module, POLL_SECONDS))
    check_limits(started)
  metrics = {}
  for metric in ("wall_ms", "cpu_ms", "cpu_fraction", "iterations", "iterations_per_second"):
    base = [sample[metric] for sample in output["baseline"]]
    cand = [sample[metric] for sample in output["candidate"]]
    metrics[metric] = summarize_pair(base, cand, SEED + 100 + len(metric))
  return {"window_ms": POLL_SECONDS * 1000.0, "ratekeeper_hz": 104.0, "metrics": metrics,
          "baseline_raw": output["baseline"], "candidate_raw": output["candidate"]}


def calibration_sample(module) -> float:
  cls = module.ICM42670_Gyro
  sensor = object.__new__(cls)
  sensor.bus = DummyBus()
  sensor.bias_counts = (0.0, 0.0, 0.0)
  index = 0

  def read_counts():
    nonlocal index
    i = index
    index += 1
    return (10 + (i % 3), -20 + (i % 5), 30 + (i % 2))

  sensor._read_raw_counts = read_counts
  start = time.perf_counter_ns()
  sensor._calibrate_zero_rate_bias()
  elapsed = time.perf_counter_ns() - start
  if index != 300:
    raise RuntimeError(f"calibration consumed {index} samples instead of 300")
  return float(elapsed)


def run_calibration_workload(baseline, candidate, samples: int, warmups: int, started: float) -> dict:
  import time as time_module

  original_sleep = time_module.sleep
  time_module.sleep = lambda _duration: None
  try:
    for pair_index in range(warmups):
      for arm in trial_order(pair_index):
        module = baseline["icm_gyro"] if arm == "baseline" else candidate["icm_gyro"]
        calibration_sample(module)
      check_limits(started)
    base_values, cand_values = [], []
    for pair_index in range(samples):
      for arm in trial_order(pair_index):
        module = baseline["icm_gyro"] if arm == "baseline" else candidate["icm_gyro"]
        value = calibration_sample(module)
        (base_values if arm == "baseline" else cand_values).append(value)
      check_limits(started)
  finally:
    time_module.sleep = original_sleep
  return {
    "unit": "ns/calibration (300 samples; sleep excluded)",
    **summarize_pair(base_values, cand_values, SEED + 500),
  }


def environment() -> dict:
  affinity = sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None
  def read(path):
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
    "processor": platform.processor(),
    "cpu_count": os.cpu_count(),
    "affinity": affinity,
    "load_average_1_5_15": load,
    "governor_cpu0": read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
    "frequency_cpu0_khz": read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"),
    "current_git_head": subprocess.run(["git", "rev-parse", "HEAD"], check=True, capture_output=True, text=True).stdout.strip(),
  }


def main() -> None:
  mode = sys.argv[1] if len(sys.argv) > 1 else "pilot"
  if mode not in ("parity", "pilot", "final"):
    raise SystemExit("usage: benchmark_sensord_hotpaths.py [parity|pilot|final]")
  started = time.perf_counter()
  baseline, candidate = get_modules()
  pairs = make_cases(baseline, candidate)
  try:
    import time as time_module
    old_monotonic = time_module.monotonic
    old_monotonic_ns = time_module.monotonic_ns
    time_module.monotonic = lambda: 123.456789
    time_module.monotonic_ns = lambda: 123456789
    # Recheck each fixture with fixed clocks before collecting timing data.
    for key, (base_sensor, cand_sensor) in pairs.items():
      if event_dict(base_sensor, key) != event_dict(cand_sensor, key):
        raise RuntimeError(f"fixed-clock parity check failed for {key}")
    time_module.monotonic = old_monotonic
    time_module.monotonic_ns = old_monotonic_ns
  except Exception:
    try:
      time_module.monotonic = old_monotonic
      time_module.monotonic_ns = old_monotonic_ns
    except Exception:
      pass
    raise

  if mode == "parity":
    print(json.dumps({
      "mode": "parity-only",
      "baseline_commit": BASELINE,
      "candidate_source_sha256": {path: sha256(path) for path in SOURCE_PATHS},
      "environment": environment(),
      "parity": {key: "pass" for key in pairs},
      "hardware_access": "none",
    }, sort_keys=True))
    return

  samples = PILOT_PAIRS if mode == "pilot" else FINAL_PAIRS
  warmups = WARMUP_PAIRS
  result = {
    "mode": mode,
    "baseline_commit": BASELINE,
    "candidate_source_sha256": {path: sha256(path) for path in SOURCE_PATHS},
    "environment_before": environment(),
    "sampling": {
      "warmup_pairs_discarded": warmups,
      "pilot_pairs": samples if mode == "pilot" else 0,
      "final_pairs": samples if mode == "final" else 0,
      "events_per_sample": EVENTS_PER_SAMPLE,
      "poll_window_seconds": POLL_SECONDS,
      "order": "interleaved, alternating baseline-first/candidate-first",
      "parity": "serialized fields matched for every fixed fixture; dynamic timestamp field excluded",
    },
    "event_workloads": run_event_workloads(pairs, EVENTS_PER_SAMPLE, samples, warmups, started),
    "poll_invalid": run_polling_workload(baseline, candidate, samples, warmups, started),
    "gyro_calibration": run_calibration_workload(baseline, candidate, samples, warmups, started),
  }
  result["environment_after"] = environment()
  result["elapsed_seconds"] = time.perf_counter() - started
  result["peak_rss_mib"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
  result["restore_register"] = []
  if result["environment_before"]["current_git_head"] != result["environment_after"]["current_git_head"]:
    raise RuntimeError("baseline revision changed during benchmark")
  if result["candidate_source_sha256"] != {path: sha256(path) for path in SOURCE_PATHS}:
    raise RuntimeError("candidate source changed during benchmark")
  check_limits(started)
  print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
  main()

from types import SimpleNamespace
from statistics import fmean, pstdev

from openpilot.system.sensord.sensors import icm42670_gyro
from openpilot.system.sensord.sensors.icm42670_gyro import GYRO_CAL_MAX_MEAN_RAD_S, GYRO_CAL_MAX_STD_RAD_S, ICM42670_Gyro


def run_calibration(monkeypatch, samples):
  sensor = object.__new__(ICM42670_Gyro)
  sensor.bus = SimpleNamespace(close=lambda: None)
  sensor.bias_counts = (0.0, 0.0, 0.0)
  sample_iter = iter(samples)
  sensor._read_raw_counts = lambda: next(sample_iter)
  monkeypatch.setattr(icm42670_gyro.time, "sleep", lambda _duration: None)
  seen_fmean = []
  seen_pstdev = []
  original_fmean = fmean
  original_pstdev = pstdev

  def record_fmean(values):
    observed = tuple(values)
    seen_fmean.append(observed)
    return original_fmean(observed)

  def record_pstdev(values):
    observed = tuple(values)
    seen_pstdev.append(observed)
    return original_pstdev(observed)

  monkeypatch.setattr(icm42670_gyro, "fmean", record_fmean)
  monkeypatch.setattr(icm42670_gyro, "pstdev", record_pstdev)
  sensor._calibrate_zero_rate_bias()
  return sensor, seen_fmean, seen_pstdev


def expected_calibration(samples):
  axes = tuple(tuple(sample[axis] for sample in samples) for axis in range(3))
  means = tuple(fmean(axis) for axis in axes)
  stds = tuple(pstdev(axis) for axis in axes)
  counts_to_rad = lambda value: value * (3.141592653589793 / 180.0) / 131.0
  mean_rad = tuple(counts_to_rad(value) for value in means)
  std_rad = tuple(counts_to_rad(value) for value in stds)
  stationary = sum(value * value for value in mean_rad) ** 0.5 <= GYRO_CAL_MAX_MEAN_RAD_S and max(std_rad) <= GYRO_CAL_MAX_STD_RAD_S
  bias = means if stationary else (0.0, 0.0, 0.0)
  return axes, means, stds, stationary, bias


def test_calibration_preserves_ordered_signed_16_bit_samples_and_statistics(monkeypatch):
  samples = []
  for index in range(300):
    if index == 0:
      sample = (-32768, 32767, 0)
    elif index == 1:
      sample = (32767, -32768, -32768)
    elif index == 2:
      sample = (index - 150, 150 - index, 32767)
    else:
      sample = (index - 150, 150 - index, (index % 21) - 10)
    samples.append(sample)

  sensor, means_inputs, std_inputs = run_calibration(monkeypatch, samples)
  axes, means, stds, stationary, bias = expected_calibration(samples)

  assert len(means_inputs) == len(std_inputs) == 3
  assert tuple(means_inputs) == axes
  assert tuple(std_inputs) == axes
  assert tuple(fmean(axis) for axis in means_inputs) == means
  assert tuple(pstdev(axis) for axis in std_inputs) == stds
  assert not stationary
  assert sensor.bias_counts == bias


def test_stationary_samples_select_the_same_mean_bias(monkeypatch):
  samples = [(10 + index % 2, -20 + index % 3 - 1, 30 - index % 2) for index in range(300)]

  sensor, means_inputs, std_inputs = run_calibration(monkeypatch, samples)
  axes, means, stds, stationary, bias = expected_calibration(samples)

  assert tuple(means_inputs) == axes
  assert tuple(std_inputs) == axes
  assert tuple(fmean(axis) for axis in axes) == means
  assert tuple(pstdev(axis) for axis in axes) == stds
  assert stationary
  assert sensor.bias_counts == bias == means

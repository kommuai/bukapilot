import json
import os
from pathlib import Path
import shutil
import subprocess
import uuid

import pytest

from openpilot.system.hardware import KA2
from openpilot.system.loggerd.tests.test_dmabuf_fixture import _assert_camera_stack_idle


RUN_ID = "loggerd-ka2-efficiency-20261029"
TARGET = "build/system/loggerd/tests/test_loggerd_rotation"
NON_RK3588_TARGET = "build/system/loggerd/tests/test_loggerd_rotation_non_rk3588"


def _build_rotation_helper(repo_root: Path) -> Path:
  scons = shutil.which("scons")
  assert scons is not None, "SCons is required to build the loggerd rotation contract helper"
  command = [scons, "-Q", "--ccflags=-DLOGGERD_TEST_BUILD", TARGET]
  result = subprocess.run(command, cwd=repo_root, text=True, capture_output=True, check=False)
  print("rotation helper build:", " ".join(command))
  print(result.stdout, end="")
  print(result.stderr, end="")
  assert result.returncode == 0, f"rotation helper build exited {result.returncode}"
  binary = repo_root / TARGET
  assert binary.is_file(), f"missing test binary: {binary}"
  return binary


def _run_loggerd_rotation_clock_report_and_audio_contracts(tmp_path, monkeypatch):
  if not KA2:
    pytest.skip("deterministic rotation contracts require the RK3588 target")
  repo_root = Path(__file__).resolve().parents[3]
  binary = _build_rotation_helper(repo_root)

  log_root = (tmp_path / "logs").resolve()
  params_root = (tmp_path / "params").resolve()
  log_root.mkdir(parents=True)
  params_root.mkdir(parents=True)
  prefix = f"loggerd_rotation_{os.getpid()}_{uuid.uuid4().hex[:8]}"
  env = os.environ.copy()
  env.update({
    "LOG_ROOT": str(log_root),
    "PARAMS_ROOT": str(params_root),
    "OPENPILOT_PREFIX": prefix,
    "LOGGERD_TEST": "1",
    "LOGGERD_TEST_DETERMINISTIC": "1",
    "LOGGERD_SEGMENT_LENGTH": "5",
    "ENCODERD_DETERMINISTIC_DISABLE": "0",
  })
  for key in ("LOG_ROOT", "PARAMS_ROOT", "OPENPILOT_PREFIX", "LOGGERD_TEST",
              "LOGGERD_TEST_DETERMINISTIC", "LOGGERD_SEGMENT_LENGTH", "ENCODERD_DETERMINISTIC_DISABLE"):
    monkeypatch.setenv(key, env[key])

  msgq_dir = Path("/dev/shm") / f"msgq_{prefix}"
  assert not msgq_dir.exists(), f"isolated MSGQ directory already exists: {msgq_dir}"
  msgq_dir.mkdir(mode=0o700)
  try:
    _assert_camera_stack_idle()
    result = subprocess.run([str(binary)], cwd=repo_root, env=env, text=True,
                            capture_output=True, check=False, timeout=30)
    print(result.stdout, end="")
    print(result.stderr, end="")
    assert result.returncode == 0, f"loggerd rotation helper exited {result.returncode}: {result.stdout}{result.stderr}"
  finally:
    shutil.rmtree(msgq_dir, ignore_errors=True)

  report_lines = {
    "REPORT": [line.partition(" ")[2] for line in result.stdout.splitlines() if line.startswith("REPORT ")],
    "MISSING_REPORT": [line.partition(" ")[2] for line in result.stdout.splitlines()
                       if line.startswith("MISSING_REPORT ")],
  }
  assert len(report_lines["REPORT"]) == 1 and len(report_lines["MISSING_REPORT"]) == 1, (
    f"expected one audio-tie and one missing-stream report, got {report_lines!r}"
  )
  parsed_reports = [json.loads(report) for reports in report_lines.values() for report in reports]
  expected_keys = {
    "route", "epoch", "segment", "degraded", "missing_streams", "elapsed_ms", "known_drop_counts",
  }
  expected_drop_keys = {
    "stale_video_messages", "future_video_queue_drops", "audio_no_writer_events",
    "audio_expired_events", "audio_capacity_drops",
  }
  configured_order = ["road", "qRoad"]
  for report in parsed_reports:
    assert set(report) == expected_keys
    assert type(report["route"]) is str
    assert type(report["epoch"]) is int and report["epoch"] >= 0
    assert type(report["segment"]) is int and report["segment"] >= 0
    assert report["epoch"] == report["segment"] + 1
    assert type(report["degraded"]) is bool
    assert type(report["elapsed_ms"]) is int and report["elapsed_ms"] >= 0
    missing = report["missing_streams"]
    assert type(missing) is list and all(type(name) is str for name in missing)
    assert len(missing) == len(set(missing))
    assert missing == [name for name in configured_order if name in missing]
    drops = report["known_drop_counts"]
    assert set(drops) == expected_drop_keys
    assert all(type(value) is int and value >= 0 for value in drops.values())
    assert report["degraded"] is bool(missing or any(drops.values()))

  missing_report = json.loads(report_lines["MISSING_REPORT"][0])
  assert missing_report["missing_streams"] == ["qRoad"]
  assert missing_report["degraded"] is True
  tie_report = json.loads(report_lines["REPORT"][0])
  assert tie_report["missing_streams"] == []
  assert tie_report["known_drop_counts"]["audio_expired_events"] == 1
  assert tie_report["known_drop_counts"]["audio_no_writer_events"] == 0
  assert tie_report["degraded"] is True


@pytest.mark.xdist_group("loggerd_tests")
def test_non_rk3588_loggerd_legacy_guard_compile_only(tmp_path, monkeypatch):
  if not KA2:
    pytest.skip("compile-only non-RK3588 guard probe is scoped to the pinned KA2 build host")

  repo_root = Path(__file__).resolve().parents[3]
  scons = shutil.which("scons")
  assert scons is not None, "SCons is required for the compile-only non-RK3588 guard probe"
  build_env = os.environ.copy()
  build_env.pop("OPENPILOT_DEVICE", None)
  command = [scons, "-Q", "--ccflags=-DLOGGERD_TEST_BUILD", NON_RK3588_TARGET]
  print("isolated non-RK3588 loggerd guard target:", " ".join(command))
  print("no device identity override; the dedicated test target undefines RK3588 and keeps __TICI__ unset")
  built = subprocess.run(command, cwd=repo_root, env=build_env, text=True, capture_output=True, check=False)
  print(built.stdout, end="")
  print(built.stderr, end="")
  assert built.returncode == 0, f"non-RK3588 loggerd guard build exited {built.returncode}"

  log_root = (tmp_path / "logs").resolve()
  params_root = (tmp_path / "params").resolve()
  log_root.mkdir(parents=True)
  params_root.mkdir(parents=True)
  prefix = f"loggerd_legacy_guard_{os.getpid()}_{uuid.uuid4().hex[:8]}"
  env = os.environ.copy()
  env.pop("OPENPILOT_DEVICE", None)
  env.update({
    "LOG_ROOT": str(log_root),
    "PARAMS_ROOT": str(params_root),
    "OPENPILOT_PREFIX": prefix,
    "LOGGERD_TEST": "1",
    "LOGGERD_TEST_DETERMINISTIC": "1",
    "LOGGERD_SEGMENT_LENGTH": "2",
    "ENCODERD_DETERMINISTIC_DISABLE": "0",
  })
  for key in ("LOG_ROOT", "PARAMS_ROOT", "OPENPILOT_PREFIX", "LOGGERD_TEST",
              "LOGGERD_TEST_DETERMINISTIC", "LOGGERD_SEGMENT_LENGTH", "ENCODERD_DETERMINISTIC_DISABLE"):
    monkeypatch.setenv(key, env[key])
  binary = repo_root / NON_RK3588_TARGET
  result = subprocess.run([str(binary), "--non-rk3588-legacy-guard"], cwd=repo_root, env=env,
                          text=True, capture_output=True, check=False, timeout=15)
  print(result.stdout, end="")
  print(result.stderr, end="")
  assert result.returncode == 0, f"non-RK3588 loggerd guard helper exited {result.returncode}"
  assert "NON_RK3588_LOGGERD_GUARD deterministic_test_mode=1 protocol_active=0" in result.stdout


@pytest.mark.xdist_group("loggerd_tests")
def test_loggerd_rotation_clock_report_and_audio_contracts(tmp_path, monkeypatch):
  try:
    _run_loggerd_rotation_clock_report_and_audio_contracts(tmp_path, monkeypatch)
  finally:
    shutil.rmtree(tmp_path, ignore_errors=True)


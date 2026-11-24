import os
from pathlib import Path
import shutil
import subprocess
import uuid

import pytest

from openpilot.system.hardware import KA2
from openpilot.system.loggerd.tests.test_dmabuf_fixture import _assert_camera_stack_idle


TARGET = "build/system/loggerd/tests/test_encoderd_recovery"


def _build_recovery_helper(repo_root: Path) -> Path:
  scons = shutil.which("scons")
  assert scons is not None, "SCons is required to build the encoderd recovery contract helper"
  command = [scons, "-Q", "--ccflags=-DLOGGERD_TEST_BUILD", TARGET]
  result = subprocess.run(command, cwd=repo_root, text=True, capture_output=True, check=False)
  print("recovery helper build:", " ".join(command))
  print(result.stdout, end="")
  print(result.stderr, end="")
  assert result.returncode == 0, f"recovery helper build exited {result.returncode}"
  binary = repo_root / TARGET
  assert binary.is_file(), f"missing test binary: {binary}"
  return binary


def _isolated_env(tmp_path: Path, monkeypatch) -> tuple[dict[str, str], Path]:
  root = tmp_path.resolve()
  log_root = root / "logs"
  params_root = root / "params"
  log_root.mkdir(parents=True, exist_ok=True)
  params_root.mkdir(parents=True, exist_ok=True)
  env = os.environ.copy()
  prefix = f"encoderd_recovery_{os.getpid()}_{uuid.uuid4().hex[:8]}"
  env.update({
    "LOG_ROOT": str(log_root),
    "PARAMS_ROOT": str(params_root),
    "OPENPILOT_PREFIX": prefix,
    "LOGGERD_TEST": "1",
    "LOGGERD_TEST_DETERMINISTIC": "1",
    "LOGGERD_SEGMENT_LENGTH": "5",
    "ENCODERD_DETERMINISTIC_DISABLE": "0",
    "LOGGERD_TEST_MPP_FAULT_STREAM": "roadEncodeData",
  })
  for key in ("LOG_ROOT", "PARAMS_ROOT", "OPENPILOT_PREFIX", "LOGGERD_TEST",
              "LOGGERD_TEST_DETERMINISTIC", "LOGGERD_SEGMENT_LENGTH",
              "ENCODERD_DETERMINISTIC_DISABLE"):
    monkeypatch.setenv(key, env[key])
  msgq_dir = Path("/dev/shm") / f"msgq_{prefix}"
  assert not msgq_dir.exists(), f"test prefix already exists: {msgq_dir}"
  msgq_dir.mkdir(mode=0o700)
  return env, msgq_dir


def _run_encoderd_authority_parser_and_mpp_recovery_seams(tmp_path, monkeypatch):
  if not KA2:
    pytest.skip("recovery integration requires RK3588 MPP")

  repo_root = Path(__file__).resolve().parents[3]
  binary = _build_recovery_helper(repo_root)
  # Policy-clock probes construct no MPP context and need no device preflight.
  for mode in ("parse", "counter_contract", "policy_state"):
    parse_env, parse_msgq = _isolated_env(tmp_path / mode, monkeypatch)
    try:
      parsed = subprocess.run([str(binary), mode], cwd=repo_root, env=parse_env,
                              text=True, capture_output=True, check=False, timeout=10)
      print(parsed.stdout, end="")
      print(parsed.stderr, end="")
      assert parsed.returncode == 0, parsed.stdout + parsed.stderr
    finally:
      shutil.rmtree(parse_msgq, ignore_errors=True)

  for mode in ("watchdog_retry", "drain_quarantine", "invalid_fd", "shm", "shm_rga", "reject", "silent", "drain_timeout"):
    _assert_camera_stack_idle()
    env, msgq_dir = _isolated_env(tmp_path / mode, monkeypatch)
    if mode == "watchdog_retry":
      env["LOGGERD_TEST_MPP_FAULT_STREAM"] = "qRoadEncodeData"
    elif mode == "drain_quarantine":
      env["LOGGERD_TEST_MPP_FAULT_STREAM"] = "roadEncodeData"
    if mode == "shm_rga":
      env.pop("LOGGERD_TEST_MPP_FAULT_STREAM", None)
      env.pop("LOGGERD_TEST_MPP_FAULT_MODE", None)
      monkeypatch.delenv("LOGGERD_TEST_MPP_FAULT_STREAM", raising=False)
      monkeypatch.delenv("LOGGERD_TEST_MPP_FAULT_MODE", raising=False)
    else:
      env["LOGGERD_TEST_MPP_FAULT_MODE"] = {
        "watchdog_retry": "silent",
        "drain_quarantine": "drain_timeout",
      }.get(mode, mode)
      for key, value in env.items():
        if key.startswith("LOGGERD_TEST_MPP_"):
          monkeypatch.setenv(key, value)
    try:
      result = subprocess.run([str(binary), mode], cwd=repo_root, env=env,
                              text=True, capture_output=True, check=False, timeout=15)
      print(result.stdout, end="")
      print(result.stderr, end="")
      assert result.returncode == 0, f"MPP mode {mode} exited {result.returncode}: {result.stdout}{result.stderr}"
    finally:
      shutil.rmtree(msgq_dir, ignore_errors=True)


@pytest.mark.xdist_group("loggerd_tests")
def test_encoderd_authority_parser_and_mpp_recovery_seams(tmp_path, monkeypatch):
  try:
    _run_encoderd_authority_parser_and_mpp_recovery_seams(tmp_path, monkeypatch)
  finally:
    shutil.rmtree(tmp_path, ignore_errors=True)


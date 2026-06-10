#!/usr/bin/env python3
"""KA2 ignition on/off transition QC test.

Runs ONOFF_CYCLES ignition on/off cycles and checks:
  - Onroad/offroad transition timing
  - All required processes start/stop correctly
  - No zombie processes after ignition off
  - Memory does not grow significantly across cycles
  - Status LED (indicatord) alive in both states
  - soundd alive during onroad

Random hold durations are used to stress-test state transitions
under varied timing conditions — catches race conditions and
bugs that only appear after sustained running.

Run:
  KA2_CAN_REPLAY=1 python3 selfdrive/test/test_onoff.py --can-replay
"""

import inspect
import os
import random
import signal
import subprocess
import sys
import textwrap
import threading
import time
import traceback
from contextlib import nullcontext
from pathlib import Path

import pytest

import cereal.messaging as messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.timeout import Timeout
from openpilot.selfdrive.test.helpers import set_params_enabled, unset_params_enabled
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.ka2.hardware import Ka2
from openpilot.system.hardware.hw import Paths

# ============================================================
# CONSTANTS
# ============================================================

QC_REPORT_WIDTH = 72

# Number of ignition on/off cycles to run
ONOFF_CYCLES = 5

# Max seconds to wait for onroad transition (ignition on → carState seen)
ONROAD_TRANSITION_TIMEOUT_S = 120

# Max seconds to wait for offroad transition (ignition off → IsOffroad confirmed)
OFFROAD_TRANSITION_TIMEOUT_S = 60

# Max seconds to wait for onroad processes to stop after ignition off
PROCESS_STOP_TIMEOUT_S = 30

# Random hold duration while onroad before toggling ignition off (seconds)
ONROAD_HOLD_MIN_S = 30
ONROAD_HOLD_MAX_S = 60

# Random hold duration while offroad before next cycle (seconds)
OFFROAD_HOLD_MIN_S = 10
OFFROAD_HOLD_MAX_S = 30

# Memory growth tolerance across all cycles (percentage points)
MEMORY_GROWTH_MAX_PCT = 10.0

# Onroad processes that must be running during onroad state
ONROAD_PROCS = (
  "selfdrive.selfdrived.selfdrived",
  "selfdrive.controls.controlsd",
  "selfdrive.car.card",
  "./loggerd",
  "./camerad",
  "./encoderd",
  "selfdrive.modeld.modeld",
)

# Processes that must NOT be running (zombie) after offroad transition
NO_ZOMBIE_PROCS = (
  "selfdrive.controls.controlsd",
  "selfdrive.modeld.modeld",
  "./camerad",
  "./encoderd",
)

SD_MEDIA_MOUNT = "/data/media"


# ============================================================
# PANDA HEARTBEAT GUARD
# ============================================================

_panda_hb_guard_stop = threading.Event()
_panda_hb_guard_thread: threading.Thread | None = None


def _start_ka2_panda_heartbeat_guard_after_manager_exit() -> None:
  global _panda_hb_guard_thread
  if _panda_hb_guard_thread is not None and _panda_hb_guard_thread.is_alive():
    return
  _panda_hb_guard_stop.clear()

  def run() -> None:
    time.sleep(0.5)
    try:
      from panda import Panda
      p = Panda()
    except Exception:
      return
    while not _panda_hb_guard_stop.wait(0.25):
      try:
        p.send_heartbeat(False)
      except Exception:
        break

  _panda_hb_guard_thread = threading.Thread(
    target=run, name="ka2_onoff_panda_heartbeat_guard", daemon=True,
  )
  _panda_hb_guard_thread.start()


# ============================================================
# QC REPORT HELPERS
# ============================================================

def _safe_print(*args, **kwargs):
  try:
    print(*args, **kwargs)
  except (BlockingIOError, OSError):
    pass


def _qc_rule(ch: str = "=", width: int = QC_REPORT_WIDTH) -> str:
  return ch * width


def _qc_centered_title(title: str, width: int = QC_REPORT_WIDTH) -> str:
  title = f" {title.strip()} "
  if len(title) >= width:
    return title[:width]
  pad = width - len(title)
  return "=" * (pad // 2) + title + "=" * (pad - pad // 2)


def _qc_section(title: str) -> list[str]:
  title = title.strip()
  return ["", title, "-" * min(len(title), QC_REPORT_WIDTH)]


def _qc_kv(rows: list[tuple[str, str]], key_width: int = 22) -> list[str]:
  lines: list[str] = []
  for key, val in rows:
    val = str(val).replace("\n", " ").strip()
    wrapped = textwrap.wrap(val, width=QC_REPORT_WIDTH - key_width - 2) or [""]
    lines.append(f"  {key:<{key_width}} {wrapped[0]}")
    for cont in wrapped[1:]:
      lines.append(f"  {' ' * key_width} {cont}")
  return lines


def _qc_format_test_results(run_results: list[tuple[str, bool, str]]) -> list[str]:
  if not run_results:
    return ["  (no test results recorded)"]

  passed = sum(1 for _, ok, err in run_results if ok and err != "skipped")
  skipped = sum(1 for _, ok, err in run_results if ok and err == "skipped")
  failed = sum(1 for _, ok, _ in run_results if not ok)
  total = len(run_results)

  lines = [
    f"  Total {total}   passed {passed}   failed {failed}" + (f"   skipped {skipped}" if skipped else ""),
    "",
  ]
  name_w = max((len(n) for n, _, _ in run_results), default=4)
  name_w = min(max(name_w, 20), 40)

  for name, ok, err in run_results:
    if err == "skipped":
      tag, detail = "SKIP", ""
    elif ok:
      tag, detail = "PASS", ""
    else:
      tag, detail = "FAIL", err or ""

    lines.append(f"  [{tag:<4}]  {name:<{name_w}}")
    if detail:
      detail = detail.strip()
      if detail.startswith("AssertionError(") and detail.endswith(")"):
        detail = detail[len("AssertionError("):-1].strip() or detail
      for part in textwrap.wrap(detail, width=QC_REPORT_WIDTH - 10):
        lines.append(f"          {part}")
  return lines


def _format_ka2_onoff_qc_report(cls) -> str:
  out: list[str] = []
  out.append(_qc_rule())
  out.append(_qc_centered_title("KA2 ON/OFF TRANSITION QC REPORT"))
  out.append(_qc_rule())

  run_rows: list[tuple[str, str]] = []
  if cls._run_start_epoch is not None:
    run_rows.append(("Start (UTC)", time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(cls._run_start_epoch))))
    duration_s = max(0.0, time.time() - cls._run_start_epoch)
    run_rows.append(("Duration", f"{duration_s:.0f}s ({duration_s / 60:.1f} min)"))
  run_rows.append(("End (UTC)", time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())))
  run_rows.append(("Mode", str(cls._run_mode)))
  run_rows.append(("Cycles", str(ONOFF_CYCLES)))
  if getattr(cls, "_can_replay_route", None):
    run_rows.append(("CAN replay route", cls._can_replay_route))
  if getattr(cls, "_cycles_completed", None) is not None:
    run_rows.append(("Cycles completed", str(cls._cycles_completed)))

  out.extend(_qc_section("Run"))
  out.extend(_qc_kv(run_rows))

  cycle_timings = getattr(cls, "_cycle_timings", [])
  if cycle_timings:
    out.extend(_qc_section("Cycle timings"))
    out.extend(_qc_kv([("cycle", "onroad_s   offroad_s")]))
    for i, (on_s, off_s) in enumerate(cycle_timings, 1):
      on_str = f"{on_s:.2f}s" if on_s is not None else "n/a"
      off_str = f"{off_s:.2f}s" if off_s is not None else "n/a"
      out.extend(_qc_kv([(f"cycle_{i}", f"onroad={on_str}  offroad={off_str}")]))

  hold_timings = getattr(cls, "_hold_timings", [])
  if hold_timings:
    out.extend(_qc_section("Random hold durations"))
    out.extend(_qc_kv([("cycle", "onroad_hold_s   offroad_hold_s")]))
    for i, (on_h, off_h) in enumerate(hold_timings, 1):
      off_str = f"{off_h}s" if off_h > 0 else "n/a (last cycle)"
      out.extend(_qc_kv([(f"cycle_{i}", f"onroad={on_h}s  offroad={off_str}")]))

  memory_samples = getattr(cls, "_memory_samples", [])
  if memory_samples:
    out.extend(_qc_section("Memory across cycles"))
    out.extend(_qc_kv([(f"sample_{i}", f"{v:.1f}%") for i, v in enumerate(memory_samples)]))

  run_results = getattr(cls, "_run_results", None) or []
  failed = sum(1 for _, ok, _ in run_results if not ok)
  verdict = "OVERALL: PASS" if run_results and failed == 0 else "OVERALL: FAIL" if run_results else "OVERALL: (unknown)"
  out.extend(_qc_section(f"Tests  —  {verdict}"))
  out.extend(_qc_format_test_results(run_results))

  out.append("")
  out.append(_qc_rule())
  return "\n".join(out) + "\n"


def _emit_qc_report(text: str, report_path: str) -> None:
  footer = f"\n{_qc_rule()}\n  Report file: {report_path}\n{_qc_rule()}\n"
  try:
    sys.stderr.write("\n")
    sys.stderr.write(text)
    sys.stderr.write(footer)
    sys.stderr.flush()
  except (OSError, UnicodeEncodeError) as e:
    try:
      sys.stderr.write(f"[ka2_onoff] could not print QC report: {e!r}\n")
      sys.stderr.flush()
    except OSError:
      pass


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def _fmt_num(v) -> str:
  try:
    return f"{float(v):.2f}"
  except (TypeError, ValueError):
    return "n/a"


def _stop_existing_openpilot() -> None:
  for pattern in (
    "launch_openpilot.sh",
    "launch_chffrplus.sh",
    "system.manager.manager",
    "system/manager/manager.py",
    "selfdrive/test/ka2_can_replay_feeder.py",
  ):
    try:
      subprocess.run(["pkill", "-f", pattern], check=False)
    except OSError:
      pass
  time.sleep(3.0)


def _proc_running(proc_name: str) -> bool:
  try:
    p = subprocess.run(
      ["pgrep", "-f", proc_name],
      capture_output=True, text=True, timeout=5,
    )
    return p.returncode == 0
  except Exception:
    return False


def _read_memory_usage_pct() -> float | None:
  try:
    sm = messaging.SubMaster(["deviceState"])
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
      sm.update(200)
      if sm.seen["deviceState"]:
        return float(sm["deviceState"].memoryUsagePercent)
    return None
  except Exception:
    return None


def _set_ignition(feeder_proc, *, on: bool) -> None:
  """
  Toggle ignition by stopping or starting the CAN replay feeder.

  Ignition on  = feeder running  → pandaStates has ignitionLine=True
  Ignition off = feeder stopped  → no pandaStates → selfdrived sets IsOffroad
  """
  if on:
    pass
  else:
    if feeder_proc is not None and feeder_proc.poll() is None:
      feeder_proc.terminate()
      try:
        feeder_proc.wait(5)
      except subprocess.TimeoutExpired:
        feeder_proc.kill()


def _wait_for_onroad(timeout_s: float = ONROAD_TRANSITION_TIMEOUT_S) -> float:
  sm = messaging.SubMaster(["carState"])
  t0 = time.monotonic()
  with Timeout(timeout_s, "onroad transition timed out: carState not seen"):
    while not sm.seen["carState"]:
      sm.update(500)
  return time.monotonic() - t0


def _wait_for_offroad(timeout_s: float = OFFROAD_TRANSITION_TIMEOUT_S) -> float:
  params = Params()
  t0 = time.monotonic()
  deadline = t0 + timeout_s
  while time.monotonic() < deadline:
    if params.get_bool("IsOffroad"):
      return time.monotonic() - t0
    time.sleep(0.5)
  raise TimeoutError(
    f"offroad transition timed out after {timeout_s}s: IsOffroad never became True"
  )


def _start_feeder(can_replay_route: str) -> subprocess.Popen:
  feeder_script = os.path.join(BASEDIR, "selfdrive/test/ka2_can_replay_feeder.py")
  proc = subprocess.Popen(
    [sys.executable, feeder_script, can_replay_route],
    cwd=BASEDIR,
    env=os.environ.copy(),
  )
  time.sleep(1.0)
  assert proc.poll() is None, "CAN replay feeder exited early on start"
  return proc


def _skip_if_not_ka2() -> None:
  if HARDWARE.get_device_type() != "ka2":
    pytest.skip("KA2 only")


# ============================================================
# MAIN TEST CLASS
# ============================================================

@pytest.mark.ka2
class TestOnOff:
  _run_results: list[tuple[str, bool, str]] = []
  _run_mode: str = "pytest"
  _run_start_epoch: float | None = None
  _can_replay_route: str | None = None
  _cycles_completed: int = 0
  _hold_timings: list[tuple[int, int]] = []
  _cycle_timings: list[tuple[float | None, float | None]] = []
  _memory_samples: list[float] = []
  _proc_results: list[dict] = []
  _manager_proc: subprocess.Popen | None = None
  _feeder_proc: subprocess.Popen | None = None

  @classmethod
  def setup_class(cls):
    cls._run_results = []
    cls._run_mode = "pytest"
    cls._run_start_epoch = time.time()
    cls._can_replay_route = None
    cls._cycles_completed = 0
    cls._hold_timings = [] 
    cls._cycle_timings = []
    cls._memory_samples = []
    cls._proc_results = []
    cls._manager_proc = None
    cls._feeder_proc = None

    _stop_existing_openpilot()

    from openpilot.selfdrive.test.ka2_can_replay_feeder import KA2_QC_RLOG_URL, load_route_can_msgs
    from opendbc.car.car_helpers import interfaces

    params = Params()
    params.remove("CurrentRoute")
    params.put_bool("RecordFront", True)
    set_params_enabled()
    os.environ["REPLAY"] = "1"
    os.environ["MSGQ_PREALLOC"] = "1"
    os.environ["TESTING_CLOSET"] = "1"
    os.environ["IGNORE_RELAY_MALFUNCTION_IN_REPLAY"] = "1"
    os.environ["SKIP_FW_QUERY"] = "1"

    cls._can_replay_route = os.environ.get("KA2_CAN_REPLAY_ROUTE", KA2_QC_RLOG_URL).strip()
    _, cp, cp_bytes = load_route_can_msgs(cls._can_replay_route)
    fingerprint = cp.carFingerprint
    assert fingerprint in interfaces, f"unsupported fingerprint {fingerprint!r}"
    params.put("CarParams", cp_bytes)
    params.put("CarParamsCache", cp_bytes)
    params.put("CarParamsPersistent", cp_bytes)
    params.put_bool("FirmwareQueryDone", True)
    os.environ["BLOCK"] = "pandad,uploader"
    os.environ["FINGERPRINT"] = fingerprint

    if isinstance(HARDWARE, Ka2):
      HARDWARE.set_power_save(False)

    try:
      subprocess.run(["sudo", "-n", "killall", "-q", "/usr/kommu/rkaiq_3A_server"], check=False)
      time.sleep(2.5)
      subprocess.run(["sudo", "-n", "bash", "-lc", "/usr/kommu/rkaiq_3A_server >/dev/null 2>&1 &"], check=False)
    except OSError:
      pass

    try:
      subprocess.run(["pkill", "-f", "system.loggerd.uploader"], check=False)
    except OSError:
      pass

    try:
      cls._feeder_proc = _start_feeder(cls._can_replay_route)

      env = os.environ.copy()
      cls._manager_proc = subprocess.Popen(
        ["bash", "-lc", "exec ./launch_openpilot.sh"],
        cwd=BASEDIR,
        env=env,
        preexec_fn=os.setsid,
      )

      # Wait for first onroad transition
      _wait_for_onroad(ONROAD_TRANSITION_TIMEOUT_S)

      # ── 5 ignition on/off cycles ─────────────────────────────────────────
      for cycle in range(ONOFF_CYCLES):
        cycle_on_s: float | None = None
        cycle_off_s: float | None = None
        proc_result: dict = {"cycle": cycle + 1}

        # sample memory at start of cycle
        mem = _read_memory_usage_pct()
        if mem is not None:
          cls._memory_samples.append(mem)

        # restart feeder if it died between cycles
        if cls._feeder_proc is None or cls._feeder_proc.poll() is not None:
          cls._feeder_proc = _start_feeder(cls._can_replay_route)

        # measure onroad transition time
        t_on_start = time.monotonic()
        try:
          _wait_for_onroad(ONROAD_TRANSITION_TIMEOUT_S)
          cycle_on_s = time.monotonic() - t_on_start
        except Exception as e:
          _safe_print(f"[cycle {cycle + 1}] onroad transition failed: {e!r}")
          cycle_on_s = None

        # record which onroad processes are running
        proc_result["onroad_procs"] = {
          p: _proc_running(p) for p in ONROAD_PROCS
        }
        proc_result["soundd_running"] = _proc_running("selfdrive.ui.soundd")
        proc_result["indicatord_running"] = _proc_running("system.hardware.ka2.status_led.indicatord")

        # ── random onroad hold before toggling ignition off ───────────────
        onroad_hold_s = random.randint(ONROAD_HOLD_MIN_S, ONROAD_HOLD_MAX_S)
        _safe_print(f"[cycle {cycle + 1}] staying onroad for {onroad_hold_s}s...")
        time.sleep(onroad_hold_s)

        # ── OFFROAD: kill feeder → ignition off ──────────────────────────
        _set_ignition(cls._feeder_proc, on=False)
        cls._feeder_proc = None

        t_off_start = time.monotonic()
        try:
          cycle_off_s = _wait_for_offroad(OFFROAD_TRANSITION_TIMEOUT_S)
        except Exception as e:
          _safe_print(f"[cycle {cycle + 1}] offroad transition failed: {e!r}")
          cycle_off_s = None

        # record which processes stopped after going offroad
        time.sleep(2.0)
        proc_result["offroad_zombie_procs"] = {
          p: _proc_running(p) for p in NO_ZOMBIE_PROCS
        }
        proc_result["indicatord_offroad"] = _proc_running("system.hardware.ka2.status_led.indicatord")

        cls._cycle_timings.append((cycle_on_s, cycle_off_s))
        cls._proc_results.append(proc_result)
        cls._cycles_completed += 1

        _safe_print(
          f"[cycle {cycle + 1}/{ONOFF_CYCLES}] "
          f"onroad={_fmt_num(cycle_on_s)}s  offroad={_fmt_num(cycle_off_s)}s"
        )

        # ── random offroad hold before next cycle ─────────────────────────
        if cycle < ONOFF_CYCLES - 1:
          offroad_hold_s = random.randint(OFFROAD_HOLD_MIN_S, OFFROAD_HOLD_MAX_S)
          _safe_print(f"[cycle {cycle + 1}] staying offroad for {offroad_hold_s}s...")
          cls._hold_timings.append((onroad_hold_s, offroad_hold_s))  # ADD THIS
          time.sleep(offroad_hold_s)
        else:
          cls._hold_timings.append((onroad_hold_s, 0))  # last cycle, no offroad hold

        # restart feeder for next cycle (except after last cycle)
        if cycle < ONOFF_CYCLES - 1:
          cls._feeder_proc = _start_feeder(cls._can_replay_route)

      # final memory sample after all cycles
      mem = _read_memory_usage_pct()
      if mem is not None:
        cls._memory_samples.append(mem)

    finally:
      if cls._feeder_proc is not None and cls._feeder_proc.poll() is None:
        cls._feeder_proc.terminate()
        try:
          cls._feeder_proc.wait(10)
        except subprocess.TimeoutExpired:
          cls._feeder_proc.kill()

      if cls._manager_proc is not None:
        try:
          os.killpg(os.getpgid(cls._manager_proc.pid), signal.SIGTERM)
        except OSError:
          pass
        try:
          cls._manager_proc.wait(60)
        except subprocess.TimeoutExpired:
          try:
            os.killpg(os.getpgid(cls._manager_proc.pid), signal.SIGKILL)
          except OSError:
            pass

      _start_ka2_panda_heartbeat_guard_after_manager_exit()

  @classmethod
  def teardown_class(cls):
    try:
      cls._write_qc_report()
    except Exception as e:
      try:
        sys.stderr.write(f"[ka2_onoff] QC report generation failed: {e!r}\n")
        sys.stderr.write(traceback.format_exc())
        sys.stderr.flush()
      except OSError:
        pass
    try:
      unset_params_enabled()
      Params().remove("RecordFront")
      if isinstance(HARDWARE, Ka2):
        try:
          HARDWARE.set_power_save(True)
        except Exception:
          pass
    except Exception as e:
      try:
        sys.stderr.write(f"[ka2_onoff] teardown cleanup failed: {e!r}\n")
        sys.stderr.flush()
      except OSError:
        pass

  @classmethod
  def _write_qc_report(cls):
    text = _format_ka2_onoff_qc_report(cls)
    try:
      with open("/data/onoff_qc.log", "w", encoding="utf-8") as f:
        f.write(text)
      report_path = "/data/onoff_qc.log"
    except OSError as e:
      report_path = f"/data/onoff_qc.log (write failed: {e})"
    _emit_qc_report(text, report_path)

  # ============================================================
  # TEST METHODS
  # ============================================================

  def test_transition_timing(self):
    _skip_if_not_ka2()
    assert self._cycle_timings, "no cycle timing data — setup_class may have failed"

    _safe_print("\n------------------------------------------------")
    _safe_print("------------- Transition Timings ---------------")
    _safe_print("------------------------------------------------")
    _safe_print(f"  {'Cycle':<8} {'Onroad (s)':<15} {'Offroad (s)':<15}")
    _safe_print(f"  {'-'*6:<8} {'-'*12:<15} {'-'*12:<15}")

    failed_cycles = []
    for i, (on_s, off_s) in enumerate(self._cycle_timings, 1):
      on_str = f"{on_s:.2f}" if on_s is not None else "TIMEOUT"
      off_str = f"{off_s:.2f}" if off_s is not None else "TIMEOUT"
      _safe_print(f"  {i:<8} {on_str:<15} {off_str:<15}")

      if on_s is None:
        failed_cycles.append(f"cycle {i}: onroad transition timed out (>{ONROAD_TRANSITION_TIMEOUT_S}s)")
      elif on_s > ONROAD_TRANSITION_TIMEOUT_S:
        failed_cycles.append(f"cycle {i}: onroad too slow ({on_s:.2f}s > {ONROAD_TRANSITION_TIMEOUT_S}s)")

      if off_s is None:
        failed_cycles.append(f"cycle {i}: offroad transition timed out (>{OFFROAD_TRANSITION_TIMEOUT_S}s)")
      elif off_s > OFFROAD_TRANSITION_TIMEOUT_S:
        failed_cycles.append(f"cycle {i}: offroad too slow ({off_s:.2f}s > {OFFROAD_TRANSITION_TIMEOUT_S}s)")

    _safe_print("------------------------------------------------")
    assert not failed_cycles, "transition timing failures:\n" + "\n".join(f"  {e}" for e in failed_cycles)

  def test_process_start_stop(self):
    _skip_if_not_ka2()
    assert self._proc_results, "no process data — setup_class may have failed"

    start_failures = []
    stop_failures = []

    for r in self._proc_results:
      cycle = r["cycle"]
      for proc, running in r.get("onroad_procs", {}).items():
        if not running:
          start_failures.append(f"cycle {cycle}: {proc} not running after ignition on")
      for proc, running in r.get("offroad_zombie_procs", {}).items():
        if running:
          stop_failures.append(f"cycle {cycle}: {proc} still running after ignition off (zombie)")

    errors = start_failures + stop_failures
    assert not errors, "process start/stop failures:\n" + "\n".join(f"  {e}" for e in errors)

  def test_memory_across_cycles(self):
    _skip_if_not_ka2()
    assert len(self._memory_samples) >= 2, (
      f"not enough memory samples to compare: got {len(self._memory_samples)}"
    )

    baseline = self._memory_samples[0]
    final = self._memory_samples[-1]
    growth = final - baseline

    _safe_print("\n------------------------------------------------")
    _safe_print("------------- Memory Across Cycles -------------")
    _safe_print("------------------------------------------------")
    for i, v in enumerate(self._memory_samples):
      label = "baseline" if i == 0 else f"after cycle {i}" if i < len(self._memory_samples) - 1 else "final"
      _safe_print(f"  {label:<20} {v:.1f}%")
    _safe_print(f"  {'growth':<20} {growth:+.1f}%  (max allowed: +{MEMORY_GROWTH_MAX_PCT}%)")
    _safe_print("------------------------------------------------")

    assert growth <= MEMORY_GROWTH_MAX_PCT, (
      f"memory leak across {ONOFF_CYCLES} cycles: "
      f"baseline={baseline:.1f}% final={final:.1f}% growth={growth:+.1f}% "
      f"max_allowed={MEMORY_GROWTH_MAX_PCT}%"
    )

  def test_led_state_per_transition(self):
    _skip_if_not_ka2()
    assert self._proc_results, "no process data — setup_class may have failed"

    failures = []
    _safe_print("\n------------------------------------------------")
    _safe_print("------------- LED State Per Cycle --------------")
    _safe_print("------------------------------------------------")
    _safe_print(f"  {'Cycle':<8} {'Onroad indicatord':<22} {'Offroad indicatord':<22}")
    _safe_print(f"  {'-'*6:<8} {'-'*18:<22} {'-'*18:<22}")

    for r in self._proc_results:
      cycle = r["cycle"]
      on_led = r.get("indicatord_running", False)
      off_led = r.get("indicatord_offroad", False)
      _safe_print(
        f"  {cycle:<8} "
        f"{'✅ running' if on_led else '❌ NOT running':<22} "
        f"{'✅ running' if off_led else '❌ NOT running':<22}"
      )
      if not on_led:
        failures.append(f"cycle {cycle}: indicatord not running during onroad state")
      if not off_led:
        failures.append(f"cycle {cycle}: indicatord not running during offroad state")

    _safe_print("------------------------------------------------")
    assert not failures, "LED indicatord failures:\n" + "\n".join(f"  {e}" for e in failures)

  def test_speaker_on_onroad_entry(self):
    _skip_if_not_ka2()
    assert self._proc_results, "no process data — setup_class may have failed"

    failures = []
    for r in self._proc_results:
      cycle = r["cycle"]
      soundd_ok = r.get("soundd_running", False)
      if not soundd_ok:
        failures.append(f"cycle {cycle}: soundd not running during onroad state")

    assert not failures, "soundd failures:\n" + "\n".join(f"  {e}" for e in failures)


# ============================================================
# __main__ BLOCK
# ============================================================

if __name__ == "__main__":
  if "--can-replay" in sys.argv:
    sys.argv = [a for a in sys.argv if a != "--can-replay"]
    os.environ["KA2_CAN_REPLAY"] = "1"
  else:
    os.environ.pop("KA2_CAN_REPLAY", None)

  class _DummySubtests:
    def test(self, *args, **kwargs):
      return nullcontext()

  test_obj = TestOnOff()
  subtests = _DummySubtests()
  failed = 0
  setup_error: BaseException | None = None

  TestOnOff._run_mode = "__main__"
  try:
    TestOnOff.setup_class()
  except BaseException as e:
    setup_error = e
    TestOnOff._run_results.append(("setup_class", False, repr(e)))
    _safe_print(f"FAIL: setup_class -> {repr(e)}")
    _safe_print(traceback.format_exc())

  try:
    if setup_error is None:
      test_methods = sorted(m for m in dir(TestOnOff) if m.startswith("test_"))
      for name in test_methods:
        fn = getattr(test_obj, name)
        sig = inspect.signature(fn)
        _safe_print(f"\n=== Running {name} ===")
        try:
          if "subtests" in sig.parameters:
            fn(subtests)
          else:
            fn()
          TestOnOff._run_results.append((name, True, ""))
          _safe_print(f"PASS: {name}")
        except pytest.skip.Exception:
          TestOnOff._run_results.append((name, True, "skipped"))
          _safe_print(f"SKIP: {name}")
        except Exception as e:
          failed += 1
          TestOnOff._run_results.append((name, False, repr(e)))
          _safe_print(f"FAIL: {name} -> {repr(e)}")
          _safe_print(traceback.format_exc())
  finally:
    teardown = getattr(TestOnOff, "teardown_class", None)
    if callable(teardown):
      teardown()

  if setup_error is not None:
    raise SystemExit(1)
  if failed:
    raise SystemExit(1)
  try:
    print("\nAll tests passed.", flush=True)
  except OSError:
    pass

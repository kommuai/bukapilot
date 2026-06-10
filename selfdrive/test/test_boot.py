#!/usr/bin/env python3
"""KA2 Boot QC test — runs hardware checks across multiple boot cycles.

Flow:
  1. Technician SSHes in once and runs:
       touch /data/qc_boot_mode
     Then unplugs and replugs the power cable (cold boot).
  2. On boot, systemd service runs this script automatically.
  3. Script checks QC mode file, runs hardware checks for this cycle.
  4. Device reboots automatically via sudo reboot.
  5. On next boot, systemd runs this script again.
  6. Repeats for BOOT_CYCLES total cycles.
  7. Final cycle writes full report to /data/boot_qc.log.

QC mode:
  - Enabled by:  touch /data/qc_boot_mode
  - Disabled by: rm /data/qc_boot_mode  (auto-removed after final cycle)
  - Cycle counter stored in: /data/boot_qc_cycle
"""

import os
import sys
import inspect
import traceback
import json
import pytest
import subprocess
import textwrap
import time
from contextlib import nullcontext
from datetime import datetime
from pathlib import Path

from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.ka2.hardware import Ka2
from openpilot.system.hardware.hw import Paths

# ============================================================
# CONSTANTS
# ============================================================

QC_REPORT_WIDTH = 72
SD_MEDIA_MOUNT = "/data/media"

# Number of boot cycles to run
BOOT_CYCLES = 5

# Max seconds to wait for idle (yellow) after reboot
IDLE_WAIT_TIMEOUT_S = 120

# File that enables QC mode
QC_MODE_FILE = "/data/qc_boot_mode"

# File that tracks current cycle number (persists across reboots)
CYCLE_COUNTER_FILE = "/data/boot_qc_cycle"

# File that accumulates results across cycles (JSON)
CYCLE_RESULTS_FILE = "/data/boot_qc_cycles.json"

# Final report path
REPORT_PATH = "/data/boot_qc.log"


# ============================================================
# HELPERS — PRINT / REPORT
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


def _format_ka2_boot_qc_report(all_cycle_results: list[dict], start_epoch: float) -> str:
  out: list[str] = []
  out.append(_qc_rule())
  out.append(_qc_centered_title("KA2 BOOT QC REPORT"))
  out.append(_qc_rule())

  duration_s = max(0.0, time.time() - start_epoch)
  run_rows = [
    ("Start (UTC)", time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(start_epoch))),
    ("End (UTC)", time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())),
    ("Duration", f"{duration_s:.0f}s ({duration_s / 60:.1f} min)"),
    ("Device type", str(HARDWARE.get_device_type())),
    ("Hostname", os.uname().nodename),
    ("Total cycles", str(len(all_cycle_results))),
  ]

  out.extend(_qc_section("Run"))
  out.extend(_qc_kv(run_rows))

  # Per-cycle results
  overall_pass = True
  for cycle_data in all_cycle_results:
    cycle_num = cycle_data.get("cycle", "?")
    boot_time = cycle_data.get("boot_time_s")
    results = cycle_data.get("results", [])
    boot_time_str = f"{boot_time:.1f}s" if boot_time is not None else "n/a"

    cycle_failed = sum(1 for _, ok, _ in results if not ok)
    if cycle_failed:
      overall_pass = False

    out.extend(_qc_section(
      f"Cycle {cycle_num} / {BOOT_CYCLES}  —  boot_time={boot_time_str}  {'PASS' if not cycle_failed else 'FAIL'}"
    ))
    out.extend(_qc_format_test_results(results))

  verdict = "OVERALL: PASS" if overall_pass and all_cycle_results else "OVERALL: FAIL"
  out.extend(_qc_section(verdict))
  out.extend(_qc_kv([("Report file", REPORT_PATH)]))
  out.append("")
  out.append(_qc_rule())
  return "\n".join(out) + "\n"


def _emit_qc_report(text: str) -> None:
  try:
    sys.stdout.write("\n")
    sys.stdout.write(text)
    sys.stdout.flush()
  except (OSError, UnicodeEncodeError) as e:
    try:
      sys.stderr.write(f"[ka2_boot] could not print QC report: {e!r}\n")
    except OSError:
      pass


# ============================================================
# HELPERS — CYCLE STATE (persists across reboots)
# ============================================================

def _read_cycle() -> int:
  """Read current cycle number from disk (0 = not started)."""
  try:
    return int(Path(CYCLE_COUNTER_FILE).read_text(encoding="utf-8").strip())
  except Exception:
    return 0


def _write_cycle(n: int) -> None:
  Path(CYCLE_COUNTER_FILE).write_text(str(n), encoding="utf-8")


def _read_all_cycle_results() -> list[dict]:
  try:
    return json.loads(Path(CYCLE_RESULTS_FILE).read_text(encoding="utf-8"))
  except Exception:
    return []


def _append_cycle_result(cycle_data: dict) -> None:
  results = _read_all_cycle_results()
  results.append(cycle_data)
  Path(CYCLE_RESULTS_FILE).write_text(json.dumps(results), encoding="utf-8")


def _clear_cycle_state() -> None:
  for f in (CYCLE_COUNTER_FILE, CYCLE_RESULTS_FILE):
    try:
      Path(f).unlink(missing_ok=True)
    except Exception:
      pass


def _read_start_epoch() -> float:
  try:
    return float(Path("/data/boot_qc_start_epoch").read_text(encoding="utf-8").strip())
  except Exception:
    return time.time()


def _write_start_epoch(epoch: float) -> None:
  Path("/data/boot_qc_start_epoch").write_text(str(epoch), encoding="utf-8")


# ============================================================
# HELPERS — HARDWARE
# ============================================================

def _mmcli_modem_enumerated() -> bool:
  try:
    p = subprocess.run(["mmcli", "-L"], capture_output=True, text=True, timeout=3)
    return p.returncode == 0 and "/Modem/" in (p.stdout or "")
  except Exception:
    return False


def _ka2_sd_mounted() -> tuple[bool, str]:
  try:
    p = subprocess.run(["df", "-h"], capture_output=True, text=True, timeout=10)
    if p.returncode != 0:
      return False, (p.stderr or p.stdout or "df failed").strip()
    for line in p.stdout.splitlines():
      line = line.strip()
      if not line or line.startswith("Filesystem"):
        continue
      parts = line.split()
      if len(parts) < 6:
        continue
      if parts[-1] == SD_MEDIA_MOUNT and "mmcblk1" in parts[0]:
        return True, parts[0]
    return False, f"no df -h line with mmcblk1 on {SD_MEDIA_MOUNT}"
  except Exception as e:
    return False, repr(e)


def _ka2_verify_log_root_writable() -> tuple[bool, str]:
  log_root = Path(Paths.log_root().rstrip("/"))
  media_root = Path(SD_MEDIA_MOUNT)
  try:
    if not _ka2_sd_mounted()[0]:
      return False, f"{SD_MEDIA_MOUNT} not mounted"
    media_dev = os.stat(media_root).st_dev
    (media_root / "0").mkdir(parents=True, exist_ok=True)
    log_root.mkdir(parents=True, exist_ok=True)
    if os.stat(log_root).st_dev != media_dev:
      return False, f"{log_root} is not on {SD_MEDIA_MOUNT} (wrong filesystem)"
    test_file = log_root / ".ka2_boot_preflight_write_test"
    test_file.write_text("ok\n")
    test_file.unlink(missing_ok=True)
    if not os.access(log_root, os.W_OK):
      return False, f"{log_root} not writable (access)"
    return True, str(log_root)
  except Exception as e:
    return False, repr(e)


def _skip_if_not_ka2() -> None:
  if HARDWARE.get_device_type() != "ka2":
    pytest.skip("KA2 only")


def _wait_for_idle(timeout_s: float = IDLE_WAIT_TIMEOUT_S) -> float | None:
  """Wait until NetworkManager + ModemManager are up (= yellow/idle).
  Returns boot time in seconds, or None on timeout."""
  t0 = time.monotonic()
  deadline = t0 + timeout_s
  while time.monotonic() < deadline:
    nm = subprocess.run(["pgrep", "-x", "NetworkManager"], capture_output=True)
    mm = subprocess.run(["pgrep", "-x", "ModemManager"], capture_output=True)
    if nm.returncode == 0 and mm.returncode == 0:
      return time.monotonic() - t0
    time.sleep(2)
  return None


# ============================================================
# TEST CLASS
# ============================================================

@pytest.mark.ka2
class TestBoot:
  _run_results: list[tuple[str, bool, str]] = []
  _run_mode = "pytest"
  _preflight: dict = {}
  _run_start_epoch: float | None = None

  @classmethod
  def setup_class(cls):
    cls._run_results = []
    cls._run_mode = "pytest"
    cls._preflight = {}
    cls._run_start_epoch = time.time()
    cls._preflight["device_type"] = HARDWARE.get_device_type()
    cls._preflight["hostname"] = os.uname().nodename
    cls._preflight["paths_log_root"] = Paths.log_root()
    cls._preflight["params_path_available"] = isinstance(Params(), Params)

  @classmethod
  def teardown_class(cls):
    try:
      cls._write_qc_report()
    except Exception as e:
      try:
        sys.stderr.write(f"[ka2_boot] QC report generation failed: {e!r}\n")
        sys.stderr.write(traceback.format_exc())
        sys.stderr.flush()
      except OSError:
        pass

  @classmethod
  def _write_qc_report(cls):
    """Write single-cycle report (used when running via pytest directly)."""
    out: list[str] = []
    out.append(_qc_rule())
    out.append(_qc_centered_title("KA2 BOOT QC REPORT"))
    out.append(_qc_rule())

    run_rows: list[tuple[str, str]] = []
    if cls._run_start_epoch is not None:
      run_rows.append(("Start (UTC)", time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(cls._run_start_epoch))))
      duration_s = max(0.0, time.time() - cls._run_start_epoch)
      run_rows.append(("Duration", f"{duration_s:.0f}s ({duration_s / 60:.1f} min)"))
    run_rows.append(("End (UTC)", time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())))
    run_rows.append(("Mode", str(cls._run_mode)))
    run_rows.append(("Device type", str(HARDWARE.get_device_type())))
    if cls._preflight:
      for k in sorted(cls._preflight.keys()):
        run_rows.append((k, str(cls._preflight[k])))

    out.extend(_qc_section("Run"))
    out.extend(_qc_kv(run_rows))

    run_results = getattr(cls, "_run_results", None) or []
    failed = sum(1 for _, ok, _ in run_results if not ok)
    verdict = "OVERALL: PASS" if run_results and failed == 0 else "OVERALL: FAIL" if run_results else "OVERALL: (unknown)"
    out.extend(_qc_section(f"Tests  —  {verdict}"))
    out.extend(_qc_format_test_results(run_results))
    out.extend(_qc_section("Boot notes"))
    out.extend(_qc_kv([("Report file", REPORT_PATH)]))
    out.append("")
    out.append(_qc_rule())
    text = "\n".join(out) + "\n"

    try:
      with open(REPORT_PATH, "w", encoding="utf-8") as f:
        f.write(text)
    except OSError as e:
      sys.stderr.write(f"[ka2_boot] report write failed: {e!r}\n")
    _emit_qc_report(text)

  # ----------------------------------------------------------
  # TESTS
  # ----------------------------------------------------------

  def test_all_processes_up(self):
    _skip_if_not_ka2()
    for proc in ("systemd", "NetworkManager", "ModemManager"):
      p = subprocess.run(["pgrep", "-x", proc], capture_output=True, text=True, timeout=5)
      assert p.returncode == 0, (
        f"required process missing: process={proc} rc={p.returncode} stdout={p.stdout!r} stderr={p.stderr!r}"
      )

  def test_sd_card_mounted(self):
    _skip_if_not_ka2()
    mounted, mount_src = _ka2_sd_mounted()
    assert mounted, f"SD mount check failed: expected mmcblk1 on {SD_MEDIA_MOUNT}, got {mount_src}"
    log_ok, log_detail = _ka2_verify_log_root_writable()
    assert log_ok, f"log root writable check failed: expected writable under {SD_MEDIA_MOUNT}, got {log_detail}"

  def test_modem_enumerated(self):
    _skip_if_not_ka2()
    p = subprocess.run(["mmcli", "-L"], capture_output=True, text=True, timeout=5)
    assert p.returncode == 0, f"mmcli -L failed: rc={p.returncode} stderr={p.stderr!r}"
    assert _mmcli_modem_enumerated(), f"no modem enumerated: stdout={p.stdout!r}"

  def test_rtc_not_reset(self):
    _skip_if_not_ka2()
    now = datetime.now()
    assert now.year >= 2024, (
      f"system time looks reset: actual_year={now.year} expected_year>=2024 now={now.isoformat()}"
    )
    since_epoch_path = Path("/sys/class/rtc/rtc0/since_epoch")
    if since_epoch_path.exists():
      raw = since_epoch_path.read_text(encoding="utf-8").strip()
      since_epoch = int(raw)
      min_epoch = int(datetime(2024, 1, 1).timestamp())
      assert since_epoch >= min_epoch, (
        f"RTC since_epoch too old: actual={since_epoch} expected>={min_epoch} path={since_epoch_path}"
      )

  def test_can_interface_up(self):
    _skip_if_not_ka2()
    p = subprocess.run(["ip", "link", "show", "can0"], capture_output=True, text=True, timeout=5)
    assert p.returncode == 0, f"can0 missing: rc={p.returncode} stderr={p.stderr!r}"
    out = (p.stdout or "").lower()
    assert "can0" in out, f"can0 missing from ip output: stdout={p.stdout!r}"


# ============================================================
# MULTI-CYCLE RUNNER  (used when run as __main__)
# ============================================================

def _run_single_cycle(cycle_num: int) -> tuple[list[tuple[str, bool, str]], bool]:
  """Run all TestBoot tests for one cycle. Returns (results, all_passed)."""
  test_obj = TestBoot()
  results: list[tuple[str, bool, str]] = []

  try:
    TestBoot.setup_class()
  except Exception as e:
    results.append(("setup_class", False, repr(e)))
    _safe_print(f"FAIL: setup_class -> {repr(e)}")
    return results, False

  test_methods = sorted(m for m in dir(TestBoot) if m.startswith("test_"))
  for name in test_methods:
    fn = getattr(test_obj, name)
    _safe_print(f"  [{cycle_num}/{BOOT_CYCLES}] Running {name}...")
    try:
      fn()
      results.append((name, True, ""))
      _safe_print(f"  PASS: {name}")
    except pytest.skip.Exception:
      results.append((name, True, "skipped"))
      _safe_print(f"  SKIP: {name}")
    except Exception as e:
      results.append((name, False, repr(e)))
      _safe_print(f"  FAIL: {name} -> {repr(e)}")

  all_passed = all(ok for _, ok, err in results if err != "skipped")
  return results, all_passed


if __name__ == "__main__":
  # ── Check QC mode ────────────────────────────────────────────────────────
  if not Path(QC_MODE_FILE).exists():
    _safe_print(f"[ka2_boot] QC mode not enabled. Run: touch {QC_MODE_FILE}")
    _safe_print(f"[ka2_boot] Then unplug and replug the device to cold boot.")
    sys.exit(0)

  cycle = _read_cycle()

  # ── First cycle — initialise state ───────────────────────────────────────
  if cycle == 0:
    _clear_cycle_state()
    _write_start_epoch(time.time())
    _safe_print(f"\n{'='*QC_REPORT_WIDTH}")
    _safe_print(_qc_centered_title(f"KA2 BOOT QC — {BOOT_CYCLES} CYCLES"))
    _safe_print(f"{'='*QC_REPORT_WIDTH}\n")

  cycle += 1
  _write_cycle(cycle)

  # ── Wait for idle/yellow before running checks ───────────────────────────
  # Skip on cycle 1 since we're already up (just booted into this script)
  boot_time_s: float | None = None
  if cycle > 1:
    _safe_print(f"\n[ka2_boot] Cycle {cycle}/{BOOT_CYCLES} — waiting for idle (yellow)...")
    boot_time_s = _wait_for_idle()
    if boot_time_s is None:
      _safe_print(f"[ka2_boot] TIMEOUT waiting for idle on cycle {cycle}!")
    else:
      _safe_print(f"[ka2_boot] Idle reached in {boot_time_s:.1f}s")
  else:
    # Cycle 1: read uptime from /proc/uptime — this IS the cold boot time
    try:
      uptime_raw = Path("/proc/uptime").read_text().split()[0]
      boot_time_s = float(uptime_raw)
    except Exception:
      boot_time_s = None

  _safe_print(f"\n[ka2_boot] === Cycle {cycle}/{BOOT_CYCLES} ===")
  results, passed = _run_single_cycle(cycle)

  cycle_data = {
    "cycle": cycle,
    "boot_time_s": boot_time_s,
    "results": results,
    "passed": passed,
    "timestamp_utc": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),
  }
  _append_cycle_result(cycle_data)

  status = "PASS" if passed else "FAIL"
  _safe_print(f"\n[ka2_boot] Cycle {cycle}/{BOOT_CYCLES}: {status}")

  # ── Not last cycle — reboot for next cycle ───────────────────────────────
  if cycle < BOOT_CYCLES:
    _safe_print(f"[ka2_boot] Rebooting for next cycle in 3s...")
    time.sleep(3)
    try:
      subprocess.run(["sudo", "reboot"], check=False)
    except Exception as e:
      _safe_print(f"[ka2_boot] reboot failed: {e!r} — please reboot manually")
    sys.exit(0)

  # ── Final cycle — write full report ──────────────────────────────────────
  _safe_print(f"\n[ka2_boot] All {BOOT_CYCLES} cycles complete. Writing report...")

  all_cycle_results = _read_all_cycle_results()
  start_epoch = _read_start_epoch()
  report_text = _format_ka2_boot_qc_report(all_cycle_results, start_epoch)

  try:
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
      f.write(report_text)
  except OSError as e:
    _safe_print(f"[ka2_boot] report write failed: {e!r}")

  _emit_qc_report(report_text)

  # ── Cleanup ───────────────────────────────────────────────────────────────
  _clear_cycle_state()
  try:
    Path("/data/boot_qc_start_epoch").unlink(missing_ok=True)
  except Exception:
    pass
  try:
    Path(QC_MODE_FILE).unlink(missing_ok=True)
  except Exception:
    pass

  overall_pass = all(c.get("passed", False) for c in all_cycle_results)
  sys.exit(0 if overall_pass else 1)

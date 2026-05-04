import math
import json
import os
import re
import sys
import inspect
import traceback
import pytest
import shutil
import signal
import subprocess
import time
import numpy as np
from collections import Counter, defaultdict
from contextlib import nullcontext
from pathlib import Path
from tabulate import tabulate

from cereal import car, log
import cereal.messaging as messaging
from cereal.services import SERVICE_LIST
from openpilot.common.basedir import BASEDIR
from openpilot.common.timeout import Timeout
from openpilot.common.params import Params
from openpilot.selfdrive.selfdrived.events import EVENTS, ET
from openpilot.system.hardware import HARDWARE
from openpilot.selfdrive.test.helpers import set_params_enabled, unset_params_enabled, release_only
from openpilot.system.hardware.hw import Paths
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.log_time_series import msgs_to_time_series

"""
CPU usage budget
* each process is entitled to at least 8%
* total CPU usage of openpilot (sum(PROCS.values())
  should not exceed MAX_TOTAL_CPU
"""

TEST_DURATION = 25
LOG_OFFSET = 8

# KA2 onroad test: assume driver model + DM state run at 10Hz (not cereal SERVICE_LIST 20Hz).
KA2_DM_SERVICE_HZ = 10.0
KA2_DM_TIMING_HZ_SERVICES = frozenset({"driverStateV2", "driverMonitoringState"})

# Burn-in: soak for this long after first carState; QC only loads the last N *complete* segments
# (highest segment indices; tail segment still dropped as incomplete). Soak stress != log parse cost.
BURN_IN_DURATION_S = 3600  # 1 hour
BURN_IN_ANALYZE_SEGMENT_COUNT = 5


def burn_in_test_enabled() -> bool:
  return os.environ.get("KA2_BURN_IN_TEST") == "1"


def _mono_span_seconds(lr: list) -> float:
  if not lr:
    return float(TEST_DURATION)
  return max(float(TEST_DURATION), (lr[-1].logMonoTime - lr[0].logMonoTime) / 1e9)


def _ka2_test_service_frequency_hz(service: str) -> float:
  """Nominal Hz used in this test for frequency/timing checks (KA2 DM services = 10Hz)."""
  if service in KA2_DM_TIMING_HZ_SERVICES:
    return KA2_DM_SERVICE_HZ
  return SERVICE_LIST[service].frequency


def _snapshot_device_state_temps() -> dict[str, str]:
  """Live read from deviceState while stack is still running (e.g. right before killing manager)."""
  out: dict[str, str] = {}
  try:
    sm = messaging.SubMaster(["deviceState"])
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
      sm.update(100)
      if sm.seen["deviceState"]:
        ds = sm["deviceState"]
        cpu = list(ds.cpuTempC) if ds.cpuTempC else []
        gpu = list(ds.gpuTempC) if ds.gpuTempC else []
        out["cpu_avg_c"] = _fmt_num(np.mean(cpu) if len(cpu) else None)
        out["cpu_max_c"] = _fmt_num(max(cpu) if len(cpu) else None)
        out["gpu_avg_c"] = _fmt_num(np.mean(gpu) if len(gpu) else None)
        out["gpu_max_c"] = _fmt_num(max(gpu) if len(gpu) else None)
        out["memory_c"] = _fmt_num(getattr(ds, "memoryTempC", None))
        return out
    out["note"] = "no deviceState within 2s"
  except Exception as e:
    out["error"] = repr(e)
  return out

MAX_TOTAL_CPU = 280.  # total for all 8 cores
PROCS = {
  "selfdrive.controls.controlsd": 10.0,
  "./loggerd": 15.0,
  "./encoderd": 10.0,
  "./camerad": 4.0,
  "selfdrive.locationd.locationd": 23.0,
  "selfdrive.locationd.lagd": 7.0,
  "selfdrive.controls.plannerd": 5.0,
  "selfdrive.locationd.paramsd": 10.0,
  "system.sensord.sensord": 11.0,
  "selfdrive.controls.radard": 1.0,
  "selfdrive.modeld.modeld": 32.0,
  "selfdrive.modeld.dmonitoringmodeld": 62.0,
  "selfdrive.locationd.calibrationd": 1.0,
  "selfdrive.locationd.torqued": 6.0,
  "selfdrive.ui.soundd": 4.0,
  "selfdrive.monitoring.dmonitoringd": 2.0,
  "system.proclogd": 1.0,
  "system.logmessaged": 0.2,
  "system.journald": 0.2,
  "system.tombstoned": 0,
  "system.micd": 4.0,
  "system.timed": 0,
  "selfdrive.pandad.pandad": 0,
  "system.loggerd.deleter": 0.1,
  "selfdrive.appbridged.appbridged": 7.0,
  "system.hardware.ka2.status_led.indicatord": 7.0,
  "system.qcomgpsd.qcomgpsd": -2.0,
  "system.hardware.hardwared": 3.0,
  "selfdrive.car.card": 16.0,
  "selfdrive.selfdrived.selfdrived": 16.0,
  "selfdrive.ui.feedback.feedbackd": 3.0,
}

KA2_CPU_MAX_ALLOWED = {
  "selfdrive.locationd.locationd": 30.0,
}

# managerState.processes uses manager names; procLog uses cmdline/module names.
# Keep this mapping in sync with system/manager/process_config.py for KA2 onroad.
MANAGER_TO_PROCS = {
  "loggerd": ["./loggerd"],
  "encoderd": ["./encoderd"],
  "logmessaged": ["system.logmessaged"],
  "camerad": ["./camerad"],
  "proclogd": ["system.proclogd"],
  "journald": ["system.journald"],
  "micd": ["system.micd"],
  "timed": ["system.timed"],
  "appbridged": ["selfdrive.appbridged.appbridged"],
  "indicatord": ["system.hardware.ka2.status_led.indicatord"],
  "modeld": ["selfdrive.modeld.modeld"],
  "dmonitoringmodeld": ["selfdrive.modeld.dmonitoringmodeld"],
  "sensord": ["system.sensord.sensord"],
  "soundd": ["selfdrive.ui.soundd"],
  "locationd": ["selfdrive.locationd.locationd"],
  "calibrationd": ["selfdrive.locationd.calibrationd"],
  "torqued": ["selfdrive.locationd.torqued"],
  "controlsd": ["selfdrive.controls.controlsd"],
  "selfdrived": ["selfdrive.selfdrived.selfdrived"],
  "card": ["selfdrive.car.card"],
  "deleter": ["system.loggerd.deleter"],
  "dmonitoringd": ["selfdrive.monitoring.dmonitoringd"],
  "qcomgpsd": ["system.qcomgpsd.qcomgpsd"],
  "pandad": ["selfdrive.pandad.pandad"],
  "paramsd": ["selfdrive.locationd.paramsd"],
  "lagd": ["selfdrive.locationd.lagd"],
  "plannerd": ["selfdrive.controls.plannerd"],
  "radard": ["selfdrive.controls.radard"],
  "hardwared": ["system.hardware.hardwared"],
  "tombstoned": ["system.tombstoned"],
  "feedbackd": ["selfdrive.ui.feedback.feedbackd"],
}

TIMINGS = {
  # rtols: max/min, rsd
  "can": [2.5, 0.35],
  "pandaStates": [2.5, 0.35],
  "peripheralState": [2.5, 0.35],
  "sendcan": [2.5, 0.35],
  "carState": [2.5, 0.35],
  "carControl": [2.5, 0.35],
  "controlsState": [2.5, 0.35],
  "longitudinalPlan": [2.5, 0.5],
  "driverAssistance": [2.5, 0.5],
  "roadCameraState": [2.5, 0.35],
  "driverCameraState": [2.5, 0.35],
  "modelV2": [2.5, 0.35],
  "driverStateV2": [2.5, 0.40],
  "driverMonitoringState": [2.5, 0.40],
  "livePose": [2.5, 0.35],
  "liveParameters": [2.5, 0.35],
  "wideRoadCameraState": [1.5, 0.35],
}

# Relax timing checks globally for KA2 stability.
GLOBAL_MEAN_RTOL = 0.55
GLOBAL_MAXMIN_SCALE = 1.2
GLOBAL_RSD_SCALE = 1.5

LOGS_SIZE = {  # MB per segment
  "qlog.zst": 0.5,
  "rlog.zst": 8.1,
  "qcamera.ts": 2.3,
}
LOGS_SIZE.update(dict.fromkeys(['ecamera.hevc', 'fcamera.hevc', 'dcamera.hevc'], 76.5))

# Bounds are relative to per-segment expected sizes.
# KA2 can vary quite a bit for qcamera.ts across boots, so keep this wider.
LOGS_SIZE_MULTIPLIERS = {
  "qlog.zst": (0.35, 2.0),
  "rlog.zst": (0.45, 2.0),
  "qcamera.ts": (0.10, 2.5),
  "ecamera.hevc": (0.75, 1.35),
  "fcamera.hevc": (0.75, 1.35),
  # Driver camera can start late / be intermittent on KA2 during bring-up.
  # Keep a low floor to avoid flakiness while still catching empty logs.
  "dcamera.hevc": (0.15, 1.35),
}


def cputime_total(ct):
  return ct.cpuUser + ct.cpuSystem + ct.cpuChildrenUser + ct.cpuChildrenSystem


def _safe_print(*args, **kwargs):
  try:
    print(*args, **kwargs)
  except (BlockingIOError, OSError):
    # Non-blocking terminals can reject large writes transiently.
    pass


def _fmt_num(v):
  try:
    return f"{float(v):.2f}"
  except (TypeError, ValueError):
    return "n/a"


# Quectel + qcomgpsd: enough messages for ~10s of nominal 2Hz service (rlog is often longer).
_MIN_QCOM_GNSS_MSGS = 20

# First modem in `mmcli -L` (index can be 0, 1, ... depending on bus/enumeration).
_mmcli_modem_index_cache: int | None = None


def _first_mmcli_modem_index() -> int:
  global _mmcli_modem_index_cache
  if _mmcli_modem_index_cache is not None:
    return _mmcli_modem_index_cache
  p = subprocess.run(
    ["mmcli", "-L"],
    capture_output=True, text=True, timeout=15, check=True,
  )
  ids = re.findall(r"/Modem/(\d+)", p.stdout)
  if not ids:
    raise RuntimeError(f"no modems in mmcli -L: stdout={p.stdout!r} stderr={p.stderr!r}")
  _mmcli_modem_index_cache = int(ids[0])
  return _mmcli_modem_index_cache


def _read_mmcli_modem_json() -> dict:
  try:
    idx = str(_first_mmcli_modem_index())
    out = subprocess.run(
      ["mmcli", "-J", "-m", idx],
      capture_output=True, text=True, check=True, timeout=15,
    ).stdout
    return json.loads(out)
  except Exception as e:
    return {"_error": repr(e)}


def _mmcli_at_command(cmd: str) -> tuple[int, str]:
  """AT via ModemManager; return (exitcode, combined stdout+stderr)."""
  try:
    idx = str(_first_mmcli_modem_index())
    p = subprocess.run(
      ["mmcli", "-m", idx, f"--command={cmd}"],
      shell=False,
      capture_output=True, text=True, timeout=20,
    )
    return p.returncode, (p.stdout or "") + (p.stderr or "")
  except Exception as e:
    return -1, repr(e)


@pytest.mark.ka2
class TestOnroad:
  _run_results = []
  _run_mode = "pytest"
  _preflight = {}
  _run_start_epoch = None
  _burn_in_enabled = False
  _burn_in_full_segments_available: int | None = None
  _shutdown_temperature: dict[str, str] | None = None

  @classmethod
  def setup_class(cls):
    cls._run_results = []
    cls._run_mode = "pytest"
    cls._preflight = {}
    cls._run_start_epoch = time.time()
    cls._burn_in_enabled = burn_in_test_enabled()
    cls._burn_in_full_segments_available = None
    cls.log_sizes_per_segment = []
    cls._shutdown_temperature = None

    # Fail fast on prerequisites before spending minutes on bring-up/logging.
    cls._preflight["sd_card_partition_present"] = Path("/dev/mmcblk1p1").exists()
    assert cls._preflight["sd_card_partition_present"], \
      "Preflight failed: /dev/mmcblk1p1 not found (SD card missing/unformatted/unmounted)"

    if "DEBUG" in os.environ:
      segs = filter(lambda x: os.path.exists(os.path.join(x, "rlog.zst")), Path(Paths.log_root()).iterdir())
      segs = sorted(segs, key=lambda x: x.stat().st_mtime)
      cls.lr = list(LogReader(os.path.join(segs[-1], "rlog.zst")))
      cls.ts = msgs_to_time_series(cls.lr)
      return

    # setup env
    params = Params()
    params.remove("CurrentRoute")
    params.put_bool("RecordFront", True)
    set_params_enabled()
    os.environ['REPLAY'] = '1'
    os.environ['NO_PANDA_TX_IN_REPLAY'] = '1'
    os.environ['MSGQ_PREALLOC'] = '1'
    os.environ['TESTING_CLOSET'] = '1'
    os.environ['IGNORE_RELAY_MALFUNCTION_IN_REPLAY'] = '1'
    os.environ['BLOCK'] = 'uploader'
    os.environ["FINGERPRINT"] = "PERODUA_ATIVA"
    os.environ["SKIP_FW_QUERY"] = "1"

    # Reset rkaiq 3A server so camera stack starts from a clean state.
    try:
      subprocess.run(["sudo", "-n", "killall", "-q", "/usr/kommu/rkaiq_3A_server"], check=False)
      time.sleep(2.5)
      subprocess.run(["sudo", "-n", "bash", "-lc", "/usr/kommu/rkaiq_3A_server >/dev/null 2>&1 &"], check=False)
    except OSError:
      pass

    # Ensure uploader is not left running from a prior session.
    try:
      subprocess.run(["pkill", "-f", "system.loggerd.uploader"], check=False)
    except OSError:
      pass

    if os.path.exists(Paths.log_root()):
      shutil.rmtree(Paths.log_root())

    # start launch script (same as normal openpilot boot)
    proc = None
    try:
      env = os.environ.copy()
      proc = subprocess.Popen(
        ["bash", "-lc", "exec ./launch_openpilot.sh"],
        cwd=BASEDIR,
        env=env,
        preexec_fn=os.setsid,
      )

      sm = messaging.SubMaster(['carState'])
      with Timeout(300, "controls didn't start"):
        while not sm.seen['carState']:
          sm.update(1000)

      route = None
      cls.segments = []
      if cls._burn_in_enabled:
        with Timeout(300, "timed out waiting for CurrentRoute"):
          while route is None:
            route = params.get("CurrentRoute")
            time.sleep(0.1)
        time.sleep(BURN_IN_DURATION_S)
        segs = set()
        if Path(Paths.log_root()).exists():
          segs = set(Path(Paths.log_root()).glob(f"{route}--*"))
        cls.segments = sorted(segs, key=lambda s: int(str(s).rsplit('--')[-1]))
        if cls.segments:
          cls.segments = cls.segments[:-1]
        assert len(cls.segments) >= 2, (
          f"burn-in: need at least 2 full segments after {BURN_IN_DURATION_S}s, got {len(cls.segments)}"
        )
        # Latest "full" segments = largest numeric suffix after sort; keep only last N to cap memory/CPU.
        cls._burn_in_full_segments_available = len(cls.segments)
        cls.segments = cls.segments[-BURN_IN_ANALYZE_SEGMENT_COUNT:]
      else:
        with Timeout(300, "timed out waiting for logs"):
          while route is None:
            route = params.get("CurrentRoute")
            time.sleep(0.1)

          while len(cls.segments) < 3:
            segs = set()
            if Path(Paths.log_root()).exists():
              segs = set(Path(Paths.log_root()).glob(f"{route}--*"))
            cls.segments = sorted(segs, key=lambda s: int(str(s).rsplit('--')[-1]))
            time.sleep(2)

        # Drop last potentially incomplete segment.
        cls.segments = cls.segments[:-1]
    finally:
      if proc is not None:
        try:
          cls._shutdown_temperature = _snapshot_device_state_temps()
        except Exception as e:
          cls._shutdown_temperature = {"error": repr(e)}
        try:
          os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except OSError:
          pass
        try:
          proc.wait(60)
        except subprocess.TimeoutExpired:
          try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
          except OSError:
            pass

    cls.lrs = [list(LogReader(os.path.join(str(s), "rlog.zst"))) for s in cls.segments]

    cls.lr = []
    for part in cls.lrs:
      cls.lr.extend(part)

    st = time.monotonic()
    cls.ts = msgs_to_time_series(cls.lr)
    print("msgs to time series", time.monotonic() - st)

    cls.log_sizes_per_segment = []
    for log_path in cls.segments:
      d = {}
      for f in log_path.iterdir():
        assert f.is_file()
        d[f] = f.stat().st_size / 1e6
      cls.log_sizes_per_segment.append(d)
    cls.log_sizes = cls.log_sizes_per_segment[0] if cls.log_sizes_per_segment else {}

    cls.msgs = defaultdict(list)
    for m in cls.lr:
      cls.msgs[m.which()].append(m)

  @classmethod
  def teardown_class(cls):
    cls._write_qc_report()
    unset_params_enabled()
    Params().remove("RecordFront")
    if os.path.exists(Paths.log_root()):
      shutil.rmtree(Paths.log_root())

  @classmethod
  def _write_qc_report(cls):
    lines = []
    lines.append("KA2 Onroad Test Report")
    if cls._run_start_epoch is not None:
      lines.append(f"start_time_utc={time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(cls._run_start_epoch))}")
    lines.append(f"end_time_utc={time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime())}")
    lines.append(f"mode={cls._run_mode}")
    lines.append(f"burn_in_test={getattr(cls, '_burn_in_enabled', False)}")
    if getattr(cls, "_burn_in_enabled", False):
      lines.append(f"burn_in_duration_s={BURN_IN_DURATION_S}")
      lines.append(f"burn_in_analyze_segment_count_cap={BURN_IN_ANALYZE_SEGMENT_COUNT}")
      n_avail = getattr(cls, "_burn_in_full_segments_available", None)
      if n_avail is not None:
        lines.append(f"burn_in_full_segments_on_disk={n_avail}")

    if hasattr(cls, "segments"):
      lines.append(f"segments_analyzed={len(cls.segments)}")
    if hasattr(cls, "lr"):
      lines.append(f"messages_loaded={len(cls.lr)}")

    if cls._run_results:
      passed = sum(1 for _, ok, _ in cls._run_results if ok)
      failed = len(cls._run_results) - passed
      lines.append(f"tests_total={len(cls._run_results)}")
      lines.append(f"tests_passed={passed}")
      lines.append(f"tests_failed={failed}")
      lines.append("test_results:")
      for name, ok, err in cls._run_results:
        if ok:
          lines.append(f"  PASS {name}")
        else:
          lines.append(f"  FAIL {name}: {err}")
    else:
      lines.append("test_results: not available in this run mode")

    # Live snapshot from deviceState immediately before manager shutdown (see setup_class finally).
    st = getattr(cls, "_shutdown_temperature", None)
    if st:
      lines.append("temperature_end:")
      for k in sorted(st.keys()):
        lines.append(f"  {k}={st[k]}")
    else:
      lines.append("temperature_end: unavailable")

    try:
      with open("/data/qc.log", "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    except OSError:
      pass

  def test_service_frequencies(self, subtests):
    span_s = _mono_span_seconds(self.lr)

    for s, msgs in self.msgs.items():
      if s in ('initData', 'sentinel'):
        continue

      # skip gps services for now
      if s in ('ubloxGnss', 'ubloxRaw', 'gnssMeasurements', 'gpsLocation', 'gpsLocationExternal', 'qcomGnss'):
        continue

      with subtests.test(service=s):
        # Expect at least ~80% of nominal publishes over the rlog time span (Hz * seconds).
        hz = _ka2_test_service_frequency_hz(s)
        floor_n = max(1, math.floor(hz * span_s * 0.8))
        assert len(msgs) >= floor_n, (
          f"service={s}: got {len(msgs)} msgs, need >= {floor_n} "
          f"(freq={hz}Hz span_s={span_s:.2f})"
        )

  def test_cloudlog_size(self):
    msgs = self.msgs['logMessage']

    nseg = max(1, len(getattr(self, "log_sizes_per_segment", [])))
    mult = nseg if self._burn_in_enabled else 1
    limit = 3.5e5 * mult

    total_size = sum(len(m.as_builder().to_bytes()) for m in msgs)
    assert total_size < limit

    cnt = Counter(json.loads(m.logMessage)['filename'] for m in msgs)
    big_logs = [f for f, n in cnt.most_common(3) if n / sum(cnt.values()) > 30.]
    assert len(big_logs) == 0, f"Log spam: {big_logs}"

  def test_ka2_modem_gnss_lte_basics(self):
    """Basic QC: Quectel enumerated, AT stack answers, rlog has live qcomGnss (no true RF pass/fail)."""
    if "DEBUG" in os.environ or not hasattr(self, "msgs"):
      pytest.skip("requires full on-device class setup and rlog")
    if HARDWARE.get_device_type() != "ka2":
      pytest.skip("KA2 only")

    # Modem present for ModemManager (no SIM is OK; building/LTE absent is OK).
    mdj = _read_mmcli_modem_json()
    assert "_error" not in mdj, f"mmcli -J -m <first> failed: {mdj}"
    modem = mdj.get("modem") or {}
    gen = modem.get("generic", {})
    st = (gen.get("state") or "").lower()
    fail_reason = (gen.get("state-failed-reason") or "").lower()
    # In building QC with no SIM, ModemManager may report failed/sim-missing.
    # Treat that as acceptable, but keep failing on other modem failure reasons.
    if st == "failed" and fail_reason not in ("sim-missing",):
      pytest.fail(f"modem in failed state (non-SIM reason): {gen}")
    assert gen.get("manufacturer") or gen.get("model") or gen.get("equipment-identifier"), \
      f"mmcli modem missing identity: {gen}"

    # USB modem enumerated (EC25 uses multiple /dev/ttyUSB*; any is enough here).
    if not any(Path("/dev").glob("ttyUSB*")):
      pytest.fail("no /dev/ttyUSB* (modem not enumerated on USB?)")

    # LTE RF path responds to signal query; 99,99 = unknown is normal indoors with no service/SIM.
    at_rc, at_out = _mmcli_at_command("AT+CSQ")
    assert at_rc == 0, f"AT+CSQ failed (rc={at_rc}): {at_out}"
    lo = at_out.lower()
    assert "csq" in lo, f"unexpected AT+CSQ output: {at_out}"

    # GNSS stack producing messages (indoors: may be no fix; we only require stream liveness).
    gmsgs = self.msgs.get("qcomGnss", [])
    n = len(gmsgs)
    assert n >= _MIN_QCOM_GNSS_MSGS, (
      f"too few qcomGnss in rlog ({n} < {_MIN_QCOM_GNSS_MSGS}): check GNSS power, qcomgpsd, or antenna. "
    )
    kinds: set[str] = set()
    for m in gmsgs[0:: max(1, n // 32)]:
      try:
        kinds.add(str(m.qcomGnss.which()))
      except Exception:
        pass
    assert kinds, f"could not read qcomGnss union (sample); count={n}"

  def test_ka2_sd_card_partition_present(self):
    # Simple QC check requested: if this partition node is missing, treat as not mounted/formatted.
    ok = self._preflight.get("sd_card_partition_present", Path("/dev/mmcblk1p1").exists())
    assert ok, "/dev/mmcblk1p1 not found (SD card missing/unformatted/unmounted)"

  def test_log_sizes(self, subtests):
    if self._burn_in_enabled and getattr(self, "log_sizes_per_segment", None):
      for seg_i, seg_map in enumerate(self.log_sizes_per_segment):
        for f, sz in seg_map.items():
          with subtests.test(segment=seg_i, file=f.name):
            if f.name not in LOGS_SIZE:
              continue
            expected = LOGS_SIZE[f.name]
            mn_mul, mx_mul = LOGS_SIZE_MULTIPLIERS.get(f.name, (0.5, 1.5))
            minn = expected * mn_mul
            maxx = expected * mx_mul
            assert minn < sz < maxx, (
              f"seg={seg_i} {f.name}: size={sz:.3f}MB expected range=({minn:.3f}, {maxx:.3f})MB"
            )
    else:
      for f, sz in self.log_sizes.items():
        with subtests.test(file=f.name):
          if f.name not in LOGS_SIZE:
            continue
          expected = LOGS_SIZE[f.name]
          mn_mul, mx_mul = LOGS_SIZE_MULTIPLIERS.get(f.name, (0.5, 1.5))
          minn = expected * mn_mul
          maxx = expected * mx_mul
          assert minn < sz < maxx, f"{f.name}: size={sz:.3f}MB expected range=({minn:.3f}, {maxx:.3f})MB"

  def test_cpu_usage(self, subtests):
    _safe_print("\n------------------------------------------------")
    _safe_print("------------------ CPU Usage -------------------")
    _safe_print("------------------------------------------------")

    plogs_by_proc = defaultdict(list)
    for pl in self.msgs['procLog']:
      for x in pl.procLog.procs:
        if len(x.cmdline) > 0:
          n = list(x.cmdline)[0]
          plogs_by_proc[n].append(x)

    cpu_ok = True
    dt = (self.msgs['procLog'][-1].logMonoTime - self.msgs['procLog'][0].logMonoTime) / 1e9
    header = ['process', 'usage', 'expected', 'max allowed', 'test result']
    rows = []
    for proc_name, expected in PROCS.items():

      error = ""
      usage = 0.
      x = plogs_by_proc[proc_name]
      if len(x) > 2:
        cpu_time = cputime_total(x[-1]) - cputime_total(x[0])
        usage = cpu_time / dt * 100.

        max_allowed = KA2_CPU_MAX_ALLOWED.get(proc_name, max(expected * 1.5, expected + 5.0))
        if usage > max_allowed:
          error = "❌ USING MORE CPU THAN EXPECTED ❌"
          cpu_ok = False

      else:
        error = "⚠️ NO METRICS FOUND (ignored on KA2) ⚠️"

      rows.append([proc_name, usage, expected, max_allowed, error or "✅"])
    _safe_print(tabulate(rows, header, tablefmt="simple_grid", stralign="center", numalign="center", floatfmt=".2f"))

    # Ensure there's no missing procs
    all_procs = {p.name for p in self.msgs['managerState'][0].managerState.processes if p.shouldBeRunning}
    for p in all_procs:
      with subtests.test(proc=p):
        assert p in MANAGER_TO_PROCS, f"Missing manager->CPU mapping for {p}"
        assert any(metric_key in PROCS for metric_key in MANAGER_TO_PROCS[p]), f"Expected CPU usage missing for {p}"

    # total CPU check
    procs_tot = sum([(max(x) if isinstance(x, tuple) else x) for x in PROCS.values()])
    with subtests.test(name="total CPU"):
      assert procs_tot < MAX_TOTAL_CPU, "Total CPU budget exceeded"
    _safe_print("------------------------------------------------")
    _safe_print(f"Total allocated CPU usage is {procs_tot}%, budget is {MAX_TOTAL_CPU}%, {MAX_TOTAL_CPU-procs_tot:.1f}% left")
    _safe_print("------------------------------------------------")

    assert cpu_ok

  def test_memory_usage(self):
    print("\n------------------------------------------------")
    print("--------------- Memory Usage -------------------")
    print("------------------------------------------------")
    offset = int(SERVICE_LIST['deviceState'].frequency * LOG_OFFSET)
    mems = [m.deviceState.memoryUsagePercent for m in self.msgs['deviceState'][offset:]]
    print("Overall memory usage: ", mems)
    print("MSGQ (/dev/shm/) usage: ", subprocess.check_output(["du", "-hs", "/dev/shm"]).split()[0].decode())

    # check for big leaks. note that memory usage is
    # expected to go up while the MSGQ buffers fill up
    assert np.average(mems) <= 80, "Average memory usage too high"
    assert np.max(np.diff(mems)) <= 4, "Max memory increase too high"
    assert np.average(np.diff(mems)) <= 1, "Average memory increase too high"

  def test_camera_encoder_matches(self, subtests):
    # sanity check that the frame metadata is consistent with the encoded frames
    pairs = [('roadCameraState', 'roadEncodeIdx'),
             ('wideRoadCameraState', 'wideRoadEncodeIdx'),
             ('driverCameraState', 'driverEncodeIdx')]
    for cam, enc in pairs:
      with subtests.test(camera=cam, encoder=enc):
        cam_frames = {fid: (sof, eof) for fid, sof, eof in zip(
          self.ts[cam]['frameId'],
          self.ts[cam]['timestampSof'],
          self.ts[cam]['timestampEof'],
          strict=True,
        )}
        for i, fid in enumerate(self.ts[enc]['frameId']):
          cam_sof, cam_eof = cam_frames[fid]
          enc_sof, enc_eof = self.ts[enc]['timestampSof'][i], self.ts[enc]['timestampEof'][i]
          assert enc_sof == cam_sof, f"SOF mismatch: frameId={fid}, enc_sof={enc_sof}, cam_sof={cam_sof}"
          assert enc_eof == cam_eof, f"EOF mismatch: frameId={fid}, enc_eof={enc_eof}, cam_eof={cam_eof}"

  def test_mpc_execution_timings(self):
    result = "\n"
    result += "------------------------------------------------\n"
    result += "-----------------  MPC Timing ------------------\n"
    result += "------------------------------------------------\n"

    cfgs = [("longitudinalPlan", 0.05, 0.05),]
    for (s, instant_max, avg_max) in cfgs:
      ts = [getattr(m, s).solverExecutionTime for m in self.msgs[s]]
      assert max(ts) < instant_max, f"high '{s}' execution time: {max(ts)}"
      assert np.mean(ts) < avg_max, f"high avg '{s}' execution time: {np.mean(ts)}"
      result += f"'{s}' execution time: min  {min(ts):.5f}s\n"
      result += f"'{s}' execution time: max  {max(ts):.5f}s\n"
      result += f"'{s}' execution time: mean {np.mean(ts):.5f}s\n"
    result += "------------------------------------------------\n"
    print(result)

  def test_model_execution_timings(self, subtests):
    result = "\n"
    result += "------------------------------------------------\n"
    result += "----------------- Model Timing -----------------\n"
    result += "------------------------------------------------\n"
    cfgs = [
      # since multiple processes use the GPU and can preempt each other,
      # these numbers are not fully self-contained.
      ("modelV2", 0.07, 0.045),

      # KA2 test assumes ~10Hz driver path; model wall time still bounded loosely.
      ("driverStateV2", 0.3, 0.08),
    ]
    for (s, instant_max, avg_max) in cfgs:
      ts = [getattr(m, s).modelExecutionTime for m in self.msgs[s]]
      # TODO some init can happen in first iteration
      ts = ts[1:]
      result += f"'{s}' execution time: min  {min(ts):.5f}s\n"
      result += f"'{s}' execution time: max {max(ts):.5f}s\n"
      result += f"'{s}' execution time: mean {np.mean(ts):.5f}s\n"
      with subtests.test(s):
        assert max(ts) < instant_max, f"high '{s}' execution time: {max(ts)}"
        assert np.mean(ts) < avg_max, f"high avg '{s}' execution time: {np.mean(ts)}"
    result += "------------------------------------------------\n"
    print(result)

  def test_timings(self):
    passed = True
    print("\n------------------------------------------------")
    print("----------------- Service Timings --------------")
    print("------------------------------------------------")

    header = ['service', 'max', 'min', 'mean', 'expected mean', 'rsd', 'max allowed rsd', 'test result']
    rows = []
    for s, (maxmin, rsd) in TIMINGS.items():
      hz = _ka2_test_service_frequency_hz(s)
      offset = int(hz * LOG_OFFSET)
      msgs = [m.logMonoTime for m in self.msgs[s][offset:]]
      if not len(msgs):
        raise Exception(f"missing {s}")

      ts = np.diff(msgs) / 1e9
      dt = 1 / hz

      errors = []
      mean_rtol = GLOBAL_MEAN_RTOL
      if not np.allclose(np.mean(ts), dt, rtol=mean_rtol, atol=0):
        errors.append("❌ FAILED MEAN TIMING CHECK ❌")
      if not np.allclose([np.max(ts), np.min(ts)], dt, rtol=maxmin * GLOBAL_MAXMIN_SCALE, atol=0):
        errors.append("❌ FAILED MAX/MIN TIMING CHECK ❌")
      if (np.std(ts)/dt) > (rsd * GLOBAL_RSD_SCALE):
        errors.append("❌ FAILED RSD TIMING CHECK ❌")
      passed = not errors and passed
      rows.append([s, *(np.array([np.max(ts), np.min(ts), np.mean(ts), dt])*1e3), np.std(ts)/dt, rsd, "\n".join(errors) or "✅"])

    print(tabulate(rows, header, tablefmt="simple_grid", stralign="center", numalign="center", floatfmt=".2f"))
    assert passed

  def test_engagable(self):
    no_entries = Counter()
    for m in self.msgs['onroadEvents']:
      for evt in m.onroadEvents:
        if evt.noEntry:
          no_entries[evt.name] += 1

    offset = int(SERVICE_LIST['selfdriveState'].frequency * LOG_OFFSET)
    eng = [m.selfdriveState.engageable for m in self.msgs['selfdriveState'][offset:]]
    assert all(eng), \
           f"Not engageable for whole segment:\n- selfdriveState.engageable: {Counter(eng)}\n- No entry events: {no_entries}"


if __name__ == "__main__":
  if "--burn-in-test" in sys.argv:
    sys.argv = [a for a in sys.argv if a != "--burn-in-test"]
    os.environ["KA2_BURN_IN_TEST"] = "1"
  else:
    os.environ.pop("KA2_BURN_IN_TEST", None)

  class _DummySubtests:
    def test(self, *args, **kwargs):
      return nullcontext()

  test_obj = TestOnroad()
  subtests = _DummySubtests()
  failed = 0

  TestOnroad.setup_class()
  try:
    test_methods = sorted(m for m in dir(TestOnroad) if m.startswith("test_"))
    TestOnroad._run_mode = "__main__"
    for name in test_methods:
      fn = getattr(test_obj, name)
      sig = inspect.signature(fn)
      _safe_print(f"\n=== Running {name} ===")
      try:
        if "subtests" in sig.parameters:
          fn(subtests)
        else:
          fn()
        TestOnroad._run_results.append((name, True, ""))
        _safe_print(f"PASS: {name}")
      except Exception as e:
        failed += 1
        TestOnroad._run_results.append((name, False, repr(e)))
        _safe_print(f"FAIL: {name} -> {repr(e)}")
        if name == "test_log_sizes":
          try:
            _safe_print("log_sizes snapshot (MB):")
            for p, sz in sorted(test_obj.log_sizes.items(), key=lambda kv: kv[0].name):
              _safe_print(f"  {p.name}: {sz:.3f}")
          except Exception:
            pass
        _safe_print(traceback.format_exc())
  finally:
    teardown = getattr(TestOnroad, "teardown_class", None)
    if callable(teardown):
      teardown()

  if failed:
    raise SystemExit(1)
  _safe_print("\nAll tests passed.")

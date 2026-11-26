from collections import defaultdict
import gc
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
import uuid

import pytest

import cereal.messaging as messaging
from cereal import log
from msgq.visionipc import VisionStreamType
from openpilot.common.params import Params
from openpilot.system.hardware import KA2
from openpilot.system.loggerd.tests.test_dmabuf_fixture import (
  _assert_camera_stack_idle,
  _send_frame,
  _test_environment,
  _wait_matching_line,
)
from openpilot.tools.lib.logreader import LogReader


LOGGERD_TARGET = "system/loggerd/loggerd"
ENCODERD_TARGET = "system/loggerd/encoderd"
PUBLISHER_TARGET = "build/system/loggerd/tests/visionipc_dmabuf_publisher"
LEGACY_PC_LOGGERD_TARGET = "build/system/loggerd/tests/loggerd_non_rk3588"
LEGACY_PC_ENCODERD_TARGET = "build/system/loggerd/tests/encoderd_non_rk3588"
FPS = 20
WIDTH = 1920
HEIGHT = 1200
FRAME_BYTES = WIDTH * HEIGHT * 3 // 2
FRAME_DURATION_NS = 1_000_000_000 // FPS
EXPECTED_FILES = {"rlog.zst", "qlog.zst", "qcamera.ts", "fcamera.hevc", "dcamera.hevc", "ecamera.hevc"}
INDEX_SERVICES = {
  "roadEncodeIdx": ("fcamera.hevc", 0, WIDTH, HEIGHT),
  "driverEncodeIdx": ("dcamera.hevc", 1, WIDTH, HEIGHT),
  "wideRoadEncodeIdx": ("ecamera.hevc", 2, WIDTH, HEIGHT),
  "qRoadEncodeIdx": ("qcamera.ts", 0, 526, 330),
}
CAMERAS = (
  (VisionStreamType.VISION_STREAM_ROAD, "roadCameraState"),
  (VisionStreamType.VISION_STREAM_DRIVER, "driverCameraState"),
  (VisionStreamType.VISION_STREAM_WIDE_ROAD, "wideRoadCameraState"),
)


def _build_test_binaries(repo_root: Path) -> dict[str, Path]:
  scons = shutil.which("scons")
  assert scons is not None, "SCons is required to build the isolated lifecycle targets"
  command = [
    scons, "-Q", "-j2", "--ccflags=-DLOGGERD_TEST_BUILD",
    LOGGERD_TARGET, ENCODERD_TARGET, PUBLISHER_TARGET,
  ]
  result = subprocess.run(command, cwd=repo_root, text=True, capture_output=True, check=False)
  print("KA2 test-build command:", " ".join(command))
  print(result.stdout, end="")
  print(result.stderr, end="")
  assert result.returncode == 0, f"test-build exited {result.returncode}"
  binaries = {
    "loggerd": repo_root / LOGGERD_TARGET,
    "encoderd": repo_root / ENCODERD_TARGET,
    "publisher": repo_root / PUBLISHER_TARGET,
  }
  assert all(path.is_file() for path in binaries.values()), f"missing test-build binaries: {binaries}"
  return binaries


def _build_legacy_pc_binaries(repo_root: Path) -> dict[str, Path]:
  scons = shutil.which("scons")
  assert scons is not None, "SCons is required to build the isolated non-RK3588 legacy test targets"
  command = [scons, "-Q", "--ccflags=-DLOGGERD_TEST_BUILD",
             LEGACY_PC_LOGGERD_TARGET, LEGACY_PC_ENCODERD_TARGET]
  build_env = os.environ.copy()
  build_env.pop("OPENPILOT_DEVICE", None)
  print("KA2-host non-RK3588/HardwarePC test-build command:", " ".join(command))
  print("hardware identity is untouched; dedicated SCons targets compile with -URK3588, without __TICI__")
  result = subprocess.run(command, cwd=repo_root, env=build_env, text=True, capture_output=True, check=False)
  print(result.stdout, end="")
  print(result.stderr, end="")
  assert result.returncode == 0, f"non-RK3588 legacy test-build exited {result.returncode}"
  binaries = {
    "loggerd": repo_root / LEGACY_PC_LOGGERD_TARGET,
    "encoderd": repo_root / LEGACY_PC_ENCODERD_TARGET,
  }
  assert all(path.is_file() for path in binaries.values()), f"missing non-RK3588 test binaries: {binaries}"
  return binaries


def _make_params_and_environment(tmp_path: Path, monkeypatch, fault_mode: str | None,
                                 legacy_mode: bool = False):
  env, prefix = _test_environment(tmp_path)
  env.update({
    "LOGGERD_TEST": "1",
    "LOGGERD_TEST_DETERMINISTIC": "0" if legacy_mode else "1",
    "LOGGERD_SEGMENT_LENGTH": "2",
    "ENCODERD_DETERMINISTIC_DISABLE": "1" if legacy_mode else "0",
    # This synthetic VisionIPC publisher exposes the public camera streams.
    # Real KA2 camerad/encoderd coverage uses the private leased DMA-BUF path.
    "LOGGERD_TEST_LEGACY_VIPC": "1",
  })
  if fault_mode is not None:
    env["LOGGERD_TEST_MPP_FAULT_STREAM"] = "roadEncodeData"
    env["LOGGERD_TEST_MPP_FAULT_MODE"] = fault_mode
  else:
    env.pop("LOGGERD_TEST_MPP_FAULT_STREAM", None)
    env.pop("LOGGERD_TEST_MPP_FAULT_MODE", None)

  for key, value in env.items():
    if key in ("LOG_ROOT", "PARAMS_ROOT", "OPENPILOT_PREFIX", "LOGGERD_TEST",
               "LOGGERD_TEST_DETERMINISTIC", "LOGGERD_SEGMENT_LENGTH",
               "LOGGERD_TEST_MPP_FAULT_STREAM", "LOGGERD_TEST_MPP_FAULT_MODE",
               "ENCODERD_DETERMINISTIC_DISABLE", "LOGGERD_TEST_LEGACY_VIPC"):
      if value is None:
        monkeypatch.delenv(key, raising=False)
      else:
        monkeypatch.setenv(key, value)

  params = Params(env["PARAMS_ROOT"])
  params.put_bool("RecordFront", True)
  params.put_bool("RecordAudio", True)
  msgq_dir = Path("/dev/shm") / f"msgq_{prefix}"
  assert not msgq_dir.exists(), f"isolated MSGQ directory already exists: {msgq_dir}"
  msgq_dir.mkdir(mode=0o700)
  return env, prefix, params, msgq_dir


def _read_param(params: Params, key: str, timeout: float = 1.0) -> str | None:
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    value = params.get(key)
    if value is not None:
      return value.decode() if isinstance(value, bytes) else str(value)
    time.sleep(0.01)
  return None


def _read_raw_param(params: Params, key: str, timeout: float = 1.0) -> str | None:
  """Read test-only Params written by C++ but not registered in Python's key table."""
  path = Path(params.get_param_path(key))
  deadline = time.monotonic() + timeout
  while True:
    try:
      value = path.read_text()
      if value:
        return value
    except FileNotFoundError:
      pass
    if time.monotonic() >= deadline:
      return None
    time.sleep(0.01)


def _wait_process_started(proc: subprocess.Popen, label: str) -> None:
  assert proc.poll() is None, f"{label} exited before the lifecycle started (returncode={proc.returncode})"


def _stop_process(proc: subprocess.Popen | None, name: str, timeout: float = 8.0) -> None:
  if proc is None or proc.poll() is not None:
    return
  try:
    proc.send_signal(signal.SIGINT)
    proc.wait(timeout=timeout)
  except subprocess.TimeoutExpired:
    proc.kill()
    proc.wait(timeout=3.0)
  assert proc.poll() is not None, f"test-owned {name} process did not stop"


def _ffprobe(path: Path) -> dict:
  command = [
    "ffprobe", "-v", "error", "-show_entries", "stream=codec_name,codec_type,width,height",
    "-of", "json", str(path),
  ]
  result = subprocess.run(command, text=True, capture_output=True, check=False, timeout=20)
  assert result.returncode == 0, f"ffprobe failed for {path}: {result.stderr}"
  return json.loads(result.stdout)


def _decode_video_and_check_pixels(path: Path, codec: str, width: int, height: int,
                                   frame_ids: list[int], stream_value: int) -> None:
  ffmpeg = shutil.which("ffmpeg")
  assert ffmpeg is not None, "ffmpeg is required to decode generated video for the DMA-BUF lifecycle assertion"
  probe = _ffprobe(path)
  video = [stream for stream in probe["streams"] if stream["codec_type"] == "video"]
  assert len(video) == 1
  assert video[0]["codec_name"] == codec
  assert (video[0]["width"], video[0]["height"]) == (width, height)

  command = [ffmpeg, "-v", "error", "-i", str(path), "-map", "0:v:0", "-f", "rawvideo", "-pix_fmt", "yuv420p", "-"]
  process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  assert process.stdout is not None
  frame_size = width * height * 3 // 2
  decoded = 0
  try:
    for frame_id in frame_ids:
      raw = process.stdout.read(frame_size)
      assert len(raw) == frame_size, f"decoded frame count ended before EncodeIdx frame {frame_id} in {path}"
      center_y = raw[(height // 2) * width + (width // 2)]
      expected_y = 16 + ((frame_id * 13 + stream_value * 29) % 200)
      assert abs(center_y - expected_y) <= 15, (
        f"decoded pixel does not match DMA-BUF frame identity {frame_id} in {path}: "
        f"expected Y={expected_y}, observed Y={center_y}"
      )
      decoded += 1
    extra = process.stdout.read(1)
    assert extra == b"", f"decoder emitted more frames than EncodeIdx lists for {path}"
    returncode = process.wait(timeout=15)
    stderr = process.stderr.read().decode(errors="replace") if process.stderr is not None else ""
    assert returncode == 0, f"ffmpeg decode failed for {path}: {stderr}"
  finally:
    if process.poll() is None:
      process.kill()
      process.wait(timeout=3)
    process.stdout.close()
    if process.stderr is not None:
      process.stderr.close()
  assert decoded == len(frame_ids)


def _validate_segments(log_root: Path, params: Params, reports: dict[int, dict], last_frame_id: int,
                       scenario: str, encoderd_log: str) -> None:
  segment_dirs = sorted(log_root.glob("*--*"), key=lambda path: int(path.name.rsplit("--", 1)[1]))
  segments = [int(path.name.rsplit("--", 1)[1]) for path in segment_dirs]
  assert segments == [0, 1, 2], f"expected two runtime rotations, found {segments}"

  frame_ids_by_segment: dict[int, dict[str, list[int]]] = defaultdict(lambda: defaultdict(list))
  for segment_num, segment_dir in enumerate(segment_dirs):
    expected_start = log.Sentinel.SentinelType.startOfRoute if segment_num == 0 else log.Sentinel.SentinelType.startOfSegment
    expected_end = log.Sentinel.SentinelType.endOfRoute if segment_num == 2 else log.Sentinel.SentinelType.endOfSegment
    files = {path.name for path in segment_dir.iterdir() if path.is_file()}
    if scenario in ("valid", "legacy", "legacy_non_rk"):
      assert files == EXPECTED_FILES, f"segment {segment_num} file set changed: {files ^ EXPECTED_FILES}"
    elif scenario == "shm":
      expected_shm_files = {"rlog.zst", "qlog.zst", "qcamera.ts"}
      assert files == expected_shm_files, (
        f"SHM full-resolution rejects must not create fabricated video files: {files ^ expected_shm_files}"
      )
    else:
      assert "qcamera.ts" in files and "rlog.zst" in files and "qlog.zst" in files, (
        f"failed sibling scenario lost qRoad or route logs in segment {segment_num}: {files}"
      )
      if scenario in ("reject", "silent"):
        assert "fcamera.hevc" not in files, "permanently unavailable road output unexpectedly created a video file"

    rlog = list(LogReader(str(segment_dir / "rlog.zst")))
    assert rlog[0].which() == "initData"
    if scenario == "legacy_non_rk":
      assert rlog[0].initData.deviceType == log.InitData.DeviceType.pc, (
        "legacy lifecycle must report the HardwarePC software branch while the actual host remains KA2"
      )
    assert any(message.which() == "rawAudioData" for message in rlog)
    assert rlog[1].sentinel.type == expected_start
    assert rlog[-1].sentinel.type == expected_end

    qlog = list(LogReader(str(segment_dir / "qlog.zst")))
    assert qlog[0].which() == "initData"
    assert qlog[1].sentinel.type == expected_start
    assert qlog[-1].sentinel.type == expected_end
    for message in qlog:
      service = message.which()
      if service not in INDEX_SERVICES:
        continue
      index = getattr(message, service)
      assert index.segmentNum == segment_num, f"cross-segment EncodeIdx in {segment_dir}: {service}={index.segmentNum}"
      assert 1 <= index.frameId <= last_frame_id
      frame_ids_by_segment[segment_num][service].append(index.frameId)

    assert "qRoadEncodeIdx" in frame_ids_by_segment[segment_num], f"qRoad emitted no indexes in segment {segment_num}"
    qcamera = segment_dir / "qcamera.ts"
    media = _ffprobe(qcamera)
    assert any(stream["codec_type"] == "video" and stream["codec_name"] == "h264" for stream in media["streams"])
    assert any(stream["codec_type"] == "audio" for stream in media["streams"]), "qRoad lost its audio track"
    decoded_audio = subprocess.run(
      ["ffmpeg", "-v", "error", "-i", str(qcamera), "-map", "0:a:0", "-f", "s16le", "-acodec", "pcm_s16le", "-"],
      capture_output=True, check=False, timeout=20,
    )
    assert decoded_audio.returncode == 0 and len(decoded_audio.stdout) > 0, f"qRoad audio decode failed: {decoded_audio.stderr!r}"

    if scenario in ("valid", "legacy", "legacy_non_rk"):
      for service, (filename, stream_value, width, height) in INDEX_SERVICES.items():
        frame_ids = frame_ids_by_segment[segment_num][service]
        assert frame_ids, f"{service} emitted no packets in segment {segment_num}"
        assert len(frame_ids) == len(set(frame_ids)), f"duplicate {service} frame ids in segment {segment_num}"
        if service != "qRoadEncodeIdx":
          assert frame_ids == sorted(frame_ids), f"out-of-order {service} identities"
        codec = "h264" if service == "qRoadEncodeIdx" else "ffvhuff" if scenario == "legacy_non_rk" else "hevc"
        _decode_video_and_check_pixels(segment_dir / filename, codec,
                                       width, height, frame_ids, stream_value)
    elif scenario == "shm":
      qroad_ids = frame_ids_by_segment[segment_num]["qRoadEncodeIdx"]
      assert qroad_ids == sorted(qroad_ids) and len(qroad_ids) == len(set(qroad_ids))
      assert not any(frame_ids_by_segment[segment_num].get(service)
                     for service in ("roadEncodeIdx", "driverEncodeIdx", "wideRoadEncodeIdx")), (
        f"full-resolution SHM rejection fabricated an encoded index: {frame_ids_by_segment[segment_num]}"
      )
      _decode_video_and_check_pixels(segment_dir / "qcamera.ts", "h264", 526, 330, qroad_ids, 0)

  qroad_counters = []
  for line in encoderd_log.splitlines():
    marker = "LOGGERD_TEST_MPP_COUNTERS "
    if marker not in line:
      continue
    fields = dict(part.split("=", 1) for part in line.split(marker, 1)[1].split() if "=" in part)
    if fields.get("stream") != "qRoadEncodeData":
      continue
    qroad_counters.append((
      int(fields["cur_seg"]),
      int(fields["segment_transition_reopen_count"]),
      int(fields["failure_recovery_reopen_count"]),
      int(fields["failure_retry_count"]),
    ))
  if scenario in ("legacy", "legacy_non_rk"):
    assert not qroad_counters, f"legacy rotation must not emit deterministic MPP counter snapshots: {qroad_counters}"
  else:
    expected_counters = [(1, 1, 0, 0), (2, 2, 0, 0)]
    assert qroad_counters == expected_counters, (
      "qRoad must add exactly one planned transition reopen per boundary without failure recovery/retry; "
      f"expected (segment, planned, recovery, retry)={expected_counters}, observed={qroad_counters}"
    )
  if scenario == "drain_timeout":
    drain_lines = [line for line in encoderd_log.splitlines()
                   if "shared 900 ms drain deadline expired" in line and "roadEncodeData" in line]
    assert drain_lines, "drain_timeout arm did not exercise the shared 900 ms road-output drain deadline"
    all_reopen_records = {}
    marker = "LOGGERD_TEST_ALL_OUTPUT_REOPEN_COUNT "
    for line in encoderd_log.splitlines():
      if marker not in line:
        continue
      fields = dict(part.split("=", 1) for part in line.split(marker, 1)[1].split() if "=" in part)
      thread = fields.get("thread")
      assert thread and thread not in all_reopen_records, f"missing/duplicate all-output reopen record: {line}"
      all_reopen_records[thread] = int(fields["count"])
    expected_threads = {"road_cam_encoder", "driver_cam_encoder", "wide_road_cam_encoder"}
    assert all_reopen_records == {thread: 0 for thread in expected_threads}, (
      "drain timeout must not invoke the all-output recovery helper in any encoder thread; "
      f"expected one zero-count record per thread, observed={all_reopen_records}"
    )

  if scenario == "shm":
    for stream in ("roadEncodeData", "wideRoadEncodeData", "driverEncodeData"):
      assert any("refusing non-DMA-BUF input fd" in line and f"for {stream}" in line
                 for line in encoderd_log.splitlines()), (
        f"full-resolution MPP did not log non-DMA-BUF rejection for {stream}"
      )
    assert "for qRoadEncodeData" not in encoderd_log, "qRoad must use RGA resize without full-resolution DMA-BUF import"

  if scenario in ("legacy", "legacy_non_rk"):
    assert not reports, f"legacy mode must not publish deterministic rotation reports: {reports}"
    assert _read_raw_param(params, "LoggerdRotationReport") is None, "legacy mode published a deterministic report"
    assert "shared 900 ms drain deadline expired" not in encoderd_log
    assert "packet progress watchdog" not in encoderd_log
    assert "retry in " not in encoderd_log
    assert "LOGGERD_TEST_MPP_COUNTERS" not in encoderd_log
    return

  current_route = _read_param(params, "CurrentRoute")
  assert current_route
  for segment in (1, 2):
    report = reports.get(segment)
    assert report is not None, f"missing LoggerdRotationReport for segment {segment}"
    assert set(report) == {"route", "epoch", "segment", "degraded", "missing_streams", "elapsed_ms", "known_drop_counts"}
    assert report["route"] == current_route
    assert report["epoch"] == segment + 1 and report["segment"] == segment
    assert type(report["degraded"]) is bool
    assert type(report["elapsed_ms"]) is int and 0 <= report["elapsed_ms"] <= 5050
    assert set(report["known_drop_counts"]) == {
      "stale_video_messages", "future_video_queue_drops", "audio_no_writer_events",
      "audio_expired_events", "audio_capacity_drops",
    }
    assert all(type(value) is int and value >= 0 for value in report["known_drop_counts"].values())
    assert type(report["missing_streams"]) is list
    assert len(report["missing_streams"]) == len(set(report["missing_streams"]))
    configured_order = ["roadEncodeData", "qRoadEncodeData", "wideRoadEncodeData", "driverEncodeData"]
    assert report["missing_streams"] == [name for name in configured_order if name in report["missing_streams"]]
    if scenario == "valid":
      assert report["missing_streams"] == [] and report["degraded"] is False
      assert all(value == 0 for value in report["known_drop_counts"].values())
    elif scenario == "shm":
      expected_missing = ["roadEncodeData", "wideRoadEncodeData", "driverEncodeData"]
      assert report["missing_streams"] == expected_missing and report["degraded"] is True
      assert all(value == 0 for value in report["known_drop_counts"].values())
    else:
      assert "qRoadEncodeData" not in report["missing_streams"], "qRoad readiness must survive a sibling fault"
      assert all(value == 0 for value in report["known_drop_counts"].values()), (
        "encoder-only rejects/stalls must not increment loggerd-observed drop counters"
      )
      if scenario in ("reject", "silent"):
        assert "roadEncodeData" in report["missing_streams"] and report["degraded"] is True
      else:
        assert report["degraded"] is bool(report["missing_streams"] or any(report["known_drop_counts"].values()))


def _run_lifecycle(tmp_path: Path, monkeypatch, binaries: dict[str, Path], scenario: str) -> None:
  if not KA2:
    pytest.skip("full MPP lifecycle requires KA2")

  _assert_camera_stack_idle()
  fault_mode = None if scenario in ("valid", "legacy", "legacy_non_rk", "shm") else scenario
  env, prefix, params, msgq_dir = _make_params_and_environment(
    tmp_path, monkeypatch, fault_mode, legacy_mode=(scenario in ("legacy", "legacy_non_rk")))
  repo_root = Path(__file__).resolve().parents[3]
  fixture_binary = binaries["publisher"]
  assert fixture_binary.is_file()
  vipc_socket = Path(f"/tmp/{prefix}_visionipc_camerad")
  assert not vipc_socket.exists(), f"isolated VisionIPC socket already exists: {vipc_socket}"

  publisher = None
  loggerd = None
  encoderd = None
  pm = None
  publisher_stdout = None
  loggerd_output = None
  encoderd_output = None
  loggerd_log_path = Path(env["LOG_ROOT"]).parent / "loggerd.stdout.log"
  encoderd_log_path = Path(env["LOG_ROOT"]).parent / "encoderd.stdout.log"
  reports: dict[int, dict] = {}
  observed_segment_times: dict[int, float] = {}
  last_frame_id = 0
  try:
    _assert_camera_stack_idle()
    publisher_mode = "--serve-shm" if scenario == "shm" else "--serve"
    publisher = subprocess.Popen([str(fixture_binary), publisher_mode], cwd=repo_root, env=env,
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, bufsize=1)
    backing = "shm" if scenario == "shm" else "dma_heap"
    _wait_matching_line(
      publisher,
      f"READY streams=road,driver,wideRoad width=1920 height=1200 stride=1920 "
      f"uv_offset=2304000 buffers=4 backing={backing}",
      10,
    )
    pm = messaging.PubMaster([state for _, state in CAMERAS] + ["rawAudioData"])

    loggerd_output = loggerd_log_path.open("w")
    encoderd_output = encoderd_log_path.open("w")
    loggerd = subprocess.Popen([str(binaries["loggerd"])], cwd=repo_root, env=env,
                               stdout=loggerd_output, stderr=subprocess.STDOUT, text=True)
    encoderd = subprocess.Popen([str(binaries["encoderd"])], cwd=repo_root, env=env,
                                stdout=encoderd_output, stderr=subprocess.STDOUT, text=True)

    for _, state_name in CAMERAS:
      assert pm.wait_for_readers_to_update(state_name, timeout=10), f"loggerd/encoderd did not subscribe to {state_name}"
    assert pm.wait_for_readers_to_update("rawAudioData", timeout=10), "loggerd did not subscribe to rawAudioData"
    assert _read_param(params, "LoggerdSegment", timeout=10) == "0"
    observed_segment_times[0] = time.monotonic()

    frame = time.monotonic()
    audio_samples = (b"\xe8\x03\x18\xfc" * 400)
    post_segment_two = 0
    deadline = time.monotonic() + 24
    while time.monotonic() < deadline:
      last_frame_id += 1
      target = frame + last_frame_id / FPS
      delay = target - time.monotonic()
      if delay > 0:
        time.sleep(delay)
      timestamp = last_frame_id * FRAME_DURATION_NS
      for stream_type, state_name in CAMERAS:
        _send_frame(publisher, int(stream_type), last_frame_id, timestamp, timestamp + FRAME_DURATION_NS // 2)
        camera_state = messaging.new_message(state_name)
        getattr(camera_state, state_name).frameId = last_frame_id
        pm.send(state_name, camera_state)
      audio = messaging.new_message("rawAudioData")
      audio.rawAudioData.data = audio_samples
      audio.rawAudioData.sampleRate = 16000
      pm.send("rawAudioData", audio)

      segment_value = _read_param(params, "LoggerdSegment", timeout=0.1)
      if segment_value is not None:
        segment = int(segment_value)
        observed_segment_times.setdefault(segment, time.monotonic())
        if segment >= 1:
          report_text = _read_raw_param(params, "LoggerdRotationReport", timeout=0.05)
          if report_text:
            try:
              report = json.loads(report_text)
              if report.get("segment") == segment:
                reports[segment] = report
            except json.JSONDecodeError:
              pass
        if segment >= 2:
          post_segment_two += 1
          if post_segment_two >= 20:
            break

      _wait_process_started(loggerd, "loggerd")
      _wait_process_started(encoderd, "encoderd")
      _wait_process_started(publisher, "SHM publisher" if scenario == "shm" else "DMA-BUF publisher")
    else:
      raise AssertionError(f"loggerd did not complete two rotations; reports={reports}")

    assert post_segment_two >= 20, "did not keep sending frames after the second boundary"
    if scenario in ("legacy", "legacy_non_rk"):
      assert not reports, f"legacy mode must not publish LoggerdRotationReport: {reports}"
      assert _read_raw_param(params, "LoggerdRotationReport") is None
      intervals = [observed_segment_times[n] - observed_segment_times[n - 1] for n in (1, 2)]
      assert all(interval < 6.5 for interval in intervals), (
        f"legacy all-ready rotation waited for the deterministic deadline: intervals={intervals}"
      )
    else:
      assert len(reports) == 2, f"did not observe both segment reports: {reports}"
  finally:
    pm = None
    gc.collect()
    _stop_process(encoderd, "encoderd")
    _stop_process(loggerd, "loggerd")
    if publisher is not None and publisher.poll() is None:
      try:
        assert publisher.stdin is not None
        publisher.stdin.write("STOP\n")
        publisher.stdin.flush()
        _wait_matching_line(publisher, "CLEANUP_OK", 10)
        publisher.wait(timeout=10)
      except Exception:
        publisher.kill()
        publisher.wait(timeout=3)
    if publisher is not None:
      if publisher.stdout is not None:
        publisher_stdout = publisher.stdout.read()
        publisher.stdout.close()
      if publisher.stdin is not None and not publisher.stdin.closed:
        publisher.stdin.close()
    for handle in (loggerd_output, encoderd_output):
      if handle is not None:
        handle.close()
    loggerd_log = loggerd_log_path.read_text(errors="replace") if loggerd_log_path.exists() else ""
    encoderd_log = encoderd_log_path.read_text(errors="replace") if encoderd_log_path.exists() else ""
    print("publisher stdout after command reads:\n", publisher_stdout or "(no remaining output)")
    print("complete loggerd stdout:\n", loggerd_log or "(empty)")
    print("complete encoderd stdout:\n", encoderd_log or "(empty)")
    param_snapshot = {
      key: _read_param(params, key, timeout=0.05)
      for key in ("CurrentRoute", "LoggerdSegment", "LoggerdSegmentEpoch", "LoggerdRotateState")
    }
    param_snapshot["LoggerdRotationReport"] = _read_raw_param(params, "LoggerdRotationReport", timeout=0.05)
    print("isolated Params at cleanup:", json.dumps(param_snapshot, sort_keys=True))
    log_root = Path(env["LOG_ROOT"])
    segment_files = [
      (segment.name, sorted((item.name, item.stat().st_size) for item in segment.iterdir() if item.is_file()))
      for segment in sorted(log_root.glob("*--*"))
    ] if log_root.exists() else []
    print("isolated segment files at cleanup:", json.dumps(segment_files, sort_keys=True))
    assert not vipc_socket.exists(), f"test publisher left VisionIPC socket {vipc_socket}"
    shutil.rmtree(msgq_dir, ignore_errors=True)

  _validate_segments(Path(env["LOG_ROOT"]), params, reports, last_frame_id, scenario, encoderd_log)


def _run_lifecycle_and_cleanup(tmp_path: Path, monkeypatch, binaries: dict[str, Path], scenario: str) -> None:
  try:
    _run_lifecycle(tmp_path, monkeypatch, binaries, scenario)
  finally:
    shutil.rmtree(tmp_path, ignore_errors=True)


@pytest.mark.xdist_group("loggerd_tests")
def test_ka2_valid_dmabuf_rotation_end_to_end(tmp_path, monkeypatch):
  if not KA2:
    pytest.skip("full deterministic KA2 MPP lifecycle requires KA2")
  binaries = _build_test_binaries(Path(__file__).resolve().parents[3])
  _run_lifecycle_and_cleanup(tmp_path / "valid", monkeypatch, binaries, "valid")


@pytest.mark.parametrize("fault_mode", ["reject", "silent", "drain_timeout"])
@pytest.mark.xdist_group("loggerd_tests")
def test_ka2_output_fault_preserves_qroad(tmp_path, monkeypatch, fault_mode):
  if not KA2:
    pytest.skip("per-output MPP failure isolation requires KA2")
  binaries = _build_test_binaries(Path(__file__).resolve().parents[3])
  _run_lifecycle_and_cleanup(tmp_path / fault_mode, monkeypatch, binaries, fault_mode)


@pytest.mark.xdist_group("loggerd_tests")
def test_ka2_shm_rejection_preserves_qroad(tmp_path, monkeypatch):
  if not KA2:
    pytest.skip("full SHM RGA/MPP lifecycle requires KA2")
  binaries = _build_test_binaries(Path(__file__).resolve().parents[3])
  _run_lifecycle_and_cleanup(tmp_path / "shm", monkeypatch, binaries, "shm")


@pytest.mark.xdist_group("loggerd_tests")
def test_loggerd_legacy_mode_preserves_rotation_behavior(tmp_path, monkeypatch):
  if not KA2:
    pytest.skip("isolated RK3588 deterministic-disable compatibility run requires KA2")
  binaries = _build_test_binaries(Path(__file__).resolve().parents[3])
  _run_lifecycle_and_cleanup(tmp_path / "legacy", monkeypatch, binaries, "legacy")


@pytest.mark.xdist_group("loggerd_tests")
def test_non_rk3588_hardware_pc_legacy_lifecycle_on_ka2_host(tmp_path, monkeypatch):
  if not KA2:
    pytest.skip("isolated non-RK3588 software-branch qualification requires the pinned KA2 host")
  repo_root = Path(__file__).resolve().parents[3]
  publisher = _build_test_binaries(repo_root)["publisher"]
  legacy_binaries = _build_legacy_pc_binaries(repo_root)
  binaries = {**legacy_binaries, "publisher": publisher}
  _run_lifecycle_and_cleanup(tmp_path / "legacy_non_rk", monkeypatch, binaries, "legacy_non_rk")

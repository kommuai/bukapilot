import json
import numpy as np
import os
import re
import random
import string
import subprocess
import time
from collections import Counter, defaultdict
from pathlib import Path
import pytest

import cereal.messaging as messaging
from cereal import log
from cereal.services import SERVICE_LIST
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.timeout import Timeout
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.xattr_cache import getxattr
from openpilot.system.loggerd.deleter import PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.version import get_version
from openpilot.tools.lib.helpers import RE
from openpilot.tools.lib.logreader import LogReader
from msgq.visionipc import VisionIpcServer, VisionStreamType

SentinelType = log.Sentinel.SentinelType

CEREAL_SERVICES = [f for f in log.Event.schema.union_fields if f in SERVICE_LIST
                   and SERVICE_LIST[f].should_log and "encode" not in f.lower()]


@pytest.mark.xdist_group("loggerd_tests")
class TestLoggerd:
  def _get_latest_log_dir(self):
    log_dirs = sorted(Path(Paths.log_root()).iterdir(), key=lambda f: f.stat().st_mtime)
    return log_dirs[-1]

  def _get_log_dir(self, x):
    for l in x.splitlines():
      for p in l.split(' '):
        path = Path(p.strip())
        if path.is_dir():
          return path
    return None

  def _get_log_fn(self, x):
    for l in x.splitlines():
      for p in l.split(' '):
        path = Path(p.strip())
        if path.is_file():
          return path
    return None

  def _gen_bootlog(self):
    with Timeout(5):
      out = subprocess.check_output("./bootlog", cwd=os.path.join(BASEDIR, "system/loggerd"), encoding='utf-8')

    log_fn = self._get_log_fn(out)

    # check existence
    assert log_fn is not None

    return log_fn

  def _check_init_data(self, msgs):
    msg = msgs[0]
    assert msg.which() == 'initData'

  def _check_sentinel(self, msgs, route):
    start_type = SentinelType.startOfRoute if route else SentinelType.startOfSegment
    assert msgs[1].sentinel.type == start_type

    end_type = SentinelType.endOfRoute if route else SentinelType.endOfSegment
    assert msgs[-1].sentinel.type == end_type

  def _publish_random_messages(self, services: list[str]) -> dict[str, list]:
    pm = messaging.PubMaster(services)

    managed_processes["loggerd"].start()
    for s in services:
      assert pm.wait_for_readers_to_update(s, timeout=5)

    sent_msgs = defaultdict(list)
    for _ in range(random.randint(2, 10) * 100):
      for s in services:
        try:
          m = messaging.new_message(s)
        except Exception:
          m = messaging.new_message(s, random.randint(2, 10))
        pm.send(s, m)
        sent_msgs[s].append(m)

    for s in services:
      assert pm.wait_for_readers_to_update(s, timeout=5)
    managed_processes["loggerd"].stop()

    return sent_msgs

  def _publish_camera_and_audio_messages(self, num_segs=1, segment_length=5):
    # Use small frame sizes for testing (width, height, size, stride, uv_offset)
    # NV12 format: size = stride * height * 1.5, uv_offset = stride * height
    w, h = 320, 240
    frame_spec = (w, h, w * h * 3 // 2, w, w * h)
    streams = [
      (VisionStreamType.VISION_STREAM_ROAD, frame_spec, "roadCameraState"),
      (VisionStreamType.VISION_STREAM_DRIVER, frame_spec, "driverCameraState"),
      (VisionStreamType.VISION_STREAM_WIDE_ROAD, frame_spec, "wideRoadCameraState"),
    ]

    sm = messaging.SubMaster(["roadEncodeData"])
    pm = messaging.PubMaster([s for _, _, s in streams] + ["rawAudioData"])
    vipc_server = VisionIpcServer("camerad")
    for stream_type, frame_spec, _ in streams:
      vipc_server.create_buffers_with_sizes(stream_type, 40, *(frame_spec))
    vipc_server.start_listener()

    os.environ["LOGGERD_TEST"] = "1"
    os.environ["LOGGERD_SEGMENT_LENGTH"] = str(segment_length)
    managed_processes["loggerd"].start()
    managed_processes["encoderd"].start()
    assert pm.wait_for_readers_to_update("roadCameraState", timeout=5)

    fps = 20
    for n in range(1, int(num_segs * segment_length * fps) + 1):
      # send video
      for stream_type, frame_spec, state in streams:
        dat = np.empty(frame_spec[2], dtype=np.uint8)
        vipc_server.send(stream_type, dat[:].flatten().tobytes(), n, n / fps, n / fps)

        camera_state = messaging.new_message(state)
        frame = getattr(camera_state, state)
        frame.frameId = n
        pm.send(state, camera_state)

      # send audio
      msg = messaging.new_message('rawAudioData')
      msg.rawAudioData.data = bytes(800 * 2) # 800 samples of int16
      msg.rawAudioData.sampleRate = 16000
      pm.send('rawAudioData', msg)

      for _, _, state in streams:
        assert pm.wait_for_readers_to_update(state, timeout=5, dt=0.001)

      sm.update(100)  # wait for encode data publish

    managed_processes["loggerd"].stop()
    managed_processes["encoderd"].stop()

  def test_init_data_values(self):
    os.environ["CLEAN"] = random.choice(["0", "1"])

    dongle  = ''.join(random.choice(string.printable) for n in range(random.randint(1, 100)))
    fake_params = [
      # param, initData field, value
      ("DongleId", "dongleId", dongle),
      ("GitCommit", "gitCommit", "commit"),
      ("GitCommitDate", "gitCommitDate", "date"),
      ("GitBranch", "gitBranch", "branch"),
      ("GitRemote", "gitRemote", "remote"),
    ]
    params = Params()
    for k, _, v in fake_params:
      params.put(k, v)
    params.put("AccessToken", "abc")

    lr = list(LogReader(str(self._gen_bootlog())))
    initData = lr[0].initData

    assert initData.dirty != bool(os.environ["CLEAN"])
    assert initData.version == get_version()

    if os.path.isfile("/proc/cmdline"):
      with open("/proc/cmdline") as f:
        # Match logger parsing semantics (operator>> collapses repeated whitespace).
        assert list(initData.kernelArgs) == f.read().strip().split()

      with open("/proc/version") as f:
        assert initData.kernelVersion == f.read()

    # check params
    logged_params = {entry.key: entry.value for entry in initData.params.entries}
    expected_params = {k for k, _, __ in fake_params} | {'AccessToken', 'BootCount'}
    assert set(logged_params.keys()) == expected_params, set(logged_params.keys()) ^ expected_params
    assert logged_params['AccessToken'] == b'', f"DONT_LOG param value was logged: {repr(logged_params['AccessToken'])}"
    for param_key, initData_key, v in fake_params:
      assert getattr(initData, initData_key) == v
      assert logged_params[param_key].decode() == v

  def test_rotation(self):
    Params().put("RecordFront", True)

    expected_files = {"rlog.zst", "qlog.zst", "qcamera.ts", "fcamera.hevc", "dcamera.hevc", "ecamera.hevc"}

    num_segs = random.randint(2, 3)
    length = random.randint(4, 5) # H264 encoder uses 40 lookahead frames and does B-frame reordering, so minimum 3 seconds before qcam output

    self._publish_camera_and_audio_messages(num_segs=num_segs, segment_length=length)

    route_path = str(self._get_latest_log_dir()).rsplit("--", 1)[0]
    for n in range(num_segs):
      p = Path(f"{route_path}--{n}")
      logged = {f.name for f in p.iterdir() if f.is_file()}
      diff = logged ^ expected_files
      assert len(diff) == 0, f"didn't get all expected files. seg={n} {route_path=}, {diff=}\n{logged=} {expected_files=}"

  def test_ka2_deterministic_rotation_alignment(self, tmp_path, monkeypatch):
    loggerd = managed_processes["loggerd"]
    encoderd = managed_processes["encoderd"]
    assert loggerd.proc is None and encoderd.proc is None, "loggerd test processes must not already be running"

    log_root = tmp_path / "logs"
    params_root = tmp_path / "params"
    segment_length = 5
    fps = 20
    monkeypatch.setenv("LOG_ROOT", str(log_root))
    monkeypatch.setenv("PARAMS_ROOT", str(params_root))
    monkeypatch.setenv("LOGGERD_TEST", "1")
    monkeypatch.setenv("LOGGERD_TEST_DETERMINISTIC", "1")
    monkeypatch.setenv("LOGGERD_SEGMENT_LENGTH", str(segment_length))
    monkeypatch.setenv("ENCODER_ZERO_COPY", "0")

    params = Params(str(params_root))
    params.put_bool("RecordFront", True)
    params.put_bool("RecordAudio", True)

    w, h = 320, 240
    frame_size = w * h * 3 // 2
    frame_spec = (w, h, frame_size, w, w * h)
    streams = [
      (VisionStreamType.VISION_STREAM_ROAD, "roadCameraState"),
      (VisionStreamType.VISION_STREAM_WIDE_ROAD, "wideRoadCameraState"),
      (VisionStreamType.VISION_STREAM_DRIVER, "driverCameraState"),
    ]
    services = [state for _, state in streams] + ["rawAudioData"]
    pm = messaging.PubMaster(services)
    vipc_server = None

    try:
      vipc_server = VisionIpcServer("camerad")
      for stream_type, _ in streams:
        vipc_server.create_buffers_with_sizes(stream_type, 40, *frame_spec)
      vipc_server.start_listener()

      loggerd.start()
      encoderd.start()
      for service in services:
        assert pm.wait_for_readers_to_update(service, timeout=5)

      deadline = time.monotonic() + 10
      initial_state = None
      while time.monotonic() < deadline:
        segment = params.get("LoggerdSegment")
        epoch = params.get("LoggerdSegmentEpoch")
        if segment is not None and epoch is not None:
          initial_state = (int(segment), int(epoch))
          if initial_state[0] == 0:
            break
        time.sleep(0.05)
      assert initial_state is not None and initial_state[0] == 0

      frame = np.zeros(frame_size, dtype=np.uint8).tobytes()
      audio_data = bytes(800 * 2)
      frame_start = time.monotonic()
      post_boundary_frames = 0
      last_frame_id = 0

      # Keep the synthetic stream flowing across loggerd's time boundary. Stop
      # after enough segment-1 frames for the delayed qcamera encoder to emit.
      for frame_id in range(1, segment_length * 2 * fps + 1):
        target = frame_start + frame_id / fps
        delay = target - time.monotonic()
        if delay > 0:
          time.sleep(delay)

        timestamp_ns = frame_id * 1_000_000_000 // fps
        for stream_type, state_name in streams:
          vipc_server.send(stream_type, frame, frame_id, timestamp_ns, timestamp_ns)
          camera_state = messaging.new_message(state_name)
          getattr(camera_state, state_name).frameId = frame_id
          pm.send(state_name, camera_state)
        audio = messaging.new_message("rawAudioData")
        audio.rawAudioData.data = audio_data
        audio.rawAudioData.sampleRate = 16000
        pm.send("rawAudioData", audio)
        last_frame_id = frame_id

        segment = params.get("LoggerdSegment")
        if segment is not None and int(segment) >= 1:
          post_boundary_frames += 1
          if post_boundary_frames >= 45:
            break

      assert post_boundary_frames >= 45, "loggerd did not rotate and keep processing encoder frames"
      final_segment = int(params.get("LoggerdSegment"))
      final_epoch = int(params.get("LoggerdSegmentEpoch"))
      assert final_segment == 1
      assert final_epoch == initial_state[1] + 1
    finally:
      try:
        encoderd.stop()
      finally:
        try:
          loggerd.stop()
        finally:
          vipc_server = None

    segment_dirs = sorted(log_root.glob("*--*"), key=lambda path: int(path.name.rsplit("--", 1)[1]))
    assert [int(path.name.rsplit("--", 1)[1]) for path in segment_dirs] == [0, 1]
    expected_files = {"rlog.zst", "qlog.zst", "qcamera.ts", "fcamera.hevc", "dcamera.hevc", "ecamera.hevc"}
    encode_services = ("roadEncodeIdx", "wideRoadEncodeIdx", "driverEncodeIdx", "qRoadEncodeIdx")
    frame_ids_by_service = defaultdict(list)
    publish_times_by_service = defaultdict(list)

    for segment_num, segment_dir in enumerate(segment_dirs):
      files = {path.name for path in segment_dir.iterdir() if path.is_file()}
      assert files == expected_files, f"unexpected segment {segment_num} files: {files ^ expected_files}"
      assert all((segment_dir / name).stat().st_size > 0 for name in expected_files)

      rlog = list(LogReader(str(segment_dir / "rlog.zst")))
      self._check_init_data(rlog)
      expected_start = SentinelType.startOfRoute if segment_num == 0 else SentinelType.startOfSegment
      expected_end = SentinelType.endOfRoute if segment_num == 1 else SentinelType.endOfSegment
      assert rlog[1].sentinel.type == expected_start
      assert rlog[-1].sentinel.type == expected_end
      assert any(message.which() == "rawAudioData" for message in rlog)

      qlog = list(LogReader(str(segment_dir / "qlog.zst")))
      self._check_init_data(qlog)
      assert qlog[1].sentinel.type == expected_start
      assert qlog[-1].sentinel.type == expected_end
      for message in qlog:
        service = message.which()
        if service in encode_services:
          index = getattr(message, service)
          assert index.segmentNum == segment_num
          assert 1 <= index.frameId <= last_frame_id
          frame_ids_by_service[service].append(index.frameId)
          publish_times_by_service[service].append(message.logMonoTime)

    assert set(frame_ids_by_service) == set(encode_services)
    expected_frame_ids = set(range(1, last_frame_id + 1))
    for service, frame_ids in frame_ids_by_service.items():
      frame_counts = Counter(frame_ids)
      emitted_frame_ids = set(frame_counts)
      missing_frame_ids = sorted(expected_frame_ids - emitted_frame_ids)
      unexpected_frame_ids = sorted(emitted_frame_ids - expected_frame_ids)
      duplicate_frame_ids = sorted(frame_id for frame_id, count in frame_counts.items() if count > 1)
      assert not missing_frame_ids and not unexpected_frame_ids and not duplicate_frame_ids, (
        f"incomplete EncodeIdx frame set for {service}: "
        f"expected {len(expected_frame_ids)} frames 1..{last_frame_id}, got {len(frame_ids)}; "
        f"missing={missing_frame_ids}, unexpected={unexpected_frame_ids}, duplicates={duplicate_frame_ids}"
      )
      publish_gaps_ns = [
        later - earlier
        for earlier, later in zip(publish_times_by_service[service], publish_times_by_service[service][1:])
      ]
      max_publish_gap_ns = max(publish_gaps_ns)
      assert max_publish_gap_ns <= 2_000_000_000 // fps, (
        f"encoder output stalled for {service}: largest publish gap {max_publish_gap_ns / 1e6:.1f} ms"
      )
      # The qRoad H.264 stream may reorder B-frames; the other KA2 outputs
      # are HEVC and should retain input frame order.
      if service != "qRoadEncodeIdx":
        assert frame_ids == sorted(frame_ids), f"out-of-order EncodeIndex frame ids for {service}"

    qcamera = subprocess.run(
      ["ffprobe", "-v", "error", "-show_entries", "stream=codec_type", "-of", "json", str(segment_dirs[0] / "qcamera.ts")],
      check=True, capture_output=True, text=True,
    )
    assert any(stream["codec_type"] == "audio" for stream in json.loads(qcamera.stdout)["streams"])

  def test_bootlog(self):
    # generate bootlog with fake launch log
    launch_log = ''.join(str(random.choice(string.printable)) for _ in range(100))
    with open("/tmp/launch_log", "w") as f:
      f.write(launch_log)

    bootlog_path = self._gen_bootlog()
    lr = list(LogReader(str(bootlog_path)))

    # check length
    assert len(lr) == 2  # boot + initData

    self._check_init_data(lr)

    # check msgs
    bootlog_msgs = [m for m in lr if m.which() == 'boot']
    assert len(bootlog_msgs) == 1

    # sanity check values
    boot = bootlog_msgs.pop().boot
    assert abs(boot.wallTimeNanos - time.time_ns()) < 5*1e9 # within 5s
    assert boot.launchLog == launch_log

    expected_pstore_files = []
    for fn in ["console-ramoops", "console-ramoops-0", "pmsg-ramoops-0"]:
      path = Path(os.path.join("/sys/fs/pstore/", fn))
      if path.is_file():
        expected_pstore_files.append((fn, path))

    if expected_pstore_files:
      for fn, path in expected_pstore_files:
        with open(path, "rb") as f:
          expected_val = f.read()
        bootlog_val = [e.value for e in boot.pstore.entries if e.key == fn][0]
        assert expected_val == bootlog_val
    else:
      assert len(boot.pstore.entries) == 0

    # bootlog filename should follow timestamp-based boot naming
    bootlog_name_re = re.compile(fr"^{RE.TIMESTAMP}---boot\.zst$")
    assert bootlog_name_re.match(bootlog_path.name) is not None
    assert bootlog_name_re.match(self._gen_bootlog().name) is not None

  def test_qlog(self):
    qlog_services = [s for s in CEREAL_SERVICES if SERVICE_LIST[s].decimation is not None]
    no_qlog_services = [s for s in CEREAL_SERVICES if SERVICE_LIST[s].decimation is None]

    services = random.sample(qlog_services, random.randint(2, min(10, len(qlog_services)))) + \
               random.sample(no_qlog_services, random.randint(2, min(10, len(no_qlog_services))))
    sent_msgs = self._publish_random_messages(services)

    qlog_path = os.path.join(self._get_latest_log_dir(), "qlog.zst")
    lr = list(LogReader(qlog_path))

    # check initData and sentinel
    self._check_init_data(lr)
    self._check_sentinel(lr, True)

    recv_msgs = defaultdict(list)
    for m in lr:
      recv_msgs[m.which()].append(m)

    for s, msgs in sent_msgs.items():
      recv_cnt = len(recv_msgs[s])

      if s in no_qlog_services:
        # check services with no specific decimation aren't in qlog
        assert recv_cnt == 0, f"got {recv_cnt} {s} msgs in qlog"
      else:
        # check logged message count matches decimation
        expected_cnt = (len(msgs) - 1) // SERVICE_LIST[s].decimation + 1
        assert recv_cnt == expected_cnt, f"expected {expected_cnt} msgs for {s}, got {recv_cnt}"

  def test_rlog(self):
    services = random.sample(CEREAL_SERVICES, random.randint(5, 10))
    sent_msgs = self._publish_random_messages(services)

    lr = list(LogReader(os.path.join(self._get_latest_log_dir(), "rlog.zst")))

    # check initData and sentinel
    self._check_init_data(lr)
    self._check_sentinel(lr, True)

    # check all messages were logged and in order
    lr = lr[2:-1] # slice off initData and both sentinels
    for m in lr:
      sent = sent_msgs[m.which()].pop(0)
      sent.clear_write_flag()
      assert sent.to_bytes() == m.as_builder().to_bytes()

  def test_preserving_bookmarked_segments(self):
    services = set(random.sample(CEREAL_SERVICES, random.randint(5, 10))) | {"userBookmark"}
    self._publish_random_messages(services)

    segment_dir = self._get_latest_log_dir()
    assert getxattr(segment_dir, PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE

  def test_not_preserving_nonbookmarked_segments(self):
    services = set(random.sample(CEREAL_SERVICES, random.randint(5, 10))) - {"userBookmark", "audioFeedback"}
    self._publish_random_messages(services)

    segment_dir = self._get_latest_log_dir()
    assert getxattr(segment_dir, PRESERVE_ATTR_NAME) is None

  @pytest.mark.parametrize("record_front", [True, False])
  def test_record_front(self, record_front):
    params = Params()
    params.put_bool("RecordFront", record_front)

    self._publish_camera_and_audio_messages()

    dcamera_hevc_exists = os.path.exists(os.path.join(self._get_latest_log_dir(), 'dcamera.hevc'))
    assert dcamera_hevc_exists == record_front

  @pytest.mark.parametrize("record_audio", [True, False])
  def test_record_audio(self, record_audio):
    params = Params()
    params.put_bool("RecordAudio", record_audio)

    self._publish_camera_and_audio_messages()

    qcamera_ts_path = os.path.join(self._get_latest_log_dir(), 'qcamera.ts')
    ffprobe_cmd = f"ffprobe -i {qcamera_ts_path} -show_streams -select_streams a -loglevel error"
    has_audio_stream = subprocess.run(ffprobe_cmd, shell=True, capture_output=True).stdout.strip() != b''
    assert has_audio_stream == record_audio

    raw_audio_in_rlog = any(m.which() == 'rawAudioData' for m in LogReader(os.path.join(self._get_latest_log_dir(), 'rlog.zst')))
    assert raw_audio_in_rlog == record_audio

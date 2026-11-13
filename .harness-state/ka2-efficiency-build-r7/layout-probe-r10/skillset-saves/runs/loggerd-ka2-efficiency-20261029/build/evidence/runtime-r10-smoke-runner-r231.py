import gc
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import traceback
import uuid

import cereal.messaging as messaging
from cereal import log
from msgq.visionipc import VisionStreamType
from openpilot.common.params import Params
from openpilot.system.loggerd.tests.test_dmabuf_fixture import (
  _assert_camera_stack_idle, _send_frame, _wait_matching_line,
)
from openpilot.tools.lib.logreader import LogReader

ROOT = Path('/data/openpilot')
EVIDENCE = ROOT / 'skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence'
LOGGERD = ROOT / 'system/loggerd/loggerd'
ENCODERD = ROOT / 'system/loggerd/encoderd'
PUBLISHER = ROOT / 'build/system/loggerd/tests/visionipc_dmabuf_publisher'
FPS = 20
FRAME_DURATION_NS = 1_000_000_000 // FPS
CAMERAS = (
  (VisionStreamType.VISION_STREAM_ROAD, 'roadCameraState'),
  (VisionStreamType.VISION_STREAM_DRIVER, 'driverCameraState'),
  (VisionStreamType.VISION_STREAM_WIDE_ROAD, 'wideRoadCameraState'),
)
EXPECTED_FILES = {'rlog.zst', 'qlog.zst', 'qcamera.ts', 'fcamera.hevc', 'dcamera.hevc', 'ecamera.hevc'}
INDEX_SERVICES = ('roadEncodeIdx', 'qRoadEncodeIdx', 'wideRoadEncodeIdx', 'driverEncodeIdx')

start_time = time.monotonic()
records = []
status = {'pass': False, 'error': None}
root = Path(tempfile.mkdtemp(prefix=f'loggerd_runtime_r10_{os.getpid()}_', dir='/tmp'))
log_root = root / 'logs'
params_root = root / 'params'
log_root.mkdir()
params_root.mkdir()
prefix = f'loggerd_runtime_{os.getpid()}_{uuid.uuid4().hex[:8]}'
msgq_dir = Path('/dev/shm') / f'msgq_{prefix}'
vipc_socket = Path(f'/tmp/{prefix}_visionipc_camerad')
env = os.environ.copy()
env.update({
  'LOG_ROOT': str(log_root),
  'PARAMS_ROOT': str(params_root),
  'OPENPILOT_PREFIX': prefix,
  'LOGGERD_TEST': '1',
  'LOGGERD_SEGMENT_LENGTH': '2',
  'LOGGERD_TEST_DETERMINISTIC': '1',
  'ENCODERD_DETERMINISTIC_DISABLE': '0',
})
for key in ('ZMQ', 'CEREAL_FAKE', 'LOGGERD_TEST_MPP_FAULT_STREAM', 'LOGGERD_TEST_MPP_FAULT_MODE'):
  env.pop(key, None)
for key in ('LOG_ROOT', 'PARAMS_ROOT', 'OPENPILOT_PREFIX', 'LOGGERD_TEST',
            'LOGGERD_SEGMENT_LENGTH', 'LOGGERD_TEST_DETERMINISTIC', 'ENCODERD_DETERMINISTIC_DISABLE'):
  os.environ[key] = env[key]
for key in ('ZMQ', 'CEREAL_FAKE', 'LOGGERD_TEST_MPP_FAULT_STREAM', 'LOGGERD_TEST_MPP_FAULT_MODE'):
  os.environ.pop(key, None)
params = Params(str(params_root))
params.put_bool('RecordFront', True)
params.put_bool('RecordAudio', True)

publisher = loggerd = encoderd = None
publisher_log = []
loggerd_handle = encoderd_handle = None
loggerd_log_path = root / 'loggerd.stdout.log'
encoderd_log_path = root / 'encoderd.stdout.log'
reports = {}
seen_segments = set()
last_frame_id = 0


def event(message):
  elapsed = time.monotonic() - start_time
  records.append(f'{elapsed:8.3f}s {message}')


def param_text(key):
  raw = params.get(key)
  return raw.decode() if isinstance(raw, bytes) else (str(raw) if raw is not None else None)


def raw_param(key):
  path = Path(params.get_param_path(key))
  try:
    return path.read_text()
  except FileNotFoundError:
    return None


def alive(proc, label):
  if proc is None or proc.poll() is not None:
    raise RuntimeError(f'{label} exited early: {None if proc is None else proc.returncode}')


def stop(proc, label, timeout=5.0):
  if proc is None or proc.poll() is not None:
    return
  proc.send_signal(signal.SIGINT)
  try:
    proc.wait(timeout=timeout)
  except subprocess.TimeoutExpired:
    event(f'{label} did not stop within {timeout}s; sending SIGKILL')
    proc.kill()
    proc.wait(timeout=2.0)
  event(f'{label} stopped returncode={proc.returncode}')


def read_report(segment):
  data = raw_param('LoggerdRotationReport')
  if not data:
    return
  try:
    report = json.loads(data)
  except json.JSONDecodeError:
    return
  if report.get('segment') == segment:
    reports[segment] = report


try:
  preflight = subprocess.run(
    ['python3', '-c', 'from openpilot.system.loggerd.tests.test_dmabuf_fixture import _assert_camera_stack_idle; _assert_camera_stack_idle()'],
    cwd=ROOT, env=env, text=True, capture_output=True, check=False,
  )
  (EVIDENCE / 'runtime-r10-preflight-r231.log').write_text(
    'fresh preflight immediately before production runtime workload\n'
    f'exit={preflight.returncode}\nstdout:\n{preflight.stdout}\nstderr:\n{preflight.stderr}'
  )
  if preflight.returncode:
    raise RuntimeError(f'board preflight failed, stopping before device workload: {preflight.stdout}{preflight.stderr}')
  event('fresh board preflight passed: no active camera/logger/encoder processes and no device users')
  if not (Path('/dev/dma_heap/system').exists() and Path('/dev/mpp_service').exists() and Path('/dev/rga').exists()):
    raise RuntimeError('expected DMA-heap, MPP, and RGA device nodes are not all present')
  event('dependencies present: /dev/dma_heap/system, /dev/mpp_service, /dev/rga')
  if any(Path('/dev/shm').glob(f'msgq_{prefix}')) or vipc_socket.exists():
    raise RuntimeError('unique test IPC paths unexpectedly existed before launch')
  msgq_dir.mkdir(mode=0o700)
  event(f'created isolated test MSGQ path {msgq_dir}')

  publisher = subprocess.Popen([str(PUBLISHER), '--serve'], cwd=ROOT, env=env,
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, bufsize=1)
  ready = _wait_matching_line(
    publisher,
    'READY streams=road,driver,wideRoad width=1920 height=1200 stride=1920 uv_offset=2304000 buffers=4 backing=dma_heap',
    10.0,
  )
  publisher_log.append(ready)
  event(f'DMA-heap VisionIPC publisher ready: {ready}')

  pm = messaging.PubMaster([state for _, state in CAMERAS] + ['rawAudioData'])
  event('PubMaster for camera states and rawAudioData initialized before daemon launch')
  loggerd_handle = loggerd_log_path.open('w')
  encoderd_handle = encoderd_log_path.open('w')
  loggerd = subprocess.Popen([str(LOGGERD)], cwd=ROOT, env=env,
                             stdout=loggerd_handle, stderr=subprocess.STDOUT, text=True)
  encoderd = subprocess.Popen([str(ENCODERD)], cwd=ROOT, env=env,
                              stdout=encoderd_handle, stderr=subprocess.STDOUT, text=True)
  event(f'production loggerd started pid={loggerd.pid}; command={LOGGERD}')
  event(f'production encoderd started pid={encoderd.pid}; command={ENCODERD}')
  startup_deadline = time.monotonic() + 10.0
  for _, service in CAMERAS:
    alive(loggerd, 'loggerd')
    alive(encoderd, 'encoderd')
    remaining = max(0.0, startup_deadline - time.monotonic())
    if not pm.wait_for_readers_to_update(service, timeout=remaining):
      raise RuntimeError(f'{service} subscription was not ready within the 10s startup window')
    event(f'startup ready: subscribed reader for {service}')
  if not pm.wait_for_readers_to_update('rawAudioData', timeout=max(0.0, startup_deadline-time.monotonic())):
    raise RuntimeError('rawAudioData subscription was not ready within the 10s startup window')
  event('startup ready: subscribed reader for rawAudioData')

  segment_zero_deadline = time.monotonic() + max(0.0, startup_deadline - time.monotonic())
  while time.monotonic() < segment_zero_deadline:
    alive(loggerd, 'loggerd')
    alive(encoderd, 'encoderd')
    current = param_text('LoggerdSegment')
    if current == '0':
      break
    time.sleep(0.05)
  if param_text('LoggerdSegment') != '0':
    raise RuntimeError('loggerd did not enter segment 0 within the 10s startup window')
  event('startup ready: LoggerdSegment=0; loggerd and encoderd still alive')

  flow_start = time.monotonic()
  flow_deadline = flow_start + 24.0
  frame_clock = flow_start
  audio_samples = b'\xe8\x03\x18\xfc' * 400
  last_poll = 0.0
  post_segment_two = 0
  while time.monotonic() < flow_deadline:
    last_frame_id += 1
    scheduled = frame_clock + last_frame_id / FPS
    delay = scheduled - time.monotonic()
    if delay > 0:
      time.sleep(delay)
    timestamp = last_frame_id * FRAME_DURATION_NS
    for stream_type, service in CAMERAS:
      alive(loggerd, 'loggerd')
      alive(encoderd, 'encoderd')
      _send_frame(publisher, int(stream_type), last_frame_id, timestamp, timestamp + FRAME_DURATION_NS // 2)
      camera_state = messaging.new_message(service)
      getattr(camera_state, service).frameId = last_frame_id
      pm.send(service, camera_state)
    audio = messaging.new_message('rawAudioData')
    audio.rawAudioData.data = audio_samples
    audio.rawAudioData.sampleRate = 16000
    pm.send('rawAudioData', audio)

    segment_text = param_text('LoggerdSegment')
    if segment_text is not None:
      segment = int(segment_text)
      if segment not in seen_segments:
        seen_segments.add(segment)
        event(f'readiness progression: LoggerdSegment={segment}; process_alive=true')
      if segment >= 1:
        read_report(segment)
      if segment >= 2:
        post_segment_two += 1
        if post_segment_two >= 20:
          break
    alive(loggerd, 'loggerd')
    alive(encoderd, 'encoderd')
    if time.monotonic() - last_poll >= 2.0:
      last_poll = time.monotonic()
      event(f'2s liveness poll: loggerd=alive encoderd=alive segment={segment_text}')
  else:
    raise RuntimeError(f'two rotations did not finish within 24s; observed segments={sorted(seen_segments)}')
  if post_segment_two < 20:
    raise RuntimeError(f'only {post_segment_two} frames followed segment 2; need 20')
  event(f'smoke flow complete: frames_sent={last_frame_id}; reports_seen={sorted(reports)}')
  # Close the route writers before inspecting the final file set and media, as
  # the daemons intentionally keep lock sidecars while their segment is open.
  stop(encoderd, 'encoderd')
  stop(loggerd, 'loggerd')

  route = param_text('CurrentRoute')
  if not route:
    raise RuntimeError('CurrentRoute was not published')
  for segment in (1, 2):
    report = reports.get(segment)
    if report is None:
      raise RuntimeError(f'missing rotation report for segment {segment}')
    expected_keys = {'route', 'epoch', 'segment', 'degraded', 'missing_streams', 'elapsed_ms', 'known_drop_counts'}
    if set(report) != expected_keys:
      raise RuntimeError(f'segment {segment} report schema mismatch: {sorted(report)}')
    if report['route'] != route or report['epoch'] != segment + 1 or report['segment'] != segment:
      raise RuntimeError(f'segment {segment} route/epoch mismatch: {report}')
    if report['degraded'] is not False or report['missing_streams'] != []:
      raise RuntimeError(f'segment {segment} reported degradation: {report}')
    if any(type(value) is not int or value != 0 for value in report['known_drop_counts'].values()):
      raise RuntimeError(f'segment {segment} reported drops: {report}')
    event(f'healthy rotation report segment={segment}: elapsed_ms={report["elapsed_ms"]} missing=[] drops=0')

  segment_dirs = sorted(log_root.glob('*--*'), key=lambda path: int(path.name.rsplit('--', 1)[1]))
  numbers = [int(path.name.rsplit('--', 1)[1]) for path in segment_dirs]
  if numbers != [0, 1, 2]:
    raise RuntimeError(f'expected segment directories [0,1,2], found {numbers}')
  for segment_num, segment_dir in enumerate(segment_dirs):
    files = {path.name for path in segment_dir.iterdir() if path.is_file()}
    if files != EXPECTED_FILES:
      raise RuntimeError(f'segment {segment_num} file set mismatch: {sorted(files ^ EXPECTED_FILES)}')
    if any((segment_dir / name).stat().st_size <= 0 for name in EXPECTED_FILES):
      raise RuntimeError(f'segment {segment_num} contains an empty media/log file')
    rlog = list(LogReader(str(segment_dir / 'rlog.zst')))
    qlog = list(LogReader(str(segment_dir / 'qlog.zst')))
    if rlog[0].which() != 'initData' or qlog[0].which() != 'initData':
      raise RuntimeError(f'segment {segment_num} route logs lack initData')
    expected_start = log.Sentinel.SentinelType.startOfRoute if segment_num == 0 else log.Sentinel.SentinelType.startOfSegment
    expected_end = log.Sentinel.SentinelType.endOfRoute if segment_num == 2 else log.Sentinel.SentinelType.endOfSegment
    if rlog[1].sentinel.type != expected_start or rlog[-1].sentinel.type != expected_end:
      raise RuntimeError(f'segment {segment_num} rlog boundary sentinels mismatch')
    if qlog[1].sentinel.type != expected_start or qlog[-1].sentinel.type != expected_end:
      raise RuntimeError(f'segment {segment_num} qlog boundary sentinels mismatch')
    indices = []
    for message in qlog:
      service = message.which()
      if service in INDEX_SERVICES:
        index = getattr(message, service)
        if index.segmentNum != segment_num:
          raise RuntimeError(f'{service} crossed segment boundary in segment {segment_num}: {index.segmentNum}')
        indices.append(service)
    if 'qRoadEncodeIdx' not in indices:
      raise RuntimeError(f'segment {segment_num} has no qRoadEncodeIdx')
    ffprobe = subprocess.run(
      ['ffprobe', '-v', 'error', '-show_entries', 'stream=codec_name,codec_type,width,height', '-of', 'json', str(segment_dir / 'qcamera.ts')],
      text=True, capture_output=True, check=False, timeout=10,
    )
    if ffprobe.returncode:
      raise RuntimeError(f'ffprobe failed on segment {segment_num}: {ffprobe.stderr}')
    media = json.loads(ffprobe.stdout)
    streams = media.get('streams', [])
    if not any(s.get('codec_type') == 'video' and s.get('codec_name') == 'h264' for s in streams):
      raise RuntimeError(f'qRoad H264 video absent in segment {segment_num}: {streams}')
    if not any(s.get('codec_type') == 'audio' for s in streams):
      raise RuntimeError(f'qRoad audio stream absent in segment {segment_num}: {streams}')
    decoded_audio = subprocess.run(
      ['ffmpeg', '-v', 'error', '-i', str(segment_dir / 'qcamera.ts'), '-map', '0:a:0', '-f', 's16le', '-acodec', 'pcm_s16le', '-'],
      capture_output=True, check=False, timeout=10,
    )
    if decoded_audio.returncode or not decoded_audio.stdout:
      raise RuntimeError(f'qRoad audio did not decode in segment {segment_num}: {decoded_audio.stderr!r}')
    event(f'segment {segment_num} artifacts verified: all six files nonempty; qRoad H264+audio decodable; audio_samples={len(decoded_audio.stdout)//2}')

  status['pass'] = True
  event('runtime readiness PASS for synthetic DMA-BUF board target; live V4L2 export was not exercised')
except Exception as exc:
  status['error'] = f'{type(exc).__name__}: {exc}'
  event(f'FAIL: {status["error"]}')
  event(traceback.format_exc().replace('\n', ' | '))
finally:
  gc.collect()
  stop(encoderd, 'encoderd')
  stop(loggerd, 'loggerd')
  if publisher is not None and publisher.poll() is None:
    try:
      if publisher.stdin is not None:
        publisher.stdin.write('STOP\n')
        publisher.stdin.flush()
      cleanup = _wait_matching_line(publisher, 'CLEANUP_OK', 5.0)
      publisher_log.append(cleanup)
      publisher.wait(timeout=5.0)
      event(f'DMA-heap publisher stopped returncode={publisher.returncode}; {cleanup}')
    except Exception as exc:
      event(f'publisher graceful cleanup failed: {type(exc).__name__}: {exc}; sending kill')
      publisher.kill()
      publisher.wait(timeout=2.0)
  elif publisher is not None:
    event(f'DMA-heap publisher had already exited returncode={publisher.returncode}')
  for handle in (loggerd_handle, encoderd_handle):
    if handle is not None:
      handle.close()
  if loggerd_log_path.exists():
    event('loggerd stdout follows:')
    records.extend('LOGGERD ' + line for line in loggerd_log_path.read_text(errors='replace').splitlines())
  if encoderd_log_path.exists():
    event('encoderd stdout follows:')
    records.extend('ENCODERD ' + line for line in encoderd_log_path.read_text(errors='replace').splitlines())
  records.extend('PUBLISHER ' + line for line in publisher_log)
  if publisher is not None and publisher.stdout is not None:
    try:
      records.extend('PUBLISHER ' + line.rstrip('\n') for line in publisher.stdout.readlines())
      publisher.stdout.close()
    except Exception:
      pass
  (EVIDENCE / 'runtime-r10-startup-r231.log').write_text('\n'.join(r for r in records if 'started pid=' in r or 'startup ready:' in r or 'DMA-heap VisionIPC publisher ready' in r) + '\n')
  (EVIDENCE / 'runtime-r10-readiness-r231.log').write_text('\n'.join(r for r in records if 'readiness progression:' in r or 'liveness poll:' in r or 'healthy rotation report' in r) + '\n')
  (EVIDENCE / 'runtime-r10-dependencies-r231.log').write_text('\n'.join(r for r in records if 'dependencies present:' in r or 'publisher ready:' in r or 'subscribed reader' in r or 'artifacts verified:' in r) + '\n')
  (EVIDENCE / 'runtime-r10-smoke-r231.log').write_text(
    f'target=local KA2/RK3588 board, isolated synthetic DMA-heap VisionIPC\n'
    f'production_binaries={LOGGERD},{ENCODERD}\n'
    f'LOGGERD_TEST=1 LOGGERD_SEGMENT_LENGTH=2 (runtime test override only)\n'
    f'test-only hooks/counters/fault injection compiled out; no fault selectors set\n'
    f'readiness_window=10s for subscriptions and segment 0; lifecycle_window=24s for segments 1 and 2\n'
    f'predicate=both processes alive; camera/audio subscriptions present; reports 1/2 healthy; three segments have all six files; qRoad H264+audio decode\n'
    f'probe_result={"pass" if status["pass"] else "fail"}\n'
    f'error={status["error"] or "none"}\n' + '\n'.join(records) + '\n'
  )
  try:
    assert not vipc_socket.exists(), f'test publisher left VisionIPC socket {vipc_socket}'
    shutil.rmtree(msgq_dir, ignore_errors=True)
    _assert_camera_stack_idle()
    status['postflight'] = 'pass'
    (EVIDENCE / 'runtime-r10-postflight-r231.log').write_text('postflight=PASS\n' + 'test-owned VisionIPC socket absent\n' + 'isolated MSGQ directory removed\nno camera/logger/encoder processes or users of video/media/subdev/MPP/RGA nodes\n')
    event('postflight passed: isolated socket/MSGQ removed and board device nodes idle')
  except Exception as exc:
    status['postflight'] = f'fail: {type(exc).__name__}: {exc}'
    (EVIDENCE / 'runtime-r10-postflight-r231.log').write_text(f'postflight=FAIL\n{status["postflight"]}\n')
    event(f'postflight FAIL: {status["postflight"]}')
  shutil.rmtree(root, ignore_errors=True)
  (EVIDENCE / 'runtime-r10-smoke-r231.log').write_text(
    f'target=local KA2/RK3588 board, isolated synthetic DMA-heap VisionIPC\n'
    f'production_binaries={LOGGERD},{ENCODERD}\n'
    f'LOGGERD_TEST=1 LOGGERD_SEGMENT_LENGTH=2 (runtime test override only)\n'
    f'test-only hooks/counters/fault injection compiled out; no fault selectors set\n'
    f'readiness_window=10s for subscriptions and segment 0; lifecycle_window=24s for segments 1 and 2\n'
    f'predicate=both processes alive; camera/audio subscriptions present; reports 1/2 healthy; three segments have all six files; qRoad H264+audio decode\n'
    f'probe_result={"pass" if status["pass"] and status.get("postflight") == "pass" else "fail"}\n'
    f'error={status["error"] or "none"}; postflight={status.get("postflight", "not completed")}\n' + '\n'.join(records) + '\n'
  )
print(json.dumps(status, sort_keys=True))
if not status['pass'] or status.get('postflight') != 'pass':
  raise SystemExit(1)

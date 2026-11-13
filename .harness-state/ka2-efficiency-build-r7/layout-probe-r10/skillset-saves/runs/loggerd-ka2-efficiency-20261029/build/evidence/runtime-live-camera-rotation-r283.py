#!/usr/bin/env python3
"""Exercise real KA2 camera DMA-BUFs through production loggerd/encoderd rotations."""
import gc, json, os, signal, subprocess, tempfile, time, traceback, uuid
from pathlib import Path
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.system.loggerd.tests.test_dmabuf_fixture import _assert_camera_stack_idle
from openpilot.tools.lib.logreader import LogReader

ROOT = Path('/data/openpilot')
EVIDENCE = ROOT / 'skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence'
BINARIES = {'camerad': ROOT/'system/camerad/camerad', 'loggerd': ROOT/'system/loggerd/loggerd', 'encoderd': ROOT/'system/loggerd/encoderd'}
CAMERAS = ('roadCameraState', 'driverCameraState', 'wideRoadCameraState')
EXPECTED = {'rlog.zst', 'qlog.zst', 'qcamera.ts', 'fcamera.hevc', 'dcamera.hevc', 'ecamera.hevc'}
INDEXES = {'roadEncodeIdx', 'qRoadEncodeIdx', 'wideRoadEncodeIdx', 'driverEncodeIdx'}
root = Path(tempfile.mkdtemp(prefix=f'ka2_live_rotation_{os.getpid()}_', dir='/data'))
log_root, params_root = root / 'logs', root / 'params'
log_root.mkdir(); params_root.mkdir()
prefix = f'ka2_live_rotation_{os.getpid()}_{uuid.uuid4().hex[:8]}'
msgq_dir = Path('/dev/shm') / f'msgq_{prefix}'
vipc_socket = Path(f'/tmp/{prefix}_visionipc_camerad')
env = os.environ.copy()
for key in ('ZMQ', 'CEREAL_FAKE', 'ENCODERD_DETERMINISTIC_DISABLE', 'LOGGERD_TEST_MPP_FAULT_STREAM', 'LOGGERD_TEST_MPP_FAULT_MODE'):
  env.pop(key, None)
env.update(LOG_ROOT=str(log_root), PARAMS_ROOT=str(params_root), OPENPILOT_PREFIX=prefix,
           LOGGERD_TEST='1', LOGGERD_TEST_DETERMINISTIC='1', LOGGERD_SEGMENT_LENGTH='2', TMPDIR=str(root))
os.environ.update(env)
params = Params(str(params_root)); params.put_bool('RecordFront', True); params.put_bool('RecordAudio', True)
procs, handles, events, reports = {}, {}, [], {}
counts = {name: 0 for name in CAMERAS}
seen_segments, status = set(), {'pass': False, 'error': None, 'postflight': 'not completed', 'data_root': str(root)}
t0 = time.monotonic()

def event(msg): events.append(f'{time.monotonic()-t0:8.3f}s {msg}')
def alive(name):
  p = procs[name]
  if p.poll() is not None: raise RuntimeError(f'{name} exited early rc={p.returncode}')
def start(name):
  handles[name] = (root / f'{name}.stdout.log').open('w')
  command = [str(BINARIES[name])]
  if name == 'loggerd' and os.getenv('LIVE_LOGGERD_GDB') == '1':
    command = ['/usr/bin/gdb','--batch','-q','-ex','set debuginfod enabled off','-ex','run','-ex','thread apply all bt full','--args',str(BINARIES[name])]
  procs[name] = subprocess.Popen(command, cwd=ROOT, env=env, stdout=handles[name], stderr=subprocess.STDOUT, text=True)
  event(f'{name} started pid={procs[name].pid}')
def stop(name):
  p = procs.get(name)
  if p is None or p.poll() is not None: return
  p.send_signal(signal.SIGINT)
  if name == 'camerad':
    time.sleep(4)
    if p.poll() is None:
      try:
        trace = subprocess.run(['/usr/bin/gdb','--batch','-q','-ex','set debuginfod enabled off',
                                '-ex','thread apply all bt full','-ex','detach','-p',str(p.pid)],
                               capture_output=True,text=True,timeout=12,check=False)
        out = trace.stdout + trace.stderr
        (EVIDENCE/'camerad-shutdown-gdb-r283.log').write_text(out)
        event(f'camerad shutdown still pending after 4s; attached gdb rc={trace.returncode}')
      except Exception as exc:
        event(f'camerad shutdown gdb capture failed: {type(exc).__name__}: {exc}')
  try: p.wait(timeout=30 if name == 'camerad' else 12)
  except subprocess.TimeoutExpired:
    event(f'{name} exceeded graceful-stop window; SIGKILL'); p.kill(); p.wait(timeout=3)
  event(f'{name} stopped rc={p.returncode}')
def param(key):
  if key == 'LoggerdRotationReport':
    path = Path(params.get_param_path(key))
    try: return path.read_text()
    except FileNotFoundError: return None
  raw = params.get(key)
  return raw.decode() if isinstance(raw, bytes) else (str(raw) if raw is not None else None)
def collect_report():
  raw = param('LoggerdRotationReport')
  if raw:
    try:
      obj = json.loads(raw)
      reports[obj['segment']] = obj
    except (json.JSONDecodeError, KeyError, TypeError): pass

try:
  _assert_camera_stack_idle()
  if any(not path.is_file() for path in BINARIES.values()): raise RuntimeError('missing production binary')
  if msgq_dir.exists() or vipc_socket.exists(): raise RuntimeError('unique IPC path already exists')
  msgq_dir.mkdir(mode=0o700)
  event(f'isolated MSGQ ready: {msgq_dir}')
  # Wait for genuine camera-state publication from the V4L2 producers.
  sm = messaging.SubMaster(list(CAMERAS))
  start('camerad')
  ready, deadline = set(), time.monotonic() + 40
  while time.monotonic() < deadline and ready != set(CAMERAS):
    alive('camerad'); sm.update(500)
    for cam in CAMERAS:
      if sm.updated[cam]:
        ready.add(cam); counts[cam] += 1
        event(f'camera ready {cam} frame_id={sm[cam].frameId}')
  if ready != set(CAMERAS): raise RuntimeError(f'camera streams absent: {sorted(set(CAMERAS)-ready)}')
  audio_pm = messaging.PubMaster(['rawAudioData'])
  start('loggerd'); start('encoderd')
  if not audio_pm.wait_for_readers_to_update('rawAudioData', timeout=15): raise RuntimeError('loggerd audio reader did not attach')
  deadline = time.monotonic() + 15
  while time.monotonic() < deadline and param('LoggerdSegment') != '0':
    for name in ('camerad', 'loggerd', 'encoderd'): alive(name)
    time.sleep(.05)
  if param('LoggerdSegment') != '0': raise RuntimeError('loggerd did not open segment 0')
  event('loggerd entered segment 0')

  stop_at, next_audio, post_two, last_poll = time.monotonic()+24, time.monotonic(), 0, time.monotonic()
  audio_samples = b'\xe8\x03\x18\xfc' * 400
  while time.monotonic() < stop_at:
    for name in ('camerad', 'loggerd', 'encoderd'): alive(name)
    sm.update(0)
    for cam in CAMERAS:
      if sm.updated[cam]: counts[cam] += 1
    now = time.monotonic()
    if now >= next_audio:
      msg = messaging.new_message('rawAudioData'); msg.rawAudioData.data = audio_samples; msg.rawAudioData.sampleRate = 16000
      audio_pm.send('rawAudioData', msg); next_audio += .05
    seg = param('LoggerdSegment')
    if seg is not None:
      seg = int(seg)
      if seg not in seen_segments: seen_segments.add(seg); event(f'LoggerdSegment={seg}; daemons alive')
      collect_report()
      if seg >= 2:
        post_two += 1
        if post_two >= 20: break
    if now-last_poll >= 2:
      event(f'liveness camerad=alive loggerd=alive encoderd=alive segment={seg}'); last_poll=now
    time.sleep(.01)
  if post_two < 20: raise RuntimeError(f'rotation timeout; observed segments={sorted(seen_segments)}')
  for name in ('encoderd', 'loggerd', 'camerad'): stop(name)
  route = param('CurrentRoute')
  if not route: raise RuntimeError('CurrentRoute missing')
  for seg in (1, 2):
    report = reports.get(seg)
    if report is None: raise RuntimeError(f'rotation report missing for segment {seg}; reports={sorted(reports)}')
    if report.get('route') != route or report.get('epoch') != seg+1 or report.get('segment') != seg: raise RuntimeError(f'rotation lineage mismatch: {report}')
    if report.get('degraded') is not False or report.get('missing_streams') != []: raise RuntimeError(f'degraded rotation: {report}')
    if any(type(v) is not int or v != 0 for v in report.get('known_drop_counts', {}).values()): raise RuntimeError(f'rotation reported known drops: {report}')
    event(f'healthy rotation segment={seg} elapsed_ms={report.get("elapsed_ms")}')
  dirs = sorted(log_root.glob('*--*'), key=lambda p: int(p.name.rsplit('--', 1)[1]))
  if [int(p.name.rsplit('--', 1)[1]) for p in dirs] != [0, 1, 2]: raise RuntimeError(f'expected segment dirs 0,1,2; found {[p.name for p in dirs]}')
  for seg, path in enumerate(dirs):
    files = {p.name for p in path.iterdir() if p.is_file()}
    if files != EXPECTED: raise RuntimeError(f'segment {seg} file mismatch: {sorted(files ^ EXPECTED)}')
    if any((path/name).stat().st_size <= 0 for name in EXPECTED): raise RuntimeError(f'segment {seg} has empty output')
    qlog = list(LogReader(str(path/'qlog.zst')))
    if not qlog or qlog[0].which() != 'initData': raise RuntimeError(f'segment {seg} qlog lacks initData')
    if 'qRoadEncodeIdx' not in {m.which() for m in qlog}: raise RuntimeError(f'segment {seg} has no qRoad index')
    for media in ('qcamera.ts', 'fcamera.hevc', 'dcamera.hevc', 'ecamera.hevc'):
      decoded = subprocess.run(['ffmpeg','-v','error','-i',str(path/media),'-f','null','-'],capture_output=True,timeout=35,check=False)
      if decoded.returncode: raise RuntimeError(f'{media} decode failed segment {seg}: {decoded.stderr[-1500:]!r}')
    event(f'segment {seg} all six files nonempty; qRoad, road, driver and wide-road decode passed')
  if any(n < 80 for n in counts.values()): raise RuntimeError(f'camera message counts too low: {counts}')
  status['pass'] = True
  event(f'live camera path PASS frame_counts={counts}')
except Exception as exc:
  status['error'] = f'{type(exc).__name__}: {exc}'
  event(f'FAIL {status["error"]}'); event(traceback.format_exc().replace('\n',' | '))
finally:
  gc.collect()
  for name in ('encoderd','loggerd','camerad'): stop(name)
  for handle in handles.values(): handle.close()
  for name in handles:
    path = root / f'{name}.stdout.log'
    if path.exists():
      event(f'{name} output follows'); events.extend(f'{name.upper()} {line}' for line in path.read_text(errors='replace').splitlines())
  try:
    if vipc_socket.exists(): raise RuntimeError(f'VisionIPC socket remains: {vipc_socket}')
    import shutil
    shutil.rmtree(msgq_dir, ignore_errors=True)
    _assert_camera_stack_idle()
    status['postflight'] = 'pass'; event('postflight PASS: no camera/logger/encoder process or video/MPP/RGA device users')
  except Exception as exc:
    status['postflight'] = f'fail: {type(exc).__name__}: {exc}'; event(f'POSTFLIGHT FAIL {status["postflight"]}')
  summary = ('target=local KA2 RK3588 board, actual camerad V4L2 DMA-BUFs\n'
             f'runner={Path(__file__)}\nproduction_binaries={",".join(str(p) for p in BINARIES.values())}\n'
             'LOGGERD_TEST=1, segment length 2s, deterministic KA2 rotation\n'
             'acceptance=3 live camera streams, 2 healthy rotations, zero known drops, 3 segment dirs, all video files decode, idle postflight\n'
             f'result={"pass" if status["pass"] and status["postflight"]=="pass" else "fail"}\n'
             f'error={status["error"] or "none"}; postflight={status["postflight"]}; data_root={root}\n'+'\n'.join(events)+'\n')
  EVIDENCE.mkdir(parents=True, exist_ok=True)
  (EVIDENCE/'runtime-live-camera-rotation-r283.log').write_text(summary)
  (EVIDENCE/'runtime-live-camera-rotation-r283.json').write_text(json.dumps(status,sort_keys=True,indent=2)+'\n')
print(json.dumps(status, sort_keys=True))
if not status['pass'] or status['postflight'] != 'pass': raise SystemExit(1)

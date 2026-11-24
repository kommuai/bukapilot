#!/usr/local/venv/bin/python3
import hashlib
import json
import os
import queue
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path('/data/openpilot')
RUN_ID = 'sidecamerad-mvp-20261113'
SUBMISSION = 'sidecamerad-mvp-20261113_health-check-ac4-r2.3'
DEVICE = '/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0'
SERVICE = 'frontLeftCameraState'
STREAM_NAME = 'VISION_STREAM_SIDE_FRONT_LEFT'
FRAME_TARGET = 6050
STEADY_NS = 300_000_000_000
MAX_WALL_S = 360
LOG_PATH = ROOT / 'skillset-saves/runs/sidecamerad-mvp-20261113/build/evidence/runtime-smoke-ac4-r2.3.log'
INTERVAL_PATH = ROOT / 'skillset-saves/runs/sidecamerad-mvp-20261113/build/artifacts/runtime_cadence_intervals-r2.3.json'
PROBE_PATH = ROOT / 'skillset-saves/runs/sidecamerad-mvp-20261113/build/artifacts/runtime_probe-ac4-r2.3.json'
REPORT_PATH = ROOT / 'skillset-saves/runs/sidecamerad-mvp-20261113/build/reports/report_runtime_health-ac4-r2.3.md'
EXPECTED = {
  'system/sidecamerad.py': '494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b',
  'system/manager/process_config.py': 'b7fdfda2878cae1fa6b354ed512bc33ad66f1abe1ed25c90ed980b547a8aa02d',
  'common/params_keys.h': 'c2925ebd8f9f26cc04951afb0e3c67ac7cb20a1c370b79aa93a25a4aaef2a031',
  'common/params_pyx.so': '3108e29407f31b9bd86a86ccafc82a4f98a225884412987a24be5c357dbfb586',
  'cereal/log.capnp': 'd5c0372d714e65461942874c91c522b205ce208261b382283839ed03b9731f64',
  'cereal/services.py': 'cacd4b749d9a0a3e19e3c466784dcf34925f263fdca3c26af72578c47d740a6b',
  'msgq_repo/msgq/visionipc/visionbuf.h': 'c971a1afa601f5bb8f560de6fb1dd78c2121cee61ea41c169f00a4154566e870',
  'msgq_repo/msgq/visionipc/visionipc_pyx.pyx': '36247e353901eb80e52735c91f466151b3026e77152dde9ad9e3bcd682c66f32',
  'msgq_repo/msgq/visionipc/visionipc_server.cc': '36b383bfcaf10eaa69f6844a579c86204b4a07464a889208fe059a6c1202d198',
}

def sha(path):
  h = hashlib.sha256()
  with open(path, 'rb') as f:
    for chunk in iter(lambda: f.read(1024 * 1024), b''):
      h.update(chunk)
  return h.hexdigest()

def source_hashes():
  return {p: sha(ROOT / p) for p in EXPECTED}

def utc_now():
  return datetime.now(timezone.utc).isoformat()

LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
INTERVAL_PATH.parent.mkdir(parents=True, exist_ok=True)
PROBE_PATH.parent.mkdir(parents=True, exist_ok=True)
REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
logf = open(LOG_PATH, 'w', encoding='utf-8', buffering=1)
def log(msg):
  line = f'{utc_now()} {msg}'
  print(line, flush=True)
  logf.write(line + '\n')
  logf.flush()

proc = None
reader_stop = threading.Event()
condition = threading.Condition()
readers_ready = {'cereal': threading.Event(), 'visionipc': threading.Event()}
records = {'cereal': {}, 'visionipc': {}}
duplicates = {'cereal': 0, 'visionipc': 0}
thread_errors = queue.Queue()
params_root = None
params_obj = None
capture_error = None
child_exit = None
startup_params = None
pre_shutdown_params = None
post_shutdown_params = None
initial_tcp = None
tcp_after = None
remaining_ffmpeg = []
sidecar_stdout = None
started_monotonic_ns = None
capture_started_monotonic_ns = None
capture_start_eof_ns = None
stop_reason = 'not-started'


def port_listeners():
  active = []
  for port in range(9007, 9011):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.05)
    try:
      if s.connect_ex(('127.0.0.1', port)) == 0:
        active.append(port)
    finally:
      s.close()
  return active

def read_params():
  try:
    raw = params_obj.get('SideCameraAvailability')
    if raw is None:
      return None
    if isinstance(raw, bytes):
      raw = raw.decode('utf-8')
    if isinstance(raw, str):
      return json.loads(raw)
    return raw
  except Exception as e:
    return {'read_error': f'{type(e).__name__}: {e}'}

def ffmpeg_pids():
  result = subprocess.run(['ps', '-eo', 'pid=,ppid=,comm='], check=False, capture_output=True, text=True)
  pids = []
  for line in result.stdout.splitlines():
    bits = line.split()
    if len(bits) == 3 and bits[2] == 'ffmpeg':
      try:
        pids.append(int(bits[0]))
      except ValueError:
        pass
  return pids

def add_record(kind, rec):
  fid = rec['frame_id']
  with condition:
    if fid in records[kind]:
      duplicates[kind] += 1
    records[kind][fid] = rec
    condition.notify_all()

def cereal_reader():
  try:
    from cereal import messaging
    poller = messaging.Poller()
    sock = messaging.sub_sock(SERVICE, poller=poller, addr='127.0.0.1', conflate=False)
    readers_ready['cereal'].set()
    while not reader_stop.is_set():
      events = poller.poll(100)
      if not events:
        continue
      # There is one registered topic. Drain it directly rather than comparing
      # transient SubSocket wrappers by object identity.
      while True:
        msg = messaging.recv_one_or_none(sock)
        if msg is None:
          break
        frame = getattr(msg, SERVICE)
        add_record('cereal', {
          'frame_id': int(frame.frameId),
          'log_mono_time_ns': int(msg.logMonoTime),
          'timestamp_sof_ns': int(frame.timestampSof),
          'timestamp_eof_ns': int(frame.timestampEof),
          'valid': bool(msg.valid),
          'received_monotonic_ns': time.monotonic_ns(),
        })
  except Exception as e:
    thread_errors.put(('cereal', f'{type(e).__name__}: {e}'))
    readers_ready['cereal'].set()
    with condition:
      condition.notify_all()

def vision_reader():
  try:
    from msgq.visionipc import VisionIpcClient, VisionStreamType
    client = VisionIpcClient('sidecamerad', VisionStreamType.VISION_STREAM_SIDE_FRONT_LEFT, False)
    if not client.connect(True):
      raise RuntimeError('VisionIPC connect returned false')
    readers_ready['visionipc'].set()
    while not reader_stop.is_set():
      buf = client.recv(timeout_ms=100)
      if buf is None:
        continue
      # Store only sidecar metadata. The frame payload remains in the IPC buffer
      # and is neither sampled, serialized, nor copied into the probe record.
      add_record('visionipc', {
        'frame_id': int(client.frame_id),
        'timestamp_sof_ns': int(client.timestamp_sof),
        'timestamp_eof_ns': int(client.timestamp_eof),
        'valid': bool(client.valid),
        'received_monotonic_ns': time.monotonic_ns(),
      })
      del buf
  except Exception as e:
    thread_errors.put(('visionipc', f'{type(e).__name__}: {e}'))
    readers_ready['visionipc'].set()
    with condition:
      condition.notify_all()

def drain_errors():
  items = []
  while True:
    try:
      items.append(thread_errors.get_nowait())
    except queue.Empty:
      return items

def child_pids_of(pid):
  result = subprocess.run(['ps', '-eo', 'pid=,ppid=,comm='], check=False, capture_output=True, text=True)
  pids = []
  for line in result.stdout.splitlines():
    bits = line.split()
    if len(bits) == 3:
      try:
        if int(bits[1]) == pid:
          pids.append((int(bits[0]), bits[2]))
      except ValueError:
        pass
  return pids

try:
  os.chdir(ROOT)
  before = source_hashes()
  device_resolved = os.path.realpath(DEVICE)
  if before != EXPECTED:
    raise RuntimeError('pre-capture source input hash mismatch')
  if device_resolved != '/dev/video60':
    raise RuntimeError(f'assigned camera resolved unexpectedly: {device_resolved}')
  if ffmpeg_pids():
    raise RuntimeError(f'pre-existing ffmpeg process(es): {ffmpeg_pids()}')
  log(f'PRECHECK source_hashes_match=true source_input_count={len(before)} device={DEVICE} resolves={device_resolved} parent_ZMQ_present={"ZMQ" in os.environ}')
  log(f'CONTRACT steady_state_window_at_least_s=300.0; target_matched_frames={FRAME_TARGET}; interval_mean_fps>=20.0; every_eof_gap_ms<=60; cereal_VisionIPC_frame_ids_must_match; payload_saved=false')
  temp_dir = Path(tempfile.mkdtemp(prefix='sidecam-health-ac4-r2.3-', dir=ROOT / '.harness-state'))
  params_root = temp_dir / 'params'
  params_root.mkdir()
  from openpilot.common.params import Params
  params_obj = Params(str(params_root))
  startup_params = read_params()
  child_env = os.environ.copy()
  child_env['PARAMS_ROOT'] = str(params_root)
  child_env['ZMQ'] = '1'
  child_env['SIDECAM_FRONT_LEFT'] = DEVICE
  for var in ('SIDECAM_FRONT_RIGHT', 'SIDECAM_BACK_LEFT', 'SIDECAM_BACK_RIGHT'):
    child_env.pop(var, None)
  sidecar_stdout = open(temp_dir / 'sidecamerad.stdout.log', 'w', encoding='utf-8')
  command = [sys.executable, 'system/sidecamerad.py']
  proc = subprocess.Popen(command, cwd=ROOT, env=child_env, stdin=subprocess.DEVNULL,
                          stdout=sidecar_stdout, stderr=subprocess.STDOUT, close_fds=True)
  started_monotonic_ns = time.monotonic_ns()
  initial_tcp = port_listeners()
  log(f'SIDECAMERAD_STARTED pid={proc.pid} entry={sys.executable} system/sidecamerad.py isolated_params=true child_ZMQ=1 assigned_device={DEVICE}')
  log(f'STARTUP tcp_listeners_9007_9010={initial_tcp} params={json.dumps(startup_params, sort_keys=True)}')
  threads = [threading.Thread(target=cereal_reader, name='ac4-cereal-reader', daemon=True),
             threading.Thread(target=vision_reader, name='ac4-visionipc-reader', daemon=True)]
  for t in threads:
    t.start()
  ready_deadline = time.monotonic() + 20
  while time.monotonic() < ready_deadline and not all(e.is_set() for e in readers_ready.values()):
    if proc.poll() is not None:
      raise RuntimeError(f'sidecamerad exited before readers ready: {proc.returncode}')
    time.sleep(0.05)
  if not all(e.is_set() for e in readers_ready.values()):
    raise RuntimeError('reader startup timeout')
  errors = drain_errors()
  if errors:
    raise RuntimeError(f'reader setup failed: {errors}')
  log('READERS_READY cereal=frontLeftCameraState conflate=false; VisionIPC=sidecamerad/VISION_STREAM_SIDE_FRONT_LEFT conflate=false; metadata_only=true')
  last_progress = 0
  controller_start = time.monotonic()
  while True:
    if proc.poll() is not None:
      raise RuntimeError(f'sidecamerad exited during capture: {proc.returncode}')
    errors = drain_errors()
    if errors:
      raise RuntimeError(f'reader failed during capture: {errors}')
    with condition:
      cereal_ids = set(records['cereal'])
      vipc_ids = set(records['visionipc'])
      matched_ids = sorted(cereal_ids & vipc_ids)
      if matched_ids and capture_start_eof_ns is None:
        first_id = matched_ids[0]
        capture_start_eof_ns = records['cereal'][first_id]['timestamp_eof_ns']
        capture_started_monotonic_ns = time.monotonic_ns()
        log(f'CAPTURE_START first_matching_frame_id={first_id} cereal_eof_ns={capture_start_eof_ns} visionipc_eof_ns={records["visionipc"][first_id]["timestamp_eof_ns"]} params={json.dumps(read_params(), sort_keys=True)}')
      selected_now = matched_ids[:FRAME_TARGET]
      if selected_now:
        first_eof = records['cereal'][selected_now[0]]['timestamp_eof_ns']
        last_eof = records['cereal'][selected_now[-1]]['timestamp_eof_ns']
        span_ns = last_eof - first_eof
      else:
        span_ns = 0
      elapsed = (time.monotonic_ns() - capture_started_monotonic_ns) / 1e9 if capture_started_monotonic_ns else 0.0
      count_now = len(matched_ids)
      cereal_count = len(cereal_ids)
      vipc_count = len(vipc_ids)
      if count_now >= FRAME_TARGET and span_ns >= STEADY_NS:
        stop_reason = 'target_frames_and_steady_span_reached'
        break
    if capture_started_monotonic_ns and elapsed - last_progress >= 30:
      log(f'PROGRESS elapsed_s={elapsed:.3f} cereal_unique={cereal_count} visionipc_unique={vipc_count} matched_unique={count_now} selected_span_s={span_ns/1e9:.6f}')
      last_progress = elapsed
    if capture_started_monotonic_ns and elapsed >= MAX_WALL_S:
      stop_reason = 'six_minute_safety_limit_reached_before_acceptance_target'
      capture_error = stop_reason
      break
    with condition:
      condition.wait(timeout=0.1)
  with condition:
    matched_ids = sorted(set(records['cereal']) & set(records['visionipc']))
    selected_ids = matched_ids[:FRAME_TARGET]
    elapsed_wall = (time.monotonic_ns() - capture_started_monotonic_ns) / 1e9 if capture_started_monotonic_ns else 0
  log(f'CAPTURE_STOP matched_unique={len(matched_ids)} selected={len(selected_ids)} elapsed_wall_s={elapsed_wall:.6f} reason={stop_reason}')
  pre_shutdown_params = read_params()
  log(f'PRE_SHUTDOWN child_pids={child_pids_of(proc.pid)} tcp_listeners_9007_9010={port_listeners()} params={json.dumps(pre_shutdown_params, sort_keys=True)}')
except Exception as e:
  capture_error = capture_error or f'{type(e).__name__}: {e}'
  stop_reason = stop_reason if stop_reason != 'not-started' else 'preflight_or_capture_error'
  log(f'CAPTURE_ERROR {capture_error}')
finally:
  if proc is not None and proc.poll() is None:
    try:
      proc.send_signal(signal.SIGTERM)
      try:
        child_exit = proc.wait(timeout=10)
      except subprocess.TimeoutExpired:
        proc.kill()
        child_exit = proc.wait(timeout=5)
        log('TEARDOWN escalated_to_SIGKILL=true')
    except Exception as e:
      capture_error = capture_error or f'teardown {type(e).__name__}: {e}'
  elif proc is not None:
    child_exit = proc.poll()
  reader_stop.set()
  for thread in threading.enumerate():
    if thread.name in ('ac4-cereal-reader', 'ac4-visionipc-reader') and thread is not threading.current_thread():
      thread.join(timeout=2)
  post_shutdown_params = read_params() if params_obj is not None else None
  tcp_after = port_listeners()
  remaining_ffmpeg = ffmpeg_pids()
  if proc is not None:
    try:
      proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
      capture_error = capture_error or 'sidecamerad process remained after teardown'
  if sidecar_stdout is not None:
    sidecar_stdout.close()
  log(f'SHUTDOWN sidecamerad_exit={child_exit} child_reaped={proc is None or proc.poll() is not None} tcp_listeners_after={tcp_after} remaining_ffmpeg_pids={remaining_ffmpeg} final_params={json.dumps(post_shutdown_params, sort_keys=True)}')
  if params_root is not None:
    shutil.rmtree(params_root.parent, ignore_errors=True)
    log(f'ISOLATED_PARAMS_REMOVED path={params_root.parent} exists={params_root.parent.exists()}')

# Build immutable metadata-only series and acceptance measurements.
with condition:
  cereal_map = dict(records['cereal'])
  vipc_map = dict(records['visionipc'])
matched_ids = sorted(set(cereal_map) & set(vipc_map))
selected_ids = matched_ids[:FRAME_TARGET]
cereal_selected = [cereal_map[i] for i in selected_ids]
vipc_selected = [vipc_map[i] for i in selected_ids]
intervals = []
for i in range(1, len(selected_ids)):
  prev_id, frame_id = selected_ids[i - 1], selected_ids[i]
  prev_eof = cereal_map[prev_id]['timestamp_eof_ns']
  eof = cereal_map[frame_id]['timestamp_eof_ns']
  delta = eof - prev_eof
  intervals.append({'from_frame_id': prev_id, 'to_frame_id': frame_id,
                    'frame_id_step': (frame_id - prev_id) & 0xFFFFFFFF,
                    'delta_ns': delta, 'delta_ms': delta / 1e6})
deltas = [i['delta_ns'] for i in intervals]
span_ns = cereal_selected[-1]['timestamp_eof_ns'] - cereal_selected[0]['timestamp_eof_ns'] if len(cereal_selected) > 1 else 0
mean_fps = ((len(deltas) * 1e9) / sum(deltas)) if deltas and sum(deltas) > 0 else 0.0
max_gap_ms = max((d / 1e6 for d in deltas), default=0.0)
gaps_over_60 = sum(d > 60_000_000 for d in deltas)
frame_gaps = [{'from': selected_ids[i-1], 'to': selected_ids[i], 'step': (selected_ids[i]-selected_ids[i-1]) & 0xFFFFFFFF}
              for i in range(1, len(selected_ids)) if ((selected_ids[i]-selected_ids[i-1]) & 0xFFFFFFFF) != 1]
timestamp_mismatches = []
for fid in selected_ids:
  c, v = cereal_map[fid], vipc_map[fid]
  if (c['timestamp_eof_ns'] != v['timestamp_eof_ns'] or c['timestamp_sof_ns'] != v['timestamp_sof_ns']
      or c['log_mono_time_ns'] != c['timestamp_eof_ns'] or not c['valid'] or not v['valid']):
    timestamp_mismatches.append({'frame_id': fid, 'cereal': c, 'visionipc': v})
continuous = len(frame_gaps) == 0
cadence_pass = (len(selected_ids) >= FRAME_TARGET and span_ns >= STEADY_NS and mean_fps >= 20.0
                and gaps_over_60 == 0 and continuous and not timestamp_mismatches)
params_transition_pass = bool(startup_params is not None and pre_shutdown_params is not None and post_shutdown_params is not None
  and isinstance(pre_shutdown_params, dict) and isinstance(post_shutdown_params, dict)
  and pre_shutdown_params.get(SERVICE) is True and all(v is False for k, v in post_shutdown_params.items() if k.endswith('CameraState')))
functional_pass = (capture_error is None and child_exit == 0 and not remaining_ffmpeg and not tcp_after
                   and len(selected_ids) >= FRAME_TARGET and continuous and not timestamp_mismatches
                   and params_transition_pass)
after = source_hashes()
drift = {p: {'before': before.get(p), 'after': after.get(p)} for p in EXPECTED if before.get(p) != after.get(p)} if 'before' in globals() else {}
observed = utc_now()
analysis = {
  'ac4_cadence_pass': cadence_pass,
  'selected_matched_frames': len(selected_ids),
  'matched_unique_total': len(matched_ids),
  'cereal_unique': len(cereal_map), 'visionipc_unique': len(vipc_map),
  'cereal_duplicates': duplicates['cereal'], 'visionipc_duplicates': duplicates['visionipc'],
  'frame_id_gaps': len(frame_gaps), 'frame_id_gap_details': frame_gaps,
  'timestamp_mismatches': timestamp_mismatches,
  'mean_interval_fps': mean_fps, 'eof_span_s': span_ns / 1e9,
  'max_eof_gap_ms': max_gap_ms, 'gaps_over_60ms': gaps_over_60,
  'functional_pass': functional_pass, 'runtime_functional_status': 'pass' if functional_pass else 'fail',
  'child_exit': child_exit, 'remaining_ffmpeg_pids': remaining_ffmpeg,
  'tcp_listeners_after_shutdown': tcp_after,
  'params_transition_pass': params_transition_pass,
  'capture_error': capture_error, 'stop_reason': stop_reason,
}
record = {
  'schema_version': 1, 'run_id': RUN_ID, 'submission_id': SUBMISSION,
  'package_input_revision': 68, 'target_revision': 'SideCamera r2.2; source HEAD recorded at capture',
  'target': DEVICE, 'observed_at': observed,
  'capture': {
    'target_matched_frames': FRAME_TARGET, 'actual_matched_frames': len(selected_ids),
    'matched_unique_total': len(matched_ids), 'minimum_steady_state_seconds': 300,
    'first_eof_ns': cereal_selected[0]['timestamp_eof_ns'] if cereal_selected else None,
    'last_eof_ns': cereal_selected[-1]['timestamp_eof_ns'] if cereal_selected else None,
    'eof_span_s': span_ns / 1e9, 'wall_elapsed_s': elapsed_wall if 'elapsed_wall' in globals() else 0,
    'start_frame_id': selected_ids[0] if selected_ids else None,
    'stop_reason': stop_reason,
  },
  'source_hashes_before': before if 'before' in globals() else {},
  'source_hashes_after': after,
  'source_hash_drift': drift,
  'cereal_records': cereal_selected,
  'visionipc_records': vipc_selected,
  'selected_matched_ids': selected_ids,
  'intervals': intervals,
  'param_observations': {
    'startup': startup_params, 'pre_shutdown': pre_shutdown_params, 'post_shutdown': post_shutdown_params,
    'isolated_param_root_removed': params_root is not None and not params_root.parent.exists(),
  },
  'environment': {
    'host': '/data/openpilot', 'device_resolved': os.path.realpath(DEVICE),
    'python': sys.version, 'sidecamerad_command': [sys.executable, 'system/sidecamerad.py'],
    'child_env_changes': {'ZMQ': '1', 'PARAMS_ROOT': 'isolated temporary directory', 'SIDECAM_FRONT_LEFT': DEVICE},
    'payload_saved': False, 'video_recording': False, 'data_dir': '/data/params untouched',
  },
  'analysis': analysis,
}
INTERVAL_PATH.write_text(json.dumps(record, sort_keys=True, separators=(',', ':')) + '\n', encoding='utf-8')
interval_hash = sha(INTERVAL_PATH)
log(f'ANALYSIS selected_frames={len(selected_ids)} matched_total={len(matched_ids)} eof_span_s={span_ns/1e9:.9f} mean_interval_fps={mean_fps:.12f} max_gap_ms={max_gap_ms:.6f} gaps_over_60ms={gaps_over_60} frame_id_gaps={len(frame_gaps)} timestamp_mismatches={len(timestamp_mismatches)} functional_pass={functional_pass} ac4_cadence_pass={cadence_pass} source_hash_drift={json.dumps(drift, sort_keys=True)}')
log(f'RAW_INTERVALS path={INTERVAL_PATH.relative_to(ROOT)} frames={len(selected_ids)} intervals={len(intervals)} sha256={interval_hash}')
log(f'CAPTURE_ERROR {capture_error}' if capture_error else 'CAPTURE_ERROR None')
logf.close()
log_hash = sha(LOG_PATH)
probe = {
  'artifacts': [
    {'path': str(LOG_PATH.relative_to(ROOT)), 'sha256': log_hash},
    {'path': str(INTERVAL_PATH.relative_to(ROOT)), 'sha256': interval_hash},
  ],
  'command': f"Python 3.12 controller ran {sys.executable} system/sidecamerad.py with isolated PARAMS_ROOT and child ZMQ=1 on assigned front-left by-path UVC device; local cereal/msgq and VisionIPC metadata readers captured up to {FRAME_TARGET} matched frame IDs with per-frame EOF timestamps; checked frame IDs/timestamps, Params transition, TCP ports 9007-9010; SIGTERM-stopped process and verified child cleanup.",
  'dependencies': [
    {'name':'V4L2 UVC decoder','status':'reachable' if len(selected_ids) else 'unverified','signal':f'{DEVICE} resolved to {os.path.realpath(DEVICE)}; observed {len(selected_ids)} dual-reader matched frames.'},
    {'name':'FFmpeg','status':'reachable' if len(selected_ids) else 'unverified','signal':f'sidecamerad published frames; exit={child_exit}; remaining ffmpeg pids={remaining_ffmpeg}.'},
    {'name':'cereal/msgq','status':'reachable' if cereal_map else 'unverified','signal':f'{len(cereal_map)} unique camera-state records; selected {len(selected_ids)} IDs matched VisionIPC.'},
    {'name':'VisionIPC','status':'reachable' if vipc_map else 'unverified','signal':f'{len(vipc_map)} unique frames on sidecamerad front-left stream.'},
    {'name':'Params','status':'reachable' if params_transition_pass else 'degraded','signal':f'startup={startup_params}; pre-shutdown={pre_shutdown_params}; post-shutdown={post_shutdown_params}; isolated root removed.'},
    {'name':'local IPC transport','status':'reachable' if not tcp_after else 'degraded','signal':f'listeners on ports 9007-9010 after shutdown: {tcp_after}; stream reader connected.'},
  ],
  'environment': f"Local /data/openpilot host; assigned physical UVC {DEVICE} resolved to {os.path.realpath(DEVICE)}; Python {sys.version.split()[0]}; isolated temporary PARAMS_ROOT removed after shutdown; no video payload saved or recording created; /data/params untouched; no packages installed; source input drift={json.dumps(drift, sort_keys=True)}.",
  'exit_code': child_exit,
  'input_revision': 68,
  'inputs': [{'path':p,'sha256':h} for p,h in after.items()],
  'observed_at': observed,
  'result': {
    'status': 'pass' if functional_pass else 'fail',
    'runtime_functional_status': 'pass' if functional_pass else 'fail',
    'ac4_one_camera_cadence': 'pass' if cadence_pass else 'fail',
    'summary': f"{len(selected_ids)} matched frames over {span_ns/1e9:.9f}s; mean interval {mean_fps:.12f} fps; max EOF gap {max_gap_ms:.6f} ms; {gaps_over_60} intervals >60 ms; {len(frame_gaps)} frame-ID gaps; {len(timestamp_mismatches)} timestamp/validity mismatches.",
    'capture_error': capture_error,
    'stop_reason': stop_reason,
    'functional_summary': f"Sidecamerad exit={child_exit}; unique cereal={len(cereal_map)}, VisionIPC={len(vipc_map)}, matched={len(matched_ids)}; Params transition pass={params_transition_pass}; listeners after shutdown={tcp_after}; FFmpeg children remaining={remaining_ffmpeg}; source drift={drift}.",
    'raw_per_frame_intervals_artifact': str(INTERVAL_PATH.relative_to(ROOT)),
    'param_transition_observations': {'startup_all_false': startup_params is not None and all(v is False for k,v in startup_params.items() if k.endswith('CameraState')),
      'front_left_true_before_shutdown': isinstance(pre_shutdown_params,dict) and pre_shutdown_params.get(SERVICE) is True,
      'all_false_after_shutdown': isinstance(post_shutdown_params,dict) and all(v is False for k,v in post_shutdown_params.items() if k.endswith('CameraState'))},
  },
  'target_revision': 'SideCamera implementation source hash checked against r2.2 package inputs',
  'tool': 'Python controller with local cereal/msgq and VisionIPC metadata readers; launched sidecamerad and its configured FFmpeg child',
  'warnings': [
    f'AC-4 strict cadence {"passes" if cadence_pass else "fails"}; this is the sole five-minute acceptance attempt in this handoff.',
    'AC-6 physical reconnect/sibling continuity remains unverified; only one USB decoder is attached.',
    'AC-8 four-camera 30-minute soak and CPU/resource headroom remain unverified; the FFmpeg resource-cap residual remains open.',
    'No image payload was retained; the capture stores only frame IDs, EOF/SOF timestamps, receive timestamps, Params and cleanup observations.',
  ],
}
PROBE_PATH.write_text(json.dumps(probe, indent=2, sort_keys=True) + '\n', encoding='utf-8')
probe_hash = sha(PROBE_PATH)
interval_hash = sha(INTERVAL_PATH)
report = f'''---\nsubmission_id: {SUBMISSION}\nrevision: "build-ac4-r2.3"\nrun_id: {RUN_ID}\n---\n\n# Runtime health — AC-4 five-minute acceptance rerun\n\n**Result:** runtime functional **{analysis["runtime_functional_status"]}**; strict AC-4 cadence **{"pass" if cadence_pass else "fail"}**. This was one run on the assigned front-left UVC device using unchanged product source and default sidecamerad FFmpeg argv.\n\n## Acceptance measurement\n\n- Capture window: {span_ns/1e9:.9f} seconds of EOF-to-EOF steady-state data, {elapsed_wall if 'elapsed_wall' in globals() else 0:.6f} seconds wall elapsed from the first matching frame.\n- Matched cereal/VisionIPC frames selected: {len(selected_ids)} (target {FRAME_TARGET}); total unique matched IDs observed: {len(matched_ids)}.\n- Mean EOF interval rate: {mean_fps:.12f} fps; maximum gap: {max_gap_ms:.6f} ms; intervals over 60 ms: {gaps_over_60} of {len(intervals)}.\n- Frame-ID gaps: {len(frame_gaps)}; cereal/VisionIPC EOF/SOF/validity mismatches: {len(timestamp_mismatches)}.\n- AC-4 threshold: >=6,000 successfully published frames over >=300 seconds, mean >=20.0 fps, every EOF interval <=60 ms, matching cereal/VisionIPC frame IDs. Outcome: **{"PASS" if cadence_pass else "FAIL"}**.\n\n## Runtime and cleanup\n\n- Assigned by-path target `{DEVICE}` resolved to `{os.path.realpath(DEVICE)}`. V4L2 mode was MJPG 1280x720 at 20 fps. Only the front-left source was configured.\n- Source inputs were verified against the saved package before capture and after shutdown; drift: `{json.dumps(drift, sort_keys=True)}`.\n- Startup Params: `{json.dumps(startup_params, sort_keys=True)}`; pre-shutdown: `{json.dumps(pre_shutdown_params, sort_keys=True)}`; post-shutdown: `{json.dumps(post_shutdown_params, sort_keys=True)}`. Param transition check: {params_transition_pass}.\n- Sidecamerad exit code: {child_exit}; remaining FFmpeg PIDs: `{remaining_ffmpeg}`; TCP listeners on 9007–9010 after shutdown: `{tcp_after}`. Isolated Params root removed: {params_root is not None and not params_root.parent.exists()}.\n- The controller retained metadata only: frame IDs, timestamps, receive times, Params, and cleanup status. It saved no image payload or video. `/data/params` was untouched.\n\n## Evidence and limits\n\n- Raw per-frame matched IDs and EOF intervals: `{INTERVAL_PATH.relative_to(ROOT)}` (SHA-256 `{interval_hash}`).\n- Runtime smoke log: `{LOG_PATH.relative_to(ROOT)}` (SHA-256 `{log_hash}`).\n- Typed probe: `{PROBE_PATH.relative_to(ROOT)}` (SHA-256 `{probe_hash}`).\n- AC-6 physical reconnect/sibling continuity remains unverified because only one USB decoder is attached. AC-8 four-camera 30-minute soak and resource headroom remain unverified; the FFmpeg resource-cap residual remains open.\n- {"AC-4 passed the exact single-camera cadence criteria for this capture." if cadence_pass else "AC-4 failed the strict cadence criterion; preserve these measurements and route the unresolved mechanism to Admiral/investigation. No speculative source fix is made."}\n'''
REPORT_PATH.write_text(report, encoding='utf-8')
report_hash = sha(REPORT_PATH)
print(f'ARTIFACTS report={REPORT_PATH} sha256={report_hash}', flush=True)
print(f'ARTIFACTS log={LOG_PATH} sha256={log_hash}', flush=True)
print(f'ARTIFACTS intervals={INTERVAL_PATH} sha256={interval_hash}', flush=True)
print(f'ARTIFACTS probe={PROBE_PATH} sha256={probe_hash}', flush=True)
print(f'FINAL frames={len(selected_ids)} span_s={span_ns/1e9:.9f} mean_fps={mean_fps:.12f} max_gap_ms={max_gap_ms:.6f} over_60ms={gaps_over_60}/{len(intervals)} cadence_pass={cadence_pass} functional_pass={functional_pass} cleanup_exit={child_exit} remaining_ffmpeg={remaining_ffmpeg}', flush=True)

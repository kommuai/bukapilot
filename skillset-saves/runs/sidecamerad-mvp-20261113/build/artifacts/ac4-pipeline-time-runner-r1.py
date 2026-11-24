#!/usr/bin/env python3
"""Bounded metadata-only AC-4 FFmpeg pipeline timing capture; never writes frame bytes."""
from __future__ import annotations

import hashlib
import json
import os
import select
import signal
import struct
import subprocess
import time
from pathlib import Path

ROOT = Path('/data/openpilot')
RUN = ROOT / 'skillset-saves/runs/sidecamerad-mvp-20261113'
OBSERVER = RUN / 'build/artifacts/ac4-dqbuf-write-observer-r1.so'
SCALE_OBSERVER = RUN / 'build/artifacts/ac4-scale-time-observer-r1.so'
PIPELINE_OBSERVER = RUN / 'build/artifacts/ac4-pipeline-time-observer-r1.so'
PIPELINE_RING = RUN / 'build/evidence/ac4-pipeline-time-api-observer-r1.bin'
SCALE_RING = RUN / 'build/evidence/ac4-pipeline-time-scale-observer-r1.bin'
FFMPEG_OBSERVER_RING = RUN / 'build/evidence/ac4-pipeline-time-ffmpeg-observer-r1.bin'
PIPELINE_RING_MAGIC = 0x414334504950454C
SCALE_RING_MAGIC = 0x4143345343414C45
OBSERVER_RING_MAGIC = 0x4143345452414345
DEVICE_BY_PATH = Path('/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0')
DEVICE = Path('/dev/video60')
WIDTH, HEIGHT, FPS = 1280, 720, 20
FRAME_BYTES = WIDTH * HEIGHT * 3 // 2
WARMUP_FRAMES = 20
MAX_FRAMES = 600
GAP_LIMIT_NS = 60_000_000
FRAME_TIMEOUT_S = 3.0

ARGV = [
    '/usr/bin/ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
    '-thread_queue_size', '1', '-threads', '1', '-filter_threads', '1',
    '-f', 'v4l2', '-input_format', 'mjpeg', '-framerate', str(FPS),
    '-video_size', f'{WIDTH}x{HEIGHT}', '-i', str(DEVICE),
    '-an', '-sn', '-dn', '-vf', f'scale={WIDTH}:{HEIGHT}:flags=neighbor,format=nv12',
    '-pix_fmt', 'nv12', '-threads', '1', '-fps_mode', 'passthrough',
    '-f', 'rawvideo', 'pipe:1',
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def signed64(value: int) -> int:
    return value if value < (1 << 63) else value - (1 << 64)


def read_frame(fd: int, buffer: bytearray) -> tuple[int, int]:
    view = memoryview(buffer)
    received = 0
    deadline = time.monotonic() + FRAME_TIMEOUT_S
    while received < len(buffer):
        timeout = deadline - time.monotonic()
        if timeout <= 0:
            raise TimeoutError(f'no complete frame within {FRAME_TIMEOUT_S:.1f}s')
        ready, _, _ = select.select([fd], [], [], timeout)
        if not ready:
            raise TimeoutError(f'no complete frame within {FRAME_TIMEOUT_S:.1f}s')
        try:
            size = os.readv(fd, [view[received:]])
        except BlockingIOError:
            continue
        if size == 0:
            return received, time.monotonic_ns()
        received += size
    return received, time.monotonic_ns()


def stop_child(proc: subprocess.Popen[bytes]) -> tuple[int | None, str]:
    method = 'already-exited'
    if proc.poll() is None:
        method = 'SIGINT then close reader pipe'
        try:
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
        if proc.stdout is not None:
            proc.stdout.close()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            method = 'SIGINT, pipe close, then SIGTERM'
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                method = 'SIGINT, pipe close, SIGTERM, then SIGKILL'
                proc.kill()
                proc.wait(timeout=2.0)
    elif proc.stdout is not None:
        proc.stdout.close()
    return proc.returncode, method


def parse_old_ring(path: Path) -> tuple[list[dict], dict]:
    header = struct.Struct('<QQQQ')
    event_struct = struct.Struct('<IIiiQQQQQQ')
    data = path.read_bytes()
    if len(data) < header.size:
        raise RuntimeError(f'observer ring missing or short: {path}')
    magic, count, overflow, stdout_bytes = header.unpack_from(data)
    if magic != OBSERVER_RING_MAGIC:
        raise RuntimeError(f'unexpected observer ring magic: {magic:#x}')
    count = min(count, (len(data) - header.size) // event_struct.size)
    names = {1: 'v4l2_dqbuf', 2: 'stdout_write', 3: 'stdout_writev', 4: 'decoder', 5: 'filter_sink', 6: 'mux', 7: 'avio'}
    events = []
    for index in range(count):
        kind, tid, fd, result, begin, end, a, b, c, d = event_struct.unpack_from(data, header.size + index * event_struct.size)
        event = {'record_type': names.get(kind, f'event_{kind}'), 'tid': tid, 'fd': fd, 'result': result,
                 'begin_mono_ns': begin, 'end_mono_ns': end, 'a': a, 'b': b, 'c': c, 'd': d}
        if kind in (4, 5):
            event.update(pts=signed64(a), best_effort_pts=signed64(b), pkt_dts=signed64(c))
        elif kind == 6:
            event.update(pts=signed64(a), dts=signed64(b), packet_bytes=c)
        elif kind == 1:
            event.update(sequence=a, source_timestamp_ns=b, buffer_index=c >> 32, bytes_used=c & 0xFFFFFFFF, flags=d)
        events.append(event)
    return events, {'magic': magic, 'events': count, 'overflow': overflow, 'stdout_bytes_recorded': stdout_bytes}


def parse_scale_ring(path: Path) -> tuple[list[dict], dict]:
    header = struct.Struct('<QQQ')
    event_struct = struct.Struct('<IIiiQQQQqq')
    data = path.read_bytes()
    if len(data) < header.size:
        raise RuntimeError(f'scale observer ring missing or short: {path}')
    magic, count, overflow = header.unpack_from(data)
    if magic != SCALE_RING_MAGIC:
        raise RuntimeError(f'unexpected scale ring magic: {magic:#x}')
    count = min(count, (len(data) - header.size) // event_struct.size)
    names = {1: 'sws_scale_frame', 2: 'sws_scale'}
    events = []
    for index in range(count):
        kind, tid, result, _reserved, begin, end, begin_cpu, end_cpu, src_pts, dst_pts = event_struct.unpack_from(data, header.size + index * event_struct.size)
        events.append({'record_type': names.get(kind, f'scale_event_{kind}'), 'tid': tid, 'result': result,
                       'begin_mono_ns': begin, 'end_mono_ns': end, 'begin_thread_cpu_ns': begin_cpu,
                       'end_thread_cpu_ns': end_cpu, 'src_pts': src_pts, 'dst_pts': dst_pts,
                       'wall_ms': (end - begin) / 1e6, 'thread_cpu_ms': (end_cpu - begin_cpu) / 1e6})
    return events, {'magic': magic, 'events': count, 'overflow': overflow}


def parse_pipeline_ring(path: Path) -> tuple[list[dict], dict]:
    header = struct.Struct('<QQQ')
    event_struct = struct.Struct('<IIiiQQQQqqq')
    data = path.read_bytes()
    if len(data) < header.size:
        raise RuntimeError(f'pipeline observer ring missing or short: {path}')
    magic, count, overflow = header.unpack_from(data)
    if magic != PIPELINE_RING_MAGIC:
        raise RuntimeError(f'unexpected pipeline ring magic: {magic:#x}')
    count = min(count, (len(data) - header.size) // event_struct.size)
    names = {1: 'av_read_frame', 2: 'avcodec_send_packet', 3: 'avcodec_receive_frame',
             4: 'buffersrc_add_frame', 5: 'buffersink_get_frame', 6: 'av_interleaved_write_frame',
             7: 'avio_write'}
    events = []
    for index in range(count):
        kind, tid, result, flags, begin, end, begin_cpu, end_cpu, pts, dts, value = event_struct.unpack_from(data, header.size + index * event_struct.size)
        events.append({'record_type': names.get(kind, f'pipeline_event_{kind}'), 'tid': tid,
                       'result': result, 'flags': flags, 'begin_mono_ns': begin, 'end_mono_ns': end,
                       'begin_thread_cpu_ns': begin_cpu, 'end_thread_cpu_ns': end_cpu,
                       'pts': pts, 'dts': dts, 'value': value,
                       'wall_ms': (end - begin) / 1e6,
                       'thread_cpu_ms': (end_cpu - begin_cpu) / 1e6})
    return events, {'magic': magic, 'events': count, 'overflow': overflow}


def nearest_rank(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int((percentile / 100.0) * len(ordered) + 0.999999) - 1))
    return ordered[index]


def main() -> int:
    if DEVICE_BY_PATH.resolve(strict=True) != DEVICE.resolve(strict=True):
        raise RuntimeError(f'camera mapping changed: {DEVICE_BY_PATH} -> {DEVICE_BY_PATH.resolve()}')
    expected_hashes = {
        '/usr/bin/ffmpeg': '9af5a141b45d32f64fe7bb5cd2c939d56fb9972234705513c45575ef7a1da38e',
        str(ROOT / 'system/sidecamerad.py'): '494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b',
    }
    before_hashes = {path: sha256(Path(path)) for path in expected_hashes}
    if before_hashes != expected_hashes:
        raise RuntimeError(f'production input hash drift: {before_hashes}')
    for ring_path in (PIPELINE_RING, SCALE_RING, FFMPEG_OBSERVER_RING):
        if ring_path.exists():
            raise RuntimeError(f'refusing to overwrite existing evidence: {ring_path}')

    output_frame_events = []
    trigger = None
    partial_bytes = 0
    capture_error = None
    started_ns = time.monotonic_ns()
    env = os.environ.copy()
    env['LD_PRELOAD'] = f'{PIPELINE_OBSERVER}:{SCALE_OBSERVER}:{OBSERVER}'
    env['AC4_PIPELINE_RING'] = str(PIPELINE_RING)
    env['AC4_SCALE_RING'] = str(SCALE_RING)
    env['AC4_OBSERVER_RING'] = str(FFMPEG_OBSERVER_RING)
    proc = subprocess.Popen(ARGV, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, bufsize=0, close_fds=True, env=env)
    assert proc.stdout is not None
    os.set_blocking(proc.stdout.fileno(), False)
    reusable_frame = bytearray(FRAME_BYTES)
    previous_eof = None
    stop_reason = 'not-stopped'
    try:
        while len(output_frame_events) < MAX_FRAMES:
            byte_count, eof_ns = read_frame(proc.stdout.fileno(), reusable_frame)
            if byte_count == 0:
                stop_reason = 'ffmpeg stdout EOF'
                break
            if byte_count != FRAME_BYTES:
                partial_bytes = byte_count
                stop_reason = 'partial raw frame at EOF'
                break
            frame_no = len(output_frame_events) + 1
            interval_ns = None if previous_eof is None else eof_ns - previous_eof
            record = {'record_type': 'reader_frame', 'frame_no': frame_no, 'bytes': byte_count,
                      'eof_mono_ns': eof_ns, 'gap_ms': None if interval_ns is None else interval_ns / 1e6,
                      'warmup': frame_no <= WARMUP_FRAMES}
            output_frame_events.append(record)
            previous_eof = eof_ns
            if frame_no > WARMUP_FRAMES and interval_ns is not None and interval_ns > GAP_LIMIT_NS:
                trigger = {'from_frame': frame_no - 1, 'to_frame': frame_no, 'gap_ms': interval_ns / 1e6,
                           'from_eof_mono_ns': output_frame_events[-2]['eof_mono_ns'], 'to_eof_mono_ns': eof_ns}
                stop_reason = 'first post-warmup complete EOF gap over 60 ms'
                break
        else:
            stop_reason = '600 complete output frame cap reached'
    except (TimeoutError, OSError) as exc:
        capture_error = f'{type(exc).__name__}: {exc}'
        stop_reason = 'capture error'
    finally:
        returncode, cleanup_method = stop_child(proc)

    pipeline_events, pipeline_ring_info = parse_pipeline_ring(PIPELINE_RING)
    old_events, old_ring_info = parse_old_ring(FFMPEG_OBSERVER_RING)
    scale_events, scale_ring_info = parse_scale_ring(SCALE_RING)
    records = output_frame_events + old_events + scale_events + pipeline_events
    records.sort(key=lambda item: item.get('end_mono_ns', item.get('eof_mono_ns', 0)))
    jsonl_path = RUN / 'build/evidence/ac4-pipeline-time-diagnostic-r1.jsonl'
    with jsonl_path.open('w') as stream:
        for record in records:
            stream.write(json.dumps(record, sort_keys=True, separators=(',', ':')) + '\n')

    source_events = sorted((e for e in old_events if e['record_type'] == 'v4l2_dqbuf'), key=lambda e: e['end_mono_ns'])
    decoder_events = sorted((e for e in old_events if e['record_type'] == 'decoder'), key=lambda e: e['end_mono_ns'])
    filter_events = sorted((e for e in old_events if e['record_type'] == 'filter_sink'), key=lambda e: e['end_mono_ns'])
    mux_events = sorted((e for e in old_events if e['record_type'] == 'mux'), key=lambda e: e['end_mono_ns'])
    frame_scale = [e for e in scale_events if e['record_type'] == 'sws_scale_frame']
    slice_scale = [e for e in scale_events if e['record_type'] == 'sws_scale']
    pipeline_stats = {}
    for name in sorted({e['record_type'] for e in pipeline_events}):
        values = [e for e in pipeline_events if e['record_type'] == name]
        wall = [e['wall_ms'] for e in values]
        cpu = [e['thread_cpu_ms'] for e in values]
        pipeline_stats[name] = {
            'calls': len(values), 'successful_calls': sum(e['result'] >= 0 for e in values),
            'wall_ms_mean': (sum(wall) / len(wall)) if wall else None,
            'wall_ms_p50': nearest_rank(wall, 50), 'wall_ms_p95': nearest_rank(wall, 95),
            'wall_ms_max': max(wall) if wall else None,
            'thread_cpu_ms_mean': (sum(cpu) / len(cpu)) if cpu else None,
            'thread_cpu_ms_p95': nearest_rank(cpu, 95), 'thread_cpu_ms_max': max(cpu) if cpu else None,
        }
    reader_intervals = [e['gap_ms'] for e in output_frame_events if e['gap_ms'] is not None and e['frame_no'] > WARMUP_FRAMES]
    scale_stats = {}
    for name, values in (('sws_scale_frame', frame_scale), ('sws_scale', slice_scale)):
        wall = [e['wall_ms'] for e in values]
        cpu = [e['thread_cpu_ms'] for e in values]
        scale_stats[name] = {'calls': len(values), 'wall_ms_mean': (sum(wall) / len(wall)) if wall else None,
                             'wall_ms_p50': nearest_rank(wall, 50), 'wall_ms_p95': nearest_rank(wall, 95),
                             'wall_ms_p99': nearest_rank(wall, 99), 'wall_ms_max': max(wall) if wall else None,
                             'thread_cpu_ms_mean': (sum(cpu) / len(cpu)) if cpu else None,
                             'thread_cpu_ms_p95': nearest_rank(cpu, 95), 'thread_cpu_ms_max': max(cpu) if cpu else None}

    trigger_timeline = None
    if trigger is not None:
        frame_to = trigger['to_frame']
        trigger_mux = mux_events[frame_to - 1] if len(mux_events) >= frame_to else None
        target_pts = trigger_mux['pts'] if trigger_mux is not None else None
        target_filter = next((e for e in reversed(filter_events) if target_pts is not None and e['end_mono_ns'] <= (trigger_mux['end_mono_ns'] if trigger_mux else 0)), None)
        target_decoder = next((e for e in reversed(decoder_events) if target_filter and e['pts'] == target_filter['pts']), None)
        target_scale = [e for e in frame_scale if target_decoder and e['src_pts'] == target_decoder['pts']]
        target_pipeline = [e for e in pipeline_events if target_decoder and e['pts'] == target_decoder['pts']]
        trigger_timeline = {
            'reader_from_frame': trigger['from_frame'], 'reader_to_frame': frame_to,
            'reader_gap_ms': trigger['gap_ms'],
            'mux_pts': target_pts,
            'filter_sink_pts_near_mux': target_filter['pts'] if target_filter else None,
            'decoder_pts_matching_filter': target_decoder['pts'] if target_decoder else None,
            'matching_sws_scale_frame': [{'src_pts': e['src_pts'], 'dst_pts': e['dst_pts'], 'wall_ms': e['wall_ms'], 'thread_cpu_ms': e['thread_cpu_ms'], 'wall_minus_cpu_ms': e['wall_ms'] - e['thread_cpu_ms']} for e in target_scale],
            'pipeline_calls_matching_decoder_pts': [{'record_type': e['record_type'], 'result': e['result'], 'begin_mono_ns': e['begin_mono_ns'], 'end_mono_ns': e['end_mono_ns'], 'wall_ms': e['wall_ms'], 'thread_cpu_ms': e['thread_cpu_ms'], 'pts': e['pts'], 'dts': e['dts'], 'value': e['value']} for e in target_pipeline],
            'pipeline_calls_near_trigger_eof': [{'record_type': e['record_type'], 'result': e['result'], 'begin_mono_ns': e['begin_mono_ns'], 'end_mono_ns': e['end_mono_ns'], 'wall_ms': e['wall_ms'], 'thread_cpu_ms': e['thread_cpu_ms'], 'pts': e['pts'], 'dts': e['dts'], 'value': e['value']} for e in pipeline_events if abs(e['end_mono_ns'] - trigger['to_eof_mono_ns']) < 10_000_000],
            'matching_direct_sws_scale_calls_by_time': [{'wall_ms': e['wall_ms'], 'thread_cpu_ms': e['thread_cpu_ms'], 'wall_minus_cpu_ms': e['wall_ms'] - e['thread_cpu_ms']} for e in slice_scale if abs(e['end_mono_ns'] - trigger['to_eof_mono_ns']) < 20_000_000],
        }

    after_hashes = {path: sha256(Path(path)) for path in expected_hashes}
    summary = {
        'schema_version': 1,
        'execution': {'argv': ARGV, 'ffmpeg_sha256_before_after': [before_hashes['/usr/bin/ffmpeg'], after_hashes['/usr/bin/ffmpeg']],
                      'sidecamerad_sha256_before_after': [before_hashes[str(ROOT / 'system/sidecamerad.py')], after_hashes[str(ROOT / 'system/sidecamerad.py')]],
                      'product_source_drift': before_hashes != after_hashes, 'payload_saved': False,
                      'device_by_path': str(DEVICE_BY_PATH), 'device_by_path_resolved': str(DEVICE_BY_PATH.resolve()),
                      'device_format': f'{WIDTH}x{HEIGHT} MJPEG', 'requested_fps': FPS, 'output_format': '1280x720 NV12 rawvideo',
                      'warmup_frames': WARMUP_FRAMES, 'max_complete_frames': MAX_FRAMES,
                      'complete_output_frames': len(output_frame_events), 'partial_frame_bytes': partial_bytes,
                      'capture_error': capture_error, 'stop_reason': stop_reason, 'trigger': trigger,
                      'process_returncode': returncode, 'cleanup_method': cleanup_method,
                      'capture_elapsed_s': (time.monotonic_ns() - started_ns) / 1e9},
        'reader': {'post_warmup_intervals': len(reader_intervals), 'gaps_over_60ms': sum(v > 60.0 for v in reader_intervals),
                   'max_gap_ms': max(reader_intervals) if reader_intervals else None,
                   'mean_gap_ms': sum(reader_intervals) / len(reader_intervals) if reader_intervals else None},
        'rings': {'ffmpeg_observer': old_ring_info, 'scale_observer': scale_ring_info},
        'stage_counts': {'v4l2_dqbuf': len(source_events), 'decoder': len(decoder_events), 'filter_sink': len(filter_events),
                         'mux': len(mux_events), 'scale_frame': len(frame_scale), 'scale_slice': len(slice_scale)},
        'scale_call_statistics': scale_stats,
        'pipeline_call_statistics': pipeline_stats,
        'trigger_timeline': trigger_timeline,
        'limitations': ['LD_PRELOAD observer overhead is not calibrated.', 'The direct sws_scale API does not include frame PTS; its nearest-time attribution is indicative.',
                        'A single instrumented capture localizes and compares work-versus-wait timing but is not an AC-4 acceptance run.'],
        'evidence_paths': {'jsonl': str(jsonl_path.relative_to(ROOT)),
                           'ffmpeg_ring': str(FFMPEG_OBSERVER_RING.relative_to(ROOT)),
                           'scale_ring': str(SCALE_RING.relative_to(ROOT)),
                           'pipeline_ring': str(PIPELINE_RING.relative_to(ROOT))},
    }
    summary['rings']['pipeline_observer'] = pipeline_ring_info
    summary['stage_counts'].update({name: sum(e['record_type'] == name for e in pipeline_events)
                                    for name in pipeline_stats})
    summary_path = RUN / 'build/artifacts/ac4-pipeline-time-diagnostic-r1.json'
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
    print(json.dumps({'frames': len(output_frame_events), 'trigger': trigger, 'stop_reason': stop_reason,
                      'scale_calls': {'sws_scale_frame': len(frame_scale), 'sws_scale': len(slice_scale)},
                      'scale_stats': scale_stats, 'pipeline_stats': pipeline_stats,
                      'trigger_timeline': trigger_timeline,
                      'cleanup_method': cleanup_method, 'returncode': returncode,
                      'capture_error': capture_error, 'summary': str(summary_path.relative_to(ROOT))}, indent=2))
    return 0 if output_frame_events and capture_error is None else 2


if __name__ == '__main__':
    raise SystemExit(main())

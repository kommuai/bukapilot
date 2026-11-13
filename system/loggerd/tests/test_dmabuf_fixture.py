import fcntl
import gc
import os
from pathlib import Path
import platform
import re
import select
import shutil
import struct
import subprocess
import time
import uuid

import pytest

from msgq.visionipc import VisionIpcClient, VisionStreamType
from openpilot.system.hardware import KA2


WIDTH = 1920
HEIGHT = 1200
STRIDE = (WIDTH + 15) & ~15
UV_OFFSET = STRIDE * HEIGHT
BUFFER_SIZE = UV_OFFSET * 3 // 2
BUFFERS_PER_STREAM = 4

DMA_BUF_IOCTL_SYNC = 0x40086200
DMA_BUF_SYNC_READ = 1
DMA_BUF_SYNC_END = 4

STREAMS = (
  (VisionStreamType.VISION_STREAM_ROAD, 0),
  (VisionStreamType.VISION_STREAM_DRIVER, 1),
  (VisionStreamType.VISION_STREAM_WIDE_ROAD, 2),
)


def _test_environment(tmp_path: Path) -> tuple[dict[str, str], str]:
  root = tmp_path.resolve()
  env = os.environ.copy()
  env["LOG_ROOT"] = str(root / "logs")
  env["PARAMS_ROOT"] = str(root / "params")
  prefix = f"loggerd_dmabuf_{os.getpid()}_{uuid.uuid4().hex[:8]}"
  env["OPENPILOT_PREFIX"] = prefix
  env.pop("ZMQ", None)
  env.pop("CEREAL_FAKE", None)
  for key in ("LOG_ROOT", "PARAMS_ROOT"):
    path = Path(env[key]).resolve()
    assert path.is_relative_to(root), f"{key} escaped isolated test root: {path}"
    path.mkdir(parents=True, exist_ok=True)
  assert "/" not in prefix and ".." not in prefix
  return env, prefix


def _wait_line(process: subprocess.Popen[str], timeout: float) -> str:
  assert process.stdout is not None
  readable, _, _ = select.select([process.stdout], [], [], timeout)
  if not readable:
    raise AssertionError(f"publisher did not produce output within {timeout}s; returncode={process.poll()}")
  line = process.stdout.readline()
  assert line, f"publisher closed stdout; returncode={process.poll()}"
  return line.rstrip("\n")


def _wait_matching_line(process: subprocess.Popen[str], expected: str, timeout: float) -> str:
  deadline = time.monotonic() + timeout
  lines = []
  while time.monotonic() < deadline:
    line = _wait_line(process, max(0.0, deadline - time.monotonic()))
    lines.append(line)
    if line == expected:
      return line
    if line.startswith("ERROR") or line.startswith("VisionIPC DMA-BUF fixture failed:"):
      raise AssertionError(f"publisher reported an error: {line}")
  raise AssertionError(f"publisher never printed {expected!r}; output={lines!r}")


def _send_frame(process: subprocess.Popen[str], stream: int, frame_id: int,
                timestamp_sof: int | None = None, timestamp_eof: int | None = None) -> None:
  assert process.stdin is not None
  timestamp_sof = frame_id * 1000 if timestamp_sof is None else timestamp_sof
  timestamp_eof = timestamp_sof + 500 if timestamp_eof is None else timestamp_eof
  process.stdin.write(f"FRAME {stream} {frame_id} {timestamp_sof} {timestamp_eof}\n")
  process.stdin.flush()
  _wait_matching_line(process, f"SENT {stream} {frame_id}", 3.0)


def _recv_frame(client: VisionIpcClient, timeout_ms: int = 2000):
  deadline = time.monotonic() + timeout_ms / 1000
  while time.monotonic() < deadline:
    frame = client.recv(50)
    if frame is not None:
      return frame
  raise AssertionError(f"VisionIPC receive timed out on stream {client}")


def _pattern(stream: int, frame_id: int) -> tuple[int, int, int]:
  return (
    16 + ((frame_id * 13 + stream * 29) % 200),
    32 + ((frame_id * 7 + stream * 17) % 190),
    32 + ((frame_id * 11 + stream * 23) % 190),
  )


def _sync_for_cpu_read(fd: int) -> None:
  fcntl.ioctl(fd, DMA_BUF_IOCTL_SYNC, struct.pack("Q", DMA_BUF_SYNC_READ))


def _sync_for_cpu_read_end(fd: int) -> None:
  fcntl.ioctl(fd, DMA_BUF_IOCTL_SYNC, struct.pack("Q", DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ))


def _fd_is_open(fd: int) -> bool:
  try:
    fcntl.fcntl(fd, fcntl.F_GETFD)
    return True
  except OSError:
    return False


def _mapping_exists(address: int) -> bool:
  with open("/proc/self/maps") as maps:
    for line in maps:
      interval = line.split(maxsplit=1)[0]
      start, end = (int(part, 16) for part in interval.split("-", 1))
      if start <= address < end:
        return True
  return False


def _assert_camera_stack_idle() -> None:
  pgrep = shutil.which("pgrep")
  fuser = shutil.which("fuser")
  assert pgrep is not None and fuser is not None, "pgrep and fuser are required for the board safety preflight"

  for name in ("camerad", "encoderd", "loggerd"):
    result = subprocess.run([pgrep, "-x", name], text=True, capture_output=True, check=False)
    assert result.returncode == 1, f"camera/recording process {name} is active or process check failed: {result.stdout}{result.stderr}"
    print(f"preflight pgrep -x {name}: no active process")

  device_pattern = re.compile(r"^(video.*|media.*|v4l-subdev.*|mpp_service|rga)$")
  device_nodes = [path for path in Path("/dev").iterdir() if device_pattern.match(path.name) and path.is_char_device()]
  assert device_nodes, "no KA2 video/MPP device nodes found; target is not the expected board"
  for path in sorted(device_nodes):
    result = subprocess.run([fuser, "-v", str(path)], text=True, capture_output=True, check=False)
    print(f"preflight fuser -v {path}: exit={result.returncode} {result.stdout}{result.stderr}", end="")
    assert result.returncode == 1, f"device {path} is in use or fuser failed: {result.stdout}{result.stderr}"


@pytest.mark.xdist_group("loggerd_tests")
def test_visionipc_dmabuf_fixture(tmp_path, monkeypatch):
  if not KA2 or platform.machine() != "aarch64" or not Path("/dev/dma_heap/system").exists():
    pytest.skip("fixture qualification requires the pinned KA2 DMA-heap device")

  _assert_camera_stack_idle()

  repo_root = Path(__file__).resolve().parents[3]
  scons = shutil.which("scons")
  assert scons is not None, "SCons is required to build the DMA-BUF VisionIPC publisher"
  target = "build/system/loggerd/tests/visionipc_dmabuf_publisher"
  binary = repo_root / target
  build_command = [scons, "-j2", target]
  print("fixture compile command:", build_command)
  compiled = subprocess.run(build_command, cwd=repo_root, text=True, capture_output=True, check=False)
  print(compiled.stdout, end="")
  print(compiled.stderr, end="")
  assert compiled.returncode == 0, f"fixture compile exited {compiled.returncode}"
  assert binary.is_file(), f"SCons did not produce {binary}"

  env, prefix = _test_environment(tmp_path)
  # MSGQ maps queues beneath /dev/shm/msgq_<prefix>; it does not create this
  # directory itself. OpenpilotPrefix normally owns that setup, but its cleanup
  # also removes unrelated roots, so keep the fixture's queue directory local
  # and remove only its three known endpoints below.
  msgq_dir = Path("/dev/shm") / f"msgq_{prefix}"
  assert not msgq_dir.exists(), f"isolated MSGQ directory already exists: {msgq_dir}"
  msgq_dir.mkdir(mode=0o700)
  socket_path = Path(f"/tmp/{prefix}_visionipc_camerad")
  process = None
  owns_socket_path = False
  clients: dict[int, VisionIpcClient] = {}
  received_fds: list[int] = []
  received_fd_identities: list[tuple[int, int]] = []
  client_mapping_addresses: list[int] = []
  try:
    for key in ("LOG_ROOT", "PARAMS_ROOT", "OPENPILOT_PREFIX"):
      monkeypatch.setenv(key, env[key])
    monkeypatch.delenv("ZMQ", raising=False)
    monkeypatch.delenv("CEREAL_FAKE", raising=False)
    assert not socket_path.exists(), f"isolated fixture socket already exists: {socket_path}"

    _assert_camera_stack_idle()
    process = subprocess.Popen([str(binary), "--serve"], cwd=repo_root, env=env,
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, bufsize=1)
    owns_socket_path = True
    ready = _wait_matching_line(process,
                                f"READY streams=road,driver,wideRoad width={WIDTH} height={HEIGHT} "
                                f"stride={STRIDE} uv_offset={UV_OFFSET} buffers={BUFFERS_PER_STREAM} backing=dma_heap", 10.0)
    assert ready

    for stream_type, stream_id in STREAMS:
      client = VisionIpcClient("camerad", stream_type, False)
      deadline = time.monotonic() + 5.0
      connected = False
      while time.monotonic() < deadline:
        if client.connect(False):
          connected = True
          break
        time.sleep(0.01)
      assert connected, f"could not connect to isolated stream {stream_id}"
      assert client.num_buffers == BUFFERS_PER_STREAM
      assert client.width == WIDTH and client.height == HEIGHT
      assert client.stride == STRIDE and client.uv_offset == UV_OFFSET
      assert client.buffer_len == BUFFER_SIZE
      clients[stream_id] = client

    observed_y_values: dict[int, set[int]] = {stream_id: set() for _, stream_id in STREAMS}
    for stream_type, stream_id in STREAMS:
      client = clients[stream_id]
      observed_indices = set()
      for n in range(BUFFERS_PER_STREAM):
        frame_id = 100 + stream_id * 10 + n
        _send_frame(process, stream_id, frame_id)
        received = _recv_frame(client)
        assert received is not None
        assert received.width == WIDTH and received.height == HEIGHT
        assert received.stride == STRIDE and received.uv_offset == UV_OFFSET
        assert received.fd >= 0 and fcntl.fcntl(received.fd, fcntl.F_GETFD) >= 0
        assert received.idx == frame_id % BUFFERS_PER_STREAM
        assert client.valid and client.frame_id == frame_id
        assert client.timestamp_sof == frame_id * 1000
        assert client.timestamp_eof == frame_id * 1000 + 500

        expected_y, expected_u, expected_v = _pattern(stream_id, frame_id)
        _sync_for_cpu_read(received.fd)
        try:
          data = received.data
          assert data.size == BUFFER_SIZE
          assert int(data[0]) == expected_y
          assert int(data[STRIDE * (HEIGHT - 1) + WIDTH - 1]) == expected_y
          assert int(data[UV_OFFSET]) == expected_u
          assert int(data[UV_OFFSET + 1]) == expected_v
          assert int(data[BUFFER_SIZE - 2]) == expected_u
          assert int(data[BUFFER_SIZE - 1]) == expected_v
          observed_y_values[stream_id].add(int(data[0]))
          client_mapping_addresses.append(int(data.__array_interface__["data"][0]))
        finally:
          _sync_for_cpu_read_end(received.fd)

        received_fds.append(received.fd)
        observed_indices.add(received.idx)
        data = None
        received = None
      assert observed_indices == set(range(BUFFERS_PER_STREAM))
      assert len(observed_y_values[stream_id]) == BUFFERS_PER_STREAM, "frame content did not change"

    # Each received fd is a duplicate of one of the publisher's open DMA-BUFs.
    publisher_fd_dir = Path(f"/proc/{process.pid}/fd")
    publisher_fds = [int(entry.name) for entry in publisher_fd_dir.iterdir() if entry.name.isdigit()]
    publisher_stats = {(os.stat(f"/proc/{process.pid}/fd/{fd}").st_dev,
                        os.stat(f"/proc/{process.pid}/fd/{fd}").st_ino) for fd in publisher_fds}
    for fd in received_fds:
      stat = os.fstat(fd)
      identity = (stat.st_dev, stat.st_ino)
      assert identity in publisher_stats, "received fd is not backed by a publisher-owned DMA-BUF"
      received_fd_identities.append(identity)
      assert fcntl.fcntl(fd, fcntl.F_GETFD) >= 0
    assert process.poll() is None, "publisher exited while client fds were still in use"

    # Closing clients must close the received fds and their imported mappings,
    # while the publisher still owns its original DMA-BUF fds/mappings.
    client = None
    clients.clear()
    gc.collect()
    assert all(not _fd_is_open(fd) for fd in received_fds)
    assert not any(_mapping_exists(address) for address in client_mapping_addresses)
    assert process.poll() is None, "publisher did not retain its buffers through client shutdown"
    publisher_fds_after_client_close = [int(entry.name) for entry in publisher_fd_dir.iterdir() if entry.name.isdigit()]
    publisher_stats_after_client_close = {
      (os.stat(f"/proc/{process.pid}/fd/{fd}").st_dev, os.stat(f"/proc/{process.pid}/fd/{fd}").st_ino)
      for fd in publisher_fds_after_client_close
    }
    assert all(identity in publisher_stats_after_client_close for identity in received_fd_identities), \
      "publisher released an external DMA-BUF before client shutdown"

    assert process.stdin is not None
    process.stdin.write("STOP\n")
    process.stdin.flush()
    _wait_matching_line(process, "CLEANUP_OK", 10.0)
    assert process.wait(timeout=10.0) == 0
    assert not socket_path.exists(), "publisher left its isolated VisionIPC socket behind"
  finally:
    client = None
    received = None
    data = None
    clients.clear()
    gc.collect()
    for fd in received_fds:
      if _fd_is_open(fd):
        os.close(fd)
    if process is not None and process.poll() is None:
      if process.stdin is not None:
        try:
          process.stdin.write("STOP\n")
          process.stdin.flush()
        except BrokenPipeError:
          pass
      try:
        process.wait(timeout=3.0)
      except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3.0)
    if process is not None and process.stdout is not None:
      remaining_output = process.stdout.read()
      if remaining_output:
        print(remaining_output, end="")
      process.stdout.close()
    if process is not None and process.stdin is not None and not process.stdin.closed:
      process.stdin.close()
    if owns_socket_path:
      socket_path.unlink(missing_ok=True)
    # This directory belongs only to the fresh test prefix.
    try:
      shutil.rmtree(msgq_dir, ignore_errors=True)
    finally:
      shutil.rmtree(tmp_path, ignore_errors=True)

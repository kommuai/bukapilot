#!/usr/bin/env python3
import os
import re
import select
import signal
import stat
import subprocess
import threading
import time

from cereal import messaging
from msgq.visionipc import VisionIpcServer, VisionStreamType
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


WIDTH = 1280
HEIGHT = 720
FRAME_BYTES = WIDTH * HEIGHT * 3 // 2
FPS = 20
FRAME_PERIOD_NS = 1_000_000_000 // FPS
READ_POLL_MS = 250
STARTUP_FRAME_TIMEOUT_S = 5
FRAME_STALL_TIMEOUT_S = 2
INITIAL_RETRY_S = 1
MAX_RETRY_S = 30
MPP_DECODER = os.path.join(BASEDIR, "build", "system", "sidecamerad", "sidecamerad_decoder")

DEVICE_RE = re.compile(r"/dev/video[0-9]+\Z")
BY_PATH_RE = re.compile(r"/dev/v4l/by-path/[A-Za-z0-9:_.+-]+-video-index0\Z")
AVAILABILITY_PARAM = "SideCameraAvailability"

CAMERAS = (
  ("frontLeftCameraState", "SIDECAM_FRONT_LEFT", VisionStreamType.VISION_STREAM_SIDE_FRONT_LEFT),
  ("frontRightCameraState", "SIDECAM_FRONT_RIGHT", VisionStreamType.VISION_STREAM_SIDE_FRONT_RIGHT),
  ("backLeftCameraState", "SIDECAM_BACK_LEFT", VisionStreamType.VISION_STREAM_SIDE_BACK_LEFT),
  ("backRightCameraState", "SIDECAM_BACK_RIGHT", VisionStreamType.VISION_STREAM_SIDE_BACK_RIGHT),
)
CAMERA_SERVICES = tuple(service for service, _, _ in CAMERAS)


def valid_device_path(device: str) -> bool:
  return bool(DEVICE_RE.fullmatch(device) or BY_PATH_RE.fullmatch(device))


def validate_device(device: str) -> None:
  if not valid_device_path(device):
    raise ValueError("device path must be /dev/videoN or a /dev/v4l/by-path/*-video-index0 path")

  # Resolve the stable physical-port symlink on every retry, but pass its
  # original name to the helper so a replug can bind to the current video node.
  resolved = os.path.realpath(device)
  if not DEVICE_RE.fullmatch(resolved):
    raise OSError(f"{device} does not resolve to a /dev/videoN image node")
  device_stat = os.stat(resolved)
  if not stat.S_ISCHR(device_stat.st_mode):
    raise OSError(f"{resolved} is not a character device")


class Availability:
  def __init__(self, params: Params):
    self.params = params
    self.lock = threading.Lock()
    self.state = {service: False for service in CAMERA_SERVICES}
    self._write()

  def _write(self) -> None:
    # Params.put serializes Python dictionaries as JSON; passing JSON text here
    # is a type error in the existing Params binding.
    self.params.put(AVAILABILITY_PARAM, dict(self.state))

  def set(self, service: str, available: bool) -> None:
    with self.lock:
      if self.state[service] != available:
        previous = self.state[service]
        self.state[service] = available
        try:
          self._write()
        except Exception:
          self.state[service] = previous
          raise

  def reset(self) -> None:
    with self.lock:
      if any(self.state.values()):
        previous = self.state.copy()
        for service in CAMERA_SERVICES:
          self.state[service] = False
        try:
          self._write()
        except Exception:
          self.state = previous
          raise


class CameraWorker(threading.Thread):
  def __init__(self, service: str, device: str, stream: VisionStreamType, vipc: VisionIpcServer,
               pm: messaging.PubMaster, stop_event: threading.Event, availability: Availability):
    super().__init__(name=f"sidecam-{service}", daemon=True)
    self.service = service
    self.device = device
    self.stream = stream
    self.vipc = vipc
    self.pm = pm
    self.stop_event = stop_event
    self.availability = availability
    self.available = False
    self.frame = bytearray(FRAME_BYTES)
    self.frame_view = memoryview(self.frame)
    self.frame_id = 0

  def _start_decoder(self) -> subprocess.Popen:
    validate_device(self.device)

    argv = [
      MPP_DECODER, self.device, str(WIDTH), str(HEIGHT), str(FPS),
    ]
    # stderr goes straight to the null device: native diagnostics cannot form
    # an unbounded pipe, and lifecycle messages stay bounded here.
    return subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, bufsize=0, close_fds=True)

  def _stop_decoder(self, proc: subprocess.Popen | None) -> None:
    if proc is None:
      return
    if proc.poll() is None:
      try:
        proc.terminate()
      except ProcessLookupError:
        pass
      try:
        proc.wait(timeout=1)
      except subprocess.TimeoutExpired:
        try:
          proc.kill()
        except ProcessLookupError:
          pass
        proc.wait()
    if proc.stdout is not None:
      try:
        proc.stdout.close()
      except OSError:
        pass

  def _publish(self, timestamp_eof: int) -> None:
    timestamp_sof = timestamp_eof - FRAME_PERIOD_NS
    frame_id = (self.frame_id + 1) & 0xFFFFFFFF
    self.frame_id = frame_id

    self.vipc.send(self.stream, self.frame_view, frame_id, timestamp_sof, timestamp_eof, True)

    msg = messaging.new_message(self.service, valid=True, logMonoTime=timestamp_eof)
    frame = getattr(msg, self.service)
    frame.frameId = frame_id
    frame.timestampSof = timestamp_sof
    frame.timestampEof = timestamp_eof
    self.pm.send(self.service, msg)

  def _read_frames(self, proc: subprocess.Popen) -> None:
    if proc.stdout is None:
      raise RuntimeError("MPP decoder stdout pipe was not created")

    fd = proc.stdout.fileno()
    poller = select.poll()
    poller.register(fd, select.POLLIN | select.POLLHUP | select.POLLERR)
    frame_deadline = time.monotonic() + STARTUP_FRAME_TIMEOUT_S

    while not self.stop_event.is_set():
      offset = 0
      while offset < FRAME_BYTES and not self.stop_event.is_set():
        ready = poller.poll(READ_POLL_MS)
        if not ready:
          if proc.poll() is not None:
            raise RuntimeError(f"MPP decoder exited with status {proc.returncode}")
          if time.monotonic() >= frame_deadline:
            raise RuntimeError("timed out waiting for a complete frame")
          continue

        try:
          count = os.readv(fd, [self.frame_view[offset:]])
        except InterruptedError:
          continue
        except BlockingIOError:
          continue
        if count == 0:
          raise RuntimeError(f"MPP decoder ended after {offset} of {FRAME_BYTES} frame bytes")
        offset += count

      if self.stop_event.is_set():
        return
      if offset != FRAME_BYTES:
        raise RuntimeError(f"short frame: {offset} of {FRAME_BYTES} bytes")
      self._publish(time.monotonic_ns())
      if not self.available:
        self.availability.set(self.service, True)
        self.available = True
      frame_deadline = time.monotonic() + FRAME_STALL_TIMEOUT_S

  def run(self) -> None:
    retry_s = INITIAL_RETRY_S
    while not self.stop_event.is_set():
      proc = None
      try:
        proc = self._start_decoder()
        cloudlog.info(f"sidecamerad {self.service}: started {self.device}")
        self._read_frames(proc)
        retry_s = INITIAL_RETRY_S
      except Exception as exc:
        if self.available:
          try:
            self.availability.set(self.service, False)
            self.available = False
          except Exception as status_exc:
            cloudlog.error(f"sidecamerad {self.service}: failed to clear availability ({status_exc})")
        if not self.stop_event.is_set():
          exit_code = proc.poll() if proc is not None else None
          cloudlog.warning(f"sidecamerad {self.service}: capture stopped ({exc}, exit={exit_code})")
      finally:
        self._stop_decoder(proc)

      if self.stop_event.is_set():
        if self.available:
          try:
            self.availability.set(self.service, False)
            self.available = False
          except Exception as status_exc:
            cloudlog.error(f"sidecamerad {self.service}: failed to clear availability at stop ({status_exc})")
        break
      cloudlog.info(f"sidecamerad {self.service}: retrying in {retry_s}s")
      if self.stop_event.wait(retry_s):
        break
      retry_s = min(MAX_RETRY_S, retry_s * 2)


def main() -> None:
  # Side camera IPC is host-local by contract. The generic messaging backend
  # switches to TCP when ZMQ is present, binding camera ports to all interfaces.
  os.environ.pop("ZMQ", None)

  stop_event = threading.Event()
  signal.signal(signal.SIGINT, lambda *_: stop_event.set())
  signal.signal(signal.SIGTERM, lambda *_: stop_event.set())

  availability = Availability(Params())
  configured = []
  for service, env_var, stream in CAMERAS:
    device = os.getenv(env_var)
    if not device:
      cloudlog.warning(f"sidecamerad {service}: no device configured ({env_var}); unavailable")
      continue
    if not valid_device_path(device):
      cloudlog.error(f"sidecamerad {service}: ignoring invalid {env_var} path")
      continue
    configured.append((service, device, stream))

  if not configured:
    cloudlog.warning("sidecamerad: no valid camera paths configured; waiting for shutdown")
    try:
      while not stop_event.wait(60):
        pass
    finally:
      availability.reset()
    return

  vipc = VisionIpcServer("sidecamerad")
  for _, _, stream in configured:
    vipc.create_buffers(stream, 3, WIDTH, HEIGHT)
  vipc.start_listener()

  pm = messaging.PubMaster([service for service, _, _ in configured])
  workers = [CameraWorker(service, device, stream, vipc, pm, stop_event, availability)
             for service, device, stream in configured]
  for worker in workers:
    worker.start()

  try:
    while not stop_event.wait(1):
      pass
  finally:
    stop_event.set()
    for worker in workers:
      worker.join()
    availability.reset()


if __name__ == "__main__":
  main()

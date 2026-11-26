import json
import os
import signal
import stat
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from cereal import messaging
from cereal.services import SERVICE_LIST
from msgq.visionipc import VisionStreamType
from openpilot.common.params import Params
from openpilot.system import sidecamerad


AVAILABILITY_DEFAULT = {
  "frontLeftCameraState": False,
  "frontRightCameraState": False,
  "backLeftCameraState": False,
  "backRightCameraState": False,
}


class FakeParams:
  def __init__(self, fail_next=False):
    self.writes = []
    self.fail_next = fail_next

  def put(self, key, value):
    if self.fail_next:
      self.fail_next = False
      raise OSError("injected Params write failure")
    self.writes.append((key, value.copy()))


class FakeCloudlog:
  def __init__(self):
    self.warnings = []
    self.errors = []
    self.infos = []

  def warning(self, message):
    self.warnings.append(message)

  def error(self, message):
    self.errors.append(message)

  def info(self, message):
    self.infos.append(message)


def test_params_default_and_json_dict_round_trip(tmp_path):
  params = Params(str(tmp_path / "params"))
  assert params.get_default_value(sidecamerad.AVAILABILITY_PARAM) == AVAILABILITY_DEFAULT

  params.put(sidecamerad.AVAILABILITY_PARAM, AVAILABILITY_DEFAULT)
  assert params.get(sidecamerad.AVAILABILITY_PARAM) == AVAILABILITY_DEFAULT


def test_manager_enables_sidecamerad_by_default_and_keeps_camerad(monkeypatch):
  code = """
import json
from openpilot.system.manager.process_config import managed_processes
side = managed_processes['sidecamerad']
camera = managed_processes['camerad']
print(json.dumps({'side_enabled': side.enabled,
                  'camera_enabled': camera.enabled,
                  'camera_condition': camera.should_run.__name__}))
"""

  def load(value):
    env = os.environ.copy()
    env.pop("SIDECAMERAD", None)
    if value is not None:
      env["SIDECAMERAD"] = value
    result = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True,
                            text=True, check=True, timeout=10)
    return json.loads(result.stdout)

  default = load(None)
  disabled = load("0")
  assert default["side_enabled"] is True
  assert disabled["side_enabled"] is False
  assert default["camera_enabled"] == disabled["camera_enabled"]
  assert default["camera_condition"] == disabled["camera_condition"] == "driverview"


def test_no_paths_acknowledges_each_feed_and_waits_for_signal(monkeypatch):
  for _, env_var, _ in sidecamerad.CAMERAS:
    monkeypatch.delenv(env_var, raising=False)

  params = FakeParams()
  cloudlog = FakeCloudlog()
  handlers = {}

  class FakeEvent:
    def __init__(self):
      self.waits = []
      self.is_set = False

    def wait(self, timeout):
      self.waits.append(timeout)
      if len(self.waits) == 2:
        handlers[signal.SIGTERM](signal.SIGTERM, None)
      return self.is_set

    def set(self):
      self.is_set = True

  event = FakeEvent()
  monkeypatch.setattr(sidecamerad.threading, "Event", lambda: event)
  monkeypatch.setattr(sidecamerad.signal, "signal", lambda sig, handler: handlers.__setitem__(sig, handler))
  monkeypatch.setattr(sidecamerad, "Params", lambda: params)
  monkeypatch.setattr(sidecamerad, "cloudlog", cloudlog)
  monkeypatch.setattr(sidecamerad, "VisionIpcServer", lambda *_: pytest.fail("no streams without paths"))
  monkeypatch.setattr(messaging, "PubMaster", lambda *_: pytest.fail("no publishers without paths"))

  sidecamerad.main()

  assert event.waits == [60, 60]
  assert signal.SIGINT in handlers and signal.SIGTERM in handlers
  assert len(params.writes) == 1
  assert params.writes[0] == (sidecamerad.AVAILABILITY_PARAM, AVAILABILITY_DEFAULT)
  for service, env_var, _ in sidecamerad.CAMERAS:
    assert any(service in message and env_var in message for message in cloudlog.warnings)


def test_main_removes_inherited_zmq_before_ipc_and_pubmaster(monkeypatch):
  monkeypatch.setenv("ZMQ", "1")
  for _, env_var, _ in sidecamerad.CAMERAS:
    monkeypatch.delenv(env_var, raising=False)
  monkeypatch.setenv("SIDECAM_FRONT_LEFT", "/dev/video60")

  constructor_env = []
  handlers = {}

  class FakeEvent:
    def __init__(self):
      self.stopped = False

    def wait(self, _timeout):
      if not self.stopped:
        handlers[signal.SIGTERM](signal.SIGTERM, None)
      return self.stopped

    def set(self):
      self.stopped = True

  event = FakeEvent()

  class IsolatedParams(FakeParams):
    def __init__(self):
      constructor_env.append(("Params", "ZMQ" in os.environ))
      super().__init__()

  class FakeVisionIpcServer:
    def __init__(self, name):
      constructor_env.append(("VisionIpcServer", "ZMQ" in os.environ))
      assert name == "sidecamerad"

    def create_buffers(self, *_args):
      pass

    def start_listener(self):
      pass

  class FakePubMaster:
    def __init__(self, services):
      constructor_env.append(("PubMaster", "ZMQ" in os.environ))
      assert services == ["frontLeftCameraState"]

  workers = []

  class FakeWorker:
    def __init__(self, service, device, *_args):
      self.service = service
      self.device = device
      self.started = False
      self.joined = False
      workers.append(self)

    def start(self):
      self.started = True

    def join(self):
      self.joined = True

  monkeypatch.setattr(sidecamerad, "Params", IsolatedParams)
  monkeypatch.setattr(sidecamerad, "VisionIpcServer", FakeVisionIpcServer)
  monkeypatch.setattr(messaging, "PubMaster", FakePubMaster)
  monkeypatch.setattr(sidecamerad, "CameraWorker", FakeWorker)
  monkeypatch.setattr(sidecamerad.threading, "Event", lambda: event)
  monkeypatch.setattr(
    sidecamerad.signal, "signal", lambda sig, handler: handlers.__setitem__(sig, handler)
  )

  sidecamerad.main()

  assert constructor_env == [
    ("Params", False),
    ("VisionIpcServer", False),
    ("PubMaster", False),
  ]
  assert "ZMQ" not in os.environ
  assert len(workers) == 1
  assert workers[0].service == "frontLeftCameraState"
  assert workers[0].device == "/dev/video60"
  assert workers[0].started and workers[0].joined
  assert event.stopped


def test_explicit_device_paths_resolve_only_to_video_character_nodes(monkeypatch):
  monkeypatch.setattr(sidecamerad.os.path, "realpath", lambda path: "/dev/video60")
  monkeypatch.setattr(sidecamerad.os, "stat", lambda _path: SimpleNamespace(st_mode=stat.S_IFCHR | 0o600))

  sidecamerad.validate_device("/dev/video60")
  sidecamerad.validate_device("/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0")
  assert not sidecamerad.valid_device_path("/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index1")
  with pytest.raises(ValueError):
    sidecamerad.validate_device("/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index1")

  monkeypatch.setattr(sidecamerad.os.path, "realpath", lambda _path: "/etc/passwd")
  with pytest.raises(OSError, match="does not resolve"):
    sidecamerad.validate_device("/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0")


def test_worker_starts_native_mpp_capture_helper(monkeypatch):
  calls = {}
  monkeypatch.setattr(sidecamerad, "validate_device", lambda device: calls.setdefault("device", device))
  monkeypatch.setattr(sidecamerad.subprocess, "Popen", lambda argv, **kwargs: calls.update(argv=argv, kwargs=kwargs) or object())

  worker = sidecamerad.CameraWorker(
    "frontLeftCameraState", "/dev/video60", sidecamerad.CAMERAS[0][2],
    object(), object(), object(), object(),
  )
  process = worker._start_decoder()

  assert process is not None
  assert calls["device"] == "/dev/video60"
  assert calls["argv"] == [sidecamerad.MPP_DECODER, "/dev/video60", "1280", "720", "20"]
  assert calls["kwargs"] == {
    "stdin": sidecamerad.subprocess.DEVNULL,
    "stdout": sidecamerad.subprocess.PIPE,
    "stderr": sidecamerad.subprocess.DEVNULL,
    "bufsize": 0,
    "close_fds": True,
  }


def test_side_camera_service_stream_and_schema_mapping_is_stable():
  mapping = (
    ("frontLeftCameraState", VisionStreamType.VISION_STREAM_SIDE_FRONT_LEFT, 150),
    ("frontRightCameraState", VisionStreamType.VISION_STREAM_SIDE_FRONT_RIGHT, 151),
    ("backLeftCameraState", VisionStreamType.VISION_STREAM_SIDE_BACK_LEFT, 152),
    ("backRightCameraState", VisionStreamType.VISION_STREAM_SIDE_BACK_RIGHT, 153),
  )
  schema = (Path(__file__).resolve().parents[3] / "cereal/log.capnp").read_text()
  for service, stream, ordinal in mapping:
    assert SERVICE_LIST[service].frequency == 20.0
    assert SERVICE_LIST[service].should_log is False
    assert int(stream) == 7 + ordinal - 150
    assert f"{service} @{ordinal} :FrameData;" in schema


def test_availability_writes_only_state_transitions_and_recovers_write_failure():
  params = FakeParams()
  availability = sidecamerad.Availability(params)
  assert params.writes == [(sidecamerad.AVAILABILITY_PARAM, AVAILABILITY_DEFAULT)]

  service = "frontLeftCameraState"
  availability.set(service, False)
  availability.set(service, True)
  availability.set(service, True)
  availability.set(service, False)
  availability.reset()
  assert [value[service] for _, value in params.writes] == [False, True, False]

  failing_params = FakeParams()
  failing = sidecamerad.Availability(failing_params)
  failing_params.fail_next = True
  with pytest.raises(OSError, match="injected"):
    failing.set(service, True)
  assert failing.state[service] is False
  failing.set(service, True)
  assert failing.state[service] is True
  assert failing_params.writes[-1][1][service] is True


def test_worker_clears_availability_after_capture_failure(monkeypatch):
  params = FakeParams()
  availability = sidecamerad.Availability(params)
  availability.set("frontLeftCameraState", True)
  cloudlog = FakeCloudlog()

  class FakeEvent:
    def is_set(self):
      return False

    def wait(self, _timeout):
      return True

  class FakeDecoder:
    returncode = 1

    def poll(self):
      return self.returncode

  worker = sidecamerad.CameraWorker(
    "frontLeftCameraState", "/dev/video60", sidecamerad.CAMERAS[0][2],
    object(), object(), FakeEvent(), availability,
  )
  worker.available = True

  def fail_read(_proc):
    raise RuntimeError("injected capture loss")

  monkeypatch.setattr(worker, "_start_decoder", lambda: FakeDecoder())
  monkeypatch.setattr(worker, "_read_frames", fail_read)
  monkeypatch.setattr(worker, "_stop_decoder", lambda _proc: None)
  monkeypatch.setattr(sidecamerad, "cloudlog", cloudlog)

  worker.run()

  assert availability.state["frontLeftCameraState"] is False
  assert [value["frontLeftCameraState"] for _, value in params.writes] == [False, True, False]
  assert any("injected capture loss" in message for message in cloudlog.warnings)

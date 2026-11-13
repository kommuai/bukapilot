import os
import random
import tempfile
from pathlib import Path


import openpilot.system.loggerd.deleter as deleter
import openpilot.system.loggerd.uploader as uploader
from openpilot.common.params import Params
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.xattr_cache import setxattr


def create_random_file(file_path: Path, size_mb: float, lock: bool = False, upload_xattr: bytes | None = None) -> None:
  file_path.parent.mkdir(parents=True, exist_ok=True)

  if lock:
    lock_path = str(file_path) + ".lock"
    os.close(os.open(lock_path, os.O_CREAT | os.O_EXCL))

  chunks = 128
  chunk_bytes = int(size_mb * 1024 * 1024 / chunks)
  data = os.urandom(chunk_bytes)

  with open(file_path, "wb") as f:
    for _ in range(chunks):
      f.write(data)

  if upload_xattr is not None:
    setxattr(str(file_path), uploader.UPLOAD_ATTR_NAME, upload_xattr)

class MockResponse:
  def __init__(self, text, status_code):
    self.text = text
    self.status_code = status_code

  def json(self):
    return {"segments": []}

class UploaderTestCase:
  f_type = "UNKNOWN"

  root: Path
  seg_num: int
  seg_format: str
  seg_format2: str
  seg_dir: str

  def set_ignore(self):
    uploader.fake_upload = False
    uploader.fia_upload = lambda *args, **kwargs: MockResponse('', 412)

  def setup_method(self):
    self._temp_dir = tempfile.TemporaryDirectory(prefix="loggerd-tests-", dir=Path(__file__).resolve().parents[3])
    self.root = Path(self._temp_dir.name)
    self._original_env = {key: os.environ.get(key) for key in ("LOG_ROOT", "PARAMS_ROOT", "OPENPILOT_PREFIX")}
    os.environ.update({
      "LOG_ROOT": str(self.root / "logs"),
      "PARAMS_ROOT": str(self.root / "params"),
      "OPENPILOT_PREFIX": f"_loggerd_test_{self.root.name}",
    })
    Path(os.environ["LOG_ROOT"]).mkdir()
    Path(os.environ["PARAMS_ROOT"]).mkdir()

    self._original_uploader = {
      "kapi": uploader.kapi,
      "fia_upload": uploader.fia_upload,
      "fake_upload": uploader.fake_upload,
      "force_wifi": uploader.force_wifi,
      "allow_sleep": uploader.allow_sleep,
    }
    uploader.kapi = lambda *args, **kwargs: MockResponse('', 200)
    uploader.fake_upload = True
    uploader.force_wifi = True
    uploader.allow_sleep = False
    from openpilot.system.loggerd import xattr_cache
    xattr_cache._cached_attributes.clear()

    self.seg_num = random.randint(1, 300)
    self.seg_format = "00000004--0ac3964c96--{}"
    self.seg_format2 = "00000005--4c4e99b08b--{}"
    self.seg_dir = self.seg_format.format(self.seg_num)

    self.params = Params()
    self.params.put("IsOffroad", True)
    self.params.put("DongleId", "0000000000000000")

  def teardown_method(self):
    from openpilot.system.loggerd import xattr_cache
    xattr_cache._cached_attributes.clear()
    for key, value in self._original_uploader.items():
      setattr(uploader, key, value)
    for key, value in self._original_env.items():
      if value is None:
        os.environ.pop(key, None)
      else:
        os.environ[key] = value
    self._temp_dir.cleanup()

  def make_file_with_data(self, f_dir: str, fn: str, size_mb: float = .1, lock: bool = False,
                          upload_xattr: bytes | None = None, preserve_xattr: bytes | None = None) -> Path:
    file_path = Path(Paths.log_root()) / f_dir / fn
    create_random_file(file_path, size_mb, lock, upload_xattr)

    if preserve_xattr is not None:
      setxattr(str(file_path.parent), deleter.PRESERVE_ATTR_NAME, preserve_xattr)

    return file_path

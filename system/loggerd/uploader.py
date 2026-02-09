#!/usr/bin/env python3
import bz2
import io
import os
import random
import requests
import threading
import time
import traceback
import datetime
from typing import BinaryIO
from collections.abc import Iterator

from cereal import log
import cereal.messaging as messaging
from openpilot.common.api import Api
from openpilot.common.params import Params
from openpilot.common.kommu import *
from openpilot.common.realtime import set_core_affinity
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr
from openpilot.system.loggerd.kommu import fia_upload, fia_upload_bytes, kapi, WEB_BASE
from openpilot.system.loggerd.memory_pressure import (
  is_memory_pressure_critical,
  should_skip_filesystem_operation,
  handle_memory_pressure,
)
from openpilot.common.swaglog import cloudlog

NetworkType = log.DeviceState.NetworkType
UPLOAD_ATTR_NAME = 'user.upload'
UPLOAD_ATTR_VALUE = b'1'

UPLOAD_QLOG_QCAM_MAX_SIZE = 100 * 1e6  # MB
UPLOAD_FULL_LOG_MAX_SIZE = 300 * 1e6  # MB (on-demand full-segment uploads)

FULL_SEGMENT_FILES = ("fcamera.hevc", "ecamera.hevc", "dcamera.hevc", "rlog", "qlog", "qcamera.ts")

allow_sleep = bool(os.getenv("UPLOADER_SLEEP", "1"))
force_wifi = os.getenv("FORCEWIFI") is not None
fake_upload = os.getenv("FAKEUPLOAD") is not None


class FakeRequest:
  def __init__(self):
    self.headers = {"Content-Length": "0"}


class FakeResponse:
  def __init__(self):
    self.status_code = 200
    self.request = FakeRequest()


def get_directory_sort(d: str) -> list[str]:
  return [s.rjust(10, '0') for s in d.rsplit('--', 1)]

def listdir_by_creation(d: str) -> list[str]:
  if not os.path.isdir(d):
    return []

  try:
    paths = [f for f in os.listdir(d) if os.path.isdir(os.path.join(d, f))]
    paths = sorted(paths, key=get_directory_sort)
    return paths
  except OSError:
    cloudlog.exception("listdir_by_creation failed")
    return []

def clear_locks(root: str) -> None:
  for logdir in os.listdir(root):
    path = os.path.join(root, logdir)
    try:
      for fname in os.listdir(path):
        if fname.endswith(".lock"):
          os.unlink(os.path.join(path, fname))
    except OSError:
      cloudlog.exception("clear_locks failed")


def get_pending_full_upload_segments() -> list[str]:
  """Return list of logdirs for which the server requested a full upload."""
  if should_skip_filesystem_operation():
    return []
  dongle_id = Params().get("DongleId", encoding="utf8")
  if not dongle_id:
    return []
  try:
    resp = kapi(
      requests.get,
      WEB_BASE + "/fia/pending_full_uploads",
      headers={"X-Kaac-Id": dongle_id},
    )
    if resp.status_code != 200:
      return []
    data = resp.json()
    return data.get("segments") or []
  except Exception:
    cloudlog.exception("get_pending_full_upload_segments failed")
    return []


def post_full_upload_done(logdir: str) -> None:
  """Notify server that full upload for this segment is done."""
  dongle_id = Params().get("DongleId", encoding="utf8")
  headers = {"X-Kaac-Id": dongle_id} if dongle_id else {}
  try:
    kapi(
      requests.post,
      WEB_BASE + "/fia/full_upload_done",
      json={"logdir": logdir},
      headers=headers,
    )
  except Exception:
    cloudlog.exception("post_full_upload_done failed", logdir=logdir)


def resolve_logdir_to_segment_paths(root: str, logdir: str) -> list[str]:
  """
  Resolve server logdir (e.g. '2026-02-07--14-20-43') to all matching segment directories on disk.
  On device, segment paths are route_name + '--' + part, e.g. '2026-02-07--14-20-43--0', '--1', ...
  Returns a sorted list of segment dir names (e.g. ['2026-02-07--14-20-43--0', '...--1', ...]), or [] if none.
  """
  path = os.path.join(root, logdir)
  if os.path.isdir(path):
    return [logdir]
  try:
    candidates = [
      name for name in os.listdir(root)
      if name.startswith(logdir + "--") and os.path.isdir(os.path.join(root, name))
    ]
    if candidates:
      return sorted(candidates)  # --0, --1, --2, ...
  except OSError:
    pass
  return []


class Uploader:
  def __init__(self, dongle_id: str, root: str):
    self.dongle_id = dongle_id
    self.root = root

    self.params = Params()

    # stats for last successfully uploaded file
    self.last_filename = ""

    self.immediate_folders = ["crash/", "boot/"]
    self.immediate_priority = {"qlog": 0, "qlog.bz2": 0, "qcamera.ts": 1}

  def list_upload_files(self, metered: bool) -> Iterator[tuple[str, str, str]]:
    # Skip directory scanning if memory is critical
    if should_skip_filesystem_operation():
      return

    r = self.params.get("AthenadRecentlyViewedRoutes", encoding="utf8")
    requested_routes = [] if r is None else r.split(",")

    for logdir in listdir_by_creation(self.root):
      # Check memory again before processing each directory
      if should_skip_filesystem_operation():
        break

      path = os.path.join(self.root, logdir)
      try:
        names = os.listdir(path)
      except OSError:
        continue

      if any(name.endswith(".lock") for name in names):
        continue

      for name in sorted(names, key=lambda n: self.immediate_priority.get(n, 1000)):
        key = os.path.join(logdir, name)
        fn = os.path.join(path, name)
        # skip files already uploaded
        try:
          ctime = os.path.getctime(fn)
          is_uploaded = getxattr(fn, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE
        except OSError:
          cloudlog.event("uploader_getxattr_failed", key=key, fn=fn)
          # deleter could have deleted, so skip
          continue
        if is_uploaded:
          continue

        yield name, key, fn

  def next_file_to_upload(self, metered: bool) -> tuple[str, str, str] | None:
    upload_files = list(self.list_upload_files(metered))

    for name, key, fn in upload_files:
      if any(f in fn for f in self.immediate_folders):
        return name, key, fn

    for name, key, fn in upload_files:
      if name in self.immediate_priority:
        return name, key, fn

    return None

  def do_upload(self, key: str, fn: str):
    key, ext = os.path.splitext(key.replace("/", "---"))

    if "boot" in key or "crash" in key:
      key = "---".join([self.dongle_id] + list(reversed(key.split("---")))) + ext
    else:
      key = self.dongle_id + "---" + key + ext

    cloudlog.info("upload_kommu s4-v1 %s, %s", key, fn)

    if fake_upload:
      return FakeResponse()
    else:
      return fia_upload(key, fn)

  def upload(self, name: str, key: str, fn: str, network_type: int, metered: bool, max_size: int | None = None) -> bool:
    if max_size is None:
      max_size = UPLOAD_QLOG_QCAM_MAX_SIZE
    try:
      sz = os.path.getsize(fn)
    except OSError:
      cloudlog.exception("upload: getsize failed")
      return False

    cloudlog.event("upload_start", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)

    if sz == 0:
      # tag files of 0 size as uploaded
      success = True
    elif name in self.immediate_priority and sz > max_size:
      cloudlog.event("uploader_too_large", key=key, fn=fn, sz=sz)
      success = True
    else:
      start_time = time.monotonic()

      stat = None
      last_exc = None
      try:
        stat = self.do_upload(key, fn)
      except Exception as e:
        last_exc = (e, traceback.format_exc())

      if stat is not None and stat.status_code in (200, 201, 401, 403, 412):
        self.last_filename = fn
        dt = time.monotonic() - start_time
        if stat.status_code == 412:
          cloudlog.event("upload_ignored", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
        else:
          content_length = int(stat.request.headers.get("Content-Length", 0))
          speed = (content_length / 1e6) / dt
          cloudlog.event("upload_success", key=key, fn=fn, sz=sz, content_length=content_length,
                         network_type=network_type, metered=metered, speed=speed)
        success = True
      else:
        success = False
        cloudlog.event("upload_failed", stat=stat, exc=last_exc, key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)

    if success:
      # tag file as uploaded
      try:
        setxattr(fn, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)
      except OSError:
        cloudlog.event("uploader_setxattr_failed", exc=last_exc, key=key, fn=fn, sz=sz)

    return success

  def upload_full_segment_file(
    self, logdir: str, name: str, fn: str, network_type: int, metered: bool
  ) -> bool:
    """Upload one file for an on-demand full segment. Uses UPLOAD_FULL_LOG_MAX_SIZE; compresses rlog/qlog."""
    try:
      sz = os.path.getsize(fn)
    except OSError:
      return False
    if sz > UPLOAD_FULL_LOG_MAX_SIZE:
      cloudlog.event("uploader_full_too_large", key=logdir, name=name, sz=sz)
      return True  # mark as done so we don't retry
    # Key: dongle_id---logdir---filename (rlog/qlog use .bz2 suffix)
    if name in ("rlog", "qlog"):
      key_suffix = name + ".bz2"
      try:
        with open(fn, "rb") as f:
          content = f.read()
        data = bz2.compress(content)
      except OSError:
        cloudlog.exception("upload_full_segment_file compress failed", fn=fn)
        return False
      key = self.dongle_id + "---" + logdir + "---" + key_suffix
      cloudlog.event("upload_start", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
      if fake_upload:
        stat = FakeResponse()
      else:
        try:
          stat = fia_upload_bytes(key, data)
        except Exception as e:
          cloudlog.event("upload_failed", exc=(e, traceback.format_exc()), key=key, fn=fn, sz=sz)
          return False
      if stat.status_code in (200, 201, 401, 403, 412):
        try:
          setxattr(fn, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)
        except OSError:
          pass
        cloudlog.event("upload_success", key=key, fn=fn, sz=sz)
        return True
      cloudlog.event("upload_failed", stat=stat, key=key, fn=fn, sz=sz)
      return False
    # Non-compressed file (do_upload prepends dongle_id, so pass segment_dir---name only)
    key = logdir + "---" + name
    return self.upload(name, key, fn, network_type, metered, max_size=UPLOAD_FULL_LOG_MAX_SIZE)

  def step(self, network_type: int, metered: bool) -> bool | None:
    # On-demand full upload when we have pending segments (Wi‑Fi, not metered)
    if not metered:
      segments = get_pending_full_upload_segments()
      if segments:
        logdir = segments[0]
        segment_dirs = resolve_logdir_to_segment_paths(self.root, logdir)
        cloudlog.event("upload_full_resolve", logdir=logdir, root=self.root, segment_dirs=segment_dirs)
        if not segment_dirs:
          cloudlog.event("upload_full_skip_no_segment", logdir=logdir, root=self.root)
          post_full_upload_done(logdir)
          return True
        if len(segment_dirs) > 1 or segment_dirs[0] != logdir:
          cloudlog.event("upload_full_resolved_logdir", logdir=logdir, segment_dirs=segment_dirs)
        any_success = False
        for segment_dir in segment_dirs:
          path = os.path.join(self.root, segment_dir)
          try:
            names = os.listdir(path)
          except OSError:
            cloudlog.event("upload_full_listdir_failed", segment_dir=segment_dir)
            continue
          if any(n.endswith(".lock") for n in names):
            cloudlog.event("upload_full_skip_locked", segment_dir=segment_dir)
            continue
          cloudlog.event("upload_full_start", segment_dir=segment_dir, logdir=logdir)
          for name in FULL_SEGMENT_FILES:
            fn = os.path.join(path, name)
            if not os.path.isfile(fn):
              continue
            try:
              if getxattr(fn, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE:
                continue
            except OSError:
              continue
            if self.upload_full_segment_file(segment_dir, name, fn, network_type, metered):
              any_success = True
            else:
              return False
          cloudlog.event("upload_full_segment_done", segment_dir=segment_dir)
        post_full_upload_done(logdir)
        cloudlog.event("upload_full_done", logdir=logdir, segment_count=len(segment_dirs))
        return True

    d = self.next_file_to_upload(metered)
    if d is not None:
      name, key, fn = d
      # qlogs and bootlogs need to be compressed before uploading
      if key.endswith(('qlog', 'rlog')) or (key.startswith('boot/') and not key.endswith('.bz2')):
        key += ".bz2"
      return self.upload(name, key, fn, network_type, metered)

    return None


def main(exit_event: threading.Event = None) -> None:
  if exit_event is None:
    exit_event = threading.Event()

  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("failed to set core affinity")

  clear_locks(Paths.log_root())

  params = Params()
  dongle_id = params.get("DongleId", encoding='utf8')

  if dongle_id is None:
    cloudlog.info("uploader missing dongle_id")
    raise Exception("uploader can't start without dongle id")

  sm = messaging.SubMaster(['deviceState'])
  uploader = Uploader(dongle_id, Paths.log_root())

  backoff = 0.1
  while not exit_event.is_set():
    sm.update(0)
    
    # Check memory pressure - skip upload operations if critical
    if is_memory_pressure_critical():
      handle_memory_pressure(clear_caches=True, clear_fs_cache=False)
      cloudlog.warning("Skipping uploader operations due to critical memory pressure")
      if allow_sleep:
        time.sleep(30)  # Wait longer when memory is critical
      continue

    offroad = params.get_bool("IsOffroad")
    network_type = sm['deviceState'].networkType if not force_wifi else NetworkType.wifi
    if network_type == NetworkType.none:
      if allow_sleep:
        time.sleep(60 if offroad else 5)
      continue

    success = uploader.step(sm['deviceState'].networkType.raw, sm['deviceState'].networkMetered)
    if success is None:
      backoff = 60 if offroad else 5
    elif success:
      backoff = 0.1
    else:
      cloudlog.info("upload backoff %r", backoff)
      backoff = min(backoff*2, 120)
    if allow_sleep:
      time.sleep(backoff + random.uniform(0, backoff))


if __name__ == "__main__":
  main()

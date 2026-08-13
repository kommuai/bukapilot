import os
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.appbridged.video_constants import (
  CACHE_ROOT,
  FFMPEG_HEVC_INPUT_ARGS,
  FFMPEG_H264_ENCODE_ARGS,
  FFMPEG_H264_HW_ENCODE_ARGS,
  FFMPEG_NO_SUBSTREAMS,
  MP4_CACHE_DIR,
  MP4_CONVERT_TIMEOUT_SEC,
)

# RK3588: stage remux + encode on tmpfs (~660 MiB free) to avoid SD I/O during convert.
TMP_CONVERT_DIR = Path("/tmp")


def mp4_cache_path(drive_id: str, segment: int, camera: str) -> Path:
  return Path(MP4_CACHE_DIR) / f"{drive_id}--{segment}--{camera}.mp4"


def _staging_paths(mp4_path: Path) -> tuple[Path, Path]:
  tag = mp4_path.stem
  return (
    TMP_CONVERT_DIR / f"kommu_{tag}.hevcwrap.mp4",
    TMP_CONVERT_DIR / f"kommu_{tag}.tmp.mp4",
  )


def _hevc_wrap_path(mp4_path: Path) -> Path:
  return _staging_paths(mp4_path)[0]


def _staging_tmp_path(mp4_path: Path) -> Path:
  return _staging_paths(mp4_path)[1]


def _finalize_tmp_mp4(tmp: Path, mp4_path: Path) -> None:
  mp4_path.unlink(missing_ok=True)
  try:
    os.replace(tmp, mp4_path)
  except OSError:
    shutil.move(str(tmp), str(mp4_path))


def _is_mp4_valid(mp4_path: Path, hevc_path: str) -> bool:
  if not mp4_path.is_file() or mp4_path.stat().st_size <= 0:
    return False
  if not os.path.isfile(hevc_path):
    return False
  return mp4_path.stat().st_mtime >= os.path.getmtime(hevc_path)


def delete_mp4_cache(mp4_path: Path | None) -> None:
  if not mp4_path:
    return
  try:
    mp4_path.unlink(missing_ok=True)
    mp4_path.with_suffix(".tmp.mp4").unlink(missing_ok=True)
    mp4_path.with_suffix(".hevcwrap.mp4").unlink(missing_ok=True)
    wrap, tmp = _staging_paths(mp4_path)
    wrap.unlink(missing_ok=True)
    tmp.unlink(missing_ok=True)
    Path(MP4_CACHE_DIR).rmdir()
    Path(CACHE_ROOT).rmdir()
  except OSError:
    pass


def clear_mp4_cache() -> None:
  shutil.rmtree(MP4_CACHE_DIR, ignore_errors=True)
  try:
    Path(CACHE_ROOT).rmdir()
  except OSError:
    pass


class Mp4ConvertJob:
  def __init__(self, hevc_path: str, mp4_path: Path):
    self.hevc_path = hevc_path
    self.mp4_path = mp4_path
    self._lock = threading.Lock()
    self._done = False
    self._ok = False
    self._error: str | None = None
    self._cancelled = False
    self._proc: subprocess.Popen | None = None
    threading.Thread(target=self._run, daemon=True).start()

  def _spawn_ffmpeg(self, cmd: list[str]) -> bool:
    proc = None
    try:
      proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
      with self._lock:
        if self._cancelled:
          try:
            os.killpg(proc.pid, signal.SIGTERM)
          except OSError:
            pass
          return False
        self._proc = proc
      _, stderr = proc.communicate(timeout=MP4_CONVERT_TIMEOUT_SEC)
      if proc.returncode != 0 and stderr:
        err = stderr.decode("utf-8", errors="replace").strip()
        if err:
          cloudlog.warning(f"ffmpeg failed rc={proc.returncode}: {err[:500]}")
      return proc.returncode == 0
    except subprocess.TimeoutExpired:
      if proc is not None:
        try:
          os.killpg(proc.pid, signal.SIGTERM)
        except OSError:
          pass
        try:
          proc.wait(timeout=5)
        except subprocess.SubprocessError:
          pass
      return False
    finally:
      with self._lock:
        self._proc = None

  def _valid_output(self, path: Path) -> bool:
    try:
      return path.is_file() and path.stat().st_size > 0
    except OSError:
      return False

  def _h264_mp4_cmd(self, input_path: Path, output_path: Path, *, hw_decode: bool) -> list[str]:
    cmd = ["ffmpeg", "-y", "-nostdin", "-loglevel", "error"]
    if hw_decode:
      cmd.extend(["-probesize", "32768", "-analyzeduration", "0", "-c:v", "hevc_rkmpp", "-i", str(input_path)])
    else:
      cmd.extend([*FFMPEG_HEVC_INPUT_ARGS, "-i", str(input_path)])
    cmd.extend([*FFMPEG_NO_SUBSTREAMS, *FFMPEG_H264_HW_ENCODE_ARGS, "-movflags", "+faststart", str(output_path)])
    return cmd

  def _remux_hevc_wrap(self, wrap: Path) -> bool:
    wrap.unlink(missing_ok=True)
    cmd = [
      "ffmpeg", "-y", "-nostdin", "-loglevel", "error",
      "-probesize", "32768", "-analyzeduration", "0",
      *FFMPEG_HEVC_INPUT_ARGS, "-i", self.hevc_path,
      "-c", "copy", str(wrap),
    ]
    if not self._spawn_ffmpeg(cmd) or not self._valid_output(wrap):
      wrap.unlink(missing_ok=True)
      return False
    return True

  def _encode_mp4(self, tmp: Path) -> bool:
    wrap = _hevc_wrap_path(self.mp4_path)
    t0 = time.monotonic()
    try:
      if self._remux_hevc_wrap(wrap):
        tmp.unlink(missing_ok=True)
        if self._spawn_ffmpeg(self._h264_mp4_cmd(wrap, tmp, hw_decode=True)) and self._valid_output(tmp):
          cloudlog.info(
            f"mp4 convert ok path=rkmpp_hw hevc={self.hevc_path} "
            f"elapsed={time.monotonic() - t0:.2f}s"
          )
          return True
        cloudlog.warning("mp4 h264 encode failed (rkmpp_hw), trying fallback")
      else:
        cloudlog.warning("mp4 remux failed, trying sw_decode path")

      if self._cancelled:
        return False
      tmp.unlink(missing_ok=True)
      if self._spawn_ffmpeg(self._h264_mp4_cmd(Path(self.hevc_path), tmp, hw_decode=False)) and self._valid_output(tmp):
        cloudlog.info(
          f"mp4 convert ok path=rkmpp_sw_decode hevc={self.hevc_path} "
          f"elapsed={time.monotonic() - t0:.2f}s"
        )
        return True
      cloudlog.warning("mp4 h264 encode failed (rkmpp_sw_decode), trying libx264")

      if self._cancelled:
        return False
      tmp.unlink(missing_ok=True)
      cmd = [
        "ffmpeg", "-y", "-nostdin", "-loglevel", "error",
        *FFMPEG_HEVC_INPUT_ARGS, "-i", self.hevc_path,
        *FFMPEG_NO_SUBSTREAMS, *FFMPEG_H264_ENCODE_ARGS, "-movflags", "+faststart",
        str(tmp),
      ]
      if self._spawn_ffmpeg(cmd) and self._valid_output(tmp):
        cloudlog.info(
          f"mp4 convert ok path=libx264 hevc={self.hevc_path} "
          f"elapsed={time.monotonic() - t0:.2f}s"
        )
        return True
      cloudlog.warning("mp4 h264 encode failed (libx264)")
      return False
    finally:
      wrap.unlink(missing_ok=True)

  def _run(self):
    ok = False
    err = None
    tmp = _staging_tmp_path(self.mp4_path)
    try:
      if _is_mp4_valid(self.mp4_path, self.hevc_path):
        ok = True
      elif self._cancelled:
        err = "conversion_cancelled"
      else:
        cloudlog.info(f"mp4 convert start hevc={self.hevc_path}")
        tmp.unlink(missing_ok=True)
        os.makedirs(self.mp4_path.parent, exist_ok=True)
        ok = self._encode_mp4(tmp)
        if self._cancelled:
          ok = False
          err = "conversion_cancelled"
        elif ok:
          if not self._valid_output(tmp):
            ok = False
            err = "conversion_failed"
          else:
            _finalize_tmp_mp4(tmp, self.mp4_path)
        else:
          err = "conversion_failed"
        if not ok:
          tmp.unlink(missing_ok=True)
          self.mp4_path.unlink(missing_ok=True)
    except Exception as e:
      cloudlog.error(f"mp4 convert job error: {e}")
      err = "conversion_cancelled" if self._cancelled else "conversion_failed"
      delete_mp4_cache(self.mp4_path)
    with self._lock:
      self._ok = ok
      self._error = err
      self._done = True

  def cancel(self) -> None:
    with self._lock:
      self._cancelled = True
      proc = self._proc
    if proc is not None:
      try:
        os.killpg(proc.pid, signal.SIGTERM)
      except OSError:
        pass
    delete_mp4_cache(self.mp4_path)

  def poll(self) -> tuple[bool, bool, str | None]:
    with self._lock:
      return self._done, self._ok, self._error

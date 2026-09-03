import os
import shutil
import subprocess
from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.appbridged.video_constants import (
  CACHE_ROOT,
  FFMPEG_HEVC_INPUT_ARGS,
  FFMPEG_NO_SUBSTREAMS,
  THUMB_CACHE_DIR,
  THUMB_FFMPEG_THREADS,
  THUMB_FFMPEG_TIMEOUT_SEC,
)


def thumb_cache_path(drive_id: str, segment: int, camera: str) -> Path:
  return Path(THUMB_CACHE_DIR) / f"{drive_id}--{segment}--{camera}.jpg"


def delete_thumb_cache(cache_path: Path | None) -> None:
  if not cache_path:
    return
  try:
    cache_path.unlink(missing_ok=True)
    cache_path.with_suffix(".tmp.jpg").unlink(missing_ok=True)
  except OSError:
    pass


def _parse_thumb_cache_name(name: str) -> tuple[str, int, str] | None:
  if not name.endswith(".jpg") or name.endswith(".tmp.jpg"):
    return None
  base = name[:-4]
  parts = base.rsplit("--", 2)
  if len(parts) != 3:
    return None
  drive_id, seg_str, camera = parts
  if camera not in ("road", "wide"):
    return None
  try:
    segment = int(seg_str)
  except ValueError:
    return None
  return drive_id, segment, camera


def prune_orphan_thumb_cache() -> None:
  """Delete thumbnail cache entries whose source HEVC no longer exists."""
  thumb_dir = Path(THUMB_CACHE_DIR)
  if not thumb_dir.is_dir():
    return
  from openpilot.selfdrive.appbridged.video_scanner import resolve_hevc_path
  for entry in thumb_dir.iterdir():
    if not entry.is_file():
      continue
    if entry.name.endswith(".tmp.jpg"):
      continue
    if not (parsed := _parse_thumb_cache_name(entry.name)):
      continue
    drive_id, segment, camera = parsed
    if resolve_hevc_path(drive_id, segment, camera) is None:
      delete_thumb_cache(entry)


def _is_cache_valid(cache_path: Path, source_path: str | None) -> bool:
  if not cache_path.is_file() or cache_path.stat().st_size <= 0:
    return False
  if not source_path or not os.path.isfile(source_path):
    delete_thumb_cache(cache_path)
    return False
  return cache_path.stat().st_mtime >= os.path.getmtime(source_path)


def needs_thumb_generation(hevc_path: str | None, cache_path: Path) -> bool:
  if not hevc_path:
    return False
  try:
    cst = cache_path.stat()
    if cst.st_size <= 0:
      return True
    if not os.path.isfile(hevc_path):
      delete_thumb_cache(cache_path)
      return False
    return cst.st_mtime < os.stat(hevc_path).st_mtime
  except OSError:
    return True


def read_cached_thumb(hevc_path: str | None, cache_path: Path) -> bytes | None:
  if not hevc_path or not _is_cache_valid(cache_path, hevc_path):
    return None
  try:
    data = cache_path.read_bytes()
    return data if data else None
  except OSError:
    return None


def _generate_thumb(hevc_path: str, cache_path: Path) -> bool:
  # SW decode, one ffmpeg at a time (video_protocol worker); HW remux path is slower for 1 frame.
  tmp = cache_path.with_suffix(".tmp.jpg")
  try:
    os.makedirs(cache_path.parent, exist_ok=True)
    tmp.unlink(missing_ok=True)
    cmd = [
      "ffmpeg", "-y", "-nostdin", "-loglevel", "error",
      "-probesize", "32768", "-analyzeduration", "0",
      *FFMPEG_HEVC_INPUT_ARGS, "-i", hevc_path,
      *FFMPEG_NO_SUBSTREAMS, "-threads", str(THUMB_FFMPEG_THREADS),
      "-vframes", "1", "-vf", "scale=160:-1:flags=fast_bilinear", "-q:v", "8",
      str(tmp),
    ]
    proc = subprocess.run(
      cmd, timeout=THUMB_FFMPEG_TIMEOUT_SEC,
      stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
    )
    if (rc := proc.returncode) != 0:
      cloudlog.warning(
        f"thumbnail generation failed: {cmd} exit {rc}: "
        f"{proc.stderr.decode('utf-8', errors='replace').strip()}"
      )
      return False
    if not tmp.is_file() or tmp.stat().st_size <= 0:
      return False
    os.replace(tmp, cache_path)
    return True
  except (OSError, subprocess.SubprocessError) as e:
    cloudlog.warning(f"thumbnail generation failed: {e}")
    try:
      tmp.unlink(missing_ok=True)
    except OSError:
      pass
    return False


def clear_thumb_cache() -> None:
  shutil.rmtree(THUMB_CACHE_DIR, ignore_errors=True)
  try:
    Path(CACHE_ROOT).rmdir()
  except OSError:
    pass

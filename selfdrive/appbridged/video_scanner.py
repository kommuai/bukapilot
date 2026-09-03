import os
import subprocess
import time
from pathlib import Path

from openpilot.selfdrive.appbridged.video_constants import (
  CAMERA_HEVC,
  MAX_DRIVES,
  MIN_SEGMENTS_PER_DRIVE,
  MEDIA_MOUNT,
  REALDATA_ROOT,
  SEGMENT_RE,
)

MOUNT_RETRY_SEC = 2.0
_last_mount_attempt = 0.0
_SD_UNSET = object()


def _try_remount_media() -> bool:
  global _last_mount_attempt
  if os.path.ismount(MEDIA_MOUNT):
    return True
  now = time.monotonic()
  if now - _last_mount_attempt < MOUNT_RETRY_SEC:
    return False
  _last_mount_attempt = now
  try:
    subprocess.run(["sudo", "mount", MEDIA_MOUNT], capture_output=True, text=True, timeout=10, check=False)
  except (OSError, subprocess.TimeoutExpired):
    return False
  return os.path.ismount(MEDIA_MOUNT)


def validate_storage(hw_helper, sd_status=_SD_UNSET) -> tuple[bool, str | None]:
  try:
    if (sd_status if sd_status is not _SD_UNSET else hw_helper.get_sd_status()) is not None:
      return False, "sd_invalid"
    if not os.path.ismount(MEDIA_MOUNT) and not _try_remount_media():
      return False, "realdata_unavailable"
    root = REALDATA_ROOT
    if not os.path.isdir(root):
      try:
        os.makedirs(root, exist_ok=True)
      except OSError:
        return False, "realdata_unavailable"
    os.listdir(root)
    return True, None
  except OSError:
    return False, "realdata_unavailable"


def _segment_folder_name(drive_id: str, segment: int) -> str:
  return f"{drive_id}--{segment}"


def resolve_segment_path(drive_id: str, segment: int) -> Path:
  return Path(REALDATA_ROOT) / _segment_folder_name(drive_id, segment)


def resolve_hevc_path(drive_id: str, segment: int, camera: str) -> Path | None:
  if not (hevc_name := CAMERA_HEVC.get(camera)):
    return None
  path = resolve_segment_path(drive_id, segment) / hevc_name
  return path if path.is_file() else None


def scan_drives(limit: int = MAX_DRIVES) -> list[dict]:
  root = Path(REALDATA_ROOT)
  try:
    entries = os.listdir(root)
  except OSError:
    return []

  grouped: dict[str, list[dict]] = {}
  for name in entries:
    if not (m := SEGMENT_RE.match(name)):
      continue
    drive_id, seg_str = m.group(1), m.group(2)
    try:
      segment = int(seg_str)
    except ValueError:
      continue

    seg_dir = root / name
    if not seg_dir.is_dir():
      continue

    has_road = (seg_dir / CAMERA_HEVC["road"]).is_file()
    has_wide = (seg_dir / CAMERA_HEVC["wide"]).is_file()
    if not has_road and not has_wide:
      continue

    grouped.setdefault(drive_id, []).append({
      "segment": segment,
      "hasRoad": has_road,
      "hasWide": has_wide,
      "roadHevc": str(seg_dir / CAMERA_HEVC["road"]) if has_road else None,
      "wideHevc": str(seg_dir / CAMERA_HEVC["wide"]) if has_wide else None,
    })

  drives = []
  for drive_id in sorted(grouped.keys(), reverse=True):
    segments = sorted(grouped[drive_id], key=lambda s: s["segment"])
    if len(segments) < MIN_SEGMENTS_PER_DRIVE:
      continue
    drives.append({"driveId": drive_id, "segments": segments})
    if len(drives) >= limit:
      break
  return drives


def get_drive(drive_id: str) -> dict | None:
  for drive in scan_drives(MAX_DRIVES):
    if drive["driveId"] == drive_id:
      return drive
  return None

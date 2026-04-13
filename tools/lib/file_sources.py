from collections.abc import Callable

import requests

from openpilot.tools.lib.comma_car_segments import get_url as get_comma_segments_url
from openpilot.tools.lib.openpilotci import get_url
from openpilot.tools.lib.filereader import DATA_ENDPOINT, file_exists, internal_source_available
from openpilot.tools.lib.route import Route, SegmentRange, FileName

# When passed a tuple of file names, each source will return the first that exists (rlog.zst, rlog.bz2)
FileNames = tuple[str, ...]
Source = Callable[[SegmentRange, list[int], FileNames], dict[int, str]]

InternalUnavailableException = Exception("Internal source not available")


def comma_api_source(sr: SegmentRange, seg_idxs: list[int], fns: FileNames) -> dict[int, str]:
  route = Route(sr.route_name)

  # comma api will have already checked if the file exists
  if fns == FileName.RLOG:
    return {seg: route.log_paths()[seg] for seg in seg_idxs if route.log_paths()[seg] is not None}
  else:
    return {seg: route.qlog_paths()[seg] for seg in seg_idxs if route.qlog_paths()[seg] is not None}


def internal_source(sr: SegmentRange, seg_idxs: list[int], fns: FileNames, endpoint_url: str = DATA_ENDPOINT) -> dict[int, str]:
  if not internal_source_available(endpoint_url):
    raise InternalUnavailableException

  def get_internal_url(sr: SegmentRange, seg, file):
    return f"{endpoint_url.rstrip('/')}/{sr.dongle_id}/{sr.log_id}/{seg}/{file}"

  return eval_source({seg: [get_internal_url(sr, seg, fn) for fn in fns] for seg in seg_idxs})


def openpilotci_source(sr: SegmentRange, seg_idxs: list[int], fns: FileNames) -> dict[int, str]:
  return eval_source({seg: [get_url(sr.route_name, seg, fn) for fn in fns] for seg in seg_idxs})


def comma_car_segments_source(sr: SegmentRange, seg_idxs: list[int], fns: FileNames) -> dict[int, str]:
  return eval_source({seg: get_comma_segments_url(sr.route_name, seg) for seg in seg_idxs})


# Depot objects may be .zst (loggerd uploader) or legacy .bz2 — same as route.cc loadFromKommuFallback.
_KOMMU_LOG_EXTS = (".zst", ".bz2")
# Same upper bound as tools/replay/route.cc loadFromKommuFallback.
_KOMMU_MAX_SEG_SCAN = 100


def _kommu_remote_log_probe(url: str) -> bool:
  try:
    head = requests.get(url, headers={"Range": "bytes=0-200"}, timeout=2)
    if head.status_code == 404 or head.status_code >= 400:
      return False
    if b'"message":"Not Found"' in head.content or (b"Not Found" in head.content and len(head.content) < 512):
      return False
    return True
  except requests.RequestException:
    return False


def _kommu_segment_has_any_log(base_url: str, prefix: str, seg: int) -> bool:
  for log_type in ("rlog", "qlog"):
    for ext in _KOMMU_LOG_EXTS:
      url = f"{base_url}/{prefix}{seg}---{log_type}{ext}"
      if _kommu_remote_log_probe(url):
        return True
  return False


def kommu_max_seg_number(sr: SegmentRange) -> int | None:
  """Max segment index (inclusive) on Kommu depot, or None. Contiguous from 0; stops at first empty index (replay semantics)."""
  base = f"https://web.kommu.ai/depot/upload/{sr.dongle_id}"
  prefix = f"{sr.dongle_id}---{sr.log_id}--"
  last = -1
  for seg in range(_KOMMU_MAX_SEG_SCAN):
    if not _kommu_segment_has_any_log(base, prefix, seg):
      break
    last = seg
  return last if last >= 0 else None


def _try_download_kommu_segment(base_url: str, prefix: str, seg: int, log_type: str, ext: str) -> str | None:
  remote_url = f"{base_url}/{prefix}{seg}---{log_type}{ext}"
  safe_prefix = prefix.replace("|", "_")
  local_path = f"/tmp/{safe_prefix}{seg}---{log_type}{ext}"

  try:
    if not _kommu_remote_log_probe(remote_url):
      return None

    r = requests.get(remote_url, timeout=120)
    r.raise_for_status()

    print(f"Downloading Kommu {log_type} segment {seg} ({ext})")
    with open(local_path, "wb") as f:
      f.write(r.content)
    return local_path
  except requests.RequestException:
    return None


def _kommu_first_available(base: str, prefix: str, seg: int, log_type: str) -> str | None:
  for ext in _KOMMU_LOG_EXTS:
    path = _try_download_kommu_segment(base, prefix, seg, log_type, ext)
    if path:
      return path
  return None


def kommu_source(sr: SegmentRange, seg_idxs: list[int], fns: FileNames) -> dict[int, str]:
  """Download logs from web.kommu.ai depot (fallback after comma/connect sources)."""
  dongle = sr.dongle_id
  log_id = sr.log_id
  base = f"https://web.kommu.ai/depot/upload/{dongle}"
  prefix = f"{dongle}---{log_id}--"
  out: dict[int, str] = {}

  for seg in seg_idxs:
    if fns == FileName.RLOG:
      path = _kommu_first_available(base, prefix, seg, "rlog")
      if not path:
        path = _kommu_first_available(base, prefix, seg, "qlog")
    else:
      path = _kommu_first_available(base, prefix, seg, "qlog")

    if path:
      out[seg] = path

  return out


def eval_source(files: dict[int, list[str] | str]) -> dict[int, str]:
  # Returns valid file URLs given a list of possible file URLs for each segment (e.g. rlog.bz2, rlog.zst)
  valid_files: dict[int, str] = {}

  for seg_idx, urls in files.items():
    if isinstance(urls, str):
      urls = [urls]

    # Add first valid file URL
    for url in urls:
      if file_exists(url):
        valid_files[seg_idx] = url
        break

  return valid_files

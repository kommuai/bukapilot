import msgpack
import os
import secrets
import threading
import time

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.appbridged.video_constants import (
  CHANNEL_VIDEO,
  HOTSPOT_WAIT_SEC,
  LIST_PAYLOAD_BUDGET,
  MAX_DRIVES,
  MSG,
  MEDIA_MOUNT,
  MP4_CACHE_DIR,
  THUMB_CACHE_DIR,
  THUMB_SEND_BURST,
  TRANSPORT_EXPIRES_SEC,
  VIDEO_HTTP_PORT,
  WIFI_FALLBACK_GRACE_SEC,
)
from openpilot.selfdrive.appbridged.video_hotspot import (
  compute_hotspot_credentials,
  disable_hotspot,
  enable_hotspot,
  get_hotspot_ip,
  is_hotspot_active,
  is_hotspot_joinable,
)
from openpilot.selfdrive.appbridged.video_http_server import VideoHttpServer
from openpilot.selfdrive.appbridged.video_mp4_cache import (
  Mp4ConvertJob,
  clear_mp4_cache,
  delete_mp4_cache,
  mp4_cache_path,
)
from openpilot.selfdrive.appbridged.video_scanner import (
  resolve_hevc_path,
  scan_drives,
  validate_storage,
)
from openpilot.selfdrive.appbridged.video_thumbnail_cache import (
  _generate_thumb,
  prune_orphan_thumb_cache,
  delete_thumb_cache,
  needs_thumb_generation,
  read_cached_thumb,
  thumb_cache_path,
)

HOTSPOT_WARM_SEC = 45.0

HOTSPOT_READY_RESEND_SEC = 2.0


class VideoProtocolHandler:
  def __init__(self, ble, hw_helper):
    self.ble = ble
    self.hw_helper = hw_helper
    self._lock = threading.RLock()
    self._convert_job: Mp4ConvertJob | None = None
    self._next_transfer_id = 1
    self._pending_download: dict | None = None
    self._hotspot_ready_sent_for: int | None = None
    self._hotspot_ready_last_at = 0.0
    self._cached_drives: list[dict] = []
    self._open_drive_id: str | None = None
    self._thumb_queue: list[dict] = []
    self._thumb_gen_busy = False
    self._thumb_gen_wake = threading.Event()
    self._thumb_gen_started = False
    self._wifi_session: dict | None = None
    self._http_server: VideoHttpServer | None = None
    self._hotspot_warm_until: float | None = None
    self._scan_in_progress = False
    self._list_req_pending = False
    self._clear_mp4_after_scan = False
    self._scan_gen = 0
    self._tick_storage_ok = True
    self._tick_storage_at = 0.0
    self._post_transfer_until = 0.0
    self._ensure_http_server()
    try:
      had_orphan_mp4 = bool(os.listdir(MP4_CACHE_DIR))
    except OSError:
      had_orphan_mp4 = False
    clear_mp4_cache()
    if had_orphan_mp4 and is_hotspot_active():
      disable_hotspot()

  def _ensure_http_server(self) -> None:
    if self._http_server is not None:
      return
    server = VideoHttpServer(VIDEO_HTTP_PORT)
    server.on_complete = self._on_wifi_http_complete
    server.on_aborted = self._on_wifi_http_aborted
    server.on_resumed = self._on_wifi_http_resumed
    if server.start():
      self._http_server = server

  def _slide_hotspot_warm_unlocked(self, cur_time: float) -> None:
    self._hotspot_warm_until = max(self._hotspot_warm_until or 0.0, cur_time + HOTSPOT_WARM_SEC)

  def _ensure_video_hotspot_for_download_unlocked(self, cur_time: float) -> None:
    self._slide_hotspot_warm_unlocked(cur_time)
    if not is_hotspot_joinable(fresh=True):
      enable_hotspot()

  @staticmethod
  def _hotspot_base_url() -> str | None:
    try:
      return f"http://{get_hotspot_ip()}:{VIDEO_HTTP_PORT}"
    except OSError:
      return None

  @staticmethod
  def _optional_int(msg: dict, key: str) -> int | None:
    raw = msg.get(key)
    if raw is None:
      return None
    try:
      return int(raw)
    except (TypeError, ValueError):
      return None

  @staticmethod
  def _append_thumb_items(queue: list[dict], drive_id: str, segment: int, seg: dict) -> None:
    if seg.get("hasRoad") and seg.get("roadHevc"):
      queue.append({
        "driveId": drive_id,
        "segment": segment,
        "camera": "road",
        "hevc": seg["roadHevc"],
        "cache": thumb_cache_path(drive_id, segment, "road"),
      })
    if seg.get("hasWide") and seg.get("wideHevc"):
      queue.append({
        "driveId": drive_id,
        "segment": segment,
        "camera": "wide",
        "hevc": seg["wideHevc"],
        "cache": thumb_cache_path(drive_id, segment, "wide"),
      })

  def _browse_clean(self, *, respond_list: bool = False) -> None:
    with self._lock:
      idle = self._scan_idle_unlocked()
    self._request_drive_scan(respond_list=respond_list, clear_mp4_when_idle=idle)

  def _orphan_prune_worker(self) -> None:
    try:
      ok, _ = validate_storage(self.hw_helper)
      if ok and self._ensure_cache_dirs():
        prune_orphan_thumb_cache()
    except Exception as e:
      cloudlog.error(f"orphan thumb prune error: {e}")

  def _mp4_cache_clear_worker(self) -> None:
    with self._lock:
      if not self._scan_idle_unlocked():
        return
    try:
      clear_mp4_cache()
    except Exception as e:
      cloudlog.error(f"clear_mp4_cache error: {e}")

  def _ble_reset_cleanup_worker(self) -> None:
    try:
      clear_mp4_cache()
    except Exception as e:
      cloudlog.error(f"clear_mp4_cache error: {e}")
    self._browse_clean()

  def _request_orphan_prune(self) -> None:
    threading.Thread(target=self._orphan_prune_worker, daemon=True).start()

  def _request_mp4_cache_clear(self) -> None:
    threading.Thread(target=self._mp4_cache_clear_worker, daemon=True).start()

  def _request_session_hygiene(self, *, include_scan: bool = True, respond_list: bool = False) -> None:
    with self._lock:
      idle = self._scan_idle_unlocked()
    if include_scan:
      self._request_drive_scan(respond_list=respond_list, clear_mp4_when_idle=idle)
    elif idle:
      self._request_mp4_cache_clear()
    else:
      self._request_orphan_prune()

  def _request_drive_scan(self, *, respond_list: bool = False, clear_mp4_when_idle: bool = False) -> None:
    with self._lock:
      if respond_list:
        self._list_req_pending = True
      if clear_mp4_when_idle:
        self._clear_mp4_after_scan = True
      if self._thumb_work_active_unlocked() and not respond_list:
        return
      if self._scan_in_progress:
        return
      self._scan_in_progress = True
      gen = self._scan_gen
    threading.Thread(target=self._drive_scan_worker, args=(gen,), daemon=True).start()

  def _drive_scan_worker(self, gen: int | None = None) -> None:
    drives, video_dl_valid = [], False
    try:
      ok, _ = validate_storage(self.hw_helper)
      video_dl_valid = ok
      if ok and self._ensure_cache_dirs():
        with self._lock:
          thumb_busy = self._thumb_work_active_unlocked()
          cached = list(self._cached_drives)
        if thumb_busy:
          drives = cached
        else:
          prune_orphan_thumb_cache()
          drives = scan_drives(MAX_DRIVES)
    except Exception as e:
      cloudlog.error(f"drive scan error: {e}")
    with self._lock:
      if gen is not None and gen != self._scan_gen:
        self._scan_in_progress = False
        return
      self._cached_drives = drives
      self._scan_in_progress = False
      respond_list = self._list_req_pending
      self._list_req_pending = False
      clear_mp4 = self._clear_mp4_after_scan and self._scan_idle_unlocked()
      self._clear_mp4_after_scan = False
    if clear_mp4:
      try:
        clear_mp4_cache()
      except Exception as e:
        cloudlog.error(f"clear_mp4_cache error: {e}")
    if respond_list:
      self._send(self._build_list_response(drives, video_dl_valid))

  def on_channel_active(self) -> None:
    with self._lock:
      storage_ok, _ = validate_storage(self.hw_helper)
      if not storage_ok:
        self._cached_drives = []
      elif not self._ensure_cache_dirs():
        return
    if storage_ok:
      self._browse_clean()
    else:
      self._request_mp4_cache_clear()

  def _reset_ble_session(self) -> None:
    with self._lock:
      self._cancel_thumbs()
      self._abort_active_unlocked()
      self._cached_drives = []
      self._list_req_pending = False
      self._clear_mp4_after_scan = False
      self._scan_in_progress = False
      self._scan_gen += 1
    threading.Thread(target=self._ble_reset_cleanup_worker, daemon=True).start()

  def on_ble_connected(self) -> None:
    with self._lock:
      if self._wifi_session is not None:
        return
    self._reset_ble_session()

  def on_ble_disconnected(self) -> None:
    self._reset_ble_session()

  def _send(self, obj: dict) -> bool:
    try:
      payload = msgpack.packb(obj, use_bin_type=True)
      self.ble.chunk_and_send(CHANNEL_VIDEO, payload)
      return True
    except ValueError as e:
      cloudlog.error(f"BLE video send failed: {e}")
      return False
    except Exception as e:
      cloudlog.error(f"video send error: {e}")
      return False

  def _send_error(self, reason: str):
    self._send({"msgType": MSG["ERROR"], "reason": reason})

  def send_list_keepalive(self) -> None:
    self._send({"msgType": MSG["KEEPALIVE"], "videoDlValid": self._storage_ok_for_tick()})

  def _ensure_cache_dirs(self) -> bool:
    try:
      os.makedirs(MP4_CACHE_DIR, exist_ok=True)
      os.makedirs(THUMB_CACHE_DIR, exist_ok=True)
      return True
    except OSError:
      return False

  def _is_busy(self) -> bool:
    return self._convert_job is not None or self._pending_download is not None or self._wifi_session is not None

  def _thumbnails_paused(self) -> bool:
    return self._is_busy()

  def _thumb_work_active_unlocked(self) -> bool:
    return self._open_drive_id and (self._thumb_queue or self._thumb_gen_busy)

  def _scan_idle_unlocked(self) -> bool:
    return not self._is_busy() and not self._thumb_work_active_unlocked()

  def _storage_ok_for_tick(self) -> bool:
    now = time.monotonic()
    interval = 0.5 if self._is_busy() else 2.0
    if now - self._tick_storage_at < interval:
      return self._tick_storage_ok
    self._tick_storage_at = now
    try:
      if self.hw_helper.get_sd_status() is not None:
        self._tick_storage_ok = False
        return False
      if not os.path.ismount(MEDIA_MOUNT):
        self._tick_storage_ok = False
        return False
      self._tick_storage_ok = True
      return True
    except OSError:
      self._tick_storage_ok = False
      return False

  def is_transfer_active(self) -> bool:
    return self._is_busy()

  def should_pause_video_keepalive(self) -> bool:
    return self._wifi_session is not None


  @staticmethod
  def _matches_download(
    *,
    transfer_id: int | None,
    drive_id: str | None,
    segment: int | None,
    camera: str | None,
    item_transfer_id: int | None = None,
    item_drive_id: str | None = None,
    item_segment: int | None = None,
    item_camera: str | None = None,
  ) -> bool:
    if transfer_id is not None and item_transfer_id == transfer_id:
      return True
    return drive_id is not None and item_drive_id == drive_id and item_segment == segment and item_camera == camera


  def _disable_transfer_hotspot(self, started_for_transfer: bool) -> None:
    self._hotspot_warm_until = None
    if started_for_transfer and is_hotspot_active():
      disable_hotspot()

  def _defer_delete_mp4(self, mp4_path) -> None:
    if not mp4_path:
      return
    threading.Thread(target=lambda p=mp4_path: delete_mp4_cache(p), daemon=True).start()

  def _cleanup_wifi_session_unlocked(self, *, delete_mp4: bool = True, disable_hotspot_on_cleanup: bool = True) -> None:
    session, self._wifi_session = self._wifi_session, None
    if not session:
      return
    if self._http_server and (transfer_id := session.get("transferId")) is not None:
      self._http_server.unregister(transfer_id)
    if delete_mp4 and (mp4 := session.get("mp4")) is not None:
      self._defer_delete_mp4(mp4)
    if disable_hotspot_on_cleanup and session.get("hotspot_started_for_transfer"):
      self._disable_transfer_hotspot(True)

  def _abort_active_unlocked(self):
    pending = self._pending_download
    hotspot_started = False
    if self._wifi_session:
      self._cleanup_wifi_session_unlocked(delete_mp4=True)
    elif convert_job := self._convert_job:
      convert_job.cancel()
      self._defer_delete_mp4(pending.get("mp4") if pending else None)
      if pending:
        hotspot_started = bool(pending.get("hotspot_started_for_transfer"))
      self._disable_transfer_hotspot(hotspot_started)
    elif pending:
      self._defer_delete_mp4(pending.get("mp4"))
      hotspot_started = bool(pending.get("hotspot_started_for_transfer"))
      self._disable_transfer_hotspot(hotspot_started)
    self._convert_job = None
    self._pending_download = None
    self._hotspot_ready_sent_for = None
    self._hotspot_ready_last_at = 0.0
    self._post_transfer_until = 0.0

  def _cancel_download(self, transfer_id: int | None = None, *, drive_id: str | None = None, segment: int | None = None, camera: str | None = None) -> bool:
    cancelled = False
    if (wifi := self._wifi_session) and self._matches_download(
      transfer_id=transfer_id,
      drive_id=drive_id,
      segment=segment,
      camera=camera,
      item_transfer_id=wifi.get("transferId"),
      item_drive_id=wifi.get("driveId"),
      item_segment=wifi.get("segment"),
      item_camera=wifi.get("camera"),
    ):
      self._cleanup_wifi_session_unlocked(delete_mp4=True, disable_hotspot_on_cleanup=True)
      cancelled = True
    if (pending := self._pending_download) and self._matches_download(
      transfer_id=transfer_id,
      drive_id=drive_id,
      segment=segment,
      camera=camera,
      item_transfer_id=pending.get("transferId"),
      item_drive_id=pending.get("driveId"),
      item_segment=pending.get("segment"),
      item_camera=pending.get("camera"),
    ):
      hotspot_started = bool(pending.get("hotspot_started_for_transfer"))
      if self._convert_job:
        self._convert_job.cancel()
      self._defer_delete_mp4(pending.get("mp4"))
      self._convert_job = None
      self._pending_download = None
      self._disable_transfer_hotspot(hotspot_started)
      cancelled = True
    return cancelled

  def _cancel_thumbs(self):
    self._open_drive_id = None
    self._thumb_queue.clear()
    self._thumb_gen_busy = False

  def _pick_thumb_gen_item_unlocked(self) -> dict | None:
    for x in self._thumb_queue:
      if needs_thumb_generation(x["hevc"], x["cache"]):
        return x
    return None

  def _wake_thumb_gen(self) -> None:
    if not self._thumb_gen_started:
      self._thumb_gen_started = True
      threading.Thread(target=self._thumb_gen_worker, daemon=True).start()
    self._thumb_gen_wake.set()

  def _thumb_gen_worker(self) -> None:
    while True:
      self._thumb_gen_wake.wait()
      while True:
        with self._lock:
          if not self._open_drive_id or self._thumbnails_paused():
            self._thumb_gen_wake.clear()
            break
          open_drive_id = self._open_drive_id
          item = self._pick_thumb_gen_item_unlocked()
          if not item:
            self._thumb_gen_wake.clear()
            break
          self._thumb_gen_busy = True
        ok = _generate_thumb(item["hevc"], item["cache"])
        send_now = False
        with self._lock:
          self._thumb_gen_busy = False
          if self._open_drive_id != open_drive_id:
            continue
          if not ok:
            if not os.path.isfile(item["hevc"]):
              delete_thumb_cache(item["cache"])
              self._drop_thumb_item(item)
            else:
              # Defer retry until next tick wake; avoid ffmpeg/SD spin.
              self._thumb_gen_wake.clear()
              break
          else:
            send_now = True
        if send_now:
          self._advance_thumbnails()

  def _send_thumbnail(self, drive_id: str, segment: int, camera: str, thumbnail_jpeg: bytes) -> bool:
    return self._send({
      "msgType": MSG["THUMBNAIL"],
      "driveId": drive_id,
      "segment": segment,
      "camera": camera,
      "thumbnailJpeg": thumbnail_jpeg,
    })

  def _drop_thumb_item(self, item: dict | None) -> None:
    if not item:
      return
    self._thumb_queue = [
      x for x in self._thumb_queue
      if (x["driveId"], x["segment"], x["camera"]) != (item["driveId"], item["segment"], item["camera"])
    ]

  def _send_ready_cached_thumbs(
    self,
    queue: list[dict],
    *,
    max_send: int | None = None,
  ) -> tuple[list[dict], bool]:
    """Send cached thumbnails in strict queue order (oldest segment first)."""
    sent = 0
    to_remove: list[dict] = []
    send_limit = max_send if max_send is not None else THUMB_SEND_BURST
    for item in queue:
      if not (jpeg := read_cached_thumb(item["hevc"], item["cache"])):
        break
      if not self._send_thumbnail(item["driveId"], item["segment"], item["camera"], jpeg):
        return to_remove, False
      to_remove.append(item)
      sent += 1
      if send_limit and sent >= send_limit:
        break
    return to_remove, True

  def _advance_thumbnails(self) -> None:
    with self._lock:
      if not self._open_drive_id or self._thumbnails_paused():
        return
      open_drive_id = self._open_drive_id
      queue_snapshot = list(self._thumb_queue)
      gen_busy = self._thumb_gen_busy
    if not queue_snapshot:
      if not gen_busy:
        self._wake_thumb_gen()
      return
    send_limit = 8 if gen_busy else THUMB_SEND_BURST
    if time.monotonic() < self._post_transfer_until:
      send_limit = min(send_limit, 2)
    to_remove, send_ok = self._send_ready_cached_thumbs(queue_snapshot, max_send=send_limit)
    if not send_ok:
      return
    with self._lock:
      if self._open_drive_id != open_drive_id:
        return
      for item in to_remove:
        self._drop_thumb_item(item)
    if not gen_busy:
      self._wake_thumb_gen()

  def _build_list_response(
    self,
    drives: list[dict],
    video_dl_valid: bool = True,
  ) -> dict:
    out_drives = []
    for drive in drives:
      drive_id = drive["driveId"]
      segments_out = []
      for seg in drive["segments"]:
        segment = seg["segment"]
        item: dict = {
          "segment": segment,
          "hasRoad": seg["hasRoad"],
          "hasWide": seg["hasWide"],
        }
        segments_out.append(item)
      out_drives.append({"driveId": drive_id, "segments": segments_out})

    resp = {"msgType": MSG["LIST_RESP"], "videoDlValid": video_dl_valid, "drives": out_drives}
    while out_drives:
      if len(msgpack.packb(resp)) <= LIST_PAYLOAD_BUDGET:
        break
      out_drives.pop()
      resp["drives"] = out_drives
      if not out_drives:
        break
    return resp

  def _handle_list_req(self, msg: dict):
    ok, _ = validate_storage(self.hw_helper)
    if not ok or not self._ensure_cache_dirs():
      return self._send({"msgType": MSG["LIST_RESP"], "videoDlValid": False, "drives": []})
    with self._lock:
      cached = list(self._cached_drives)
    if cached:
      self._send(self._build_list_response(cached, ok))
    self._browse_clean(respond_list=not cached)

  def _handle_drive_open(self, msg: dict):
    ok, _ = validate_storage(self.hw_helper)
    if not ok:
      return self._send_error("sd_invalid")
    if not self._ensure_cache_dirs():
      return self._send_error("realdata_unavailable")
    if not (drive_id := str(msg.get("driveId") or "").strip()):
      return self._send_error("drive_not_found")
    with self._lock:
      self._cancel_thumbs()
      if drive := next((d for d in self._cached_drives if d["driveId"] == drive_id), None):
        self._open_drive_id = drive_id
        queue: list[dict] = []
        for seg in drive["segments"]:
          self._append_thumb_items(queue, drive_id, seg["segment"], seg)
        self._thumb_queue = queue
    if not drive:
      self._browse_clean()
      return self._send_error("drive_not_found")
    self._wake_thumb_gen()

  def _handle_drive_close(self):
    with self._lock:
      self._cancel_thumbs()
      busy = self._is_busy()
    if not busy:
      self._browse_clean()

  def _start_download(self, drive_id: str, segment: int, camera: str):
    self._cancel_thumbs()
    ok, reason = validate_storage(self.hw_helper)
    if not ok:
      self._send_error(reason or "sd_invalid")
      return
    if camera not in ("road", "wide"):
      self._send_error("camera_not_found")
      return
    if not (hevc := resolve_hevc_path(drive_id, segment, camera)):
      self._send_error("camera_not_found")
      return
    if not self._ensure_cache_dirs():
      self._send_error("realdata_unavailable")
      return

    mp4_path = mp4_cache_path(drive_id, segment, camera)
    now = time.monotonic()
    self._ensure_video_hotspot_for_download_unlocked(now)
    self._hotspot_ready_sent_for = None
    self._hotspot_ready_last_at = 0.0
    transfer_id = self._next_transfer_id
    self._next_transfer_id += 1
    self._pending_download = {
      "transferId": transfer_id,
      "driveId": drive_id,
      "segment": segment,
      "camera": camera,
      "hevc": str(hevc),
      "mp4": mp4_path,
      "hotspot_started_for_transfer": True,
    }
    self._convert_job = Mp4ConvertJob(str(hevc), mp4_path)
    self._send_convert_started()
    self._try_send_hotspot_ready(now)


  def _try_send_hotspot_ready(self, cur_time: float) -> None:
    if (pending := self._pending_download) and self._convert_job:
      transfer_id = pending["transferId"]
    elif (session := self._wifi_session) and session.get("phase") == "waiting_hotspot":
      pending, transfer_id = session, session["transferId"]
    else:
      return
    if self._hotspot_ready_sent_for == transfer_id and (cur_time - self._hotspot_ready_last_at) < HOTSPOT_READY_RESEND_SEC:
      return
    if not is_hotspot_joinable(fresh=True):
      return
    ssid, password = compute_hotspot_credentials()
    if not (base_url := self._hotspot_base_url()):
      return
    hotspot_ip = base_url.rsplit(":", 1)[0].removeprefix("http://")
    sent = self._send({
      "msgType": MSG["HOTSPOT_READY"],
      "transferId": transfer_id,
      "driveId": pending["driveId"],
      "segment": pending["segment"],
      "camera": pending["camera"],
      "ssid": ssid,
      "password": password,
      "hotspotIp": hotspot_ip,
      "baseUrl": base_url,
    })
    if sent:
      self._hotspot_ready_sent_for = transfer_id
      self._hotspot_ready_last_at = cur_time
      cloudlog.info(
        f"video hotspot ready sent transfer={transfer_id} ip={hotspot_ip} "
        f"during_convert={self._convert_job is not None}"
      )

  def _send_convert_started(self) -> None:
    if not (pending := self._pending_download):
      return
    self._send({
      "msgType": MSG["CONVERT_PROGRESS"],
      "transferId": pending["transferId"],
      "driveId": pending["driveId"],
      "segment": pending["segment"],
      "camera": pending["camera"],
      "phase": "started",
    })

  def _try_finish_convert(self, cur_time: float):
    if not (convert_job := self._convert_job) or not (pending := self._pending_download):
      return
    done, ok, err = convert_job.poll()
    if not done:
      return
    self._convert_job = None

    if not ok:
      hotspot_started = bool(pending.get("hotspot_started_for_transfer"))
      self._pending_download = None
      self._defer_delete_mp4(pending["mp4"])
      self._disable_transfer_hotspot(hotspot_started)
      if err != "conversion_cancelled":
        self._send_error(err or "conversion_failed")
      return

    mp4_path = pending["mp4"]
    if not mp4_path.is_file() or mp4_path.stat().st_size <= 0:
      hotspot_started = bool(pending.get("hotspot_started_for_transfer"))
      self._pending_download = None
      self._defer_delete_mp4(mp4_path)
      self._disable_transfer_hotspot(hotspot_started)
      self._send_error("file_not_ready")
      return

    self._pending_download = None
    self._begin_wifi_download(pending, mp4_path, cur_time)

  def _begin_wifi_download(self, pending: dict, mp4_path, cur_time: float) -> None:
    self._ensure_video_hotspot_for_download_unlocked(cur_time)
    ssid, password = compute_hotspot_credentials()
    joinable = is_hotspot_joinable(fresh=True)
    self._wifi_session = {
      "transferId": pending["transferId"],
      "driveId": pending["driveId"],
      "segment": pending["segment"],
      "camera": pending["camera"],
      "mp4": mp4_path,
      "ssid": ssid,
      "password": password,
      "totalBytes": mp4_path.stat().st_size,
      "phase": "ready_hotspot" if joinable else "waiting_hotspot",
      "started_at": cur_time,
      "hotspot_started_for_transfer": True,
      "transport_sent": False,
    }
    cloudlog.info(
      f"video wifi begin transfer={pending['transferId']} "
      f"phase={self._wifi_session['phase']} joinable={joinable} "
      f"bytes={self._wifi_session['totalBytes']}"
    )

  def _send_wifi_transport(self, session: dict, cur_time: float) -> bool:
    if session.get("transport_sent"):
      return True
    if not is_hotspot_joinable(fresh=True):
      return False
    if not self._http_server:
      return False
    transfer_id = session["transferId"]
    if not (base_url := self._hotspot_base_url()):
      return False
    hotspot_ip = base_url.rsplit(":", 1)[0].removeprefix("http://")
    token = secrets.token_urlsafe(24)
    expires_at = time.monotonic() + TRANSPORT_EXPIRES_SEC
    self._http_server.register(transfer_id, token, session["mp4"], expires_at)
    sent = self._send({
      "msgType": MSG["DOWNLOAD_TRANSPORT"],
      "transferId": transfer_id,
      "driveId": session["driveId"],
      "segment": session["segment"],
      "camera": session["camera"],
      "baseUrl": base_url,
      "token": token,
      "ssid": session["ssid"],
      "password": session["password"],
      "totalBytes": session["totalBytes"],
      "expiresSec": TRANSPORT_EXPIRES_SEC,
    })
    if not sent:
      self._http_server.unregister(transfer_id)
      return False
    session["token"] = token
    session["transport_sent"] = True
    session["transport_sent_at"] = cur_time
    session["download_start_deadline"] = cur_time + WIFI_FALLBACK_GRACE_SEC
    session["expires_at"] = cur_time + TRANSPORT_EXPIRES_SEC
    session["phase"] = "awaiting_http"
    cloudlog.info(
      f"video wifi transport sent transfer={transfer_id} "
      f"bytes={session.get('totalBytes')} ip={hotspot_ip} grace_sec={WIFI_FALLBACK_GRACE_SEC}"
    )
    return True


  def _fail_wifi_session(self, cur_time: float, reason: str = "wifi_download_failed") -> None:
    if not (session := self._wifi_session):
      return
    transfer_id = session["transferId"]
    mp4_path = session["mp4"]
    if self._http_server:
      self._http_server.unregister(transfer_id)
    hotspot_started = bool(session.get("hotspot_started_for_transfer"))
    self._wifi_session = None
    self._defer_delete_mp4(mp4_path)
    self._disable_transfer_hotspot(hotspot_started)
    self._send_error(reason)

  def _finish_wifi_transfer_success(self) -> None:
    if not (session := self._wifi_session):
      return
    transfer_id = session.get("transferId")
    mp4_path = session.get("mp4")
    self._wifi_session = None
    if self._http_server and transfer_id is not None:
      self._http_server.unregister(transfer_id)
    self._defer_delete_mp4(mp4_path)
    now = time.monotonic()
    if session.get("hotspot_started_for_transfer") and is_hotspot_active():
      self._slide_hotspot_warm_unlocked(now)
    else:
      self._hotspot_warm_until = None
    self._post_transfer_until = now + 1.0

  def _on_wifi_http_resumed(self, transfer_id: int, start: int, total: int) -> None:
    with self._lock:
      if (session := self._wifi_session) and session.get("transferId") == transfer_id:
        session.pop("http_aborted", None)
        session.pop("http_aborted_at", None)
        session["http_abort_grace_sec"] = 25.0 if total > 0 and start >= total * 0.95 else 20.0

  def _on_wifi_http_complete(self, transfer_id: int, _total_bytes: int) -> None:
    with self._lock:
      if (session := self._wifi_session) and session.get("transferId") == transfer_id:
        session["http_finished_at"] = time.monotonic()
        session["phase"] = "complete"

  def _on_wifi_http_aborted(self, transfer_id: int) -> None:
    with self._lock:
      if (session := self._wifi_session) and session.get("transferId") == transfer_id:
        session["http_aborted"] = True
        session["http_aborted_at"] = time.monotonic()
        started = self._http_server and self._http_server.is_started(transfer_id)
        session["http_abort_grace_sec"] = 25.0 if started else 8.0

  def _tick_wifi_session(self, cur_time: float) -> None:
    if not (session := self._wifi_session):
      return
    if (phase := session.get("phase")) == "complete":
      return self._finish_wifi_transfer_success()
    if phase == "waiting_hotspot":
      if not is_hotspot_joinable(fresh=True) and (cur_time - session["started_at"]) < HOTSPOT_WAIT_SEC:
        return
      if not is_hotspot_joinable(fresh=True):
        return self._fail_wifi_session(cur_time)
      session["phase"] = "ready_hotspot"
      phase = "ready_hotspot"
    if phase == "ready_hotspot":
      if self._send_wifi_transport(session, cur_time):
        return
      if (cur_time - session["started_at"]) < HOTSPOT_WAIT_SEC:
        return
      return self._fail_wifi_session(cur_time)
    if phase != "awaiting_http":
      return
    tid = session["transferId"]
    if session.get("http_aborted"):
      join_deadline = session.get("download_start_deadline", cur_time)
      if cur_time < join_deadline:
        session.pop("http_aborted", None)
        session.pop("http_aborted_at", None)
        return
      aborted_at = session.get("http_aborted_at", cur_time)
      grace = session.get("http_abort_grace_sec", 8.0)
      if cur_time - aborted_at < grace:
        return
      return self._fail_wifi_session(cur_time)
    if self._http_server and self._http_server.is_complete(tid):
      session["phase"] = "complete"
      return self._finish_wifi_transfer_success()
    if cur_time > session.get("expires_at", cur_time):
      return self._fail_wifi_session(cur_time)
    if cur_time <= session.get("download_start_deadline", cur_time):
      return
    if self._http_server and not self._http_server.is_started(tid):
      return self._fail_wifi_session(cur_time)

  def _handle_download_req(self, msg: dict):
    if not (drive_id := str(msg.get("driveId") or "")):
      self._send_error("drive_not_found")
      return
    try:
      segment = int(msg.get("segment"))
    except (TypeError, ValueError):
      self._send_error("segment_not_found")
      return
    camera = str(msg.get("camera") or "")
    if self._is_busy():
      cloudlog.info("video download superseding in-flight transfer")
      self._abort_active_unlocked()
    self._start_download(drive_id, segment, camera)
    self._try_send_hotspot_ready(time.monotonic())

  def _handle_cancel(self, msg: dict):
    transfer_id = self._optional_int(msg, "transferId")
    drive_id = str(msg.get("driveId") or "").strip() or None
    camera = str(msg.get("camera") or "").strip() or None
    segment = self._optional_int(msg, "segment")
    if transfer_id is None and (drive_id is None or camera is None or segment is None):
      return False
    cancelled = self._cancel_download(transfer_id, drive_id=drive_id, segment=segment, camera=camera)
    return cancelled

  def handle_message(self, msg: dict, cur_time: float):
    if not msg or not msg.get("msgType"):
      return
    t = msg.get("msgType")
    if t == MSG["LIST_REQ"]:
      self._handle_list_req(msg)
    elif t == MSG["DRIVE_OPEN"]:
      self._handle_drive_open(msg)
    elif t == MSG["DRIVE_CLOSE"]:
      self._handle_drive_close()
    elif t == MSG["DOWNLOAD_REQ"]:
      with self._lock:
        self._handle_download_req(msg)
    elif t == MSG["DOWNLOAD_CANCEL"]:
      with self._lock:
        cancelled = self._handle_cancel(msg)
      if cancelled:
        self._request_session_hygiene(include_scan=False)

  def _tick_hotspot_warm(self, cur_time: float) -> None:
    if self._is_busy():
      self._slide_hotspot_warm_unlocked(cur_time)
      return
    if (warm_until := self._hotspot_warm_until) is None or cur_time < warm_until:
      return
    self._hotspot_warm_until = None
    if is_hotspot_active():
      disable_hotspot()

  def tick(self, cur_time: float):
    thumbs_ok = False
    storage_ok = self._storage_ok_for_tick()
    with self._lock:
      if not storage_ok and (self._convert_job or self._pending_download or self._wifi_session):
        self._abort_active_unlocked()
        self._send_error("transfer_timeout")
        return
      self._try_finish_convert(cur_time)
      self._try_send_hotspot_ready(cur_time)
      self._tick_wifi_session(cur_time)
      self._tick_hotspot_warm(cur_time)
      thumbs_ok = bool(self._open_drive_id) and not self._thumbnails_paused()
    if thumbs_ok:
      self._advance_thumbnails()

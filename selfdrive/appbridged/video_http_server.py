import http.server
import socketserver
import threading
import time
from pathlib import Path
from urllib.parse import parse_qs, urlparse


class _VideoDownloadHandler(http.server.BaseHTTPRequestHandler):
  server_version = "KommuVideoHTTP/1.0"

  def log_message(self, format, *args):
    return

  def _session(self):
    wrapper: "VideoHttpServer" = self.server.video_server  # type: ignore[attr-defined]
    parsed = urlparse(self.path)
    parts = [p for p in parsed.path.split("/") if p]
    if len(parts) != 3 or parts[0] != "video" or parts[1] != "download":
      return None, None
    try:
      transfer_id = int(parts[2])
    except ValueError:
      return None, None
    token = (parse_qs(parsed.query).get("token") or [None])[0]
    return wrapper.get_session(transfer_id, token), transfer_id

  def do_GET(self):
    parsed = urlparse(self.path)
    parts = [p for p in parsed.path.split("/") if p]
    if parts == ["health"]:
      self.send_response(200)
      self.send_header("Content-Type", "text/plain")
      self.send_header("Content-Length", "2")
      self.end_headers()
      self.wfile.write(b"ok")
      return

    session, transfer_id = self._session()
    wrapper: "VideoHttpServer" = self.server.video_server  # type: ignore[attr-defined]
    if not session:
      self.send_error(404)
      return
    path = Path(session["path"])
    if not path.is_file():
      self.send_error(404)
      return
    total = path.stat().st_size
    start = 0
    end = total - 1
    range_header = self.headers.get("Range")
    if range_header and range_header.startswith("bytes="):
      spec = range_header[6:].strip().split("-", 1)
      if spec[0]:
        start = max(0, int(spec[0]))
      if len(spec) > 1 and spec[1]:
        end = min(total - 1, int(spec[1]))
    if start >= total:
      self.send_error(416)
      return
    if start == 0:
      wrapper.mark_started(transfer_id)
    else:
      wrapper.mark_resumed(transfer_id, start, total)
    length = end - start + 1
    try:
      if start == 0 and end >= total - 1:
        self.send_response(200)
        self.send_header("Content-Type", "video/mp4")
        self.send_header("Content-Length", str(total))
      else:
        self.send_response(206)
        self.send_header("Content-Type", "video/mp4")
        self.send_header("Content-Range", f"bytes {start}-{end}/{total}")
        self.send_header("Content-Length", str(length))
      self.send_header("Connection", "close")
      self.end_headers()
      with path.open("rb") as f:
        f.seek(start)
        remaining = length
        while remaining > 0:
          chunk = f.read(min(256 * 1024, remaining))
          if not chunk:
            break
          self.wfile.write(chunk)
          remaining -= len(chunk)
      if end >= total - 1:
        wrapper.mark_complete(transfer_id, total)
    except (BrokenPipeError, ConnectionResetError, OSError):
      wrapper.mark_aborted(transfer_id)


class _ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
  daemon_threads = True
  allow_reuse_address = True
  video_server: "VideoHttpServer | None" = None


class VideoHttpServer:
  def __init__(self, port: int):
    self.port = port
    self._sessions: dict[int, dict] = {}
    self._lock = threading.Lock()
    self._httpd: _ThreadingHTTPServer | None = None
    self._thread: threading.Thread | None = None
    self.on_complete = None
    self.on_aborted = None
    self.on_resumed = None

  def start(self) -> bool:
    with self._lock:
      if self._httpd is not None:
        return True
      try:
        self._httpd = _ThreadingHTTPServer(("0.0.0.0", self.port), _VideoDownloadHandler)
        self._httpd.video_server = self
      except OSError:
        self._httpd = None
        return False
      self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
      self._thread.start()
      return True

  def stop(self) -> None:
    with self._lock:
      httpd, self._httpd = self._httpd, None
      thread, self._thread = self._thread, None
      self._sessions.clear()
    if httpd:
      httpd.shutdown()
      httpd.server_close()
    if thread:
      thread.join(timeout=2)

  def register(self, transfer_id: int, token: str, mp4_path: Path, expires_at: float) -> None:
    with self._lock:
      self._sessions[transfer_id] = {
        "token": token,
        "path": str(mp4_path),
        "expires_at": expires_at,
        "started": False,
        "complete": False,
        "aborted": False,
      }

  def unregister(self, transfer_id: int) -> None:
    with self._lock:
      self._sessions.pop(transfer_id, None)

  def get_session(self, transfer_id: int, token: str | None) -> dict | None:
    with self._lock:
      if not (session := self._sessions.get(transfer_id)) or not token or token != session["token"]:
        return None
      if time.monotonic() > session["expires_at"]:
        return None
      return session

  def mark_started(self, transfer_id: int) -> None:
    with self._lock:
      if not (session := self._sessions.get(transfer_id)) or session["started"]:
        return
      session["started"] = True

  def mark_complete(self, transfer_id: int, total_bytes: int) -> None:
    with self._lock:
      if not (session := self._sessions.get(transfer_id)):
        return
      session["complete"] = True
    if self.on_complete:
      self.on_complete(transfer_id, total_bytes)

  def mark_resumed(self, transfer_id: int, start: int, total: int) -> None:
    with self._lock:
      if not (session := self._sessions.get(transfer_id)):
        return
      session["aborted"] = False
      session["resume_start"] = start
      session["resume_total"] = total
    if self.on_resumed:
      self.on_resumed(transfer_id, start, total)

  def mark_aborted(self, transfer_id: int) -> None:
    with self._lock:
      if not (session := self._sessions.get(transfer_id)):
        return
      session["aborted"] = True
    if self.on_aborted:
      self.on_aborted(transfer_id)

  def is_started(self, transfer_id: int) -> bool:
    with self._lock:
      session = self._sessions.get(transfer_id)
      return bool(session and session["started"])

  def is_complete(self, transfer_id: int) -> bool:
    with self._lock:
      session = self._sessions.get(transfer_id)
      return bool(session and session["complete"])


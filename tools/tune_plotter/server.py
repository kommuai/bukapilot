#!/usr/bin/env python3
import argparse
import json
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cereal.messaging as messaging


HOST = "0.0.0.0"
PORT = 8080
STATIC_DIR = Path(__file__).resolve().parent / "static"


class SampleStore:
  def __init__(self, maxlen: int):
    self._samples = deque(maxlen=maxlen)
    self._idx = 0
    self._lock = threading.Lock()

  def add(self, sample: dict) -> None:
    with self._lock:
      self._idx += 1
      sample["idx"] = self._idx
      self._samples.append(sample)

  def latest_idx(self) -> int:
    with self._lock:
      return self._idx

  def since(self, last_idx: int, limit: int) -> list[dict]:
    with self._lock:
      out = [s for s in self._samples if s["idx"] > last_idx]
      return out[:limit]


STORE = SampleStore(maxlen=60 * 60)


def safe_get(obj, name: str, default):
  try:
    return getattr(obj, name)
  except Exception:
    return default


def sampler_loop(sample_hz: float):
  sm = messaging.SubMaster(["controlsState", "carState", "longitudinalPlan", "radarState", "carOutput"])
  dt = 1.0 / sample_hz
  last_t = 0.0
  while True:
    sm.update(50)
    now = time.time()
    if (now - last_t) < dt:
      continue
    last_t = now

    cs = sm["controlsState"]
    car_state = sm["carState"]
    long_plan = sm["longitudinalPlan"]
    lead_one = sm["radarState"].leadOne
    car_output = sm["carOutput"]

    lat_source = "none"
    lat_p = 0.0
    lat_i = 0.0
    lat_f = 0.0
    if cs.lateralControlState.which() == "pidState":
      lat_source = "pidState"
      lat_p = float(cs.lateralControlState.pidState.p)
      lat_i = float(cs.lateralControlState.pidState.i)
      lat_f = float(cs.lateralControlState.pidState.f)
    elif cs.lateralControlState.which() == "torqueState":
      lat_source = "torqueState"
      lat_p = float(cs.lateralControlState.torqueState.p)
      lat_i = float(cs.lateralControlState.torqueState.i)
      lat_f = float(cs.lateralControlState.torqueState.f)

    lead_prob = 0.0
    if lead_one.status:
      lead_prob = float(safe_get(lead_one, "prob", safe_get(lead_one, "modelProb", 0.0)))

    STORE.add({
      "t": now,
      "ctrlP": float(cs.upAccelCmd),
      "ctrlI": float(cs.uiAccelCmd),
      "ctrlF": float(cs.ufAccelCmd),
      "vEgo": float(car_state.vEgo) * 3.6,
      "vCruise": float(car_state.vCruise),
      "overrideActive": bool(safe_get(long_plan, "dangerOverrideActive", False)),
      "latSource": lat_source,
      "latP": lat_p,
      "latI": lat_i,
      "latF": lat_f,
      "stockAccCmd": float(car_state.stockAccelCmd),
      "dngaBrakeMagnitudeRaw": float(car_output.actuatorsOutput.brake),
      "leadRelDist": float(lead_one.dRel) if lead_one.status else 0.0,
      "leadRelSpeed": float(lead_one.vRel) if lead_one.status else 0.0,
      "leadProb": lead_prob,
    })


class Handler(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"

  def log_message(self, fmt: str, *args):
    return

  def do_GET(self):
    parsed = urlparse(self.path)
    if parsed.path == "/":
      return self._serve_file(STATIC_DIR / "index.html", "text/html; charset=utf-8")
    if parsed.path == "/health":
      return self._send_json({"ok": True, "latestIdx": STORE.latest_idx()})
    if parsed.path == "/stream":
      return self._serve_sse(parsed.query)
    if parsed.path.startswith("/static/"):
      requested = parsed.path.replace("/static/", "", 1)
      target = (STATIC_DIR / requested).resolve()
      if not str(target).startswith(str(STATIC_DIR.resolve())):
        self.send_error(HTTPStatus.FORBIDDEN)
        return
      if target.exists() and target.is_file():
        mime = "text/plain; charset=utf-8"
        if target.suffix == ".js":
          mime = "application/javascript; charset=utf-8"
        elif target.suffix == ".css":
          mime = "text/css; charset=utf-8"
        elif target.suffix == ".html":
          mime = "text/html; charset=utf-8"
        return self._serve_file(target, mime)

    self.send_error(HTTPStatus.NOT_FOUND)

  def _send_json(self, data: dict):
    b = json.dumps(data).encode("utf-8")
    self.send_response(HTTPStatus.OK)
    self.send_header("Content-Type", "application/json; charset=utf-8")
    self.send_header("Content-Length", str(len(b)))
    self.send_header("Cache-Control", "no-store")
    self.end_headers()
    self.wfile.write(b)

  def _serve_file(self, path: Path, content_type: str):
    if not path.exists():
      self.send_error(HTTPStatus.NOT_FOUND)
      return
    b = path.read_bytes()
    self.send_response(HTTPStatus.OK)
    self.send_header("Content-Type", content_type)
    self.send_header("Content-Length", str(len(b)))
    self.send_header("Cache-Control", "no-store")
    self.end_headers()
    self.wfile.write(b)

  def _serve_sse(self, query: str):
    qs = parse_qs(query)
    hz = 20.0
    limit = 64
    try:
      hz = max(1.0, min(60.0, float(qs.get("hz", ["20"])[0])))
    except Exception:
      hz = 20.0
    try:
      limit = max(1, min(512, int(qs.get("limit", ["64"])[0])))
    except Exception:
      limit = 64

    interval = 1.0 / hz
    self.send_response(HTTPStatus.OK)
    self.send_header("Content-Type", "text/event-stream")
    self.send_header("Cache-Control", "no-cache")
    self.send_header("Connection", "keep-alive")
    self.end_headers()

    last_idx = 0
    try:
      while True:
        rows = STORE.since(last_idx, limit=limit)
        if rows:
          last_idx = rows[-1]["idx"]
          payload = json.dumps(rows, separators=(",", ":"))
          self.wfile.write(f"data:{payload}\n\n".encode("utf-8"))
          self.wfile.flush()
        time.sleep(interval)
    except (BrokenPipeError, ConnectionResetError):
      return


def main():
  parser = argparse.ArgumentParser(description="Live tune plotter (on-demand)")
  parser.add_argument("--sample-hz", type=float, default=20.0, help="Backend sampling rate (Hz)")
  args = parser.parse_args()

  if not STATIC_DIR.exists():
    raise RuntimeError(f"Missing static dir: {STATIC_DIR}")

  t = threading.Thread(target=sampler_loop, args=(args.sample_hz,), daemon=True)
  t.start()

  server = ThreadingHTTPServer((HOST, PORT), Handler)
  print(f"Tune plotter is up at http://{HOST}:{PORT} (port {PORT})")
  server.serve_forever()


if __name__ == "__main__":
  main()


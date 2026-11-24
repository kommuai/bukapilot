#!/usr/bin/env python3
"""
Stream replay UI (with model overlays) over WebRTC.

This mirrors the core overlay behavior from tools/replay/ui.py:
  - road camera frame decode
  - model path/laneline projection
  - lead indicator overlay

Uses OpenCV for camera-space warp to match replay UI projection.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import cereal.messaging as messaging
import numpy as np
from aiohttp import web
from PIL import Image, ImageDraw, ImageTransform

from msgq.visionipc import VisionIpcClient, VisionStreamType
from openpilot.common.transformations.camera import DEVICE_CAMERAS, get_view_frame_from_calib_frame
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA

try:
  from aiortc import RTCPeerConnection, RTCSessionDescription
  from aiortc.mediastreams import VideoStreamTrack
  from av import VideoFrame
except Exception as e:  # pragma: no cover
  raise ImportError(
    "ui_webrtc.py requires aiortc + av. Install with: pip install aiortc av"
  ) from e


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8083
VIEWER_FILE = Path(__file__).with_name("ui_webrtc_viewer.html")


@dataclass
class LatestFrame:
  frame: np.ndarray | None = None
  lock: threading.Lock = field(default_factory=threading.Lock)

class Calibration:
  def __init__(self, rpy: np.ndarray, intrinsic: np.ndarray, calib_scale: float):
    self.intrinsic = intrinsic
    self.extrinsics_matrix = get_view_frame_from_calib_frame(rpy[0], rpy[1], rpy[2], 0.0)[:, :3]
    self.zoom = calib_scale

  def car_space_to_bb(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
    car_space_projective = np.column_stack((x, y, z)).T
    ep = self.extrinsics_matrix.dot(car_space_projective)
    kep = self.intrinsic.dot(ep)
    # Avoid divide-by-zero for points behind/at camera plane.
    denom = kep[-1, :]
    valid = np.abs(denom) > 1e-6
    pts = np.full((kep.shape[1], 2), np.nan, dtype=np.float64)
    if np.any(valid):
      pts[valid] = (kep[:-1, valid] / denom[valid]).T
    return pts / self.zoom


def _to_points(path, z_off: float = 0.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  return np.asarray(path.x), np.asarray(path.y), np.asarray(path.z) + z_off


def _draw_projected_polyline(draw: ImageDraw.ImageDraw, pts: np.ndarray, width: int, height: int, color: tuple[int, int, int]) -> None:
  valid_seq: list[tuple[int, int]] = []
  for p in pts:
    if not np.isfinite(p[0]) or not np.isfinite(p[1]):
      if len(valid_seq) >= 2:
        draw.line(valid_seq, fill=color, width=2)
      valid_seq = []
      continue
    x, y = int(round(p[0])), int(round(p[1]))
    if 0 <= x < width and 0 <= y < height:
      valid_seq.append((x, y))
    else:
      if len(valid_seq) >= 2:
        draw.line(valid_seq, fill=color, width=2)
      valid_seq = []
  if len(valid_seq) >= 2:
    draw.line(valid_seq, fill=color, width=2)


def _overlay_model(img: np.ndarray, model, calibration: Calibration | None) -> np.ndarray:
  if calibration is None:
    return img

  h, w = img.shape[:2]
  pil = Image.fromarray(img)
  draw = ImageDraw.Draw(pil)

  for lead in model.leadsV3:
    if lead.prob < 0.5:
      continue
    x, y = lead.x[0] - RADAR_TO_CAMERA, lead.y[0]
    px = int(round(-y * 6 + (w // 2)))
    py = int(round(h - x * 4))
    if 0 <= px < w and 0 <= py < h:
      draw.ellipse((px - 5, py - 5, px + 5, py + 5), outline=(255, 255, 0), width=2)

  for lane, prob in zip(model.laneLines, model.laneLineProbs, strict=True):
    x, y, z = _to_points(lane)
    pts = calibration.car_space_to_bb(x, y, z)
    _draw_projected_polyline(draw, pts, w, h, (0, int(255 * prob), 0))

  for edge, std in zip(model.roadEdges, model.roadEdgeStds, strict=True):
    x, y, z = _to_points(edge)
    pts = calibration.car_space_to_bb(x, y, z)
    prob = max(1 - std, 0)
    _draw_projected_polyline(draw, pts, w, h, (int(255 * prob), 0, 0))

  x, y, z = _to_points(model.position, z_off=1.22)
  pts = calibration.car_space_to_bb(x, y, z)
  _draw_projected_polyline(draw, pts, w, h, (255, 0, 0))

  return np.asarray(pil)


def _warp_to_replay_view(rgb: np.ndarray, bb_scale: float, out_size: tuple[int, int] = (640, 480)) -> np.ndarray:
  # Match ui.py's cv2.warpAffine(..., WARP_INVERSE_MAP) using PIL's output->input affine map.
  out_w, out_h = out_size
  pil = Image.fromarray(rgb)
  transform = ImageTransform.AffineTransform((bb_scale, 0.0, 0.0, 0.0, bb_scale, 0.0))
  warped = pil.transform((out_w, out_h), transform, resample=Image.Resampling.BILINEAR)
  return np.asarray(warped)


class ReplayUIRenderer(threading.Thread):
  def __init__(self, ip_address: str):
    super().__init__(daemon=True)
    self.ip_address = ip_address
    self.latest = LatestFrame()
    self.running = True

  def stop(self) -> None:
    self.running = False

  @staticmethod
  def decode_nv12_to_rgb(data: bytes, width: int, height: int, stride: int) -> np.ndarray:
    imgff = np.frombuffer(data, dtype=np.uint8).reshape((len(data) // stride, stride))
    nv12 = np.ascontiguousarray(imgff[: height * 3 // 2, : width])
    frame = VideoFrame.from_ndarray(nv12, format="nv12")
    return frame.reformat(width=width, height=height, format="rgb24").to_ndarray()

  def run(self) -> None:
    sm = messaging.SubMaster(
      ["modelV2", "radarState", "liveCalibration", "liveTracks", "roadCameraState"],
      addr=self.ip_address,
    )
    vipc = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
    calibration = None
    img = np.zeros((480, 640, 3), dtype=np.uint8)

    while self.running:
      if not vipc.is_connected():
        vipc.connect(True)

      yuv = vipc.recv()
      if yuv is None or not yuv.data.any():
        time.sleep(0.01)
        continue

      sm.update(0)
      rgb = self.decode_nv12_to_rgb(yuv.data, vipc.width, vipc.height, vipc.stride)
      camera = DEVICE_CAMERAS[("tici", str(sm["roadCameraState"].sensor))]
      qcam = "QCAM" in os.environ
      bb_scale = (528 if qcam else camera.fcam.width) / 640.0
      warped = _warp_to_replay_view(rgb, bb_scale, (img.shape[1], img.shape[0]))
      np.copyto(img, warped)

      if sm.updated["liveCalibration"]:
        rpy_calib = np.asarray(sm["liveCalibration"].rpyCalib)
        calib_scale = camera.fcam.width / 640.0
        calibration = Calibration(rpy_calib, camera.fcam.intrinsics, calib_scale)

      if sm.recv_frame["modelV2"]:
        img_out = _overlay_model(img, sm["modelV2"], calibration)
      else:
        img_out = img

      with self.latest.lock:
        self.latest.frame = img_out.copy()


class ReplayUIVideoTrack(VideoStreamTrack):
  kind = "video"

  def __init__(self, renderer: ReplayUIRenderer):
    super().__init__()
    self.renderer = renderer

  async def recv(self) -> VideoFrame:
    pts, time_base = await self.next_timestamp()
    frame = None
    with self.renderer.latest.lock:
      if self.renderer.latest.frame is not None:
        frame = self.renderer.latest.frame.copy()
    if frame is None:
      frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    vf = VideoFrame.from_ndarray(frame, format="rgb24")
    vf.pts = pts
    vf.time_base = time_base
    return vf


class ReplayUIWebRTCServer:
  def __init__(self, ip_address: str):
    self.renderer = ReplayUIRenderer(ip_address)
    self.pcs: set[RTCPeerConnection] = set()
    self.log = logging.getLogger("ui_webrtc")

  async def index(self, _: web.Request) -> web.Response:
    content = VIEWER_FILE.read_text(encoding="utf-8")
    return web.Response(text=content, content_type="text/html")

  async def offer(self, request: web.Request) -> web.Response:
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    self.pcs.add(pc)
    self.log.info("Peer connected (%d active)", len(self.pcs))

    @pc.on("connectionstatechange")
    async def _on_state_change() -> None:
      if pc.connectionState in ("failed", "closed", "disconnected"):
        await pc.close()
        self.pcs.discard(pc)
        self.log.info("Peer removed (%d active)", len(self.pcs))

    pc.addTrack(ReplayUIVideoTrack(self.renderer))
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})

  async def on_shutdown(self, _: web.Application) -> None:
    self.renderer.stop()
    await asyncio.gather(*[pc.close() for pc in list(self.pcs)], return_exceptions=True)
    self.pcs.clear()

  def run(self) -> None:
    self.renderer.start()
    app = web.Application()
    app.router.add_get("/", self.index)
    app.router.add_post("/offer", self.offer)
    app.on_shutdown.append(self.on_shutdown)
    web.run_app(app, host=DEFAULT_HOST, port=DEFAULT_PORT)


def main() -> None:
  parser = argparse.ArgumentParser(description="Stream replay UI over WebRTC")
  parser.add_argument("ip_address", nargs="?", default="127.0.0.1", help="Address where replay publishes ZMQ services")
  args = parser.parse_args()

  logging.basicConfig(level=logging.INFO)
  if args.ip_address != "127.0.0.1":
    os.environ["ZMQ"] = "1"
    messaging.reset_context()
  ReplayUIWebRTCServer(args.ip_address).run()


if __name__ == "__main__":
  main()

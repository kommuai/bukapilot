#!/usr/bin/env python3

import argparse
import asyncio
import json
import uuid
import logging
from dataclasses import dataclass, field
from typing import Any, TYPE_CHECKING

# aiortc and its dependencies have lots of internal warnings :(
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=RuntimeWarning) # TODO: remove this when google-crc32c publish a python3.12 wheel

import capnp
from aiohttp import web
if TYPE_CHECKING:
  from aiortc.rtcdatachannel import RTCDataChannel

from openpilot.system.webrtc.schema import generate_field
from cereal import messaging, log


# =========================
# CORS MIDDLEWARE (NEW)
# =========================
@web.middleware
async def cors_middleware(request: web.Request, handler):
  if request.method == "OPTIONS":
    resp = web.Response(status=204)
  else:
    resp = await handler(request)

  resp.headers["Access-Control-Allow-Origin"] = "*"
  resp.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
  resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
  resp.headers["Access-Control-Max-Age"] = "86400"
  return resp


# =========================
# CEREAL BRIDGES
# =========================
class CerealOutgoingMessageProxy:
  def __init__(self, sm: messaging.SubMaster):
    self.sm = sm
    self.channels: list[RTCDataChannel] = []

  def add_channel(self, channel: 'RTCDataChannel'):
    self.channels.append(channel)

  def to_json(self, msg_content: Any):
    if isinstance(msg_content, capnp._DynamicStructReader):
      return msg_content.to_dict()
    elif isinstance(msg_content, capnp._DynamicListReader):
      return [self.to_json(msg) for msg in msg_content]
    elif isinstance(msg_content, bytes):
      return msg_content.decode()
    return msg_content

  def update(self):
    self.sm.update(0)
    for service, updated in self.sm.updated.items():
      if not updated:
        continue
      msg_dict = self.to_json(self.sm[service])
      mono_time = self.sm.logMonoTime[service]
      valid = self.sm.valid[service]
      outgoing_msg = {
        "type": service,
        "logMonoTime": mono_time,
        "valid": valid,
        "data": msg_dict,
      }
      encoded_msg = json.dumps(outgoing_msg).encode()
      for ch in self.channels:
        ch.send(encoded_msg)


class CerealIncomingMessageProxy:
  def __init__(self, pm: messaging.PubMaster):
    self.pm = pm

  def send(self, message: bytes):
    msg_json = json.loads(message)
    msg_type = msg_json["type"]
    msg_data = msg_json["data"]

    size = None
    if not isinstance(msg_data, dict):
      size = len(msg_data)

    msg = messaging.new_message(msg_type, size=size)
    setattr(msg, msg_type, msg_data)
    self.pm.send(msg_type, msg)


class CerealProxyRunner:
  def __init__(self, proxy: CerealOutgoingMessageProxy):
    self.proxy = proxy
    self.task: asyncio.Task | None = None
    self.logger = logging.getLogger("webrtcd")

  def start(self):
    if self.task is None:
      self.task = asyncio.create_task(self.run())

  def stop(self):
    if self.task:
      self.task.cancel()
      self.task = None

  async def run(self):
    from aiortc.exceptions import InvalidStateError

    while True:
      try:
        self.proxy.update()
      except InvalidStateError:
        self.logger.warning("Cereal outgoing proxy invalid state (connection closed)")
        break
      except Exception:
        self.logger.exception("Cereal outgoing proxy failure")
      await asyncio.sleep(0.01)


class DynamicPubMaster(messaging.PubMaster):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.lock = asyncio.Lock()

  async def add_services_if_needed(self, services):
    async with self.lock:
      for service in services:
        if service not in self.sock:
          self.sock[service] = messaging.pub_sock(service)


class StreamSession:
  shared_pub_master = DynamicPubMaster([])

  def __init__(self, sdp: str, cameras: list[str], incoming_services: list[str], outgoing_services: list[str], debug_mode: bool = False):
    from aiortc.mediastreams import VideoStreamTrack, AudioStreamTrack
    from aiortc.contrib.media import MediaBlackhole
    from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack
    from openpilot.system.webrtc.device.audio import AudioInputStreamTrack, AudioOutputSpeaker
    from teleoprtc import WebRTCAnswerBuilder
    from teleoprtc.info import parse_info_from_offer

    config = parse_info_from_offer(sdp)
    builder = WebRTCAnswerBuilder(sdp)

    assert len(cameras) == config.n_expected_camera_tracks, \
      "Incoming stream has misconfigured number of video tracks"

    for cam in cameras:
      track = LiveStreamVideoStreamTrack(cam) if not debug_mode else VideoStreamTrack()
      builder.add_video_stream(cam, track)

    if config.expected_audio_track:
      track = AudioInputStreamTrack() if not debug_mode else AudioStreamTrack()
      builder.add_audio_stream(track)

    if config.incoming_audio_track:
      self.audio_output_cls = AudioOutputSpeaker if not debug_mode else MediaBlackhole
      builder.offer_to_receive_audio_stream()

    self.stream = builder.stream()
    self.identifier = str(uuid.uuid4())

    self.incoming_bridge: CerealIncomingMessageProxy | None = None
    self.incoming_bridge_services = incoming_services
    self.outgoing_bridge: CerealOutgoingMessageProxy | None = None
    self.outgoing_bridge_runner: CerealProxyRunner | None = None
    if len(incoming_services) > 0:
      self.incoming_bridge = CerealIncomingMessageProxy(self.shared_pub_master)
    if len(outgoing_services) > 0:
      self.outgoing_bridge = CerealOutgoingMessageProxy(messaging.SubMaster(outgoing_services))
      self.outgoing_bridge_runner = CerealProxyRunner(self.outgoing_bridge)

    self.audio_output = None
    self.run_task: asyncio.Task | None = None
    self.logger = logging.getLogger("webrtcd")

    self.logger.info(
      "New stream session (%s), cameras=%s, in_audio=%s, out_audio=%s",
      self.identifier, cameras,
      config.incoming_audio_track,
      config.expected_audio_track,
    )

  async def get_answer(self):
    return await self.stream.start()

  def start(self):
    self.run_task = asyncio.create_task(self.run())

  async def message_handler(self, message: bytes):
    assert self.incoming_bridge is not None
    try:
      self.incoming_bridge.send(message)
    except Exception:
      self.logger.exception("Cereal incoming proxy failure")

  async def run(self):
    try:
      await self.stream.wait_for_connection()

      if self.stream.has_messaging_channel():
        if self.incoming_bridge is not None:
          await self.shared_pub_master.add_services_if_needed(self.incoming_bridge_services)
          self.stream.set_message_handler(self.message_handler)
        if self.outgoing_bridge_runner is not None:
          channel = self.stream.get_messaging_channel()
          self.outgoing_bridge_runner.proxy.add_channel(channel)
          self.outgoing_bridge_runner.start()

      if self.stream.has_incoming_audio_track():
        track = self.stream.get_incoming_audio_track(buffered=False)
        self.audio_output = self.audio_output_cls()
        self.audio_output.addTrack(track)
        self.audio_output.start()

      self.logger.info("Stream session (%s) connected", self.identifier)

      await self.stream.wait_for_disconnection()
    except Exception:
      self.logger.exception("Stream session failure")
    finally:
      await self.stream.stop()
      if self.outgoing_bridge_runner is not None:
        self.outgoing_bridge_runner.stop()
      if self.audio_output:
        self.audio_output.stop()
      self.logger.info("Stream session (%s) ended", self.identifier)

  async def post_run_cleanup(self):
    await self.stream.stop()
    if self.outgoing_bridge is not None:
      self.outgoing_bridge_runner.stop()
    if self.audio_output:
      self.audio_output.stop()


# =========================
# HTTP HANDLERS
# =========================
@dataclass
class StreamRequestBody:
  sdp: str
  cameras: list[str]
  bridge_services_in: list[str] = field(default_factory=list)
  bridge_services_out: list[str] = field(default_factory=list)


async def get_stream(request: web.Request):
  raw_body = await request.json()
  body = StreamRequestBody(**raw_body)

  session = StreamSession(
    body.sdp,
    body.cameras,
    body.bridge_services_in,
    body.bridge_services_out,
    request.app["debug"],
  )

  answer = await session.get_answer()
  session.start()

  request.app["streams"][session.identifier] = session
  return web.json_response({"sdp": answer.sdp, "type": answer.type})


async def get_schema(request: web.Request):
  services = [s for s in request.query.get("services", "").split(",") if s]
  assert all(
    s in log.Event.schema.fields and not s.endswith("DEPRECATED")
    for s in services
  ), "Invalid service name"

  schema_dict = {
    s: generate_field(log.Event.schema.fields[s])
    for s in services
  }
  return web.json_response(schema_dict)

async def post_notify(request: 'web.Request'):
  try:
    payload = await request.json()
  except Exception as e:
    raise web.HTTPBadRequest(text="Invalid JSON") from e

  for session in list(request.app.get('streams', {}).values()):
    try:
      ch = session.stream.get_messaging_channel()
      ch.send(json.dumps(payload))
    except Exception:
      continue

  return web.Response(status=200, text="OK")

async def on_shutdown(app: web.Application):
  for session in app["streams"].values():
    if session.run_task:
      session.run_task.cancel()
  app["streams"].clear()


# =========================
# MAIN
# =========================
def webrtcd_thread(host: str, port: int, debug: bool):
  logging.basicConfig(level=logging.INFO)
  if debug:
    logging.getLogger("webrtcd").setLevel(logging.DEBUG)

  app = web.Application(middlewares=[cors_middleware])
  app["streams"] = {}
  app["debug"] = debug

  app.on_shutdown.append(on_shutdown)
  app.router.add_post("/stream", get_stream)
  app.router.add_post("/notify", post_notify)
  app.router.add_get("/schema", get_schema)

  web.run_app(app, host=host, port=port)


def main():
  parser = argparse.ArgumentParser(description="WebRTC daemon")
  parser.add_argument("--host", default="0.0.0.0")
  parser.add_argument("--port", type=int, default=5001)
  parser.add_argument("--debug", action="store_true")
  args = parser.parse_args()

  webrtcd_thread(args.host, args.port, args.debug)


if __name__ == "__main__":
  main()


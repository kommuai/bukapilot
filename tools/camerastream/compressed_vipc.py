#!/usr/bin/env python3
import av
import os
import sys
import argparse
import numpy as np
import multiprocessing
import time
import signal


import cereal.messaging as messaging
from msgq.visionipc import VisionIpcServer, VisionStreamType

V4L2_BUF_FLAG_KEYFRAME = 8

# start encoderd
# also start cereal messaging bridge
# then run this "./compressed_vipc.py <ip>"

ENCODE_SOCKETS = {
  VisionStreamType.VISION_STREAM_ROAD: "roadEncodeData",
  VisionStreamType.VISION_STREAM_DRIVER: "driverEncodeData",
  VisionStreamType.VISION_STREAM_WIDE_ROAD: "wideRoadEncodeData",
}

def decoder(addr, vipc_server, vst, nvidia, W, H, debug=False):
  """
  HEVC → VIPC (NV12)
  Optimized version:
  - Uses PyAV software decode
  - Respects plane stride
  - Converts YUV420p → NV12
  - Minimal allocations, safe across resolutions
  """
  import os, time, av, numpy as np
  import cereal.messaging as messaging

  W, H = int(W), int(H)
  sock_name = ENCODE_SOCKETS[vst]

  def parse_nals(data: bytes):
    """Return True if an IDR/CRA NAL (19–21) exists in data."""
    i, n = 0, len(data)
    while i + 4 <= n:
      if data[i:i+3] == b"\x00\x00\x01":
        s = i + 3
      elif data[i:i+4] == b"\x00\x00\x00\x01":
        s = i + 4
      else:
        i += 1
        continue
      nal_type = (data[s] >> 1) & 0x3F
      if nal_type in (19, 20, 21):
        return True
      i = s + 1
    return False

  codec = av.CodecContext.create("hevc", "r")
  os.environ["ZMQ"] = "1"
  messaging.context = messaging.Context()
  sock = messaging.sub_sock(ENCODE_SOCKETS[vst], None, addr=addr, conflate=False)

  seen_iframe = False
  frame_id = 0

  while True:
    msgs = messaging.drain_sock(sock, wait_for_one=True)
    for evt in msgs:
      evta = getattr(evt, evt.which())
      pkt = evta.header + evta.data if len(evta.header) else evta.data

      # wait for IDR/CRA before decoding
      if not seen_iframe:
        if not parse_nals(pkt):
          continue
        seen_iframe = True

      try:
        frames = codec.decode(av.packet.Packet(pkt))
      except av.AVError:
        continue
      if not frames:
        continue

      f = frames[-1]
      if f.format.name != "yuv420p":
        f = f.reformat(format="yuv420p")

      y_p, u_p, v_p = f.planes
      fy, fx = int(f.height), int(f.width)

      # read planes using stride without extra padding copies
      y_bytes = np.frombuffer(y_p, dtype=np.uint8).reshape(y_p.height, y_p.line_size)
      u_bytes = np.frombuffer(u_p, dtype=np.uint8).reshape(u_p.height, u_p.line_size)
      v_bytes = np.frombuffer(v_p, dtype=np.uint8).reshape(v_p.height, v_p.line_size)

      # crop to visible area
      y_plane = y_bytes[:fy, :fx]
      u_plane = u_bytes[:fy // 2, :fx // 2]
      v_plane = v_bytes[:fy // 2, :fx // 2]

      # interleave chroma → NV12 layout
      uv = np.empty((fy // 2, fx), dtype=np.uint8)
      uv[:, 0::2] = u_plane
      uv[:, 1::2] = v_plane

      # combine planes (no copy until needed)
      yuv_flat = np.concatenate((y_plane.ravel(), uv.ravel()))
      if yuv_flat.size != (fx * fy * 3) // 2:
        continue

      sof_ns = int(evta.idx.timestampSof or time.monotonic() * 1e9)
      eof_ns = int(time.monotonic() * 1e9)
      vipc_server.send(vst, yuv_flat.data, frame_id, sof_ns, eof_ns)
      frame_id += 1


class CompressedVipc:
  def __init__(self, addr, vision_streams, server_name, nvidia=False, debug=False):
    print("getting frame sizes")
    os.environ["ZMQ"] = "1"
    messaging.reset_context()
    sm = messaging.SubMaster([ENCODE_SOCKETS[s] for s in vision_streams], addr=addr)
    while min(sm.recv_frame.values()) == 0:
      sm.update(100)
    os.environ.pop("ZMQ")
    messaging.reset_context()

    self.vipc_server = VisionIpcServer(server_name)
    for vst in vision_streams:
      ed = sm[ENCODE_SOCKETS[vst]]
      self.vipc_server.create_buffers(vst, 4, ed.width, ed.height)
    self.vipc_server.start_listener()

    self.procs = []
    for vst in vision_streams:
      ed = sm[ENCODE_SOCKETS[vst]]
      p = multiprocessing.Process(target=decoder, args=(addr, self.vipc_server, vst, nvidia, ed.width, ed.height, debug))
      p.start()
      self.procs.append(p)

  def join(self):
    for p in self.procs:
      p.join()

  def kill(self):
    for p in self.procs:
      p.terminate()
    self.join()

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Decode video streams and broadcast on VisionIPC")
  parser.add_argument("addr", help="Address of comma three")
  parser.add_argument("--nvidia", action="store_true", help="Use nvidia instead of ffmpeg")
  parser.add_argument("--cams", default="0,1,2", help="Cameras to decode")
  parser.add_argument("--server", default="camerad", help="choose vipc server name")
  parser.add_argument("--silent", action="store_true", help="Suppress debug output")
  args = parser.parse_args()

  vision_streams = [
    VisionStreamType.VISION_STREAM_ROAD,
    VisionStreamType.VISION_STREAM_DRIVER,
    VisionStreamType.VISION_STREAM_WIDE_ROAD,
  ]

  vsts = [vision_streams[int(x)] for x in args.cams.split(",")]
  cvipc = CompressedVipc(args.addr, vsts, args.server, args.nvidia, debug=(not args.silent))

  # register exit handler
  signal.signal(signal.SIGINT, lambda sig, frame: cvipc.kill())

  cvipc.join()

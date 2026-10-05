import re
from openpilot.system.hardware.hw import Paths

CHANNEL_VIDEO = 0x03

MEDIA_MOUNT = "/data/media"
REALDATA_ROOT = Paths.log_root()
CACHE_ROOT = "/data/media/0/kommu_video_cache"
# Separate from legacy mp4/ remux cache (HEVC-in-MP4); download output is always H.264 AVC.
MP4_CACHE_DIR = f"{CACHE_ROOT}/mp4_avc"
THUMB_CACHE_DIR = f"{CACHE_ROOT}/thumbs"

MAX_DRIVES = 10
MIN_SEGMENTS_PER_DRIVE = 2

# Video BLE link frames: keep 240 B default in ble_helper (hard limit for phone compatibility).
THUMB_SEND_BURST = 16

MP4_CONVERT_TIMEOUT_SEC = 120.0
VIDEO_HTTP_PORT = 8089
TRANSPORT_EXPIRES_SEC = 600
HOTSPOT_WAIT_SEC = 5.0
WIFI_FALLBACK_GRACE_SEC = 45.0
THUMB_FFMPEG_TIMEOUT_SEC = 10.0
# ka2 bench (8 fcamera.hevc, 5 cores): 0=401ms 1=378 2=375 4=373 6=378 8=387; one job at a time.
# Single-frame thumbs: SW decode + 4 threads (~430ms) beats remux+hevc_rkmpp (~490ms); keep SW here.
THUMB_FFMPEG_THREADS = 4
# App BLEService Watchcat RESET_TIMEOUT is 2000 ms; video page messageHz is 2.
VIDEO_KEEPALIVE_PERIOD_SEC = 0.5

# Raw HEVC segments from loggerd need explicit decoder + showall (see tools/lib/framereader.py).
FFMPEG_HEVC_INPUT_ARGS = ["-c:v", "hevc", "-vsync", "0", "-f", "hevc", "-flags2", "showall"]
FFMPEG_NO_SUBSTREAMS = ["-an", "-sn", "-dn"]

# Download MP4 output: H.264/AVC for Windows, Android, and iOS without extra HEVC codecs.
# RK3588 fast path: remux on tmpfs, hevc_rkmpp → h264_rkmpp MPP zero-copy (~5.1s on ka2).
# Fallback: SW hevc decode + h264_rkmpp (~30s), then libx264 veryfast CRF 23.
FFMPEG_H264_HW_ENCODE_ARGS = ["-c:v", "h264_rkmpp"]
FFMPEG_H264_ENCODE_ARGS = [
  "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
  "-pix_fmt", "yuv420p", "-threads", "0",
]

LIST_PAYLOAD_BUDGET = 55000

SEGMENT_RE = re.compile(r"^(\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2})--(\d+)$")

CAMERA_HEVC = {
  "road": "fcamera.hevc",
  "wide": "ecamera.hevc",
}

MSG = {
  "LIST_REQ": "videoListReq",
  "LIST_RESP": "videoListResp",
  "KEEPALIVE": "videoKeepalive",
  "DRIVE_OPEN": "videoDriveOpen",
  "DRIVE_CLOSE": "videoDriveClose",
  "THUMBNAIL": "videoThumbnail",
  "DOWNLOAD_REQ": "videoDownloadReq",
  "CONVERT_PROGRESS": "videoConvertProgress",
  "HOTSPOT_READY": "videoHotspotReady",
  "DOWNLOAD_TRANSPORT": "videoDownloadTransport",
  "DOWNLOAD_CANCEL": "videoDownloadCancel",
  "ERROR": "videoError",
}

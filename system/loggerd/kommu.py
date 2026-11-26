from openpilot.common.kommu import *

import requests

def fia_upload(base_fn, fn):
  headers = {
      "X-Fia-Class": "drive_log",
      "X-Fia-Filename": base_fn,
      "X-Fia-Tag": "v0",
  }

  resp = kapi(requests.get, WEB_BASE + "/fia/get_upload_url", headers=headers)
  if resp.status_code != 200:
    raise Exception("can't get upload url")

  with open(fn, "rb") as f:
    resp = kapi(requests.put, WEB_BASE + "/fia" + resp.text, data=f, headers=headers, timeout=10)
  return resp


def fia_upload_bytes(base_fn: str, data: bytes):
  """Upload raw bytes (e.g. bz2-compressed rlog/qlog) with the same key/URL flow as fia_upload."""
  headers = {
      "X-Fia-Class": "drive_log",
      "X-Fia-Filename": base_fn,
      "X-Fia-Tag": "v0",
  }

  resp = kapi(requests.get, WEB_BASE + "/fia/get_upload_url", headers=headers)
  if resp.status_code != 200:
    raise Exception("can't get upload url")

  resp = kapi(requests.put, WEB_BASE + "/fia" + resp.text, data=data, headers=headers, timeout=10)
  return resp

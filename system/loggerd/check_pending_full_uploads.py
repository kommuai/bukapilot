#!/usr/bin/env python3
"""
Test script to run ON THE DEVICE to verify GET /fia/pending_full_uploads.

Uses the same auth as the uploader (Bearer from RsjSession, X-Kaac-Id from DongleId).
Run from repo root or with PYTHONPATH set so openpilot imports work.

  python system/loggerd/check_pending_full_uploads.py

Prints the JSON response. If the server returns 401, the token may be missing or
invalid (re-login may be required). If the rule does not match, you get 404 from Oathkeeper.
"""

import json
import sys

import requests

from openpilot.common.params import Params
from openpilot.system.loggerd.kommu import WEB_BASE

def main():
  params = Params()
  token = params.get("RsjSession")
  dongle_id = params.get("DongleId")

  if not token:
    print("No RsjSession in Params. Device may need to log in first.", file=sys.stderr)
    sys.exit(1)

  url = WEB_BASE + "/fia/pending_full_uploads"
  headers = {"Authorization": "Bearer " + token}
  if dongle_id:
    headers["X-Kaac-Id"] = dongle_id

  try:
    resp = requests.get(url, headers=headers, timeout=10)
  except Exception as e:
    print(f"Request failed: {e}", file=sys.stderr)
    sys.exit(2)

  print(f"HTTP {resp.status_code}", file=sys.stderr)
  try:
    body = resp.json()
    print(json.dumps(body, indent=2))
  except Exception:
    print(resp.text)

  if resp.status_code == 401:
    print("Token missing or invalid. Re-login may be required.", file=sys.stderr)
    sys.exit(3)
  if resp.status_code == 404:
    print("404: path may not match Oathkeeper rule (check proxy config).", file=sys.stderr)
    sys.exit(4)

  sys.exit(0 if resp.status_code == 200 else 1)


if __name__ == "__main__":
  main()

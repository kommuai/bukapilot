#!/usr/bin/env python3

import argparse
import re
import subprocess
import sys


SMS_ID_RE = re.compile(r"/SMS/(\d+)")


def _run(cmd: list[str]) -> str:
  return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT)


def latest_sms_id(modem: int) -> int:
  out = _run(["mmcli", "-m", str(modem), "--messaging-list-sms"])
  ids = [int(m.group(1)) for m in SMS_ID_RE.finditer(out)]
  if not ids:
    raise RuntimeError("No SMS found (mmcli list empty)")
  return max(ids)


def sms_details(sms_id: int) -> str:
  return _run(["mmcli", "-s", str(sms_id)])


def parse_field(details: str, field: str) -> str | None:
  # Matches lines like: '             |      text: ...'
  m = re.search(rf"^\s*\|\s*{re.escape(field)}:\s*(.*)\s*$", details, flags=re.MULTILINE)
  return m.group(1) if m else None


def main() -> int:
  p = argparse.ArgumentParser(description="Print latest SMS from ModemManager (mmcli).")
  p.add_argument("--modem", type=int, default=0, help="Modem index (default: 0)")
  p.add_argument("--raw", action="store_true", help="Print full `mmcli -s <id>` output")
  args = p.parse_args()

  try:
    sid = latest_sms_id(args.modem)
    details = sms_details(sid)
  except subprocess.CalledProcessError as e:
    sys.stderr.write(e.output if isinstance(e.output, str) else str(e) + "\n")
    return 2
  except Exception as e:
    sys.stderr.write(f"{e}\n")
    return 2

  if args.raw:
    sys.stdout.write(details)
    return 0

  number = parse_field(details, "number") or ""
  text = parse_field(details, "text") or ""
  timestamp = parse_field(details, "timestamp") or ""

  print(f"sms_id: {sid}")
  if timestamp:
    print(f"timestamp: {timestamp}")
  if number:
    print(f"number: {number}")
  print(f"text: {text}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())


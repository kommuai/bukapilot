#!/usr/bin/env python3
import traceback

import cereal.messaging as messaging
from panda.python.uds import FUNCTIONAL_ADDRS
from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery
from selfdrive.swaglog import cloudlog

VIN_REQUEST = b'\x09\x02'
VIN_RESPONSE = b'\x49\x02\x01'
VIN_UNKNOWN = "0" * 17


def get_vin(logcan, sendcan, bus, timeout=0.1, debug=False):
  try:
    query = IsoTpParallelQuery(sendcan, logcan, bus, FUNCTIONAL_ADDRS, [VIN_REQUEST], [VIN_RESPONSE], functional_addr=True, debug=debug)
    for addr, vin in query.get_data(timeout).items():
      return addr[0], vin.decode()
  except Exception:
    cloudlog.warning(f"Clear code exception: {traceback.format_exc()}")

  return 0, VIN_UNKNOWN


if __name__ == "__main__":
  import time
  sendcan = messaging.pub_sock('sendcan')
  logcan = messaging.sub_sock('can')
  time.sleep(1)
  addr, vin = get_vin(logcan, sendcan, 0, debug=False)
  print(hex(addr), vin)

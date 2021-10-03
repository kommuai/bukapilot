#!/usr/bin/env python3
import traceback

import cereal.messaging as messaging
from panda.python.uds import FUNCTIONAL_ADDRS
from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery
from selfdrive.swaglog import cloudlog

DTC_REQUEST = b'\x03'
DTC_CLEAR_REQUEST = b'\x04'
DTC_RESPONSE = b'\x43'

def clear_dtc(logcan, sendcan, bus, timeout=1, retry=3, debug=False):
  for i in range(retry):
    try:
      dtc_query = IsoTpParallelQuery(sendcan, logcan, bus, FUNCTIONAL_ADDRS, [DTC_REQUEST], [DTC_RESPONSE], functional_addr=True, debug=debug)
      for item in dtc_query.get_data(timeout, expect_rx=True).items():
        print(hex(item))
      time.sleep(1)
      clear = IsoTpParallelQuery(sendcan, logcan, bus, FUNCTIONAL_ADDRS, [DTC_CLEAR_REQUEST], [None], functional_addr=True, debug=debug)
      clear.get_data(timeout, expect_rx=False).items()
      print(f"Clearing DTC attempt ({i+1}) ...")
    except Exception:
      cloudlog.warning(f"DTC Clear faield: {traceback.format_exc()}")

  return 0


if __name__ == "__main__":
  import time
  sendcan = messaging.pub_sock('sendcan')
  logcan = messaging.sub_sock('can')
  time.sleep(1)
  clear_dtc(logcan, sendcan, 1, debug=False)

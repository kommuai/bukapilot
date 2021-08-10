#!/usr/bin/env python3
import binascii
import sys
from collections import defaultdict
import numpy as np
import cereal.messaging as messaging
from common.realtime import sec_since_boot


def filter_static_bit(bus=0, wait_time = 10, f):
  """Collects messages and prints when a new bit transition is observed.
  This is very useful to find signals based on user triggered actions, such as blinkers and seatbelt.
  """

  static_msgs = ["handbrake"]
  dat = np.load(f,allow_pickle=True)
  dat = [dat.item()[i] for i in dat.item()]

  logcan = messaging.sub_sock('can')

  found = defaultdict(set)
  last_known = None

  for msg in static_msgs:
    addr = [i[1] for i in dat i[0] == msg]
    while 1:
      can_recv = messaging.drain_sock(logcan, wait_for_one=True)
      for x in can_recv:
        for y in x.can:
          if y.src == bus and y.address in addr:

            i = int.from_bytes(y.dat, byteorder='big')

            output_lh = i & (~p & 2**64-1)

            output_hl = p & (~i & 2**64-1)

            last_known = sec_since_boot()

            if started:
              found[last_known] = [static_msgs[msg_c-1], y.address, output_lh, output_hl]
              print(f"{sec_since_boot():.2f}\t{hex(y.address)} ({y.address})\t-{bin(output_hl)} +{bin(output_lh)}")


if __name__ == "__main__":
  print("Usage: Leave the script running until no new transitions are seen. This might take awhile.")
  print("Waiting for CAN Bus to get stable")

  if len(sys.argv) > 1:
    filter_static_bit(int(sys.argv[1]))
  else:
    filter_static_bit()

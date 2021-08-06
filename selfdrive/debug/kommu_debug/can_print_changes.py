#!/usr/bin/env python3
import binascii
import sys
from collections import defaultdict

import cereal.messaging as messaging
from common.realtime import sec_since_boot


def can_printer(bus=0):
  """Collects messages and prints when a new bit transition is observed.
  This is very useful to find signals based on user triggered actions, such as blinkers and seatbelt.
  """

  logcan = messaging.sub_sock('can')

  low_to_high = defaultdict(int)
  high_to_low = defaultdict(int)
  prev = defaultdict(int)

  while 1:
    can_recv = messaging.drain_sock(logcan, wait_for_one=True)
    for x in can_recv:
      for y in x.can:
        if y.src == bus:
          i = int.from_bytes(y.dat, byteorder='big')
          l_h = low_to_high[y.address]
          h_l = high_to_low[y.address]
          p = prev[y.address]

          if (i | l_h) != l_h:
            low_to_high[y.address] = i | l_h
            output = p & (~i & 2**64-1)
            print(f"{sec_since_boot():.2f}\t{hex(y.address)} ({y.address})\t+{bin(output)}")

          if (~i | h_l) != h_l:
            high_to_low[y.address] = ~i | h_l
            output = i & (~p & 2**64-1)
            print(f"{sec_since_boot():.2f}\t{hex(y.address)} ({y.address})\t-{bin(output)}")

          prev[y.address] = i

if __name__ == "__main__":
  print("Usage: Leave the script running until no new transitions are seen. Then perform the action.")

  if len(sys.argv) > 1:
    can_printer(int(sys.argv[1]))
  else:
    can_printer()

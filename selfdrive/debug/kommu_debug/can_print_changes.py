#!/usr/bin/env python3
import binascii
import sys
from collections import defaultdict

import cereal.messaging as messaging
from common.realtime import sec_since_boot


def can_printer(bus=0, wait_time = 10):
  """Collects messages and prints when a new bit transition is observed.
  This is very useful to find signals based on user triggered actions, such as blinkers and seatbelt.
  """

  static_msgs = ["handbrake","driver door","passenger door","back left door","back right door", \
      "seatbelt","left signal","right signal","brake","gas","generic toggle","steering"]
  msg_c = 0
  started = False

  logcan = messaging.sub_sock('can')

  low_to_high = defaultdict(int)
  high_to_low = defaultdict(int)
  prev = defaultdict(int)
  found = defaultdict(set)
  last_known = None

  while 1:
    can_recv = messaging.drain_sock(logcan, wait_for_one=True)
    for x in can_recv:
      for y in x.can:
        if y.src == bus:
          i = int.from_bytes(y.dat, byteorder='big')
          l_h = low_to_high[y.address]
          h_l = high_to_low[y.address]
          p = prev[y.address]

          changes = None
          if (i | l_h) != l_h:
            low_to_high[y.address] = i | l_h
            output_lh = p & (~i & 2**64-1)
            changes = True

          if (~i | h_l) != h_l:
            high_to_low[y.address] = ~i | h_l
            output_hl = i & (~p & 2**64-1)
            changes = True

          if changes:
            last_known = sec_since_boot()
            if started:
              found[last_known] = [static_msgs[msg_c-1], y.address, output_lh, output_hl]
              print(f"{sec_since_boot():.2f}\t{hex(y.address)} ({y.address})\t-{bin(output_hl)} +{bin(output_lh)}")

          # to make sure ignition on so there is the first message coming in
          if last_known:
            # time related logic
            if sec_since_boot() - last_known > wait_time:
              last_known = sec_since_boot()
              started = True
              print(f"Please engage then release the {static_msgs[msg_c]} within {wait_time}s.")
              msg_c += 1

          if msg_c == len(static_msgs):
            print(found)
            return

          prev[y.address] = i

if __name__ == "__main__":
  print("Usage: Leave the script running until no new transitions are seen. This might take awhile.")
  print("Waiting for CAN Bus to get stable")

  if len(sys.argv) > 1:
    can_printer(int(sys.argv[1]))
  else:
    can_printer()

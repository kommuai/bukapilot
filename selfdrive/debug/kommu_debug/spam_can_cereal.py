#!/usr/bin/env python3
import cereal.messaging as messaging
import random
import numpy as np
from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp
import selfdrive.boardd.boardd as boardd
import time

def spam_random_can_cereal():
  pm = messaging.PubMaster(['can'])
  counter = 0

  while 1:
    counter += 1
    if counter % 200 == 0:
      can_list = [[0x4f3, 0, (15).to_bytes(1,byteorder="big"), 0]]
    elif counter % 2 == 0:
      can_list = [[0x4f3, 0, (0).to_bytes(1,byteorder="big"), 0]]
    else:
      can_list = [[0x4f3, 0, (2).to_bytes(1,byteorder="big"), 0]]
    m = boardd.can_list_to_can_capnp(can_list, 'can')
    pm.send('can',m)
    time.sleep(0.1)

if __name__ == "__main__":
  spam_random_can_cereal()

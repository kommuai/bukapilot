#!/usr/bin/env python
import time
from panda import Panda

p = Panda()

while True:
  p.set_safety_mode(Panda.SAFETY_TOYOTA)
  p.send_heartbeat()
  print("ON")
  for i in range(50):
    p.send_heartbeat()
    time.sleep(0.1)
  p.set_safety_mode(Panda.SAFETY_NOOUTPUT)
  p.send_heartbeat()
  print("OFF")
  time.sleep(5)


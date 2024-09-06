#!/usr/bin/env python3
import cereal.messaging as messaging
from panda import Panda
from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp
from opendbc.can.packer import CANPacker

# Car interface related imports
from selfdrive.car.perodua.peroduacan import create_can_steer_command
from selfdrive.car.perodua.values import DBC

VEHICLE = "PERODUA MYVI PSD"

def signal_injection():
    
  p = Panda()
  # ensure it uses the safety filters of the vehicle
  # it will fall back to SAFETY_SILENT when this program is killed
  p.set_safety_mode(Panda.SAFETY_PERODUA)

  pm = messaging.PubMaster(['sendcan'])
  sm = messaging.SubMaster(['testJoystick'])
  packer = CANPacker(DBC[VEHICLE]['pt'])
  frame = 0
  gb = 0
  steer = 0
  
  while 1:
    can_sends = []
    sm.update()
    frame += 1
    
    # heartbeat must be sent to the STM32 to ensure connection is not lost
    p.send_heartbeat(True)

    if len(sm['testJoystick'].axes) != 0:
      gb = sm['testJoystick'].axes[0]
      steer = sm['testJoystick'].axes[1] * 255
      
      # since sm.update has a ratekeeper of 100Hz, perodua steer usually
      # is sent at 50hz
      if (frame % 2 == 0):
        can_sends.append(create_can_steer_command(packer, steer, 1, (frame/2) % 16))
    
    p.can_send_many(can_sends)
    #pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=True))


if __name__ == "__main__":
  signal_injection()

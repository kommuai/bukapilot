#!/usr/bin/env python
import os
import argparse
import threading
import socket
import sys
import struct

import cereal.messaging as messaging
from common.realtime import Ratekeeper
from common.numpy_fast import interp, clip
from common.params import Params
from tools.lib.kbhit import KBHit


class Joystick:
  def __init__(self, controller=None):
    if controller:
      self.cancel_button = 'BTN_NORTH'  # (BTN_NORTH=X, ABS_RZ=Right Trigger)
      accel_axis = 'ABS_Y'
      steer_axis = 'ABS_RX'
    else:
      self.cancel_button = 'BTN_TRIGGER'
      accel_axis = 'ABS_Y'
      steer_axis = 'ABS_RZ'
    self.min_axis_value = {accel_axis: 0., steer_axis: 0.}
    self.max_axis_value = {accel_axis: 255., steer_axis: 255.}
    self.axes_values = {accel_axis: 0., steer_axis: 0.}
    self.axes_order = [accel_axis, steer_axis]
    self.cancel = False

    self.controller_type = controller

  def update(self):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("192.168.100.1", 1234))
    # recvfrom is a blocking operation
    data, _ = s.recvfrom(4096)
    j_accel, j_steer, button_pressed = struct.unpack("BB?", data)

    self.axes_values['ABS_RX'] = -interp(j_accel, [self.min_axis_value['ABS_RX'], self.max_axis_value['ABS_RX']], [-1., 1.])
    self.axes_values['ABS_Y'] = interp(j_steer, [self.min_axis_value['ABS_Y'], self.max_axis_value['ABS_Y']], [-1., 1.])
    self.axes_values = {key: value if abs(value) > 0.110 else 0 for key, value in self.axes_values.items()} # center can be noisy, deadzone of 10%

    if button_pressed:
      self.cancel = True
    else:
      self.cancel = False
    return True


def send_thread(joystick):
  joystick_sock = messaging.pub_sock('testJoystick')
  rk = Ratekeeper(100, print_delay_threshold=None)
  while 1:
    dat = messaging.new_message('testJoystick')
    dat.testJoystick.axes = [joystick.axes_values[a] for a in joystick.axes_order]
    dat.testJoystick.buttons = [joystick.cancel]
    joystick_sock.send(dat.to_bytes())
    print('\n' + ', '.join(f'{name}: {round(v, 3)}' for name, v in joystick.axes_values.items()))
    rk.keep_time()

def joystick_thread(joystick):
  Params().put_bool('JoystickDebugMode', True)
  threading.Thread(target=send_thread, args=(joystick,), daemon=True).start()
  while True:
    joystick.update()

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Publishes events from your joystick to control your car.\n' +
                                               'openpilot must be offroad before starting joysticked.',
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  args = parser.parse_args()


  if not Params().get_bool("IsOffroad") and "ZMQ" not in os.environ and "WEB" not in os.environ:
    print("The car must be off before running joystickd.")
    exit()

  print()
  print('Using ESP UDP joystick, waiting for connection')

  joystick = Joystick(args)
  joystick_thread(joystick)

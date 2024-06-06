#!/usr/bin/env python3
import pickle
import socket
import json

from common.realtime import Ratekeeper
import cereal.messaging as messaging
from selfdrive.swaglog import cloudlog

class Streamer:
  def __init__(self, sm=None):
    # UDP sockets
    self.ip = '192.168.100.125'
    self.port = 5006
    self.sock = None

    # Setup subscriber
    self.sm = sm
    if self.sm is None:
      self.sm = messaging.SubMaster(['pandaStates'])

    # Sending data at 10hz
    self.rk = Ratekeeper(10, print_delay_threshold=None)

  def setup_udp_endpoint(self):
    try:
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.sock.bind((self.ip, self.port))
      print(f"UDP endpoint set up at {self.ip}:{self.port}")
    except socket.error as e:
      print(f"Failed to set up UDP endpoint: {e}")
      self.sock = None

  def is_connected(self):
    # Since UDP is connectionless, check if the socket is created and bound
    return self.sock is not None

  def update_and_publish(self):
    if self.is_connected():
      self.sm.update()
      for message in self.sm['pandaStates']:
        serialised = json.dumps(message)
        #serialised = pickle.dumps(message, protocol = pickle.DEFAULT_PROTOCOL)
      #  self.sock.sendto(serialised, ('192.168.100.59', self.port))

  def streamd_thread(self):
    while True:
      self.rk.monitor_time()
      self.update_and_publish()
      self.rk.keep_time()

def main():
  streamer = Streamer()

  # check for hotspot on, then setup the udp endpoint
  streamer.setup_udp_endpoint()
  streamer.streamd_thread()


if __name__ == "__main__":
  main()

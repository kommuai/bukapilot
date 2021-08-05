#!/usr/bin/env python3
from selfdrive.swaglog import cloudlog
from kommu.unlogger.logreader import LogReader
import cereal.messaging as messaging
import os
import sys
import numpy as np

# events which we are interested to unlog
INTERESTED = {'carState'}


def get_log_path(n):
  logs = sorted(os.listdir('/sdcard/realdata/'))
  logs = ['/sdcard/realdata/'+i for i in logs]
  f_logs = []

  for i in range(n):
    c = logs[-1-i]
    if 'rlog.bz2' in os.listdir(c):
      f_logs.append(c+'/rlog.bz2')

  return f_logs

def main():
  cloudlog.info("Unlogging file, might take a while...")

  # get the latest 10 logs.
  log_path = get_log_path(10)
  lr = LogReader(log_path)
  msgs = [m for m in lr if m.which() in INTERESTED]

  try:

    steering_angles = [msg.carState.steeringAngle for msg in msgs if msg.carState.steeringAngle < 4 and msg.carState.steeringAngle > -4]
    mean = np.mean(steering_angles)
    std = np.std(steering_angles)
    median = np.median(steering_angles)

    print("[+] Ten logs with steering angle of mean %.2f deg, std %.2f deg and median %.2f deg" % (mean,std,median))

  except RuntimeWarning:
    print("[-] Not enough logs, please drive for at least 10 minutes")


if __name__ == "__main__":
   main()

#!/usr/bin/env python3
# type: ignore
# steeringPressed threshold calibration tool
# procedure: 1. Let the steeringPressed threshold set to an adequate high value so no many false positive
#            2. Run this file and get the calibrated equation for dynamic steering thres
#            3. The calibrated multivariate steering equation will be directly saved into a file

import os
import argparse
import numpy as np
import cereal.messaging as messaging

vel_arr = np.array([], np.int8)
torque_arr = np.array([], np.int8)
angle_arr = np.array([], np.int8)

if __name__ == "__main__":

  parser = argparse.ArgumentParser(description='Sniff a communcation socket')
  parser.add_argument('--addr', default='127.0.0.1')
  args = parser.parse_args()

  if args.addr != "127.0.0.1":
    os.environ["ZMQ"] = "1"
    messaging.context = messaging.Context()

  carControl = messaging.sub_sock('carControl', addr=args.addr, conflate=True)
  sm = messaging.SubMaster(['carState', 'carControl', 'controlsState'], addr=args.addr)

  msg_cnt = 0
  while messaging.recv_one(carControl):
    sm.update()

    steering_angle = sm['carState'].steeringAngle
    actual_speed = sm['carState'].vEgo
    enabled = sm['controlsState'].enabled
    steer_override = sm['carState'].steeringPressed
    steering_torque = sm['carState'].steeringTorque
    toggle_enabled = sm['carState'].genericToggle

    # must be above 3 m/s, engaged and not overriding steering
    if actual_speed > 3.0 and enabled and not steer_override:
    #if steer_override:
      msg_cnt += 1

      # wait 5 seconds after engage/override
      if msg_cnt >= 500:
        if msg_cnt == 501:
          print("[+] Data collection started")
      vel_arr = np.append(vel_arr, actual_speed)
      torque_arr = np.append(torque_arr, steering_torque)
      angle_arr = np.append(angle_arr, steering_angle)

    if toggle_enabled:
      dat_to_save = np.vstack([vel_arr, angle_arr, torque_arr])
      print("Number of messages: " + str(msg_cnt))
      print("Saving data to /data/steering_calib_data.npy...")
      np.save("/data/steering_calib_dat", dat_to_save)
      print("Exiting...")
      exit(0)

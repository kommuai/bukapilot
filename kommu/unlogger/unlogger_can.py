#!/usr/bin/env python3
from selfdrive.swaglog import cloudlog
from logreader import LogReader
from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp
from panda import Panda
import cereal.messaging as messaging
from common.realtime import Ratekeeper
import os
import time
import sys

# events which we are interested to unlog
INTERESTED = {'can'}

def main(log_path):
    cloudlog.info('[+] unlogger_can started')
    cloudlog.info('[unlogger_can] Please wait a few seconds to process the rlog files...')

    lr = LogReader(log_path)
    msgs = [m for m in lr if m.which() in INTERESTED]
    msgs = sorted(msgs, key=lambda m: m.logMonoTime)
    times = [m.logMonoTime for m in msgs]

    lag = 0
    i = 0
    while True:

        # message builder
        msg = msgs[i].as_builder()
        next_msg = msgs[i+1]
        start_time = time.time()
        # unlog can
        if msg.which() == 'can':
            for c in msg.can:
                print(c.address, str(c.dat))

        lag += (next_msg.logMonoTime - msg.logMonoTime) / 1e9
        lag -= time.time() - start_time

        dt = max(lag,0)
        lag -= dt
        time.sleep(dt)

        if lag < -1.0 and i % 1000 == 0:
            print(f"{-lag:.2f} s behind")

        if i == len(msgs)-2:
            break
            cloudlog.info('[unlogger_can] Done! CAN Exhausted')

        i += 1


if __name__ == "__main__":
    print("Usage: python unlogger_can.py <path file/dir_of_logs>")
    main(sys.argv[1])

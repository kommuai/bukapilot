#!/usr/bin/env python3
import unittest
import time

from common.params import Params
import cereal.messaging as messaging
from common.basedir import BASEDIR
from common.timeout import Timeout, TimeoutException

params = Params()

# random calibration matrix to spoof
SPOOF_CALIB = b'\x00\x00\x00\x00\x16\x00\x00\x00\x00\x00\x00\x00\x02\x00\x01\x00T\xa3\x9a\x04!\x01\x00\x00\x12\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x06\x00\x01d\x00\x00\x00\x00\x00\x002\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x11\x00\x00\x00d\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x1d\x00\x00\x00\x1c\x00\x00\x00!\x00\x00\x00\x1c\x00\x00\x00\x88r\xa3<\xec\xf2\x7f\xbf\xb3W\x85:\x00\x00\x00\x00\xc9\x93P=\x00\x00\x00\x00\xfa\xaa\x7f\xbf\xf6(\x9c?\xea\x9d\x7f?\xe3\xa8\xa3<!\x89P=\x00\x00\x00\x00\t`\xb9\xb8\xe3\xaaP\xbd\xac\xab\xa3<\x00\x00\x00\x00\xe6\xbf\x1b:\xfc\xda\xd5;\xdf7=<\x00\x00\x00\x00'

def spoof_valid_startup(p):
  p.put("CalibrationParams", SPOOF_CALIB)
  p.put("HasAcceptedTerms","2")
  p.put("CompletedTrainingVersion","0.2.0")

def spoof_cleanup(p):
  p.put("CalibrationParams","")
  p.put("HasAcceptedTerms","")
  p.put("CompletedTrainingVersion","")

class QCTest(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    params.put("QC_Test", "1")
    spoof_valid_startup(params)

    # check for ignition to start the actual test
    sm = messaging.SubMaster(['pandaStates'])
    ignition = False
    while not ignition:
      sm.update(1000)
      ignition = sm['pandaStates'][0].ignitionLine or sm['pandaStates'][0].ignitionCan
      time.sleep(1)

  @classmethod
  def tearDownClass(cls):
    params.put("QC_Test", "")
    spoof_cleanup(params)

  # note: all methods must start with the name 'test'
  def test_confirm_pass_one_minute(self):
    time.sleep(60)
    self.assertEqual('foo'.upper(), 'FOO')

  def test_cpu_spike_surge(self):
    sm = messaging.SubMaster(['deviceState'])
    start = time.time()
    while True:
      sm.update(1000)
      now = time.time()
      if len(sm['deviceState'].gpuTempC) != 0:
        if int(sm['deviceState'].gpuTempC[0]) < 58 and (now - start) < 120:
          self.assertLess(sm['deviceState'].batteryCurrent, 2000000)
        else:
          self.assertTrue(True, True)
          break

if __name__ == "__main__":
  unittest.main()

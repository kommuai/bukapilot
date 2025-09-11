#!/usr/bin/env python3
import time
import random

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.honda.interface import CarInterface
from openpilot.selfdrive.controls.lib.events import ET, Events, EVENT_NAME
from openpilot.selfdrive.controls.lib.alertmanager import AlertManager
from openpilot.selfdrive.manager.process_config import managed_processes

EventName = car.CarEvent.EventName

def cycle_alerts(duration=200, is_metric=False):
  # this plays each type of audible alert
  '''
  alerts = [
    (EventName.buttonEnable, ET.ENABLE),
    (EventName.buttonCancel, ET.USER_DISABLE),
    (EventName.wrongGear, ET.NO_ENTRY),

    (EventName.locationdTemporaryError, ET.SOFT_DISABLE),
    (EventName.paramsdTemporaryError, ET.SOFT_DISABLE),
    (EventName.accFaulted, ET.IMMEDIATE_DISABLE),
    (EventName.preLaneChangeLeft, ET.WARNING),

    # DM sequence
    (EventName.preDriverDistracted, ET.WARNING),
    (EventName.promptDriverDistracted, ET.WARNING),
    (EventName.driverDistracted, ET.WARNING),
  ]
   '''

  alerts = [
    (EventName.startup, ET.PERMANENT),
    (EventName.wrongGear, ET.NO_ENTRY),
    (EventName.buttonEnable, ET.ENABLE),

    (EventName.steerSaturated, ET.WARNING),
    (None, None),
    (None, None),
    (EventName.buttonEnable, ET.ENABLE),
    (EventName.buttonEnable, ET.ENABLE),

    # DM sequence
    (EventName.preDriverDistracted, ET.WARNING),
    (EventName.promptDriverDistracted, ET.WARNING),
    (EventName.driverDistracted, ET.WARNING),
    (EventName.buttonCancel, ET.USER_DISABLE),


    (EventName.overheat, ET.PERMANENT),
    (EventName.overheat, ET.PERMANENT),
  ]
  '''
  # debug alerts
  alerts = [
    (EventName.highCpuUsage, ET.NO_ENTRY),
    (EventName.lowMemory, ET.PERMANENT),
    (EventName.overheat, ET.PERMANENT),
    (EventName.outOfSpace, ET.PERMANENT),
    (EventName.modeldLagging, ET.PERMANENT),
    (EventName.processNotRunning, ET.NO_ENTRY),
    (EventName.commIssue, ET.NO_ENTRY),
    (EventName.calibrationInvalid, ET.PERMANENT),
    (EventName.cameraMalfunction, ET.PERMANENT),
    (EventName.cameraFrameRate, ET.PERMANENT),
  ]
  '''

  CS = car.CarState.new_message()
  sm = messaging.SubMaster(['deviceState', 'pandaStates', 'roadCameraState', 'modelV2', 'liveCalibration',
                            'driverMonitoringState', 'longitudinalPlan', 'liveLocationKalman',
                            'managerState'])

  pm = messaging.PubMaster(['controlsState'])

  events = Events()
  AM = AlertManager()

  frame = 0

  enabled = False
  while True:
    for al, et in alerts:
      events.clear()
      events.add(al)

      if al != None:
        a = events.create_alerts([et, ], [None, CS, sm, is_metric, 0])
        AM.add_many(frame, a)
        alert = AM.process_alerts(frame, [])
      else:
        alert = None
      print(alert)
      for _ in range(duration):
        dat = messaging.new_message('controlsState')
        if al == EventName.buttonEnable:
          enabled = True
        if al == EventName.buttonCancel:
          enabled = False
        dat.controlsState.active = enabled

        if alert:
          dat.controlsState.alertText1 = alert.alert_text_1
          dat.controlsState.alertText2 = alert.alert_text_2
          dat.controlsState.alertSize = alert.alert_size
          dat.controlsState.alertStatus = alert.alert_status
          dat.controlsState.alertBlinkingRate = alert.alert_rate
          dat.controlsState.alertType = alert.alert_type
          dat.controlsState.alertSound = alert.audible_alert
        pm.send('controlsState', dat)

        frame += 1
        time.sleep(DT_CTRL)

if __name__ == '__main__':
  cycle_alerts()

#!/usr/bin/env python3
import time
import random

from cereal import car, log
import cereal.messaging as messaging
from opendbc.car.honda.interface import CarInterface
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.events import ET, EVENTS, Events
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.system.manager.process_config import managed_processes

EventName = log.OnroadEvent.EventName

def randperc() -> float:
  return 100. * random.random()

def cycle_alerts(duration=200, is_metric=False):
  # all alerts
  #alerts = list(EVENTS.keys())

  # Simulate alert events (only those that exist in EVENTS)
  alerts_raw = [
    (EventName.startup, ET.PERMANENT),
    (EventName.wrongGear, ET.NO_ENTRY),
    (EventName.buttonEnable, ET.ENABLE),
    (EventName.steerSaturated, ET.WARNING),
    (None, None),
    (None, None),
    (EventName.buttonEnable, ET.ENABLE),
    (EventName.buttonEnable, ET.ENABLE),
    # DM sequence (EVENTS has ET.PERMANENT for these)
    (EventName.preDriverDistracted, ET.PERMANENT),
    (EventName.promptDriverDistracted, ET.PERMANENT),
    (EventName.driverDistracted, ET.PERMANENT),
    (EventName.buttonCancel, ET.USER_DISABLE),
    (EventName.overheat, ET.PERMANENT),
    (EventName.overheat, ET.PERMANENT),
  ]
  alerts = [
    (ev, et) for ev, et in alerts_raw
    if ev is not None and ev in EVENTS and et in EVENTS.get(ev, {})
  ]

  cameras = ['roadCameraState', 'wideRoadCameraState', 'driverCameraState']

  CS = car.CarState.new_message()
  CP = CarInterface.get_non_essential_params("HONDA_CIVIC")
  sm = messaging.SubMaster(['deviceState', 'pandaStates', 'roadCameraState', 'modelV2', 'liveCalibration',
                            'driverMonitoringState', 'longitudinalPlan', 'livePose',
                            'managerState'] + cameras)

  pm = messaging.PubMaster(['selfdriveState', 'pandaStates', 'deviceState'])

  events = Events()
  AM = AlertManager()

  frame = 0
  while True:
    for alert, et in alerts:
      events.clear()
      events.add(alert)

      # Build fresh messages (SubMaster stores readers; we need builders to mutate)
      ds = messaging.new_message('deviceState')
      ds.deviceState.freeSpacePercent = randperc()
      ds.deviceState.memoryUsagePercent = int(randperc())
      ds.deviceState.cpuTempC = [randperc() for _ in range(3)]
      ds.deviceState.gpuTempC = [randperc() for _ in range(3)]
      ds.deviceState.cpuUsagePercent = [int(randperc()) for _ in range(8)]
      ds.deviceState.memoryTempC = randperc()
      sm.data['deviceState'] = ds.deviceState

      mv = messaging.new_message('modelV2')
      mv.modelV2.frameDropPerc = randperc()
      if random.random() > 0.25:
        mv.modelV2.velocity.x = [random.random(), ]
      sm.data['modelV2'] = mv.modelV2

      if random.random() > 0.25:
        CS.vEgo = random.random()

      procs = [p.get_process_state_msg() for p in managed_processes.values()]
      random.shuffle(procs)
      for i in range(min(random.randint(0, 10), len(procs))):
        procs[i].shouldBeRunning = True
      ms = messaging.new_message('managerState')
      ms.managerState.processes = procs
      sm.data['managerState'] = ms.managerState

      lc = messaging.new_message('liveCalibration')
      lc.liveCalibration.rpyCalib = [-1 * random.random() for _ in range(random.randint(0, 3))]
      sm.data['liveCalibration'] = lc.liveCalibration

      for s in sm.data.keys():
        prob = 0.3 if s in cameras else 0.08
        sm.alive[s] = random.random() > prob
        sm.valid[s] = random.random() > prob
        sm.freq_ok[s] = random.random() > prob

      a = events.create_alerts([et, ], [CP, CS, sm, is_metric, 0, log.LongitudinalPersonality.standard])
      AM.add_many(frame, a)
      AM.process_alerts(frame, [])
      alert = AM.current_alert
      print(alert)
      for _ in range(duration):
        dat = messaging.new_message('selfdriveState')
        dat.selfdriveState.enabled = False

        if alert:
          dat.selfdriveState.alertText1 = alert.alert_text_1
          dat.selfdriveState.alertText2 = alert.alert_text_2
          dat.selfdriveState.alertSize = alert.alert_size
          dat.selfdriveState.alertStatus = alert.alert_status
          dat.selfdriveState.alertType = alert.alert_type
          dat.selfdriveState.alertSound = alert.audible_alert
        pm.send('selfdriveState', dat)

        dat = messaging.new_message('deviceState')
        dat.deviceState.started = True
        pm.send('deviceState', dat)

        dat = messaging.new_message('pandaStates', 1)
        dat.pandaStates[0].ignitionLine = True
        dat.pandaStates[0].pandaType = log.PandaState.PandaType.uno
        pm.send('pandaStates', dat)

        frame += 1
        time.sleep(DT_CTRL)

if __name__ == '__main__':
  cycle_alerts()

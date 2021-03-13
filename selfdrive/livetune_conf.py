import json
import os

class livetune_conf():
  def __init__(self, CP=None):
    self.conf = self.read_config()
    if CP is not None:
      self.init_config(CP)

  def init_config(self, CP):
    write_conf = False
    if self.conf['tuneGernby'] != "1":
      self.conf['tuneGernby'] = str(1)
      write_conf = True

    # only fetch Kp, Ki, Kf sR and sRC from interface.py if it's a PID controlled car
    if CP.lateralTuning.which() == 'pid':
      if self.conf['Kp'] == "-1":
        self.conf['Kp'] = str(round(CP.lateralTuning.pid.kpV[0],3))
        write_conf = True
      if self.conf['Ki'] == "-1":
        self.conf['Ki'] = str(round(CP.lateralTuning.pid.kiV[0],3))
        write_conf = True
      if self.conf['Kf'] == "-1":
        self.conf['Kf'] = str('{:f}'.format(CP.lateralTuning.pid.kf))
        write_conf = True

    if self.conf['steerRatio'] == "-1":
      self.conf['steerRatio'] = str(round(CP.steerRatio,3))
      write_conf = True

    if self.conf['steerRateCost'] == "-1":
      self.conf['steerRateCost'] = str(round(CP.steerRateCost,3))
      write_conf = True

    if write_conf:
      self.write_config(self.config)

  def read_config(self):
    self.element_updated = False

    if os.path.isfile('/data/livetune.json'):
      with open('/data/livetune.json', 'r') as f:
        self.config = json.load(f)

      if "battPercOff" not in self.config:
        self.config.update({"battPercOff":"25"})
        self.config.update({"carVoltageMinEonShutdown":"11800"})
        self.config.update({"brakeStoppingTarget":"0.25"})
        self.element_updated = True

      if "tuneGernby" not in self.config:
        self.config.update({"tuneGernby":"1"})
        self.config.update({"Kp":"-1"})
        self.config.update({"Ki":"-1"})
        self.element_updated = True

      if "liveParams" not in self.config:
        self.config.update({"liveParams":"1"})
        self.element_updated = True

      if "steerRatio" not in self.config:
        self.config.update({"steerRatio":"-1"})
        self.config.update({"steerRateCost":"-1"})
        self.element_updated = True

      if "leadDistance" not in self.config:
        self.config.update({"leadDistance":"5"})
        self.element_updated = True

      if "deadzone" not in self.config:
        self.config.update({"deadzone":"0.0"})
        self.element_updated = True

      if "1barBP0" not in self.config:
        self.config.update({"1barBP0":"-0.1"})
        self.config.update({"1barBP1":"2.25"})
        self.config.update({"2barBP0":"-0.1"})
        self.config.update({"2barBP1":"2.5"})
        self.config.update({"3barBP0":"0.0"})
        self.config.update({"3barBP1":"3.0"})
        self.element_updated = True


      if "1barMax" not in self.config:
        self.config.update({"1barMax":"2.1"})
        self.config.update({"2barMax":"2.1"})
        self.config.update({"3barMax":"2.1"})
        self.element_updated = True

      if "1barHwy" not in self.config:
        self.config.update({"1barHwy":"0.4"})
        self.config.update({"2barHwy":"0.3"})
        self.config.update({"3barHwy":"0.1"})
        self.element_updated = True

      if "slowOnCurves" not in self.config:
        self.config.update({"slowOnCurves":"0"})
        self.element_updated = True

      if "Kf" not in self.config:
        self.config.update({"Kf":"-1"})
        self.element_updated = True

      if "sR_boost" not in self.config:
        self.config.update({"sR_boost":"0"})
        self.config.update({"sR_BP0":"0"})
        self.config.update({"sR_BP1":"0"})
        self.config.update({"sR_time":"1"})
        self.element_updated = True

      if "ALCnudgeLess" not in self.config:
        self.config.update({"ALCnudgeLess":"0"})
        self.config.update({"ALCminSpeed":"20.1168"})
        self.element_updated = True

      if "ALCtimer" not in self.config:
        self.config.update({"ALCtimer":"2.0"})
        self.element_updated = True

      if "CruiseDelta" not in self.config:
        self.config.update({"CruiseDelta":"8"})
        self.element_updated = True

      if "CruiseEnableMin" not in self.config:
        self.config.update({"CruiseEnableMin":"40"})
        self.element_updated = True

      if "epsModded" not in self.config:
        self.config.update({"epsModded":"0"})
        self.element_updated = True

      if "accelerationMode" not in self.config:
        self.config.update({"accelerationMode":"1"})
        self.element_updated = True

      if "AutoHold" not in self.config:
        self.config.update({"AutoHold":"0"})
        self.element_updated = True
     
      if "fakeEngage" not in self.config:
        self.config.update({"fakeEngage": "0"})
        self.element_updated = True

      if "maxSteer" not in self.config:
        self.config.update({"maxSteer":"0"})
        self.config.update({"steerDeltaUp":"0"})
        self.config.update({"steerDeltaDown":"0"})
        self.config.update({"steerDriverAllowance":"0"})
        self.config.update({"steerDriverMult":"0"})

      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = {"cameraOffset":"0.06", "lastTrMode":"1", "battChargeMin":"60", "battChargeMax":"70", \
                     "wheelTouchSeconds":"180", "accelerationMode":"1","battPercOff":"25", "carVoltageMinEonShutdown":"11800", \
                     "brakeStoppingTarget":"0.25", "tuneGernby":"1", "AutoHold":"0",\
                     "Kp":"-1", "Ki":"-1", "liveParams":"1", "leadDistance":"5", "deadzone":"0.0", \
                     "1barBP0":"-0.1", "1barBP1":"2.25", "2barBP0":"-0.1", "2barBP1":"2.5", "3barBP0":"0.0", \
                     "3barBP1":"3.0", "1barMax":"2.1", "2barMax":"2.1", "3barMax":"2.1", \
                     "1barHwy":"0.4", "2barHwy":"0.3", "3barHwy":"0.1", \
                     "steerRatio":"-1", "steerRateCost":"-1", "slowOnCurves":"0", "Kf":"-1", \
                     "sR_boost":"0", "sR_BP0":"0", "sR_BP1":"0", "sR_time":"1", \
                     "ALCnudgeLess":"0", "ALCminSpeed":"20.1168", "ALCtimer":"2.0", "CruiseDelta":"8", \
                     "CruiseEnableMin":"40", "epsModded": "0", "steerMax": "0", "steerDeltaUp" : "0", \
                     "steerDeltaDown" : "0", "steerDriverAllowance" : "0", "steerDriverMult" : "0", "fakeEngage" : "0"}


      self.write_config(self.config)
    return self.config

  def write_config(self, config):
    try:
      with open('/data/livetune.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/livetune.json", 0o764)
    except IOError:
      os.mkdir('/data')
      with open('/data/livetune.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/livetune.json", 0o764)

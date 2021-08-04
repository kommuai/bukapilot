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

    if os.path.isfile('/data/openpilot/kommu/livetuner/livetune.json'):
      with open('/data/openpilot/kommu/livetuner/livetune.json', 'r') as f:
        self.config = json.load(f)

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

      if "Kf" not in self.config:
        self.config.update({"Kf":"-1"})
        self.element_updated = True

      if "sR_boost" not in self.config:
        self.config.update({"sR_boost":"0"})
        self.config.update({"sR_BP0":"0"})
        self.config.update({"sR_BP1":"0"})
        self.config.update({"sR_time":"1"})
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

      if "maxGas" not in self.config:
        self.config.update({"maxGas":"0"})

      if "cruiseSetSpeed" not in self.config:
        self.config.update({"cruiseSetSpeed":"20"})

      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = {"cameraOffset":"0.06", "Kp":"-1", "Ki":"-1", "leadDistance":"5", "deadzone":"0.0", \
                     "1barBP0":"-0.1", "1barBP1":"2.25", "2barBP0":"-0.1", "2barBP1":"2.5", "3barBP0":"0.0", \
                     "3barBP1":"3.0", "1barMax":"2.1", "2barMax":"2.1", "3barMax":"2.1", \
                     "1barHwy":"0.4", "2barHwy":"0.3", "3barHwy":"0.1", \
                     "steerRatio":"-1", "steerRateCost":"-1", "Kf":"-1", \
                     "sR_boost":"0", "sR_BP0":"0", "sR_BP1":"0", "sR_time":"1", \
                     "steerMax": "0", "steerDeltaUp" : "0", \
                     "steerDeltaDown" : "0", "steerDriverAllowance" : "0", "steerDriverMult" : "0", "fakeEngage" : "0", \
                     "maxGas" : "0", "cruiseSetSpeed" : "20"}


      self.write_config(self.config)
    return self.config

  def write_config(self, config):
    try:
      with open('/data/openpilot/kommu/livetuner/livetune.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/openpilot/kommu/livetuner/livetune.json", 0o764)
    except IOError:
      os.mkdir('/data')
      with open('/data/openpilot/kommu/livetuner/livetune.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/openpilot/kommu/livetuner/livetune.json", 0o764)

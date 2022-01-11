from selfdrive.hardware import EON

import json
import os

KRATOS_BASE="http://192.168.0.10:4433"
KRATOS_ADMIN="http://192.168.0.10:4434"


class Karams:
  def __init__(self):

    self.path = None
    if EON:
      self.path = "/data/openpilot/kommu.karams"
      if not os.path.exists(self.path):
        with open(self.path, "w") as f:
          f.write("{}")

    with open(self.path) as f:
      self.data = json.load(f)

  def get(self, key):
    return self.data[key]

  def put(self, key, val):
    self.data[key] = val
    with open(self.path, "w") as f:
      json.dump(self.data, f)


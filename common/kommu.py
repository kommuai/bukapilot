from selfdrive.hardware import EON, HARDWARE

import requests

import json
import os

WEB_BASE="https://web.kommu.ai"

class AuthException(Exception):
    pass

class Karams:
  def __init__(self):
    if EON:
      self.path = "/data/kommu.karams"
    else:
      self.path = "kommu.karams"
    if not os.path.exists(self.path):
      with open(self.path, "w") as f:
        f.write("{}")
    with open(self.path) as f:
      self.data = json.load(f)

  def get(self, key, soft=True):
    if soft:
      try:
        ret = self.data[key]
        return ret
      except KeyError:
        return ""
    return self.data[key]

  def put(self, key, val):
    self.data[key] = val
    with open(self.path, "w") as f:
      json.dump(self.data, f)


def refresh_session():
  k = Karams()

  init = requests.get(WEB_BASE + "/self-service/login/api")
  if init.status_code != 200:
    raise Exception("can't init kratos login flow")

  data = {
      "method": "password",
      "password_identifier": k.get("rsj_dongle"),
      "password": HARDWARE.get_imei(1) + HARDWARE.get_serial(),
  }
  resp = requests.post(init.json()["ui"]["action"], data=data)
  if resp.status_code != 200:
    raise AuthException("can't login into system")

  k.put("rsj_session", resp.json()["session_token"])


def _kapi_raw(func, *args, **kwargs):
  k = Karams()
  auth = k.get("rsj_session")

  if "headers" in kwargs:
    headers = kwargs["headers"]
  else:
    headers = {}

  headers["Authorization"] = "Bearer " + auth
  kwargs["headers"] = headers
  return func(*args, **kwargs)


def kapi(func, *args, **kwargs):
  resp = _kapi_raw(func, *args, **kwargs)
  if resp.status_code == 401:
    # one try only!
    refresh_session()
    return _kapi_raw(func, *args, **kwargs)
  return resp


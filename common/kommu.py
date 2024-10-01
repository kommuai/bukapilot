from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE

import requests
import json
import os

WEB_BASE="https://web.kommu.ai"

class AuthException(Exception):
    pass

def refresh_session():
  params = Params()

  init = requests.get(WEB_BASE + "/self-service/login/api")
  if init.status_code != 200:
    raise Exception("can't init kratos login flow")

  data = {
      "method": "password",
      "password_identifier": params.get("DongleId"),
      "password": HARDWARE.get_imei(1) + HARDWARE.get_serial(),
  }
  resp = requests.post(init.json()["ui"]["action"], data=data)
  if resp.status_code != 200:
    raise AuthException("can't login into system")

  params.put("RsjSession", resp.json()["session_token"])


def _kapi_raw(func, *args, **kwargs):
  params = Params()
  auth = params.get("RsjSession", encoding="utf-8")

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

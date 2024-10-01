from openpilot.common.kommu import AuthException, WEB_BASE, kapi
from openpilot.common.params import Params

from Crypto import Random
from Crypto.Hash import SHA224
import requests

from base64 import b64encode

import json

def register_user(imei, serial):
  params = Params()

  # make sure the algo is the same with rsj!
  dongle_id = SHA224.new(data=(imei + serial).encode()).hexdigest()[:16]
  params.put("DongleId", dongle_id)

  # first, try to login, WE MUST BE ABLE TO LOGIN IF DONGLE EXISTS
  # TODO: review threat model
  auth = True
  try:
    resp = kapi(requests.get, WEB_BASE + "/sessions/whoami")
  except AuthException:
    auth = False
  if auth:
    return dongle_id

  params.put("DongleId", "")
  init = requests.get(WEB_BASE + "/self-service/registration/api")
  if init.status_code != 200:
    raise Exception("can't init kratos flow")

  data = {
      "method": "password",
      "traits.username": dongle_id,
      "traits.email": f"dongle_{dongle_id}@kommu.ai",
      "password": imei + serial,
  }

  resp = requests.post(init.json()["ui"]["action"], data=data)
  # when 400, assume previously incomplete registration and try to update schema
  if resp.status_code not in (200, 400):
    return None

  rr = resp.json()
  if resp.status_code == 200:
    params.put("RsjSession", rr["session_token"])

  data = {
    "imei": imei,
    "serial": serial,
  }
  resp = kapi(requests.post, f"{WEB_BASE}/rsj/register", data=data)
  if resp.status_code != 200:
    return None

  assert resp.text == dongle_id
  return resp.text

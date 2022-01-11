from common.kommu import Karams, KRATOS_BASE, KRATOS_ADMIN

from Crypto import Random
from Crypto.Hash import SHA224
import requests

from base64 import b64encode

import json


def register_user(imei, serial):
  init = requests.get(KRATOS_BASE + "/self-service/registration/api")
  if init.status_code != 200:
    raise Exception("can't init kratos flow")

  di_raw = SHA224.new(data=(imei + serial).encode()).hexdigest()
  dongle_id = di_raw[:16]

  data = {
      "method": "password",
      "traits.username": dongle_id,
      "traits.email": f"dongle_{dongle_id}@kommu.ai",
      "password": b64encode(Random.get_random_bytes(12)).decode(),
  }

  resp = requests.post(init.json()["ui"]["action"], data=data)
  # when 400, assume exists and try to update schema
  if resp.status_code not in (200, 400):
    return None

  karams = Karams()
  rr = resp.json()
  if resp.status_code == 200:
    ident = rr["identity"]["id"]
    karams.put("rsj_password", data["password"])
    karams.put("rsj_ident", ident)
    karams.put("rsj_session", rr["session_token"])
  else:
    ident = karams.get("rsj_ident")

  data = {
      "schema_id": "dongle",
      "state": "active",
      "traits": {
        "dongle_id": dongle_id,
      },
  }

  resp = requests.put(f"{KRATOS_ADMIN}/identities/{ident}", json=data)
  if resp.status_code != 200:
    return None

  return dongle_id


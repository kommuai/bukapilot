#!/usr/bin/env python3
import datetime
import json
import os
import signal
import subprocess
import sys
import traceback
from multiprocessing import Process
from typing import List, Tuple, Union

import cereal.messaging as messaging
import selfdrive.sentry as sentry
from common.basedir import BASEDIR
from common.features import Features
from common.params import Params, ParamKeyType
from common.text_window import TextWindow
from selfdrive.hardware import HARDWARE, PC
from selfdrive.manager.helpers import unblock_stdout
from selfdrive.manager.process import ensure_running
from selfdrive.manager.process_config import managed_processes
from selfdrive.athena.registration import register, UNREGISTERED_DONGLE_ID
from selfdrive.swaglog import cloudlog, add_file_handler
from selfdrive.version import is_dirty, get_commit, get_version, get_origin, get_short_branch, \
                              terms_version, training_version


sys.path.append(os.path.join(BASEDIR, "pyextra"))

MIN_DATE = datetime.datetime(year=2022, month=4, day=1)

done_bootlog = False

def try_bootlog(can_block = False) -> bool:

  sys_time = datetime.datetime.today()
  time_valid = sys_time > MIN_DATE

  if not time_valid and not can_block:
    return False

  if not time_valid:
    # set time from gps
    os.system(BASEDIR + "/selfdrive/sensord/set_time")

  # save boot log
  subprocess.call("./bootlog", cwd=os.path.join(BASEDIR, "selfdrive/loggerd"))
  return True

def manager_init() -> None:
  global done_bootlog
  done_bootlog = try_bootlog(False)

  if done_bootlog:
    cloudlog.info("System time valid")

  params = Params()
  params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)

  default_params: List[Tuple[str, Union[str, bytes]]] = [
    ("CompletedTrainingVersion", "0"),
    ("HasAcceptedTerms", "0"),
    ("OpenpilotEnabledToggle", "1"),
    ("IsMetric", "1"),
    ("IsLdwEnabled", "1"),
    ("IsRHD", "1"),
    ("RecordFront", "1"),
    ("RsjSession", "invalidsession"),
    ("FanPwmOverride", "70.0"),
    ("PowerSaverEntryDuration", "360.0"),
    ("StoppingDistanceOffset", "0.0"),
    ("DrivePathOffset", "0.0"),
  ]
  if not PC:
    default_params.append(("LastUpdateTime", datetime.datetime.utcnow().isoformat().encode('utf8')))

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  if not params.get_bool("DisableRadar_Allow"):
    params.delete("DisableRadar")

  # set unset params
  for k, v in default_params:
    if params.get(k) is None:
      params.put(k, v)

  # is this dashcam?
  if os.getenv("PASSIVE") is not None:
    params.put_bool("Passive", bool(int(os.getenv("PASSIVE", "0"))))

  if params.get("Passive") is None:
    raise Exception("Passive must be set to continue")

  # Create folders needed for msgq
  try:
    os.mkdir("/dev/shm")
  except FileExistsError:
    pass
  except PermissionError:
    print("WARNING: failed to make /dev/shm")

  # set release notes
  with open(BASEDIR + "/RELEASES.md", "rb") as f:
    r = f.read().split(b'\n\n', 1)[0]  # Slice latest release notes
    params.put("ReleaseNotes", r.decode("utf-8"))

  # set version params
  params.put("Version", get_version())
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", get_commit(default=""))
  params.put("GitBranch", get_short_branch(default=""))
  params.put("GitRemote", get_origin(default=""))

  # set default feature dict
  new_dict = {
      "features": {
        "MyviAzri"           :    1 << 0,
        "MyviKevin"          :    1 << 1,
        "ClearCode"          :    1 << 2,
        "StockAcc"           :    1 << 3,
        "IgnoreHardIgnition" :    1 << 4,
        "IgnoreDM" :              1 << 5,
        },
      "packages": {
        "default": [],
        "myvi-a": ["MyviAzri"],
        "myvi-b": ["MyviKevin"],
        "clear-code": ["ClearCode"],
        "stock-acc" : ["StockAcc"],
        "ignore-ignition-line" : ["IgnoreHardIgnition"],
        "ignore-dm" : ["IgnoreDM"]
        },
      "version": 4,
      }
  cur_dict = params.get("FeaturesDict")
  if cur_dict is None or (json.loads(cur_dict)["version"] < new_dict["version"]):
    params.put("FeaturesDict", json.dumps(new_dict))
    Features().set_package("default")

  if Features().has("StockAcc"):
    params.put("StockAccToggle_Allow", b'1')

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    serial = params.get("HardwareSerial")
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog

  if not is_dirty():
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id, version=get_version(), dirty=is_dirty(),
                       device=HARDWARE.get_device_type())


def manager_prepare() -> None:
  for p in managed_processes.values():
    p.prepare()


def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: List[str] = []
  if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  ensure_running(managed_processes.values(), started=False, not_run=ignore)

  started_prev = False
  sm = messaging.SubMaster(['deviceState'])
  pm = messaging.PubMaster(['managerState'])

  while True:
    sm.update()
    not_run = ignore[:]

    started = sm['deviceState'].started
    driverview = params.get_bool("IsDriverViewEnabled")
    ensure_running(managed_processes.values(), started, driverview, not_run)

    # trigger an update after going offroad
    if started_prev and not started and 'updated' in managed_processes:
      os.sync()
      managed_processes['updated'].signal(signal.SIGHUP)

    started_prev = started

    running = ' '.join("%s%s\u001b[0m" % ("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState')
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", param)
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break


def main() -> None:
  prepare_only = os.getenv("PREPAREONLY") is not None

  manager_init()

  # Start UI early so prepare can happen in the background
  if not prepare_only:
    managed_processes['ui'].start()

  # flip screen, set to 1 first to trigger rotation
  os.system("settings put system user_rotation 1")
  os.system("settings put system user_rotation 3")

  global done_bootlog
  if not done_bootlog:
    sys_time = datetime.datetime.today()
    p = Process(target=try_bootlog, args=(True,))
    p.start()
    cur_time = datetime.datetime.today()
    cloudlog.info(f"adjusting time from '{sys_time}' to '{cur_time}'")

  manager_prepare()

  if prepare_only:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)

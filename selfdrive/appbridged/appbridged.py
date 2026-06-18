#!/usr/bin/env python3
import socket
import msgpack
import subprocess
import psutil
import threading
import re
import math
from time import monotonic
import datetime
import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_version, get_commit, terms_version, training_version
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from opendbc.car.car_helpers import supported_cars
from openpilot.common.features import Features
from openpilot.selfdrive.appbridged.ble_helper import BLEBridge, ChunkReceiver
from openpilot.selfdrive.appbridged.hardware_helper import HardwareHelper

# BLE Constants
MESSAGE_HZ = 16 # Expected message rate, must match app visualisation value
params = Params()
DONGLE_ID = params.get("DongleId") or ""

# BLE Channel IDs
CHANNEL_VISUALISATION = 0x01
CHANNEL_SETTINGS = 0x02

# Wi-Fi/nmcli Constants
WIFI_CONNECT_TIMEOUT_SECONDS = 20 # Timeout for device Wi-Fi connection attempts
NO_NETWORK_REGEX = re.compile(r"no network.*ssid", re.IGNORECASE)
WIFI_SCAN_SIGNAL_THRESHOLD = 31 # Minimum signal strength required for Wi-Fi scan result

# Device Constants
UPDATE_PROCESS = "system.updated.updated"
HOTSPOT_SERVICE = "wlan1-setup.service"
SM_UPDATE_INTERVAL = 33 # in ms, the interval where capnp submaster updates
features = Features()

# Call functions with cached values only once
SUPPORTED_CARS = supported_cars()
GIT_COMMIT = get_commit()[:7]
CUR_VERSION = get_version()
OS_VERSION = HARDWARE.get_os_version()

def forget_wifi_network(ssid):
  if not ssid:
    return False
  threading.Thread(daemon=True, target=lambda: subprocess.run(["sudo", "nmcli", "con", "delete", ssid], text=True)).start()
  return True

def send_update_signal(action="check"):
  subprocess.Popen(["pkill", f"-{'SIGHUP' if action == 'fetch' else 'SIGUSR1'}", "-f", UPDATE_PROCESS])

def change_branch_and_update(target_branch):
  params.put("UpdaterTargetBranch", target_branch)
  send_update_signal("check")

def resample(data, target=None):
  """Resamples data by a fraction of its original length."""
  # original op list length 33, target 4 to 33 for upsampling in app
  m = target or 8
  if (t := type(data)) is list and (n := len(data)) > 1 and m > 1:
    return [data[0]] + [data[int(i*(n-1)/m)] for i in range(1, m)]
  if t is dict and all(k in data for k in 'xyz'):
    return {k: resample(data[k]) for k in 'xyz'}
  return data

def extract_model_data(d):
  data = {'f': d['frameId']}
  if pos := d.get('position'):
    data['p'] = resample(pos)
  data['a'] = resample(d.get('acceleration', {}).get('x'), 12)
  for k, p, v in (
    ('laneLine', 'l', 1),
    ('roadEdge', 'r', 1),
    ('laneLineProb', 'p', 0),
    ('roadEdgeStd', 's', 0)
  ):
    for i, item in enumerate(d.get(f"{k}s", []), 1):
      data[f"{p}{i}"] = resample(item) if v else item
  return data

def safe_get(key, is_bool=False):
  """Safely retrieve a parameter value."""
  try:
    if is_bool:
      return params.get_bool(key)
    return (v.isoformat() if isinstance((v := params.get(key)), datetime.datetime) else str(v or ""))
  except Exception:
    return False if is_bool else ""

def safe_put_all(settings_to_put, is_bool=False):
  """Safely store multiple parameters."""
  for param_key, value in settings_to_put.items():
    try:
      (params.put_bool_nonblocking if is_bool else params.put_nonblocking)(
        param_key, value if is_bool else str(value).strip())
    except Exception as e:
      cloudlog.error(f"Error putting {param_key}: {e}")

def reset_calibration(state):
  if state == log.SelfdriveState.OpenpilotState.disabled:
    # Parameters will change depending on openpilot version. (Currently follow 0.10)
    # Keep above comment for future reference, do not delete comment.
    params.remove("CalibrationParams")
    params.remove("LiveTorqueParameters")
    params.remove("LiveParameters")
    params.remove("LiveParametersV2")
    params.remove("LiveDelay")
    params.put_bool_nonblocking("OnroadCycleRequested", True)

def do_reboot(state):
  if state == log.SelfdriveState.OpenpilotState.disabled:
    params.put_bool_nonblocking("DoReboot", True)

def _systemctl(action, service=HOTSPOT_SERVICE):
  try:
    subprocess.run(["sudo", "systemctl", action, service], check=True)
    cloudlog.info(f"systemctl {action} {service} succeeded")
  except Exception as e:
    cloudlog.error(f"systemctl {action} {service} failed: {e}")

def enable_hotspot():
  def worker():
    _systemctl("start", HOTSPOT_SERVICE)
  threading.Thread(target=worker, daemon=True).start()

def disable_hotspot():
  def worker():
    subprocess.run(["sudo", "ip", "link", "set", "wlan1", "down"], check=False)
    _systemctl("stop", HOTSPOT_SERVICE)
  threading.Thread(target=worker, daemon=True).start()

def update_dict_from_sm(target_dict, sm_subset, keys):
  try:
    c = sm_subset.to_dict()
    for k in keys:
      target_dict[k] = c[k]
  except KeyError:
    pass

def extract_lead(r, k):
  return {'s': r[k]['status'], 'd': r[k]['dRel'], 'y': r[k]['yRel']} if k in r else {}

def quantize(o):
  if isinstance(o, dict):
    return {k: quantize(v) for k, v in o.items()}
  if isinstance(o, list):
    return [quantize(v) for v in o]
  if isinstance(o, float):
    return None if math.isnan(o) else round(o, 3)
  return o

class AppBridge:
  """Handles visualisation and settings BLE streams."""
  def __init__(self, sm=None):
    self.ble = BLEBridge()
    self.sm = sm if sm else messaging.SubMaster([
      'modelV2', 'selfdriveState', 'radarState', 'liveCalibration',
      'driverMonitoringState', 'carState',
      'uploaderState'
    ])
    self.rk = Ratekeeper(MESSAGE_HZ) # Ratekeeper for loop
    self.last_periodic_time = 0 # Track last periodic task
    self.last_1hz_task_time = 0
    self.local_wlan_ip = None
    self.active_wlan_ssid = None
    self.wifi_connect_attempt_ssid = None
    self.wifi_connect_attempt_start_time = None
    threading.Thread(target=self.ble.start, daemon=True).start() # Start BLE peripheral
    self.receiver = ChunkReceiver(self.ble) # Handle incoming messages in separate thread
    self.send_channel = None # Keep track of which channel to send messages
    self.send_car_names_cnt = -1
    self.hotspot_enabled = False
    self.hotspot_ip = None
    self.hw_helper = HardwareHelper()

  def scan_wifi(self):
    if hasattr(self, "wifiScanProcess"): # Avoid starting a new scan until the previous one finishes
      cloudlog.info("Wi-Fi scan already in progress, skipping")
      return
    def worker():
      try:
        cloudlog.info("Scanning for Wi-Fi")
        result = subprocess.run(
          ["sudo", "nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY", "device", "wifi", "list", "ifname", "wlan0"],
          text=True, capture_output=True, timeout=30
        )
        cloudlog.info("nmcli raw result:\n" + result.stdout)
        ssid_map = {}
        if result.returncode == 0:
          for line in result.stdout.strip().splitlines():
            if (parts := line.split(":")) and len(parts) >= 3:
              ssid, signal_str, security = parts[0], parts[1], ":".join(parts[2:])
              if not (signal_str.isdigit() and (signal := int(signal_str)) >= 0):
                cloudlog.warning(f"Skipping malformed line {line}")
                continue
              if ssid in ssid_map: # Deduplicate keep strongest signal
                if signal > ssid_map[ssid]["signal"]: ssid_map[ssid].update({"signal": signal, "security": security})
              else:
                ssid_map[ssid] = {"ssid": ssid, "signal": signal, "security": security}
          ssid_list = [
            {"ssid": e["ssid"], "password": bool(e["security"] and e["security"] != "--"), "signal": e["signal"]}
            for e in ssid_map.values()
            if "enterprise" not in e["security"].lower() and "802.1x" not in e["security"].lower() # Skip networks that require username
            and e["ssid"] and e["signal"] >= WIFI_SCAN_SIGNAL_THRESHOLD
          ]
          ssid_list.sort(key=lambda x: x["signal"], reverse=True)
          cloudlog.info("Wi-Fi list after filtering:\n" + "\n".join(f"{e['ssid']} pw={e['password']} sig={e['signal']}%" for e in ssid_list))
          self.wifiList = [{"ssid": e["ssid"], "password": e["password"]} for e in ssid_list]
      except Exception as e:
        cloudlog.error(f"Wi-Fi scan error {e}")
      finally:
        delattr(self, "wifiScanProcess") # Mark process finished
    self.wifiScanProcess = True # Mark process started
    threading.Thread(target=worker, daemon=True).start()

  def connect_to_wifi(self, ssid, password, cur_time):
    if not (ssid := ssid.strip()):
      return False
    self.wifi_connect_attempt_ssid = ssid
    self.wifi_connect_attempt_start_time = cur_time
    cmd = ['dev', 'wifi', 'connect', ssid, 'ifname', 'wlan0']
    if password:
      cmd += ['password', password]
    def run_nmcli():
      result = subprocess.run(["sudo", "nmcli"] + cmd, text=True, capture_output=True)
      if result.returncode != 0 and NO_NETWORK_REGEX.search(result.stderr):
        cloudlog.warning(f"Wi-Fi SSID {ssid} not found, clearing attempt.")
        self.wifi_connect_attempt_ssid = None
        self.wifi_connect_attempt_start_time = None
        return False
    threading.Thread(target=run_nmcli, daemon=True).start()
    return True

  def update_wlan_info(self):
    def get_wlan_info():
      def get_ip(iface):
        return next((a.address for a in psutil.net_if_addrs().get(iface, []) if a.family == socket.AF_INET), None)
      try:
        self.local_wlan_ip = get_ip("wlan0")
        self.active_wlan_ssid = (subprocess.run(["iwgetid", "wlan0", "-r"], capture_output=True, text=True, timeout=0.2).stdout.strip() or None)
        wlan1_ip = get_ip("wlan1")
        self.hotspot_enabled = bool(wlan1_ip)
        self.hotspot_ip = wlan1_ip
      except Exception:
        self.local_wlan_ip, self.active_wlan_ssid, self.hotspot_enabled, self.hotspot_ip = None, None, False, None
    threading.Thread(target=get_wlan_info, daemon=True).start()

  def send_visualisation_message(self, is_metric):
    (data := extract_model_data((sm := self.sm)['modelV2'].to_dict()))
    data["m"] = is_metric
    data['d'] = DONGLE_ID
    update_dict_from_sm(data, sm['selfdriveState'], ["enabled", "state", "experimentalMode",
                                                     "alertText1", "alertText2", "alertStatus",
                                                     "alertSize", "personality"])
    rd = sm['radarState'].to_dict()
    data["o"] = extract_lead(rd, "leadOne")
    data["t"] = extract_lead(rd, "leadTwo")
    update_dict_from_sm(data, sm['driverMonitoringState'], ["isActiveMode"])
    data["h"] = sm['liveCalibration'].to_dict().get("height", [None])[0]
    update_dict_from_sm(data, sm['carState'], ["vEgoCluster", "vCruiseCluster"])
    data = quantize(data)
    try:
      self.ble.chunk_and_send(CHANNEL_VISUALISATION, msgpack.packb(data))
    except Exception as e:
      cloudlog.error(f"BLE visualisation sending error: {e}")

  def send_settings_message(self, is_offroad, state, is_metric):
    sett = {'isOffroad': is_offroad}
    sett['dongleID'] = DONGLE_ID
    sett['gitCommit'] = GIT_COMMIT
    sett['currentVersion'] = CUR_VERSION
    sett['osVersion'] = OS_VERSION
    sett["state"] = str(state)
    sett['IsMetric'] = is_metric
    sett['localIP'] = self.local_wlan_ip
    sett['activeWlanSSID'] = \
      f"Connecting to\n{attempt_ssid}" if (attempt_ssid := self.wifi_connect_attempt_ssid) else self.active_wlan_ssid
    sett['hotspotEnabled'] = self.hotspot_enabled
    sett['hotspotIp'] = self.hotspot_ip
    sett['networkType'] = self.hw_helper.get_network_type()
    sett['simStatus'] = self.hw_helper.get_sim_status()
    sett['remainingDataUpload'] = f"{int(self.sm['uploaderState'].immediateQueueSize)} MB" if (sd := self.hw_helper.get_sd_status()) is None else sd

    if 0 <= self.send_car_names_cnt < 3:
      sett['carNames'] = SUPPORTED_CARS
      self.send_car_names_cnt += 1

    if hasattr(self, "supportTunnelOutput"):
      sett["supportTunnelOutput"] = self.supportTunnelOutput
      del self.supportTunnelOutput # remove temporary attribute from self

    if hasattr(self, "wifiList"): # Send Wi-Fi scan result
      sett["wifiList"] = self.wifiList
      del self.wifiList

    bool_keys = {
      'OpenpilotEnabledToggle', 'QuietMode', 'IsAlcEnabled', 'IsLdwEnabled',
      'SshEnabled', 'ConditionalExperimentalMode', 'RecordFront', 'UpdateAvailable',
      'UpdaterFetchAvailable'
    }
    string_keys = {
      'FeaturesPackage', 'CarName', 'UpdaterTargetBranch',
      'UpdaterState', 'UpdateFailedCount', 'LastUpdateTime',
      'GithubUsername', 'GsmApn'
    }

    for key in bool_keys:
      sett[key] = safe_get(key, True)
    for key in string_keys:
      sett[key] = safe_get(key, False)
    try:
      self.ble.chunk_and_send(CHANNEL_SETTINGS, msgpack.packb(sett))
    except Exception as e:
      cloudlog.error(f"BLE settings sending error: {e}")

  def run_remote_support(self):
    def worker():
      proc = subprocess.Popen(
        ["python3", "-u", "/usr/kommu/support_tunnel.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1
      )
      self.supportTunnelProcess = proc

      # Capture the first line (the port)
      if (line := proc.stdout.readline()):
        self.supportTunnelOutput = line.strip()
    threading.Thread(target=worker, daemon=True).start()

  def apply_settings_message(self, message, state, cur_time, is_offroad):
    """Apply a valid assembled settings message immediately."""
    try:
      c, settings = message
      if c != CHANNEL_SETTINGS:
        return
      match settings.pop('msgType', None):
        case 'saveToggle':
          safe_put_all(settings, True)
        case 'saveConfig':
          if (car_name := settings.pop('CarName', None)) is not None:
            safe_put_all({"CarName": car_name})
          if (features_to_set := settings.pop('FeaturesPackage', None)) is not None:
            features.set_features(features_to_set)
          if (apn := settings.pop('GsmApn', None)) is not None:
            self.hw_helper.update_gsm_apn(apn)
          # Put string setting if not one of the above keys, ensure above keys are popped so they will not be set below
          safe_put_all(settings)
        case 'resetCalibration':
          reset_calibration(state)
        case 'reboot':
          do_reboot(state)
        case 'tncAccepted':
          params.put_nonblocking("HasAcceptedTerms", terms_version)
          params.put_nonblocking("CompletedTrainingVersion", training_version)
        case 'changeTargetBranch':
          if targetBranch := settings.get('targetBranch'):
            threading.Thread(target=change_branch_and_update, args=(targetBranch,)).start()
        case 'update':
          match settings.get('action'):
            case 'check':
              send_update_signal("check")
            case 'install':
              do_reboot(state)
            case 'fetch':
              send_update_signal("fetch")
        case 'ssh':
          if username := settings.get('username'):
            params.put_nonblocking("GithubUsername", username)
            params.put_nonblocking("GithubSshKeys", settings.get('keys'))
        case 'wifi':
          if ssid := settings.get('ssid'):
            match settings.get('action'):
              case 'connect':
                self.connect_to_wifi(ssid, settings.get('password'), cur_time)
              case 'forget':
                forget_wifi_network(ssid)
        case 'formatSD':
          if is_offroad:
            self.hw_helper.format_sd()
        case 'remoteSupport':
          self.run_remote_support()
        case 'scanWifi':
          self.scan_wifi()
        case 'enableHotspot':
          enable_hotspot()
        case 'disableHotspot':
          disable_hotspot()
    except Exception as e:
      cloudlog.error(f"Apply BLE settings error: {e}")

  def handle_send_channel(self, msg):
    """Check for dongle ID and send channel message for received messages"""
    c, p = msg
    if len(p) > 128 * 1024:  # Reject oversized payloads before unpack (avoid native alloc/heap issues)
      cloudlog.error("appbridged: dropped oversized BLE message")
      return None
    try:
      m = msgpack.unpackb(p)
    except Exception as e:
      cloudlog.error(f"msgpack unpack error: {e}")
      return None
    device_list = m.pop('deviceList', []) # Always pop
    if not m.pop('devMode', False) and DONGLE_ID not in device_list:
      return None
    if m.get('msgType') == 'curPage':
      self.send_channel = c
      self.send_car_names_cnt = 0
      return None
    return c, m # Other message types, pass to next function

  def appbridged_thread(self):
    is_metric = None
    while True:
      (sm := self.sm).update(SM_UPDATE_INTERVAL)
      (rk := self.rk).monitor_time()

      # 1 Hz WiFi/hotspot task
      if (cur_time := monotonic()) - self.last_1hz_task_time >= 1:
        self.last_1hz_task_time = cur_time
        # Check WiFi and hotspot
        self.update_wlan_info()
        if attempt_ssid := self.wifi_connect_attempt_ssid:
          if ((connected := self.active_wlan_ssid == attempt_ssid) or
              (cur_time - self.wifi_connect_attempt_start_time) >= WIFI_CONNECT_TIMEOUT_SECONDS):
            if not connected:
              cloudlog.warning(f"Timeout reached, forgetting SSID {attempt_ssid}")
              forget_wifi_network(attempt_ssid)
            else:
              cloudlog.info(f"Wi-Fi {attempt_ssid} connected")
            self.wifi_connect_attempt_ssid = None
            self.wifi_connect_attempt_start_time = None

      if self.ble.connected: # Only receive/send if connected
        is_offroad = None # Always get latest is_offroad
        state = None
        # Apply any newly received message before sending
        while (msg := self.receiver.get_message()) is not None:
          if not (res := self.handle_send_channel(msg)):
            continue # If dongle ID does not match or it is a curPage message
          if is_offroad is None:
            is_offroad = params.get_bool("IsOffroad")
          if state is None:
            state = sm['selfdriveState'].state
          self.apply_settings_message(res, state, cur_time, is_offroad)

        # 3 Hz settings send
        if cur_time - self.last_periodic_time >= 0.333:
          self.last_periodic_time = cur_time
          is_metric = params.get_bool("IsMetric") # Always update at 3 Hz
          if self.send_channel == CHANNEL_SETTINGS:
            if is_offroad is None:
              is_offroad = params.get_bool("IsOffroad")
            if state is None:
              state = sm['selfdriveState'].state
            self.send_settings_message(is_offroad, state, is_metric)

        # Visualisation send
        if self.send_channel == CHANNEL_VISUALISATION:
          self.send_visualisation_message(is_metric)

      rk.keep_time()

def main():
  AppBridge().appbridged_thread()

if __name__ == "__main__":
  main()

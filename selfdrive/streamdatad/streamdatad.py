#!/usr/bin/env python3
import socket
import msgpack
import subprocess
import psutil
import threading
import re
import math
from time import monotonic
import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_version, get_commit, terms_version, training_version
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from openpilot.selfdrive.car.fingerprints import _FINGERPRINTS as FINGERPRINTS
from openpilot.common.features import Features
from openpilot.selfdrive.streamdatad.ble_helper import BLEBridge, ChunkReceiver

MESSAGE_HZ = 16 # Expected message rate, must match app visualisation value
params = Params()
features = Features()
DONGLE_ID = (params.get("DongleId") or b"").decode()
BLE_NAME = f"kommu-{DONGLE_ID}"  # BLE advertising name

# Channel IDs
CHANNEL_VISUALISATION = 0x01
CHANNEL_SETTINGS = 0x02

SM_UPDATE_INTERVAL = 33  # in ms, the interval where capnp submaster updates
WIFI_CONNECT_TIMEOUT_SECONDS = 20  # Timeout for device Wi-Fi connection attempts
NO_NETWORK_REGEX = re.compile(r"no network.*ssid", re.IGNORECASE)
SUPPORTED_MODELS = {getattr(car, 'value', car) for car in FINGERPRINTS}

# Call functions with cached values only once
GIT_COMMIT = get_commit()[:7]
CUR_VERSION = get_version()
OS_VERSION = HARDWARE.get_os_version()

def forget_wifi_network(ssid):
  if not ssid:
    return False
  threading.Thread(daemon=True, target=lambda: subprocess.run(["sudo", "nmcli", "con", "delete", ssid], text=True)).start()
  return True

def check_for_updates():
  subprocess.Popen(["pkill", "-SIGUSR1", "-f", "system.updated.updated"])

def fetch_update():
  subprocess.Popen(["pkill", "-SIGHUP", "-f", "system.updated.updated"])

def change_branch_and_update(target_branch):
  params.put("UpdaterTargetBranch", target_branch)
  check_for_updates()

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
    return params.get_bool(key) if is_bool else params.get(key).decode()
  except Exception:
    return False if is_bool else ''

def safe_put_all(settings_to_put, is_bool=False):
  """Safely store multiple parameters."""
  for param_key, value in settings_to_put.items():
    try:
      (params.put_bool_nonblocking if is_bool else params.put_nonblocking)(
        param_key, value if is_bool else str(value).strip())
    except Exception as e:
      cloudlog.error(f"Error putting {param_key}: {e}")

def reset_calibration(state):
  if state == log.ControlsState.OpenpilotState.disabled:
    params.remove("CalibrationParams")
    params.remove("LiveTorqueParameters")
    # Parameters below need to be removed for newer op version (v0.9.9)
    # keep this part for future reference, do not delete comment
    # https://github.com/commaai/openpilot/commit/1a3e3423035112b287a8fd0f73ef4222e4dd58ef
    # params.remove("LiveParameters")
    # params.remove("LiveParametersV2")
    # params.remove("LiveDelay")

def do_reboot(state):
  if state == log.ControlsState.OpenpilotState.disabled:
    params.put_bool_nonblocking("DoReboot", True)

def enable_hotspot():
  def start_service():
    try:
      subprocess.run(["sudo", "systemctl", "start", "wlan1-setup.service"])
    except Exception as e:
      cloudlog.error(f"Failed to start hotspot service: {e}")
  threading.Thread(target=start_service, daemon=True).start()

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

def is_supported_model(name: str) -> bool:
  return name.upper() in SUPPORTED_MODELS

class Streamer:
  """Handles visualisation and settings BLE streams."""
  def __init__(self, sm=None):
    self.ble = BLEBridge(local_name=BLE_NAME)
    self.sm = sm if sm else messaging.SubMaster([
      'modelV2', 'controlsState', 'radarState', 'liveCalibration',
      'driverMonitoringState', 'carState', 'longitudinalPlan',
    ])
    self.rk = Ratekeeper(MESSAGE_HZ) # Ratekeeper for loop
    self.last_periodic_time = 0 # Track last periodic task
    self.last_1hz_task_time = 0
    self.local_wlan_ip = None
    self.active_wlan_ssid = None
    self.current_wifi_iface_name = None
    self.wifi_connect_attempt_ssid = None
    self.wifi_connect_attempt_start_time = None
    threading.Thread(target=self.ble.start, daemon=True).start() # Start BLE peripheral
    self.receiver = ChunkReceiver(self.ble) # Handle incoming messages in separate thread
    self.send_channel = None # Keep track of which channel to send messages
    self.hotspot_enabled = False
    self.hotspot_ip = None

  def connect_to_wifi(self, ssid, password, cur_time):
    if not (ssid := ssid.strip()):
      return False
    self.wifi_connect_attempt_ssid = ssid
    self.wifi_connect_attempt_start_time = cur_time
    cmd = ['dev', 'wifi', 'connect', ssid]
    if password:
      cmd.extend(['password', password])
    if ifname := self.current_wifi_iface_name:
      cmd.extend(['ifname', ifname])
    def run_nmcli():
      result = subprocess.run(["sudo", "nmcli"] + cmd, text=True, capture_output=True)
      if result.returncode != 0 and NO_NETWORK_REGEX.search(result.stderr):
        cloudlog.warning(f"Wi-Fi SSID {ssid} not found, clearing attempt.")
        self.wifi_connect_attempt_ssid = None
        self.wifi_connect_attempt_start_time = None
        return False
    threading.Thread(target=run_nmcli, daemon=True).start()
    return True

  def update_wlan_info_async(self):
    def get_wlan_info():
      if (interfaces := psutil.net_if_addrs()) and (stats := psutil.net_if_stats()) and "wlan0" in interfaces and stats.get("wlan0", {}).isup:
        selected_iface = "wlan0"
      else:
        selected_iface = next(
          (iface for iface in interfaces if iface.startswith("wl") and iface != "wlan1" and stats.get(iface, {}).isup and
           any(a.family == socket.AF_INET for a in interfaces[iface])), None)
      ip_address = ssid = None
      if selected_iface:
        ip_address = next((a.address for a in interfaces[selected_iface] if a.family == socket.AF_INET), None)
        try:
          if (result := subprocess.run(
            ['nmcli', '-t', '-f', 'active,ssid,device', 'dev', 'wifi'], capture_output=True, text=True, timeout=0.1
          )).returncode == 0 and (output := result.stdout):
            for line in output.splitlines():
              if (parts := line.split(':')) and len(parts) >= 3 and parts[0] == 'yes' and parts[2] == selected_iface:
                ssid = parts[1]
                break
        except subprocess.TimeoutExpired:
          pass
        except Exception:
          pass
      self.local_wlan_ip = ip_address
      self.active_wlan_ssid = ssid
      self.current_wifi_iface_name = selected_iface
    threading.Thread(target=get_wlan_info, daemon=True).start()

  def check_hotspot_enabled(self):
    def check_interface():
      try:
        addrs, stats = psutil.net_if_addrs(), psutil.net_if_stats()
        self.hotspot_enabled = (s := stats.get(w1 := "wlan1")) and s.isup
        self.hotspot_ip = next((a.address for a in addrs.get(w1, []) if a.family == 2), None) if self.hotspot_enabled else None
      except Exception:
        self.hotspot_enabled = False
        self.hotspot_ip = None
    threading.Thread(target=check_interface, daemon=True).start()

  def send_visualisation_message(self, is_metric):
    (data := extract_model_data((sm := self.sm)['modelV2'].to_dict()))
    data["m"] = is_metric
    data['d'] = DONGLE_ID
    update_dict_from_sm(data, sm['controlsState'], ["enabled", "state", "experimentalMode", "vCruiseCluster",
                                                    "alertText1", "alertText2", "alertStatus", "alertSize"])
    rd = sm['radarState'].to_dict()
    data["o"] = extract_lead(rd, "leadOne")
    data["t"] = extract_lead(rd, "leadTwo")
    update_dict_from_sm(data, sm['driverMonitoringState'], ["isActiveMode"])
    data["h"] = sm['liveCalibration'].to_dict().get("height", [None])[0]
    update_dict_from_sm(data, sm['carState'], ["vEgoCluster"])
    update_dict_from_sm(data, sm['longitudinalPlan'], ["personality"])
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

    if hasattr(self, "supportTunnelOutput"):
      sett["supportTunnelOutput"] = self.supportTunnelOutput
      del self.supportTunnelOutput # remove temporary attribute from self

    bool_keys = {
      'OpenpilotEnabledToggle', 'QuietMode', 'IsAlcEnabled', 'IsLdwEnabled',
      'SshEnabled', 'ExperimentalMode', 'RecordFront', 'UpdateAvailable',
      'UpdaterFetchAvailable'
    }
    string_keys = {
      'LongitudinalPersonality', 'FeaturesPackage', 'FixFingerprint',
      'UpdaterTargetBranch', 'UpdaterState', 'UpdateFailedCount',
      'LastUpdateTime', 'GithubUsername', 'GsmApn'
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
          # Keep 'is not None' check for fingerprint and features to ensure empty strings are allowed (for unset)
          if (fix_fp := settings.pop('FixFingerprint', None)) is not None:
            if (fix_fp := fix_fp.strip()) == "" or is_supported_model(fix_fp):
              safe_put_all({'FixFingerprint': fix_fp})
          if (features_to_set := settings.pop('FeaturesPackage', None)) is not None:
            features.set_features(features_to_set)
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
              check_for_updates()
            case 'install':
              do_reboot(state)
            case 'fetch':
              fetch_update()
        case 'ssh':
          # Empty string is falsy, allows removal
          if username := settings.get('username'):
            params.put_nonblocking("GithubUsername", username)
            params.put_nonblocking("GithubSshKeys", settings.get('keys'))
          else:
            params.remove("GithubUsername")
            params.remove("GithubSshKeys")
        case 'wifi':
          if (ssid := settings.get('ssid')):
            match settings.get('action'):
              case 'connect':
                self.connect_to_wifi(ssid, settings.get('password'), cur_time)
              case 'forget':
                forget_wifi_network(ssid)
        case 'formatSD':
          if is_offroad:
            safe_put_all({"FormatSDCard": True}, True)
        case 'remoteSupport':
          self.run_remote_support()
        case 'enableHotspot':
          enable_hotspot()
    except Exception as e:
      cloudlog.error(f"Apply BLE settings error: {e}")

  def handle_send_channel(self, msg):
    """Check for dongle ID and send channel message for received messages"""
    c, p = msg
    try:
      m = msgpack.unpackb(p)
    except Exception as e:
      cloudlog.error(f"msgpack unpack error: {e}")
      return None
    if DONGLE_ID not in (m.pop('deviceList', None) or []):
      return None
    if m.get('msgType') == 'curPage':
      self.send_channel = c
      return None
    return c, m # Other message types, pass to next function

  def streamd_thread(self):
    is_metric = None
    while True:
      (sm := self.sm).update(SM_UPDATE_INTERVAL)
      (rk := self.rk).monitor_time()

      # 1 Hz WiFi/hotspot task
      if (cur_time := monotonic()) - self.last_1hz_task_time >= 1:
        self.last_1hz_task_time = cur_time
        # Check WiFi
        self.update_wlan_info_async()
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
        # Check hotspot
        self.check_hotspot_enabled()

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
            state = sm['controlsState'].state
          self.apply_settings_message(res, state, cur_time, is_offroad)

        # 3 Hz settings send
        if cur_time - self.last_periodic_time >= 0.333:
          self.last_periodic_time = cur_time
          is_metric = params.get_bool("IsMetric") # Always update at 3 Hz
          if self.send_channel == CHANNEL_SETTINGS:
            if is_offroad is None:
              is_offroad = params.get_bool("IsOffroad")
            if state is None:
              state = sm['controlsState'].state
            self.send_settings_message(is_offroad, state, is_metric)

        # Visualisation send
        if self.send_channel == CHANNEL_VISUALISATION:
          self.send_visualisation_message(is_metric)

      rk.keep_time()

def main():
  Streamer().streamd_thread()

if __name__ == "__main__":
  main()

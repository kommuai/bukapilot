#!/usr/bin/env python3
import socket
import msgpack
import subprocess
import psutil
import threading
import re
from time import monotonic, sleep
from openpilot.common.realtime import Ratekeeper
import cereal.messaging as messaging
from cereal import log
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_version, get_commit, terms_version, training_version
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from openpilot.selfdrive.car.fingerprints import _FINGERPRINTS as FINGERPRINTS
from openpilot.common.features import Features

SM_UPDATE_INTERVAL = 33
BUFFER_SIZE = 65536   # If buffer too small, SSH keys will not be fully received.
BIND_IP = "0.0.0.0"   # Bind to all network interfaces, allowing connections from any available network.
UDP_PORT = 5006
TCP_PORT = 5007
WIFI_CONNECT_TIMEOUT_SECONDS = 20 # Timeout for Wi-Fi connection attempts
NO_NETWORK_REGEX = re.compile(r"no network.*ssid", re.IGNORECASE)
params = Params()
DONGLE_ID = params.get("DongleId").decode("utf-8")
SUPPORTED_MODELS = {getattr(car, 'value', car) for car in FINGERPRINTS}
features = Features()

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

def extract_model_data(data_dict):
  try:
    return {key: data_dict[key] for key in ("position", "acceleration", "frameId")} | {
      f"{key[:-1]}{i}": item for key in ("laneLines", "roadEdges", "laneLineProbs", "roadEdgeStds")
      for i, item in enumerate(data_dict[key], 1)
    }
  except Exception:
    return {}

def safe_get(key, is_bool=False):
  """Retrieves a parameter value safely."""
  try:
    return params.get_bool(key) if is_bool else params.get(key).decode()
  except Exception:
    return False if is_bool else ''

def safe_put_all(settings_to_put, is_bool=False):
  """Stores multiple parameters safely."""
  for param_key, value in settings_to_put.items():
    try:
      (params.put_bool_nonblocking if is_bool else params.put_nonblocking)(param_key, value if is_bool else str(value))
    except Exception as e:
      cloudlog.error(f"Error putting {param_key}: {e}")

def reset_calibration():
  params.remove("CalibrationParams")
  params.remove("LiveTorqueParameters")
  # Parameters below need to be removed for newer op version
  # params.remove("LiveParameters")
  # params.remove("LiveParametersV2")
  # params.remove("LiveDelay")

def do_reboot(state):
  if state == log.ControlsState.OpenpilotState.disabled:
    params.put_bool_nonblocking("DoReboot", True)

def update_dict_from_sm(target_dict, sm_subset, keys):
  try:
    c = sm_subset.to_dict()
    for k in keys:
      target_dict[k] = c[k]
  except KeyError:
    pass

def is_supported_model(name: str) -> bool:
  return name.upper() in SUPPORTED_MODELS

class Streamer:
  def __init__(self, sm=None):
    self.udp_send_ip = None
    self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.tcp_conn = None
    self.sm = sm if sm else messaging.SubMaster([
      'modelV2', 'controlsState', 'radarState', 'liveCalibration',
      'driverMonitoringState', 'carState', 'longitudinalPlan',
    ])
    self.rk = Ratekeeper(25)  # Ratekeeper for 25 Hz loop
    self.last_periodic_time = 0  # Track last periodic task
    self.last_1hz_task_time = 0
    self.local_wlan_ip = None
    self.active_wlan_ssid = None
    self.current_wifi_iface_name = None
    self.wifi_connect_attempt_ssid = None
    self.wifi_connect_attempt_start_time = None
    self.setup_sockets()

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
      sleep(5) # Wait 5 seconds for user to get hotspot/Wi-Fi ready
      result = subprocess.run(["sudo", "nmcli"] + cmd, text=True, capture_output=True)
      if result.returncode != 0 and NO_NETWORK_REGEX.search(result.stderr):
        cloudlog.warning(f"Wi-Fi SSID {ssid} not found, clearing attempt.")
        self.wifi_connect_attempt_ssid = None
        self.wifi_connect_attempt_start_time = None
        return False
    threading.Thread(target=run_nmcli, daemon=True).start()
    return True

  def setup_sockets(self):
    (udp_sock := self.udp_sock).bind((BIND_IP, UDP_PORT))
    udp_sock.setblocking(False)
    (tcp_sock := self.tcp_sock).setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Enable reuse for TCP socket
    tcp_sock.bind((BIND_IP, TCP_PORT))
    tcp_sock.listen(1)
    tcp_sock.setblocking(False)

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

  def send_udp_message(self, is_metric):
    if send_ip := self.udp_send_ip:
      (data := extract_model_data((sm := self.sm)['modelV2'].to_dict())).update(sm['controlsState'].to_dict())
      data["IsMetric"] = is_metric
      data['dongleID'] = DONGLE_ID
      update_dict_from_sm(data, sm['radarState'], ["leadOne", "leadTwo"])
      update_dict_from_sm(data, sm['driverMonitoringState'], ["isActiveMode", "events"])
      update_dict_from_sm(data, sm['liveCalibration'], ["height"])
      update_dict_from_sm(data, sm['carState'], ["vEgoCluster"])
      update_dict_from_sm(data, sm['longitudinalPlan'], ["personality"])
      try: # Pack and send
        self.udp_sock.sendto(msgpack.packb(data), (send_ip, UDP_PORT))
      except (BlockingIOError, OSError):
        pass
      except Exception:
        pass

  def send_tcp_message(self, is_offroad, state, is_metric):
    if tcp_conn := self.tcp_conn:
      try:
        sett = {'isOffroad': is_offroad}
        sett['dongleID'] = DONGLE_ID
        sett['gitCommit'] = get_commit()[:7]
        sett['currentVersion'] = get_version()
        sett['osVersion'] = HARDWARE.get_os_version()
        sett["state"] = str(state)
        sett['IsMetric'] = is_metric
        sett['localIP'] = self.local_wlan_ip
        sett['activeWlanSSID'] = \
          f"Connecting to\n{attempt_ssid}" if (attempt_ssid := self.wifi_connect_attempt_ssid) else self.active_wlan_ssid

        bool_keys = {
          'OpenpilotEnabledToggle', 'QuietMode', 'IsAlcEnabled', 'IsLdwEnabled',
          'SshEnabled', 'ExperimentalMode', 'RecordFront', 'UpdateAvailable',
          'UpdaterFetchAvailable'
        }
        string_keys = {
          'LongitudinalPersonality', 'HardwareSerial', 'FeaturesPackage', 'FixFingerprint',
          'UpdaterCurrentReleaseNotes', 'UpdaterTargetBranch', 'UpdaterState', 'UpdateFailedCount',
          'LastUpdateTime', 'GithubUsername'
        }

        for key in bool_keys:
          sett[key] = safe_get(key, True)
        for key in string_keys:
          sett[key] = safe_get(key, False)
        tcp_conn.sendall(msgpack.packb(sett))

      except socket.error:
        self.tcp_conn = None # Reset connection on error

  def accept_new_connection(self):
    if not self.tcp_conn:
      try:
        self.tcp_conn, addr = self.tcp_sock.accept()
      except socket.error:
        pass

  def receive_udp_message(self):
    try:
      message, addr = self.udp_sock.recvfrom(BUFFER_SIZE)
      if message and DONGLE_ID in msgpack.unpackb(message): # Message only contains dongle ID list
        self.udp_send_ip = addr[0] # Update client IP
    except Exception:
      pass

  def receive_tcp_message(self, state, cur_time):
    if tcp_conn := self.tcp_conn:
      try:
        if message := tcp_conn.recv(BUFFER_SIZE, socket.MSG_DONTWAIT):
          try:
            settings = msgpack.unpackb(message)
            # Check if account is valid
            if DONGLE_ID in settings.pop('deviceList', []):
              match settings.pop('msgType'):
                case 'saveToggle':
                  safe_put_all(settings, True)
                case 'saveConfig':

                  if (fix_fp := settings.pop('FixFingerprint', None)) is not None:
                    if (fix_fp := fix_fp.strip()) == "" or is_supported_model(fix_fp):
                      safe_put_all({'FixFingerprint': fix_fp})
                  if (features_to_add := settings.pop('FeaturesPackage', None)) is not None:
                    features.set_features(features_to_add)

                  safe_put_all(settings)
                case 'resetCalibration':
                  reset_calibration()
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
                  safe_put_all({"FormatSDCard": True}, True)

          except Exception as e:
            cloudlog.error(f"\nError: {e}\nRaw TCP: {message}")
      except Exception:
        pass

  def streamd_thread(self):
    is_metric = None
    while True:
      (sm := self.sm).update(SM_UPDATE_INTERVAL)
      (rk:= self.rk).monitor_time()

      if (cur_time := monotonic()) - self.last_1hz_task_time >= 1: # 1 Hz
        self.last_1hz_task_time = cur_time
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

      if cur_time - self.last_periodic_time >= 0.333: # 3 Hz
        self.last_periodic_time = cur_time
        self.accept_new_connection()
        self.receive_tcp_message(state := sm['controlsState'].state, cur_time)
        self.send_tcp_message(params.get_bool("IsOffroad"), state, is_metric := params.get_bool("IsMetric"))
        self.receive_udp_message()

      self.send_udp_message(is_metric)
      rk.keep_time()

def main():
  Streamer().streamd_thread()

if __name__ == "__main__":
  main()

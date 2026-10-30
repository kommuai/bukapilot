#!/usr/bin/env python3
import msgpack
import hashlib
import subprocess
import threading
import re
import math
from time import monotonic
from uuid import uuid4
import datetime
import cereal.messaging as messaging
from cereal import log
from cereal.services import SERVICE_LIST
from openpilot.common.realtime import Ratekeeper
from openpilot.common.constants import CV
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_version, get_commit, terms_version, training_version
from openpilot.common.params import Params
from openpilot.selfdrive.nav.destination_store import (
  NAV_LAST_OPERATION_KEY, NAV_ROUTE_FAILURE_KEY, NAV_ROUTE_REQUEST_KEY,
  NAV_SESSION_ID_KEY, NAV_STATUS_KEY, apply_stop_operation, nav_stop_list_lock,
  NAV_STOP_INDEX_KEY, NAV_STOP_VERSION_KEY,
  parse_destination_json, read_stop_list, write_stop_list,
)
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.selfdrive.appbridged.nav_route_transfer import NavRouteTransfer, ROUTE_TRANSFER_IDLE_TIMEOUT_SECONDS
import json
from openpilot.system.hardware import HARDWARE
from opendbc.car.car_helpers import supported_cars
from openpilot.common.features import Features
from openpilot.selfdrive.appbridged.ble_helper import BLEBridge, ChunkReceiver
from openpilot.selfdrive.appbridged.hardware_helper import HardwareHelper
from openpilot.selfdrive.appbridged.video_constants import VIDEO_KEEPALIVE_PERIOD_SEC
from openpilot.selfdrive.appbridged.video_protocol import VideoProtocolHandler
from openpilot.selfdrive.appbridged.video_scanner import validate_storage
from openpilot.selfdrive.appbridged.video_hotspot import disable_hotspot, enable_hotspot

# BLE Constants
MAX_MESSAGE_HZ = SERVICE_LIST["modelV2"].frequency  # Cap the loop and Drive sends at fresh model updates.
MAX_IN_FLIGHT_FRAMES = 3  # Allows multiple frames to stay in flight while ACKs return.
params = Params()

def _nav_dest_key(dest) -> str:
  if not dest:
    return ""
  try:
    return f"{float(dest['latitude']):.5f}|{float(dest['longitude']):.5f}"
  except (KeyError, TypeError, ValueError):
    return ""


def _nav_dest_key_from_params(params) -> str:
  return _nav_dest_key(parse_destination_json(params.get("NavDestination")))


def _nav_json_param(key: str, default):
  raw = params.get(key)
  if isinstance(raw, bytes): raw = raw.decode("utf-8", errors="replace")
  if not raw: return default
  try:
    value = json.loads(raw)
  except (TypeError, ValueError, json.JSONDecodeError):
    return default
  return value

def _nav_int_param(key: str, default: int = 0) -> int:
  try:
    return int(params.get(key) or default)
  except (TypeError, ValueError, OverflowError):
    return default

DONGLE_ID = params.get("DongleId") or ""

# BLE Channel IDs
CHANNEL_VISUALISATION = 0x01
CHANNEL_SETTINGS = 0x02
CHANNEL_VIDEO = 0x03
EVENT_HISTORY_TYPES = {ET.NO_ENTRY, ET.SOFT_DISABLE, ET.IMMEDIATE_DISABLE}


# Wi-Fi/nmcli Constants
WIFI_CONNECT_TIMEOUT_SECONDS = 20 # Timeout for device Wi-Fi connection attempts
NO_NETWORK_REGEX = re.compile(r"no network.*ssid", re.IGNORECASE)
WIFI_SCAN_SIGNAL_THRESHOLD = 31 # Minimum signal strength required for Wi-Fi scan result
NAV_BLE_DIAG_MAX_PROBES = 30
NAV_BLE_DIAG_MIN_INTERVAL_SEC = 1.0
NAV_BLE_DIAG_MAX_AGE_SEC = 310.0
NAV_BLE_DIAG_MAPBOX_TIMEOUT_SEC = 30.0
NAV_BACKGROUND_SYNC_INTERVAL_SEC = 1.0
NAV_PROTOCOL_VERSION = 2
NAV_ROUTE_TRANSFER_VERSION = 2
NAV_ROUTE_SELECTION_MAX_BYTES = 1024 * 1024
NAV_ROUTE_SELECTION_MAX_PARTS = 255
NAV_ROUTE_SELECTION_MAX_CANDIDATES = 16
NAV_ROUTE_SELECTION_MAX_SET_BYTES = 4 * 1024 * 1024
NAV_ROUTE_SELECTION_IDLE_TIMEOUT_SEC = ROUTE_TRANSFER_IDLE_TIMEOUT_SECONDS


def _nav_validate_candidate_manifest(candidate_set_id, active_route_id, candidates):
  if not isinstance(candidate_set_id, str) or not 1 <= len(candidate_set_id) <= 100:
    return False
  if not isinstance(active_route_id, str) or not 1 <= len(active_route_id) <= 160:
    return False
  if not isinstance(candidates, list) or not 1 <= len(candidates) <= NAV_ROUTE_SELECTION_MAX_CANDIDATES:
    return False
  seen = set()
  aggregate_bytes = 0
  for entry in candidates:
    if not isinstance(entry, dict):
      return False
    route_id, digest, payload_bytes = entry.get('routeId'), entry.get('payloadSha256'), entry.get('payloadBytes')
    if (not isinstance(route_id, str) or not 1 <= len(route_id) <= 160 or route_id in seen
        or not isinstance(digest, str) or re.fullmatch(r'[0-9a-f]{64}', digest) is None
        or type(payload_bytes) is not int or not 32 <= payload_bytes <= NAV_ROUTE_SELECTION_MAX_BYTES):
      return False
    seen.add(route_id)
    aggregate_bytes += payload_bytes
    if aggregate_bytes > NAV_ROUTE_SELECTION_MAX_SET_BYTES:
      return False
  return active_route_id in seen


def _nav_validate_banner_instructions(banners):
  if not isinstance(banners, list) or len(banners) > 8:
    return False
  for banner in banners:
    if not isinstance(banner, dict):
      return False
    distance = banner.get('distanceAlongGeometry')
    if (isinstance(distance, bool) or not isinstance(distance, (int, float))
        or not math.isfinite(distance) or distance < 0):
      return False
    for group_name in ('primary', 'secondary', 'sub'):
      group = banner.get(group_name)
      if group is None:
        continue
      if not isinstance(group, dict):
        return False
      for field in ('text', 'type', 'modifier'):
        value = group.get(field)
        if value is not None and (not isinstance(value, str) or len(value) > 256):
          return False
      components = group.get('components')
      if components is not None:
        if not isinstance(components, list) or len(components) > 16:
          return False
        for component in components:
          if not isinstance(component, dict):
            return False
          text = component.get('text')
          if text is not None and (not isinstance(text, str) or len(text) > 256):
            return False
  return True


def _nav_parse_route_payload(raw, route_id, digest, payload_bytes):
  if (not isinstance(raw, bytes) or type(payload_bytes) is not int
      or not 32 <= payload_bytes <= NAV_ROUTE_SELECTION_MAX_BYTES
      or len(raw) != payload_bytes):
    raise ValueError('invalid_route_bytes')
  if not isinstance(digest, str) or re.fullmatch(r'[0-9a-f]{64}', digest) is None:
    raise ValueError('invalid_route_digest')
  if hashlib.sha256(raw).hexdigest() != digest:
    raise ValueError('route_digest_mismatch')
  text = raw.decode('utf-8', errors='strict')
  data = json.loads(text)
  if not isinstance(data, dict) or data.get('schemaVersion') != 1 or data.get('routeId') != route_id:
    raise ValueError('invalid_route_identity')
  geometry, steps = data.get('geometry'), data.get('steps')
  if not isinstance(geometry, list) or not 2 <= len(geometry) <= 50000:
    raise ValueError('invalid_route_geometry')
  for point in geometry:
    if not isinstance(point, dict): raise ValueError('invalid_route_geometry')
    lat, lon = point.get('latitude'), point.get('longitude')
    if (isinstance(lat, bool) or not isinstance(lat, (int, float)) or not math.isfinite(lat) or not -90 <= lat <= 90
        or isinstance(lon, bool) or not isinstance(lon, (int, float)) or not math.isfinite(lon) or not -180 <= lon <= 180):
      raise ValueError('invalid_route_geometry')
  if not isinstance(steps, list) or not 1 <= len(steps) <= 10000:
    raise ValueError('invalid_route_steps')
  for step in steps:
    if not isinstance(step, dict): raise ValueError('invalid_route_step')
    loc, distance, duration = step.get('location'), step.get('distance'), step.get('duration')
    if (not isinstance(loc, dict) or isinstance(distance, bool) or not isinstance(distance, (int, float))
        or not math.isfinite(distance) or distance < 0 or isinstance(duration, bool)
        or not isinstance(duration, (int, float)) or not math.isfinite(duration) or duration < 0
        or not isinstance(step.get('maneuver'), str) or len(step['maneuver']) > 64
        or not isinstance(step.get('instruction', ''), str) or len(step.get('instruction', '')) > 500
        or not isinstance(loc.get('latitude'), (int, float)) or isinstance(loc.get('latitude'), bool)
        or not math.isfinite(loc['latitude']) or not -90 <= loc['latitude'] <= 90
        or not isinstance(loc.get('longitude'), (int, float)) or isinstance(loc.get('longitude'), bool)
        or not math.isfinite(loc['longitude']) or not -180 <= loc['longitude'] <= 180
        or not _nav_validate_banner_instructions(step.get('bannerInstructions', []))):
      raise ValueError('invalid_route_step')
  speech = data.get('speechInstructions', [])
  if not isinstance(speech, list) or len(speech) > 20000:
    raise ValueError('invalid_route_guidance')
  for item in speech:
    if (not isinstance(item, dict) or not isinstance(item.get('announcement'), str)
        or not 1 <= len(item['announcement']) <= 500):
      raise ValueError('invalid_route_guidance')
  return text, data

# Device Constants
UPDATE_PROCESS = "system.updated.updated"
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
      'driverMonitoringState', 'carState', 'carControl', 'carOutput',
      'controlsState', 'liveParameters', 'longitudinalPlan',
      'uploaderState', 'gpsLocation', 'navInstruction'
    ])
    self._event_history = {}
    self.nav_panel_subscribed = False
    self.nav_panel_sync_requested = False
    self._video_dl_valid_at = 0.0
    self._video_dl_valid = False
    self.rk = Ratekeeper(MAX_MESSAGE_HZ) # Ratekeeper for loop
    self.last_periodic_time = 0 # Track last periodic task
    self.last_video_heartbeat_time = 0
    self.last_background_nav_sync_time = 0
    self.last_nav_control_sync_time = 0.0
    self.last_nav_control_log_key = ''
    self.last_1hz_task_time = 0
    self.local_wlan_ip = None
    self.active_wlan_ssid = None
    self.wifi_connect_attempt_ssid = None
    self.wifi_connect_attempt_start_time = None
    threading.Thread(target=self.ble.start, daemon=True).start() # Start BLE peripheral
    self.receiver = ChunkReceiver(self.ble) # Handle incoming messages in separate thread
    self.send_channel = None # Keep track of which channel to send messages
    self._latest_ack_supported = False
    self._latest_inflight_msg_ids = []
    self.send_car_names_cnt = -1
    self._nav_ble_diag_test_id = ''
    self._nav_ble_diag_last_sequence = 0
    self._nav_ble_diag_probe_count = 0
    self._nav_ble_diag_sequence_gaps = 0
    self._nav_ble_diag_started_at = 0.0
    self._nav_ble_diag_last_probe_at = 0.0
    self._nav_ble_diag_last_rate_log_at = 0.0
    self._nav_ble_diag_mapbox_last_slot = 0
    self._nav_ble_diag_mapbox_pending_sequence = 0
    self._nav_ble_diag_mapbox_pending_at = 0.0
    self._nav_ble_diag_stop_index = 0
    self._nav_ble_diag_route_stops = []
    self._nav_ble_diag_fake_gps = None
    self._nav_ble_diag_route_speed_mps = 0.0
    self._nav_ble_diag_route_started_elapsed = 0.0
    self._nav_ble_diag_mapbox_interval_sec = 0.0
    self._nav_ble_diag_max_mapbox_probes = 0
    self._nav_ble_diag_request_id = ""
    self._nav_ble_diag_dest_key = ""
    self._nav_ble_diag_route = None
    self._nav_ble_diag_route_accepted = 0
    self._nav_ble_diag_route_rejected = 0
    self._nav_ble_diag_route_too_large = 0
    self._nav_ble_diag_envelope_too_large = 0
    self._nav_ble_diag_skip_count = 0
    self._nav_ble_diag_status = "idle"
    self._nav_ble_diag_status_text = "No replay"
    self._nav_ble_diag_maneuver_type = ""
    self._nav_ble_diag_maneuver_modifier = ""
    self._nav_ble_diag_maneuver_distance = None
    self._nav_ble_diag_maneuver_primary_text = ""
    self._nav_ble_diag_maneuver_secondary_text = ""
    self._nav_ble_diag_maneuver_lanes = []
    self.hotspot_enabled = False
    self.hotspot_ip = None
    self.hw_helper = HardwareHelper()
    self.video_handler = VideoProtocolHandler(self.ble, self.hw_helper)
    self._nav_sync_error = ''
    self._nav_route_transfer = NavRouteTransfer()
    self._nav_route_select_transfer = NavRouteTransfer()
    self._nav_route_select_transfer_context = None
    self._nav_ble_diag_route_transfer = NavRouteTransfer()
    self._nav_ble_diag_select_transfer = NavRouteTransfer()
    self._nav_ble_diag_select_context = None
    self._nav_ble_diag_candidate_context = None
    self.ble.on_connect_callback = self.video_handler.on_ble_connected
    self.ble.on_disconnect_callback = self.video_handler.on_ble_disconnected
    self._wlan_info_wake = threading.Event()
    threading.Thread(target=self._wlan_info_worker, daemon=True, name="appbridge-wlan-info").start()
    self._wlan_info_wake.set()

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
    self._wlan_info_wake.set()

  def _wlan_info_worker(self):
    while True:
      self._wlan_info_wake.wait()
      self._wlan_info_wake.clear()
      try:
        if (result := subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True, timeout=0.3)).returncode == 0:
          ips = {}
          for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) >= 4 and fields[2] == "inet":
              ips.setdefault(fields[1].split("@", 1)[0], fields[3].split("/", 1)[0])
          self.local_wlan_ip = ips.get("wlan0")
          self.hotspot_ip = ips.get("wlan1")
          self.hotspot_enabled = bool(self.hotspot_ip)
      except (OSError, subprocess.SubprocessError):
        pass
      try:
        if (result := subprocess.run(["iwgetid", "wlan0", "-r"], capture_output=True, text=True, timeout=0.2)).returncode in (0, 1):
          self.active_wlan_ssid = result.stdout.strip() or None
      except (OSError, subprocess.SubprocessError):
        pass


  def _nav_control_snapshot(self) -> dict:
    now = monotonic()
    if now - self.last_nav_control_sync_time < NAV_BACKGROUND_SYNC_INTERVAL_SEC:
      return {}
    self.last_nav_control_sync_time = now

    instruction = _nav_json_param('NavInstructionState', {})
    if not isinstance(instruction, dict): instruction = {}
    route_active = bool(safe_get('NavHasRoute', True) or safe_get('NavDestination'))
    if not route_active or not instruction.get('valid'):
      return {}

    def number(value):
      try:
        value = float(value)
      except (TypeError, ValueError, OverflowError):
        return None
      return round(value, 3) if math.isfinite(value) else None

    nav_desires_allowed = safe_get('NavDesiresAllowed', True)
    nav_longitudinal_allowed = safe_get('NavLongitudinalAllowed', True)
    nav_lane_positioning_allowed = safe_get('NavLanePositioningAllowed', True)
    snapshot = {
      'routeActive': route_active,
      'instructionValid': bool(instruction.get('valid')),
      'navDesiresAllowed': nav_desires_allowed,
      'navLongitudinalAllowed': nav_longitudinal_allowed,
      'navLanePositioningAllowed': nav_lane_positioning_allowed,
      'lateralNavEligible': nav_desires_allowed,
      'longitudinalNavEligible': nav_longitudinal_allowed,
    }
    for key in ('maneuverType', 'maneuverModifier'):
      if (value := str(instruction.get(key) or '').strip()): snapshot[key] = value[:16]
    for key in ('maneuverDistance', 'distanceRemaining'):
      if (value := number(instruction.get(key))) is not None: snapshot[key] = value

    if self.sm.valid.get('carState'):
      cs = self.sm['carState']
      for name, field in (('vEgoMps', 'vEgo'), ('vCruiseKph', 'vCruise')):
        if (value := number(getattr(cs, field, None))) is not None: snapshot[name] = value
      for name, field in (
        ('steeringPressed', 'steeringPressed'),
        ('gasPressed', 'gasPressed'),
        ('brakePressed', 'brakePressed'),
      ):
        snapshot[name] = bool(getattr(cs, field, False))
      cruise = getattr(cs, 'cruiseState', None)
      snapshot['cruiseEnabled'] = bool(getattr(cruise, 'enabled', False))

    if self.sm.valid.get('selfdriveState'):
      sd = self.sm['selfdriveState']
      snapshot['controlsEnabled'] = bool(getattr(sd, 'enabled', False))
      snapshot['controlsActive'] = bool(getattr(sd, 'active', False))

    if self.sm.valid.get('carControl'):
      cc = self.sm['carControl']
      snapshot['latActive'] = bool(getattr(cc, 'latActive', False))
      snapshot['longActive'] = bool(getattr(cc, 'longActive', False))
      actuators = getattr(cc, 'actuators', None)
      for name, field in (
        ('steerTorqueCmd', 'torque'),
        ('steeringAngleCmdDeg', 'steeringAngleDeg'),
        ('accelCmdMps2', 'accel'),
        ('curvatureCmd', 'curvature'),
      ):
        if actuators is not None and (value := number(getattr(actuators, field, None))) is not None:
          snapshot[name] = value

    if self.sm.valid.get('carOutput'):
      output = getattr(self.sm['carOutput'], 'actuatorsOutput', None)
      for name, field in (
        ('steerTorqueApplied', 'torque'),
        ('steeringAngleAppliedDeg', 'steeringAngleDeg'),
      ):
        if output is not None and (value := number(getattr(output, field, None))) is not None:
          snapshot[name] = value
      if 'steerTorqueCmd' in snapshot and 'steerTorqueApplied' in snapshot:
        snapshot['steerTorqueDelta'] = round(snapshot['steerTorqueCmd'] - snapshot['steerTorqueApplied'], 3)
      if 'steeringAngleCmdDeg' in snapshot and 'steeringAngleAppliedDeg' in snapshot:
        snapshot['steeringAngleDeltaDeg'] = round(
          snapshot['steeringAngleCmdDeg'] - snapshot['steeringAngleAppliedDeg'], 3
        )

    if self.sm.valid.get('longitudinalPlan'):
      plan = self.sm['longitudinalPlan']
      if len(plan.speeds) and (value := number(plan.speeds[0])) is not None:
        snapshot['speedTargetMps'] = value
      if (value := number(getattr(plan, 'aTarget', None))) is not None:
        snapshot['accelTargetMps2'] = value
      snapshot['shouldStop'] = bool(getattr(plan, 'shouldStop', False))
      if (value := str(getattr(plan, 'longitudinalPlanSource', '') or '').strip()):
        snapshot['longitudinalPlanSource'] = value[:16]
      if 'accelCmdMps2' in snapshot and 'accelTargetMps2' in snapshot:
        snapshot['accelCommandDeltaMps2'] = round(
          snapshot['accelCmdMps2'] - snapshot['accelTargetMps2'], 3
        )

    if self.sm.valid.get('controlsState'):
      state = str(getattr(self.sm['controlsState'], 'longControlState', '') or '')
      if state: snapshot['longControlState'] = state
    return snapshot

  def _nav_ble_sync(self, out: dict, destination) -> bool:
    """Send the canonical navigation snapshot on every navigation channel."""
    try:
      out['navProtocolVersion'] = NAV_PROTOCOL_VERSION
      out['navRouteTransferVersion'] = NAV_ROUTE_TRANSFER_VERSION
      if self.sm.valid.get('gpsLocation'):
        g = self.sm['gpsLocation']
        out['lastNavPosition'] = {
          'latitude': float(g.latitude), 'longitude': float(g.longitude),
          'bearing': float(getattr(g, 'bearingDeg', 0.0) or 0.0),
          'speedMps': float(getattr(g, 'speed', 0.0) or 0.0),
          'horizontalAccuracyM': float(getattr(g, 'horizontalAccuracy', 0.0) or 0.0),
          'hasFix': bool(getattr(g, 'hasFix', False)),
          'unixTimestampMillis': int(getattr(g, 'unixTimestampMillis', 0) or 0),
        }
      with nav_stop_list_lock(blocking=False) as lock_acquired:
        if not lock_acquired:
          return False
        out['hasRoute'] = safe_get('NavHasRoute', True)
        out['rerouteNeeded'] = safe_get('NavRerouteNeeded', True)
        out['navStatus'] = params.get(NAV_STATUS_KEY) or ''
        candidate_context = _nav_json_param('NavRouteCandidateContext', None)
        installed_route_id = str(params.get('NavRouteInstalledRouteId') or '')
        if (out['hasRoute'] is True and out['navStatus'] == 'route_active'
            and isinstance(candidate_context, dict)
            and candidate_context.get('activeRouteId') == installed_route_id
            and candidate_context.get('destKey') == _nav_dest_key(destination)):
          out['navRouteCandidateContext'] = candidate_context
        selection_ack = _nav_json_param('NavRouteSelectAck', None)
        if isinstance(selection_ack, dict) and selection_ack.get('candidateSetId') == (
            candidate_context.get('candidateSetId') if isinstance(candidate_context, dict) else None
        ):
          out['navRouteSelectAck'] = selection_ack
        out['navStopList'] = read_stop_list(params)
        out['navStopIndex'] = _nav_int_param(NAV_STOP_INDEX_KEY)
        out['navStopListVersion'] = _nav_int_param(NAV_STOP_VERSION_KEY)
        request = _nav_json_param(NAV_ROUTE_REQUEST_KEY, None)
        request_dest = parse_destination_json(request.get('destination')) if isinstance(request, dict) else None
        request_dest_key = (
          f"{request_dest['latitude']:.5f}|{request_dest['longitude']:.5f}" if request_dest else ''
        )
        active_dest_key = _nav_dest_key(destination)
        if (isinstance(request, dict) and request.get('requestId')
            and request_dest_key and request_dest_key == active_dest_key):
          request = dict(request)
          request['destinationKey'] = active_dest_key
          out['navRouteRequest'] = request
        else:
          out['navRouteRequest'] = None
      out['navCapable'] = True
      if control := self._nav_control_snapshot():
        out['navControl'] = control
        log_key = json.dumps(control, sort_keys=True, separators=(',', ':'))
        if log_key != self.last_nav_control_log_key:
          self.last_nav_control_log_key = log_key
          cloudlog.warning(f"nav_control {log_key}")
      self._nav_sync_error = ''
      return True
    except Exception as e:
      error = f'{type(e).__name__}:{e}'
      if error != self._nav_sync_error:
        cloudlog.error(f'appbridged nav BLE sync failed error={error}')
      self._nav_sync_error = error
      return False

  def _nav_ble_status(self, out: dict, is_offroad: bool | None) -> bool:
    if is_offroad is not None:
      out['isOffroad'] = is_offroad
    destination_value = params.get('NavDestination')
    destination = parse_destination_json(destination_value)
    if not self._nav_ble_sync(out, destination):
      return False
    out['navActive'] = bool(destination_value) or out['hasRoute']
    try:
      clear_reason = params.get('NavDestinationWaypoints')
      if isinstance(clear_reason, bytes):
        clear_reason = clear_reason.decode()
      out['navClearReason'] = (json.loads(clear_reason) or {}).get('clearReason', '') if clear_reason else ''
    except Exception:
      out['navClearReason'] = ''
    if destination:
      out['navDestination'] = destination
    return True

  def send_background_nav_message(self, is_metric: bool, force: bool = False) -> None:
    nav = {}
    if not self._nav_ble_status(nav, None):
      return
    nav['IsMetric'] = is_metric
    if not force and not (
      nav.get('navActive') or nav.get('navStopList') or nav.get('navRouteRequest')
    ):
      return
    nav['isOffroad'] = params.get_bool('IsOffroad')
    if force:
      nav['NavDesiresAllowed'] = safe_get('NavDesiresAllowed', True)
      nav['NavLongitudinalAllowed'] = safe_get('NavLongitudinalAllowed', True)
      nav['NavLanePositioningAllowed'] = safe_get('NavLanePositioningAllowed', True)
    try:
      self.ble.chunk_and_send(CHANNEL_SETTINGS, msgpack.packb(nav))
    except Exception as e:
      cloudlog.error(f'appbridged background nav BLE send failed error={e}')

  def _update_event_history(self):
    if self.sm.valid.get('selfdriveState'):
      alert_data = self.sm['selfdriveState'].to_dict()
      alert_key = str(alert_data.get('alertType') or '')
      event_name, _, alert_type = alert_key.partition('/')
      if event_name and alert_type in EVENT_HISTORY_TYPES and (title := str(alert_data.get('alertText2') or '').strip()):
        self._event_history[alert_key] = {
          'lastSeenAt': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z'),
          'title': title,
          'alertStatus': alert_data.get('alertStatus'),
        }

  def _video_download_valid(self):
    # Live frames repeat this status; list requests and video actions validate independently.
    if (now := monotonic()) - self._video_dl_valid_at >= 1.0:
      self._video_dl_valid_at = now
      try:
        self._video_dl_valid = validate_storage(self.hw_helper)[0]
      except Exception:
        self._video_dl_valid = False
    return self._video_dl_valid

  def send_visualisation_message(self, is_metric):
    if self.ble.tx_char is None:
      self._latest_ack_supported = False
      self._latest_inflight_msg_ids.clear()
    if self._latest_ack_supported and len(self._latest_inflight_msg_ids) >= MAX_IN_FLIGHT_FRAMES:
      return

    # Offroad / no model: still send a light frame so nav guidance (and cancel) can update.
    sm = self.sm
    try:
      data = extract_model_data(sm["modelV2"].to_dict())
    except Exception:
      data = {}
    data["m"] = is_metric
    data["d"] = DONGLE_ID
    data['eventHistory'] = sorted(self._event_history.values(), key=lambda event: event['lastSeenAt'], reverse=True)
    try:
      update_dict_from_sm(data, sm["selfdriveState"], ["enabled", "state", "experimentalMode",
                                                       "alertText1", "alertText2", "alertStatus",
                                                       "alertSize", "personality"])
      rd = sm["radarState"].to_dict()
      data["o"] = extract_lead(rd, "leadOne")
      data["t"] = extract_lead(rd, "leadTwo")
      update_dict_from_sm(data, sm["driverMonitoringState"], ["isActiveMode"])
      data["h"] = sm["liveCalibration"].to_dict().get("height", [None])[0]
      update_dict_from_sm(data, (car_state := sm["carState"]), ["vEgoCluster"])
      update_dict_from_sm(data, car_state.cruiseState, ["speedCluster"])
      data["vCruiseCluster"] = data.pop("speedCluster") * CV.MS_TO_KPH  # Device field is in m/s.
    except Exception:
      pass
    if sm.valid.get('controlsState'):
      try:
        controls_state = sm['controlsState']
        steering_demand = None
        if controls_state.lateralControlState.which() == 'angleState':
          if sm.valid.get('carState') and sm.valid.get('liveParameters'):
            car_state = sm['carState']
            speed = float(car_state.vEgo)
            curvature = float(controls_state.curvature)
            desired_curvature = float(controls_state.desiredCurvature)
            roll = float(sm['liveParameters'].roll)
            steering_demand = (curvature * speed ** 2 - roll * ACCELERATION_DUE_TO_GRAVITY) / 3.0 \
              + (desired_curvature - curvature) * speed ** 2
        elif sm.valid.get('carOutput'):
          steering_demand = -float(sm['carOutput'].actuatorsOutput.torque)
        elif sm.valid.get('carControl'):
          steering_demand = -float(sm['carControl'].actuators.torque)
        if steering_demand is not None and math.isfinite(steering_demand):
          data['steeringDemand'] = min(max(steering_demand, -1.0), 1.0)
      except (AttributeError, TypeError, ValueError, OverflowError):
        pass
    # Drive navigation for Visualisation (nv only when active; old apps ignore unknown keys).
    destination = None
    try:
      nv = None
      dn = ''
      destination = parse_destination_json(params.get('NavDestination'))
      if destination:
        dn = str(destination.get('name') or destination.get('place_name') or 'Destination')[:48]
      if sm.valid.get('navInstruction') and (ni := sm['navInstruction']):
        primary = str(getattr(ni, 'maneuverPrimaryText', '') or '')
        secondary = str(getattr(ni, 'maneuverSecondaryText', '') or '')
        md = float(getattr(ni, 'maneuverDistance', 0.0) or 0.0)
        rd_m = float(getattr(ni, 'distanceRemaining', 0.0) or 0.0)
        if primary or md > 0 or rd_m > 0 or dn:
          lanes = [[
            [direction for direction in (str(value) for value in lane.directions) if direction != 'none'],
            bool(lane.active), str(lane.activeDirection),
          ] for lane in ni.lanes]
          nv = {
            'p': (primary[:48] or dn), 'q': secondary[:48], 'md': md, 'rd': rd_m,
            'mm': str(getattr(ni, 'maneuverModifier', '') or '')[:16],
            'mt': str(getattr(ni, 'maneuverType', '') or '')[:16], 'dn': dn,
          }
          if lanes:
            nv['l'] = lanes
      if nv is None and dn:
        nv = {'p': dn, 'q': '', 'md': -1.0, 'rd': -1.0, 'mm': '', 'mt': 'destination', 'dn': dn}
      if nv is not None:
        data['nv'] = nv
    except Exception as e:
      cloudlog.error(f"BLE visualisation nav guidance error: {e}")
    self._nav_ble_sync(data, destination)
    data['videoDlValid'] = self._video_download_valid()
    data = quantize(data)
    try:
      msg_id = self.ble.chunk_and_send(CHANNEL_VISUALISATION, msgpack.packb(data))
      if msg_id is not None:
        self._latest_inflight_msg_ids.append(msg_id)
        if len(self._latest_inflight_msg_ids) > MAX_IN_FLIGHT_FRAMES:
          del self._latest_inflight_msg_ids[:-MAX_IN_FLIGHT_FRAMES]
    except Exception as e:
      cloudlog.error(f"BLE visualisation sending error: {e}")

  def send_settings_message(self, is_offroad, state, is_metric):
    sett = {}
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
    sett['videoDlValid'] = self._video_download_valid()

    self._nav_ble_status(sett, is_offroad)
    sett['NavDesiresAllowed'] = safe_get('NavDesiresAllowed', True)
    sett['NavLongitudinalAllowed'] = safe_get('NavLongitudinalAllowed', True)
    sett['NavLanePositioningAllowed'] = safe_get('NavLanePositioningAllowed', True)
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
      'GithubUsername', 'GsmApn', 'DrivePathOffset'
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

      # Capture the first line (the port)
      if (line := proc.stdout.readline()):
        self.supportTunnelOutput = line.strip()
    threading.Thread(target=worker, daemon=True).start()

  def _apply_nav_stop_operation(self, command: str, settings: dict) -> None:
    operation_id = settings.get('operationId')
    if not isinstance(operation_id, str) or not operation_id or len(operation_id) > 96:
      cloudlog.warning(f'appbridged nav operation rejected command={command} reason=invalid_operation_id')
      return

    with nav_stop_list_lock():
      last_operation_id = str(params.get(NAV_LAST_OPERATION_KEY) or '')
      if operation_id == last_operation_id:
        cloudlog.warning(f'appbridged nav operation duplicate command={command} operation={operation_id}')
        return

      stops = read_stop_list(params)
      old_head_id = stops[0]['id'] if stops else ''
      updated, ok, reason, changed = apply_stop_operation(stops, command, settings)
      new_head_id = updated[0]['id'] if updated else ''
      active_stop_changed = old_head_id != new_head_id
      route_reset = active_stop_changed or command == 'navClearStops'

      if ok:
        if changed:
          updated = write_stop_list(params, updated, index=0)
        if route_reset:
          self._nav_route_select_transfer.clear()
          self._nav_route_select_transfer_context = None
          self._nav_route_transfer.clear()
          params.remove(NAV_ROUTE_REQUEST_KEY)
          params.remove(NAV_ROUTE_FAILURE_KEY)
          params.remove('NavRouteData')
          params.remove('NavInstructionState')
          params.remove('NavDestinationWaypoints')
          params.remove('NavRouteCandidateContext')
          params.remove('NavRouteInstalledRouteId')
          params.remove('NavRouteSelectEnvelope')
          params.remove('NavRouteSelectAck')
          params.put_bool('NavHasRoute', False)
          if updated:
            params.put(NAV_SESSION_ID_KEY, f'nav-{uuid4().hex}')
            params.put_bool('NavRerouteNeeded', True)
            params.put(NAV_STATUS_KEY, 'paused_offroad' if params.get_bool('IsOffroad') else 'route_pending')
          else:
            params.remove(NAV_SESSION_ID_KEY)
            params.put_bool('NavRerouteNeeded', False)
            params.put(NAV_STATUS_KEY, 'idle')

      count = len(updated if ok else stops)
      version = _nav_int_param(NAV_STOP_VERSION_KEY)
      params.put(NAV_LAST_OPERATION_KEY, operation_id)
      skip_fields = ''
      if command == 'navSkipStop':
        skip_fields = ' mode=manual'
        if self.sm.valid.get('navInstruction'):
          try:
            distance = float(getattr(self.sm['navInstruction'], 'distanceRemaining', float('nan')))
            if math.isfinite(distance) and distance >= 0.0:
              skip_fields += f' remaining_m={distance:.1f}'
          except (TypeError, ValueError, OverflowError):
            pass
      cloudlog.warning(
        f'appbridged nav operation command={command}{skip_fields} operation={operation_id} ok={int(ok)} '
        f'reason={reason} count={count} version={version} '
        f'active_changed={int(active_stop_changed)}'
      )

  def _apply_nav_route_response(self, settings: dict, is_offroad: bool) -> None:
    with nav_stop_list_lock():
      request_id = str(settings.get('requestId') or '')
      request = _nav_json_param(NAV_ROUTE_REQUEST_KEY, None)
      expected_request = request.get('requestId', '') if isinstance(request, dict) else ''
      if not request_id or request_id != expected_request:
        cloudlog.warning(
          f'appbridged navRouteResponse stale request={request_id or "none"} '
          f'expected={expected_request or "none"}'
        )
        return

      expected = _nav_dest_key_from_params(params)
      request_dest = parse_destination_json(request.get('destination')) if isinstance(request, dict) else None
      request_dest_key = (
        f"{request_dest['latitude']:.5f}|{request_dest['longitude']:.5f}" if request_dest else ''
      )
      request_session = str(request.get('sessionId') or 'legacy') if isinstance(request, dict) else ''
      active_session = str(params.get(NAV_SESSION_ID_KEY) or 'legacy')
      pushed = str(settings.get('destKey') or '')
      response_session = settings.get('sessionId')
      response_generation = settings.get('routeGeneration')
      request_generation = request.get('routeGeneration') if isinstance(request, dict) else None
      request_stop_version = request.get('stopVersion') if isinstance(request, dict) else None
      response_stop_version = settings.get('stopVersion')
      active_stop_version = _nav_int_param(NAV_STOP_VERSION_KEY)
      if is_offroad or params.get_bool('IsOffroad'):
        self._nav_route_transfer.clear()
        params.remove(NAV_ROUTE_REQUEST_KEY)
        params.put(NAV_STATUS_KEY, 'paused_offroad')
        cloudlog.warning(f'appbridged navRouteResponse discarded request={request_id} reason=offroad')
        return
      stop_version_matches = request_stop_version is None or (
        type(request_stop_version) is int
        and response_stop_version == request_stop_version
        and active_stop_version == request_stop_version
      )
      if (
        not expected or request_dest_key != expected or (pushed and expected != pushed)
        or request_session != active_session
        or (response_session is not None and str(response_session) != request_session)
        or (response_generation is not None and response_generation != request_generation)
        or not stop_version_matches
      ):
        params.remove(NAV_ROUTE_REQUEST_KEY)
        params.put_bool('NavHasRoute', False)
        stops = read_stop_list(params)
        params.put_bool('NavRerouteNeeded', bool(stops))
        params.put(NAV_STATUS_KEY, 'route_pending' if stops else 'idle')
        cloudlog.warning(
          f'appbridged navRouteResponse stale_dest request={request_id} '
          f'requested={request_dest_key or "none"} active={expected or "none"} '
          f'pushed={pushed or "none"} session_match={int(request_session == active_session)}'
        )
        return

      raw_payload = settings.get('routePayloadUtf8')
      if raw_payload is None:
        self._nav_route_transfer.clear()
        params.remove(NAV_ROUTE_REQUEST_KEY)
        params.put(NAV_ROUTE_FAILURE_KEY, json.dumps({
          'requestId': request_id,
          'reason': str(settings.get('error') or 'route_unavailable')[:120],
        }))
        if not params.get_bool('NavHasRoute'):
          params.put(NAV_STATUS_KEY, 'route_unavailable')
        cloudlog.warning(f'appbridged navRouteResponse unavailable request={request_id}')
        return
      candidate_set_id = settings.get('candidateSetId')
      active_route_id = settings.get('activeRouteId')
      candidate_routes = settings.get('candidateRoutes')
      if not _nav_validate_candidate_manifest(candidate_set_id, active_route_id, candidate_routes):
        cloudlog.warning(f'appbridged navRouteResponse invalid request={request_id} reason=invalid_manifest')
        return
      primary = next((entry for entry in candidate_routes if entry['routeId'] == active_route_id), None)
      try:
        route_str, route_data = _nav_parse_route_payload(raw_payload, active_route_id, primary['payloadSha256'], primary['payloadBytes'])
      except (TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError) as e:
        cloudlog.warning(
          f'appbridged navRouteResponse invalid request={request_id} bytes={len(raw_payload) if isinstance(raw_payload, bytes) else 0} '
          f'error={type(e).__name__}'
        )
        return
      if route_data.get('totalDistance', 0) <= 0 or route_data.get('totalDuration', 0) <= 0:
        cloudlog.warning(f'appbridged navRouteResponse invalid request={request_id} reason=invalid_metrics')
        return
      candidate_context = {
        'candidateSetId': candidate_set_id,
        'activeRouteId': active_route_id,
        'candidateRoutes': candidate_routes,
        'sessionId': request_session,
        'requestId': request_id,
        'routeGeneration': request_generation,
        'destKey': expected,
        'stopVersion': request_stop_version,
      }
      params.put('NavRouteCandidateContext', json.dumps(candidate_context, separators=(',', ':')))
      params.put('NavRouteData', route_str)
      params.put_bool('NavHasRoute', True)
      params.put_bool('NavRerouteNeeded', False)
      params.remove(NAV_ROUTE_FAILURE_KEY)
      params.put(NAV_STATUS_KEY, 'route_received')
      self._nav_route_transfer.clear()
      cloudlog.warning(
        f'appbridged navRouteResponse ok request={request_id} dest={expected} bytes={len(route_str)} '
        f'points={len(route_data.get("geometry") or [])} steps={len(route_data.get("steps") or [])}'
      )

  def _send_nav_route_select_ack(self, settings: dict, status: str, reason: str = '') -> None:
    ack = {
      'selectionId': str(settings.get('selectionId') or '')[:100],
      'candidateSetId': str(settings.get('candidateSetId') or '')[:100],
      'selectedRouteId': str(settings.get('selectedRouteId') or '')[:160],
      'selectionSequence': settings.get('selectionSequence'),
      'payloadSha256': str(settings.get('_candidateDigest') or settings.get('payloadSha256') or '')[:64],
      'status': status,
    }
    if reason:
      ack['reason'] = str(reason)[:40]
    try:
      self.ble.chunk_and_send(CHANNEL_SETTINGS, msgpack.packb({'navRouteSelectAck': ack}))
    except Exception as e:
      cloudlog.error(f'appbridged nav route select ACK send failed error={type(e).__name__}')

  def _nav_route_select_context_reason(self, settings: dict) -> str:
    context = _nav_json_param('NavRouteCandidateContext', None)
    if not isinstance(settings, dict) or not isinstance(context, dict):
      return 'stale_context'
    if params.get_bool('IsOffroad') or not params.get_bool('NavHasRoute'):
      return 'stale_context'
    if str(params.get('NavRouteInstalledRouteId') or '') != str(context.get('activeRouteId') or ''):
      return 'stale_context'
    if (settings.get('candidateSetId') != context.get('candidateSetId')
        or settings.get('sessionId') != context.get('sessionId')
        or settings.get('requestId') != context.get('requestId')
        or settings.get('routeGeneration') != context.get('routeGeneration')
        or settings.get('destKey') != context.get('destKey')
        or context.get('destKey') != _nav_dest_key_from_params(params)
        or context.get('sessionId') != str(params.get(NAV_SESSION_ID_KEY) or 'legacy')):
      return 'stale_context'
    if context.get('stopVersion') is not None and (
        type(context.get('stopVersion')) is not int
        or settings.get('stopVersion') != context.get('stopVersion')
        or _nav_int_param(NAV_STOP_VERSION_KEY) != context.get('stopVersion')):
      return 'stale_context'
    if not _nav_validate_candidate_manifest(
        context.get('candidateSetId'), context.get('activeRouteId'), context.get('candidateRoutes')):
      return 'stale_context'
    if not isinstance(settings.get('selectionId'), str) or not 1 <= len(settings['selectionId']) <= 100:
      return 'invalid_route'
    if type(settings.get('selectionSequence')) is not int or settings['selectionSequence'] < 1:
      return 'invalid_route'
    route_id = settings.get('selectedRouteId')
    if not isinstance(route_id, str) or not 1 <= len(route_id) <= 160:
      return 'wrong_route'
    manifest = next((entry for entry in context['candidateRoutes'] if entry['routeId'] == route_id), None)
    if manifest is None:
      return 'wrong_route'
    settings['_candidateDigest'] = manifest['payloadSha256']
    settings['_candidateBytes'] = manifest['payloadBytes']
    return ''

  def _stage_nav_route_selection(self, settings: dict, raw: bytes) -> None:
    reason = self._nav_route_select_context_reason(settings)
    if reason:
      self._send_nav_route_select_ack(settings, 'rejected', reason)
      return
    try:
      route_text, route_data = _nav_parse_route_payload(raw, settings['selectedRouteId'], settings['_candidateDigest'], settings['_candidateBytes'])
      if route_data.get('totalDistance', 0) <= 0 or route_data.get('totalDuration', 0) <= 0:
        raise ValueError('invalid_route_metrics')
    except (TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError):
      self._send_nav_route_select_ack(settings, 'rejected', 'invalid_route')
      return
    context = _nav_json_param('NavRouteCandidateContext', None)
    highest_sequence = context.get('selectionSequence', 0) if isinstance(context, dict) else -1
    ack = _nav_json_param('NavRouteSelectAcceptedAck', None)
    if type(highest_sequence) is not int or highest_sequence < 0:
      self._send_nav_route_select_ack(settings, 'rejected', 'stale_context')
      return
    if settings['selectionSequence'] <= highest_sequence:
      if (settings['selectionSequence'] == highest_sequence and isinstance(ack, dict)
          and ack.get('selectionSequence') == settings['selectionSequence']
          and ack.get('selectionId') == settings['selectionId']
          and ack.get('candidateSetId') == settings['candidateSetId']
          and ack.get('selectedRouteId') == settings['selectedRouteId']
          and ack.get('payloadSha256') == settings['_candidateDigest']):
        self._send_nav_route_select_ack(settings, ack.get('status', 'rejected'), ack.get('reason', ''))
      else:
        self._send_nav_route_select_ack(settings, 'rejected', 'stale_selection')
      return
    previous = _nav_json_param('NavRouteSelectEnvelope', None)
    envelope = {key: settings[key] for key in (
      'selectionId', 'selectionSequence', 'candidateSetId', 'selectedRouteId', 'sessionId', 'requestId',
      'routeGeneration', 'destKey',
    ) if key in settings}
    if 'stopVersion' in settings:
      envelope['stopVersion'] = settings['stopVersion']
    envelope['payloadSha256'] = settings['_candidateDigest']
    envelope['routePayloadBase64'] = __import__('base64').b64encode(raw).decode('ascii')
    envelope['receivedAtMonotonic'] = monotonic()
    if isinstance(previous, dict) and previous.get('selectionId') == envelope['selectionId']:
      if all(previous.get(key) == envelope.get(key) for key in (
          'selectionSequence', 'candidateSetId', 'selectedRouteId', 'payloadSha256', 'routePayloadBase64')):
        return
      self._send_nav_route_select_ack(settings, 'rejected', 'wrong_route')
      return
    params.put('NavRouteSelectEnvelope', json.dumps(envelope, separators=(',', ':')))
    cloudlog.warning(
      f'appbridged nav route selection staged set={settings["candidateSetId"][:36]} '
      f'route={settings["selectedRouteId"][:48]} bytes={len(raw)}'
    )

  def _apply_nav_route_select(self, settings: dict, is_offroad: bool) -> None:
    if is_offroad or params.get_bool('IsOffroad'):
      self._send_nav_route_select_ack(settings, 'rejected', 'stale_context')
      return
    raw = settings.get('routePayloadUtf8')
    if not isinstance(raw, bytes) or len(raw) > NAV_ROUTE_SELECTION_MAX_BYTES:
      self._send_nav_route_select_ack(settings, 'rejected', 'invalid_route')
      return
    self._stage_nav_route_selection(settings, raw)

  def _apply_nav_route_select_start(self, settings: dict, is_offroad: bool) -> None:
    reason = self._nav_route_select_context_reason(settings)
    total_bytes, part_count = settings.get('totalBytes'), settings.get('partCount')
    if (reason or is_offroad or params.get_bool('IsOffroad')
        or type(total_bytes) is not int or not 32 <= total_bytes <= NAV_ROUTE_SELECTION_MAX_BYTES
        or type(part_count) is not int or not 1 <= part_count <= NAV_ROUTE_SELECTION_MAX_PARTS):
      self._nav_route_select_transfer.clear()
      self._nav_route_select_transfer_context = None
      self._send_nav_route_select_ack(settings, 'rejected', reason or 'invalid_route')
      return
    result = self._nav_route_select_transfer.start(settings)
    if not result.get('ok'):
      self._nav_route_select_transfer.clear()
      self._nav_route_select_transfer_context = None
      self._send_nav_route_select_ack(settings, 'rejected', 'invalid_route')
      return
    self._nav_route_select_transfer_context = dict(settings)

  def _apply_nav_route_select_part(self, settings: dict, is_offroad: bool) -> None:
    context = self._nav_route_select_transfer_context
    reason = self._nav_route_select_context_reason(context) if context else 'stale_context'
    if reason or is_offroad or params.get_bool('IsOffroad'):
      self._nav_route_select_transfer.clear()
      self._nav_route_select_transfer_context = None
      self._send_nav_route_select_ack(context or settings, 'rejected', reason or 'stale_context')
      return
    result = self._nav_route_select_transfer.add_part(settings)
    if not result.get('ok'):
      self._nav_route_select_transfer.clear()
      self._nav_route_select_transfer_context = None
      if result.get('reason') != 'stale_transfer':
        self._send_nav_route_select_ack(context, 'rejected', 'transfer_incomplete')
      return
    if result.get('complete'):
      self._nav_route_select_transfer_context = None
      self._stage_nav_route_selection(context, result['routeBytes'])

  def _nav_route_transfer_matches_request(self, message: dict, request: dict) -> bool:
    request_id = str(request.get('requestId') or '')
    session_id = str(request.get('sessionId') or 'legacy')
    generation = request.get('routeGeneration')
    destination = parse_destination_json(request.get('destination'))
    requested_dest_key = (
      f"{destination['latitude']:.5f}|{destination['longitude']:.5f}" if destination else ''
    )
    active_dest_key = _nav_dest_key_from_params(params)
    return bool(
      request_id and message.get('requestId') == request_id
      and message.get('sessionId') == session_id
      and isinstance(generation, int) and not isinstance(generation, bool)
      and message.get('routeGeneration') == generation
      and active_dest_key and requested_dest_key == active_dest_key
      and message.get('destKey') == active_dest_key
      and session_id == str(params.get(NAV_SESSION_ID_KEY) or 'legacy')
      and (
        request.get('stopVersion') is None
        or (
          type(request.get('stopVersion')) is int
          and message.get('stopVersion') == request.get('stopVersion')
          and _nav_int_param(NAV_STOP_VERSION_KEY) == request.get('stopVersion')
        )
      )
    )

  def _fail_nav_route_transfer_locked(self, request_id: str, transfer_id: str, reason: str) -> None:
    self._nav_route_transfer.clear(transfer_id or None)
    request = _nav_json_param(NAV_ROUTE_REQUEST_KEY, None)
    if not isinstance(request, dict) or request.get('requestId') != request_id:
      return
    had_active_route = params.get_bool('NavHasRoute')
    params.remove(NAV_ROUTE_REQUEST_KEY)
    params.put(NAV_ROUTE_FAILURE_KEY, json.dumps({
      'requestId': request_id, 'reason': str(reason or 'transfer_failed')[:80],
    }, separators=(',', ':')))
    if not had_active_route:
      params.put_bool('NavRerouteNeeded', bool(read_stop_list(params)))
      params.put(NAV_STATUS_KEY, 'route_unavailable')

  def _apply_nav_route_response_start(self, settings: dict, is_offroad: bool) -> None:
    request_id = str(settings.get('requestId') or '')
    transfer_id = str(settings.get('transferId') or '')
    with nav_stop_list_lock():
      request = _nav_json_param(NAV_ROUTE_REQUEST_KEY, None)
      if not isinstance(request, dict) or not self._nav_route_transfer_matches_request(settings, request):
        cloudlog.warning(
          f'appbridged navRouteTransfer stale_start request={request_id or "none"} transfer={transfer_id or "none"}'
        )
        return
      if is_offroad or params.get_bool('IsOffroad'):
        self._nav_route_transfer.clear(transfer_id or None)
        params.remove(NAV_ROUTE_REQUEST_KEY)
        params.put(NAV_STATUS_KEY, 'paused_offroad')
        return
      if (settings.get('totalBytes', 0) > NAV_ROUTE_SELECTION_MAX_BYTES
          or settings.get('partCount', 0) > NAV_ROUTE_SELECTION_MAX_PARTS
          or not _nav_validate_candidate_manifest(
            settings.get('candidateSetId'), settings.get('activeRouteId'), settings.get('candidateRoutes')
          )):
        self._fail_nav_route_transfer_locked(request_id, transfer_id, 'selection_bounds')
        return
      request['candidateSetId'] = settings.get('candidateSetId')
      request['activeRouteId'] = settings.get('activeRouteId')
      request['candidateRoutes'] = settings.get('candidateRoutes')
      result = self._nav_route_transfer.start(settings)
      if not result.get('ok'):
        self._fail_nav_route_transfer_locked(request_id, transfer_id, result.get('reason', 'invalid_start'))
        cloudlog.warning(
          f'appbridged navRouteTransfer rejected request={request_id} transfer={transfer_id} '
          f'reason={result.get("reason", "invalid_start")}'
        )
        return
      request['transferStartedAtMonotonic'] = result['startedAt']
      if (updated_at := result.get('updatedAt', result.get('startedAt'))) is not None:
        request['transferUpdatedAtMonotonic'] = updated_at
      params.put(NAV_ROUTE_REQUEST_KEY, json.dumps(request, separators=(',', ':')))
      params.put(NAV_STATUS_KEY, 'receiving_route')
      cloudlog.warning(
        f'appbridged navRouteTransfer start request={request_id} transfer={transfer_id} '
        f'bytes={result["totalBytes"]} parts={result["partCount"]} duplicate={int(bool(result.get("duplicate")))}'
      )

  def _apply_nav_route_response_part(self, settings: dict, is_offroad: bool) -> None:
    request_id = str(settings.get('requestId') or '')
    transfer_id = str(settings.get('transferId') or '')
    route_bytes = None
    completed = None
    with nav_stop_list_lock():
      request = _nav_json_param(NAV_ROUTE_REQUEST_KEY, None)
      if not isinstance(request, dict) or not self._nav_route_transfer_matches_request(settings, request):
        cloudlog.warning(
          f'appbridged navRouteTransfer stale_part request={request_id or "none"} transfer={transfer_id or "none"}'
        )
        return
      if is_offroad or params.get_bool('IsOffroad'):
        self._nav_route_transfer.clear(transfer_id or None)
        params.remove(NAV_ROUTE_REQUEST_KEY)
        params.put(NAV_STATUS_KEY, 'paused_offroad')
        return
      result = self._nav_route_transfer.add_part(settings)
      if not result.get('ok'):
        reason = result.get('reason', 'invalid_part')
        rate = result.get('rateBytesPerSecond')
        rate_text = f' rate={rate}Bps' if type(rate) is int else ''
        if reason != 'stale_transfer':
          self._fail_nav_route_transfer_locked(request_id, transfer_id, reason)
        cloudlog.warning(
          f'appbridged navRouteTransfer rejected_part request={request_id} transfer={transfer_id} '
          f'part={settings.get("partIndex", -1)} reason={reason}{rate_text}'
        )
        return
      if not result.get('duplicate') and not result.get('complete'):
        updated_at = result.get('updatedAt')
        if updated_at is not None:
          request['transferUpdatedAtMonotonic'] = updated_at
          params.put(NAV_ROUTE_REQUEST_KEY, json.dumps(request, separators=(',', ':')))
      if result.get('complete'):
        route_bytes = result['routeBytes']
        completed = {
          'requestId': request_id,
          'transferId': transfer_id,
          'sessionId': str(request.get('sessionId') or 'legacy'),
          'routeGeneration': request.get('routeGeneration'),
          'destKey': _nav_dest_key_from_params(params),
          'stopVersion': request.get('stopVersion'),
          'candidateSetId': request.get('candidateSetId'),
          'activeRouteId': request.get('activeRouteId'),
          'candidateRoutes': request.get('candidateRoutes'),
          'receivedParts': result['receivedParts'],
          'totalBytes': result['totalBytes'],
        }
      elif result['receivedParts'] == 1 or result['receivedParts'] % 8 == 0:
        cloudlog.warning(
          f'appbridged navRouteTransfer progress request={request_id} transfer={transfer_id} '
          f'parts={result["receivedParts"]}/{result["partCount"]} '
          f'bytes={result["receivedBytes"]}/{result["totalBytes"]}'
        )

    if route_bytes is None or completed is None:
      return
    cloudlog.warning(
      f'appbridged navRouteTransfer complete request={request_id} transfer={transfer_id} '
      f'parts={completed["receivedParts"]} bytes={completed["totalBytes"]} '
      f'dest={completed["destKey"]} stop_version={completed.get("stopVersion")} '
      f'candidate_set={str(completed.get("candidateSetId") or "")[:36]}'
    )
    self._apply_nav_route_response({**completed, 'routePayloadUtf8': route_bytes}, is_offroad)

  def _apply_nav_route_response_abort(self, settings, is_offroad):
    request_id = str(settings.get('requestId') or '')
    transfer_id = str(settings.get('transferId') or '')
    with nav_stop_list_lock():
      request = _nav_json_param(NAV_ROUTE_REQUEST_KEY, None)
      if not isinstance(request, dict) or not self._nav_route_transfer_matches_request(settings, request):
        cloudlog.warning(f'appbridged navRouteTransfer stale_abort request={request_id or "none"}')
        return
      progress = self._nav_route_transfer.progress()
      if not progress.get('ok') or progress.get('transferId') != transfer_id:
        cloudlog.warning(f'appbridged navRouteTransfer stale_abort request={request_id} transfer={transfer_id}')
        return
      if is_offroad or params.get_bool('IsOffroad'):
        self._nav_route_transfer.clear(transfer_id)
        params.remove(NAV_ROUTE_REQUEST_KEY)
        params.put(NAV_STATUS_KEY, 'paused_offroad')
        return
      reason = str(settings.get('reason') or 'transfer_aborted')[:80]
      self._fail_nav_route_transfer_locked(request_id, transfer_id, reason)
    cloudlog.warning(
      f'appbridged navRouteTransfer aborted request={request_id} transfer={transfer_id} reason={reason}'
    )

  def _send_nav_ble_diag_ack(self, test_id, sequence, accepted, reason, is_offroad):
    ack = {
      'testId': test_id,
      'sequence': sequence,
      'accepted': accepted,
      'reason': reason,
      'offroad': is_offroad,
    }
    try:
      self.ble.chunk_and_send(CHANNEL_SETTINGS, msgpack.packb({'navBleDiagAck': ack}))
    except Exception as e:
      cloudlog.error(f'appbridged nav BLE diagnostic ACK send failed test={test_id} seq={sequence} error={e}')

  def _nav_ble_diag_fake_position(self, elapsed):
    route = self._nav_ble_diag_route
    if self._nav_ble_diag_stop_index == 1 and route is not None and route.geometry:
      distances = route.geometry_cumulative_distances
      if distances and len(distances) == len(route.geometry):
        distance = min(
          distances[-1],
          max(0.0, elapsed - self._nav_ble_diag_route_started_elapsed) * self._nav_ble_diag_route_speed_mps,
        )
        from openpilot.selfdrive.nav.route_engine import bearing_between_two_points
        for index in range(1, len(distances)):
          segment = distances[index] - distances[index - 1]
          if segment > 0.0 and distance <= distances[index]:
            fraction = (distance - distances[index - 1]) / segment
            start = route.geometry[index - 1]
            end = route.geometry[index]
            return (
              start.latitude + (end.latitude - start.latitude) * fraction,
              start.longitude + (end.longitude - start.longitude) * fraction,
              bearing_between_two_points(start, end),
            )
        last = route.geometry[-1]
        bearing = bearing_between_two_points(route.geometry[-2], last) if len(route.geometry) > 1 else None
        return last.latitude, last.longitude, bearing
    if self._nav_ble_diag_fake_gps:
      return (*self._nav_ble_diag_fake_gps, None)
    return 0.0, 0.0, None

  def _reset_nav_ble_diag_route_state(self, status, status_text):
    self._nav_ble_diag_route_transfer.clear()
    self._nav_ble_diag_stop_index = 0
    self._nav_ble_diag_route_stops = []
    self._nav_ble_diag_fake_gps = None
    self._nav_ble_diag_route_speed_mps = 0.0
    self._nav_ble_diag_route_started_elapsed = 0.0
    self._nav_ble_diag_mapbox_interval_sec = 0.0
    self._nav_ble_diag_max_mapbox_probes = 0
    self._nav_ble_diag_request_id = ""
    self._nav_ble_diag_dest_key = ""
    self._nav_ble_diag_route = None
    self._nav_ble_diag_route_accepted = 0
    self._nav_ble_diag_route_rejected = 0
    self._nav_ble_diag_route_too_large = 0
    self._nav_ble_diag_transfer_started = 0
    self._nav_ble_diag_transfer_completed = 0
    self._nav_ble_diag_transfer_failed = 0
    self._nav_ble_diag_transfer_parts = 0
    self._nav_ble_diag_transfer_bytes = 0
    self._nav_ble_diag_envelope_too_large = 0
    self._nav_ble_diag_skip_count = 0
    self._nav_ble_diag_status = status
    self._nav_ble_diag_status_text = status_text
    self._nav_ble_diag_maneuver_type = ""
    self._nav_ble_diag_maneuver_modifier = ""
    self._nav_ble_diag_maneuver_distance = None
    self._nav_ble_diag_maneuver_primary_text = ""
    self._nav_ble_diag_maneuver_secondary_text = ""
    self._nav_ble_diag_maneuver_lanes = []

  def _send_nav_ble_diag_scenario_state(self):
    test_id = self._nav_ble_diag_test_id
    if not test_id or len(self._nav_ble_diag_route_stops) != 2:
      return
    transfer = self._nav_ble_diag_route_transfer.progress()
    elapsed = max(0.0, monotonic() - self._nav_ble_diag_started_at)
    latitude, longitude, bearing = self._nav_ble_diag_fake_position(elapsed)
    stop_index = min(self._nav_ble_diag_stop_index, len(self._nav_ble_diag_route_stops) - 1)
    stop_name = self._nav_ble_diag_route_stops[stop_index][0]
    distance_remaining = None
    if self._nav_ble_diag_route is not None:
      try:
        from openpilot.selfdrive.nav.route_engine import Coordinate
        progress = self._nav_ble_diag_route.get_progress(Coordinate(latitude, longitude))
        if progress is not None and math.isfinite(progress.distance_remaining):
          distance_remaining = max(0.0, float(progress.distance_remaining))
      except (TypeError, ValueError, OverflowError):
        pass
    fake_gps = {'latitude': latitude, 'longitude': longitude}
    if bearing is not None:
      fake_gps['bearing'] = bearing
    state = {
      'testId': test_id,
      'stopIndex': stop_index,
      'stopName': stop_name,
      'requestId': self._nav_ble_diag_request_id,
      'destKey': self._nav_ble_diag_dest_key,
      'candidateSetId': (self._nav_ble_diag_candidate_context or {}).get('candidateSetId', ''),
      'activeRouteId': (self._nav_ble_diag_candidate_context or {}).get('activeRouteId', ''),
      'status': self._nav_ble_diag_status,
      'statusText': self._nav_ble_diag_status_text,
      'maneuverType': self._nav_ble_diag_maneuver_type,
      'maneuverModifier': self._nav_ble_diag_maneuver_modifier,
      'skipAvailable': stop_index == 0,
      'routeAccepted': self._nav_ble_diag_route_accepted,
      'routeRejected': self._nav_ble_diag_route_rejected,
      'routeTooLarge': self._nav_ble_diag_route_too_large,
      'routeTransferStarted': self._nav_ble_diag_transfer_started,
      'routeTransferCompleted': self._nav_ble_diag_transfer_completed,
      'routeTransferFailed': self._nav_ble_diag_transfer_failed,
      'routeTransferParts': self._nav_ble_diag_transfer_parts,
      'routeTransferBytes': self._nav_ble_diag_transfer_bytes,
      'routeTransferPartsReceived': transfer.get('receivedParts', 0),
      'routeTransferPartCount': transfer.get('partCount', 0),
      'routeTransferBytesReceived': transfer.get('receivedBytes', 0),
      'routeTransferTotalBytes': transfer.get('totalBytes', 0),
      'diagnosticEnvelopeTooLarge': self._nav_ble_diag_envelope_too_large,
      'skipCount': self._nav_ble_diag_skip_count,
      'elapsedSec': int(elapsed),
      'fakeGps': fake_gps,
    }
    if self._nav_ble_diag_maneuver_distance is not None:
      state['maneuverDistance'] = self._nav_ble_diag_maneuver_distance
    if self._nav_ble_diag_maneuver_primary_text:
      state['maneuverPrimaryText'] = self._nav_ble_diag_maneuver_primary_text
    if self._nav_ble_diag_maneuver_secondary_text:
      state['maneuverSecondaryText'] = self._nav_ble_diag_maneuver_secondary_text
    if self._nav_ble_diag_maneuver_lanes:
      state['lanes'] = self._nav_ble_diag_maneuver_lanes
    if distance_remaining is not None:
      state['distanceRemainingM'] = distance_remaining
    try:
      self.ble.chunk_and_send(CHANNEL_SETTINGS, msgpack.packb({'navBleDiagScenarioState': state}))
    except Exception as e:
      cloudlog.error(f'appbridged nav BLE replay state send failed test={test_id} error={type(e).__name__}')

  def _set_nav_ble_diag_status(self, status, status_text, maneuver_type="", maneuver_modifier="",
                               maneuver_distance=None, maneuver_primary_text="",
                               maneuver_secondary_text="", lanes=None):
    self._nav_ble_diag_status = str(status)[:40]
    self._nav_ble_diag_status_text = str(status_text).replace('\n', ' ')[:120]
    self._nav_ble_diag_maneuver_type = str(maneuver_type or "")[:24]
    self._nav_ble_diag_maneuver_modifier = str(maneuver_modifier or "")[:24]
    self._nav_ble_diag_maneuver_distance = (
      float(maneuver_distance) if maneuver_distance is not None and math.isfinite(maneuver_distance) and maneuver_distance >= 0 else None
    )
    self._nav_ble_diag_maneuver_primary_text = str(maneuver_primary_text or "")[:48]
    self._nav_ble_diag_maneuver_secondary_text = str(maneuver_secondary_text or "")[:48]
    self._nav_ble_diag_maneuver_lanes = [[
      [str(direction) for direction in lane.get("directions", [])[:5]],
      bool(lane.get("active")), str(lane.get("activeDirection", "none")),
    ] for lane in (lanes or [])[:8] if isinstance(lane, dict)]
    cloudlog.warning(
      f'appbridged nav BLE replay state test={self._nav_ble_diag_test_id} '
      f'stop={self._nav_ble_diag_stop_index} status={self._nav_ble_diag_status} '
      f'request={self._nav_ble_diag_request_id or "none"} dest={self._nav_ble_diag_dest_key or "none"} '
      f'text={self._nav_ble_diag_status_text}'
    )
    self._send_nav_ble_diag_scenario_state()

  def _update_nav_ble_diag_progress(self, now):
    if self._nav_ble_diag_status == 'arrived':
      return
    if self._nav_ble_diag_route is None:
      return
    elapsed = max(0.0, now - self._nav_ble_diag_started_at)
    latitude, longitude, _ = self._nav_ble_diag_fake_position(elapsed)
    try:
      from openpilot.selfdrive.nav.route_engine import Coordinate
      progress = self._nav_ble_diag_route.get_progress(Coordinate(latitude, longitude))
    except Exception as e:
      cloudlog.warning(
        f'appbridged nav BLE replay progress failed test={self._nav_ble_diag_test_id} '
        f'error={type(e).__name__}'
      )
      self._set_nav_ble_diag_status('progress_error', 'Route progress unavailable')
      return
    if progress is None:
      self._set_nav_ble_diag_status('route_unavailable', 'Route progress unavailable')
      return
    if self._nav_ble_diag_stop_index == 1 and self._nav_ble_diag_route.arrived(progress):
      stop_name = self._nav_ble_diag_route_stops[1][0]
      self._set_nav_ble_diag_status('arrived', f'{stop_name}: You have arrived at your destination')
      cloudlog.warning(
        f'appbridged nav BLE replay final arrival test={self._nav_ble_diag_test_id} '
        f'fakeGps={latitude:.5f},{longitude:.5f} distanceRemaining={progress.distance_remaining:.1f}'
      )
      return
    step = progress.current_step
    nav_instruction = self._nav_ble_diag_route.build_instruction_payload(progress)
    instruction = str(step.instruction or step.maneuver or 'Continue').replace('\n', ' ')[:72]
    self._set_nav_ble_diag_status(
      'route_progress',
      f'{self._nav_ble_diag_route_stops[self._nav_ble_diag_stop_index][0]}: {instruction} '
      f'({int(progress.distance_to_end_of_step)} m to step)',
      maneuver_type=nav_instruction['maneuverType'],
      maneuver_modifier=nav_instruction['maneuverModifier'],
      maneuver_distance=nav_instruction['maneuverDistance'],
      maneuver_primary_text=nav_instruction['maneuverPrimaryText'],
      maneuver_secondary_text=nav_instruction['maneuverSecondaryText'],
      lanes=nav_instruction['lanes'],
    )
    cloudlog.warning(
      f'appbridged nav BLE replay progress test={self._nav_ble_diag_test_id} '
      f'stop={self._nav_ble_diag_stop_index} fakeGps={latitude:.5f},{longitude:.5f} '
      f'step={progress.current_step_index} distanceToStep={progress.distance_to_end_of_step:.1f} '
      f'distanceRemaining={progress.distance_remaining:.1f} offRoute={progress.distance_from_route:.1f}'
    )

  def _apply_nav_ble_diag_probe(self, settings):
    test_id = settings.get('testId')
    sequence = settings.get('sequence')
    if not isinstance(test_id, str) or not 1 <= len(test_id) <= 40 or not test_id.isascii() or not test_id.isalnum():
      cloudlog.warning('appbridged nav BLE diagnostic ignored invalid test id')
      return

    now = monotonic()
    if now - self._nav_ble_diag_last_probe_at < NAV_BLE_DIAG_MIN_INTERVAL_SEC:
      if now - self._nav_ble_diag_last_rate_log_at >= 5.0:
        cloudlog.warning(f'appbridged nav BLE diagnostic rate limited test={test_id} seq={sequence}')
        self._nav_ble_diag_last_rate_log_at = now
      return
    self._nav_ble_diag_last_probe_at = now

    if type(sequence) is not int or not 1 <= sequence <= NAV_BLE_DIAG_MAX_PROBES:
      is_offroad = params.get_bool('IsOffroad')
      cloudlog.warning(f'appbridged nav BLE diagnostic rejected test={test_id} seq=invalid reason=invalid_sequence')
      self._send_nav_ble_diag_ack(test_id, sequence if type(sequence) is int else 0, False, 'invalid_sequence', is_offroad)
      return

    is_offroad = params.get_bool('IsOffroad')
    fake_gps = settings.get('fakeGps')
    latitude = fake_gps.get('latitude') if isinstance(fake_gps, dict) else None
    longitude = fake_gps.get('longitude') if isinstance(fake_gps, dict) else None
    valid_fake_gps = (
      type(latitude) in (int, float) and math.isfinite(latitude) and abs(latitude) <= 90.0
      and type(longitude) in (int, float) and math.isfinite(longitude) and abs(longitude) <= 180.0
    )
    accepted = False
    reason = ''
    duplicate = False
    if not is_offroad:
      reason = 'onroad'
      self._nav_ble_diag_test_id = ''
      self._nav_ble_diag_last_sequence = 0
      self._nav_ble_diag_probe_count = 0
      self._nav_ble_diag_sequence_gaps = 0
      self._nav_ble_diag_started_at = 0.0
      self._nav_ble_diag_mapbox_last_slot = 0
      self._nav_ble_diag_mapbox_pending_sequence = 0
      self._nav_ble_diag_mapbox_pending_at = 0.0
      self._reset_nav_ble_diag_route_state('idle', 'No replay')
    elif not valid_fake_gps:
      reason = 'invalid_fake_gps'
    else:
      if self._nav_ble_diag_test_id != test_id:
        scenario = settings.get('scenarioConfig')
        stops = scenario.get('stops') if isinstance(scenario, dict) else None
        speed = scenario.get('routeSpeedMps') if isinstance(scenario, dict) else None
        interval = scenario.get('mapboxIntervalSec') if isinstance(scenario, dict) else None
        max_route_probes = scenario.get('maxRouteProbes') if isinstance(scenario, dict) else None
        parsed_stops = []
        if isinstance(stops, list) and len(stops) == 2:
          for stop in stops:
            if not isinstance(stop, dict):
              break
            name = stop.get('name')
            stop_lat = stop.get('latitude')
            stop_lon = stop.get('longitude')
            if (not isinstance(name, str) or not name.strip() or len(name) > 60
                or type(stop_lat) not in (int, float) or not math.isfinite(stop_lat) or abs(stop_lat) > 90.0
                or type(stop_lon) not in (int, float) or not math.isfinite(stop_lon) or abs(stop_lon) > 180.0):
              break
            parsed_stops.append((name.strip(), float(stop_lat), float(stop_lon)))
        if (len(parsed_stops) != 2 or type(speed) not in (int, float)
            or not math.isfinite(speed) or not 0.0 < speed <= 100.0
            or type(interval) not in (int, float) or not math.isfinite(interval) or not 1.0 <= interval <= 300.0
            or type(max_route_probes) is not int or not 1 <= max_route_probes <= NAV_BLE_DIAG_MAX_PROBES):
          reason = 'invalid_scenario'
        else:
          self._nav_ble_diag_test_id = test_id
          self._nav_ble_diag_last_sequence = 0
          self._nav_ble_diag_probe_count = 0
          self._nav_ble_diag_sequence_gaps = 0
          self._nav_ble_diag_started_at = now
          self._nav_ble_diag_mapbox_last_slot = 0
          self._nav_ble_diag_mapbox_pending_sequence = 0
          self._nav_ble_diag_mapbox_pending_at = 0.0
          self._reset_nav_ble_diag_route_state('waiting_for_route', f'Waiting for directions to {parsed_stops[0][0]}')
          self._nav_ble_diag_route_stops = parsed_stops
          self._nav_ble_diag_route_speed_mps = float(speed)
          self._nav_ble_diag_mapbox_interval_sec = float(interval)
          self._nav_ble_diag_max_mapbox_probes = max_route_probes
      if not reason:
        if now - self._nav_ble_diag_started_at > NAV_BLE_DIAG_MAX_AGE_SEC:
          reason = 'expired'
        elif sequence == self._nav_ble_diag_last_sequence:
          accepted = True
          duplicate = True
          reason = 'duplicate'
        elif sequence < self._nav_ble_diag_last_sequence:
          reason = 'stale_sequence'
        elif self._nav_ble_diag_probe_count >= NAV_BLE_DIAG_MAX_PROBES:
          reason = 'probe_limit'
        else:
          self._nav_ble_diag_sequence_gaps += sequence - self._nav_ble_diag_last_sequence - 1
          self._nav_ble_diag_last_sequence = sequence
          self._nav_ble_diag_probe_count += 1
          accepted = True
    if accepted and not duplicate:
      self._nav_ble_diag_fake_gps = (float(latitude), float(longitude))
    if accepted:
      cloudlog.warning(
        f'appbridged nav BLE diagnostic probe test={test_id} seq={sequence} accepted=1 duplicate={int(duplicate)} '
        f'probes={self._nav_ble_diag_probe_count} gaps={self._nav_ble_diag_sequence_gaps} offroad={int(is_offroad)}'
      )
    else:
      cloudlog.warning(
        f'appbridged nav BLE diagnostic probe test={test_id} seq={sequence} accepted=0 '
        f'reason={reason} offroad={int(is_offroad)}'
      )
    self._send_nav_ble_diag_ack(test_id, sequence, accepted, reason, is_offroad)
    if accepted and not duplicate and is_offroad:
      self._maybe_send_nav_ble_diag_mapbox_probe(test_id, now)
      self._update_nav_ble_diag_progress(now)

  def _send_nav_ble_diag_mapbox_probe(self, probe):
    try:
      self.ble.chunk_and_send(CHANNEL_SETTINGS, msgpack.packb({'navBleDiagMapboxProbe': probe}))
    except Exception as e:
      cloudlog.error(
        f"appbridged nav BLE diagnostic Mapbox probe send failed test={probe.get('testId', '')} "
        f"seq={probe.get('sequence', 0)} error={e}"
      )

  def _maybe_send_nav_ble_diag_mapbox_probe(self, test_id, now):
    elapsed = now - self._nav_ble_diag_started_at
    if self._nav_ble_diag_mapbox_interval_sec <= 0.0 or self._nav_ble_diag_max_mapbox_probes <= 0:
      return
    sequence = (
      self._nav_ble_diag_mapbox_last_slot + 1
      if self._nav_ble_diag_stop_index == 1 and self._nav_ble_diag_status == 'advancing_stop'
      else int(elapsed // self._nav_ble_diag_mapbox_interval_sec) + 1
    )
    if sequence > self._nav_ble_diag_max_mapbox_probes or sequence <= self._nav_ble_diag_mapbox_last_slot:
      return

    if len(self._nav_ble_diag_route_stops) != 2:
      return
    stop_index = self._nav_ble_diag_stop_index
    stop_name, dest_lat, dest_lon = self._nav_ble_diag_route_stops[stop_index]
    if self._nav_ble_diag_route is not None:
      cloudlog.warning(
        f'appbridged nav BLE replay request skipped test={test_id} slot={sequence} '
        f'reason=route_loaded stop={stop_index}'
      )
      return

    pending = self._nav_ble_diag_mapbox_pending_sequence
    if pending:
      pending_age = now - self._nav_ble_diag_mapbox_pending_at
      transfer_active = self._nav_ble_diag_route_transfer.progress().get('ok')
      pending_timeout = ROUTE_TRANSFER_IDLE_TIMEOUT_SECONDS if transfer_active else NAV_BLE_DIAG_MAPBOX_TIMEOUT_SEC
      if pending_age < pending_timeout:
        cloudlog.warning(
          f'appbridged nav BLE replay request skipped test={test_id} slot={sequence} '
          f'reason=request_inflight pending={pending} age={pending_age:.1f} timeout={pending_timeout:.1f}'
        )
        return
      cloudlog.warning(
        f'appbridged nav BLE replay request expired test={test_id} seq={pending} age={pending_age:.1f}'
      )
      if transfer_active:
        self._nav_ble_diag_route_transfer.clear()
        self._nav_ble_diag_transfer_failed += 1
        self._set_nav_ble_diag_status('transfer_unavailable', 'Directions transfer was interrupted')
      self._nav_ble_diag_mapbox_pending_sequence = 0
      self._nav_ble_diag_mapbox_pending_at = 0.0

    latitude, longitude, _ = self._nav_ble_diag_fake_position(elapsed)
    request_id = f'{test_id}-{sequence}-{stop_index}'
    dest_key = f'{dest_lat:.5f}|{dest_lon:.5f}'
    probe = {
      'testId': test_id,
      'sequence': sequence,
      'scenario': 'route_replay',
      'offroad': True,
      'requestId': request_id,
      'stopIndex': stop_index,
      'expectedDestKey': dest_key,
      'start': {'latitude': latitude, 'longitude': longitude},
      'destination': {
        'name': stop_name, 'place_name': stop_name,
        'latitude': dest_lat, 'longitude': dest_lon,
      },
      'bearing': None,
    }
    self._nav_ble_diag_request_id = request_id
    self._nav_ble_diag_dest_key = dest_key
    self._nav_ble_diag_mapbox_pending_sequence = sequence
    self._nav_ble_diag_mapbox_pending_at = now
    self._set_nav_ble_diag_status('requesting_directions', f'Getting directions to {stop_name}')
    cloudlog.warning(
      f'appbridged nav BLE replay request test={test_id} seq={sequence} stop={stop_index} '
      f'name={stop_name} request={request_id} dest={dest_key} fakeGps={latitude:.5f},{longitude:.5f}'
    )
    self._nav_ble_diag_mapbox_last_slot = sequence
    self._send_nav_ble_diag_mapbox_probe(probe)

  def _apply_nav_ble_diag_skip_stop(self, settings, is_offroad):
    test_id = settings.get('testId')
    stop_index = settings.get('stopIndex')
    now = monotonic()
    accepted = False
    reason = 'invalid_request'
    duplicate = False

    if not isinstance(test_id, str) or test_id != self._nav_ble_diag_test_id:
      reason = 'stale_test'
    elif not is_offroad or not params.get_bool('IsOffroad'):
      reason = 'onroad'
    elif now - self._nav_ble_diag_started_at > NAV_BLE_DIAG_MAX_AGE_SEC:
      reason = 'expired'
    elif type(stop_index) is not int or stop_index != 0:
      reason = 'invalid_stop'
    elif self._nav_ble_diag_stop_index == 1:
      accepted = True
      duplicate = True
      reason = 'duplicate'
    elif self._nav_ble_diag_stop_index != 0:
      reason = 'stale_stop'
    else:
      self._nav_ble_diag_stop_index = 1
      self._nav_ble_diag_request_id = ''
      self._nav_ble_diag_route_transfer.clear()
      self._nav_ble_diag_dest_key = ''
      self._nav_ble_diag_route = None
      self._nav_ble_diag_route_started_elapsed = 0.0
      self._nav_ble_diag_mapbox_pending_sequence = 0
      self._nav_ble_diag_mapbox_pending_at = 0.0
      self._nav_ble_diag_skip_count += 1
      stop_name = self._nav_ble_diag_route_stops[1][0]
      self._set_nav_ble_diag_status('advancing_stop', f'Advancing to {stop_name}')
      self._maybe_send_nav_ble_diag_mapbox_probe(test_id, now)
      accepted = True
      reason = 'skipped'

    ack = {
      'testId': str(test_id or '')[:40],
      'accepted': accepted,
      'reason': reason,
      'duplicate': duplicate,
      'stopIndex': self._nav_ble_diag_stop_index,
      'offroad': is_offroad,
    }
    try:
      self.ble.chunk_and_send(CHANNEL_SETTINGS, msgpack.packb({'navBleDiagScenarioSkipAck': ack}))
    except Exception as e:
      cloudlog.error(f'appbridged nav BLE replay skip ACK send failed test={test_id} error={type(e).__name__}')
    cloudlog.warning(
      f'appbridged nav BLE replay skip test={test_id} accepted={int(accepted)} '
      f'duplicate={int(duplicate)} reason={reason} stop={self._nav_ble_diag_stop_index}'
    )

  def _nav_ble_diag_transfer_context_error(self, settings, is_offroad):
    test_id = settings.get('testId')
    sequence = settings.get('sequence')
    if (not isinstance(test_id, str) or test_id != self._nav_ble_diag_test_id
        or type(sequence) is not int or not 1 <= sequence <= self._nav_ble_diag_max_mapbox_probes):
      return 'stale_test_or_sequence'
    if not is_offroad or not params.get_bool('IsOffroad'):
      return 'onroad'
    if monotonic() - self._nav_ble_diag_started_at > NAV_BLE_DIAG_MAX_AGE_SEC:
      return 'stale_test'
    if sequence != self._nav_ble_diag_mapbox_pending_sequence:
      return 'stale_sequence'
    if settings.get('requestId') != self._nav_ble_diag_request_id:
      return 'stale_request'
    if settings.get('destKey') != self._nav_ble_diag_dest_key:
      return 'stale_destination'
    timeout = ROUTE_TRANSFER_IDLE_TIMEOUT_SECONDS if self._nav_ble_diag_route_transfer.progress().get('ok') else NAV_BLE_DIAG_MAPBOX_TIMEOUT_SEC
    if monotonic() - self._nav_ble_diag_mapbox_pending_at > timeout:
      return 'response_timeout'
    if (settings.get('sessionId') != test_id
        or type(settings.get('routeGeneration')) is not int
        or settings.get('routeGeneration') != sequence
        or not isinstance(settings.get('transferId'), str)
        or not settings.get('transferId')):
      return 'invalid_transfer_identity'
    return ''

  def _fail_nav_ble_diag_route_transfer(self, settings, reason):
    if (settings.get('testId') != self._nav_ble_diag_test_id
        or settings.get('sequence') != self._nav_ble_diag_mapbox_pending_sequence
        or settings.get('requestId') != self._nav_ble_diag_request_id):
      return
    self._nav_ble_diag_route_transfer.clear()
    self._nav_ble_diag_mapbox_pending_sequence = 0
    self._nav_ble_diag_mapbox_pending_at = 0.0
    self._nav_ble_diag_route_rejected += 1
    self._nav_ble_diag_transfer_failed += 1
    self._set_nav_ble_diag_status('transfer_unavailable', 'Directions transfer was interrupted')
    cloudlog.warning(
      f'appbridged nav BLE replay transfer failed test={settings.get("testId")} '
      f'seq={settings.get("sequence")} reason={str(reason or "transfer_failed")[:60]}'
    )

  def _apply_nav_ble_diag_mapbox_result_start(self, settings, is_offroad):
    reason = self._nav_ble_diag_transfer_context_error(settings, is_offroad)
    if reason:
      cloudlog.warning(
        f'appbridged nav BLE replay transfer start ignored test={settings.get("testId")} '
        f'seq={settings.get("sequence")} reason={reason}'
      )
      if reason == 'response_timeout':
        self._nav_ble_diag_route_transfer.clear()
        self._nav_ble_diag_mapbox_pending_sequence = 0
        self._nav_ble_diag_mapbox_pending_at = 0.0
        self._nav_ble_diag_route_rejected += 1
        self._set_nav_ble_diag_status('route_unavailable', 'Directions response timed out')
      return
    if (type(settings.get('totalBytes')) is not int or not 32 <= settings['totalBytes'] <= NAV_ROUTE_SELECTION_MAX_BYTES
        or type(settings.get('partCount')) is not int or not 1 <= settings['partCount'] <= NAV_ROUTE_SELECTION_MAX_PARTS):
      self._fail_nav_ble_diag_route_transfer(settings, 'route_transfer_bounds')
      return
    if not _nav_validate_candidate_manifest(
        settings.get('candidateSetId'), settings.get('activeRouteId'), settings.get('candidateRoutes')):
      self._fail_nav_ble_diag_route_transfer(settings, 'candidate_manifest')
      return
    result = self._nav_ble_diag_route_transfer.start(settings)
    if not result.get('ok'):
      self._fail_nav_ble_diag_route_transfer(settings, result.get('reason'))
      return
    if not result.get('duplicate'):
      self._nav_ble_diag_transfer_started += 1
    self._nav_ble_diag_mapbox_pending_at = monotonic()
    self._set_nav_ble_diag_status(
      'receiving_route',
      f'Receiving directions response (0/{result["partCount"]} parts)'
    )
    cloudlog.warning(
      f'appbridged nav BLE replay transfer start test={settings.get("testId")} '
      f'seq={settings.get("sequence")} request={settings.get("requestId")} '
      f'transfer={settings.get("transferId")} bytes={result["totalBytes"]} '
      f'parts={result["partCount"]} duplicate={int(bool(result.get("duplicate")))}'
    )

  def _apply_nav_ble_diag_mapbox_result_part(self, settings, is_offroad):
    reason = self._nav_ble_diag_transfer_context_error(settings, is_offroad)
    if reason:
      cloudlog.warning(
        f'appbridged nav BLE replay transfer part ignored test={settings.get("testId")} '
        f'seq={settings.get("sequence")} part={settings.get("partIndex", -1)} reason={reason}'
      )
      if reason == 'response_timeout':
        self._nav_ble_diag_route_transfer.clear()
        self._nav_ble_diag_mapbox_pending_sequence = 0
        self._nav_ble_diag_mapbox_pending_at = 0.0
        self._nav_ble_diag_route_rejected += 1
        self._set_nav_ble_diag_status('route_unavailable', 'Directions response timed out')
      return
    result = self._nav_ble_diag_route_transfer.add_part(settings)
    if not result.get('ok'):
      reason = result.get('reason', 'invalid_part')
      rate = result.get('rateBytesPerSecond')
      if reason != 'stale_transfer':
        self._fail_nav_ble_diag_route_transfer(settings, f'{reason}:{rate}Bps' if type(rate) is int else reason)
      else:
        cloudlog.warning(
          f'appbridged nav BLE replay stale transfer part test={settings.get("testId")} '
          f'seq={settings.get("sequence")} transfer={settings.get("transferId")}'
        )
      return
    if not result.get('duplicate'):
      self._nav_ble_diag_transfer_parts += 1
      self._nav_ble_diag_mapbox_pending_at = monotonic()
    complete_settings = None
    if result.get('complete'):
      self._nav_ble_diag_transfer_completed += 1
      self._nav_ble_diag_transfer_bytes += result['totalBytes']
      complete_settings = dict(settings)
      complete_settings['routePayloadUtf8'] = result['routeBytes']
      self._nav_ble_diag_route_transfer.clear()
    elif result.get('receivedParts') == 1 or result.get('receivedParts') % 8 == 0:
      self._set_nav_ble_diag_status(
        'receiving_route',
        f'Receiving directions response ({result["receivedParts"]}/{result["partCount"]} parts)'
      )
    cloudlog.warning(
      f'appbridged nav BLE replay transfer part test={settings.get("testId")} '
      f'seq={settings.get("sequence")} transfer={settings.get("transferId")} '
      f'part={settings.get("partIndex", -1)} received={result.get("receivedParts", 0)}/'
      f'{result.get("partCount", 0)} bytes={result.get("receivedBytes", 0)}/'
      f'{result.get("totalBytes", 0)} complete={int(bool(result.get("complete")))}'
    )
    if complete_settings is not None:
      self._apply_nav_ble_diag_mapbox_result(complete_settings, is_offroad)

  def _send_nav_ble_diag_select_ack(self, settings, status, reason=''):
    ack = {
      key: settings.get(key) for key in
      ('selectionId', 'selectionSequence', 'testId', 'sequence', 'requestId', 'destKey', 'candidateSetId', 'selectedRouteId')
    }
    ack['payloadSha256'] = settings.get('_candidateDigest', '')
    ack['status'] = status
    if reason:
      ack['reason'] = str(reason)[:40]
    try:
      self.ble.chunk_and_send(CHANNEL_SETTINGS, msgpack.packb({'navBleDiagSelectRouteAck': ack}))
    except Exception as e:
      cloudlog.warning(f'appbridged nav BLE replay selection ACK send failed error={type(e).__name__}')

  def _nav_ble_diag_selection_reason(self, settings, is_offroad):
    context = self._nav_ble_diag_candidate_context
    if not isinstance(context, dict) or not is_offroad or not params.get_bool('IsOffroad'):
      return 'stale_context'
    now = monotonic()
    if (type(settings.get('selectionSequence')) is not int or settings['selectionSequence'] < 1
        or settings.get('testId') != context.get('testId') or settings.get('testId') != self._nav_ble_diag_test_id
        or now - self._nav_ble_diag_started_at > NAV_BLE_DIAG_MAX_AGE_SEC
        or settings.get('sequence') != context.get('sequence')
        or settings.get('requestId') != context.get('requestId')
        or settings.get('destKey') != context.get('destKey')
        or settings.get('candidateSetId') != context.get('candidateSetId')
        or (self._nav_ble_diag_mapbox_pending_sequence
            and context.get('sequence') != self._nav_ble_diag_mapbox_pending_sequence)
):
      return 'stale_context'
    if not _nav_validate_candidate_manifest(
        context.get('candidateSetId'), context.get('activeRouteId'), context.get('candidateRoutes')):
      return 'stale_context'
    manifest = next((item for item in context.get('candidateRoutes', [])
                     if isinstance(item, dict) and item.get('routeId') == settings.get('selectedRouteId')), None)
    if manifest is None:
      return 'wrong_route'
    settings['_candidateDigest'] = manifest['payloadSha256']
    settings['_candidateBytes'] = manifest['payloadBytes']
    return ''

  def _commit_nav_ble_diag_route_selection(self, settings, raw):
    reason = self._nav_ble_diag_selection_reason(settings, True)
    if reason:
      self._send_nav_ble_diag_select_ack(settings, 'rejected', reason)
      return
    context = self._nav_ble_diag_candidate_context
    highest_sequence = context.get('selectionSequence', 0)
    if type(highest_sequence) is not int or highest_sequence < 0:
      self._send_nav_ble_diag_select_ack(settings, 'rejected', 'stale_context')
      return
    if isinstance(raw, str):
      raw = raw.encode('utf-8', errors='strict')
    if (not isinstance(raw, bytes) or len(raw) != settings.get('_candidateBytes')
        or hashlib.sha256(raw).hexdigest() != settings.get('_candidateDigest')):
      self._send_nav_ble_diag_select_ack(settings, 'rejected', 'invalid_route')
      return
    if settings['selectionSequence'] <= highest_sequence:
      if (settings['selectionSequence'] == highest_sequence
          and context.get('lastSelectionId') == settings.get('selectionId')
          and context.get('lastSelectedRouteId') == settings.get('selectedRouteId')
          and context.get('lastPayloadSha256') == settings.get('_candidateDigest')):
        self._send_nav_ble_diag_select_ack(settings, 'accepted')
      else:
        self._send_nav_ble_diag_select_ack(settings, 'rejected', 'stale_selection')
      return
    context = self._nav_ble_diag_candidate_context
    manifest = next((item for item in context['candidateRoutes']
                     if item.get('routeId') == settings.get('selectedRouteId')), None)
    try:
      route_text, route_data = _nav_parse_route_payload(
        raw, settings.get('selectedRouteId'), manifest.get('payloadSha256') if manifest else None,
        manifest.get('payloadBytes') if manifest else None
      )
      from openpilot.selfdrive.nav.route_engine import Coordinate, NavigationRoute
      route = NavigationRoute.from_mapbox_route(route_data)
      now = monotonic()
      latitude, longitude, _ = self._nav_ble_diag_fake_position(now - self._nav_ble_diag_started_at)
      progress = route.get_progress(Coordinate(latitude, longitude)) if route else None
      if route is None or progress is None:
        raise ValueError('invalid_route')
    except Exception as e:
      self._send_nav_ble_diag_select_ack(settings, 'rejected', 'invalid_route')
      cloudlog.warning(f'appbridged nav BLE replay selection rejected error={type(e).__name__}')
      return
    reason = self._nav_ble_diag_selection_reason(settings, True)
    if reason:
      self._send_nav_ble_diag_select_ack(settings, 'rejected', reason)
      return
    self._nav_ble_diag_route = route
    context['activeRouteId'] = settings['selectedRouteId']
    context['selectionSequence'] = settings['selectionSequence']
    context['lastSelectionId'] = settings['selectionId']
    context['lastSelectedRouteId'] = settings['selectedRouteId']
    context['lastPayloadSha256'] = settings['_candidateDigest']
    self._nav_ble_diag_route_started_elapsed = max(0.0, now - self._nav_ble_diag_started_at)
    nav_instruction = route.build_instruction_payload(progress)
    instruction = str(nav_instruction['maneuverPrimaryText'] or 'Continue').replace('\n', ' ')[:72]
    self._set_nav_ble_diag_status(
      'route_progress',
      f'{self._nav_ble_diag_route_stops[self._nav_ble_diag_stop_index][0]}: {instruction} '
      f'({int(progress.distance_to_end_of_step)} m to step)',
      maneuver_type=nav_instruction['maneuverType'], maneuver_modifier=nav_instruction['maneuverModifier'],
      maneuver_distance=nav_instruction['maneuverDistance'],
      maneuver_primary_text=nav_instruction['maneuverPrimaryText'],
      maneuver_secondary_text=nav_instruction['maneuverSecondaryText'],
      lanes=nav_instruction['lanes'],
    )
    self._send_nav_ble_diag_select_ack(settings, 'accepted')
    cloudlog.warning(
      f'appbridged nav BLE replay selection activated test={settings.get("testId")} '
      f'route={settings.get("selectedRouteId")} bytes={len(raw)}'
    )

  def _apply_nav_ble_diag_select_route(self, settings, is_offroad):
    reason = self._nav_ble_diag_selection_reason(settings, is_offroad)
    if reason:
      self._send_nav_ble_diag_select_ack(settings, 'rejected', reason)
      return
    self._commit_nav_ble_diag_route_selection(settings, settings.get('routePayloadUtf8'))

  def _apply_nav_ble_diag_select_route_start(self, settings, is_offroad):
    reason = self._nav_ble_diag_selection_reason(settings, is_offroad)
    if reason or type(settings.get('totalBytes')) is not int or not 32 <= settings['totalBytes'] <= NAV_ROUTE_SELECTION_MAX_BYTES \
        or type(settings.get('partCount')) is not int or not 1 <= settings['partCount'] <= NAV_ROUTE_SELECTION_MAX_PARTS:
      self._send_nav_ble_diag_select_ack(settings, 'rejected', reason or 'invalid_route')
      self._nav_ble_diag_select_transfer.clear()
      self._nav_ble_diag_select_context = None
      return
    result = self._nav_ble_diag_select_transfer.start(settings)
    if not result.get('ok'):
      self._send_nav_ble_diag_select_ack(settings, 'rejected', result.get('reason', 'invalid_route'))
      return
    self._nav_ble_diag_select_context = dict(settings)

  def _apply_nav_ble_diag_select_route_part(self, settings, is_offroad):
    reason = self._nav_ble_diag_selection_reason(settings, is_offroad)
    if reason:
      self._send_nav_ble_diag_select_ack(settings, 'rejected', reason)
      self._nav_ble_diag_select_transfer.clear()
      self._nav_ble_diag_select_context = None
      return
    result = self._nav_ble_diag_select_transfer.add_part(settings)
    if not result.get('ok'):
      if result.get('reason') != 'stale_transfer':
        self._send_nav_ble_diag_select_ack(settings, 'rejected', result.get('reason', 'invalid_route'))
        self._nav_ble_diag_select_transfer.clear()
        self._nav_ble_diag_select_context = None
      return
    if result.get('complete'):
      context = self._nav_ble_diag_select_context or settings
      self._nav_ble_diag_select_transfer.clear()
      self._nav_ble_diag_select_context = None
      self._commit_nav_ble_diag_route_selection(context, result['routeBytes'])

  def _apply_nav_ble_diag_mapbox_result_abort(self, settings, is_offroad):
    reason = self._nav_ble_diag_transfer_context_error(settings, is_offroad)
    if reason:
      cloudlog.warning(
        f'appbridged nav BLE transfer abort ignored test={settings.get("testId")} '
        f'seq={settings.get("sequence")} reason={reason}'
      )
      return
    progress = self._nav_ble_diag_route_transfer.progress()
    if not progress.get('ok') or progress.get('transferId') != settings.get('transferId'):
      cloudlog.warning(
        f'appbridged nav BLE transfer stale abort test={settings.get("testId")} '
        f'seq={settings.get("sequence")} transfer={settings.get("transferId")}'
      )
      return
    rate = settings.get('rateBytesPerSecond')
    reason = str(settings.get('reason') or 'transfer_aborted')
    if type(rate) is int:
      reason = f'{reason}:{rate}Bps'
    self._fail_nav_ble_diag_route_transfer(settings, reason)

  def _apply_nav_ble_diag_mapbox_result(self, settings, is_offroad):
    test_id = settings.get('testId')
    sequence = settings.get('sequence')
    request_id = settings.get('requestId')
    dest_key = settings.get('destKey')
    if (not isinstance(test_id, str) or not 1 <= len(test_id) <= 40 or not test_id.isascii()
        or not test_id.isalnum() or type(sequence) is not int
        or not 1 <= sequence <= self._nav_ble_diag_max_mapbox_probes
        or not isinstance(request_id, str) or not 1 <= len(request_id) <= 100
        or not isinstance(dest_key, str) or len(dest_key) > 32):
      cloudlog.warning('appbridged nav BLE replay result ignored reason=invalid_fields')
      return

    now = monotonic()
    reason = ''
    if not is_offroad or not params.get_bool('IsOffroad'):
      reason = 'onroad'
    elif test_id != self._nav_ble_diag_test_id or now - self._nav_ble_diag_started_at > NAV_BLE_DIAG_MAX_AGE_SEC:
      reason = 'stale_test'
    elif sequence != self._nav_ble_diag_mapbox_pending_sequence:
      reason = 'stale_sequence'
    elif request_id != self._nav_ble_diag_request_id:
      reason = 'stale_request'
    elif dest_key != self._nav_ble_diag_dest_key:
      reason = 'stale_destination'
    elif now - self._nav_ble_diag_mapbox_pending_at > NAV_BLE_DIAG_MAPBOX_TIMEOUT_SEC:
      reason = 'response_timeout'

    if reason:
      self._nav_ble_diag_route_rejected += 1
      cloudlog.warning(
        f'appbridged nav BLE replay result rejected test={test_id} seq={sequence} '
        f'request={request_id} dest={dest_key} reason={reason}'
      )
      if reason == 'response_timeout' and sequence == self._nav_ble_diag_mapbox_pending_sequence:
        self._nav_ble_diag_route_transfer.clear()
        self._nav_ble_diag_mapbox_pending_sequence = 0
        self._nav_ble_diag_mapbox_pending_at = 0.0
        self._set_nav_ble_diag_status('route_unavailable', 'Directions response timed out')
      return

    self._nav_ble_diag_route_transfer.clear()
    self._nav_ble_diag_mapbox_pending_sequence = 0
    self._nav_ble_diag_mapbox_pending_at = 0.0
    duration_ms = settings.get('durationMs')
    points = settings.get('points')
    steps = settings.get('steps')
    duration_ms = duration_ms if type(duration_ms) is int and 0 <= duration_ms <= 60000 else -1
    points = points if type(points) is int and 0 <= points <= 100000 else 0
    steps = steps if type(steps) is int and 0 <= steps <= 10000 else 0
    raw_segments = settings.get('productionRawSegments')
    trimmed_segments = settings.get('productionTrimmedSegments')
    diagnostic_segments = settings.get('diagnosticSegments')
    raw_segments = raw_segments if type(raw_segments) is int and 0 <= raw_segments <= 100000 else -1
    trimmed_segments = trimmed_segments if type(trimmed_segments) is int and 0 <= trimmed_segments <= 100000 else -1
    diagnostic_segments = diagnostic_segments if type(diagnostic_segments) is int and 0 <= diagnostic_segments <= 100000 else -1
    error = str(settings.get('error') or '').replace('\n', ' ')[:80]

    if settings.get('ok') is not True:
      self._nav_ble_diag_route_rejected += 1
      if error == 'route_too_large':
        self._nav_ble_diag_route_too_large += 1
        status, status_text = 'route_unavailable', 'Route too large for Bluetooth'
      elif error == 'diagnostic_payload_too_large':
        self._nav_ble_diag_envelope_too_large += 1
        status, status_text = 'transfer_unavailable', 'Diagnostic response too large for Bluetooth'
      elif error in ('route_transfer_failed', 'route_transfer_unavailable', 'route_transfer_unsupported'):
        self._nav_ble_diag_transfer_failed += 1
        status, status_text = 'transfer_unavailable', 'Directions transfer unavailable'
      else:
        status, status_text = 'route_unavailable', f'Directions unavailable: {error or "no_route"}'
      self._set_nav_ble_diag_status(status, status_text)
      cloudlog.warning(
        f'appbridged nav BLE replay result test={test_id} seq={sequence} ok=0 '
        f'request={request_id} dest={dest_key} duration_ms={duration_ms} error={error or "none"} '
        f'rawSegments={raw_segments} trimmedSegments={trimmed_segments} diagnosticSegments={diagnostic_segments}'
      )
      return

    route_bytes = settings.get('routePayloadUtf8')
    candidate_set_id = settings.get('candidateSetId')
    active_route_id = settings.get('activeRouteId')
    candidate_routes = settings.get('candidateRoutes')
    primary = next((entry for entry in candidate_routes if isinstance(entry, dict)
                   and entry.get('routeId') == active_route_id), None) if isinstance(candidate_routes, list) else None
    try:
      if not _nav_validate_candidate_manifest(candidate_set_id, active_route_id, candidate_routes):
        raise ValueError('invalid_manifest')
      if not isinstance(primary, dict):
        raise ValueError('invalid_manifest')
      if isinstance(route_bytes, str):
        route_bytes = route_bytes.encode('utf-8', errors='strict')
      route_text, route_data = _nav_parse_route_payload(
        route_bytes, active_route_id, primary.get('payloadSha256'), primary.get('payloadBytes')
      )
      from openpilot.selfdrive.nav.route_engine import Coordinate, NavigationRoute
      route = NavigationRoute.from_mapbox_route(route_data)
      latitude, longitude, _ = self._nav_ble_diag_fake_position(now - self._nav_ble_diag_started_at)
      progress = route.get_progress(Coordinate(latitude, longitude)) if route else None
    except Exception as e:
      route = None
      progress = None
      cloudlog.warning(f'appbridged nav BLE replay route parse failed test={test_id} error={type(e).__name__}')

    if route is None or progress is None:
      self._nav_ble_diag_route_rejected += 1
      self._set_nav_ble_diag_status('route_unavailable', 'Directions received but route parsing failed')
      cloudlog.warning(
        f'appbridged nav BLE replay result rejected test={test_id} seq={sequence} '
        f'request={request_id} dest={dest_key} reason=invalid_route'
      )
      return

    self._nav_ble_diag_route = route
    self._nav_ble_diag_candidate_context = {
      'testId': test_id, 'sequence': sequence, 'requestId': request_id, 'destKey': dest_key,
      'candidateSetId': candidate_set_id, 'candidateRoutes': candidate_routes,
      'activeRouteId': active_route_id, 'selectionSequence': 0,
      'lastSelectionId': '', 'lastSelectedRouteId': '', 'lastPayloadSha256': '',
    }
    self._nav_ble_diag_route_accepted += 1
    self._nav_ble_diag_route_started_elapsed = max(0.0, now - self._nav_ble_diag_started_at)
    nav_instruction = self._nav_ble_diag_route.build_instruction_payload(progress)
    instruction = str(nav_instruction['maneuverPrimaryText'] or 'Continue').replace('\n', ' ')[:72]
    self._set_nav_ble_diag_status(
      'route_progress',
      f'{self._nav_ble_diag_route_stops[self._nav_ble_diag_stop_index][0]}: {instruction} '
      f'({int(progress.distance_to_end_of_step)} m to step)',
      maneuver_type=nav_instruction['maneuverType'], maneuver_modifier=nav_instruction['maneuverModifier'],
      maneuver_distance=nav_instruction['maneuverDistance'],
      maneuver_primary_text=nav_instruction['maneuverPrimaryText'],
      maneuver_secondary_text=nav_instruction['maneuverSecondaryText'],
      lanes=nav_instruction['lanes'],
    )
    cloudlog.warning(
      f'appbridged nav BLE replay result accepted test={test_id} seq={sequence} '
      f'request={request_id} dest={dest_key} points={points} steps={steps} duration_ms={duration_ms} '
      f'rawSegments={raw_segments} trimmedSegments={trimmed_segments} diagnosticSegments={diagnostic_segments} '
      f'fakeGps={latitude:.5f},{longitude:.5f} step={progress.current_step_index} '
      f'distanceToStep={progress.distance_to_end_of_step:.1f} distanceRemaining={progress.distance_remaining:.1f} '
      f'offRoute={progress.distance_from_route:.1f}'
    )


  def _apply_nav_road_match(self, settings: dict) -> None:
    try:
      timestamp = settings.get('sourceTimestamp')
      raw_lat = settings.get('rawLatitude')
      raw_lon = settings.get('rawLongitude')
      latitude = settings.get('latitude')
      longitude = settings.get('longitude')
      confidence = settings.get('confidence')
      distance = settings.get('distanceM')
      if (type(timestamp) is not int or timestamp <= 0
          or any(type(value) not in (int, float) or not math.isfinite(value)
                 for value in (raw_lat, raw_lon, latitude, longitude, confidence, distance))
          or abs(raw_lat) > 90 or abs(latitude) > 90
          or abs(raw_lon) > 180 or abs(longitude) > 180
          or not 0.8 <= confidence <= 1.0 or not 0.0 <= distance <= 35.0):
        return
      match = {
        'sourceTimestamp': timestamp,
        'rawLatitude': float(raw_lat), 'rawLongitude': float(raw_lon),
        'latitude': float(latitude), 'longitude': float(longitude),
        'confidence': float(confidence), 'distanceM': float(distance),
      }
      params.put_nonblocking('NavRoadMatch', json.dumps(match, separators=(',', ':')))
    except (TypeError, ValueError, OverflowError):
      return

  def apply_settings_message(self, message, state, cur_time, is_offroad):
    """Apply a valid assembled settings message immediately."""
    msg_type = ''
    try:
      c, settings = message
      if c != CHANNEL_SETTINGS:
        return
      msg_type = settings.pop('msgType', None)
      match msg_type:
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
        case 'navAddStop' | 'navRemoveStop' | 'navReorderStops' | 'navSkipStop' | 'navClearStops':
          self._apply_nav_stop_operation(msg_type, settings)
        case 'navRoadMatch':
          if not is_offroad:
            self._apply_nav_road_match(settings)
        case 'navRouteResponse':
          self._apply_nav_route_response(settings, is_offroad)
        case 'navRouteResponseStart':
          self._apply_nav_route_response_start(settings, is_offroad)
        case 'navRouteResponsePart':
          self._apply_nav_route_response_part(settings, is_offroad)
        case 'navRouteResponseAbort':
          self._apply_nav_route_response_abort(settings, is_offroad)
        case 'navRouteSelect':
          self._apply_nav_route_select(settings, is_offroad)
        case 'navRouteSelectStart':
          self._apply_nav_route_select_start(settings, is_offroad)
        case 'navRouteSelectPart':
          self._apply_nav_route_select_part(settings, is_offroad)
        case 'navBleDiagProbe':
          self._apply_nav_ble_diag_probe(settings)
        case 'navBleDiagSkipStop':
          self._apply_nav_ble_diag_skip_stop(settings, is_offroad)
        case 'navBleDiagMapboxResult':
          self._apply_nav_ble_diag_mapbox_result(settings, is_offroad)
        case 'navBleDiagMapboxResultStart':
          self._apply_nav_ble_diag_mapbox_result_start(settings, is_offroad)
        case 'navBleDiagMapboxResultPart':
          self._apply_nav_ble_diag_mapbox_result_part(settings, is_offroad)
        case 'navBleDiagMapboxResultAbort':
          self._apply_nav_ble_diag_mapbox_result_abort(settings, is_offroad)
        case 'navBleDiagSelectRoute':
          self._apply_nav_ble_diag_select_route(settings, is_offroad)
        case 'navBleDiagSelectRouteStart':
          self._apply_nav_ble_diag_select_route_start(settings, is_offroad)
        case 'navBleDiagSelectRoutePart':
          self._apply_nav_ble_diag_select_route_part(settings, is_offroad)
        case 'navPanelSubscription':
          self.nav_panel_subscribed = bool(settings.get('active'))
          self.nav_panel_sync_requested = self.nav_panel_subscribed
        case 'latestAck':
          if (self.send_channel == CHANNEL_VISUALISATION
              and type((msg_id := settings.get('msgId'))) is int and 1 <= msg_id <= 255):
            self._latest_ack_supported = True
            if msg_id in self._latest_inflight_msg_ids:
              del self._latest_inflight_msg_ids[:self._latest_inflight_msg_ids.index(msg_id) + 1]
        case 'disableHotspot':
          disable_hotspot()
    except Exception as e:
      cloudlog.error(f"Apply BLE settings error msg_type={msg_type or 'unknown'}: {e}")

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
      if c != self.send_channel:
        self._latest_ack_supported = False
        self._latest_inflight_msg_ids.clear()
      self.send_channel = c
      self.send_car_names_cnt = 0
      if c == CHANNEL_VIDEO:
        self.video_handler.on_channel_active()
      return None
    return c, m # Other message types, pass to next function

  def appbridged_thread(self):
    is_metric = params.get_bool('IsMetric')
    while True:
      (sm := self.sm).update(0)
      self._update_event_history()

      # 1 Hz WiFi/hotspot task
      if (cur_time := monotonic()) - self.last_1hz_task_time >= 1:
        self.last_1hz_task_time = cur_time
        is_metric = params.get_bool('IsMetric')
        with nav_stop_list_lock():
          if expired := self._nav_route_select_transfer.expire(cur_time):
            context = self._nav_route_select_transfer_context or {}
            self._nav_route_select_transfer_context = None
            self._send_nav_route_select_ack(context, 'rejected', 'transfer_incomplete')
            cloudlog.warning('appbridged nav route selection transfer expired')
          if expired := self._nav_route_transfer.expire(cur_time):
            cloudlog.warning(
              f'appbridged navRouteTransfer expired request={expired["requestId"]} '
              f'transfer={expired["transferId"]} parts={expired["receivedParts"]}/{expired["partCount"]} '
              f'bytes={expired["receivedBytes"]}/{expired["totalBytes"]} idle={expired["idleSeconds"]:.1f}s'
            )
        if expired := self._nav_ble_diag_select_transfer.expire(cur_time):
          context = self._nav_ble_diag_select_context or {}
          self._nav_ble_diag_select_context = None
          self._nav_ble_diag_select_transfer.clear()
          self._send_nav_ble_diag_select_ack(context, 'rejected', 'transfer_incomplete')
        if expired := self._nav_ble_diag_route_transfer.expire(cur_time):
          if self._nav_ble_diag_mapbox_pending_sequence:
            self._fail_nav_ble_diag_route_transfer({
              'testId': self._nav_ble_diag_test_id,
              'sequence': self._nav_ble_diag_mapbox_pending_sequence,
              'requestId': self._nav_ble_diag_request_id,
            }, 'idle_timeout')
          cloudlog.warning(
            f'appbridged nav BLE replay transfer expired test={self._nav_ble_diag_test_id or "none"} '
            f'transfer={expired["transferId"]} parts={expired["receivedParts"]}/{expired["partCount"]} '
            f'bytes={expired["receivedBytes"]}/{expired["totalBytes"]} idle={expired["idleSeconds"]:.1f}s'
          )

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

      if not self.ble.connected:
        self.nav_panel_subscribed = self.nav_panel_sync_requested = False
        self._latest_ack_supported = False
        self._latest_inflight_msg_ids.clear()
      if self.ble.connected: # Only receive/send if connected
        is_offroad = None # Always get latest is_offroad
        state = None
        # Apply any newly received message before sending
        while (msg := self.receiver.get_message()) is not None:
          if not (res := self.handle_send_channel(msg)):
            continue # If dongle ID does not match or it is a curPage message
          if res[0] == CHANNEL_VIDEO:
            self.video_handler.handle_message(res[1], cur_time)
            continue
          if is_offroad is None:
            is_offroad = params.get_bool("IsOffroad")
          if state is None:
            state = sm['selfdriveState'].state
          self.apply_settings_message(res, state, cur_time, is_offroad)

        self.video_handler.tick(cur_time)

        # 3 Hz settings send
        if cur_time - self.last_periodic_time >= 0.333:
          self.last_periodic_time = cur_time
          if self.send_channel == CHANNEL_SETTINGS:
            if is_offroad is None:
              is_offroad = params.get_bool("IsOffroad")
            if state is None:
              state = sm['selfdriveState'].state
            self.send_settings_message(is_offroad, state, is_metric)

        # Visualisation send
        if self.send_channel == CHANNEL_VISUALISATION:
          self.send_visualisation_message(is_metric)

        if (self.nav_panel_subscribed and self.send_channel == CHANNEL_VISUALISATION
            and (self.nav_panel_sync_requested
                 or cur_time - self.last_background_nav_sync_time >= NAV_BACKGROUND_SYNC_INTERVAL_SEC)):
          self.nav_panel_sync_requested = False
          self.last_background_nav_sync_time = cur_time
          self.send_background_nav_message(is_metric, force=True)
        elif (self.send_channel not in (CHANNEL_SETTINGS, CHANNEL_VISUALISATION)
              and cur_time - self.last_background_nav_sync_time >= NAV_BACKGROUND_SYNC_INTERVAL_SEC):
          self.last_background_nav_sync_time = cur_time
          self.send_background_nav_message(is_metric)

        # 2 Hz video keepalive to avoid racing videoListReq.
        if (self.send_channel == CHANNEL_VIDEO
            and cur_time - self.last_video_heartbeat_time >= VIDEO_KEEPALIVE_PERIOD_SEC
            and not self.video_handler.should_pause_video_keepalive()):
          self.last_video_heartbeat_time = cur_time
          self.video_handler.send_list_keepalive()

      lagged = self.rk.keep_time()
      if lagged and self.rk.remaining < -(1.0 / MAX_MESSAGE_HZ):
        # Resume at the current cadence after a multi-frame pause instead of catching up stale deadlines.
        self.rk = Ratekeeper(MAX_MESSAGE_HZ)

def main():
  AppBridge().appbridged_thread()

if __name__ == "__main__":
  main()

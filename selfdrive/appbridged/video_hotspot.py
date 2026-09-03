import os
import glob
import hashlib
import re
import subprocess
import threading
import time

HOTSPOT_SERVICE = "wlan1-setup.service"
_hotspot_lock = threading.Lock()
_hotspot_cache_at = 0.0
_hotspot_cache_active = False
_hotspot_cache_joinable = False
_HOTSPOT_CACHE_SEC = 0.25
HOTSPOT_NETWORK_CONF = "/etc/systemd/network/80-wlan1.network"
DNSMASQ_CONF = "/etc/dnsmasq.conf"


def _read_cpu_serial() -> str:
  try:
    with open("/proc/cpuinfo") as f:
      for line in f:
        if line.startswith("Serial"):
          return line.split(":", 1)[1].strip()
  except OSError:
    pass
  return ""


def _read_wlan0_mac() -> str:
  try:
    with open("/sys/class/net/wlan0/address") as f:
      return f.read().strip()
  except OSError:
    pass
  return ""


def compute_hotspot_credentials() -> tuple[str, str]:
  serial = _read_cpu_serial()
  mac = _read_wlan0_mac().replace(":", "").replace("-", "")
  imei = hashlib.sha256(mac.encode()).hexdigest()[:15]
  dongleid = hashlib.sha224(f"{imei}{serial}".encode()).hexdigest()[:16]
  ssid = f"KommuAssist_{dongleid}"
  return ssid, dongleid


def _parse_networkd_address(path: str) -> str | None:
  try:
    with open(path) as f:
      for line in f:
        line = line.strip()
        if line.startswith("Address=") and (addr := line.split("=", 1)[1].split("/")[0].strip()):
          return addr
  except OSError:
    pass
  return None


def _read_configured_hotspot_ip() -> str | None:
  paths: list[str] = []
  if HOTSPOT_NETWORK_CONF:
    paths.append(HOTSPOT_NETWORK_CONF)
  paths.extend(sorted(glob.glob("/etc/systemd/network/*wlan1*.network")))
  seen: set[str] = set()
  for path in paths:
    if path in seen:
      continue
    seen.add(path)
    if addr := _parse_networkd_address(path):
      return addr
  return _read_dnsmasq_gateway_ip()


def _read_dnsmasq_gateway_ip() -> str | None:
  try:
    with open(DNSMASQ_CONF) as f:
      for line in f:
        if match := re.match(r"dhcp-range=(\d+\.\d+\.\d+)\.\d+,", line.strip()):
          return f"{match.group(1)}.1"
  except OSError:
    pass
  return None


def _iface_exists(iface: str) -> bool:
  return os.path.exists(f"/sys/class/net/{iface}")


def _iface_ipv4_addresses(iface: str) -> list[str]:
  if not _iface_exists(iface):
    return []
  try:
    result = subprocess.run(
      ["ip", "-4", "addr", "show", iface],
      text=True,
      timeout=2,
      stdout=subprocess.PIPE,
      stderr=subprocess.DEVNULL,
      check=False,
    )
  except (subprocess.SubprocessError, OSError):
    return []
  return re.findall(r"inet (\d+\.\d+\.\d+\.\d+)/", result.stdout)


def _hotspot_iface_up() -> bool:
  try:
    with open("/sys/class/net/wlan1/operstate", "r", encoding="utf-8") as f:
      state = f.read().strip()
    return state in ("up", "unknown")
  except OSError:
    return False


def _invalidate_hotspot_cache() -> None:
  global _hotspot_cache_at
  _hotspot_cache_at = 0.0


def is_hotspot_joinable(*, fresh: bool = False) -> bool:
  if fresh:
    return bool(_iface_ipv4_addresses("wlan1")) and _hotspot_iface_up()
  global _hotspot_cache_at, _hotspot_cache_joinable
  now = time.monotonic()
  if now - _hotspot_cache_at < _HOTSPOT_CACHE_SEC:
    return _hotspot_cache_joinable
  _hotspot_cache_joinable = is_hotspot_active() and _hotspot_iface_up()
  return _hotspot_cache_joinable


def is_hotspot_active() -> bool:
  global _hotspot_cache_at, _hotspot_cache_active, _hotspot_cache_joinable
  now = time.monotonic()
  if now - _hotspot_cache_at < _HOTSPOT_CACHE_SEC:
    return _hotspot_cache_active
  _hotspot_cache_active = bool(_iface_ipv4_addresses("wlan1"))
  _hotspot_cache_joinable = _hotspot_cache_active and _hotspot_iface_up()
  _hotspot_cache_at = now
  return _hotspot_cache_active


def get_hotspot_ip() -> str:
  """Return hotspot gateway IPv4 from systemd network config, dnsmasq, or live wlan1."""
  configured = _read_configured_hotspot_ip()
  dnsmasq = _read_dnsmasq_gateway_ip()
  live = _iface_ipv4_addresses("wlan1")

  if configured:
    return configured

  if dnsmasq:
    if dnsmasq in live or not live:
      return dnsmasq
    prefix = dnsmasq.rsplit(".", 1)[0]
    if not any(a.rsplit(".", 1)[0] == prefix for a in live):
      return dnsmasq

  if live:
    if len(live) == 1:
      return live[0]
    if dnsmasq:
      prefix = dnsmasq.rsplit(".", 1)[0]
      for addr in live:
        if addr.rsplit(".", 1)[0] == prefix:
          return addr
    gateways = [a for a in live if a.endswith(".1")]
    if len(gateways) == 1:
      return gateways[0]
    return live[0]

  raise OSError("No hotspot IPv4 address available on wlan1")


def _systemctl(action: str, service: str = HOTSPOT_SERVICE) -> None:
  try:
    subprocess.run(["sudo", "systemctl", action, service], check=True, timeout=30)
  except (subprocess.SubprocessError, OSError):
    pass


_hotspot_cond = threading.Condition(_hotspot_lock)
_hs = {"want": None, "started": False}


def _apply_hotspot_start() -> None:
  _invalidate_hotspot_cache()
  _systemctl("start", HOTSPOT_SERVICE)


def _apply_hotspot_stop() -> None:
  _invalidate_hotspot_cache()
  if _iface_exists("wlan1"):
    try:
      subprocess.run(
        ["sudo", "ip", "link", "set", "wlan1", "down"],
        check=False, timeout=5, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
      )
    except (subprocess.SubprocessError, OSError):
      pass
  _systemctl("stop", HOTSPOT_SERVICE)


def _hotspot_worker() -> None:
  while True:
    with _hotspot_cond:
      while not _hs["want"]:
        _hotspot_cond.wait()
      op, _hs["want"] = _hs["want"], None
    _apply_hotspot_start() if op == "start" else _apply_hotspot_stop()


def _wake_hotspot_unlocked() -> None:
  if not _hs["started"]:
    _hs["started"] = True
    threading.Thread(target=_hotspot_worker, daemon=True, name="video-hotspot").start()
  _hotspot_cond.notify()


def enable_hotspot() -> None:
  with _hotspot_cond:
    _hs["want"] = "start"
    _wake_hotspot_unlocked()


def disable_hotspot() -> None:
  with _hotspot_cond:
    if _hs["want"] == "start":
      return
    _hs["want"] = "stop"
    _wake_hotspot_unlocked()

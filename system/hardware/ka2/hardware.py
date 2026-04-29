import hashlib
import json
import os
import subprocess
import threading
import time
from pathlib import Path
import shutil
import re
import xml.etree.ElementTree as ET

from cereal import log
from openpilot.common.gpio import get_irqs_for_action
from openpilot.system.hardware.base import HardwareBase, ThermalConfig, ThermalZone
from openpilot.system.hardware.ka2 import iwlist

# Use nmcli/mmcli subprocesses instead of python-dbus to avoid
# "malloc unaligned fastbin chunk detected" thread-unsafe C-library D-Bus errors.

SD_CARD_DEVICE = "/dev/mmcblk1"
NetworkType = log.DeviceState.NetworkType
NetworkStrength = log.DeviceState.NetworkStrength


def sudo_write(val, path) -> None:
  data = str(val)
  try:
    with open(path, "w") as f:
      f.write(data)
  except PermissionError:
    if subprocess.run(["sudo", "chmod", "a+w", path], check=False).returncode == 0:
      try:
        with open(path, "w") as f:
          f.write(data)
      except PermissionError:
        subprocess.run(["sudo", "sh", "-c", f"echo {data} > {path}"], check=False)
  except Exception:
    pass


def sudo_read(path: str) -> str:
  try:
    return subprocess.run(["sudo", "cat", path], check=True, capture_output=True, text=True).stdout.strip()
  except Exception:
    return ""


def affine_irq(val, action) -> None:
  if not (irqs := get_irqs_for_action(action)):
    print(f"No IRQs found for '{action}'")
    return
  for i in irqs:
    sudo_write(str(val), f"/proc/irq/{i}/smp_affinity_list")


class Ka2(HardwareBase):
  _MODEM_CACHE_TTL = 1.0

  def __init__(self):
    super().__init__()
    self._lock = threading.RLock()
    self._modem_cache: dict = {}
    self._modem_cache_ts: float = 0
    self._last_cellular_summary: str = "Unknown"
    self._last_usage_sample: tuple[float, int, int] | None = None

  def _sd_inserted(self) -> bool:
    try:
      out = subprocess.run(["lsblk", "-d", "-n", "-o", "NAME"], capture_output=True, text=True).stdout.strip()
      return SD_CARD_DEVICE.rsplit("/", 1)[-1] in out.splitlines()
    except Exception:
      return False

  def sd_status(self) -> str | None:
    nf = "SD card not formatted"
    try:
      if not self._sd_inserted():
        return "SD card not inserted"
      if subprocess.run(["pgrep", "-x", "mkfs.ext4"], check=False).returncode == 0:
        return "Formatting SD card"
      if not (out := subprocess.run(["lsblk", "-n", "-r", "-o", "NAME,TYPE", SD_CARD_DEVICE],
                                    capture_output=True, text=True).stdout.splitlines()):
        return nf
      parts = [line.split()[0] for line in out if "part" in line]
      if len(parts) != 1 or (fs := subprocess.run(["blkid", "-o", "value", "-s", "TYPE", f"/dev/{parts[0]}"],
                                                  capture_output=True, text=True).stdout.strip()) != "ext4":
        return nf
      return None
    except Exception:
      return nf

  def format_sd(self) -> None:
    from openpilot.common.swaglog import cloudlog
    try:
      if (st := self.sd_status()) is None or ("not inserted" not in (st_l := st.lower()) and "formatting" not in st_l):
        def worker():
          try:
            # Run unmount and wipe together so the device is cleared right after unmount, avoiding remount and busy errors
            subprocess.run(f"sudo umount {SD_CARD_DEVICE}p*; sudo wipefs -a {SD_CARD_DEVICE}", shell=True, check=True)
            subprocess.run(["sudo", "sfdisk", SD_CARD_DEVICE], input="label: dos\n,;\n", text=True, check=True)
            subprocess.run(["sudo", "partprobe", SD_CARD_DEVICE], check=True)
            subprocess.run(["sudo", "udevadm", "trigger"], check=True)
            subprocess.run(f"echo y | sudo mkfs.ext4 {SD_CARD_DEVICE}p1", shell=True, check=True)
            r = subprocess.run(["sudo", "mount", "-a"], capture_output=True, text=True)
            cloudlog.info("SD card formatted and mounted successfully." if r.returncode == 0 else f"Mount failed: {r.stderr.strip()}")
          except Exception as e:
            cloudlog.warning(f"SD format error: {e}")
        threading.Thread(target=worker, daemon=True).start()
    except Exception as e:
      cloudlog.warning(f"SD format error: {e}")

  def _run_nmcli(self, args, timeout=5) -> str:
    try:
      return subprocess.run(["nmcli"] + args, capture_output=True, text=True, timeout=timeout).stdout.strip()
    except Exception:
      return ""

  def _run_mmcli_json(self, args, timeout=5) -> dict:
    try:
      return json.loads(subprocess.run(["mmcli", "-J"] + args, capture_output=True, text=True, timeout=timeout).stdout)
    except Exception:
      return {}

  def _modem_json(self) -> dict:
    """Cached mmcli -J -m 0 result (1s TTL) to avoid repeated subprocess spawns."""
    import time
    now = time.monotonic()
    if now - self._modem_cache_ts < self._MODEM_CACHE_TTL and self._modem_cache:
      return self._modem_cache
    self._modem_cache = self._run_mmcli_json(["-m", "0"])
    self._modem_cache_ts = now
    return self._modem_cache

  def _modem_generic(self) -> dict:
    return self._modem_json().get("modem", {}).get("generic", {})

  def _modem_3gpp(self) -> dict:
    return self._modem_json().get("modem", {}).get("3gpp", {}) or {}

  def _wwan0_has_ipv4(self) -> bool:
    """True if wwan0 has a non-loopback IPv4 (data path up even if MM state lags)."""
    if not Path("/sys/class/net/wwan0").exists():
      return False
    try:
      out = subprocess.run(
        ["ip", "-4", "-o", "addr", "show", "dev", "wwan0"],
        capture_output=True, text=True, timeout=2,
      ).stdout
    except Exception:
      return False
    if " inet " not in out:
      return False
    addr = out.split(" inet ", 1)[1].split()[0].split("/")[0]
    return bool(addr) and not addr.startswith("127.")

  def _modem_network_type(self):
    """Return cellular NetworkType from ModemManager state, or None."""
    mg = self._modem_generic()
    if mg.get("state", "").lower() != "connected":
      return None
    at = mg.get("access-technologies", [])
    techs = " ".join(at).lower() if isinstance(at, list) else str(at).lower()
    if "lte" in techs:
      return NetworkType.cell4G
    if any(x in techs for x in ("umts", "hspa")):
      return NetworkType.cell3G
    return NetworkType.cell2G

  def _extract_kv(self, text: str, key: str) -> str:
    return next((line.split(":",1)[1].strip() for line in text.splitlines() if line.startswith(key)), "")

  def _get_bearer_info(self, mid: str):
    for _ in range(30):
      if out := subprocess.run(["mmcli","-m",mid],capture_output=True,text=True).stdout:
        for line in out.splitlines():
          if "/Bearer/" in line and (bpath := line.strip().split()[-1]):
            bkv = subprocess.run(["mmcli","-b",bpath,"--output-keyvalue"],capture_output=True,text=True).stdout
            if "bearer.status.interface: wwan0" in bkv:
              return (
                self._extract_kv(bkv,"bearer.ipv4-config.address"),
                self._extract_kv(bkv,"bearer.ipv4-config.prefix"),
                self._extract_kv(bkv,"bearer.ipv4-config.gateway"),
                self._extract_kv(bkv,"bearer.ipv4-config.dns.value[1]"),
                self._extract_kv(bkv,"bearer.ipv4-config.dns.value[2]"),
                self._extract_kv(bkv,"bearer.ipv4-config.mtu"),
              )
      time.sleep(1)
    return None

  def lookup_apn(self) -> str:
    with self._lock:
      if not (code := str((self.get_sim_info() or {}).get("mcc_mnc", "")).strip()):
        return ""
      mcc, mnc = code[:3], code[3:]
      try:
        root = ET.parse("/usr/share/mobile-broadband-provider-info/serviceproviders.xml").getroot()
        for p in root.iter("provider"):
          if any(n.get("mcc") == mcc and n.get("mnc") == mnc for n in p.iter("network-id")):
            return next((a.get("value") for a in p.iter("apn")
                         if any(u.get("type") == "internet" for u in a.iter("usage"))), "")
      except Exception:
        return ""
      return ""

  def find_modem_id(self) -> str:
    with self._lock:
      for _ in range(30):
        if (out := subprocess.run(["mmcli", "-L"], capture_output=True, text=True).stdout):
          if (m := re.search(r'/Modem/(\d+)', out)):
            return m.group(1)
        time.sleep(1)
      print("ERROR: no modem found after 30s")
      return ""

  def configure_wwan(self) -> None:
    from openpilot.common.params import Params
    if ((apn := Params().get("GsmApn")) and (apn_source := "manual")) or ((apn := self.lookup_apn()) and (apn_source := "auto")):
      print(f"{apn_source} APN: {apn}")
    else:
      print("ERROR: no APN found")
      return

    # Find modem ID
    if not (mid := self.find_modem_id()):
      return

    # Connect with APN
    subprocess.run(["mmcli", "-m", mid, "--simple-disconnect"], capture_output=True, text=True)
    time.sleep(1)
    mmcli_out = subprocess.run(["mmcli", "-m", mid, f"--simple-connect=apn={apn},ip-type=ipv4"], capture_output=True, text=True)
    if mmcli_out.returncode and not any(x in (err := mmcli_out.stderr.lower()) for x in ("already connected", "already registered", "no actions")):
      print(err)
      return

    # Get bearer info
    if not (info := self._get_bearer_info(mid)):
      print("ERROR: bearer missing IPv4 info")
      return
    addr, prefix, gw, dns1, dns2, mtu = info

    # Configure wwan0
    subprocess.run(["sudo", "ip", "link", "set", "wwan0", "up"])
    subprocess.run(["sudo", "ip", "-4", "addr", "flush", "dev", "wwan0"])
    subprocess.run(["sudo", "ip", "-4", "addr", "add", f"{addr}/{prefix}", "dev", "wwan0"])

    # Cleanup old default routes for wwan0
    while subprocess.run(["sudo", "ip", "route", "del", "default", "dev", "wwan0"], capture_output=True).returncode == 0:
      pass

    # Add new default route
    subprocess.run(["sudo", "ip", "route", "add", "default", "via", gw, "dev", "wwan0", "metric", "2000"])

    if mtu:
        subprocess.run(["sudo", "ip", "link", "set", "dev", "wwan0", "mtu", mtu])
    if shutil.which("resolvectl"):
        subprocess.run(["sudo", "resolvectl", "dns", "wwan0", dns1, dns2])
    print(f"wwan0: {addr}/{prefix} via {gw} metric=2000 (dns: {dns1} {dns2}, mtu: {mtu})")

  # -- trivial overrides --

  def get_os_version(self):
    with open("/VERSION") as f:
      return f.read().strip()

  def get_device_type(self):
    return "ka2"

  def reboot(self, reason=None):
    subprocess.run(["sudo", "reboot"], check=False)

  def uninstall(self):
    Path("/data/__system_reset__").touch()
    os.sync()
    self.reboot()

  def get_current_power_draw(self):
    return HardwareBase.read_param_file("/sys/bus/i2c/devices/0-0040/hwmon/hwmon1/power1_input", int, 0) / 1e6

  def get_serial(self):
    try:
      if out := subprocess.run(["grep", "Serial", "/proc/cpuinfo"], capture_output=True, text=True).stdout:
        return out.split(':')[-1].strip()
    except Exception:
      pass
    return ""

  # -- network --

  def get_network_type(self):
    with self._lock:
      out = self._run_nmcli(["-t", "-f", "TYPE,STATE,NAME", "connection", "show", "--active"])
      if out:
        has_ethernet = False
        has_wifi = False
        has_gsm = False
        for line in out.splitlines():
          parts = line.split(':')
          conn_type = parts[0]
          conn_name = parts[2] if len(parts) > 2 else ""
          if "ethernet" in conn_type:
            has_ethernet = True
          elif "wireless" in conn_type and conn_name != "Hotspot":
            has_wifi = True
          elif conn_type == "gsm":
            has_gsm = True

        if has_ethernet:
          return NetworkType.ethernet
        if has_wifi:
          return NetworkType.wifi
        if has_gsm:
          if nt := self._modem_network_type():
            return nt

      # NM has no usable active connection -- check ModemManager directly
      # (wwan0 brought up via MM + ip, not managed by NM)
      if nt := self._modem_network_type():
        return nt
      return NetworkType.none

  def get_sim_info(self):
    with self._lock:
      modem_generic = self._modem_generic()
      if not (sim_path := modem_generic.get("sim")) or sim_path == "/":
        return {
          'sim_id': '',
          'mcc_mnc': None,
          'network_type': ["Unknown"],
          'sim_state': ["ABSENT"],
          'data_connected': False
        }

      sim_props = self._run_mmcli_json(["-i", sim_path.split('/')[-1]]).get("sim", {}).get("properties", {})
      state_ok = modem_generic.get("state", "").lower() == "connected"
      packet_attached = str(self._modem_3gpp().get("packet-service-state", "")).lower() == "attached"
      return {
        'sim_id': str(sim_props.get("iccid", "")),
        'mcc_mnc': str(sim_props.get("operator-code", "")),
        'network_type': ["Unknown"],
        'sim_state': ["READY"],
        'data_connected': state_ok or packet_attached or self._wwan0_has_ipv4(),
      }

  def get_imei(self, slot):
    if slot != 0:
      return ""
    with self._lock:
      if imei := self._modem_generic().get("equipment-identifier"):
        return str(imei)
    mac = subprocess.run(["cat", "/sys/class/net/wlan0/address"], capture_output=True, text=True).stdout.strip()
    return hashlib.sha256(mac.replace(":", "").replace("-", "").encode()).hexdigest()[:15]

  def get_network_info(self):
    with self._lock:
      try:
        raw = subprocess.run(["mmcli", "-m", "0", "--command=AT+QNWINFO"],
                             capture_output=True, text=True, timeout=2).stdout
        if raw and "response: '" in raw:
          m = raw.split("response: '")[1].split("'")[0]
          if m.startswith('+QNWINFO: '):
            info_list = m.replace('+QNWINFO: ', '').replace('"', '').split(',')
            if len(info_list) == 4:
              ex_raw = subprocess.run(["mmcli", "-m", "0", '--command=AT+QENG="servingcell"'],
                                      capture_output=True, text=True, timeout=2).stdout
              ex = ex_raw.split("response: '")[1].split("'")[0] if "response: '" in ex_raw else ""
              return {
                'technology': info_list[0], 'operator': info_list[1],
                'band': info_list[2], 'channel': int(info_list[3]),
                'extra': ex.replace('+QENG: "servingcell",', '').replace('"', ''),
                'state': self._modem_generic().get("state", "unknown").upper(),
              }
      except Exception:
        pass
    return None

  def parse_strength(self, percentage):
    if percentage < 25:
      return NetworkStrength.poor
    if percentage < 50:
      return NetworkStrength.moderate
    if percentage < 75:
      return NetworkStrength.good
    return NetworkStrength.great

  def get_network_strength(self, network_type):
    with self._lock:
      if network_type == NetworkType.wifi:
        if out := self._run_nmcli(["-t", "-f", "IN-USE,SIGNAL", "dev", "wifi"]):
          for line in out.splitlines():
            if line.startswith('*') and len(parts := line.split(':')) > 1:
              return self.parse_strength(int(parts[-1]))
      elif network_type != NetworkType.none:
        if quality := self._modem_generic().get("signal-quality", {}).get("value", 0):
          return self.parse_strength(int(quality))
    return NetworkStrength.unknown

  def get_network_metered(self, network_type) -> bool:
    return network_type in (NetworkType.cell2G, NetworkType.cell3G, NetworkType.cell4G, NetworkType.cell5G)

  def get_modem_version(self):
    with self._lock:
      return str(self._modem_generic().get("revision", "")) or None

  def get_modem_temperatures(self):
    with self._lock:
      try:
        res = subprocess.run(["mmcli", "-m", "0", "--command=AT+QTEMP"],
                             capture_output=True, text=True, timeout=5).stdout
        if "response: '" in res:
          return list(map(int, res.split("response: '")[1].split("'")[0].split(' ')[1].split(',')))
      except (IndexError, ValueError, Exception):
        pass
    return []

  def get_modem_data_usage(self):
    try:
      with open("/sys/class/net/wwan0/statistics/tx_bytes") as f:
        tx = int(f.read().strip())
      with open("/sys/class/net/wwan0/statistics/rx_bytes") as f:
        rx = int(f.read().strip())
      return tx, rx
    except Exception:
      return -1, -1

  def _modem_data_rates_bps(self, tx: int, rx: int) -> tuple[float, float] | None:
    """Compute instantaneous tx/rx rates from byte-counter deltas."""
    if tx < 0 or rx < 0:
      self._last_usage_sample = None
      return None

    now = time.monotonic()
    prev = self._last_usage_sample
    self._last_usage_sample = (now, tx, rx)
    if prev is None:
      return 0.0, 0.0

    prev_t, prev_tx, prev_rx = prev
    dt = now - prev_t
    if dt <= 0:
      return None

    dtx = tx - prev_tx
    drx = rx - prev_rx
    if dtx < 0 or drx < 0:
      # Counter reset/rollover.
      return None
    return dtx / dt, drx / dt

  def get_networks(self):
    r = {}
    if (wlan := iwlist.scan()) is not None:
      r['wlan'] = wlan
    if (lte := self.get_network_info()) and 'LTE' in (ex := lte['extra']):
      ex_list = ex.split(',')
      try:
        r['lte'] = [{
          "mcc": int(ex_list[3]),
          "mnc": int(ex_list[4]),
          "cid": int(ex_list[5], 16),
          "nmr": [{"pci": int(ex_list[6]), "earfcn": int(ex_list[7])}],
        }]
      except (ValueError, IndexError):
        pass
    return r

  # -- modem / hardware config --

  def shutdown(self):
    subprocess.run(["sudo", "poweroff"], check=False)

  def get_thermal_config(self):
    return ThermalConfig(
      cpu=[ThermalZone("bigcore0-thermal", 1000), ThermalZone("bigcore1-thermal", 1000)],
      gpu=[ThermalZone("gpu-thermal", 1000), ThermalZone("npu-thermal", 1000)],
      memory=ThermalZone("center-thermal", 1000),
      pmic=[ThermalZone("soc-thermal", 1000), ThermalZone("center-thermal", 1000)],
    )

  def set_power_save(self, powersave_enabled):
    for i in range(5, 8):
      sudo_write('0' if powersave_enabled else '1', f'/sys/devices/system/cpu/cpu{i}/online')
    for n in ('0', '4'):
      sudo_write('ondemand' if powersave_enabled else 'performance', f'/sys/devices/system/cpu/cpufreq/policy{n}/scaling_governor')

    # *** GPU (Mali): 300 MHz + simple_ondemand in power save, else 1000 MHz + performance ***
    _gpu_devfreq = "/sys/class/devfreq/fb000000.gpu"
    if os.path.isfile(f"{_gpu_devfreq}/governor"):
      if powersave_enabled:
        sudo_write("300000000", f"{_gpu_devfreq}/max_freq")
        sudo_write("simple_ondemand", f"{_gpu_devfreq}/governor")
      else:
        sudo_write("1000000000", f"{_gpu_devfreq}/max_freq")
        sudo_write("performance", f"{_gpu_devfreq}/governor")

  def get_gpu_usage_percent(self):
    try:
      with open("/sys/class/devfreq/fb000000.gpu/load", "r") as f:
        return int(float((s := f.read().strip()).split("@")[0] if "@" in s else s))
    except Exception:
      return 0

  def get_npu_usage_percent(self):
    with open("/sys/kernel/debug/rknpu/load", "r") as f:
      npu_load = f.read().strip()

    try:
      return [int(x.split('%')[0]) for x in npu_load.split() if '%' in x]
    except ValueError:
      return [0, 0, 0]

  def initialize_hardware(self):
    os.system("sudo chmod -R a+r /sys/class/net/wwan0/statistics/")
    subprocess.run(["sudo", "chmod", "a+w", "/dev/kmsg"], check=False)

    sudo_write("f", "/proc/irq/default_smp_affinity")

    sudo_write("userspace", "/sys/class/devfreq/fdab0000.npu/governor")
    sudo_write("1000000000", "/sys/class/devfreq/fdab0000.npu/userspace/set_freq")
    sudo_write("userspace", "/sys/class/devfreq/dmc/governor")
    sudo_write("2112000000", "/sys/class/devfreq/dmc/userspace/set_freq")

  def configure_modem(self):
    self.configure_wwan()

    for cmd in [
      'AT+QNVW=5280,0,"0102000000000000"',
      'AT+QNVFW="/nv/item_files/ims/IMS_enable",00',
      'AT+QNVFW="/nv/item_files/modem/mmode/ue_usage_setting",01',
    ]:
      try:
        subprocess.run(["mmcli", "-m", "0", f"--command={cmd}"], capture_output=True, timeout=5)
      except Exception:
        pass

    os.system("sudo ip route del default dev wwan0 2>/dev/null || true")

  def get_cellular_display_status(self) -> dict:
    tx, rx = self.get_modem_data_usage()
    out = {
      "sim_present": False,
      "data_session_up": False,
      "modem_state": "unknown",
      "technology": None,
      "operator": None,
      "wwan_tx_bytes": tx,
      "wwan_rx_bytes": rx,
      "summary": "Unknown",
    }

    with self._lock:
      try:
        sim = self.get_sim_info()
        modem_generic = self._modem_generic()
        if not modem_generic:
          out["summary"] = "Modem unavailable"
          out["modem_state"] = "unavailable"
          self._last_cellular_summary = out["summary"]
          return out

        out["sim_present"] = True
        state = modem_generic.get("state", "unknown").lower()
        out["modem_state"] = state.upper()

        ninfo = self.get_network_info()
        if ninfo:
          out["technology"] = ninfo.get("technology")
          out["operator"] = ninfo.get("operator")

        data_up = bool((sim or {}).get("data_connected")) or state == "connected"
        out["data_session_up"] = data_up
        reg_state = str(self._modem_3gpp().get("registration-state", "")).lower()

        # Keep UI status short and stable.
        transient_states = {"unknown", "initializing", "initialising", "enabling", "enabled", "searching", "connecting", "disconnecting"}

        if state in transient_states:
          summary = "SIM not ready"
        elif state == "failed":
          summary = "SIM not inserted" if modem_generic.get("state-failed-reason", "").lower() == "sim-missing" else "Modem error"
        elif state == "locked":
          summary = "SIM locked"
        elif state in ("disabled", "disabling"):
          summary = "Modem disabled"
        elif reg_state == "denied":
          summary = "SIM rejected"
        elif state == "registered" and not data_up:
          summary = "Registered, not active"
        elif state == "connected" or data_up:
          summary = "Connected"
          if (rates := self._modem_data_rates_bps(tx, rx)) is not None:
            tx_bps, rx_bps = rates
            summary += f" | TX {tx_bps/1024:.1f} KB/s | RX {rx_bps/1024:.1f} KB/s"
        else:
          summary = "SIM Not Ready"

        out["summary"] = summary
        if state not in transient_states:
          self._last_cellular_summary = summary
        return out
      except Exception:
        out["summary"] = "Unknown"
        return out

  def has_internal_panda(self):
    return True


if __name__ == "__main__":
  t = Ka2()
  t.configure_modem()
  t.initialize_hardware()
  t.set_power_save(False)
